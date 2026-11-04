#include "../../../include/bridge/core/bridge_runner.hpp"

#include <cstdio>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "../../../include/bridge/extensions/hooks.hpp"
#include "../../../include/bridge/gamepad/common_gamepad.hpp"
#include "../../../include/bridge/observability/forwarding_logger.hpp"
#include "../../../include/bridge/observability/forwarding_stats.hpp"

namespace bridge::core {
namespace {

struct CachedGamepadSample {
  gamepad::CommonGamepad gamepad;
  std::uint64_t receive_time_ns{0};
};

[[nodiscard]] std::uint64_t now_ns() noexcept {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

[[nodiscard]] std::string format_one_decimal(const double value) {
  std::ostringstream output;
  output.setf(std::ios::fixed);
  output.precision(1);
  output << value;
  return output.str();
}

[[nodiscard]] std::string format_two_decimal(const double value) {
  std::ostringstream output;
  output.setf(std::ios::fixed);
  output.precision(2);
  output << value;
  return output.str();
}

[[nodiscard]] RunnerCallStatus map_adapter_status(
    const AdapterCallStatus status) noexcept {
  switch (status) {
    case AdapterCallStatus::kOk:
      return RunnerCallStatus::kOk;
    case AdapterCallStatus::kDegraded:
      return RunnerCallStatus::kDegraded;
    case AdapterCallStatus::kUnsupported:
      return RunnerCallStatus::kUnsupported;
    case AdapterCallStatus::kRetryableError:
      return RunnerCallStatus::kRetryableError;
    case AdapterCallStatus::kFatalError:
      return RunnerCallStatus::kFatalError;
  }

  return RunnerCallStatus::kFatalError;
}

[[nodiscard]] bool is_waiting_for_initial_state_detail(
    const std::string& detail) noexcept {
  return detail.rfind("waiting for ", 0) == 0;
}

[[nodiscard]] bool is_lowstate_receive_timeout_detail(
    const std::string& detail) noexcept {
  return detail == "g1 low-level lowstate receive timeout" ||
         detail == "g1 low-level state not yet available after read retries" ||
         detail == "pm01 robot state not available";
}

[[nodiscard]] BridgeFault make_fault(const FaultCategory category,
                                     const FaultSeverity severity,
                                     const LifecycleState lifecycle_state,
                                     std::string message,
                                     const std::uint64_t timestamp_ns) {
  return {
      .category = category,
      .severity = severity,
      .lifecycle_state = lifecycle_state,
      .message = std::move(message),
      .latched = false,
      .sequence = 0,
      .timestamp_ns = timestamp_ns,
  };
}

[[nodiscard]] bool is_error_status(const RunnerCallStatus status) noexcept {
  return status == RunnerCallStatus::kRetryableError ||
         status == RunnerCallStatus::kFatalError;
}

[[nodiscard]] bool is_error_status(const AdapterCallStatus status) noexcept {
  return status == AdapterCallStatus::kRetryableError ||
         status == AdapterCallStatus::kFatalError;
}

void append_detail(std::string* detail, const std::string& next) {
  if (next.empty()) {
    return;
  }

  if (!detail->empty()) {
    detail->append("; ");
  }
  detail->append(next);
}

}

class BridgeRunner::Impl {
 public:
  Impl(BridgeRunnerConfig config, BridgeRunnerDependencies dependencies)
      : config_(std::move(config)), dependencies_(dependencies) {}

  ~Impl() { stop_gamepad_remote(); }

  [[nodiscard]] const BridgeRunnerConfig& config() const noexcept {
    return config_;
  }

  [[nodiscard]] BridgeRunnerStatus status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
  }

  [[nodiscard]] RunnerCallResult initialize() {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (status_.lifecycle_state == LifecycleState::kStopped) {
        return make_runner_result(RunnerCallStatus::kRejected,
                                  "runner already stopped");
      }
      if (status_.lifecycle_state != LifecycleState::kCreated) {
        return make_runner_result(RunnerCallStatus::kRejected,
                                  "runner already initialized");
      }

      if (dependencies_.adapter == nullptr ||
          dependencies_.command_ingress == nullptr ||
          dependencies_.state_egress == nullptr) {
        const BridgeFault fault = make_fault(
            FaultCategory::kConfiguration, FaultSeverity::kFatal,
            LifecycleState::kCreated, "runner dependencies must not be null", 0);
        return latch_fault_locked(fault, true, RunnerCallStatus::kFatalError);
      }

      if (config_.loop_period_ns == 0) {
        const BridgeFault fault = make_fault(
            FaultCategory::kConfiguration, FaultSeverity::kFatal,
            LifecycleState::kCreated, "loop_period_ns must be greater than zero",
            0);
        return latch_fault_locked(fault, true, RunnerCallStatus::kFatalError);
      }

      status_.lifecycle_state = LifecycleState::kInitializing;
    }

    AdapterCallResult adapter_init = dependencies_.adapter->initialize();
    if (!adapter_init.succeeded()) {
      std::lock_guard<std::mutex> lock(mutex_);
      return handle_adapter_failure_locked(
          adapter_init, FaultCategory::kAdapterInitialize,
          "adapter initialize failed", true);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      update_degraded_from_adapter_locked(adapter_init);
    }

    AdapterHealthResult health = dependencies_.adapter->query_health();
    if (!health.result.succeeded()) {
      std::lock_guard<std::mutex> lock(mutex_);
      return handle_adapter_failure_locked(
          health.result, FaultCategory::kAdapterHealth,
          "adapter health query failed during initialize", true);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      update_health_locked(health);
    }

    RunnerCallResult state_start = dependencies_.state_egress->start();
    if (!state_start.succeeded()) {
      std::lock_guard<std::mutex> lock(mutex_);
      return handle_runner_failure_locked(state_start, FaultCategory::kTransport,
                                          "state egress start failed",
                                          config_.fault_policy
                                              .fail_stop_on_transport_fault);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      update_degraded_from_runner_locked(state_start);
    }

    RunnerCallResult ingress_start =
        dependencies_.command_ingress->start(*owner_, *owner_);
    if (!ingress_start.succeeded()) {
      std::lock_guard<std::mutex> lock(mutex_);
      return handle_runner_failure_locked(
          ingress_start, FaultCategory::kTransport,
          "command ingress start failed",
          config_.fault_policy.fail_stop_on_transport_fault);
    }

    if (dependencies_.gamepad_remote != nullptr) {
      RunnerCallResult gamepad_start = dependencies_.gamepad_remote->start();
      if (config_.forwarding_log_enabled) {
        observability::forwarding_logger()->info("[gamepad_remote] start detail=" +
                                                 gamepad_start.detail);
      }
      if (gamepad_start.status != RunnerCallStatus::kOk) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.degraded = true;
      }
      if (gamepad_start.succeeded()) {
        gamepad_remote_started_ = true;
        start_gamepad_thread();
      } else {
        std::lock_guard<std::mutex> lock(mutex_);
        update_degraded_from_runner_locked(gamepad_start);
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    update_degraded_from_runner_locked(ingress_start);
    status_.lifecycle_state = LifecycleState::kWaitingForState;
    return make_runner_result(status_.degraded ? RunnerCallStatus::kDegraded
                                               : RunnerCallStatus::kOk,
                              "runner initialized", status_.latched_fault);
  }

  [[nodiscard]] RunnerCallResult run() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (status_.lifecycle_state == LifecycleState::kCreated) {
        return make_runner_result(RunnerCallStatus::kRejected,
                                  "runner must be initialized before run");
      }
      if (status_.lifecycle_state == LifecycleState::kStopped) {
        return make_runner_result(RunnerCallStatus::kRejected,
                                  "runner already stopped");
      }
      if (status_.stop_requested ||
          status_.lifecycle_state == LifecycleState::kStopping) {
        return make_runner_result(RunnerCallStatus::kRejected,
                                  "stop requested; call shutdown");
      }
      if (status_.fail_stop_active &&
          !config_.fault_policy.continue_publishing_in_fault) {
        return make_runner_result(RunnerCallStatus::kRejected,
                                  "runner faulted and publishing is disabled");
      }
    }

    AdapterReadResult read_result = dependencies_.adapter->read_state();
    if (!read_result.result.succeeded()) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (status_.lifecycle_state == LifecycleState::kWaitingForState &&
          !status_.first_state_received &&
          read_result.result.status == AdapterCallStatus::kRetryableError &&
          is_waiting_for_initial_state_detail(read_result.result.detail)) {
        return make_runner_result(RunnerCallStatus::kDegraded,
                                  read_result.result.detail,
                                  status_.latched_fault);
      }
      if (status_.first_state_received &&
          read_result.result.status == AdapterCallStatus::kRetryableError &&
          is_lowstate_receive_timeout_detail(read_result.result.detail)) {
        status_.degraded = true;
        return make_runner_result(RunnerCallStatus::kDegraded,
                                  read_result.result.detail,
                                  status_.latched_fault);
      }
      return handle_adapter_failure_locked(read_result.result,
                                           FaultCategory::kAdapterRead,
                                           "adapter read failed", true);
    }
    if (!read_result.state.has_value()) {
      std::lock_guard<std::mutex> lock(mutex_);
      const BridgeFault fault = make_fault(
          FaultCategory::kAdapterRead, FaultSeverity::kFatal,
          status_.lifecycle_state, "adapter read returned no state",
          status_.last_cycle_time_ns);
      return latch_fault_locked(fault, true, RunnerCallStatus::kFatalError);
    }

    StateFrame state = std::move(*read_result.state);
    RobotAuxStateSnapshot aux_state = std::move(read_result.aux_state);
    update_status_from_state(state, read_result.result);

    RunnerCallResult publish_result = publish_state(state, aux_state);
    if (is_error_status(publish_result.status)) {
      return publish_result;
    }

    RunnerCallResult health_result = evaluate_health();
    if (is_error_status(health_result.status)) {
      return health_result;
    }

    const std::uint64_t cycle_time_ns = select_cycle_time(state);
    RunnerCallResult command_result = apply_command_policy(cycle_time_ns);

    RunnerCallStatus final_status = RunnerCallStatus::kOk;
    if (status().degraded || publish_result.status == RunnerCallStatus::kDegraded ||
        health_result.status == RunnerCallStatus::kDegraded ||
        command_result.status == RunnerCallStatus::kDegraded) {
      final_status = RunnerCallStatus::kDegraded;
    }

    std::string detail;
    append_detail(&detail, publish_result.detail);
    append_detail(&detail, health_result.detail);
    append_detail(&detail, command_result.detail);

    auto current_status = status();
    return make_runner_result(final_status,
                              detail.empty() ? "runner cycle complete" : detail,
                              current_status.latched_fault);
  }

  [[nodiscard]] RunnerCallResult request_stop() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (status_.lifecycle_state == LifecycleState::kStopped) {
      return make_runner_result(RunnerCallStatus::kOk,
                                "runner already stopped");
    }

    status_.stop_requested = true;
    if (status_.lifecycle_state != LifecycleState::kCreated) {
      status_.lifecycle_state = LifecycleState::kStopping;
    }
    return make_runner_result(RunnerCallStatus::kOk, "stop requested");
  }

  [[nodiscard]] RunnerCallResult shutdown() {
    RunnerCallResult ingress_stop_result;
    RunnerCallResult egress_stop_result;
    AdapterCallResult adapter_shutdown_result;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (status_.lifecycle_state == LifecycleState::kStopped) {
        return make_runner_result(RunnerCallStatus::kOk,
                                  "runner already stopped");
      }

      status_.stop_requested = true;
      status_.lifecycle_state = LifecycleState::kStopping;
    }

    stop_gamepad_remote();

    if (dependencies_.command_ingress != nullptr) {
      ingress_stop_result = dependencies_.command_ingress->stop();
    }
    stop_gamepad_remote();
    if (dependencies_.state_egress != nullptr) {
      egress_stop_result = dependencies_.state_egress->stop();
    }
    if (dependencies_.adapter != nullptr) {
      adapter_shutdown_result = dependencies_.adapter->shutdown();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    RunnerCallStatus final_status = RunnerCallStatus::kOk;
    std::string detail = "runner shutdown complete";

    if (dependencies_.command_ingress != nullptr &&
        !ingress_stop_result.succeeded()) {
      final_status = ingress_stop_result.status;
      detail = ingress_stop_result.detail;
      update_degraded_from_runner_locked(ingress_stop_result);
    }

    if (dependencies_.state_egress != nullptr && !egress_stop_result.succeeded()) {
      final_status = egress_stop_result.status;
      detail = egress_stop_result.detail;
      update_degraded_from_runner_locked(egress_stop_result);
    }

    if (dependencies_.adapter != nullptr && !adapter_shutdown_result.succeeded()) {
      final_status = map_adapter_status(adapter_shutdown_result.status);
      detail = adapter_shutdown_result.detail;
      update_degraded_from_adapter_locked(adapter_shutdown_result);
      if (is_error_status(adapter_shutdown_result.status)) {
        BridgeFault fault = adapter_shutdown_result.fault.value_or(make_fault(
            FaultCategory::kAdapterShutdown, FaultSeverity::kRecoverable,
            LifecycleState::kStopping, "adapter shutdown failed",
            status_.last_cycle_time_ns));
        fault.lifecycle_state = LifecycleState::kStopping;
        finalize_fault_locked(&fault);
      }
    }

    status_.lifecycle_state = LifecycleState::kStopped;
    return make_runner_result(final_status, detail, status_.latched_fault);
  }

  [[nodiscard]] CommandSubmissionResult submit_latest_command(
      CommandFrame command) {
    CommandFrame command_to_write = command;
    bool replaced_previous = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (status_.lifecycle_state == LifecycleState::kStopping ||
          status_.lifecycle_state == LifecycleState::kStopped) {
        CommandSubmissionResult submission;
        submission.result = make_runner_result(RunnerCallStatus::kRejected,
                                               "runner is stopping or stopped");
        return submission;
      }
      // if (!command.valid || command.mode == ControlMode::kUnknown) {
      //   CommandSubmissionResult submission;
      //   submission.result = make_runner_result(RunnerCallStatus::kRejected,
      //                                          "invalid command rejected");
      //   return submission;
      // }

      replaced_previous = status_.command_buffer.has_latest_command;
      status_.command_buffer.has_latest_command = true;
      status_.command_buffer.latest_command_stale = false;
      status_.command_buffer.latest_command_sequence = command.sequence;
      status_.command_buffer.latest_command_receive_time_ns =
          command.receive_time_ns;
      latest_command_ = std::move(command);
      status_.first_command_received = true;
    }

    AdapterCallResult write_result =
        dependencies_.adapter->write_command(command_to_write);

    std::lock_guard<std::mutex> lock(mutex_);
    if (!write_result.succeeded()) {
      CommandSubmissionResult submission;
      submission.replaced_previous = replaced_previous;
      submission.stored_sequence = status_.command_buffer.latest_command_sequence;
      submission.result = handle_adapter_failure_locked(
          write_result, FaultCategory::kAdapterWrite,
          "adapter command write failed", true);
      return submission;
    }

    status_.command_stale = false;
    status_.command_buffer.has_active_command = true;
    status_.command_buffer.active_command_sequence = command_to_write.sequence;
    status_.command_buffer.active_mode = command_to_write.mode;
    update_degraded_from_adapter_locked(write_result);

    CommandSubmissionResult submission;
    submission.replaced_previous = replaced_previous;
    submission.stored_sequence = status_.command_buffer.latest_command_sequence;
    submission.result = make_runner_result(
        write_result.status == AdapterCallStatus::kDegraded
            ? RunnerCallStatus::kDegraded
            : RunnerCallStatus::kOk,
        "latest command written", status_.latched_fault);
    return submission;
  }

  [[nodiscard]] RunnerCallResult report_transport_fault(BridgeFault fault) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!fault.has_fault()) {
      fault = make_fault(FaultCategory::kTransport, FaultSeverity::kRecoverable,
                         status_.lifecycle_state, "transport fault reported",
                         status_.last_cycle_time_ns);
    }

    if (fault.category == FaultCategory::kNone) {
      fault.category = FaultCategory::kTransport;
    }
    if (fault.lifecycle_state == LifecycleState::kCreated) {
      fault.lifecycle_state = status_.lifecycle_state;
    }

    return latch_fault_locked(fault,
                              config_.fault_policy.fail_stop_on_transport_fault,
                              fault.severity == FaultSeverity::kFatal
                                  ? RunnerCallStatus::kFatalError
                                  : RunnerCallStatus::kDegraded);
  }

  void notify_state_available() {
    {
      std::lock_guard<std::mutex> lock(state_event_mutex_);
      state_event_pending_ = true;
    }
    state_event_cv_.notify_all();
  }

  [[nodiscard]] bool wait_for_state_available_until(
      const std::chrono::steady_clock::time_point deadline) {
    std::unique_lock<std::mutex> lock(state_event_mutex_);
    const bool notified = state_event_cv_.wait_until(lock, deadline, [&] {
      return state_event_pending_;
    });
    if (notified) {
      state_event_pending_ = false;
    }
    return notified;
  }

  void bind_owner(BridgeRunner* owner) noexcept { owner_ = owner; }

 private:
  [[nodiscard]] RunnerCallResult make_runner_result(
      const RunnerCallStatus status, std::string detail,
      std::optional<BridgeFault> fault = std::nullopt) const {
    return {
        .status = status,
        .detail = std::move(detail),
        .fault = std::move(fault),
    };
  }

  void finalize_fault_locked(BridgeFault* fault) {
    fault->latched = true;
    if (fault->sequence == 0) {
      fault->sequence = ++fault_sequence_;
    }
    if (fault->timestamp_ns == 0) {
      fault->timestamp_ns = status_.last_cycle_time_ns;
    }
    if (fault->lifecycle_state == LifecycleState::kCreated) {
      fault->lifecycle_state = status_.lifecycle_state;
    }
    if (!status_.latched_fault.has_value()) {
      status_.latched_fault = *fault;
    }
  }

  [[nodiscard]] RunnerCallResult latch_fault_locked(BridgeFault fault,
                                                    const bool activate_fail_stop,
                                                    const RunnerCallStatus status) {
    if (fault.lifecycle_state == LifecycleState::kCreated) {
      fault.lifecycle_state = status_.lifecycle_state;
    }
    finalize_fault_locked(&fault);
    status_.degraded = true;
    if (activate_fail_stop) {
      status_.fail_stop_active = true;
      status_.lifecycle_state = LifecycleState::kFaulted;
    }
    return make_runner_result(status, fault.message, status_.latched_fault);
  }

  void update_degraded_from_adapter_locked(const AdapterCallResult& result) {
    if (result.status == AdapterCallStatus::kDegraded) {
      status_.degraded = true;
    }
  }

  void update_degraded_from_runner_locked(const RunnerCallResult& result) {
    if (result.status == RunnerCallStatus::kDegraded) {
      status_.degraded = true;
    }
  }

  [[nodiscard]] RunnerCallResult handle_adapter_failure_locked(
      const AdapterCallResult& result, const FaultCategory category,
      const std::string& fallback_detail, const bool activate_fail_stop) {
    BridgeFault fault = result.fault.value_or(make_fault(
        category,
        result.status == AdapterCallStatus::kFatalError ? FaultSeverity::kFatal
                                                        : FaultSeverity::kRecoverable,
        status_.lifecycle_state,
        result.detail.empty() ? fallback_detail : result.detail,
        status_.last_cycle_time_ns));
    if (fault.category == FaultCategory::kNone) {
      fault.category = category;
    }
    return latch_fault_locked(fault, activate_fail_stop,
                              map_adapter_status(result.status));
  }

  [[nodiscard]] RunnerCallResult handle_adapter_failure_locked(
      const AdapterCallResult& result, const FaultCategory category,
      const char* fallback_detail, const bool activate_fail_stop) {
    return handle_adapter_failure_locked(result, category,
                                         std::string(fallback_detail),
                                         activate_fail_stop);
  }

  [[nodiscard]] RunnerCallResult handle_runner_failure_locked(
      const RunnerCallResult& result, const FaultCategory category,
      const std::string& fallback_detail, const bool activate_fail_stop) {
    BridgeFault fault = result.fault.value_or(make_fault(
        category,
        result.status == RunnerCallStatus::kFatalError ? FaultSeverity::kFatal
                                                       : FaultSeverity::kRecoverable,
        status_.lifecycle_state,
        result.detail.empty() ? fallback_detail : result.detail,
        status_.last_cycle_time_ns));
    if (fault.category == FaultCategory::kNone) {
      fault.category = category;
    }
    return latch_fault_locked(fault, activate_fail_stop, result.status);
  }

  void update_status_from_state(const StateFrame& state,
                                const AdapterCallResult& read_result) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.cycle_count += 1;
    status_.first_state_received = true;
    status_.last_cycle_time_ns = select_cycle_time(state);
    if (status_.lifecycle_state == LifecycleState::kWaitingForState &&
        !status_.fail_stop_active) {
      status_.lifecycle_state = LifecycleState::kActive;
    }
    if (read_result.status == AdapterCallStatus::kDegraded || state.degraded) {
      status_.degraded = true;
    }
  }

  [[nodiscard]] RunnerCallResult publish_state(
      StateFrame& state, const RobotAuxStateSnapshot& aux_state) {
    extensions::HookInvocationContext context;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state.lifecycle_state = status_.lifecycle_state;
      state.command_stale = status_.command_stale;
      state.degraded = state.degraded || status_.degraded || status_.fail_stop_active;
      if (status_.command_buffer.has_active_command) {
        state.active_mode = status_.command_buffer.active_mode;
      }
      if (status_.latched_fault.has_value()) {
        state.fault = status_.latched_fault;
      }
      state.publish_time_ns = status_.last_cycle_time_ns;
      apply_gamepad_remote_locked(&state);
      context = {.cycle_index = status_.cycle_count, .time_ns = status_.last_cycle_time_ns};
    }

    if (dependencies_.observation_hook != nullptr) {
      dependencies_.observation_hook->observe_state(state, context);
    }

    state.publish_start_time_ns = now_ns();
    RunnerCallResult publish_result =
        dependencies_.state_egress->publish(state, aux_state);
    const std::uint64_t publish_end_ns = now_ns();
    state.publish_complete_time_ns = publish_end_ns;
    std::optional<observability::ForwardingStatsSnapshot> state_forward_stats;
    if (publish_result.succeeded() && state.receive_time_ns != 0) {
      state_forward_stats = state_forward_stats_.record(publish_end_ns, state.receive_time_ns);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (publish_result.succeeded()) {
      status_.last_state_publish_time_ns = state.publish_time_ns;
      if (config_.forwarding_log_enabled && state_forward_stats.has_value()) {
        bridge::observability::forwarding_logger()->info(
            "[adapter_state_forward] publish_hz=" +
            format_two_decimal(state_forward_stats->frequency_hz) +
            " robot_rx_to_publish_avg_us=" +
            format_one_decimal(state_forward_stats->avg_latency_us) +
            " robot_rx_to_publish_max_us=" +
            format_one_decimal(state_forward_stats->max_latency_us) +
            " interval_avg_us=" +
            format_one_decimal(state_forward_stats->avg_interval_us));
        const double rx_to_read_start_us =
            state.read_start_time_ns >= state.receive_time_ns && state.receive_time_ns != 0
                ? static_cast<double>(state.read_start_time_ns - state.receive_time_ns) /
                      1000.0
                : 0.0;
        const double read_to_publish_start_us =
            state.publish_start_time_ns >= state.read_complete_time_ns &&
                    state.read_complete_time_ns != 0
                ? static_cast<double>(state.publish_start_time_ns -
                                      state.read_complete_time_ns) /
                      1000.0
                : 0.0;
        const double rx_to_publish_us =
            publish_end_ns >= state.receive_time_ns && state.receive_time_ns != 0
                ? static_cast<double>(publish_end_ns - state.receive_time_ns) / 1000.0
                : 0.0;
        bridge::observability::forwarding_logger()->info(
            "[adapter_latency_breakdown] rx_callback_store_us=" +
            format_one_decimal(static_cast<double>(state.lowstate_callback_store_duration_ns) /
                               1000.0) +
            " rx_to_runner_read_start_us=" + format_one_decimal(rx_to_read_start_us) +
            " read_snapshot_us=" +
            format_one_decimal(static_cast<double>(state.read_snapshot_duration_ns) / 1000.0) +
            " stateframe_build_us=" +
            format_one_decimal(static_cast<double>(state.stateframe_build_duration_ns) / 1000.0) +
            " read_to_publish_start_us=" + format_one_decimal(read_to_publish_start_us) +
            " lowstate_translate_us=" +
            format_one_decimal(static_cast<double>(state.lowstate_translate_duration_ns) /
                               1000.0) +
            " dds_write_us=" +
            format_one_decimal(static_cast<double>(state.dds_write_duration_ns) / 1000.0) +
            " rx_to_publish_us=" + format_one_decimal(rx_to_publish_us) +
            " callback_drops=" + std::to_string(state.callback_drops));
      }
      update_degraded_from_runner_locked(publish_result);
      return publish_result;
    }
    return handle_runner_failure_locked(
        publish_result, FaultCategory::kTransport, "state publish failed",
        config_.fault_policy.fail_stop_on_transport_fault);
  }

  void start_gamepad_thread() {
    gamepad_stop_requested_.store(false);
    gamepad_thread_ = std::thread([this] { read_gamepad_loop(); });
  }

  void stop_gamepad_remote() noexcept {
    gamepad_stop_requested_.store(true);
    if (gamepad_thread_.joinable()) {
      gamepad_thread_.join();
    }
    if (dependencies_.gamepad_remote != nullptr && gamepad_remote_started_) {
      [[maybe_unused]] const RunnerCallResult ignored =
          dependencies_.gamepad_remote->stop();
      gamepad_remote_started_ = false;
    }
  }

  void read_gamepad_loop() {
    const std::uint64_t poll_interval_ns =
        config_.gamepad_poll_interval_ns == 0 ? 1'000'000ULL
                                              : config_.gamepad_poll_interval_ns;
    const auto poll_interval = std::chrono::nanoseconds(poll_interval_ns);
    while (!gamepad_stop_requested_.load()) {
      if (dependencies_.gamepad_remote != nullptr) {
        std::optional<gamepad::CommonGamepad> sample =
            dependencies_.gamepad_remote->read();
        if (sample.has_value()) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_gamepad_sample_ = CachedGamepadSample{.gamepad = *sample,
                                                       .receive_time_ns = now_ns()};
          if (config_.forwarding_log_enabled && !gamepad_sample_logged_) {
            observability::forwarding_logger()->info(
                "[gamepad_remote] first_sample_received");
            gamepad_sample_logged_ = true;
          }
        }
      }
      std::this_thread::sleep_for(poll_interval);
    }
  }

  void apply_gamepad_remote_locked(StateFrame* state) {
    if (!latest_gamepad_sample_.has_value()) {
      return;
    }
    if (config_.gamepad_stale_timeout_ns != 0) {
      const std::uint64_t current_time_ns = now_ns();
      const std::uint64_t receive_time_ns = latest_gamepad_sample_->receive_time_ns;
      if (current_time_ns >= receive_time_ns &&
          current_time_ns - receive_time_ns > config_.gamepad_stale_timeout_ns) {
        return;
      }
    }
    state->wireless_remote =
        gamepad::compress_to_wireless_remote(latest_gamepad_sample_->gamepad);
    if (config_.forwarding_log_enabled && !gamepad_overlay_logged_) {
      observability::forwarding_logger()->info(
          "[gamepad_remote] wireless_remote_overlay_applied");
      gamepad_overlay_logged_ = true;
    }
  }

  [[nodiscard]] RunnerCallResult evaluate_health() {
    AdapterHealthResult health = dependencies_.adapter->query_health();

    std::lock_guard<std::mutex> lock(mutex_);
    if (!health.result.succeeded()) {
      return handle_adapter_failure_locked(
          health.result, FaultCategory::kAdapterHealth,
          "adapter health query failed", true);
    }

    update_health_locked(health);
    return make_runner_result(status_.degraded ? RunnerCallStatus::kDegraded
                                               : RunnerCallStatus::kOk,
                              "adapter health checked", status_.latched_fault);
  }

  void update_health_locked(const AdapterHealthResult& health) {
    if (health.result.status == AdapterCallStatus::kDegraded ||
        health.health.degraded || health.health.state == HealthState::kDegraded) {
      status_.degraded = true;

      if (config_.fault_policy.fail_stop_on_adapter_degraded &&
          !status_.fail_stop_active) {
        BridgeFault fault = health.health.fault.value_or(make_fault(
            FaultCategory::kAdapterHealth, FaultSeverity::kRecoverable,
            status_.lifecycle_state, "adapter reported degraded health",
            status_.last_cycle_time_ns));
        if (fault.category == FaultCategory::kNone) {
          fault.category = FaultCategory::kAdapterHealth;
        }
        [[maybe_unused]] const RunnerCallResult ignored =
            latch_fault_locked(fault, true, RunnerCallStatus::kDegraded);
      }
    }

    if (health.health.state == HealthState::kFaulted && !status_.fail_stop_active) {
      BridgeFault fault = health.health.fault.value_or(make_fault(
          FaultCategory::kAdapterHealth, FaultSeverity::kFatal,
          status_.lifecycle_state, "adapter reported faulted health",
          status_.last_cycle_time_ns));
      if (fault.category == FaultCategory::kNone) {
        fault.category = FaultCategory::kAdapterHealth;
      }
      [[maybe_unused]] const RunnerCallResult ignored =
          latch_fault_locked(fault, true, RunnerCallStatus::kFatalError);
    }
  }

  [[nodiscard]] RunnerCallResult apply_command_policy(
      const std::uint64_t cycle_time_ns) {
    std::optional<CommandFrame> command_to_observe;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      update_command_staleness_locked(cycle_time_ns);

      if (status_.command_buffer.has_latest_command &&
                 !status_.command_buffer.latest_command_stale) {
        command_to_observe = latest_command_;
      }
    }

    if (command_to_observe.has_value()) {
      std::lock_guard<std::mutex> lock(mutex_);
      status_.command_stale = false;
      if (dependencies_.observation_hook != nullptr) {
        dependencies_.observation_hook->observe_command(
            *command_to_observe,
            {.cycle_index = status_.cycle_count, .time_ns = cycle_time_ns});
      }
      return make_runner_result(status_.degraded ? RunnerCallStatus::kDegraded
                                                 : RunnerCallStatus::kOk,
                                "active command already written",
                                status_.latched_fault);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    return make_runner_result(status_.degraded ? RunnerCallStatus::kDegraded
                                               : RunnerCallStatus::kOk,
                              status_.command_stale ? "command stale" : "no command write",
                              status_.latched_fault);
  }

  void update_command_staleness_locked(const std::uint64_t cycle_time_ns) {
    status_.command_stale = false;

    if (!status_.command_buffer.has_latest_command) {
      status_.command_buffer.latest_command_stale = false;
      return;
    }

    if (config_.command_stale_timeout_ns == 0) {
      status_.command_buffer.latest_command_stale = false;
      return;
    }

    const std::uint64_t receive_time_ns =
        status_.command_buffer.latest_command_receive_time_ns;
    const bool stale = cycle_time_ns >= receive_time_ns &&
                       (cycle_time_ns - receive_time_ns) >
                           config_.command_stale_timeout_ns;
    status_.command_buffer.latest_command_stale = stale;
    status_.command_stale = stale;

    if (!stale || !config_.fault_policy.fail_stop_on_command_stale ||
        status_.fail_stop_active) {
      return;
    }

    BridgeFault fault = make_fault(
        FaultCategory::kCommandStale, FaultSeverity::kFatal,
        status_.lifecycle_state,
        "latest command exceeded stale timeout before command write",
        cycle_time_ns);
    [[maybe_unused]] const RunnerCallResult ignored =
        latch_fault_locked(fault, true, RunnerCallStatus::kFatalError);
  }

  [[nodiscard]] static std::uint64_t select_cycle_time(
      const StateFrame& state) noexcept {
    if (state.publish_time_ns != 0) {
      return state.publish_time_ns;
    }
    if (state.sample_time_ns != 0) {
      return state.sample_time_ns;
    }
    return 0;
  }

  BridgeRunnerConfig config_;
  BridgeRunnerDependencies dependencies_;
  BridgeRunner* owner_{nullptr};

  mutable std::mutex mutex_;
  std::mutex state_event_mutex_;
  std::condition_variable state_event_cv_;
  bool state_event_pending_{false};
  BridgeRunnerStatus status_;
  std::optional<CommandFrame> latest_command_;
  std::optional<CachedGamepadSample> latest_gamepad_sample_;
  observability::ForwardingStats state_forward_stats_;
  std::uint64_t fault_sequence_{0};
  bool gamepad_remote_started_{false};
  bool gamepad_sample_logged_{false};
  bool gamepad_overlay_logged_{false};
  std::atomic_bool gamepad_stop_requested_{false};
  std::thread gamepad_thread_;
};

BridgeRunner::BridgeRunner(BridgeRunnerConfig config,
                           BridgeRunnerDependencies dependencies)
    : impl_(std::make_unique<Impl>(std::move(config), dependencies)) {
  impl_->bind_owner(this);
}

BridgeRunner::~BridgeRunner() = default;

const BridgeRunnerConfig& BridgeRunner::config() const noexcept {
  return impl_->config();
}

BridgeRunnerStatus BridgeRunner::status() const { return impl_->status(); }

RunnerCallResult BridgeRunner::initialize() { return impl_->initialize(); }

RunnerCallResult BridgeRunner::run() { return impl_->run(); }

RunnerCallResult BridgeRunner::request_stop() { return impl_->request_stop(); }

RunnerCallResult BridgeRunner::shutdown() { return impl_->shutdown(); }

CommandSubmissionResult BridgeRunner::submit_latest_command(CommandFrame command) {
  return impl_->submit_latest_command(std::move(command));
}

RunnerCallResult BridgeRunner::report_transport_fault(BridgeFault fault) {
  return impl_->report_transport_fault(std::move(fault));
}

void BridgeRunner::notify_state_available() { impl_->notify_state_available(); }

bool BridgeRunner::wait_for_state_available_until(
    const std::chrono::steady_clock::time_point deadline) {
  return impl_->wait_for_state_available_until(deadline);
}

}
