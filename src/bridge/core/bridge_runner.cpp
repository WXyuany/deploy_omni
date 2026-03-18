#include "../../../include/bridge/core/bridge_runner.hpp"

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace bridge::core {
namespace {

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
      status_.startup_idle_active = true;
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
      cached_safe_command_supported_ = false;
      return handle_adapter_failure_locked(
          health.result, FaultCategory::kAdapterHealth,
          "adapter health query failed during initialize", true);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      cached_safe_command_supported_ = health.health.safe_command_supported;
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
    update_status_from_state(state, read_result.result);

    RunnerCallResult publish_result = publish_state(state);
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
    AdapterCallResult safe_write_result;
    AdapterCallResult adapter_shutdown_result;
    bool write_safe_command = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (status_.lifecycle_state == LifecycleState::kStopped) {
        return make_runner_result(RunnerCallStatus::kOk,
                                  "runner already stopped");
      }

      status_.stop_requested = true;
      status_.lifecycle_state = LifecycleState::kStopping;
      write_safe_command = cached_safe_command_supported_;
    }

    if (dependencies_.command_ingress != nullptr) {
      ingress_stop_result = dependencies_.command_ingress->stop();
    }
    if (write_safe_command && dependencies_.adapter != nullptr) {
      safe_write_result = dependencies_.adapter->write_safe_command();
    }
    if (dependencies_.state_egress != nullptr) {
      egress_stop_result = dependencies_.state_egress->stop();
    }
    if (dependencies_.adapter != nullptr) {
      adapter_shutdown_result = dependencies_.adapter->shutdown();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    status_.startup_idle_active = false;

    RunnerCallStatus final_status = RunnerCallStatus::kOk;
    std::string detail = "runner shutdown complete";

    if (dependencies_.command_ingress != nullptr &&
        !ingress_stop_result.succeeded()) {
      final_status = ingress_stop_result.status;
      detail = ingress_stop_result.detail;
      update_degraded_from_runner_locked(ingress_stop_result);
    }

    if (write_safe_command && !safe_write_result.succeeded()) {
      final_status = map_adapter_status(safe_write_result.status);
      detail = safe_write_result.detail;
      update_degraded_from_adapter_locked(safe_write_result);
      if (is_error_status(safe_write_result.status)) {
        BridgeFault fault = safe_write_result.fault.value_or(make_fault(
            FaultCategory::kSafety, FaultSeverity::kRecoverable,
            LifecycleState::kStopping, "safe command write during shutdown failed",
            status_.last_cycle_time_ns));
        fault.lifecycle_state = LifecycleState::kStopping;
        finalize_fault_locked(&fault);
      }
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
    std::lock_guard<std::mutex> lock(mutex_);

    if (status_.lifecycle_state == LifecycleState::kStopping ||
        status_.lifecycle_state == LifecycleState::kStopped) {
      CommandSubmissionResult submission;
      submission.result = make_runner_result(RunnerCallStatus::kRejected,
                                             "runner is stopping or stopped");
      return submission;
    }
    if (!command.valid || command.mode == ControlMode::kUnknown) {
      CommandSubmissionResult submission;
      submission.result = make_runner_result(RunnerCallStatus::kRejected,
                                             "invalid command rejected");
      return submission;
    }

    const bool replaced_previous = status_.command_buffer.has_latest_command;
    status_.command_buffer.has_latest_command = true;
    status_.command_buffer.latest_command_stale = false;
    status_.command_buffer.latest_command_sequence = command.sequence;
    status_.command_buffer.latest_command_receive_time_ns =
        command.receive_time_ns;
    latest_command_ = std::move(command);
    status_.first_command_received = true;

    CommandSubmissionResult submission;
    submission.replaced_previous = replaced_previous;
    submission.stored_sequence = status_.command_buffer.latest_command_sequence;
    submission.result = make_runner_result(RunnerCallStatus::kOk,
                                           "latest command stored");
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

  [[nodiscard]] RunnerCallResult publish_state(StateFrame& state) {
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
    }

    RunnerCallResult publish_result = dependencies_.state_egress->publish(state);
    std::lock_guard<std::mutex> lock(mutex_);
    if (publish_result.succeeded()) {
      status_.last_state_publish_time_ns = state.publish_time_ns;
      update_degraded_from_runner_locked(publish_result);
      return publish_result;
    }
    return handle_runner_failure_locked(
        publish_result, FaultCategory::kTransport, "state publish failed",
        config_.fault_policy.fail_stop_on_transport_fault);
  }

  [[nodiscard]] RunnerCallResult evaluate_health() {
    AdapterHealthResult health = dependencies_.adapter->query_health();

    std::lock_guard<std::mutex> lock(mutex_);
    cached_safe_command_supported_ =
        health.result.succeeded() && health.health.safe_command_supported;
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
    std::optional<CommandFrame> command_to_write;
    bool issue_startup_idle = false;
    bool issue_fault_safe_command = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      update_command_staleness_locked(cycle_time_ns);

      if (status_.fail_stop_active) {
        issue_fault_safe_command =
            config_.fault_policy.write_safe_command_on_fault &&
            cached_safe_command_supported_ && !fault_safe_command_written_;
      } else if (status_.command_buffer.has_latest_command &&
                 !status_.command_buffer.latest_command_stale) {
        command_to_write = latest_command_;
      } else {
        issue_startup_idle = !status_.first_command_received;
      }
    }

    if (issue_fault_safe_command) {
      AdapterCallResult safe_result = dependencies_.adapter->write_safe_command();
      std::lock_guard<std::mutex> lock(mutex_);
      if (!safe_result.succeeded()) {
        return handle_adapter_failure_locked(
            safe_result, FaultCategory::kSafety,
            "safe command write failed while faulted", true);
      }
      fault_safe_command_written_ = true;
      status_.startup_idle_active = false;
      update_degraded_from_adapter_locked(safe_result);
      return make_runner_result(safe_result.status == AdapterCallStatus::kDegraded
                                    ? RunnerCallStatus::kDegraded
                                    : RunnerCallStatus::kOk,
                                "fault safe command written",
                                status_.latched_fault);
    }

    if (command_to_write.has_value()) {
      AdapterCallResult write_result =
          dependencies_.adapter->write_command(*command_to_write);
      std::lock_guard<std::mutex> lock(mutex_);
      if (!write_result.succeeded()) {
        return handle_adapter_failure_locked(write_result,
                                             FaultCategory::kAdapterWrite,
                                             "adapter command write failed",
                                             true);
      }

      status_.startup_idle_active = false;
      status_.command_stale = false;
      status_.command_buffer.has_active_command = true;
      status_.command_buffer.active_command_sequence = command_to_write->sequence;
      status_.command_buffer.active_mode = command_to_write->mode;
      update_degraded_from_adapter_locked(write_result);
      return make_runner_result(write_result.status == AdapterCallStatus::kDegraded
                                    ? RunnerCallStatus::kDegraded
                                    : RunnerCallStatus::kOk,
                                "active command written", status_.latched_fault);
    }

    if (!issue_startup_idle) {
      std::lock_guard<std::mutex> lock(mutex_);
      return make_runner_result(status_.degraded ? RunnerCallStatus::kDegraded
                                                 : RunnerCallStatus::kOk,
                                status_.command_stale ? "command stale" : "no command write",
                                status_.latched_fault);
    }

    return apply_startup_idle_policy();
  }

  [[nodiscard]] RunnerCallResult apply_startup_idle_policy() {
    StartupIdlePolicy policy;
    bool safe_supported;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      policy = config_.startup_idle_policy;
      safe_supported = cached_safe_command_supported_;
    }

    if (policy == StartupIdlePolicy::kNoCommandWrite) {
      std::lock_guard<std::mutex> lock(mutex_);
      status_.startup_idle_active = true;
      return make_runner_result(RunnerCallStatus::kOk,
                                "startup idle holding without command write",
                                status_.latched_fault);
    }

    if (safe_supported) {
      AdapterCallResult safe_result = dependencies_.adapter->write_safe_command();
      std::lock_guard<std::mutex> lock(mutex_);
      if (!safe_result.succeeded()) {
        return handle_adapter_failure_locked(
            safe_result, FaultCategory::kSafety,
            "startup safe command write failed", true);
      }
      status_.startup_idle_active = true;
      update_degraded_from_adapter_locked(safe_result);
      return make_runner_result(safe_result.status == AdapterCallStatus::kDegraded
                                    ? RunnerCallStatus::kDegraded
                                    : RunnerCallStatus::kOk,
                                "startup safe idle command written",
                                status_.latched_fault);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (policy == StartupIdlePolicy::kRequireSafeCommand) {
      const BridgeFault fault = make_fault(
          FaultCategory::kSafety, FaultSeverity::kFatal, status_.lifecycle_state,
          "startup idle requires safe command support", status_.last_cycle_time_ns);
      return latch_fault_locked(fault, true, RunnerCallStatus::kFatalError);
    }

    status_.startup_idle_active = true;
    return make_runner_result(RunnerCallStatus::kOk,
                              "startup idle running without safe command support",
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
  BridgeRunnerStatus status_;
  std::optional<CommandFrame> latest_command_;
  std::uint64_t fault_sequence_{0};
  bool cached_safe_command_supported_{false};
  bool fault_safe_command_written_{false};
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

}
