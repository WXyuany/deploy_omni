#include "../../../../include/bridge/adapters/unitree_g1/unitree_g1_adapter.hpp"

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <unordered_set>
#include <utility>

#include "../../../../include/bridge/adapters/unitree_g1/unitree_g1_low_level_session.hpp"
#include "../../../../include/bridge/core/bridge_runner.hpp"

namespace bridge::adapters::unitree_g1 {
namespace {

[[nodiscard]] bridge::core::BridgeFault make_fault(
    const bridge::core::FaultCategory category,
    const bridge::core::FaultSeverity severity,
    const bridge::core::LifecycleState lifecycle_state, std::string message,
    const std::uint64_t sequence, const std::uint64_t timestamp_ns,
    const bool latched = false) {
  return {
      .category = category,
      .severity = severity,
      .lifecycle_state = lifecycle_state,
      .message = std::move(message),
      .latched = latched,
      .sequence = sequence,
      .timestamp_ns = timestamp_ns,
  };
}

[[nodiscard]] bool vector_width_matches(const std::vector<double>& values,
                                        const std::size_t width) noexcept {
  return values.empty() || values.size() == width;
}

}  // namespace

class UnitreeG1Adapter::Impl {
 public:
  explicit Impl(UnitreeG1AdapterConfig config)
      : config_(std::move(config)),
        session_(LowLevelSessionConfig{
            .transport_mode = config_.session.transport_mode,
            .robot_id = config_.session.robot_id,
            .transport_endpoint = config_.session.transport_endpoint,
            .control_profile = config_.session.control_profile,
            .command_topic = config_.session.command_topic,
            .state_topic = config_.session.state_topic,
            .imu_topic = config_.session.imu_topic,
            .initialize_timeout_ns = config_.session.initialize_timeout_ns,
            .read_timeout_ns = config_.session.read_timeout_ns,
            .read_retry_attempts = config_.session.read_retry_attempts,
            .read_retry_interval_ns = config_.session.read_retry_interval_ns,
            .write_timeout_ns = config_.session.write_timeout_ns,
            .stale_state_after_ns = config_.session.read_timeout_ns,
            .require_motion_release = config_.session.require_motion_release,
            .verify_low_state_crc = config_.session.verify_low_state_crc,
            .forwarding_log_enabled = config_.session.forwarding_log_enabled,
            .hardware_slot_count = config_.command_width(),
        }) {}

  [[nodiscard]] const UnitreeG1AdapterConfig& config() const noexcept {
    return config_;
  }

  [[nodiscard]] UnitreeG1SdkFacade& sdk() noexcept { return sdk_; }
  [[nodiscard]] const UnitreeG1SdkFacade& sdk() const noexcept { return sdk_; }
  [[nodiscard]] UnitreeG1SensorSession& sensors() noexcept { return sensors_; }
  [[nodiscard]] const UnitreeG1SensorSession& sensors() const noexcept { return sensors_; }

  void set_runtime_event_sink(bridge::core::BridgeRuntimeEventSink* event_sink) {
    session_.set_state_available_callback(event_sink == nullptr
                                              ? std::function<void()>{}
                                              : [event_sink] { event_sink->notify_state_available(); });
  }

  [[nodiscard]] bridge::core::AdapterCallResult initialize() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (initialized_.load()) {
      return {.status = bridge::core::AdapterCallStatus::kOk,
              .detail = "unitree_g1 adapter already initialized",
              .fault = std::nullopt};
    }

    if (const auto validation = validate_config_locked(); !validation.succeeded()) {
      lifecycle_state_.store(bridge::core::LifecycleState::kFaulted);
      return validation;
    }

    lifecycle_state_.store(bridge::core::LifecycleState::kInitializing);
    const auto init = session_.initialize();
    if (!init.succeeded()) {
      lifecycle_state_.store(bridge::core::LifecycleState::kFaulted);
      return init;
    }

    if (config_.session.transport_mode != "mock") {
      if (!sensors_.initialize()) {
        static_cast<void>(session_.shutdown());
        lifecycle_state_.store(bridge::core::LifecycleState::kFaulted);
        return {.status = bridge::core::AdapterCallStatus::kRetryableError,
                .detail = "unitree_g1 sensor session initialization failed",
                .fault = std::nullopt};
      }
      const auto sdk_init = sdk_.initialize();
      if (!sdk_init.succeeded()) {
        sensors_.shutdown();
        static_cast<void>(session_.shutdown());
        lifecycle_state_.store(bridge::core::LifecycleState::kFaulted);
        return {
            .status = bridge::core::AdapterCallStatus::kRetryableError,
            .detail = "unitree_g1 SDK facade initialization failed: " + sdk_init.detail,
            .fault = std::nullopt,
        };
      }
    }

    initialized_.store(true);
    lifecycle_state_.store(bridge::core::LifecycleState::kWaitingForState);
    return {.status = bridge::core::AdapterCallStatus::kOk,
            .detail = "unitree_g1 adapter initialized",
            .fault = std::nullopt};
  }

  [[nodiscard]] bridge::core::AdapterReadResult read_state() {
    bridge::core::AdapterReadResult result;
    if (!initialized_.load()) {
      result.result = {
          .status = bridge::core::AdapterCallStatus::kRetryableError,
          .detail = "unitree_g1 adapter must be initialized before read",
          .fault = make_fault(bridge::core::FaultCategory::kAdapterRead,
                              bridge::core::FaultSeverity::kRecoverable,
                              lifecycle_state_.load(),
                              "unitree_g1 adapter must be initialized before read",
                              0, 0),
      };
      return result;
    }

    const auto session_read = session_.read_state();
    if (!session_read.result.succeeded() || !session_read.state.has_value()) {
      result.result = session_read.result;
      return result;
    }

    has_read_state_.store(true);
    lifecycle_state_.store(bridge::core::LifecycleState::kActive);

    bridge::core::StateFrame state;
    state.sequence = ++state_sequence_;
    state.tick = session_read.state->tick;
    state.sample_time_ns = session_read.state->sample_time_ns;
    state.receive_time_ns = session_read.state->receive_time_ns;
    state.publish_time_ns = session_read.state->sample_time_ns;
    state.lowstate_callback_store_duration_ns =
        session_read.state->callback_store_duration_ns;
    state.read_start_time_ns = session_read.state->read_start_time_ns;
    state.read_complete_time_ns = session_read.state->read_complete_time_ns;
    state.read_snapshot_duration_ns = session_read.state->read_snapshot_duration_ns;
    state.stateframe_build_duration_ns =
        session_read.state->stateframe_build_duration_ns;
    state.callback_drops = session_read.state->callback_drops;
    state.low_state_version = session_read.state->low_state_version;
    state.low_state_reserve = session_read.state->low_state_reserve;
    state.mode_pr = session_read.state->mode_pr;
    state.mode_machine = session_read.state->mode_machine;
    state.lifecycle_state = lifecycle_state_.load();
    {
      std::lock_guard<std::mutex> lock(command_state_mutex_);
      state.active_mode = last_mode_;
      state.mode_label = last_mode_label_;
    }
    state.valid = session_read.state->valid;
    state.stale = session_read.state->stale;
    state.degraded = session_read.state->degraded;
    state.command_stale = false;
    state.joints.position = session_read.state->position;
    state.joints.velocity = session_read.state->velocity;
    state.joints.effort = session_read.state->effort;
    state.joints.acceleration = session_read.state->acceleration;
    state.joints.motor_mode = session_read.state->motor_mode;
    state.joints.motor_temperature = session_read.state->motor_temperature;
    state.joints.motor_voltage = session_read.state->motor_voltage;
    state.joints.motor_sensor = session_read.state->motor_sensor;
    state.joints.motor_state = session_read.state->motor_state;
    state.joints.motor_reserve = session_read.state->motor_reserve;
    state.wireless_remote = session_read.state->wireless_remote;
    if (session_read.state->imu.has_value()) {
      state.imu = *session_read.state->imu;
    }
    if (session_read.state->fault.has_value()) {
      state.fault = *session_read.state->fault;
    }

    result.state = std::move(state);
    result.result = session_read.result;
    return result;
  }

  [[nodiscard]] bridge::core::AdapterCallResult write_command(
      const bridge::core::CommandFrame& command) {
    if (!initialized_.load()) {
      return {
          .status = bridge::core::AdapterCallStatus::kRetryableError,
          .detail = "unitree_g1 adapter must be initialized before command write",
          .fault = make_fault(bridge::core::FaultCategory::kAdapterWrite,
                              bridge::core::FaultSeverity::kRecoverable,
                              lifecycle_state_.load(),
                              "unitree_g1 adapter must be initialized before command write",
                              command.sequence, 0),
      };
    }
    if (!command.valid) {
      return {
          .status = bridge::core::AdapterCallStatus::kRetryableError,
          .detail = "command frame is marked invalid",
          .fault = make_fault(bridge::core::FaultCategory::kInvalidCommand,
                              bridge::core::FaultSeverity::kRecoverable,
                              lifecycle_state_.load(), "command frame is marked invalid",
                              command.sequence, command.receive_time_ns),
      };
    }
    if (command.stale) {
      return {
          .status = bridge::core::AdapterCallStatus::kRetryableError,
          .detail = "command frame is marked stale",
          .fault = make_fault(bridge::core::FaultCategory::kCommandStale,
                              bridge::core::FaultSeverity::kRecoverable,
                              lifecycle_state_.load(), "command frame is marked stale",
                              command.sequence, command.receive_time_ns),
      };
    }
    if (!command_block_widths_valid(command)) {
      return {
          .status = bridge::core::AdapterCallStatus::kRetryableError,
          .detail = "command joint vectors must either be empty or match G1 joint_count",
          .fault = make_fault(bridge::core::FaultCategory::kInvalidCommand,
                              bridge::core::FaultSeverity::kRecoverable,
                              lifecycle_state_.load(),
                              "command joint vectors must either be empty or match G1 joint_count",
                              command.sequence, command.receive_time_ns),
      };
    }
    if (config_.safety.require_initial_state_before_motion && !has_read_state_.load() &&
        command.mode != bridge::core::ControlMode::kSafeIdle) {
      return {
          .status = bridge::core::AdapterCallStatus::kRetryableError,
          .detail = "initial state read is required before motion commands",
          .fault = make_fault(bridge::core::FaultCategory::kSafety,
                              bridge::core::FaultSeverity::kRecoverable,
                              lifecycle_state_.load(),
                              "initial state read is required before motion commands",
                              command.sequence, command.receive_time_ns),
      };
    }

    {
      std::lock_guard<std::mutex> lock(command_state_mutex_);
      last_mode_ = command.mode;
      last_mode_label_ = command.mode_label;
    }
    return session_.write_command(command);
  }

  [[nodiscard]] bridge::core::AdapterHealthResult query_health() const {
    bridge::core::AdapterHealthResult result;
    result.health = session_.health();
    result.result = {.status = bridge::core::AdapterCallStatus::kOk,
                     .detail = "unitree_g1 adapter health queried",
                     .fault = std::nullopt};
    return result;
  }

  [[nodiscard]] bridge::core::AdapterCallResult shutdown() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (!initialized_.load()) {
      lifecycle_state_.store(bridge::core::LifecycleState::kStopped);
      return {.status = bridge::core::AdapterCallStatus::kOk,
              .detail = "unitree_g1 adapter already stopped",
              .fault = std::nullopt};
    }
    lifecycle_state_.store(bridge::core::LifecycleState::kStopping);
    const auto shutdown = session_.shutdown();
    sensors_.shutdown();
    const auto sdk_shutdown = sdk_.shutdown();
    initialized_.store(false);
    has_read_state_.store(false);
    lifecycle_state_.store(bridge::core::LifecycleState::kStopped);
    if (!shutdown.succeeded()) {
      return shutdown;
    }
    return {
        .status = sdk_shutdown.succeeded()
                      ? bridge::core::AdapterCallStatus::kOk
                      : bridge::core::AdapterCallStatus::kRetryableError,
        .detail = sdk_shutdown.detail,
        .fault = std::nullopt,
    };
  }

 private:
  [[nodiscard]] bridge::core::AdapterCallResult validate_config_locked() const {
    if (config_.joint_names.size() != kUnitreeG1JointCount) {
      return {
          .status = bridge::core::AdapterCallStatus::kFatalError,
          .detail = "unitree_g1 requires exactly 29 enabled joint names",
          .fault = make_fault(bridge::core::FaultCategory::kConfiguration,
                              bridge::core::FaultSeverity::kFatal,
                              bridge::core::LifecycleState::kInitializing,
                              "unitree_g1 requires exactly 29 enabled joint names",
                              0, 0),
      };
    }
    std::unordered_set<std::string> unique_names(config_.joint_names.begin(),
                                                 config_.joint_names.end());
    if (unique_names.size() != config_.joint_names.size()) {
      return {
          .status = bridge::core::AdapterCallStatus::kFatalError,
          .detail = "unitree_g1 joint names must be unique",
          .fault = make_fault(bridge::core::FaultCategory::kConfiguration,
                              bridge::core::FaultSeverity::kFatal,
                              bridge::core::LifecycleState::kInitializing,
                              "unitree_g1 joint names must be unique",
                              0, 0),
      };
    }
    return {.status = bridge::core::AdapterCallStatus::kOk,
            .detail = "unitree_g1 config validated",
            .fault = std::nullopt};
  }

  [[nodiscard]] bool command_block_widths_valid(
      const bridge::core::CommandFrame& command) const noexcept {
    const std::size_t width = config_.joint_count();
    return vector_width_matches(command.joints.position, width) &&
           vector_width_matches(command.joints.velocity, width) &&
           vector_width_matches(command.joints.effort, width) &&
           vector_width_matches(command.joints.stiffness, width) &&
           vector_width_matches(command.joints.damping, width);
  }

  UnitreeG1AdapterConfig config_;
  UnitreeG1LowLevelSession session_;
  UnitreeG1SdkFacade sdk_;
  UnitreeG1SensorSession sensors_;
  mutable std::mutex lifecycle_mutex_;
  mutable std::mutex command_state_mutex_;
  std::atomic<bool> initialized_{false};
  std::atomic<bool> has_read_state_{false};
  std::uint64_t state_sequence_{0};
  std::atomic<bridge::core::LifecycleState> lifecycle_state_{bridge::core::LifecycleState::kCreated};
  bridge::core::ControlMode last_mode_{bridge::core::ControlMode::kUnknown};
  std::string last_mode_label_;
};

UnitreeG1Adapter::UnitreeG1Adapter(UnitreeG1AdapterConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

UnitreeG1Adapter::~UnitreeG1Adapter() = default;
UnitreeG1Adapter::UnitreeG1Adapter(UnitreeG1Adapter&&) noexcept = default;
UnitreeG1Adapter& UnitreeG1Adapter::operator=(UnitreeG1Adapter&&) noexcept = default;

const UnitreeG1AdapterConfig& UnitreeG1Adapter::config() const noexcept {
  return impl_->config();
}

UnitreeG1SdkFacade& UnitreeG1Adapter::sdk() noexcept { return impl_->sdk(); }

const UnitreeG1SdkFacade& UnitreeG1Adapter::sdk() const noexcept {
  return impl_->sdk();
}

UnitreeG1SensorSession& UnitreeG1Adapter::sensors() noexcept { return impl_->sensors(); }

const UnitreeG1SensorSession& UnitreeG1Adapter::sensors() const noexcept {
  return impl_->sensors();
}

void UnitreeG1Adapter::set_runtime_event_sink(
    bridge::core::BridgeRuntimeEventSink* event_sink) {
  impl_->set_runtime_event_sink(event_sink);
}

bridge::core::AdapterCallResult UnitreeG1Adapter::initialize() {
  return impl_->initialize();
}

bridge::core::AdapterReadResult UnitreeG1Adapter::read_state() {
  return impl_->read_state();
}

bridge::core::AdapterCallResult UnitreeG1Adapter::write_command(
    const bridge::core::CommandFrame& command) {
  return impl_->write_command(command);
}

bridge::core::AdapterHealthResult UnitreeG1Adapter::query_health() const {
  return impl_->query_health();
}

bridge::core::AdapterCallResult UnitreeG1Adapter::shutdown() {
  return impl_->shutdown();
}

}  // namespace bridge::adapters::unitree_g1
