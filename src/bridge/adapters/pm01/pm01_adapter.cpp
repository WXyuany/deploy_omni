#include "../../../../include/bridge/adapters/pm01/pm01_adapter.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bridge::adapters::pm01 {
namespace {

constexpr std::uint64_t kFallbackTickNs = 1'000'000;

[[nodiscard]] std::uint64_t tick_or_default(const std::uint64_t value) noexcept {
  return value == 0 ? kFallbackTickNs : value;
}

[[nodiscard]] std::string control_mode_label(
    const bridge::core::ControlMode mode) {
  using bridge::core::ControlMode;

  switch (mode) {
    case ControlMode::kUnknown:
      return "unknown";
    case ControlMode::kSafeIdle:
      return "safe_idle";
    case ControlMode::kTorque:
      return "torque";
    case ControlMode::kVelocity:
      return "velocity";
    case ControlMode::kPosition:
      return "position";
    case ControlMode::kHybrid:
      return "hybrid";
  }

  return "unknown";
}

[[nodiscard]] bool vector_width_matches(const std::vector<double>& values,
                                        const std::size_t width) noexcept {
  return values.empty() || values.size() == width;
}

void assign_to_slots(std::vector<double>* slots,
                     const std::vector<double>& values,
                     const std::vector<std::int64_t>& indices) {
  if (values.empty()) {
    return;
  }

  for (std::size_t joint = 0; joint < values.size(); ++joint) {
    (*slots)[static_cast<std::size_t>(indices[joint])] = values[joint];
  }
}

[[nodiscard]] std::vector<double> project_from_slots(
    const std::vector<double>& slots, const std::vector<std::int64_t>& indices) {
  std::vector<double> values(indices.size(), 0.0);
  for (std::size_t joint = 0; joint < indices.size(); ++joint) {
    values[joint] = slots[static_cast<std::size_t>(indices[joint])];
  }
  return values;
}

[[nodiscard]] bool is_motion_mode(const bridge::core::ControlMode mode) noexcept {
  return mode != bridge::core::ControlMode::kUnknown &&
         mode != bridge::core::ControlMode::kSafeIdle;
}

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

}  // namespace

class Pm01Adapter::Impl {
 public:
  explicit Impl(Pm01AdapterConfig config) : config_(std::move(config)) {}

  [[nodiscard]] const Pm01AdapterConfig& config() const noexcept {
    return config_;
  }

  void update_motion_task(std::optional<std::string> task) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_motion_task_ = std::move(task);
  }

  [[nodiscard]] bridge::core::AdapterCallResult initialize() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (lifecycle_state_ == bridge::core::LifecycleState::kStopped) {
      return make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                               bridge::core::FaultCategory::kAdapterInitialize,
                               bridge::core::FaultSeverity::kRecoverable,
                               "adapter already stopped");
    }
    if (initialized_) {
      return make_status_result(current_operational_status(),
                                "pm01 adapter already initialized",
                                persistent_fault_);
    }

    lifecycle_state_ = bridge::core::LifecycleState::kInitializing;
    synthetic_time_ns_ += tick_or_default(config_.session.initialize_timeout_ns);

    if (const auto validation = validate_config_locked(); !validation.succeeded()) {
      lifecycle_state_ = bridge::core::LifecycleState::kFaulted;
      return validation;
    }

    dds_position_.assign(kPm01DdsMotorCount, 0.0);
    dds_velocity_.assign(kPm01DdsMotorCount, 0.0);
    dds_effort_.assign(kPm01DdsMotorCount, 0.0);
    dds_stiffness_.assign(kPm01DdsMotorCount, 0.0);
    dds_damping_.assign(kPm01DdsMotorCount, 0.0);
    dds_acceleration_.assign(kPm01DdsMotorCount, 0.0);
    active_mode_ = bridge::core::ControlMode::kSafeIdle;
    mode_label_ = control_mode_label(active_mode_);
    initialized_ = true;
    connection_alive_ = true;
    has_read_state_ = false;
    current_motion_task_ = config_.motion_state.initial_task;
    lifecycle_state_ = bridge::core::LifecycleState::kWaitingForState;
    persistent_fault_.reset();

    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              "pm01 adapter initialized");
  }

  [[nodiscard]] bridge::core::AdapterReadResult read_state() {
    std::lock_guard<std::mutex> lock(mutex_);

    bridge::core::AdapterReadResult result;
    if (const auto availability = ensure_session_available_locked(
            bridge::core::FaultCategory::kAdapterRead,
            "adapter must be initialized before read");
        !availability.succeeded()) {
      result.result = availability;
      return result;
    }

    synthetic_time_ns_ += tick_or_default(config_.session.read_timeout_ns);
    last_read_time_ns_ = synthetic_time_ns_;
    if (!has_read_state_) {
      has_read_state_ = true;
      lifecycle_state_ = bridge::core::LifecycleState::kActive;
    }

    bridge::core::StateFrame state;
    state.sequence = ++state_sequence_;
    state.sample_time_ns = synthetic_time_ns_;
    state.publish_time_ns = synthetic_time_ns_;
    state.lifecycle_state = lifecycle_state_;
    state.active_mode = active_mode_;
    state.mode_label = mode_label_;
    state.valid = true;
    state.stale = false;
    state.degraded = false;
    state.command_stale = false;
    state.joints.position = project_from_slots(dds_position_, config_.dds_joint_indices);
    state.joints.velocity = project_from_slots(dds_velocity_, config_.dds_joint_indices);
    state.joints.effort = project_from_slots(dds_effort_, config_.dds_joint_indices);
    state.joints.acceleration =
        project_from_slots(dds_acceleration_, config_.dds_joint_indices);

    if (const auto imu = build_imu_sample_locked(); imu.has_value()) {
      state.imu = imu;
    }

    result.state = std::move(state);
    result.result = make_status_result(bridge::core::AdapterCallStatus::kOk,
                                       "pm01 state read complete");
    return result;
  }

  [[nodiscard]] bridge::core::AdapterCallResult write_command(
      const bridge::core::CommandFrame& command) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (const auto availability = ensure_session_available_locked(
            bridge::core::FaultCategory::kAdapterWrite,
            "adapter must be initialized before command write");
        !availability.succeeded()) {
      return availability;
    }

    if (!command.valid) {
      return make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                               bridge::core::FaultCategory::kInvalidCommand,
                               bridge::core::FaultSeverity::kRecoverable,
                               "command frame is marked invalid");
    }
    if (command.stale) {
      return make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                               bridge::core::FaultCategory::kCommandStale,
                               bridge::core::FaultSeverity::kRecoverable,
                               "command frame is marked stale");
    }
    if (command.mode == bridge::core::ControlMode::kUnknown) {
      return make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                               bridge::core::FaultCategory::kInvalidCommand,
                               bridge::core::FaultSeverity::kRecoverable,
                               "command mode must be specified");
    }
    if (!command_block_widths_valid(command)) {
      return make_error_result(
          bridge::core::AdapterCallStatus::kRetryableError,
          bridge::core::FaultCategory::kInvalidCommand,
          bridge::core::FaultSeverity::kRecoverable,
          "command joint vectors must either be empty or match PM01 joint_count");
    }
    if (config_.safety.require_initial_state_before_motion && !has_read_state_ &&
        command.mode != bridge::core::ControlMode::kSafeIdle) {
      return make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                               bridge::core::FaultCategory::kSafety,
                               bridge::core::FaultSeverity::kRecoverable,
                               "initial state read is required before motion commands");
    }
    if (is_motion_mode(command.mode) && config_.motion_state.required) {
      if (!current_motion_task_.has_value()) {
        return make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                                 bridge::core::FaultCategory::kSafety,
                                 bridge::core::FaultSeverity::kRecoverable,
                                 "motion state is required before PM01 command write");
      }
      if (*current_motion_task_ != config_.motion_state.required_task) {
        return make_error_result(
            bridge::core::AdapterCallStatus::kRetryableError,
            bridge::core::FaultCategory::kSafety,
            bridge::core::FaultSeverity::kRecoverable,
            "PM01 command forwarding is blocked until " +
                config_.motion_state.required_task + " is active");
      }
    }

    synthetic_time_ns_ += tick_or_default(config_.session.write_timeout_ns);
    last_write_time_ns_ = synthetic_time_ns_;
    dds_acceleration_.assign(kPm01DdsMotorCount, 0.0);
    if (!command.joints.velocity.empty()) {
      dds_acceleration_ = dds_velocity_;
    }
    assign_to_slots(&dds_position_, command.joints.position, config_.dds_joint_indices);
    assign_to_slots(&dds_velocity_, command.joints.velocity, config_.dds_joint_indices);
    assign_to_slots(&dds_effort_, command.joints.effort, config_.dds_joint_indices);
    assign_to_slots(&dds_stiffness_, command.joints.stiffness,
                    config_.dds_joint_indices);
    assign_to_slots(&dds_damping_, command.joints.damping, config_.dds_joint_indices);
    active_mode_ = command.mode;
    mode_label_ = command.mode_label.empty() ? control_mode_label(command.mode)
                                             : command.mode_label;

    const bridge::core::AdapterCallStatus status =
        command.degraded ? bridge::core::AdapterCallStatus::kDegraded
                         : bridge::core::AdapterCallStatus::kOk;
    return make_status_result(status, "pm01 command write complete");
  }

  [[nodiscard]] bridge::core::AdapterCallResult write_safe_command() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!config_.safety.safe_command_supported) {
      return {
          .status = bridge::core::AdapterCallStatus::kUnsupported,
          .detail = "safe command is disabled by adapter config",
          .fault = std::nullopt,
      };
    }

    if (const auto availability = ensure_session_available_locked(
            bridge::core::FaultCategory::kSafety,
            "adapter must be initialized before safe command write");
        !availability.succeeded()) {
      return availability;
    }

    synthetic_time_ns_ += tick_or_default(config_.session.write_timeout_ns);
    last_write_time_ns_ = synthetic_time_ns_;
    std::fill(dds_velocity_.begin(), dds_velocity_.end(), 0.0);
    std::fill(dds_effort_.begin(), dds_effort_.end(), 0.0);
    std::fill(dds_acceleration_.begin(), dds_acceleration_.end(), 0.0);
    active_mode_ = bridge::core::ControlMode::kSafeIdle;
    mode_label_ = control_mode_label(active_mode_);

    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              "pm01 safe command applied");
  }

  [[nodiscard]] bridge::core::AdapterHealthResult query_health() const {
    std::lock_guard<std::mutex> lock(mutex_);

    bridge::core::AdapterHealthResult result;
    result.result = make_status_result(current_operational_status(),
                                       "pm01 adapter health queried",
                                       persistent_fault_);
    result.health.state = health_state_locked();
    result.health.safe_command_supported = config_.safety.safe_command_supported;
    result.health.connection_alive = connection_alive_;
    result.health.degraded = false;
    result.health.last_read_time_ns = last_read_time_ns_;
    result.health.last_write_time_ns = last_write_time_ns_;
    result.health.fault = persistent_fault_;
    return result;
  }

  [[nodiscard]] bridge::core::AdapterCallResult shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (lifecycle_state_ == bridge::core::LifecycleState::kStopped) {
      return make_status_result(bridge::core::AdapterCallStatus::kOk,
                                "adapter already stopped");
    }

    lifecycle_state_ = bridge::core::LifecycleState::kStopping;
    synthetic_time_ns_ += tick_or_default(config_.session.write_timeout_ns);
    connection_alive_ = false;
    initialized_ = false;
    has_read_state_ = false;
    current_motion_task_.reset();
    dds_position_.clear();
    dds_velocity_.clear();
    dds_effort_.clear();
    dds_stiffness_.clear();
    dds_damping_.clear();
    dds_acceleration_.clear();
    active_mode_ = bridge::core::ControlMode::kUnknown;
    mode_label_ = control_mode_label(active_mode_);
    persistent_fault_.reset();
    lifecycle_state_ = bridge::core::LifecycleState::kStopped;

    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              "adapter shutdown complete");
  }

 private:
  [[nodiscard]] bridge::core::AdapterCallResult validate_config_locked() {
    if (config_.session.robot_id.empty()) {
      return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                               bridge::core::FaultCategory::kConfiguration,
                               bridge::core::FaultSeverity::kFatal,
                               "session.robot_id must not be empty");
    }
    if (config_.session.transport_address.empty()) {
      return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                               bridge::core::FaultCategory::kConfiguration,
                               bridge::core::FaultSeverity::kFatal,
                               "session.transport_address must not be empty");
    }
    if (config_.joint_names.size() != kPm01JointCount) {
      return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                               bridge::core::FaultCategory::kConfiguration,
                               bridge::core::FaultSeverity::kFatal,
                               "joint_names must contain exactly 24 PM01 joints");
    }

    std::unordered_set<std::string> seen_joint_names;
    seen_joint_names.reserve(config_.joint_names.size());
    for (const std::string& joint_name : config_.joint_names) {
      if (joint_name.empty()) {
        return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                                 bridge::core::FaultCategory::kConfiguration,
                                 bridge::core::FaultSeverity::kFatal,
                                 "joint_names must not contain empty entries");
      }
      if (!seen_joint_names.insert(joint_name).second) {
        return make_error_result(
            bridge::core::AdapterCallStatus::kFatalError,
            bridge::core::FaultCategory::kConfiguration,
            bridge::core::FaultSeverity::kFatal,
            "joint_names must contain unique entries");
      }
    }

    if (config_.dds_joint_indices.size() != kPm01JointCount) {
      return make_error_result(
          bridge::core::AdapterCallStatus::kFatalError,
          bridge::core::FaultCategory::kConfiguration,
          bridge::core::FaultSeverity::kFatal,
          "dds_joint_indices must contain exactly 24 entries");
    }

    std::unordered_set<std::int64_t> seen_indices;
    seen_indices.reserve(config_.dds_joint_indices.size());
    for (const std::int64_t index : config_.dds_joint_indices) {
      if (index < 0 || index >= static_cast<std::int64_t>(kPm01DdsMotorCount)) {
        return make_error_result(
            bridge::core::AdapterCallStatus::kFatalError,
            bridge::core::FaultCategory::kConfiguration,
            bridge::core::FaultSeverity::kFatal,
            "dds_joint_indices contains an out-of-range slot");
      }
      if (!seen_indices.insert(index).second) {
        return make_error_result(
            bridge::core::AdapterCallStatus::kFatalError,
            bridge::core::FaultCategory::kConfiguration,
            bridge::core::FaultSeverity::kFatal,
            "dds_joint_indices must not contain duplicate slots");
      }
    }

    if (config_.motion_state.required && config_.motion_state.required_task.empty()) {
      return make_error_result(
          bridge::core::AdapterCallStatus::kFatalError,
          bridge::core::FaultCategory::kConfiguration,
          bridge::core::FaultSeverity::kFatal,
          "motion_state.required_task must not be empty when motion gate is enabled");
    }

    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              "pm01 adapter config validated");
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

  [[nodiscard]] bridge::core::AdapterCallResult ensure_session_available_locked(
      const bridge::core::FaultCategory category,
      const char* detail) const {
    if (!initialized_ || !connection_alive_) {
      return make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                               category,
                               bridge::core::FaultSeverity::kRecoverable,
                               detail);
    }
    if (lifecycle_state_ == bridge::core::LifecycleState::kStopping ||
        lifecycle_state_ == bridge::core::LifecycleState::kStopped) {
      return make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                               category,
                               bridge::core::FaultSeverity::kRecoverable,
                               "adapter is stopping or stopped");
    }
    if (lifecycle_state_ == bridge::core::LifecycleState::kFaulted) {
      return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                               category,
                               bridge::core::FaultSeverity::kFatal,
                               "adapter is faulted");
    }

    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              "adapter session available");
  }

  [[nodiscard]] std::optional<bridge::core::ImuSample> build_imu_sample_locked() const {
    if (!config_.publish_default_imu_when_missing) {
      return std::nullopt;
    }

    bridge::core::ImuSample imu;
    imu.sample_time_ns = last_read_time_ns_;
    imu.valid = false;
    imu.stale = true;
    imu.orientation_xyzw = {0.0, 0.0, 0.0, 1.0};
    imu.angular_velocity_xyz = {0.0, 0.0, 0.0};
    imu.linear_acceleration_xyz = {0.0, 0.0, 0.0};
    return imu;
  }

  [[nodiscard]] bridge::core::HealthState health_state_locked() const noexcept {
    if (lifecycle_state_ == bridge::core::LifecycleState::kFaulted) {
      return bridge::core::HealthState::kFaulted;
    }
    if (!initialized_ || !connection_alive_) {
      return bridge::core::HealthState::kUnknown;
    }
    return bridge::core::HealthState::kHealthy;
  }

  [[nodiscard]] bridge::core::AdapterCallStatus current_operational_status() const noexcept {
    if (lifecycle_state_ == bridge::core::LifecycleState::kFaulted) {
      return bridge::core::AdapterCallStatus::kFatalError;
    }
    return bridge::core::AdapterCallStatus::kOk;
  }

  [[nodiscard]] bridge::core::AdapterCallResult make_status_result(
      const bridge::core::AdapterCallStatus status, std::string detail,
      std::optional<bridge::core::BridgeFault> fault = std::nullopt) const {
    return {
        .status = status,
        .detail = std::move(detail),
        .fault = std::move(fault),
    };
  }

  [[nodiscard]] bridge::core::AdapterCallResult make_error_result(
      const bridge::core::AdapterCallStatus status,
      const bridge::core::FaultCategory category,
      const bridge::core::FaultSeverity severity, std::string detail) const {
    const bridge::core::BridgeFault fault =
        make_fault(category, severity, lifecycle_state_, detail,
                   next_fault_sequence_locked(), synthetic_time_ns_, true);
    if (category == bridge::core::FaultCategory::kConfiguration ||
        status == bridge::core::AdapterCallStatus::kFatalError) {
      persistent_fault_ = fault;
    }

    return make_status_result(status, std::move(detail), fault);
  }

  [[nodiscard]] std::uint64_t next_fault_sequence_locked() const noexcept {
    return ++fault_sequence_;
  }

  Pm01AdapterConfig config_;
  mutable std::mutex mutex_;
  mutable std::optional<bridge::core::BridgeFault> persistent_fault_;
  mutable std::uint64_t fault_sequence_{0};
  std::uint64_t synthetic_time_ns_{0};
  std::uint64_t state_sequence_{0};
  std::uint64_t last_read_time_ns_{0};
  std::uint64_t last_write_time_ns_{0};
  bool initialized_{false};
  bool connection_alive_{false};
  bool has_read_state_{false};
  bridge::core::LifecycleState lifecycle_state_{bridge::core::LifecycleState::kCreated};
  bridge::core::ControlMode active_mode_{bridge::core::ControlMode::kUnknown};
  std::string mode_label_{control_mode_label(active_mode_)};
  std::optional<std::string> current_motion_task_;
  std::vector<double> dds_position_;
  std::vector<double> dds_velocity_;
  std::vector<double> dds_effort_;
  std::vector<double> dds_stiffness_;
  std::vector<double> dds_damping_;
  std::vector<double> dds_acceleration_;
};

Pm01Adapter::Pm01Adapter(Pm01AdapterConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

Pm01Adapter::~Pm01Adapter() = default;

Pm01Adapter::Pm01Adapter(Pm01Adapter&&) noexcept = default;

Pm01Adapter& Pm01Adapter::operator=(Pm01Adapter&&) noexcept = default;

const Pm01AdapterConfig& Pm01Adapter::config() const noexcept {
  return impl_->config();
}

void Pm01Adapter::update_motion_task(std::optional<std::string> task) {
  impl_->update_motion_task(std::move(task));
}

bridge::core::AdapterCallResult Pm01Adapter::initialize() {
  return impl_->initialize();
}

bridge::core::AdapterReadResult Pm01Adapter::read_state() {
  return impl_->read_state();
}

bridge::core::AdapterCallResult Pm01Adapter::write_command(
    const bridge::core::CommandFrame& command) {
  return impl_->write_command(command);
}

bridge::core::AdapterCallResult Pm01Adapter::write_safe_command() {
  return impl_->write_safe_command();
}

bridge::core::AdapterHealthResult Pm01Adapter::query_health() const {
  return impl_->query_health();
}

bridge::core::AdapterCallResult Pm01Adapter::shutdown() {
  return impl_->shutdown();
}

}  // namespace bridge::adapters::pm01
