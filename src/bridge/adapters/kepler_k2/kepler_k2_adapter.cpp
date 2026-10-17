#include "../../../../include/bridge/adapters/kepler_k2/kepler_k2_adapter.hpp"

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bridge::adapters::kepler_k2 {
namespace {

constexpr std::uint64_t kFallbackTickNs = 1'000'000;
constexpr char kMotorApiTransportPrefix[] = "motor_api";

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

[[nodiscard]] bool wants_real_backend(const KeplerK2AdapterConfig& config) noexcept {
  return config.session.transport_endpoint.rfind(kMotorApiTransportPrefix, 0) == 0;
}

}

class KeplerK2Adapter::Impl {
 public:
  explicit Impl(KeplerK2AdapterConfig config) : config_(std::move(config)) {}

  [[nodiscard]] const KeplerK2AdapterConfig& config() const noexcept {
    return config_;
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
                                initialized_detail());
    }

    lifecycle_state_ = bridge::core::LifecycleState::kInitializing;
    synthetic_time_ns_ += tick_or_default(config_.session.initialize_timeout_ns);

    if (const auto validation = validate_config_locked(); !validation.succeeded()) {
      lifecycle_state_ = bridge::core::LifecycleState::kFaulted;
      return validation;
    }

    if (wants_real_backend(config_)) {
      lifecycle_state_ = bridge::core::LifecycleState::kFaulted;
      return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                               bridge::core::FaultCategory::kConfiguration,
                               bridge::core::FaultSeverity::kFatal,
                               "kepler_k2 real motor_api backend requested but not compiled in");
    }

    vendor_session_ = DeterministicVendorSession(config_.command_width());
    vendor_session_.set_active_mode(bridge::core::ControlMode::kSafeIdle,
                                    control_mode_label(
                                        bridge::core::ControlMode::kSafeIdle));
    initialized_ = true;
    connection_alive_ = true;
    lifecycle_state_ = bridge::core::LifecycleState::kWaitingForState;
    persistent_fault_ = persistent_runtime_fault_locked();

    return make_status_result(current_operational_status(), initialized_detail(),
                              persistent_fault_);
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
    ////// 读取机器人：这里当前还是 deterministic session 占位
    ////// 读取机器人：未来应该在这里调用真实 K2 底层接口获取最新关节状态
    vendor_session_.advance_state();
    if (!has_read_state_) {
      has_read_state_ = true;
      lifecycle_state_ = bridge::core::LifecycleState::kActive;
    }

    bridge::core::StateFrame state;
    state.sequence = ++state_sequence_;
    state.sample_time_ns = synthetic_time_ns_;
    state.publish_time_ns = synthetic_time_ns_;
    state.lifecycle_state = lifecycle_state_;
    state.active_mode = vendor_session_.active_mode();
    state.mode_label = vendor_session_.mode_label();
    state.valid = true;
    state.stale = false;
    state.degraded = is_persistently_degraded_locked();
    state.command_stale = false;
    ////// 读取机器人：这里当前还是从 deterministic session 回填 state.joints
    ////// 读取机器人：未来应该把真实机器人返回的位置/速度/力矩/加速度写入这些字段
    state.joints.position = vendor_session_.position();
    state.joints.velocity = vendor_session_.velocity();
    state.joints.effort = vendor_session_.effort();
    state.joints.acceleration = vendor_session_.acceleration();

    if (const auto imu = build_imu_sample_locked(); imu.has_value()) {
      state.imu = imu;
    }

    if (persistent_fault_.has_value()) {
      state.fault = persistent_fault_;
    }

    result.state = std::move(state);
    result.result = make_status_result(current_operational_status(),
                                       read_detail(), persistent_fault_);
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
          "command joint vectors must either be empty or match joint_count");
    }
    if (config_.safety.require_initial_state_before_motion && !has_read_state_ &&
        command.mode != bridge::core::ControlMode::kSafeIdle) {
      return make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                               bridge::core::FaultCategory::kSafety,
                               bridge::core::FaultSeverity::kRecoverable,
                               "initial state read is required before motion commands");
    }
    if (is_motion_mode(command.mode) &&
        !config_.safety.allow_command_when_imu_missing &&
        !imu_available_for_motion_locked()) {
      return make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                               bridge::core::FaultCategory::kSafety,
                               bridge::core::FaultSeverity::kRecoverable,
                               "configured IMU source is unavailable for command write");
    }

    synthetic_time_ns_ += tick_or_default(config_.session.write_timeout_ns);
    last_write_time_ns_ = synthetic_time_ns_;
    ////// 写入机器人：这里当前还是 deterministic session 占位
    ////// 写入机器人：未来应该把共享协议里的 command 转成 K2 真实电机命令并调用底层接口下发
    vendor_session_.apply_command(command);
    persistent_fault_ = persistent_runtime_fault_locked();

    const bridge::core::AdapterCallStatus status =
        command.degraded ? bridge::core::AdapterCallStatus::kDegraded
                         : current_operational_status();
    return make_status_result(status, write_detail(command), persistent_fault_);
  }

  [[nodiscard]] bridge::core::AdapterHealthResult query_health() const {
    std::lock_guard<std::mutex> lock(mutex_);

    bridge::core::AdapterHealthResult result;
    result.result = make_status_result(current_operational_status(),
                                       health_detail_locked(),
                                       persistent_runtime_fault_locked());
    result.health.state = health_state_locked();
    result.health.connection_alive = connection_alive_;
    result.health.degraded = is_persistently_degraded_locked();
    result.health.last_read_time_ns = last_read_time_ns_;
    result.health.last_write_time_ns = last_write_time_ns_;
    result.health.fault = persistent_runtime_fault_locked();
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
    vendor_session_.reset();
    persistent_fault_.reset();
    lifecycle_state_ = bridge::core::LifecycleState::kStopped;

    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              "adapter shutdown complete");
  }

 private:
  class DeterministicVendorSession {
   public:
    explicit DeterministicVendorSession(const std::size_t joint_count = 0)
        : position_(joint_count, 0.0),
          velocity_(joint_count, 0.0),
          effort_(joint_count, 0.0),
          acceleration_(joint_count, 0.0) {}

    void apply_command(const bridge::core::CommandFrame& command) {
      const std::size_t width = position_.size();
      std::vector<double> next_effort(width, 0.0);
      std::vector<double> next_acceleration(width, 0.0);

      for (std::size_t index = 0; index < width; ++index) {
        const double commanded_position = command.joints.position.empty()
                                              ? position_[index]
                                              : command.joints.position[index];
        const double commanded_velocity = command.joints.velocity.empty()
                                              ? velocity_[index]
                                              : command.joints.velocity[index];
        const double commanded_effort = command.joints.effort.empty()
                                            ? 0.0
                                            : command.joints.effort[index];
        const double position_error = commanded_position - position_[index];
        const double velocity_error = commanded_velocity - velocity_[index];
        next_effort[index] = commanded_effort + position_error + velocity_error;
        next_acceleration[index] = next_effort[index] - effort_[index];
      }

      effort_ = std::move(next_effort);
      acceleration_ = std::move(next_acceleration);

      set_active_mode(command.mode,
                      command.mode_label.empty() ? control_mode_label(command.mode)
                                                 : command.mode_label);
    }

    void advance_state() {
      for (std::size_t index = 0; index < position_.size(); ++index) {
        velocity_[index] += effort_[index] * 0.1;
        position_[index] += velocity_[index] * 0.1;
      }
    }

    void reset() {
      position_.clear();
      velocity_.clear();
      effort_.clear();
      acceleration_.clear();
      mode_ = bridge::core::ControlMode::kUnknown;
      mode_label_ = control_mode_label(mode_);
    }

    void set_active_mode(const bridge::core::ControlMode mode,
                         std::string label) {
      mode_ = mode;
      mode_label_ = std::move(label);
    }

    [[nodiscard]] const std::vector<double>& position() const noexcept {
      return position_;
    }

    [[nodiscard]] const std::vector<double>& velocity() const noexcept {
      return velocity_;
    }

    [[nodiscard]] const std::vector<double>& effort() const noexcept {
      return effort_;
    }

    [[nodiscard]] const std::vector<double>& acceleration() const noexcept {
      return acceleration_;
    }

    [[nodiscard]] bridge::core::ControlMode active_mode() const noexcept {
      return mode_;
    }

    [[nodiscard]] const std::string& mode_label() const noexcept {
      return mode_label_;
    }

   private:
    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> effort_;
    std::vector<double> acceleration_;
    bridge::core::ControlMode mode_{bridge::core::ControlMode::kUnknown};
    std::string mode_label_{control_mode_label(mode_)};
  };

  [[nodiscard]] bridge::core::AdapterCallResult validate_config_locked() {
    if (config_.session.robot_id.empty()) {
      return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                               bridge::core::FaultCategory::kConfiguration,
                               bridge::core::FaultSeverity::kFatal,
                               "session.robot_id must not be empty");
    }
    if (config_.session.transport_endpoint.empty()) {
      return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                               bridge::core::FaultCategory::kConfiguration,
                               bridge::core::FaultSeverity::kFatal,
                                "session.transport_endpoint must not be empty");
    }
    if (config_.joint_names.empty()) {
      return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                               bridge::core::FaultCategory::kConfiguration,
                               bridge::core::FaultSeverity::kFatal,
                               "joint_names must not be empty");
    }
    if (config_.command_width() == 0) {
      return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                               bridge::core::FaultCategory::kConfiguration,
                               bridge::core::FaultSeverity::kFatal,
                               "hardware slot count must be greater than zero");
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

    if (!config_.imu.has_value()) {
      if (!config_.safety.allow_command_when_imu_missing) {
        return make_error_result(
            bridge::core::AdapterCallStatus::kFatalError,
            bridge::core::FaultCategory::kConfiguration,
            bridge::core::FaultSeverity::kFatal,
            "imu config is required when IMU is required for motion");
      }
      return make_status_result(bridge::core::AdapterCallStatus::kOk,
                                "adapter config validated");
    }

    const ImuIntegrationConfig& imu = *config_.imu;
    if (imu.mode == ImuIntegrationMode::kSharedMemory) {
      if (!imu.shared_memory.has_value()) {
        return make_error_result(
            bridge::core::AdapterCallStatus::kFatalError,
            bridge::core::FaultCategory::kConfiguration,
            bridge::core::FaultSeverity::kFatal,
            "imu.shared_memory config is required for shared-memory mode");
      }

      const SharedMemoryImuConfig& shared_memory = *imu.shared_memory;
      if (shared_memory.region_name.empty() && shared_memory.mapping_name.empty()) {
        return make_error_result(
            bridge::core::AdapterCallStatus::kFatalError,
            bridge::core::FaultCategory::kConfiguration,
            bridge::core::FaultSeverity::kFatal,
            "shared-memory IMU config needs region_name or mapping_name");
      }
    }

    if (imu.mode != ImuIntegrationMode::kSharedMemory &&
        imu.shared_memory.has_value()) {
      return make_error_result(
          bridge::core::AdapterCallStatus::kFatalError,
          bridge::core::FaultCategory::kConfiguration,
          bridge::core::FaultSeverity::kFatal,
          "imu.shared_memory config is only valid for shared-memory mode");
    }

    if (!config_.safety.allow_command_when_imu_missing &&
        imu.mode == ImuIntegrationMode::kDisabled) {
      return make_error_result(
          bridge::core::AdapterCallStatus::kFatalError,
          bridge::core::FaultCategory::kConfiguration,
          bridge::core::FaultSeverity::kFatal,
          "imu mode must provide an available seam when IMU is required for motion");
    }

    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              "adapter config validated");
  }

  [[nodiscard]] bool command_block_widths_valid(
      const bridge::core::CommandFrame& command) const noexcept {
    const std::size_t width = config_.command_width();
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
                               category, bridge::core::FaultSeverity::kFatal,
                               "adapter is faulted");
    }

    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              "adapter session available");
  }

  [[nodiscard]] std::optional<bridge::core::ImuSample> build_imu_sample_locked()
      const {
    if (!config_.imu.has_value()) {
      return std::nullopt;
    }

    const ImuIntegrationConfig& imu_config = *config_.imu;
    if (imu_config.mode == ImuIntegrationMode::kDisabled) {
      return std::nullopt;
    }

    bridge::core::ImuSample imu;
    imu.sample_time_ns = last_read_time_ns_;

    if (imu_config.mode == ImuIntegrationMode::kAdapterOwned) {
      imu.valid = true;
      imu.stale = false;
      imu.orientation_xyzw = {0.0, 0.0,
                              static_cast<double>(state_sequence_ % 2), 1.0};
      imu.angular_velocity_xyz = {
          config_.joint_count() > 0 ? vendor_session_.velocity().front() : 0.0,
          static_cast<double>(state_sequence_),
          static_cast<double>(config_.joint_count()),
      };
      imu.linear_acceleration_xyz = {
          config_.joint_count() > 0 ? vendor_session_.acceleration().front() : 0.0,
          0.0,
          9.81,
      };
      return imu;
    }

    imu.valid = false;
    imu.stale = true;
    imu.orientation_xyzw = {0.0, 0.0, 0.0, 1.0};
    imu.angular_velocity_xyz = {0.0, 0.0, 0.0};
    imu.linear_acceleration_xyz = {0.0, 0.0, 0.0};
    return imu;
  }

  [[nodiscard]] bool imu_available_for_motion_locked() const noexcept {
    if (!config_.imu.has_value()) {
      return false;
    }

    return config_.imu->mode == ImuIntegrationMode::kAdapterOwned;
  }

  [[nodiscard]] bool is_persistently_degraded_locked() const noexcept {
    return persistent_runtime_fault_locked().has_value();
  }

  [[nodiscard]] bridge::core::HealthState health_state_locked() const noexcept {
    if (lifecycle_state_ == bridge::core::LifecycleState::kFaulted) {
      return bridge::core::HealthState::kFaulted;
    }
    if (!initialized_ || !connection_alive_) {
      return lifecycle_state_ == bridge::core::LifecycleState::kStopped
                 ? bridge::core::HealthState::kUnknown
                 : bridge::core::HealthState::kUnknown;
    }
    if (is_persistently_degraded_locked()) {
      return bridge::core::HealthState::kDegraded;
    }
    return bridge::core::HealthState::kHealthy;
  }

  [[nodiscard]] bridge::core::AdapterCallStatus current_operational_status() const
      noexcept {
    if (lifecycle_state_ == bridge::core::LifecycleState::kFaulted) {
      return bridge::core::AdapterCallStatus::kFatalError;
    }
    if (is_persistently_degraded_locked()) {
      return bridge::core::AdapterCallStatus::kDegraded;
    }
    return bridge::core::AdapterCallStatus::kOk;
  }

  [[nodiscard]] std::optional<bridge::core::BridgeFault>
  persistent_runtime_fault_locked() const {
    if (persistent_fault_.has_value() &&
        persistent_fault_->category == bridge::core::FaultCategory::kConfiguration) {
      return persistent_fault_;
    }

    if (!config_.imu.has_value()) {
      return std::nullopt;
    }

    const ImuIntegrationConfig& imu = *config_.imu;
    if (imu.mode != ImuIntegrationMode::kSharedMemory ||
        !imu.shared_memory.has_value() || !imu.shared_memory->required) {
      return std::nullopt;
    }

    return make_fault(
        bridge::core::FaultCategory::kAdapterHealth,
        bridge::core::FaultSeverity::kRecoverable, lifecycle_state_,
        "required shared-memory IMU backend is unavailable in deterministic seam",
        persistent_fault_sequence_, synthetic_time_ns_);
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

  [[nodiscard]] std::string initialized_detail() const {
    if (config_.imu.has_value() &&
        config_.imu->mode == ImuIntegrationMode::kSharedMemory) {
      return "deterministic K2 adapter initialized with shared-memory IMU seam";
    }
    if (config_.imu.has_value() &&
        config_.imu->mode == ImuIntegrationMode::kAdapterOwned) {
      return "deterministic K2 adapter initialized with adapter-owned IMU seam";
    }
    return "deterministic K2 adapter initialized";
  }

  [[nodiscard]] std::string read_detail() const {
    if (config_.imu.has_value() &&
        config_.imu->mode == ImuIntegrationMode::kSharedMemory) {
      return "deterministic state read with shared-memory IMU seam";
    }
    return "deterministic state read complete";
  }

  [[nodiscard]] std::string write_detail(
      const bridge::core::CommandFrame& command) const {
    return "deterministic command write applied in mode " +
           (command.mode_label.empty() ? control_mode_label(command.mode)
                                       : command.mode_label);
  }

  [[nodiscard]] std::string health_detail_locked() const {
    if (lifecycle_state_ == bridge::core::LifecycleState::kStopped) {
      return "adapter stopped";
    }
    if (!initialized_) {
      return "adapter not initialized";
    }
    if (is_persistently_degraded_locked()) {
      return "adapter healthy enough for testing with persistent degradation";
    }
    return "adapter healthy";
  }

  [[nodiscard]] std::uint64_t next_fault_sequence_locked() const noexcept {
    return ++fault_sequence_;
  }

  KeplerK2AdapterConfig config_;
  mutable std::mutex mutex_;
  mutable std::uint64_t fault_sequence_{0};
  std::uint64_t state_sequence_{0};
  std::uint64_t synthetic_time_ns_{0};
  std::uint64_t last_read_time_ns_{0};
  std::uint64_t last_write_time_ns_{0};
  std::uint64_t persistent_fault_sequence_{1};
  bridge::core::LifecycleState lifecycle_state_{
      bridge::core::LifecycleState::kCreated};
  bool initialized_{false};
  bool connection_alive_{false};
  bool has_read_state_{false};
  mutable std::optional<bridge::core::BridgeFault> persistent_fault_;
  DeterministicVendorSession vendor_session_;
};

KeplerK2Adapter::KeplerK2Adapter(KeplerK2AdapterConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

KeplerK2Adapter::~KeplerK2Adapter() {}

KeplerK2Adapter::KeplerK2Adapter(KeplerK2Adapter&&) noexcept = default;

KeplerK2Adapter& KeplerK2Adapter::operator=(KeplerK2Adapter&&) noexcept = default;

const KeplerK2AdapterConfig& KeplerK2Adapter::config() const noexcept {
  return impl_->config();
}

bridge::core::AdapterCallResult KeplerK2Adapter::initialize() {
  return impl_->initialize();
}

bridge::core::AdapterReadResult KeplerK2Adapter::read_state() {
  return impl_->read_state();
}

bridge::core::AdapterCallResult KeplerK2Adapter::write_command(
    const bridge::core::CommandFrame& command) {
  return impl_->write_command(command);
}

bridge::core::AdapterHealthResult KeplerK2Adapter::query_health() const {
  return impl_->query_health();
}

bridge::core::AdapterCallResult KeplerK2Adapter::shutdown() {
  return impl_->shutdown();
}

}
