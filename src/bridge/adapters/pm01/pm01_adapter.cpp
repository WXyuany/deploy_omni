#include "../../../../include/bridge/adapters/pm01/pm01_adapter.hpp"
#include "../../../../include/bridge/core/bridge_runner.hpp"
#include "bridge/build_config.hpp"

#include <atomic>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include "interface_protocol/msg/imu_info.hpp"
#include "interface_protocol/msg/alert.hpp"
#include "interface_protocol/msg/joint_command.hpp"
#include "interface_protocol/msg/joint_state.hpp"
#include "interface_protocol/msg/motion_state.hpp"
#include "interface_protocol/msg/power_info.hpp"
#endif

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

[[nodiscard]] std::uint8_t encode_control_mode(
    const bridge::core::ControlMode mode) noexcept {
  switch (mode) {
    case bridge::core::ControlMode::kUnknown:
      return 0U;
    case bridge::core::ControlMode::kSafeIdle:
      return 1U;
    case bridge::core::ControlMode::kTorque:
      return 2U;
    case bridge::core::ControlMode::kVelocity:
      return 3U;
    case bridge::core::ControlMode::kPosition:
      return 4U;
    case bridge::core::ControlMode::kHybrid:
      return 5U;
  }
  return 0U;
}

[[nodiscard]] bool vector_width_matches(const std::vector<double>& values,
                                        const std::size_t width) noexcept {
  return values.empty() || values.size() == width;
}

[[nodiscard]] std::uint8_t clamp_percent_to_u8(const float value) noexcept {
  if (value <= 0.0F) {
    return 0U;
  }
  if (value >= 100.0F) {
    return 100U;
  }
  return static_cast<std::uint8_t>(value);
}

[[nodiscard]] std::uint32_t voltage_to_millivolts(const float value) noexcept {
  return value <= 0.0F ? 0U : static_cast<std::uint32_t>(value * 1000.0F);
}

[[nodiscard]] std::int32_t current_to_milliamps(const float value) noexcept {
  return static_cast<std::int32_t>(value * 1000.0F);
}

#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
[[nodiscard]] bridge::core::ImuSample make_imu_sample_from_ros2(
    const interface_protocol::msg::ImuInfo& msg,
    const std::uint64_t sample_time_ns) {
  bridge::core::ImuSample sample;
  sample.sample_time_ns = sample_time_ns;
  sample.valid = true;
  sample.stale = false;
  sample.orientation_xyzw = {msg.quaternion.x, msg.quaternion.y, msg.quaternion.z,
                             msg.quaternion.w};
  sample.rpy_xyz = {msg.rpy.x, msg.rpy.y, msg.rpy.z};
  sample.angular_velocity_xyz = {msg.angular_velocity.x, msg.angular_velocity.y,
                                 msg.angular_velocity.z};
  sample.linear_acceleration_xyz = {msg.linear_acceleration.x,
                                    msg.linear_acceleration.y,
                                    msg.linear_acceleration.z};
  return sample;
}

[[nodiscard]] bridge::core::BmsStateSample make_bms_sample_from_ros2(
    const interface_protocol::msg::PowerInfo& msg) {
  bridge::core::BmsStateSample sample;
  sample.soc = clamp_percent_to_u8(msg.percentage);
  sample.bms_voltage[0] = voltage_to_millivolts(msg.voltage);
  sample.current = current_to_milliamps(msg.current);
  if (msg.error_code > 0) {
    sample.bms_state[0] = static_cast<std::uint32_t>(msg.error_code);
  }
  sample.reserve[0] = msg.enable ? 1U : 0U;
  return sample;
}

[[nodiscard]] bridge::core::FaultCodeSample make_fault_sample_from_ros2(
    const interface_protocol::msg::Alert& msg,
    const std::uint64_t sequence,
    const std::uint64_t sample_time_ns) {
  bridge::core::FaultCodeSample sample;
  sample.sequence = sequence;
  sample.sample_time_ns = sample_time_ns;
  sample.code = static_cast<std::uint32_t>(msg.type);
  sample.level = msg.type == interface_protocol::msg::Alert::ERROR ? 3U : 1U;
  return sample;
}
#endif

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

  ~Impl() {
#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
    if (ros_spin_thread_.joinable()) {
      if (ros_executor_) {
        ros_executor_->cancel();
      }
      ros_spin_thread_.join();
    }
#endif
  }

  [[nodiscard]] const Pm01AdapterConfig& config() const noexcept {
    return config_;
  }

  void set_runtime_event_sink(bridge::core::BridgeRuntimeEventSink* event_sink) noexcept {
    runtime_event_sink_.store(event_sink, std::memory_order_release);
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
                                already_initialized_detail(),
                                persistent_fault_);
    }

    lifecycle_state_ = bridge::core::LifecycleState::kInitializing;
    synthetic_time_ns_ += tick_or_default(config_.session.initialize_timeout_ns);

    if (const auto validation = validate_config_locked(); !validation.succeeded()) {
      lifecycle_state_ = bridge::core::LifecycleState::kFaulted;
      return validation;
    }

#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
    if (const auto init = initialize_ros2_backend_locked(); !init.succeeded()) {
      lifecycle_state_ = bridge::core::LifecycleState::kFaulted;
      return init;
    }
    initialized_ = true;
    connection_alive_ = true;
    has_read_state_ = false;
    lifecycle_state_ = bridge::core::LifecycleState::kWaitingForState;
    persistent_fault_.reset();
    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              "pm01 adapter initialized with ROS2 backend");
#else
    lifecycle_state_ = bridge::core::LifecycleState::kFaulted;
    return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                             bridge::core::FaultCategory::kConfiguration,
                             bridge::core::FaultSeverity::kFatal,
                             "pm01 ROS2 backend requested but not compiled in");
#endif
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

#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
    return read_state_from_ros2_locked();
#else
    result.result = make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                                      bridge::core::FaultCategory::kConfiguration,
                                      bridge::core::FaultSeverity::kFatal,
                                      "pm01 ROS2 backend requested but not compiled in");
    return result;
#endif
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
    synthetic_time_ns_ += tick_or_default(config_.session.write_timeout_ns);
    last_write_time_ns_ = synthetic_time_ns_;

#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
    return write_command_to_ros2_locked(command);
#else
    return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                             bridge::core::FaultCategory::kConfiguration,
                             bridge::core::FaultSeverity::kFatal,
                             "pm01 ROS2 backend requested but not compiled in");
#endif
  }

  [[nodiscard]] bridge::core::AdapterHealthResult query_health() const {
    std::lock_guard<std::mutex> lock(mutex_);

#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
    return query_ros2_health_locked();
#else
    bridge::core::AdapterHealthResult result;
    result.result = make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                                      bridge::core::FaultCategory::kConfiguration,
                                      bridge::core::FaultSeverity::kFatal,
                                      "pm01 ROS2 backend requested but not compiled in");
    return result;
#endif
  }

  [[nodiscard]] bridge::core::AdapterCallResult shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (lifecycle_state_ == bridge::core::LifecycleState::kStopped) {
      return make_status_result(bridge::core::AdapterCallStatus::kOk,
                                "adapter already stopped");
    }

    lifecycle_state_ = bridge::core::LifecycleState::kStopping;
    synthetic_time_ns_ += tick_or_default(config_.session.write_timeout_ns);

#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
    return shutdown_ros2_backend_locked();
#else
    connection_alive_ = false;
    initialized_ = false;
    has_read_state_ = false;
    persistent_fault_.reset();
    lifecycle_state_ = bridge::core::LifecycleState::kStopped;
    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              shutdown_detail());
#endif
  }

 private:
#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
  [[nodiscard]] bridge::core::AdapterCallResult initialize_ros2_backend_locked() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
      owns_ros_context_ = true;
    }

    rclcpp::NodeOptions options;
    ros_node_ = std::make_shared<rclcpp::Node>("pm01_adapter_bridge", options);

    rclcpp::QoS qos(3);
    qos.best_effort();
    qos.durability_volatile();

    joint_state_sub_ = ros_node_->create_subscription<interface_protocol::msg::JointState>(
        "/hardware/joint_state", qos,
        [this](const interface_protocol::msg::JointState::SharedPtr msg) {
          if (!has_expected_joint_layout(*msg)) {
            return;
          }
          {
            std::lock_guard<std::mutex> lock(ros_mutex_);
            latest_joint_state_ = msg;
          }
          if (auto* const event_sink =
                  runtime_event_sink_.load(std::memory_order_acquire)) {
            event_sink->notify_state_available();
          }
        });
    imu_sub_ = ros_node_->create_subscription<interface_protocol::msg::ImuInfo>(
        "/hardware/imu_info", qos,
        [this](const interface_protocol::msg::ImuInfo::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(ros_mutex_);
          latest_imu_ = msg;
        });
    power_info_sub_ =
        ros_node_->create_subscription<interface_protocol::msg::PowerInfo>(
            "/hardware/power_info", qos,
            [this](const interface_protocol::msg::PowerInfo::SharedPtr msg) {
              std::lock_guard<std::mutex> lock(ros_mutex_);
              latest_power_info_ = msg;
            });
    alert_sub_ = ros_node_->create_subscription<interface_protocol::msg::Alert>(
        "/hardware/alert", qos,
        [this](const interface_protocol::msg::Alert::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(ros_mutex_);
          pending_alerts_.push_back(*msg);
        });
    motion_state_sub_ = ros_node_->create_subscription<interface_protocol::msg::MotionState>(
        "/motion/motion_state", 1,
        [this](const interface_protocol::msg::MotionState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(ros_mutex_);
          latest_motion_state_ = msg;
        });
    joint_cmd_pub_ = ros_node_->create_publisher<interface_protocol::msg::JointCommand>(
        "/hardware/joint_command", qos);

    ros_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    ros_executor_->add_node(ros_node_);
    ros_spin_thread_ = std::thread([this]() {
      auto* const executor = ros_executor_.get();
      if (executor != nullptr) {
        executor->spin();
      }
    });

    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              "pm01 ROS2 backend initialized");
  }

  [[nodiscard]] bridge::core::AdapterReadResult read_state_from_ros2_locked() {
    bridge::core::AdapterReadResult result;
    interface_protocol::msg::JointState::SharedPtr joint_state;
    interface_protocol::msg::ImuInfo::SharedPtr imu;
    interface_protocol::msg::PowerInfo::SharedPtr power_info;
    std::vector<interface_protocol::msg::Alert> alerts;
    {
      std::lock_guard<std::mutex> ros_lock(ros_mutex_);
      joint_state = latest_joint_state_;
      latest_joint_state_.reset();
      imu = latest_imu_;
      power_info = latest_power_info_;
      alerts.swap(pending_alerts_);
    }

    if (!joint_state) {
      const char* const detail = has_read_state_
                                     ? "pm01 robot state not available"
                                     : "waiting for /hardware/joint_state";
      result.result = make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                                        bridge::core::FaultCategory::kAdapterRead,
                                        bridge::core::FaultSeverity::kRecoverable,
                                        detail);
      return result;
    }

    has_read_state_ = true;
    lifecycle_state_ = bridge::core::LifecycleState::kActive;

    bridge::core::StateFrame state;
    state.sequence = ++state_sequence_;
    state.sample_time_ns = synthetic_time_ns_;
    state.publish_time_ns = synthetic_time_ns_;
    state.mode_pr = ros_mode_pr_;
    state.mode_machine = ros_mode_machine_;
    state.lifecycle_state = lifecycle_state_;
    state.active_mode = ros_active_mode_;
    state.mode_label = ros_mode_label_;
    state.valid = true;
    state.stale = false;
    state.degraded = false;
    state.joints.position = joint_state->position;
    state.joints.velocity = joint_state->velocity;
    state.joints.effort = joint_state->torque;
    state.joints.acceleration.assign(config_.joint_count(), 0.0);

    if (imu) {
      bridge::core::ImuSample sample =
          make_imu_sample_from_ros2(*imu, synthetic_time_ns_);
      result.aux_state.imu = sample;
      state.imu = std::move(sample);
    } else if (const auto default_imu = build_imu_sample_locked(); default_imu.has_value()) {
      state.imu = default_imu;
    }

    if (power_info) {
      result.aux_state.bms = make_bms_sample_from_ros2(*power_info);
    }
    result.aux_state.faults.reserve(alerts.size());
    for (const interface_protocol::msg::Alert& alert : alerts) {
      result.aux_state.faults.push_back(
          make_fault_sample_from_ros2(alert, ++alert_sequence_, synthetic_time_ns_));
    }

    result.state = std::move(state);
    result.result = make_status_result(bridge::core::AdapterCallStatus::kOk,
                                       "pm01 state read via ROS2 backend");
    return result;
  }

  [[nodiscard]] bridge::core::AdapterCallResult write_command_to_ros2_locked(
      const bridge::core::CommandFrame& command) {
    const double position0 = command.joints.position.empty() ? 0.0 : command.joints.position.front();
    const double velocity0 = command.joints.velocity.empty() ? 0.0 : command.joints.velocity.front();
    std::fprintf(stderr,
                 "pm01 ros2 write begin: seq=%llu mode=%s pos0=%.6f vel0=%.6f\n",
                 static_cast<unsigned long long>(command.sequence),
                 command.mode_label.c_str(),
                 position0,
                 velocity0);

    interface_protocol::msg::MotionState::SharedPtr motion_state;
    {
      std::lock_guard<std::mutex> ros_lock(ros_mutex_);
      motion_state = latest_motion_state_;
    }

    if (config_.motion_state.required && !motion_state) {
      std::fprintf(stderr,
                   "pm01 blocked: /motion/motion_state missing while motion gate is enabled\n");
      return make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                               bridge::core::FaultCategory::kSafety,
                               bridge::core::FaultSeverity::kRecoverable,
                               "DDS->ROS2 command forwarding is blocked until /motion/motion_state is received");
    }
    if (config_.motion_state.required && motion_state &&
        motion_state->current_motion_task != config_.motion_state.required_task) {
      std::fprintf(stderr,
                   "pm01 blocked: current_motion_task=%s required_task=%s\n",
                   motion_state->current_motion_task.c_str(),
                   config_.motion_state.required_task.c_str());
      return make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                               bridge::core::FaultCategory::kSafety,
                               bridge::core::FaultSeverity::kRecoverable,
                               "PM01 command forwarding is blocked until " +
                                   config_.motion_state.required_task + " is active");
    }

    interface_protocol::msg::JointCommand ros_command;
    ros_command.position.resize(config_.joint_count(), 0.0);
    ros_command.velocity.resize(config_.joint_count(), 0.0);
    ros_command.feed_forward_torque.resize(config_.joint_count(), 0.0);
    ros_command.torque.resize(config_.joint_count(), 0.0);
    ros_command.stiffness.resize(config_.joint_count(), 0.0);
    ros_command.damping.resize(config_.joint_count(), 0.0);
    ros_command.parallel_parser_type = 0U;

    for (std::size_t index = 0; index < config_.joint_count(); ++index) {
      ros_command.position[index] = command.joints.position.empty() ? 0.0 : command.joints.position[index];
      ros_command.velocity[index] = command.joints.velocity.empty() ? 0.0 : command.joints.velocity[index];
      ros_command.feed_forward_torque[index] = 0.0;
      ros_command.torque[index] = 0.0;
      ros_command.stiffness[index] = command.joints.stiffness.empty() ? 0.0 : command.joints.stiffness[index];
      ros_command.damping[index] = command.joints.damping.empty() ? 0.0 : command.joints.damping[index];
    }

    std::fprintf(stderr,
                 "pm01 ros2 command publish: joints=%zu pos0=%.6f vel0=%.6f\n",
                 ros_command.position.size(),
                 ros_command.position.empty() ? 0.0 : ros_command.position.front(),
                 ros_command.velocity.empty() ? 0.0 : ros_command.velocity.front());
    joint_cmd_pub_->publish(ros_command);
    ros_mode_pr_ = command.mode_pr;
    ros_mode_machine_ =
        command.mode_machine == 0U ? encode_control_mode(command.mode) : command.mode_machine;
    ros_active_mode_ = command.mode;
    ros_mode_label_ = command.mode_label.empty() ? control_mode_label(command.mode)
                                                 : command.mode_label;
    return make_status_result(command.degraded ? bridge::core::AdapterCallStatus::kDegraded
                                               : bridge::core::AdapterCallStatus::kOk,
                              "pm01 command forwarded via ROS2 backend");
  }

  [[nodiscard]] bridge::core::AdapterHealthResult query_ros2_health_locked() const {
    bridge::core::AdapterHealthResult result;
    result.result = make_status_result(current_operational_status(),
                                       connection_alive_ ? "pm01 ROS2 backend healthy"
                                                         : "pm01 ROS2 backend unavailable",
                                       persistent_fault_);
    result.health.state = connection_alive_ ? bridge::core::HealthState::kHealthy
                                            : bridge::core::HealthState::kUnknown;
    result.health.connection_alive = connection_alive_;
    result.health.degraded = false;
    result.health.last_read_time_ns = last_read_time_ns_;
    result.health.last_write_time_ns = last_write_time_ns_;
    result.health.fault = persistent_fault_;
    return result;
  }

  [[nodiscard]] bridge::core::AdapterCallResult shutdown_ros2_backend_locked() {
    connection_alive_ = false;
    initialized_ = false;
    has_read_state_ = false;
    if (ros_executor_) {
      ros_executor_->cancel();
    }
    if (ros_spin_thread_.joinable()) {
      ros_spin_thread_.join();
    }
    joint_cmd_pub_.reset();
    joint_state_sub_.reset();
    imu_sub_.reset();
    power_info_sub_.reset();
    alert_sub_.reset();
    motion_state_sub_.reset();
    latest_joint_state_.reset();
    latest_imu_.reset();
    latest_power_info_.reset();
    pending_alerts_.clear();
    latest_motion_state_.reset();
    ros_executor_.reset();
    ros_node_.reset();
    if (owns_ros_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
      owns_ros_context_ = false;
    }
    lifecycle_state_ = bridge::core::LifecycleState::kStopped;
    persistent_fault_.reset();
    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              "pm01 ROS2 backend shutdown complete");
  }

  [[nodiscard]] static bool has_expected_joint_layout(
      const interface_protocol::msg::JointState& msg) {
    return msg.position.size() == kPm01JointCount &&
           msg.velocity.size() == kPm01JointCount &&
           msg.torque.size() == kPm01JointCount;
  }
#endif

  [[nodiscard]] bridge::core::AdapterCallResult validate_config_locked() {
    if (config_.session.robot_id.empty()) {
      return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                               bridge::core::FaultCategory::kConfiguration,
                               bridge::core::FaultSeverity::kFatal,
                               "session.robot_id must not be empty");
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
    imu.rpy_xyz = {0.0, 0.0, 0.0};
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

  [[nodiscard]] std::string already_initialized_detail() const {
    return "pm01 adapter already initialized";
  }

  [[nodiscard]] std::string shutdown_detail() const {
    return "adapter shutdown complete";
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
  std::atomic<bridge::core::BridgeRuntimeEventSink*> runtime_event_sink_{nullptr};
  bridge::core::LifecycleState lifecycle_state_{bridge::core::LifecycleState::kCreated};
#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
  bool owns_ros_context_{false};
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor_;
  std::thread ros_spin_thread_;
  mutable std::mutex ros_mutex_;
  rclcpp::Subscription<interface_protocol::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<interface_protocol::msg::ImuInfo>::SharedPtr imu_sub_;
  rclcpp::Subscription<interface_protocol::msg::PowerInfo>::SharedPtr power_info_sub_;
  rclcpp::Subscription<interface_protocol::msg::Alert>::SharedPtr alert_sub_;
  rclcpp::Subscription<interface_protocol::msg::MotionState>::SharedPtr motion_state_sub_;
  rclcpp::Publisher<interface_protocol::msg::JointCommand>::SharedPtr joint_cmd_pub_;
  interface_protocol::msg::JointState::SharedPtr latest_joint_state_;
  interface_protocol::msg::ImuInfo::SharedPtr latest_imu_;
  interface_protocol::msg::PowerInfo::SharedPtr latest_power_info_;
  std::vector<interface_protocol::msg::Alert> pending_alerts_;
  interface_protocol::msg::MotionState::SharedPtr latest_motion_state_;
  bridge::core::ControlMode ros_active_mode_{bridge::core::ControlMode::kSafeIdle};
  std::string ros_mode_label_{control_mode_label(bridge::core::ControlMode::kSafeIdle)};
  std::uint8_t ros_mode_pr_{0U};
  std::uint8_t ros_mode_machine_{0U};
  std::uint64_t alert_sequence_{0};
#endif
};

Pm01Adapter::Pm01Adapter(Pm01AdapterConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

Pm01Adapter::~Pm01Adapter() {
  if (impl_) {
    const auto shutdown_result = impl_->shutdown();
    static_cast<void>(shutdown_result.status);
  }
}

Pm01Adapter::Pm01Adapter(Pm01Adapter&&) noexcept = default;

Pm01Adapter& Pm01Adapter::operator=(Pm01Adapter&&) noexcept = default;

const Pm01AdapterConfig& Pm01Adapter::config() const noexcept {
  return impl_->config();
}

bridge::core::AdapterCallResult Pm01Adapter::initialize() {
  return impl_->initialize();
}

void Pm01Adapter::set_runtime_event_sink(
    bridge::core::BridgeRuntimeEventSink* event_sink) {
  impl_->set_runtime_event_sink(event_sink);
}

bridge::core::AdapterReadResult Pm01Adapter::read_state() {
  return impl_->read_state();
}

bridge::core::AdapterCallResult Pm01Adapter::write_command(
    const bridge::core::CommandFrame& command) {
  return impl_->write_command(command);
}

bridge::core::AdapterHealthResult Pm01Adapter::query_health() const {
  return impl_->query_health();
}

bridge::core::AdapterCallResult Pm01Adapter::shutdown() {
  return impl_->shutdown();
}

}  // namespace bridge::adapters::pm01
