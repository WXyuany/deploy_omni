#include "../../../../include/bridge/adapters/pm01/pm01_adapter.hpp"
#include "bridge/build_config.hpp"

#include <algorithm>
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
#include "interface_protocol/msg/joint_command.hpp"
#include "interface_protocol/msg/joint_state.hpp"
#include "interface_protocol/msg/motion_state.hpp"
#endif

namespace bridge::adapters::pm01 {
namespace {

constexpr std::uint64_t kFallbackTickNs = 1'000'000;
constexpr char kRos2TransportPrefix[] = "ros2";

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

[[nodiscard]] bool wants_ros2_backend(const Pm01AdapterConfig& config) noexcept {
  return config.backend == kRos2TransportPrefix ||
         config.session.transport_endpoint.rfind(kRos2TransportPrefix, 0) == 0;
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

    if (wants_real_backend()) {
#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
      if (const auto init = initialize_ros2_backend_locked(); !init.succeeded()) {
        lifecycle_state_ = bridge::core::LifecycleState::kFaulted;
        return init;
      }
      initialized_ = true;
      connection_alive_ = true;
      has_read_state_ = false;
      lifecycle_state_ = bridge::core::LifecycleState::kWaitingForState;
      deterministic_state_available_ = true;
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

    vendor_session_.initialize(config_.motion_state.initial_task);
    initialized_ = true;
    connection_alive_ = true;
    has_read_state_ = false;
    lifecycle_state_ = bridge::core::LifecycleState::kWaitingForState;
    deterministic_state_available_ = true;
    persistent_fault_.reset();

    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              initialized_detail());
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

    if (wants_real_backend()) {
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

    if (!deterministic_state_available_) {
      connection_alive_ = false;
      result.result = make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                                        bridge::core::FaultCategory::kAdapterRead,
                                        bridge::core::FaultSeverity::kRecoverable,
                                        "pm01 robot state not available");
      return result;
    }

    deterministic_state_available_ = false;
    connection_alive_ = true;

    if (!has_read_state_) {
      has_read_state_ = true;
      lifecycle_state_ = bridge::core::LifecycleState::kActive;
    }

     bridge::core::StateFrame state;
     state.sequence = ++state_sequence_;
     state.sample_time_ns = synthetic_time_ns_;
     state.publish_time_ns = synthetic_time_ns_;
     state.mode_pr = vendor_session_.mode_pr();
     state.mode_machine = vendor_session_.mode_machine();
     state.lifecycle_state = lifecycle_state_;
     state.active_mode = vendor_session_.active_mode();
    state.mode_label = vendor_session_.mode_label();
    state.valid = true;
    state.stale = false;
    state.degraded = false;
    state.command_stale = false;
    ////// 读取机器人：这里当前还是 deterministic session 占位
    ////// 读取机器人：未来应该从 PM01 真实接口读取状态，再按 dds_joint_indices 投影到共享 state.joints
    state.joints.position = vendor_session_.project_position(config_.dds_joint_indices);
    state.joints.velocity = vendor_session_.project_velocity(config_.dds_joint_indices);
    state.joints.effort = vendor_session_.project_effort(config_.dds_joint_indices);
    state.joints.acceleration =
        vendor_session_.project_acceleration(config_.dds_joint_indices);

    if (const auto imu = build_imu_sample_locked(); imu.has_value()) {
      state.imu = imu;
    }

    result.state = std::move(state);
    result.result = make_status_result(bridge::core::AdapterCallStatus::kOk,
                                       read_detail());
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
    if (!wants_real_backend() && is_motion_mode(command.mode) && config_.motion_state.required) {
      if (!vendor_session_.motion_task().has_value()) {
        return make_error_result(bridge::core::AdapterCallStatus::kRetryableError,
                                 bridge::core::FaultCategory::kSafety,
                                 bridge::core::FaultSeverity::kRecoverable,
                                 "motion state is required before PM01 command write");
      }
      if (*vendor_session_.motion_task() != config_.motion_state.required_task) {
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

    if (wants_real_backend()) {
#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
      return write_command_to_ros2_locked(command); //正常这里进去
#else
      return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                               bridge::core::FaultCategory::kConfiguration,
                               bridge::core::FaultSeverity::kFatal,
                               "pm01 ROS2 backend requested but not compiled in");
#endif
    }

    ////// 写入机器人：这里当前还是 deterministic session 占位
    ////// 写入机器人：未来应该按 dds_joint_indices 把共享 command 映射成 PM01 真实电机命令并调用底层接口下发
    vendor_session_.apply_command(command, config_.dds_joint_indices);

    const bridge::core::AdapterCallStatus status =
        command.degraded ? bridge::core::AdapterCallStatus::kDegraded
                         : bridge::core::AdapterCallStatus::kOk;
    return make_status_result(status, write_detail(command));
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

    if (wants_real_backend()) {
#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
      return write_safe_command_to_ros2_locked();
#else
      return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                               bridge::core::FaultCategory::kConfiguration,
                               bridge::core::FaultSeverity::kFatal,
                               "pm01 ROS2 backend requested but not compiled in");
#endif
    }

    ////// 写入机器人安全指令：这里当前还是 deterministic session 占位
    ////// 写入机器人安全指令：未来应该调用 PM01 真实接口发送 safe command
    vendor_session_.apply_safe_command();

    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              safe_command_detail());
  }

  [[nodiscard]] bridge::core::AdapterHealthResult query_health() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (wants_real_backend()) {
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

    bridge::core::AdapterHealthResult result;
    result.result = make_status_result(current_operational_status(),
                                       health_detail_locked(),
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

    if (wants_real_backend()) {
#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
      return shutdown_ros2_backend_locked();
#else
      lifecycle_state_ = bridge::core::LifecycleState::kFaulted;
      return make_error_result(bridge::core::AdapterCallStatus::kFatalError,
                               bridge::core::FaultCategory::kConfiguration,
                               bridge::core::FaultSeverity::kFatal,
                               "pm01 ROS2 backend requested but not compiled in");
#endif
    }

    connection_alive_ = false;
    initialized_ = false;
    has_read_state_ = false;
    vendor_session_.reset();
    deterministic_state_available_ = false;
    persistent_fault_.reset();
    lifecycle_state_ = bridge::core::LifecycleState::kStopped;

    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              shutdown_detail());
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
          std::lock_guard<std::mutex> lock(ros_mutex_);
          latest_joint_state_ = msg;
        });
    imu_sub_ = ros_node_->create_subscription<interface_protocol::msg::ImuInfo>(
        "/hardware/imu_info", qos,
        [this](const interface_protocol::msg::ImuInfo::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(ros_mutex_);
          latest_imu_ = msg;
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
    {
      std::lock_guard<std::mutex> ros_lock(ros_mutex_);
      joint_state = latest_joint_state_;
      latest_joint_state_.reset();
      imu = latest_imu_;
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
      bridge::core::ImuSample sample;
      sample.sample_time_ns = synthetic_time_ns_;
       sample.valid = true;
       sample.stale = false;
       sample.orientation_xyzw = {imu->quaternion.x, imu->quaternion.y,
                                  imu->quaternion.z, imu->quaternion.w};
       sample.rpy_xyz = {imu->rpy.x, imu->rpy.y, imu->rpy.z};
       sample.angular_velocity_xyz = {imu->angular_velocity.x, imu->angular_velocity.y,
                                      imu->angular_velocity.z};
      sample.linear_acceleration_xyz = {imu->linear_acceleration.x,
                                        imu->linear_acceleration.y,
                                        imu->linear_acceleration.z};
      state.imu = std::move(sample);
    } else if (const auto default_imu = build_imu_sample_locked(); default_imu.has_value()) {
      state.imu = default_imu;
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

  [[nodiscard]] bridge::core::AdapterCallResult write_safe_command_to_ros2_locked() {
    interface_protocol::msg::JointCommand ros_command;
    ros_command.position.resize(config_.joint_count(), 0.0);
    ros_command.velocity.resize(config_.joint_count(), 0.0);
    ros_command.feed_forward_torque.resize(config_.joint_count(), 0.0);
    ros_command.torque.resize(config_.joint_count(), 0.0);
    ros_command.stiffness.resize(config_.joint_count(), 0.0);
    ros_command.damping.resize(config_.joint_count(), 0.0);
    ros_command.parallel_parser_type = 0U;
    joint_cmd_pub_->publish(ros_command);
    ros_active_mode_ = bridge::core::ControlMode::kSafeIdle;
    ros_mode_label_ = control_mode_label(bridge::core::ControlMode::kSafeIdle);
    return make_status_result(bridge::core::AdapterCallStatus::kOk,
                              "pm01 safe command forwarded via ROS2 backend");
  }

  [[nodiscard]] bridge::core::AdapterHealthResult query_ros2_health_locked() const {
    bridge::core::AdapterHealthResult result;
    result.result = make_status_result(current_operational_status(),
                                       connection_alive_ ? "pm01 ROS2 backend healthy"
                                                         : "pm01 ROS2 backend unavailable",
                                       persistent_fault_);
    result.health.state = connection_alive_ ? bridge::core::HealthState::kHealthy
                                            : bridge::core::HealthState::kUnknown;
    result.health.safe_command_supported = config_.safety.safe_command_supported;
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
    motion_state_sub_.reset();
    latest_joint_state_.reset();
    latest_imu_.reset();
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

  class DeterministicVendorSession {
   public:
    explicit DeterministicVendorSession(const std::size_t dds_motor_count =
                                            kPm01DdsMotorCount)
        : position_(dds_motor_count, 0.0),
          velocity_(dds_motor_count, 0.0),
          effort_(dds_motor_count, 0.0),
          stiffness_(dds_motor_count, 0.0),
          damping_(dds_motor_count, 0.0),
          acceleration_(dds_motor_count, 0.0) {}

    void initialize(std::optional<std::string> initial_motion_task) {
      assign_zeroed(position_);
      assign_zeroed(velocity_);
      assign_zeroed(effort_);
      assign_zeroed(stiffness_);
      assign_zeroed(damping_);
      assign_zeroed(acceleration_);
      current_motion_task_ = std::move(initial_motion_task);
      mode_pr_ = 0U;
      mode_machine_ = 0U;
      set_active_mode(bridge::core::ControlMode::kSafeIdle,
                      control_mode_label(bridge::core::ControlMode::kSafeIdle));
    }

    void apply_command(const bridge::core::CommandFrame& command,
                       const std::vector<std::int64_t>& indices) {
      static_cast<void>(command);
      static_cast<void>(indices);
    }

    void apply_safe_command() {}

    void reset() {
      position_.clear();
      velocity_.clear();
      effort_.clear();
      stiffness_.clear();
      damping_.clear();
      acceleration_.clear();
      current_motion_task_.reset();
      mode_ = bridge::core::ControlMode::kUnknown;
      mode_label_ = control_mode_label(mode_);
    }

    [[nodiscard]] std::vector<double> project_position(
        const std::vector<std::int64_t>& indices) const {
      return project_from_slots(position_, indices);
    }

    [[nodiscard]] std::vector<double> project_velocity(
        const std::vector<std::int64_t>& indices) const {
      return project_from_slots(velocity_, indices);
    }

    [[nodiscard]] std::vector<double> project_effort(
        const std::vector<std::int64_t>& indices) const {
      return project_from_slots(effort_, indices);
    }

    [[nodiscard]] std::vector<double> project_acceleration(
        const std::vector<std::int64_t>& indices) const {
      return project_from_slots(acceleration_, indices);
    }

    [[nodiscard]] bridge::core::ControlMode active_mode() const noexcept {
      return mode_;
    }

    [[nodiscard]] const std::string& mode_label() const noexcept {
      return mode_label_;
    }

    [[nodiscard]] const std::optional<std::string>& motion_task() const noexcept {
      return current_motion_task_;
    }

    [[nodiscard]] std::uint8_t mode_pr() const noexcept {
      return mode_pr_;
    }

    [[nodiscard]] std::uint8_t mode_machine() const noexcept {
      return mode_machine_;
    }

   private:
    static void assign_zeroed(std::vector<double>& slots) {
      slots.assign(slots.size(), 0.0);
    }

    static void assign_to_slots(std::vector<double>& slots,
                                const std::vector<double>& values,
                                const std::vector<std::int64_t>& indices) {
      if (values.empty()) {
        return;
      }

      for (std::size_t joint = 0; joint < values.size(); ++joint) {
        slots[static_cast<std::size_t>(indices[joint])] = values[joint];
      }
    }

    [[nodiscard]] static std::vector<double> project_from_slots(
        const std::vector<double>& slots,
        const std::vector<std::int64_t>& indices) {
      std::vector<double> values(indices.size(), 0.0);
      for (std::size_t joint = 0; joint < indices.size(); ++joint) {
        values[joint] = slots[static_cast<std::size_t>(indices[joint])];
      }
      return values;
    }

    void set_active_mode(const bridge::core::ControlMode mode, std::string label) {
      mode_ = mode;
      mode_label_ = std::move(label);
    }

    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> effort_;
    std::vector<double> stiffness_;
    std::vector<double> damping_;
    std::vector<double> acceleration_;
    std::optional<std::string> current_motion_task_;
    std::uint8_t mode_pr_{0U};
    std::uint8_t mode_machine_{0U};
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

  [[nodiscard]] bool wants_real_backend() const noexcept {
    return wants_ros2_backend(config_);
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

  [[nodiscard]] std::string initialized_detail() const {
    return config_.motion_state.initial_task.has_value()
               ? "pm01 adapter initialized with configured motion task seam"
               : "pm01 adapter initialized";
  }

  [[nodiscard]] std::string read_detail() const {
    return config_.publish_default_imu_when_missing
               ? "pm01 state read complete with default IMU seam"
               : "pm01 state read complete";
  }

  [[nodiscard]] std::string write_detail(
      const bridge::core::CommandFrame& command) const {
    return "pm01 command write complete in mode " +
           (command.mode_label.empty() ? control_mode_label(command.mode)
                                       : command.mode_label);
  }

  [[nodiscard]] std::string safe_command_detail() const {
    return "pm01 safe command applied";
  }

  [[nodiscard]] std::string health_detail_locked() const {
    if (lifecycle_state_ == bridge::core::LifecycleState::kStopped) {
      return "adapter stopped";
    }
    if (!initialized_) {
      return "adapter not initialized";
    }
    return "pm01 adapter healthy";
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
  bool deterministic_state_available_{false};
  bridge::core::LifecycleState lifecycle_state_{bridge::core::LifecycleState::kCreated};
  DeterministicVendorSession vendor_session_;
#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
  bool owns_ros_context_{false};
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor_;
  std::thread ros_spin_thread_;
  mutable std::mutex ros_mutex_;
  rclcpp::Subscription<interface_protocol::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<interface_protocol::msg::ImuInfo>::SharedPtr imu_sub_;
  rclcpp::Subscription<interface_protocol::msg::MotionState>::SharedPtr motion_state_sub_;
  rclcpp::Publisher<interface_protocol::msg::JointCommand>::SharedPtr joint_cmd_pub_;
  interface_protocol::msg::JointState::SharedPtr latest_joint_state_;
  interface_protocol::msg::ImuInfo::SharedPtr latest_imu_;
  interface_protocol::msg::MotionState::SharedPtr latest_motion_state_;
  bridge::core::ControlMode ros_active_mode_{bridge::core::ControlMode::kSafeIdle};
  std::string ros_mode_label_{control_mode_label(bridge::core::ControlMode::kSafeIdle)};
  std::uint8_t ros_mode_pr_{0U};
  std::uint8_t ros_mode_machine_{0U};
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
