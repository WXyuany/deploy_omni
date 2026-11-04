#include "../../../../include/bridge/adapters/agibot_x2/agibot_x2_adapter.hpp"

#include <chrono>
#include <array>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#if defined(BRIDGE_AGIBOT_X2_ROS2_AVAILABLE) && BRIDGE_AGIBOT_X2_ROS2_AVAILABLE
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <aimdk_msgs/msg/joint_command_array.hpp>
#include <aimdk_msgs/msg/joint_state_array.hpp>
#endif
#include <utility>

namespace bridge::adapters::agibot_x2 {
namespace {

using bridge::core::AdapterCallResult;
using bridge::core::AdapterCallStatus;
using bridge::core::AdapterHealthResult;
using bridge::core::AdapterReadResult;
using bridge::core::FaultCategory;
using bridge::core::FaultSeverity;
using bridge::core::HealthState;
using bridge::core::LifecycleState;

[[nodiscard]] std::uint64_t now_ns() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

[[nodiscard]] AdapterCallResult unavailable(const std::string& detail) {
  return {.status = AdapterCallStatus::kFatalError,
          .detail = detail,
          .fault = bridge::core::BridgeFault{
              .category = FaultCategory::kConfiguration,
              .severity = FaultSeverity::kFatal,
              .lifecycle_state = LifecycleState::kFaulted,
              .message = detail,
              .latched = true,
              .timestamp_ns = now_ns()}};
}

}  // namespace

class AgibotX2Adapter::Impl {
 public:
  explicit Impl(AgibotX2AdapterConfig config) : config_(std::move(config)) {}

#if defined(BRIDGE_AGIBOT_X2_ROS2_AVAILABLE) && BRIDGE_AGIBOT_X2_ROS2_AVAILABLE
  struct Group {
    std::string name;
    std::vector<std::string> joint_names;
    rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr command_publisher;
    rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr state_subscription;
    std::optional<aimdk_msgs::msg::JointStateArray> latest_state;
  };
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::array<Group, 4> groups_;
#endif

  AgibotX2AdapterConfig config_;
  mutable std::mutex mutex_;
  LifecycleState lifecycle_{LifecycleState::kCreated};
  bool initialized_{false};
  bool has_state_{false};
  std::uint64_t sequence_{0};
  std::uint64_t last_read_ns_{0};
  std::uint64_t last_write_ns_{0};
};

AgibotX2Adapter::AgibotX2Adapter(AgibotX2AdapterConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}
AgibotX2Adapter::~AgibotX2Adapter() = default;
AgibotX2Adapter::AgibotX2Adapter(AgibotX2Adapter&&) noexcept = default;
AgibotX2Adapter& AgibotX2Adapter::operator=(AgibotX2Adapter&&) noexcept = default;

bridge::core::RobotCapabilitySet AgibotX2Adapter::capabilities() const noexcept {
  return {.bits = static_cast<std::uint32_t>(bridge::core::RobotCapability::kJointState) |
                  static_cast<std::uint32_t>(bridge::core::RobotCapability::kJointPositionCommand) |
                  static_cast<std::uint32_t>(bridge::core::RobotCapability::kJointVelocityCommand) |
                  static_cast<std::uint32_t>(bridge::core::RobotCapability::kJointTorqueCommand)};
}

const AgibotX2AdapterConfig& AgibotX2Adapter::config() const noexcept { return impl_->config_; }

AdapterCallResult AgibotX2Adapter::initialize() {
  std::lock_guard<std::mutex> lock(impl_->mutex_);
  if (impl_->config_.joint_names.size() != kAgibotX2JointCount) {
    return unavailable("agibot_x2 requires exactly 31 joint names");
  }
#if defined(BRIDGE_AGIBOT_X2_ROS2_AVAILABLE) && BRIDGE_AGIBOT_X2_ROS2_AVAILABLE
  if (!rclcpp::ok()) {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  impl_->node_ = std::make_shared<rclcpp::Node>("agibot_x2_adapter");
  const std::array<std::string, 4> names{"head", "waist", "arm", "leg"};
  const std::array<std::size_t, 4> offsets{0U, 2U, 5U, 19U};
  const std::array<std::size_t, 4> widths{2U, 3U, 14U, 12U};
  for (std::size_t i = 0; i < names.size(); ++i) {
    auto& group = impl_->groups_[i];
    group.name = names[i];
    group.joint_names.assign(impl_->config_.joint_names.begin() + offsets[i],
                             impl_->config_.joint_names.begin() + offsets[i] + widths[i]);
    const auto command_topic = impl_->config_.topics.command_prefix + "/" + group.name + "/command";
    const auto state_topic = impl_->config_.topics.state_prefix + "/" + group.name + "/state";
    auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
    group.command_publisher = impl_->node_->create_publisher<aimdk_msgs::msg::JointCommandArray>(command_topic, command_qos);
    group.state_subscription = impl_->node_->create_subscription<aimdk_msgs::msg::JointStateArray>(
        state_topic, state_qos,
        [this, i](const aimdk_msgs::msg::JointStateArray::SharedPtr message) {
          std::lock_guard<std::mutex> callback_lock(impl_->mutex_);
          impl_->groups_[i].latest_state = *message;
        });
  }
  impl_->executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  impl_->executor_->add_node(impl_->node_);
  impl_->spin_thread_ = std::thread([this] { impl_->executor_->spin(); });
  impl_->initialized_ = true;
  impl_->lifecycle_ = LifecycleState::kWaitingForState;
  return {.status = AdapterCallStatus::kOk,
          .detail = "Agibot X2 aimdk_msgs ROS2 backend initialized",
          .fault = std::nullopt};
#else
  impl_->lifecycle_ = LifecycleState::kFaulted;
  return unavailable("Agibot X2 aimdk_msgs ROS2 backend is not compiled; install rclcpp and the aimdk_msgs package");
#endif
}

AdapterReadResult AgibotX2Adapter::read_state() {
  std::lock_guard<std::mutex> lock(impl_->mutex_);
  AdapterReadResult result;
#if defined(BRIDGE_AGIBOT_X2_ROS2_AVAILABLE) && BRIDGE_AGIBOT_X2_ROS2_AVAILABLE
  if (!impl_->initialized_) {
    result.result = unavailable("Agibot X2 adapter is not initialized");
    return result;
  }
  bridge::core::StateFrame state;
  state.sequence = ++impl_->sequence_;
  state.sample_time_ns = now_ns();
  state.publish_time_ns = state.sample_time_ns;
  state.joints.position.reserve(kAgibotX2JointCount);
  state.joints.velocity.reserve(kAgibotX2JointCount);
  state.joints.effort.reserve(kAgibotX2JointCount);
  for (const auto& group : impl_->groups_) {
    if (!group.latest_state.has_value() || group.latest_state->joints.size() != group.joint_names.size()) {
      result.result = {.status = AdapterCallStatus::kRetryableError,
                       .detail = "waiting for all Agibot X2 joint state groups",
                       .fault = std::nullopt};
      return result;
    }
    for (std::size_t i = 0; i < group.latest_state->joints.size(); ++i) {
      const auto& joint = group.latest_state->joints[i];
      if (joint.name != group.joint_names[i]) {
        result.result = {.status = AdapterCallStatus::kRetryableError,
                         .detail = "Agibot X2 joint state name/order mismatch in " + group.name,
                         .fault = std::nullopt};
        return result;
      }
      state.joints.position.push_back(joint.position);
      state.joints.velocity.push_back(joint.velocity);
      state.joints.effort.push_back(joint.effort);
    }
  }
  state.valid = true;
  state.stale = false;
  state.lifecycle_state = LifecycleState::kActive;
  impl_->has_state_ = true;
  impl_->last_read_ns_ = state.sample_time_ns;
  impl_->lifecycle_ = LifecycleState::kActive;
  result.state = std::move(state);
  result.result = {.status = AdapterCallStatus::kOk,
                   .detail = "Agibot X2 joint state read",
                   .fault = std::nullopt};
  return result;
#else
  result.result = unavailable("Agibot X2 aimdk_msgs ROS2 backend is not compiled");
  return result;
#endif
}

AdapterCallResult AgibotX2Adapter::write_command(const bridge::core::CommandFrame& command) {
  std::lock_guard<std::mutex> lock(impl_->mutex_);
#if defined(BRIDGE_AGIBOT_X2_ROS2_AVAILABLE) && BRIDGE_AGIBOT_X2_ROS2_AVAILABLE
  if (!impl_->initialized_) return unavailable("Agibot X2 adapter is not initialized");
  if (!command.valid || command.stale) {
    return {.status = AdapterCallStatus::kRetryableError,
            .detail = "invalid or stale Agibot X2 command",
            .fault = std::nullopt};
  }
  const auto width_is_valid = [](const std::vector<double>& values) {
    return values.empty() || values.size() == kAgibotX2JointCount;
  };
  if (!width_is_valid(command.joints.position) ||
      !width_is_valid(command.joints.velocity) ||
      !width_is_valid(command.joints.effort) ||
      !width_is_valid(command.joints.stiffness) ||
      !width_is_valid(command.joints.damping)) {
    return {.status = AdapterCallStatus::kRetryableError,
            .detail = "Agibot X2 command vectors must be empty or width 31",
            .fault = std::nullopt};
  }
  if (impl_->config_.safety.require_initial_state_before_motion && !impl_->has_state_ &&
      command.mode != bridge::core::ControlMode::kSafeIdle) {
    return {.status = AdapterCallStatus::kRetryableError,
            .detail = "initial Agibot X2 state is required before motion",
            .fault = std::nullopt};
  }
  const std::array<std::size_t, 4> offsets{0U, 2U, 5U, 19U};
  for (std::size_t i = 0; i < impl_->groups_.size(); ++i) {
    auto& group = impl_->groups_[i];
    aimdk_msgs::msg::JointCommandArray message;
    message.joints.resize(group.joint_names.size());
    for (std::size_t j = 0; j < group.joint_names.size(); ++j) {
      auto& joint = message.joints[j];
      joint.name = group.joint_names[j];
      const std::size_t index = offsets[i] + j;
      joint.position = command.joints.position.empty() ? 0.0 : command.joints.position[index];
      joint.velocity = command.joints.velocity.empty() ? 0.0 : command.joints.velocity[index];
      joint.effort = command.joints.effort.empty() ? 0.0 : command.joints.effort[index];
      joint.stiffness = command.joints.stiffness.empty() ? 0.0 : command.joints.stiffness[index];
      joint.damping = command.joints.damping.empty() ? 0.0 : command.joints.damping[index];
    }
    group.command_publisher->publish(message);
  }
  impl_->last_write_ns_ = now_ns();
  return {.status = command.degraded ? AdapterCallStatus::kDegraded : AdapterCallStatus::kOk,
          .detail = "Agibot X2 joint command published",
          .fault = std::nullopt};
#else
  return unavailable("Agibot X2 command forwarding requires the aimdk_msgs ROS2 backend");
#endif
}

AdapterHealthResult AgibotX2Adapter::query_health() const {
  std::lock_guard<std::mutex> lock(impl_->mutex_);
  AdapterHealthResult result;
#if defined(BRIDGE_AGIBOT_X2_ROS2_AVAILABLE) && BRIDGE_AGIBOT_X2_ROS2_AVAILABLE
  result.result = {.status = impl_->initialized_ ? AdapterCallStatus::kOk
                                                 : AdapterCallStatus::kRetryableError,
                   .detail = impl_->initialized_ ? "Agibot X2 ROS2 backend initialized"
                                                 : "Agibot X2 ROS2 backend unavailable",
                   .fault = std::nullopt};
  result.health.state = impl_->initialized_ ? HealthState::kHealthy : HealthState::kUnknown;
  result.health.connection_alive = impl_->initialized_;
#else
  result.result = unavailable("Agibot X2 aimdk_msgs ROS2 backend is not compiled");
  result.health.state = HealthState::kFaulted;
  result.health.connection_alive = false;
#endif
  result.health.last_read_time_ns = impl_->last_read_ns_;
  result.health.last_write_time_ns = impl_->last_write_ns_;
  return result;
}

AdapterCallResult AgibotX2Adapter::shutdown() {
  std::unique_lock<std::mutex> lock(impl_->mutex_);
#if defined(BRIDGE_AGIBOT_X2_ROS2_AVAILABLE) && BRIDGE_AGIBOT_X2_ROS2_AVAILABLE
  if (impl_->initialized_ && impl_->config_.safety.require_zero_command_on_shutdown) {
    for (auto& group : impl_->groups_) {
      aimdk_msgs::msg::JointCommandArray message;
      message.joints.resize(group.joint_names.size());
      for (std::size_t i = 0; i < group.joint_names.size(); ++i) {
        message.joints[i].name = group.joint_names[i];
      }
      group.command_publisher->publish(message);
    }
  }
  if (impl_->executor_) impl_->executor_->cancel();
  lock.unlock();
  if (impl_->spin_thread_.joinable()) impl_->spin_thread_.join();
  lock.lock();
  impl_->executor_.reset();
  impl_->node_.reset();
  impl_->initialized_ = false;
#endif
  impl_->initialized_ = false;
  impl_->lifecycle_ = LifecycleState::kStopped;
  return {.status = AdapterCallStatus::kOk,
          .detail = "agibot_x2 adapter stopped",
          .fault = std::nullopt};
}

}  // namespace bridge::adapters::agibot_x2
