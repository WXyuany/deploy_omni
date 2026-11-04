#include <cassert>
#include <string_view>
#include <type_traits>
#include <utility>

#include "../../include/bridge/core/robot_sdk_adapter.hpp"

namespace {

using bridge::core::AdapterCallResult;
using bridge::core::AdapterCallStatus;
using bridge::core::AdapterHealth;
using bridge::core::AdapterHealthResult;
using bridge::core::AdapterReadResult;
using bridge::core::BridgeFault;
using bridge::core::CommandFrame;
using bridge::core::ControlMode;
using bridge::core::FaultCategory;
using bridge::core::FaultSeverity;
using bridge::core::HealthState;
using bridge::core::ImuSample;
using bridge::core::JointStateBlock;
using bridge::core::LifecycleState;
using bridge::core::RobotSdkAdapter;
using bridge::core::RobotCapability;
using bridge::core::StateFrame;

template <typename Adapter, typename = void>
struct HasWriteSafeCommand : std::false_type {};

template <typename Adapter>
struct HasWriteSafeCommand<
    Adapter,
    std::void_t<decltype(std::declval<Adapter&>().write_safe_command())>>
    : std::true_type {};

template <typename Health, typename = void>
struct HasSafeCommandSupported : std::false_type {};

template <typename Health>
struct HasSafeCommandSupported<
    Health,
    std::void_t<decltype(std::declval<Health&>().safe_command_supported)>>
    : std::true_type {};

static_assert(!HasWriteSafeCommand<RobotSdkAdapter>::value);
static_assert(!HasSafeCommandSupported<AdapterHealth>::value);

class FakeRobotSdkAdapter final : public RobotSdkAdapter {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "fake";
  }

  AdapterCallResult initialize() override {
    initialized_ = true;
    return {.status = AdapterCallStatus::kOk, .detail = "initialized"};
  }

  [[nodiscard]] AdapterReadResult read_state() override {
    StateFrame state;
    state.sequence = 9;
    state.sample_time_ns = 1000;
    state.publish_time_ns = 1010;
    state.mode_pr = 3;
    state.mode_machine = 2;
    state.lifecycle_state = LifecycleState::kActive;
    state.active_mode = ControlMode::kTorque;
    state.mode_label = "closed_loop";
    state.valid = true;
    state.degraded = false;
    state.command_stale = false;
    state.joints = JointStateBlock{
        .position = {1.0, 2.0},
        .velocity = {0.1, 0.2},
        .effort = {0.0, 0.0},
        .acceleration = {0.01, 0.02},
    };
    state.imu = ImuSample{
        .sample_time_ns = 1005,
        .valid = true,
        .stale = false,
        .orientation_xyzw = {0.0, 0.0, 0.0, 1.0},
        .rpy_xyz = {0.0, 0.0, 0.0},
        .angular_velocity_xyz = {0.0, 0.0, 0.0},
        .linear_acceleration_xyz = {0.0, 0.0, 9.81},
    };
    return {
        .result = {.status = AdapterCallStatus::kOk, .detail = "read"},
        .state = std::move(state),
    };
  }

  AdapterCallResult write_command(const CommandFrame& command) override {
    last_command_ = command;
    return {.status = AdapterCallStatus::kOk, .detail = "write"};
  }

  [[nodiscard]] AdapterHealthResult query_health() const override {
    AdapterHealth health;
    health.state = initialized_ ? HealthState::kHealthy : HealthState::kUnknown;
    health.connection_alive = initialized_;
    health.degraded = false;
    health.last_write_time_ns = last_command_.receive_time_ns;
    health.last_read_time_ns = 1000;
    return {
        .result = {.status = AdapterCallStatus::kOk, .detail = "health"},
        .health = std::move(health),
    };
  }

  AdapterCallResult shutdown() override {
    initialized_ = false;
    return {.status = AdapterCallStatus::kOk, .detail = "shutdown"};
  }

  [[nodiscard]] const CommandFrame& last_command() const noexcept {
    return last_command_;
  }

 private:
  bool initialized_{false};
  CommandFrame last_command_{};
};

BridgeFault MakeFault() {
  return {
      .category = FaultCategory::kAdapterWrite,
      .severity = FaultSeverity::kFatal,
      .lifecycle_state = LifecycleState::kFaulted,
      .message = "write failed",
      .latched = true,
      .sequence = 3,
      .timestamp_ns = 77,
  };
}

}

int main() {
  FakeRobotSdkAdapter adapter;
  assert(adapter.name() == std::string_view{"fake"});
  assert(!adapter.capabilities().contains(RobotCapability::kJointState));

  const AdapterCallResult init = adapter.initialize();
  assert(init.succeeded());

  CommandFrame command;
  command.sequence = 4;
  command.source_time_ns = 50;
  command.receive_time_ns = 60;
  command.mode_pr = 3;
  command.mode_machine = 2;
  command.mode = ControlMode::kTorque;
  command.mode_label = "torque";
  command.valid = true;
  command.joints.position = {1.0, 2.0};
  command.joints.velocity = {0.0, 0.0};
  command.joints.effort = {0.5, 0.6};
  command.joints.stiffness = {10.0, 10.0};
  command.joints.damping = {1.0, 1.0};

  const AdapterCallResult write = adapter.write_command(command);
  assert(write.succeeded());
  assert(adapter.last_command().joints.width() == 2U);

  const AdapterReadResult read = adapter.read_state();
  assert(read.result.succeeded());
  assert(read.state.has_value());
  assert(read.state->joints.width() == 2U);
  assert(read.state->imu.has_value());
  assert(read.state->lifecycle_state == LifecycleState::kActive);

  const AdapterHealthResult health = adapter.query_health();
  assert(health.result.succeeded());
  assert(health.health.state == HealthState::kHealthy);

  const BridgeFault fault = MakeFault();
  assert(fault.has_fault());

  const AdapterCallResult shutdown = adapter.shutdown();
  assert(shutdown.succeeded());

  return 0;
}
