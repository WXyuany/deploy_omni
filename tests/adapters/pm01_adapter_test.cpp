#include <cassert>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "../../include/bridge/adapters/pm01/pm01_adapter.hpp"

namespace {

using bridge::adapters::pm01::Pm01Adapter;
using bridge::adapters::pm01::Pm01AdapterConfig;
using bridge::core::AdapterCallStatus;
using bridge::core::ControlMode;
using bridge::core::FaultCategory;
using bridge::core::LifecycleState;

template <typename Adapter, typename = void>
struct HasPublicUpdateMotionTask : std::false_type {};

template <typename Adapter>
struct HasPublicUpdateMotionTask<
    Adapter,
    std::void_t<decltype(std::declval<Adapter&>().update_motion_task(
        std::declval<std::optional<std::string>>()))>> : std::true_type {};

static_assert(!HasPublicUpdateMotionTask<Pm01Adapter>::value);

Pm01AdapterConfig MakeBaseConfig() {
  Pm01AdapterConfig config;
  config.session.robot_id = "pm01-test";
  config.session.transport_address = "dds:pm01";
  config.session.control_profile = "joint_bridge";
  config.session.initialize_timeout_ns = 10;
  config.session.read_timeout_ns = 20;
  config.session.write_timeout_ns = 30;
  config.motion_state.required = true;
  config.motion_state.required_task = "joint_bridge";
  config.publish_default_imu_when_missing = true;
  config.joint_names.resize(24);
  for (std::size_t index = 0; index < config.joint_names.size(); ++index) {
    config.joint_names[index] = "joint_" + std::to_string(index);
    config.dds_joint_indices.push_back(static_cast<std::int64_t>(index));
  }
  return config;
}

Pm01AdapterConfig MakeMappedConfig() {
  Pm01AdapterConfig config = MakeBaseConfig();
  config.motion_state.required = false;
  config.dds_joint_indices.clear();
  for (std::size_t index = 0; index < config.joint_names.size(); ++index) {
    config.dds_joint_indices.push_back(static_cast<std::int64_t>(23 - index));
  }
  return config;
}

bridge::core::CommandFrame MakeCommand(const ControlMode mode) {
  bridge::core::CommandFrame command;
  command.sequence = 1;
  command.source_time_ns = 100;
  command.receive_time_ns = 110;
  command.mode = mode;
  command.mode_label = mode == ControlMode::kSafeIdle ? "safe_idle" : "position";
  command.valid = true;
  command.joints.position.resize(24, 0.25);
  command.joints.velocity.resize(24, 0.0);
  command.joints.effort.resize(24, 0.0);
  command.joints.stiffness.resize(24, 10.0);
  command.joints.damping.resize(24, 1.0);
  return command;
}

void TestConfigValidationFailsWhenDdsJointIndicesWidthIsWrong() {
  Pm01AdapterConfig config = MakeBaseConfig();
  config.dds_joint_indices.pop_back();

  Pm01Adapter adapter(config);
  const auto init = adapter.initialize();
  assert(init.status == AdapterCallStatus::kFatalError);
  assert(init.fault.has_value());
  assert(init.fault->category == FaultCategory::kConfiguration);
}

void TestMotionStateGateBlocksCommandUntilJointBridgeIsActive() {
  Pm01Adapter adapter(MakeBaseConfig());
  assert(adapter.initialize().status == AdapterCallStatus::kOk);
  assert(adapter.read_state().result.status == AdapterCallStatus::kOk);

  const auto blocked = adapter.write_command(MakeCommand(ControlMode::kPosition));
  assert(blocked.status == AdapterCallStatus::kRetryableError);
  assert(blocked.fault.has_value());
  assert(blocked.fault->category == FaultCategory::kSafety);
}

void TestMotionStateGateUsesConfiguredRequiredTask() {
  Pm01AdapterConfig config = MakeBaseConfig();
  config.motion_state.required_task = "dance_mode";
  config.motion_state.initial_task = "joint_bridge";
  Pm01Adapter adapter(config);
  assert(adapter.initialize().status == AdapterCallStatus::kOk);
  assert(adapter.read_state().result.status == AdapterCallStatus::kOk);
  const auto blocked = adapter.write_command(MakeCommand(ControlMode::kPosition));
  assert(blocked.status == AdapterCallStatus::kRetryableError);
  assert(blocked.fault.has_value());
  assert(blocked.fault->category == FaultCategory::kSafety);
}

void TestInitialMotionTaskAllowsConfiguredMotionWrite() {
  Pm01AdapterConfig config = MakeBaseConfig();
  config.motion_state.initial_task = "joint_bridge";

  Pm01Adapter adapter(config);
  assert(adapter.initialize().status == AdapterCallStatus::kOk);
  assert(adapter.read_state().result.status == AdapterCallStatus::kOk);
  assert(adapter.write_command(MakeCommand(ControlMode::kPosition)).status ==
         AdapterCallStatus::kOk);
}

void TestDdsJointIndicesDriveReadbackProjection() {
  Pm01Adapter adapter(MakeMappedConfig());
  assert(adapter.initialize().status == AdapterCallStatus::kOk);
  assert(adapter.read_state().result.status == AdapterCallStatus::kOk);

  auto command = MakeCommand(ControlMode::kPosition);
  for (std::size_t index = 0; index < command.joints.position.size(); ++index) {
    command.joints.position[index] = static_cast<double>(index) + 0.5;
    command.joints.velocity[index] = static_cast<double>(index) + 10.0;
    command.joints.effort[index] = static_cast<double>(index) + 20.0;
  }

  assert(adapter.write_command(command).status == AdapterCallStatus::kOk);
  const auto read = adapter.read_state();
  assert(read.result.status == AdapterCallStatus::kOk);
  assert(read.state.has_value());
  for (std::size_t index = 0; index < command.joints.position.size(); ++index) {
    assert(read.state->joints.position[index] == command.joints.position[index]);
    assert(read.state->joints.velocity[index] == command.joints.velocity[index]);
    assert(read.state->joints.effort[index] == command.joints.effort[index]);
  }
}

void TestReadStateProvidesDefaultImuWhenUnavailable() {
  Pm01Adapter adapter(MakeBaseConfig());
  assert(adapter.initialize().status == AdapterCallStatus::kOk);

  const auto read = adapter.read_state();
  assert(read.result.status == AdapterCallStatus::kOk);
  assert(read.state.has_value());
  assert(read.state->lifecycle_state == LifecycleState::kActive);
  assert(read.state->imu.has_value());
  assert(!read.state->imu->valid);
  assert(read.state->imu->stale);
  assert(read.state->imu->orientation_xyzw.size() == 4U);
  assert(read.state->imu->orientation_xyzw[3] == 1.0);
  assert(read.state->joints.position.size() == 24U);
}

void TestSafeCommandUnsupportedReportsCapability() {
  Pm01AdapterConfig config = MakeBaseConfig();
  config.safety.safe_command_supported = false;

  Pm01Adapter adapter(config);
  assert(adapter.initialize().status == AdapterCallStatus::kOk);

  const auto safe = adapter.write_safe_command();
  assert(safe.status == AdapterCallStatus::kUnsupported);

  const auto health = adapter.query_health();
  assert(health.result.status == AdapterCallStatus::kOk);
  assert(!health.health.safe_command_supported);
  assert(health.health.connection_alive);
}

}

int main() {
  TestConfigValidationFailsWhenDdsJointIndicesWidthIsWrong();
  TestMotionStateGateBlocksCommandUntilJointBridgeIsActive();
  TestMotionStateGateUsesConfiguredRequiredTask();
  TestInitialMotionTaskAllowsConfiguredMotionWrite();
  TestDdsJointIndicesDriveReadbackProjection();
  TestReadStateProvidesDefaultImuWhenUnavailable();
  TestSafeCommandUnsupportedReportsCapability();
  return 0;
}
