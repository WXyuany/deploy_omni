#include <cassert>

#include "../../include/bridge/adapters/kepler_k2/kepler_k2_adapter.hpp"

namespace {

using bridge::adapters::kepler_k2::ImuIntegrationConfig;
using bridge::adapters::kepler_k2::ImuIntegrationMode;
using bridge::adapters::kepler_k2::KeplerK2Adapter;
using bridge::adapters::kepler_k2::KeplerK2AdapterConfig;
using bridge::adapters::kepler_k2::SharedMemoryImuConfig;
using bridge::core::AdapterCallStatus;
using bridge::core::CommandFrame;
using bridge::core::ControlMode;
using bridge::core::FaultCategory;
using bridge::core::HealthState;
using bridge::core::LifecycleState;

KeplerK2AdapterConfig MakeBaseConfig() {
  KeplerK2AdapterConfig config;
  config.session.robot_id = "k2-test";
  config.session.transport_endpoint = "loopback";
  config.session.control_profile = "deterministic";
  config.session.initialize_timeout_ns = 10;
  config.session.read_timeout_ns = 20;
  config.session.write_timeout_ns = 30;
  config.joint_names = {"left_hip", "right_hip"};
  config.hardware_slot_count = config.joint_names.size();
  return config;
}

CommandFrame MakeCommand(const ControlMode mode) {
  CommandFrame command;
  command.sequence = 1;
  command.source_time_ns = 100;
  command.receive_time_ns = 110;
  command.mode = mode;
  command.mode_label = mode == ControlMode::kSafeIdle ? "safe_idle" : "torque";
  command.valid = true;
  command.joints.position = {0.1, -0.1};
  command.joints.velocity = {0.2, -0.2};
  command.joints.effort = {0.3, -0.3};
  return command;
}

CommandFrame MakeThreeSlotCommand(const ControlMode mode) {
  CommandFrame command = MakeCommand(mode);
  command.joints.position = {0.1, -0.1, 0.0};
  command.joints.velocity = {0.2, -0.2, 0.0};
  command.joints.effort = {0.3, -0.3, 0.0};
  return command;
}

void TestConfigValidationFailsWhenImuRequiredWithoutConfig() {
  KeplerK2AdapterConfig config = MakeBaseConfig();
  config.safety.allow_command_when_imu_missing = false;

  KeplerK2Adapter adapter(config);
  const auto init = adapter.initialize();
  assert(init.status == AdapterCallStatus::kFatalError);
  assert(init.fault.has_value());
  assert(init.fault->category == FaultCategory::kConfiguration);

  const auto health = adapter.query_health();
  assert(health.result.status == AdapterCallStatus::kFatalError);
  assert(health.health.state == HealthState::kFaulted);
  assert(health.health.fault.has_value());
  assert(health.health.fault->category == FaultCategory::kConfiguration);
}

void TestInitialStateIsRequiredBeforeMotion() {
  KeplerK2Adapter adapter(MakeBaseConfig());
  assert(adapter.initialize().status == AdapterCallStatus::kOk);

  const auto motion_before_read = adapter.write_command(MakeCommand(ControlMode::kTorque));
  assert(motion_before_read.status == AdapterCallStatus::kRetryableError);
  assert(motion_before_read.fault.has_value());
  assert(motion_before_read.fault->category == FaultCategory::kSafety);

  const auto first_read = adapter.read_state();
  assert(first_read.result.status == AdapterCallStatus::kOk);
  assert(first_read.state.has_value());
  assert(first_read.state->lifecycle_state == LifecycleState::kActive);

  const auto motion_after_read = adapter.write_command(MakeCommand(ControlMode::kTorque));
  assert(motion_after_read.status == AdapterCallStatus::kOk);
}

void TestImuGuardAppliesOnlyToMotionCommands() {
  KeplerK2AdapterConfig config = MakeBaseConfig();
  config.safety.allow_command_when_imu_missing = false;
  config.imu = ImuIntegrationConfig{
      .mode = ImuIntegrationMode::kSharedMemory,
      .frame_id = "imu",
      .shared_memory = SharedMemoryImuConfig{
          .region_name = "k2-imu",
          .mapping_name = "",
          .expected_period_ns = 1000,
          .stale_after_ns = 2000,
          .required = false,
      },
  };

  KeplerK2Adapter adapter(config);
  assert(adapter.initialize().status == AdapterCallStatus::kOk);
  assert(adapter.read_state().result.status == AdapterCallStatus::kOk);

  const auto safe_idle = adapter.write_command(MakeCommand(ControlMode::kSafeIdle));
  assert(safe_idle.status == AdapterCallStatus::kOk);

  const auto motion = adapter.write_command(MakeCommand(ControlMode::kTorque));
  assert(motion.status == AdapterCallStatus::kRetryableError);
  assert(motion.fault.has_value());
  assert(motion.fault->category == FaultCategory::kSafety);
}

void TestRequiredSharedMemoryImuSurfacesDegradedHealth() {
  KeplerK2AdapterConfig config = MakeBaseConfig();
  config.imu = ImuIntegrationConfig{
      .mode = ImuIntegrationMode::kSharedMemory,
      .frame_id = "imu",
      .shared_memory = SharedMemoryImuConfig{
          .region_name = "k2-imu",
          .mapping_name = "",
          .expected_period_ns = 1000,
          .stale_after_ns = 2000,
          .required = true,
      },
  };

  KeplerK2Adapter adapter(config);
  const auto init = adapter.initialize();
  assert(init.status == AdapterCallStatus::kDegraded);

  const auto read = adapter.read_state();
  assert(read.result.status == AdapterCallStatus::kDegraded);
  assert(read.state.has_value());
  assert(read.state->degraded);
  assert(read.state->fault.has_value());
  assert(read.state->fault->category == FaultCategory::kAdapterHealth);

  const auto health = adapter.query_health();
  assert(health.result.status == AdapterCallStatus::kDegraded);
  assert(health.health.state == HealthState::kDegraded);
  assert(health.health.degraded);
  assert(health.health.connection_alive);
  assert(health.health.fault.has_value());
  assert(health.health.fault->category == FaultCategory::kAdapterHealth);
}

void TestShutdownStopsFurtherAccess() {
  KeplerK2Adapter adapter(MakeBaseConfig());
  assert(adapter.initialize().status == AdapterCallStatus::kOk);
  assert(adapter.read_state().result.status == AdapterCallStatus::kOk);
  assert(adapter.write_command(MakeCommand(ControlMode::kTorque)).status ==
         AdapterCallStatus::kOk);

  const auto shutdown = adapter.shutdown();
  assert(shutdown.status == AdapterCallStatus::kOk);

  const auto health = adapter.query_health();
  assert(health.result.status == AdapterCallStatus::kOk);
  assert(health.health.state == HealthState::kUnknown);
  assert(!health.health.connection_alive);

  const auto read_after_shutdown = adapter.read_state();
  assert(read_after_shutdown.result.status == AdapterCallStatus::kRetryableError);
}

void TestHardwareSlotWidthCanExceedEnabledJointNames() {
  KeplerK2AdapterConfig config = MakeBaseConfig();
  config.hardware_slot_count = 3;

  KeplerK2Adapter adapter(config);
  assert(adapter.initialize().status == AdapterCallStatus::kOk);
  assert(adapter.read_state().result.status == AdapterCallStatus::kOk);

  const auto write = adapter.write_command(MakeThreeSlotCommand(ControlMode::kTorque));
  assert(write.status == AdapterCallStatus::kOk);
}

void TestRepeatedWritesRecomputeEffortFromLatestState() {
  KeplerK2Adapter adapter(MakeBaseConfig());
  assert(adapter.initialize().status == AdapterCallStatus::kOk);

  const auto first_read = adapter.read_state();
  assert(first_read.result.status == AdapterCallStatus::kOk);

  CommandFrame command = MakeCommand(ControlMode::kTorque);
  assert(adapter.write_command(command).status == AdapterCallStatus::kOk);

  const auto second_read = adapter.read_state();
  assert(second_read.result.status == AdapterCallStatus::kOk);
  assert(second_read.state.has_value());
  const auto first_effort = second_read.state->joints.effort;

  assert(adapter.write_command(command).status == AdapterCallStatus::kOk);
  const auto third_read = adapter.read_state();
  assert(third_read.result.status == AdapterCallStatus::kOk);
  assert(third_read.state.has_value());

  assert(first_effort.size() == third_read.state->joints.effort.size());
  assert(first_effort != third_read.state->joints.effort);
}

}

int main() {
  TestConfigValidationFailsWhenImuRequiredWithoutConfig();
  TestInitialStateIsRequiredBeforeMotion();
  TestImuGuardAppliesOnlyToMotionCommands();
  TestRequiredSharedMemoryImuSurfacesDegradedHealth();
  TestShutdownStopsFurtherAccess();
  TestHardwareSlotWidthCanExceedEnabledJointNames();
  TestRepeatedWritesRecomputeEffortFromLatestState();
  return 0;
}
