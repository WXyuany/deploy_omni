#include <cassert>

#include "../../include/bridge/adapters/unitree_g1/unitree_g1_adapter.hpp"

namespace {

using bridge::adapters::unitree_g1::UnitreeG1Adapter;
using bridge::adapters::unitree_g1::UnitreeG1AdapterConfig;
using bridge::core::AdapterCallStatus;
using bridge::core::CommandFrame;
using bridge::core::ControlMode;
using bridge::core::FaultCategory;

UnitreeG1AdapterConfig MakeBaseConfig() {
  UnitreeG1AdapterConfig config;
  config.session.transport_mode = "mock";
  config.session.robot_id = "g1-test";
  config.session.transport_endpoint = "mock:robot_lan0";
  config.session.control_profile = "raw_low_level";
  config.session.command_topic = "rt/lowcmd";
  config.session.state_topic = "rt/lowstate";
  config.session.imu_topic = "rt/secondary_imu";
  config.session.initialize_timeout_ns = 10;
  config.session.read_timeout_ns = 20;
  config.session.read_retry_attempts = 0;
  config.session.read_retry_interval_ns = 0;
  config.session.write_timeout_ns = 30;
  config.joint_names = {
      "left_hip_pitch",  "left_hip_roll",   "left_hip_yaw",
      "left_knee",       "left_ankle_pitch", "left_ankle_roll",
      "right_hip_pitch", "right_hip_roll",  "right_hip_yaw",
      "right_knee",      "right_ankle_pitch", "right_ankle_roll",
      "waist_yaw",       "waist_roll",      "waist_pitch",
      "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
      "left_elbow",      "left_wrist_roll", "left_wrist_pitch",
      "left_wrist_yaw",  "right_shoulder_pitch", "right_shoulder_roll",
      "right_shoulder_yaw", "right_elbow",  "right_wrist_roll",
      "right_wrist_pitch", "right_wrist_yaw"};
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
  command.mode_pr = 1;
  command.mode_machine = 42;
  command.valid = true;
  command.joints.position.assign(29, 0.1);
  command.joints.velocity.assign(29, 0.0);
  command.joints.effort.assign(29, 0.0);
  command.joints.stiffness.assign(29, 60.0);
  command.joints.damping.assign(29, 1.0);
  return command;
}

void TestInitialStateIsRequiredBeforeMotion() {
  UnitreeG1Adapter adapter(MakeBaseConfig());
  assert(adapter.initialize().status == AdapterCallStatus::kOk);

  const auto motion_before_read = adapter.write_command(MakeCommand(ControlMode::kTorque));
  assert(motion_before_read.status == AdapterCallStatus::kRetryableError);
  assert(motion_before_read.fault.has_value());
  assert(motion_before_read.fault->category == FaultCategory::kSafety);

  const auto first_read = adapter.read_state();
  assert(first_read.result.status == AdapterCallStatus::kRetryableError);
  assert(!first_read.state.has_value());

  const auto motion_after_read = adapter.write_command(MakeCommand(ControlMode::kTorque));
  assert(motion_after_read.status == AdapterCallStatus::kRetryableError);
}

void TestReadStateRequiresRealSessionState() {
  UnitreeG1Adapter adapter(MakeBaseConfig());
  assert(adapter.initialize().status == AdapterCallStatus::kOk);

  const auto read = adapter.read_state();
  assert(read.result.status == AdapterCallStatus::kRetryableError);
  assert(!read.state.has_value());
}

void TestShutdownStopsFurtherAccess() {
  UnitreeG1Adapter adapter(MakeBaseConfig());
  assert(adapter.initialize().status == AdapterCallStatus::kOk);
  assert(adapter.read_state().result.status == AdapterCallStatus::kRetryableError);
  assert(adapter.shutdown().status == AdapterCallStatus::kOk);

  const auto read_after_shutdown = adapter.read_state();
  assert(read_after_shutdown.result.status == AdapterCallStatus::kRetryableError);
}

}

int main() {
  TestInitialStateIsRequiredBeforeMotion();
  TestReadStateRequiresRealSessionState();
  TestShutdownStopsFurtherAccess();
  return 0;
}
