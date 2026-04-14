#include <cassert>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "../../include/bridge/config/joint_map.hpp"
#include "../../include/bridge/dds/dds_translation.hpp"

namespace {

using bridge::config::JointFieldTransform;
using bridge::config::JointMap;
using bridge::config::JointMapEntry;
using bridge::config::JointMapValidationOptions;
using bridge::core::BridgeFault;
using bridge::core::ControlMode;
using bridge::core::LifecycleState;
using bridge::core::StateFrame;
using bridge::dds::translate_hardware_state_to_bridge_status;
using bridge::dds::translate_hardware_state_to_lowstate;
using bridge::dds::translate_lowcmd_to_command;
using bridge::dds::translate_lowcmd_to_hardware_command;

JointMap MakeSwappingJointMap() {
  std::vector<JointMapEntry> entries;
  for (std::size_t sim_index = 0; sim_index < 2U; ++sim_index) {
    JointMapEntry entry;
    entry.sim_index = sim_index;
    entry.hardware_index = 1U - sim_index;
    entry.sim_name = std::string("sim_") + std::to_string(sim_index);
    entry.hardware_name = std::string("hw_") + std::to_string(1U - sim_index);
    entry.transforms.position = JointFieldTransform{};
    entry.transforms.velocity = JointFieldTransform{};
    entry.transforms.effort = JointFieldTransform{};
    entry.transforms.stiffness = JointFieldTransform{};
    entry.transforms.damping = JointFieldTransform{};
    entry.transforms.acceleration = JointFieldTransform{};
    entries.push_back(std::move(entry));
  }

  JointMapValidationOptions options;
  options.expected_sim_joint_count = 2U;
  options.expected_hardware_joint_count = 2U;
  const auto built = JointMap::build(std::move(entries), options);
  assert(built.succeeded());
  return *built.joint_map;
}

bridge_wire::LowCmd MakeLowCmd() {
  bridge_wire::LowCmd message;
  message.seq_id(12U);
  message.source_time_ns(345U);
  message.control_mode(5U);
  message.mode_label("hybrid");
  message.valid(true);
  message.stale(false);
  message.degraded(true);
  message.position({1.0, 2.0});
  message.velocity({3.0, 4.0});
  message.effort({5.0, 6.0});
  message.stiffness({7.0, 8.0});
  message.damping({9.0, 10.0});
  return message;
}

StateFrame MakeHardwareState() {
  StateFrame state;
  state.sequence = 20U;
  state.sample_time_ns = 1000U;
  state.publish_time_ns = 1010U;
  state.lifecycle_state = LifecycleState::kActive;
  state.active_mode = ControlMode::kTorque;
  state.mode_label = "torque_mode";
  state.valid = true;
  state.degraded = true;
  state.command_stale = true;
  state.joints.position = {11.0, 22.0};
  state.joints.velocity = {33.0, 44.0};
  state.joints.effort = {55.0, 66.0};
  state.joints.acceleration = {77.0, 88.0};
  state.fault = BridgeFault{
      .category = bridge::core::FaultCategory::kCommandStale,
      .severity = bridge::core::FaultSeverity::kRecoverable,
      .lifecycle_state = LifecycleState::kFaulted,
      .message = "stale command",
      .latched = true,
      .sequence = 9U,
      .timestamp_ns = 2000U,
  };
  return state;
}

void TestTranslateLowCmdToCommand() {
  const auto translated = translate_lowcmd_to_command(MakeLowCmd(), 400U);
  assert(translated.result.succeeded());
  assert(translated.command.has_value());
  assert(translated.command->sequence == 12U);
  assert(translated.command->source_time_ns == 345U);
  assert(translated.command->receive_time_ns == 400U);
  assert(translated.command->mode == ControlMode::kHybrid);
  assert(translated.command->mode_label == "hybrid");
  assert(translated.command->degraded);
  assert(translated.command->joints.position[0] == 1.0);
  assert(translated.command->joints.position[1] == 2.0);
}

void TestTranslateLowCmdToHardwareCommandThroughJointMap() {
  const auto translated =
      translate_lowcmd_to_hardware_command(MakeLowCmd(), 400U, MakeSwappingJointMap());
  assert(translated.result.succeeded());
  assert(translated.command.has_value());
  assert(translated.command->joints.position[0] == 2.0);
  assert(translated.command->joints.position[1] == 1.0);
  assert(translated.command->joints.velocity[0] == 4.0);
  assert(translated.command->joints.velocity[1] == 3.0);
}

void TestTranslateHardwareStateToLowStateThroughJointMap() {
  const auto translated =
      translate_hardware_state_to_lowstate(MakeHardwareState(), MakeSwappingJointMap());
  assert(translated.result.succeeded());
  assert(translated.state.has_value());
  assert(translated.state->seq_id() == 20U);
  assert(translated.state->active_mode() == 2U);
  assert(translated.state->position()[0] == 22.0);
  assert(translated.state->position()[1] == 11.0);
  assert(translated.state->effort()[0] == 66.0);
  assert(translated.state->effort()[1] == 55.0);
}

void TestTranslateHardwareStateToBridgeStatus() {
  const auto translated =
      translate_hardware_state_to_bridge_status(MakeHardwareState(), MakeSwappingJointMap());
  assert(translated.result.succeeded());
  assert(translated.status.has_value());
  assert(translated.status->seq_id() == 20U);
  assert(translated.status->control_mode() == 2U);
  assert(translated.status->fault_present());
  assert(translated.status->fault_sequence() == 9U);
  assert(translated.status->fault_message() == "stale command");
}

void TestRejectsLowCmdWithNonFiniteValues() {
  auto message = MakeLowCmd();
  message.velocity({1.0, std::nan("")});

  const auto translated = translate_lowcmd_to_command(message, 400U);
  assert(!translated.result.succeeded());
  assert(!translated.command.has_value());
  assert(translated.result.fault.has_value());
  assert(translated.result.fault->category == bridge::core::FaultCategory::kInvalidCommand);
}

void TestRejectsLowCmdWithMismatchedVectorWidthsAndFault() {
  auto message = MakeLowCmd();
  message.effort({5.0});

  const auto translated = translate_lowcmd_to_command(message, 400U);
  assert(!translated.result.succeeded());
  assert(!translated.command.has_value());
  assert(translated.result.fault.has_value());
  assert(translated.result.fault->category == bridge::core::FaultCategory::kInvalidCommand);
}

void TestRejectsStateWithInvalidImuVectorWidths() {
  auto state = MakeHardwareState();
  state.imu = bridge::core::ImuSample{
      .sample_time_ns = 1005U,
      .valid = true,
      .stale = false,
      .orientation_xyzw = {0.0, 0.0, 1.0},
      .angular_velocity_xyz = {0.1, 0.2, 0.3},
      .linear_acceleration_xyz = {0.0, 0.0, 9.81},
  };

  const auto translated =
      translate_hardware_state_to_lowstate(state, MakeSwappingJointMap());
  assert(!translated.result.succeeded());
  assert(!translated.state.has_value());
}

}

int main() {
  TestTranslateLowCmdToCommand();
  TestTranslateLowCmdToHardwareCommandThroughJointMap();
  TestTranslateHardwareStateToLowStateThroughJointMap();
  TestTranslateHardwareStateToBridgeStatus();
  TestRejectsLowCmdWithNonFiniteValues();
  TestRejectsLowCmdWithMismatchedVectorWidthsAndFault();
  TestRejectsStateWithInvalidImuVectorWidths();
  return 0;
}
