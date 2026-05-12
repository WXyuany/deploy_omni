#include <cassert>
#include <cmath>
#include <string>
#include <vector>

#include <unitree/dds_wrapper/common/crc.h>

#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"

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
using bridge::dds::translate_hardware_state_to_lowstate;
using bridge::dds::translate_lowcmd_to_command;
using bridge::dds::translate_lowcmd_to_hardware_command;
using unitree_hg::msg::dds_::LowCmd_;

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
  options.require_dense_enabled_sim_indexes = false;
  options.require_dense_enabled_hardware_indexes = false;
  const auto built = JointMap::build(std::move(entries), options);
  assert(built.succeeded());
  return *built.joint_map;
}

JointMap MakeJointMapWithDisabledHighestSlot() {
  std::vector<JointMapEntry> entries;
  for (std::size_t index = 0; index < 3U; ++index) {
    JointMapEntry entry;
    entry.sim_index = index;
    entry.hardware_index = index;
    entry.sim_name = std::string("sim_") + std::to_string(index);
    entry.hardware_name = std::string("hw_") + std::to_string(index);
    entry.transforms.position = JointFieldTransform{};
    entry.transforms.velocity = JointFieldTransform{};
    entry.transforms.effort = JointFieldTransform{};
    entry.transforms.stiffness = JointFieldTransform{};
    entry.transforms.damping = JointFieldTransform{};
    entry.transforms.acceleration = JointFieldTransform{};
    entry.enabled = index != 2U;
    entries.push_back(std::move(entry));
  }

  JointMapValidationOptions options;
  options.require_dense_enabled_sim_indexes = false;
  options.require_dense_enabled_hardware_indexes = false;
  const auto built = JointMap::build(std::move(entries), options);
  assert(built.succeeded());
  return *built.joint_map;
}

LowCmd_ MakeLowCmd() {
  LowCmd_ message;
  message.mode_pr(9U);
  message.mode_machine(5U);
  auto reserve = message.reserve();
  reserve[0] = 12U;
  reserve[1] = 345U;
  message.reserve(reserve);

  auto motors = message.motor_cmd();
  motors[0].q(1.0F);
  motors[0].dq(3.0F);
  motors[0].tau(5.0F);
  motors[0].kp(7.0F);
  motors[0].kd(9.0F);
  motors[1].q(2.0F);
  motors[1].dq(4.0F);
  motors[1].tau(6.0F);
  motors[1].kp(8.0F);
  motors[1].kd(10.0F);
  message.motor_cmd(motors);
  message.crc() = crc32_core(reinterpret_cast<uint32_t*>(&message),
                             (sizeof(message) >> 2) - 1);
  return message;
}

void RecomputeLowCmdCrc(LowCmd_& message) {
  message.crc() = crc32_core(reinterpret_cast<uint32_t*>(&message),
                             (sizeof(message) >> 2) - 1);
}

StateFrame MakeHardwareState() {
  StateFrame state;
  state.sequence = 20U;
  state.sample_time_ns = 1000U;
  state.publish_time_ns = 1010U;
  state.mode_pr = 9U;
  state.mode_machine = 5U;
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
  state.imu = bridge::core::ImuSample{
      .sample_time_ns = 1005U,
      .valid = true,
      .stale = false,
      .orientation_xyzw = {0.2, 0.3, 0.4, 0.5},
      .rpy_xyz = {0.6, 0.7, 0.8},
      .angular_velocity_xyz = {0.1, 0.2, 0.3},
      .linear_acceleration_xyz = {9.1, 9.2, 9.3},
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
  assert(translated.command->mode_pr == 9U);
  assert(translated.command->mode_machine == 5U);
  assert(translated.command->mode == ControlMode::kHybrid);
  assert(translated.command->joints.position.size() == 35U);
  assert(translated.command->joints.position[0] == 1.0);
  assert(translated.command->joints.position[1] == 2.0);
  assert(translated.command->joints.effort[0] == 5.0);
  assert(translated.command->joints.stiffness[0] == 7.0);
  assert(translated.command->joints.damping[1] == 10.0);
}

void TestTranslateLowCmdToHardwareCommandThroughJointMap() {
  const auto translated =
      translate_lowcmd_to_hardware_command(MakeLowCmd(), 400U, MakeSwappingJointMap());
  assert(translated.result.succeeded());
  assert(translated.command.has_value());
  assert(translated.command->mode_pr == 9U);
  assert(translated.command->mode_machine == 5U);
  assert(translated.command->joints.position.size() == 2U);
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
  assert(translated.state->tick() == 20U);
  assert(translated.state->mode_pr() == 9U);
  assert(translated.state->mode_machine() == 5U);
  assert(translated.state->motor_state()[0].q() == 22.0F);
  assert(translated.state->motor_state()[1].q() == 11.0F);
  assert(translated.state->motor_state()[0].tau_est() == 66.0F);
  assert(translated.state->motor_state()[1].tau_est() == 55.0F);
  assert(translated.state->imu_state().quaternion()[0] == 0.5F);
  assert(translated.state->imu_state().quaternion()[1] == 0.2F);
  assert(translated.state->imu_state().quaternion()[2] == 0.3F);
  assert(translated.state->imu_state().quaternion()[3] == 0.4F);
  assert(translated.state->imu_state().rpy()[0] == 0.6F);
  assert(translated.state->imu_state().rpy()[1] == 0.7F);
  assert(translated.state->imu_state().rpy()[2] == 0.8F);
}

void TestRejectsLowCmdWithNonFiniteValues() {
  auto message = MakeLowCmd();
  auto motors = message.motor_cmd();
  motors[1].dq(std::nan(""));
  message.motor_cmd(motors);
  RecomputeLowCmdCrc(message);

  const auto translated = translate_lowcmd_to_command(message, 400U);
  assert(!translated.result.succeeded());
  assert(!translated.command.has_value());
  assert(translated.result.fault.has_value());
  assert(translated.result.fault->category == bridge::core::FaultCategory::kInvalidCommand);
}

void TestRejectsLowCmdWithInvalidCrc() {
  auto message = MakeLowCmd();
  message.crc(message.crc() + 1U);

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
      .rpy_xyz = {0.0, 0.0, 0.0},
      .angular_velocity_xyz = {0.1, 0.2, 0.3},
      .linear_acceleration_xyz = {0.0, 0.0, 9.81},
  };

  const auto translated =
      translate_hardware_state_to_lowstate(state, MakeSwappingJointMap());
  assert(!translated.result.succeeded());
  assert(!translated.state.has_value());
}

void TestDisabledJointSlotsRemainZeroInDdsTranslations() {
  auto message = MakeLowCmd();
  auto motors = message.motor_cmd();
  motors[0].q(1.0F);
  motors[0].dq(2.0F);
  motors[0].tau(3.0F);
  motors[0].kp(4.0F);
  motors[0].kd(5.0F);
  motors[1].q(9.0F);
  motors[1].dq(8.0F);
  motors[1].tau(7.0F);
  motors[1].kp(6.0F);
  motors[1].kd(5.0F);
  motors[2].q(3.0F);
  motors[2].dq(4.0F);
  motors[2].tau(5.0F);
  motors[2].kp(6.0F);
  motors[2].kd(7.0F);
  message.motor_cmd(motors);
  RecomputeLowCmdCrc(message);

  const auto command = translate_lowcmd_to_hardware_command(
      message, 400U, MakeJointMapWithDisabledHighestSlot());
  assert(command.result.succeeded());
  assert(command.command.has_value());
  assert(command.command->joints.position.size() == 2U);
  assert(command.command->joints.position[0] == 1.0);
  assert(command.command->joints.position[1] == 9.0);

  auto state = MakeHardwareState();
  state.joints.position = {11.0, 999.0};
  state.joints.velocity = {44.0, 999.0};
  state.joints.effort = {77.0, 999.0};
  state.joints.acceleration = {12.0, 999.0};

  const auto lowstate =
      translate_hardware_state_to_lowstate(state, MakeJointMapWithDisabledHighestSlot());
  assert(lowstate.result.succeeded());
  assert(lowstate.state.has_value());
  assert(lowstate.state->motor_state()[2].q() == 0.0F);
  assert(lowstate.state->motor_state()[2].dq() == 0.0F);
  assert(lowstate.state->motor_state()[2].tau_est() == 0.0F);
  assert(lowstate.state->motor_state()[2].ddq() == 0.0F);
}

}  // namespace

int main() {
  TestTranslateLowCmdToCommand();
  TestTranslateLowCmdToHardwareCommandThroughJointMap();
  TestTranslateHardwareStateToLowStateThroughJointMap();
  TestRejectsLowCmdWithNonFiniteValues();
  TestRejectsLowCmdWithInvalidCrc();
  TestRejectsStateWithInvalidImuVectorWidths();
  TestDisabledJointSlotsRemainZeroInDdsTranslations();
  return 0;
}
