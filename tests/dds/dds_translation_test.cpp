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
using bridge::dds::translate_bms_sample_to_bms_state;
using bridge::dds::translate_state_to_imu_state;
using bridge::dds::translate_mainboard_sample_to_mainboard_state;
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
  motors[0].mode(7U);
  motors[0].q(1.0F);
  motors[0].dq(3.0F);
  motors[0].tau(5.0F);
  motors[0].kp(7.0F);
  motors[0].kd(9.0F);
  motors[1].mode(0U);
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
  state.tick = 20U;
  state.sample_time_ns = 1000U;
  state.publish_time_ns = 1010U;
  state.mode_pr = 9U;
  state.mode_machine = 5U;
  state.low_state_version = {3U, 4U};
  state.low_state_reserve = {101U, 102U, 103U, 104U};
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
  state.joints.motor_mode = {7U, 8U};
  state.joints.motor_temperature = {{{31, 41}}, {{32, 42}}};
  state.joints.motor_voltage = {51.0, 52.0};
  state.joints.motor_sensor = {{{61U, 71U}}, {{62U, 72U}}};
  state.joints.motor_state = {81U, 82U};
  state.joints.motor_reserve = {{{91U, 101U, 111U, 121U}},
                                {{92U, 102U, 112U, 122U}}};
  state.wireless_remote[0] = 0x11U;
  state.wireless_remote[39] = 0x39U;
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
      .temperature = 55,
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
  assert(translated.command->joints.motor_mode.size() == 35U);
  assert(translated.command->joints.motor_mode[0] == 7U);
  assert(translated.command->joints.motor_mode[1] == 0U);
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
  assert(translated.command->joints.motor_mode.size() == 2U);
  assert(translated.command->joints.motor_mode[0] == 0U);
  assert(translated.command->joints.motor_mode[1] == 7U);
}

void TestTranslateHardwareStateToLowStateThroughJointMap() {
  const auto translated =
      translate_hardware_state_to_lowstate(MakeHardwareState(), MakeSwappingJointMap());
  assert(translated.result.succeeded());
  assert(translated.state.has_value());
  assert(translated.state->tick() == 20U);
  assert(translated.state->mode_pr() == 9U);
  assert(translated.state->mode_machine() == 5U);
  assert(translated.state->version()[0] == 3U);
  assert(translated.state->version()[1] == 4U);
  assert(translated.state->reserve()[2] == 103U);
  assert(translated.state->motor_state()[0].mode() == 8U);
  assert(translated.state->motor_state()[1].mode() == 7U);
  assert(translated.state->motor_state()[0].q() == 22.0F);
  assert(translated.state->motor_state()[1].q() == 11.0F);
  assert(translated.state->motor_state()[0].tau_est() == 66.0F);
  assert(translated.state->motor_state()[1].tau_est() == 55.0F);
  assert(translated.state->motor_state()[0].temperature()[0] == 32);
  assert(translated.state->motor_state()[1].temperature()[1] == 41);
  assert(translated.state->motor_state()[0].vol() == 52.0F);
  assert(translated.state->motor_state()[1].sensor()[1] == 71U);
  assert(translated.state->motor_state()[0].motorstate() == 82U);
  assert(translated.state->motor_state()[1].reserve()[3] == 121U);
  assert(translated.state->wireless_remote()[0] == 0x11U);
  assert(translated.state->wireless_remote()[39] == 0x39U);
  assert(translated.state->imu_state().quaternion()[0] == 0.5F);
  assert(translated.state->imu_state().quaternion()[1] == 0.2F);
  assert(translated.state->imu_state().quaternion()[2] == 0.3F);
  assert(translated.state->imu_state().quaternion()[3] == 0.4F);
  assert(translated.state->imu_state().rpy()[0] == 0.6F);
  assert(translated.state->imu_state().rpy()[1] == 0.7F);
  assert(translated.state->imu_state().rpy()[2] == 0.8F);
  assert(translated.state->imu_state().temperature() == 55);
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

void TestAcceptsLowCmdWithExternalCrcWithoutValidation() {
  auto message = MakeLowCmd();
  const std::uint32_t external_crc = message.crc() + 1U;
  message.crc(external_crc);

  const auto translated = translate_lowcmd_to_command(message, 400U);
  assert(translated.result.succeeded());
  assert(translated.command.has_value());
  assert(translated.command->low_cmd_crc.has_value());
  assert(*translated.command->low_cmd_crc == external_crc);
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

void TestRejectsStateWithMismatchedDiagnosticVectorWidths() {
  auto state = MakeHardwareState();
  state.joints.motor_voltage = {1.0};

  const auto translated =
      translate_hardware_state_to_lowstate(state, MakeSwappingJointMap());
  assert(!translated.result.succeeded());
  assert(!translated.state.has_value());
}

void TestDiagnosticOnlyStateDefaultsMissingMotionFieldsToZero() {
  StateFrame state;
  state.valid = true;
  state.joints.motor_mode = {7U, 8U};
  state.joints.motor_temperature = {{{31, 41}}, {{32, 42}}};
  state.joints.motor_voltage = {51.0, 52.0};
  state.joints.motor_sensor = {{{61U, 71U}}, {{62U, 72U}}};
  state.joints.motor_state = {81U, 82U};
  state.joints.motor_reserve = {{{91U, 101U, 111U, 121U}},
                                {{92U, 102U, 112U, 122U}}};

  const auto translated =
      translate_hardware_state_to_lowstate(state, MakeSwappingJointMap());

  assert(translated.result.succeeded());
  assert(translated.state.has_value());
  assert(translated.state->motor_state()[0].mode() == 8U);
  assert(translated.state->motor_state()[0].q() == 0.0F);
  assert(translated.state->motor_state()[0].dq() == 0.0F);
  assert(translated.state->motor_state()[0].ddq() == 0.0F);
  assert(translated.state->motor_state()[0].tau_est() == 0.0F);
  assert(translated.state->motor_state()[0].temperature()[1] == 42);
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
  assert(command.command->joints.position.size() == 3U);
  assert(command.command->joints.position[0] == 1.0);
  assert(command.command->joints.position[1] == 9.0);
  assert(command.command->joints.position[2] == 0.0);

  auto state = MakeHardwareState();
  state.joints.position = {11.0, 999.0, 123.0};
  state.joints.velocity = {44.0, 999.0, 123.0};
  state.joints.effort = {77.0, 999.0, 123.0};
  state.joints.acceleration = {12.0, 999.0, 123.0};
  state.joints.motor_mode = {7U, 8U, 9U};
  state.joints.motor_temperature = {{{31, 41}}, {{32, 42}}, {{33, 43}}};
  state.joints.motor_voltage = {51.0, 52.0, 53.0};
  state.joints.motor_sensor = {{{61U, 71U}}, {{62U, 72U}}, {{63U, 73U}}};
  state.joints.motor_state = {81U, 82U, 83U};
  state.joints.motor_reserve = {{{91U, 101U, 111U, 121U}},
                                {{92U, 102U, 112U, 122U}},
                                {{93U, 103U, 113U, 123U}}};

  const auto lowstate =
      translate_hardware_state_to_lowstate(state, MakeJointMapWithDisabledHighestSlot());
  assert(lowstate.result.succeeded());
  assert(lowstate.state.has_value());
  assert(lowstate.state->motor_state()[2].q() == 0.0F);
  assert(lowstate.state->motor_state()[2].dq() == 0.0F);
  assert(lowstate.state->motor_state()[2].tau_est() == 0.0F);
  assert(lowstate.state->motor_state()[2].ddq() == 0.0F);
  assert(lowstate.state->motor_state()[2].mode() == 0U);
  assert(lowstate.state->motor_state()[2].temperature()[0] == 0);
  assert(lowstate.state->motor_state()[2].vol() == 0.0F);
  assert(lowstate.state->motor_state()[2].sensor()[0] == 0U);
  assert(lowstate.state->motor_state()[2].motorstate() == 0U);
  assert(lowstate.state->motor_state()[2].reserve()[3] == 0U);
}

void TestTranslatesStateToStandaloneImuState() {
  const auto translated = translate_state_to_imu_state(MakeHardwareState());

  assert(translated.result.succeeded());
  assert(translated.state.has_value());
  assert(translated.state->quaternion()[0] == 0.5F);
  assert(translated.state->quaternion()[1] == 0.2F);
  assert(translated.state->gyroscope()[2] == 0.3F);
  assert(translated.state->accelerometer()[0] == 9.1F);
  assert(translated.state->rpy()[1] == 0.7F);
  assert(translated.state->temperature() == 55);
}

void TestTranslatesOptionalBmsAndMainboardStates() {
  bridge::core::BmsStateSample bms;
  bms.version_high = 1U;
  bms.version_low = 2U;
  bms.cell_voltage[0] = 3300U;
  bms.bms_voltage[1] = 48000U;
  bms.current = -123;
  bms.soc = 88U;
  bms.temperature[2] = 35;
  bms.bms_state[3] = 44U;

  bridge::core::MainBoardStateSample mainboard;
  mainboard.fan_state[0] = 9U;
  mainboard.temperature[1] = 42;
  mainboard.value[2] = 3.5F;
  mainboard.state[3] = 4U;

  const auto translated_bms = translate_bms_sample_to_bms_state(bms);
  assert(translated_bms.result.succeeded());
  assert(translated_bms.state.has_value());
  assert(translated_bms.state->version_high() == 1U);
  assert(translated_bms.state->cell_vol()[0] == 3300U);
  assert(translated_bms.state->bmsvoltage()[1] == 48000U);
  assert(translated_bms.state->current() == -123);
  assert(translated_bms.state->soc() == 88U);
  assert(translated_bms.state->temperature()[2] == 35);
  assert(translated_bms.state->bmsstate()[3] == 44U);

  const auto translated_mainboard =
      translate_mainboard_sample_to_mainboard_state(mainboard);
  assert(translated_mainboard.result.succeeded());
  assert(translated_mainboard.state.has_value());
  assert(translated_mainboard.state->fan_state()[0] == 9U);
  assert(translated_mainboard.state->temperature()[1] == 42);
  assert(translated_mainboard.state->value()[2] == 3.5F);
  assert(translated_mainboard.state->state()[3] == 4U);
}

}  // namespace

int main() {
  TestTranslateLowCmdToCommand();
  TestTranslateLowCmdToHardwareCommandThroughJointMap();
  TestTranslateHardwareStateToLowStateThroughJointMap();
  TestRejectsLowCmdWithNonFiniteValues();
  TestAcceptsLowCmdWithExternalCrcWithoutValidation();
  TestRejectsStateWithInvalidImuVectorWidths();
  TestRejectsStateWithMismatchedDiagnosticVectorWidths();
  TestDiagnosticOnlyStateDefaultsMissingMotionFieldsToZero();
  TestDisabledJointSlotsRemainZeroInDdsTranslations();
  TestTranslatesStateToStandaloneImuState();
  TestTranslatesOptionalBmsAndMainboardStates();
  return 0;
}
