#include <cassert>

#include "../../include/bridge/adapters/unitree_g1/unitree_g1_low_level_protocol.hpp"
#include <unitree/dds_wrapper/common/crc.h>

namespace {

using bridge::adapters::unitree_g1::build_low_cmd_from_command;
using bridge::adapters::unitree_g1::build_safe_low_cmd;
using bridge::adapters::unitree_g1::validate_low_state_crc;
using bridge::core::CommandFrame;
using unitree_hg::msg::dds_::LowState_;

CommandFrame MakePackedCommand() {
  CommandFrame command;
  command.valid = true;
  command.mode_pr = 3;
  command.mode_machine = 17;
  command.joints.position.assign(29, 1.25);
  command.joints.velocity.assign(29, 2.25);
  command.joints.effort.assign(29, 3.25);
  command.joints.stiffness.assign(29, 4.25);
  command.joints.damping.assign(29, 5.25);
  return command;
}

void TestBuildLowCmdPacksActiveSlotsAndCrc() {
  const auto command = MakePackedCommand();
  const auto low_cmd = build_low_cmd_from_command(command, 29, 9);
  assert(low_cmd.mode_pr() == 3);
  assert(low_cmd.mode_machine() == 9);

  for (std::size_t index = 0; index < 29; ++index) {
    const auto& motor = low_cmd.motor_cmd().at(index);
    assert(motor.mode() == 1);
    assert(motor.q() == 1.25F);
    assert(motor.dq() == 2.25F);
    assert(motor.tau() == 3.25F);
    assert(motor.kp() == 4.25F);
    assert(motor.kd() == 5.25F);
  }

  for (std::size_t index = 29; index < low_cmd.motor_cmd().size(); ++index) {
    const auto& motor = low_cmd.motor_cmd().at(index);
    assert(motor.mode() == 0);
    assert(motor.q() == 0.0F);
    assert(motor.tau() == 0.0F);
  }

  assert(low_cmd.crc() != 0U);
}

void TestBuildSafeLowCmdZerosAllSlots() {
  const auto low_cmd = build_safe_low_cmd(29, 11);
  assert(low_cmd.mode_pr() == 0);
  assert(low_cmd.mode_machine() == 11);
  for (const auto& motor : low_cmd.motor_cmd()) {
    assert(motor.mode() == 0);
    assert(motor.q() == 0.0F);
    assert(motor.dq() == 0.0F);
    assert(motor.tau() == 0.0F);
    assert(motor.kp() == 0.0F);
    assert(motor.kd() == 0.0F);
  }
  assert(low_cmd.crc() != 0U);
}

void TestBuildLowCmdTreatsMissingOptionalBlocksAsZero() {
  CommandFrame command;
  command.valid = true;
  command.mode_pr = 2;
  command.joints.position.assign(29, 1.0);
  const auto low_cmd = build_low_cmd_from_command(command, 29, 4);
  for (std::size_t index = 0; index < 29; ++index) {
    const auto& motor = low_cmd.motor_cmd().at(index);
    assert(motor.mode() == 1);
    assert(motor.q() == 1.0F);
    assert(motor.dq() == 0.0F);
    assert(motor.tau() == 0.0F);
    assert(motor.kp() == 0.0F);
    assert(motor.kd() == 0.0F);
  }
}

void TestBuildLowCmdTreatsMissingPositionBlockAsZero() {
  CommandFrame command;
  command.valid = true;
  command.mode_pr = 4;
  command.joints.velocity.assign(29, 2.0);
  command.joints.effort.assign(29, 3.0);
  const auto low_cmd = build_low_cmd_from_command(command, 29, 6);
  for (std::size_t index = 0; index < 29; ++index) {
    const auto& motor = low_cmd.motor_cmd().at(index);
    assert(motor.mode() == 1);
    assert(motor.q() == 0.0F);
    assert(motor.dq() == 2.0F);
    assert(motor.tau() == 3.0F);
  }
}

void TestValidateLowStateCrcAcceptsWellFormedState() {
  LowState_ state;
  state.mode_pr() = 1;
  state.mode_machine() = 2;
  state.crc() = crc32_core(reinterpret_cast<uint32_t*>(&state), (sizeof(state) >> 2) - 1);
  assert(validate_low_state_crc(state));
  state.mode_machine() = 3;
  assert(!validate_low_state_crc(state));
}

}

int main() {
  TestBuildLowCmdPacksActiveSlotsAndCrc();
  TestBuildSafeLowCmdZerosAllSlots();
  TestBuildLowCmdTreatsMissingOptionalBlocksAsZero();
  TestBuildLowCmdTreatsMissingPositionBlockAsZero();
  TestValidateLowStateCrcAcceptsWellFormedState();
  return 0;
}
