#include "../../../../include/bridge/adapters/unitree_g1/unitree_g1_low_level_protocol.hpp"

#include <algorithm>

#include <unitree/dds_wrapper/common/crc.h>

namespace bridge::adapters::unitree_g1 {
namespace {

constexpr std::size_t kUnitreeHgMotorSlotCount = 35;

void apply_crc(unitree_hg::msg::dds_::LowCmd_& command) {
  command.crc() = crc32_core(reinterpret_cast<uint32_t*>(&command),
                             (sizeof(command) >> 2) - 1);
}

void apply_command_crc(unitree_hg::msg::dds_::LowCmd_& low_cmd,
                       const bridge::core::CommandFrame& command) {
  if (command.low_cmd_crc.has_value()) {
    low_cmd.crc(*command.low_cmd_crc);
    return;
  }
  apply_crc(low_cmd);
}

void zero_motor_cmd(unitree_hg::msg::dds_::MotorCmd_& motor) {
  motor.mode() = 0;
  motor.q() = 0.0F;
  motor.dq() = 0.0F;
  motor.tau() = 0.0F;
  motor.kp() = 0.0F;
  motor.kd() = 0.0F;
  motor.reserve() = 0;
}

}  // namespace

bool validate_low_state_crc(const unitree_hg::msg::dds_::LowState_& state) {
  auto copy = state;
  return copy.crc() == crc32_core(reinterpret_cast<uint32_t*>(&copy),
                                  (sizeof(copy) >> 2) - 1);
}

unitree_hg::msg::dds_::LowCmd_ build_low_cmd_from_command(
    const bridge::core::CommandFrame& command, const std::size_t hardware_slot_count,
    const std::uint8_t mode_machine) {
  unitree_hg::msg::dds_::LowCmd_ low_cmd;
  low_cmd.mode_pr() = command.mode_pr;
  low_cmd.mode_machine() = mode_machine;

  for (auto& motor : low_cmd.motor_cmd()) {
    zero_motor_cmd(motor);
  }

  const std::size_t width = std::min<std::size_t>(
      std::min<std::size_t>(hardware_slot_count, command.joints.width()),
      kUnitreeHgMotorSlotCount);
  const bool has_valid_motor_modes =
      command.joints.motor_mode.empty() || command.joints.motor_mode.size() >= width;
  if (!has_valid_motor_modes) {
    apply_command_crc(low_cmd, command);
    return low_cmd;
  }

  for (std::size_t index = 0; index < width; ++index) {
    auto& motor = low_cmd.motor_cmd().at(index);
    motor.mode() = command.joints.motor_mode.empty()
                       ? 1U
                       : command.joints.motor_mode.at(index);
    motor.q() = command.joints.position.empty()
                    ? 0.0F
                    : static_cast<float>(command.joints.position.at(index));
    motor.dq() = command.joints.velocity.empty()
                     ? 0.0F
                     : static_cast<float>(command.joints.velocity.at(index));
    motor.tau() = command.joints.effort.empty()
                      ? 0.0F
                      : static_cast<float>(command.joints.effort.at(index));
    motor.kp() = command.joints.stiffness.empty()
                     ? 0.0F
                     : static_cast<float>(command.joints.stiffness.at(index));
    motor.kd() = command.joints.damping.empty()
                     ? 0.0F
                     : static_cast<float>(command.joints.damping.at(index));
  }

  apply_command_crc(low_cmd, command);
  return low_cmd;
}

unitree_hg::msg::dds_::LowCmd_ build_safe_low_cmd(
    const std::size_t hardware_slot_count, const std::uint8_t mode_machine) {
  (void)hardware_slot_count;
  unitree_hg::msg::dds_::LowCmd_ low_cmd;
  low_cmd.mode_pr() = 0;
  low_cmd.mode_machine() = mode_machine;
  for (auto& motor : low_cmd.motor_cmd()) {
    zero_motor_cmd(motor);
  }
  apply_crc(low_cmd);
  return low_cmd;
}

}  // namespace bridge::adapters::unitree_g1
