#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>

#include "../../include/bridge/gamepad/common_gamepad.hpp"

namespace {

float ReadFloat(const std::array<std::uint8_t, 40>& data,
                const std::size_t offset) {
  float value = 0.0F;
  std::memcpy(&value, data.data() + offset, sizeof(value));
  return value;
}

std::uint32_t ReadU32(const std::array<std::uint8_t, 40>& data,
                      const std::size_t offset) {
  std::uint32_t value = 0U;
  std::memcpy(&value, data.data() + offset, sizeof(value));
  return value;
}

void TestButtonEdges() {
  bridge::gamepad::CommonButton button;

  button.update(true);
  assert(button.pressed);
  assert(button.on_press);
  assert(!button.on_release);

  button.update(true);
  assert(button.pressed);
  assert(!button.on_press);
  assert(!button.on_release);

  button.update(false);
  assert(!button.pressed);
  assert(!button.on_press);
  assert(button.on_release);
}

void TestFilterAndDeadZone() {
  bridge::gamepad::CommonGamepad gamepad;
  gamepad.smooth = 0.5F;
  gamepad.dead_zone = 0.1F;
  gamepad.raw_lx = 1.0F;
  gamepad.raw_ly = 0.05F;

  gamepad.update_filter();

  assert(gamepad.lx == 0.5F);
  assert(gamepad.ly == 0.0F);
}

void TestForceZeroClearsAxes() {
  bridge::gamepad::CommonGamepad gamepad;
  gamepad.raw_lx = 1.0F;
  gamepad.raw_ly = -1.0F;
  gamepad.lx = 0.5F;
  gamepad.ly = -0.5F;
  gamepad.lt = 0.5F;
  gamepad.rt = 0.25F;
  gamepad.A.update(true);

  gamepad.force_zero();

  assert(gamepad.raw_lx == 0.0F);
  assert(gamepad.raw_ly == 0.0F);
  assert(gamepad.lx == 0.0F);
  assert(gamepad.ly == 0.0F);
  assert(gamepad.lt == 0.0F);
  assert(gamepad.rt == 0.0F);
  assert(gamepad.A.pressed);
}

void TestCompressionUsesWirelessRemoteCodecLayout() {
  bridge::gamepad::CommonGamepad gamepad;
  gamepad.lx = 0.1F;
  gamepad.rx = 0.2F;
  gamepad.ry = -0.3F;
  gamepad.lt = 0.4F;
  gamepad.ly = -0.5F;
  gamepad.rt = 0.6F;
  gamepad.A.update(true);
  gamepad.B.update(true);
  gamepad.X.update(true);
  gamepad.Y.update(true);
  gamepad.up.update(true);
  gamepad.right.update(true);
  gamepad.down.update(true);
  gamepad.left.update(true);
  gamepad.R1.update(true);
  gamepad.L1.update(true);
  gamepad.R2.update(true);
  gamepad.L2.update(true);
  gamepad.start.update(true);
  gamepad.select.update(true);
  gamepad.emergency_btn.update(true);
  gamepad.start_policy_btn.update(true);
  gamepad.moveto_default_pos_btn.update(true);
  gamepad.hold_position_mode_btn.update(true);
  gamepad.cmd_reset_btn.update(true);
  gamepad.cycle_task_btn.update(true);
  gamepad.mocap_cmd_emergency_btn.update(true);
  gamepad.mocap_cmd_moveto_default_pos_btn.update(true);
  gamepad.mocap_cmd_start_policy_btn.update(true);

  const std::array<std::uint8_t, 40> first =
      bridge::gamepad::compress_to_wireless_remote(gamepad);
  const std::array<std::uint8_t, 40> second =
      bridge::gamepad::compress_to_wireless_remote(gamepad);

  assert(first == second);
  assert(first.size() == 40U);
  assert(first[0] == 0xFEU);
  assert(first[1] == 0xEFU);
  const std::uint16_t buttons =
      static_cast<std::uint16_t>(first[2]) |
      static_cast<std::uint16_t>(static_cast<std::uint16_t>(first[3]) << 8U);
  assert(buttons == 0xFFFFU);
  assert(ReadFloat(first, 4U) == 0.1F);
  assert(ReadFloat(first, 8U) == 0.2F);
  assert(ReadFloat(first, 12U) == -0.3F);
  assert(ReadFloat(first, 16U) == 0.4F);
  assert(ReadFloat(first, 20U) == -0.5F);
  assert(ReadFloat(first, 24U) == 0.6F);
  assert(ReadU32(first, 28U) == 0x0FU);
  assert(ReadU32(first, 32U) == 0x07U);
  assert(ReadU32(first, 36U) == 0U);
}

void TestCompressionTreatsAliasesAsCanonicalButtons() {
  bridge::gamepad::CommonGamepad alias_gamepad;
  alias_gamepad.RB.update(true);
  alias_gamepad.LB.update(true);
  alias_gamepad.RT.update(true);
  alias_gamepad.LT.update(true);
  alias_gamepad.back.update(true);

  bridge::gamepad::CommonGamepad canonical_gamepad;
  canonical_gamepad.R1.update(true);
  canonical_gamepad.L1.update(true);
  canonical_gamepad.R2.update(true);
  canonical_gamepad.L2.update(true);
  canonical_gamepad.select.update(true);

  const std::array<std::uint8_t, 40> alias_encoded =
      bridge::gamepad::compress_to_wireless_remote(alias_gamepad);
  const std::array<std::uint8_t, 40> canonical_encoded =
      bridge::gamepad::compress_to_wireless_remote(canonical_gamepad);

  assert(alias_encoded == canonical_encoded);
}

}  // namespace

int main() {
  TestButtonEdges();
  TestFilterAndDeadZone();
  TestForceZeroClearsAxes();
  TestCompressionUsesWirelessRemoteCodecLayout();
  TestCompressionTreatsAliasesAsCanonicalButtons();
  return 0;
}
