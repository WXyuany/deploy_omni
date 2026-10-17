#include <array>
#include <cassert>
#include <cstring>
#include <string>

#include "../../examples/gamepad/unitree_g1_gamepad.hpp"
#include "../../include/bridge/gamepad/common_gamepad.hpp"

namespace {

void write_float(std::array<std::uint8_t, 40>& remote, const std::size_t offset,
                 const float value) {
  std::memcpy(remote.data() + offset, &value, sizeof(value));
}

void TestDecodeUnitreeRemoteLayout() {
  std::array<std::uint8_t, 40> remote{};
  const std::uint16_t buttons =
      (1U << 0U) |   // R1
      (1U << 1U) |   // L1
      (1U << 2U) |   // Start
      (1U << 3U) |   // Select
      (1U << 4U) |   // R2
      (1U << 5U) |   // L2
      (1U << 8U) |   // A
      (1U << 9U) |   // B
      (1U << 10U) |  // X
      (1U << 11U) |  // Y
      (1U << 12U) |  // up
      (1U << 13U) |  // right
      (1U << 14U) |  // down
      (1U << 15U);   // left
  remote[2] = static_cast<std::uint8_t>(buttons & 0xFFU);
  remote[3] = static_cast<std::uint8_t>((buttons >> 8U) & 0xFFU);
  write_float(remote, 4U, 0.5F);
  write_float(remote, 8U, -0.25F);
  write_float(remote, 12U, 0.75F);
  write_float(remote, 16U, 0.6F);
  write_float(remote, 20U, -0.8F);

  bridge::adapters::unitree_g1::G1Gamepad gamepad;
  bridge::adapters::unitree_g1::update_gamepad_from_wireless_remote(gamepad,
                                                                    remote);

  assert(gamepad.A.pressed);
  assert(gamepad.B.pressed);
  assert(gamepad.X.pressed);
  assert(gamepad.Y.pressed);
  assert(gamepad.LB.pressed);
  assert(gamepad.RB.pressed);
  assert(gamepad.LT.pressed);
  assert(gamepad.RT.pressed);
  assert(gamepad.start.pressed);
  assert(gamepad.select.pressed);
  assert(gamepad.up.pressed);
  assert(gamepad.down.pressed);
  assert(gamepad.left.pressed);
  assert(gamepad.right.pressed);
  assert(gamepad.raw_lx == 0.5F);
  assert(gamepad.raw_rx == -0.25F);
  assert(gamepad.raw_ry == 0.75F);
  assert(gamepad.raw_lt == 0.6F);
  assert(gamepad.raw_ly == -0.8F);
  assert(gamepad.raw_rt == 1.0F);
}

void TestDecodeCommonGamepadWirelessCodecLayout() {
  bridge::gamepad::CommonGamepad common;
  common.A.update(true);
  common.B.update(true);
  common.R1.update(true);
  common.L1.update(true);
  common.R2.update(true);
  common.L2.update(true);
  common.start.update(true);
  common.select.update(true);
  common.emergency_btn.update(true);
  common.start_policy_btn.update(true);
  common.lx = 0.5F;
  common.rx = -0.25F;
  common.ry = 0.75F;
  common.lt = 0.6F;
  common.ly = -0.8F;
  common.rt = 0.4F;

  const std::array<std::uint8_t, 40> remote =
      bridge::gamepad::compress_to_wireless_remote(common);

  bridge::adapters::unitree_g1::G1Gamepad gamepad;
  bridge::adapters::unitree_g1::update_gamepad_from_wireless_remote(gamepad,
                                                                    remote);

  assert(gamepad.A.pressed);
  assert(gamepad.B.pressed);
  assert(gamepad.LB.pressed);
  assert(gamepad.RB.pressed);
  assert(gamepad.LT.pressed);
  assert(gamepad.RT.pressed);
  assert(gamepad.start.pressed);
  assert(gamepad.select.pressed);
  assert(gamepad.emergency_btn.pressed);
  assert(gamepad.start_policy_btn.pressed);
  assert(gamepad.raw_lx == 0.5F);
  assert(gamepad.raw_rx == -0.25F);
  assert(gamepad.raw_ry == 0.75F);
  assert(gamepad.raw_lt == 0.6F);
  assert(gamepad.raw_ly == -0.8F);
  assert(gamepad.raw_rt == 0.4F);
}

void TestFormatGamepadStatus() {
  bridge::adapters::unitree_g1::G1Gamepad gamepad;
  gamepad.lx = 0.5F;
  gamepad.ly = -0.8F;
  gamepad.rx = -0.25F;
  gamepad.ry = 0.75F;
  gamepad.lt = 0.6F;
  gamepad.rt = 1.0F;
  gamepad.A.update(true);
  gamepad.start.update(true);
  gamepad.select.update(true);
  gamepad.emergency_btn.update(true);

  const std::string status =
      bridge::adapters::unitree_g1::format_gamepad_status(gamepad);
  assert(status.find("========== Gamepad ==========") != std::string::npos);
  assert(status.find("Axis  lx:0.5 ly:-0.8 rx:-0.25 ry:0.75 lt:0.6 rt:1") !=
         std::string::npos);
  assert(status.find("Buttons A:1") != std::string::npos);
  assert(status.find("start:1") != std::string::npos);
  assert(status.find("select:1") != std::string::npos);
  assert(status.find("Custom Buttons emergency_btn:1") != std::string::npos);
}

void TestFormatWirelessRemoteGamepadStatus() {
  bridge::gamepad::CommonGamepad common;
  common.A.update(true);
  common.start.update(true);
  common.emergency_btn.update(true);
  common.lx = 0.25F;
  common.ly = -0.5F;
  common.rt = 0.75F;

  bridge::adapters::unitree_g1::G1Gamepad gamepad;
  const std::string status =
      bridge::adapters::unitree_g1::format_wireless_remote_gamepad_status(
          gamepad, bridge::gamepad::compress_to_wireless_remote(common));

  assert(status.find("Axis  lx:0.25 ly:-0.5") != std::string::npos);
  assert(status.find("Buttons A:1") != std::string::npos);
  assert(status.find("start:1") != std::string::npos);
  assert(status.find("emergency_btn:1") != std::string::npos);
}

}  // namespace

int main() {
  TestDecodeUnitreeRemoteLayout();
  TestDecodeCommonGamepadWirelessCodecLayout();
  TestFormatGamepadStatus();
  TestFormatWirelessRemoteGamepadStatus();
  return 0;
}
