#include "unitree_g1_gamepad.hpp"

#include <cmath>
#include <cstring>
#include <sstream>

namespace bridge::adapters::unitree_g1 {
namespace {

[[nodiscard]] bool button_pressed(const std::uint16_t buttons,
                                  const unsigned bit) noexcept {
  return (buttons & static_cast<std::uint16_t>(1U << bit)) != 0U;
}

[[nodiscard]] float read_float(const std::array<std::uint8_t, 40>& data,
                               const std::size_t offset) noexcept {
  float value = 0.0F;
  std::memcpy(&value, data.data() + offset, sizeof(value));
  return value;
}

[[nodiscard]] std::uint32_t read_u32(const std::array<std::uint8_t, 40>& data,
                                     const std::size_t offset) noexcept {
  std::uint32_t value = 0U;
  std::memcpy(&value, data.data() + offset, sizeof(value));
  return value;
}

[[nodiscard]] bool is_common_gamepad_codec(
    const std::array<std::uint8_t, 40>& wireless_remote) noexcept {
  return wireless_remote[0] == 0xFEU && wireless_remote[1] == 0xEFU;
}

void update_from_common_gamepad_codec(
    G1Gamepad& gamepad, const std::array<std::uint8_t, 40>& wireless_remote) {
  const std::uint16_t buttons =
      static_cast<std::uint16_t>(wireless_remote[2]) |
      static_cast<std::uint16_t>(static_cast<std::uint16_t>(wireless_remote[3]) << 8U);

  gamepad.A.update(button_pressed(buttons, 0U));
  gamepad.B.update(button_pressed(buttons, 1U));
  gamepad.X.update(button_pressed(buttons, 2U));
  gamepad.Y.update(button_pressed(buttons, 3U));
  gamepad.up.update(button_pressed(buttons, 4U));
  gamepad.right.update(button_pressed(buttons, 5U));
  gamepad.down.update(button_pressed(buttons, 6U));
  gamepad.left.update(button_pressed(buttons, 7U));
  gamepad.RB.update(button_pressed(buttons, 8U));
  gamepad.LB.update(button_pressed(buttons, 9U));
  gamepad.RT.update(button_pressed(buttons, 10U));
  gamepad.LT.update(button_pressed(buttons, 11U));
  gamepad.start.update(button_pressed(buttons, 12U));
  gamepad.select.update(button_pressed(buttons, 13U));
  gamepad.back.update(gamepad.select.pressed);
  gamepad.emergency_btn.update(button_pressed(buttons, 14U));
  gamepad.start_policy_btn.update(button_pressed(buttons, 15U));

  gamepad.R1.update(gamepad.RB.pressed);
  gamepad.L1.update(gamepad.LB.pressed);
  gamepad.R2.update(gamepad.RT.pressed);
  gamepad.L2.update(gamepad.LT.pressed);

  gamepad.raw_lx = read_float(wireless_remote, 4U);
  gamepad.raw_rx = read_float(wireless_remote, 8U);
  gamepad.raw_ry = read_float(wireless_remote, 12U);
  gamepad.raw_lt = read_float(wireless_remote, 16U);
  gamepad.raw_ly = read_float(wireless_remote, 20U);
  gamepad.raw_rt = read_float(wireless_remote, 24U);

  const std::uint32_t custom_buttons = read_u32(wireless_remote, 28U);
  gamepad.moveto_default_pos_btn.update((custom_buttons & (1U << 0U)) != 0U);
  gamepad.hold_position_mode_btn.update((custom_buttons & (1U << 1U)) != 0U);
  gamepad.cmd_reset_btn.update((custom_buttons & (1U << 2U)) != 0U);
  gamepad.cycle_task_btn.update((custom_buttons & (1U << 3U)) != 0U);

  const std::uint32_t mocap_buttons = read_u32(wireless_remote, 32U);
  gamepad.mocap_cmd_emergency_btn.update((mocap_buttons & (1U << 0U)) != 0U);
  gamepad.mocap_cmd_moveto_default_pos_btn.update((mocap_buttons & (1U << 1U)) != 0U);
  gamepad.mocap_cmd_start_policy_btn.update((mocap_buttons & (1U << 2U)) != 0U);
  gamepad.update_filter();
}

}  // namespace

void G1Button::update(const bool state) noexcept {
  on_press = state && !pressed;
  on_release = !state && pressed;
  pressed = state;
}

void G1Gamepad::update_filter() noexcept {
  const auto filter = [this](const float previous, const float raw) {
    const float value = std::fabs(raw) < dead_zone ? 0.0F : raw;
    return previous * (1.0F - smooth) + value * smooth;
  };

  lx = filter(lx, raw_lx);
  ly = filter(ly, raw_ly);
  rx = filter(rx, raw_rx);
  ry = filter(ry, raw_ry);
  lt = filter(lt, raw_lt);
  rt = filter(rt, raw_rt);
}

void G1Gamepad::force_zero() noexcept {
  lx = ly = rx = ry = lt = rt = 0.0F;
  raw_lx = raw_ly = raw_rx = raw_ry = raw_lt = raw_rt = 0.0F;
}

void update_gamepad_from_wireless_remote(
    G1Gamepad& gamepad, const std::array<std::uint8_t, 40>& wireless_remote) {
  if (is_common_gamepad_codec(wireless_remote)) {
    update_from_common_gamepad_codec(gamepad, wireless_remote);
    return;
  }

  const std::uint16_t buttons =
      static_cast<std::uint16_t>(wireless_remote[2]) |
      static_cast<std::uint16_t>(static_cast<std::uint16_t>(wireless_remote[3]) << 8U);

  gamepad.RB.update(button_pressed(buttons, 0U));
  gamepad.LB.update(button_pressed(buttons, 1U));
  gamepad.start.update(button_pressed(buttons, 2U));
  gamepad.select.update(button_pressed(buttons, 3U));
  gamepad.back.update(gamepad.select.pressed);
  gamepad.RT.update(button_pressed(buttons, 4U));
  gamepad.LT.update(button_pressed(buttons, 5U));
  gamepad.A.update(button_pressed(buttons, 8U));
  gamepad.B.update(button_pressed(buttons, 9U));
  gamepad.X.update(button_pressed(buttons, 10U));
  gamepad.Y.update(button_pressed(buttons, 11U));
  gamepad.up.update(button_pressed(buttons, 12U));
  gamepad.right.update(button_pressed(buttons, 13U));
  gamepad.down.update(button_pressed(buttons, 14U));
  gamepad.left.update(button_pressed(buttons, 15U));

  gamepad.R1.update(gamepad.RB.pressed);
  gamepad.L1.update(gamepad.LB.pressed);
  gamepad.R2.update(gamepad.RT.pressed);
  gamepad.L2.update(gamepad.LT.pressed);

  gamepad.raw_lx = read_float(wireless_remote, 4U);
  gamepad.raw_rx = read_float(wireless_remote, 8U);
  gamepad.raw_ry = read_float(wireless_remote, 12U);
  gamepad.raw_lt = read_float(wireless_remote, 16U);
  gamepad.raw_ly = read_float(wireless_remote, 20U);
  gamepad.raw_rt = gamepad.RT.pressed ? 1.0F : 0.0F;
  gamepad.update_filter();
}

std::string format_gamepad_status(const G1Gamepad& gamepad) {
  std::ostringstream out;
  out << "[state_receiver.printGamepadStatus] ========== Gamepad =========="
      << '\n';
  out << "Axis  lx:" << gamepad.lx << " ly:" << gamepad.ly
      << " rx:" << gamepad.rx << " ry:" << gamepad.ry
      << " lt:" << gamepad.lt << " rt:" << gamepad.rt << '\n';
  out << "Buttons A:" << gamepad.A.pressed << " B:" << gamepad.B.pressed
      << " X:" << gamepad.X.pressed << " Y:" << gamepad.Y.pressed
      << " LB:" << gamepad.LB.pressed << " RB:" << gamepad.RB.pressed
      << " LT:" << gamepad.LT.pressed << " RT:" << gamepad.RT.pressed
      << " start:" << gamepad.start.pressed
      << " back:" << gamepad.back.pressed
      << " select:" << gamepad.select.pressed << " up:" << gamepad.up.pressed
      << " down:" << gamepad.down.pressed << " left:" << gamepad.left.pressed
      << " right:" << gamepad.right.pressed << '\n';
  out << "Custom Buttons emergency_btn:" << gamepad.emergency_btn.pressed
      << " moveto_default_pos_btn:" << gamepad.moveto_default_pos_btn.pressed
      << " start_policy_btn:" << gamepad.start_policy_btn.pressed
      << " hold_position_mode_btn:" << gamepad.hold_position_mode_btn.pressed
      << " cmd_reset_btn:" << gamepad.cmd_reset_btn.pressed << '\n';
  out << "[state_receiver.printGamepadStatus] ==========================================";
  return out.str();
}

std::string format_wireless_remote_gamepad_status(
    G1Gamepad& gamepad, const std::array<std::uint8_t, 40>& wireless_remote) {
  update_gamepad_from_wireless_remote(gamepad, wireless_remote);
  return format_gamepad_status(gamepad);
}

}  // namespace bridge::adapters::unitree_g1
