#include "../../../include/bridge/gamepad/common_gamepad.hpp"

#include <cmath>
#include <cstring>

namespace bridge::gamepad {
namespace {

enum CustomButtonBits : std::uint32_t {
  kCustomMoveToDefaultPos = 1U << 0U,
  kCustomHoldPositionMode = 1U << 1U,
  kCustomCmdReset = 1U << 2U,
  kCustomCycleTask = 1U << 3U,
};

enum MocapButtonBits : std::uint32_t {
  kMocapEmergency = 1U << 0U,
  kMocapMoveToDefaultPos = 1U << 1U,
  kMocapStartPolicy = 1U << 2U,
};

void write_float(std::array<std::uint8_t, 40>* payload,
                 const std::size_t offset,
                 const float value) noexcept {
  std::memcpy(payload->data() + offset, &value, sizeof(value));
}

void write_u32(std::array<std::uint8_t, 40>* payload,
               const std::size_t offset,
               const std::uint32_t value) noexcept {
  std::memcpy(payload->data() + offset, &value, sizeof(value));
}

}  // namespace

void CommonButton::update(const bool state) noexcept {
  on_press = state && !pressed;
  on_release = !state && pressed;
  pressed = state;
}

void CommonGamepad::update_filter() noexcept {
  const auto filter = [this](const float previous, const float raw) {
    const float value = std::fabs(raw) < dead_zone ? 0.0F : raw;
    return previous * (1.0F - smooth) + value * smooth;
  };

  lx = filter(lx, raw_lx);
  ly = filter(ly, raw_ly);
  rx = filter(rx, raw_rx);
  ry = filter(ry, raw_ry);
}

void CommonGamepad::force_zero() noexcept {
  lx = 0.0F;
  ly = 0.0F;
  rx = 0.0F;
  ry = 0.0F;
  lt = 0.0F;
  rt = 0.0F;
  raw_lx = 0.0F;
  raw_ly = 0.0F;
  raw_rx = 0.0F;
  raw_ry = 0.0F;
  raw_lt = 0.0F;
  raw_rt = 0.0F;
}

std::array<std::uint8_t, 40> compress_to_wireless_remote(
    const CommonGamepad& gamepad) noexcept {
  std::array<std::uint8_t, 40> payload{};
  payload[0] = 0xFEU;
  payload[1] = 0xEFU;

  std::uint16_t buttons = 0U;
  const auto set_button = [&buttons](const bool pressed, const unsigned bit) noexcept {
    if (pressed) {
      buttons |= static_cast<std::uint16_t>(1U << bit);
    }
  };

  set_button(gamepad.A.pressed, 0U);
  set_button(gamepad.B.pressed, 1U);
  set_button(gamepad.X.pressed, 2U);
  set_button(gamepad.Y.pressed, 3U);
  set_button(gamepad.up.pressed, 4U);
  set_button(gamepad.right.pressed, 5U);
  set_button(gamepad.down.pressed, 6U);
  set_button(gamepad.left.pressed, 7U);
  set_button(gamepad.R1.pressed || gamepad.RB.pressed, 8U);
  set_button(gamepad.L1.pressed || gamepad.LB.pressed, 9U);
  set_button(gamepad.R2.pressed || gamepad.RT.pressed, 10U);
  set_button(gamepad.L2.pressed || gamepad.LT.pressed, 11U);
  set_button(gamepad.start.pressed, 12U);
  set_button(gamepad.select.pressed || gamepad.back.pressed, 13U);
  set_button(gamepad.emergency_btn.pressed, 14U);
  set_button(gamepad.start_policy_btn.pressed, 15U);

  payload[2] = static_cast<std::uint8_t>(buttons & 0xFFU);
  payload[3] = static_cast<std::uint8_t>((buttons >> 8U) & 0xFFU);

  write_float(&payload, 4U, gamepad.lx);
  write_float(&payload, 8U, gamepad.rx);
  write_float(&payload, 12U, gamepad.ry);
  write_float(&payload, 16U, gamepad.lt);
  write_float(&payload, 20U, gamepad.ly);
  write_float(&payload, 24U, gamepad.rt);

  std::uint32_t custom_buttons = 0U;
  if (gamepad.moveto_default_pos_btn.pressed) {
    custom_buttons |= kCustomMoveToDefaultPos;
  }
  if (gamepad.hold_position_mode_btn.pressed) {
    custom_buttons |= kCustomHoldPositionMode;
  }
  if (gamepad.cmd_reset_btn.pressed) {
    custom_buttons |= kCustomCmdReset;
  }
  if (gamepad.cycle_task_btn.pressed) {
    custom_buttons |= kCustomCycleTask;
  }
  write_u32(&payload, 28U, custom_buttons);

  std::uint32_t mocap_buttons = 0U;
  if (gamepad.mocap_cmd_emergency_btn.pressed) {
    mocap_buttons |= kMocapEmergency;
  }
  if (gamepad.mocap_cmd_moveto_default_pos_btn.pressed) {
    mocap_buttons |= kMocapMoveToDefaultPos;
  }
  if (gamepad.mocap_cmd_start_policy_btn.pressed) {
    mocap_buttons |= kMocapStartPolicy;
  }
  write_u32(&payload, 32U, mocap_buttons);
  return payload;
}

}  // namespace bridge::gamepad
