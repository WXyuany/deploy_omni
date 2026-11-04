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

[[nodiscard]] float read_float(const std::array<std::uint8_t, 40>& payload,
                               const std::size_t offset) noexcept {
  float value = 0.0F;
  std::memcpy(&value, payload.data() + offset, sizeof(value));
  return value;
}

[[nodiscard]] std::uint32_t read_u32(
    const std::array<std::uint8_t, 40>& payload,
    const std::size_t offset) noexcept {
  std::uint32_t value = 0U;
  std::memcpy(&value, payload.data() + offset, sizeof(value));
  return value;
}

void update_button_from_bit(CommonButton* button, const std::uint32_t bits,
                            const unsigned bit) noexcept {
  button->update((bits & (1U << bit)) != 0U);
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

bool decompress_from_wireless_remote(
    const std::array<std::uint8_t, 40>& payload,
    CommonGamepad* const gamepad) noexcept {
  if (gamepad == nullptr || payload[0] != 0xFEU || payload[1] != 0xEFU) {
    return false;
  }

  const std::uint32_t buttons = static_cast<std::uint32_t>(payload[2]) |
                                (static_cast<std::uint32_t>(payload[3]) << 8U);
  update_button_from_bit(&gamepad->A, buttons, 0U);
  update_button_from_bit(&gamepad->B, buttons, 1U);
  update_button_from_bit(&gamepad->X, buttons, 2U);
  update_button_from_bit(&gamepad->Y, buttons, 3U);
  update_button_from_bit(&gamepad->up, buttons, 4U);
  update_button_from_bit(&gamepad->right, buttons, 5U);
  update_button_from_bit(&gamepad->down, buttons, 6U);
  update_button_from_bit(&gamepad->left, buttons, 7U);
  update_button_from_bit(&gamepad->R1, buttons, 8U);
  gamepad->RB.update(gamepad->R1.pressed);
  update_button_from_bit(&gamepad->L1, buttons, 9U);
  gamepad->LB.update(gamepad->L1.pressed);
  update_button_from_bit(&gamepad->R2, buttons, 10U);
  gamepad->RT.update(gamepad->R2.pressed);
  update_button_from_bit(&gamepad->L2, buttons, 11U);
  gamepad->LT.update(gamepad->L2.pressed);
  update_button_from_bit(&gamepad->start, buttons, 12U);
  update_button_from_bit(&gamepad->select, buttons, 13U);
  gamepad->back.update(gamepad->select.pressed);
  update_button_from_bit(&gamepad->emergency_btn, buttons, 14U);
  update_button_from_bit(&gamepad->start_policy_btn, buttons, 15U);

  gamepad->lx = gamepad->raw_lx = read_float(payload, 4U);
  gamepad->rx = gamepad->raw_rx = read_float(payload, 8U);
  gamepad->ry = gamepad->raw_ry = read_float(payload, 12U);
  gamepad->lt = gamepad->raw_lt = read_float(payload, 16U);
  gamepad->ly = gamepad->raw_ly = read_float(payload, 20U);
  gamepad->rt = gamepad->raw_rt = read_float(payload, 24U);

  const std::uint32_t custom = read_u32(payload, 28U);
  update_button_from_bit(&gamepad->moveto_default_pos_btn, custom, 0U);
  update_button_from_bit(&gamepad->hold_position_mode_btn, custom, 1U);
  update_button_from_bit(&gamepad->cmd_reset_btn, custom, 2U);
  update_button_from_bit(&gamepad->cycle_task_btn, custom, 3U);

  const std::uint32_t mocap = read_u32(payload, 32U);
  update_button_from_bit(&gamepad->mocap_cmd_emergency_btn, mocap, 0U);
  update_button_from_bit(&gamepad->mocap_cmd_moveto_default_pos_btn, mocap, 1U);
  update_button_from_bit(&gamepad->mocap_cmd_start_policy_btn, mocap, 2U);
  return true;
}

}  // namespace bridge::gamepad
