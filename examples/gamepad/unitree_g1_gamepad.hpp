#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace bridge::adapters::unitree_g1 {

struct G1Button {
  bool pressed{false};
  bool on_press{false};
  bool on_release{false};

  void update(bool state) noexcept;
};

struct G1Gamepad {
  float smooth{1.0F};
  float dead_zone{0.02F};

  float raw_lx{0.0F};
  float raw_ly{0.0F};
  float raw_rx{0.0F};
  float raw_ry{0.0F};
  float raw_lt{0.0F};
  float raw_rt{0.0F};

  float lx{0.0F};
  float ly{0.0F};
  float rx{0.0F};
  float ry{0.0F};
  float lt{0.0F};
  float rt{0.0F};

  G1Button A;
  G1Button B;
  G1Button X;
  G1Button Y;
  G1Button up;
  G1Button down;
  G1Button left;
  G1Button right;
  G1Button LB;
  G1Button RB;
  G1Button LT;
  G1Button RT;
  G1Button start;
  G1Button back;
  G1Button L1;
  G1Button R1;
  G1Button L2;
  G1Button R2;
  G1Button select;
  G1Button emergency_btn;
  G1Button moveto_default_pos_btn;
  G1Button start_policy_btn;
  G1Button hold_position_mode_btn;
  G1Button cmd_reset_btn;
  G1Button cycle_task_btn;
  G1Button mocap_cmd_emergency_btn;
  G1Button mocap_cmd_moveto_default_pos_btn;
  G1Button mocap_cmd_start_policy_btn;

  void update_filter() noexcept;
  void force_zero() noexcept;
};

void update_gamepad_from_wireless_remote(
    G1Gamepad& gamepad, const std::array<std::uint8_t, 40>& wireless_remote);

[[nodiscard]] std::string format_gamepad_status(const G1Gamepad& gamepad);

[[nodiscard]] std::string format_wireless_remote_gamepad_status(
    G1Gamepad& gamepad, const std::array<std::uint8_t, 40>& wireless_remote);

}  // namespace bridge::adapters::unitree_g1
