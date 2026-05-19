#pragma once

#include <array>
#include <cstdint>

namespace bridge::gamepad {

struct CommonButton {
  bool pressed{false};
  bool on_press{false};
  bool on_release{false};

  void update(bool state) noexcept;
};

struct CommonGamepad {
  float smooth{0.15F};
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

  CommonButton A;
  CommonButton B;
  CommonButton X;
  CommonButton Y;
  CommonButton up;
  CommonButton down;
  CommonButton left;
  CommonButton right;
  CommonButton LB;
  CommonButton RB;
  CommonButton LT;
  CommonButton RT;
  CommonButton start;
  CommonButton back;

  CommonButton L1;
  CommonButton R1;
  CommonButton L2;
  CommonButton R2;
  CommonButton select;

  CommonButton emergency_btn;
  CommonButton moveto_default_pos_btn;
  CommonButton start_policy_btn;
  CommonButton hold_position_mode_btn;
  CommonButton cmd_reset_btn;
  CommonButton cycle_task_btn;

  CommonButton mocap_cmd_emergency_btn;
  CommonButton mocap_cmd_moveto_default_pos_btn;
  CommonButton mocap_cmd_start_policy_btn;

  void update_filter() noexcept;
  void force_zero() noexcept;
};

[[nodiscard]] std::array<std::uint8_t, 40> compress_to_wireless_remote(
    const CommonGamepad& gamepad) noexcept;

}  // namespace bridge::gamepad
