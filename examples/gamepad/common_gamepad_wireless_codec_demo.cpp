#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string_view>

#include "compat/common_gamepad_wireless_codec.h"
#include "bridge/gamepad/common_gamepad.hpp"

namespace {

void print_button(std::string_view name, const bridge::gamepad::CommonButton& button) {
  std::cout << ' ' << name << ':' << button.pressed;
}

void print_gamepad(std::string_view label, const bridge::gamepad::CommonGamepad& gamepad) {
  std::cout << label << '\n';
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  axes lx:" << gamepad.lx << " ly:" << gamepad.ly << " rx:" << gamepad.rx
            << " ry:" << gamepad.ry << " lt:" << gamepad.lt << " rt:" << gamepad.rt << '\n';

  std::cout << "  buttons";
  print_button("A", gamepad.A);
  print_button("B", gamepad.B);
  print_button("X", gamepad.X);
  print_button("Y", gamepad.Y);
  print_button("up", gamepad.up);
  print_button("right", gamepad.right);
  print_button("down", gamepad.down);
  print_button("left", gamepad.left);
  print_button("R1", gamepad.R1);
  print_button("L1", gamepad.L1);
  print_button("R2", gamepad.R2);
  print_button("L2", gamepad.L2);
  print_button("start", gamepad.start);
  print_button("select", gamepad.select);
  print_button("emergency", gamepad.emergency_btn);
  print_button("start_policy", gamepad.start_policy_btn);
  std::cout << '\n';

  std::cout << "  custom";
  print_button("move_default", gamepad.moveto_default_pos_btn);
  print_button("hold_position", gamepad.hold_position_mode_btn);
  print_button("cmd_reset", gamepad.cmd_reset_btn);
  print_button("cycle_task", gamepad.cycle_task_btn);
  std::cout << '\n';

  std::cout << "  mocap";
  print_button("emergency", gamepad.mocap_cmd_emergency_btn);
  print_button("move_default", gamepad.mocap_cmd_moveto_default_pos_btn);
  print_button("start_policy", gamepad.mocap_cmd_start_policy_btn);
  std::cout << '\n';
}

void print_encoded_payload(const std::array<std::uint8_t, 40>& payload) {
  std::cout << "encoded wireless_remote[40]";
  for (const auto byte : payload) {
    std::cout << ' ' << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
              << static_cast<unsigned>(byte);
  }
  std::cout << std::dec << std::nouppercase << std::setfill(' ') << '\n';
}

bridge::gamepad::CommonGamepad make_sample_gamepad() {
  bridge::gamepad::CommonGamepad gamepad;
  gamepad.lx = 0.125F;
  gamepad.ly = -0.250F;
  gamepad.rx = 0.500F;
  gamepad.ry = -0.750F;
  gamepad.lt = 0.875F;
  gamepad.rt = 0.625F;

  gamepad.A.update(true);
  gamepad.X.update(true);
  gamepad.right.update(true);
  gamepad.R1.update(true);
  gamepad.L2.update(true);
  gamepad.start.update(true);
  gamepad.select.update(true);
  gamepad.emergency_btn.update(true);
  gamepad.start_policy_btn.update(true);
  gamepad.moveto_default_pos_btn.update(true);
  gamepad.cmd_reset_btn.update(true);
  gamepad.mocap_cmd_emergency_btn.update(true);
  gamepad.mocap_cmd_start_policy_btn.update(true);

  return gamepad;
}

}  // namespace

int main() {
  std::cout << std::boolalpha;

  const bridge::gamepad::CommonGamepad source = make_sample_gamepad();
  const std::array<std::uint8_t, 40> encoded =
      bridge::gamepad::compress_to_wireless_remote(source);

  bridge::gamepad::CommonGamepad decoded;
  const bool ok = common_gamepad_wireless::unpackWirelessRemoteToCommonGamepad(
      encoded.data(), decoded);

  print_gamepad("source CommonGamepad before encode", source);
  print_encoded_payload(encoded);
  std::cout << "decode ok: " << ok << '\n';
  print_gamepad("decoded CommonGamepad after docs codec", decoded);

  return ok ? 0 : 1;
}
