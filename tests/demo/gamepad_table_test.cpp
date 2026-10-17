#include <cassert>
#include <string>

#include "../../examples/gamepad/gamepad_table.hpp"

int main() {
  bridge::gamepad::SdlGamepadSnapshot snapshot;
  snapshot.axes[bridge::gamepad::kLeftX] = 123;
  snapshot.axes[bridge::gamepad::kRightX] = -456;
  snapshot.axes[bridge::gamepad::kTriggerRight] = 32767;

  bridge::gamepad::CommonGamepad gamepad;
  gamepad.raw_lx = 0.1F;
  gamepad.lx = 0.05F;
  gamepad.raw_rx = -0.2F;
  gamepad.rx = -0.1F;
  gamepad.raw_rt = 1.0F;
  gamepad.rt = 1.0F;
  gamepad.A.update(true);
  gamepad.start.update(true);

  const std::string table = bridge::demo::format_gamepad_sender_table(
      gamepad, snapshot, 42, 60, 0, true);

  assert(table.find("\033[2J\033[H") != std::string::npos);
  assert(table.find("Gamepad DDS Sender") != std::string::npos);
  assert(table.find("refresh=2Hz") != std::string::npos);
  assert(table.find("right_x/rx") != std::string::npos);
  assert(table.find("trigger_r/rt") != std::string::npos);
  assert(table.find("A=1") != std::string::npos);
  assert(table.find("start=1") != std::string::npos);
  return 0;
}
