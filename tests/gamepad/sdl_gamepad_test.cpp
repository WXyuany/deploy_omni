#include <cassert>

#include "../../include/bridge/gamepad/sdl_gamepad.hpp"

namespace {

void TestSnapshotConversion() {
  bridge::gamepad::SdlGamepadSnapshot snapshot;
  snapshot.axes[bridge::gamepad::SdlGamepadAxis::kLeftX] = 16384;
  snapshot.axes[bridge::gamepad::SdlGamepadAxis::kLeftY] = -32768;
  snapshot.axes[bridge::gamepad::SdlGamepadAxis::kRightX] = -16384;
  snapshot.axes[bridge::gamepad::SdlGamepadAxis::kRightY] = 32767;
  snapshot.axes[bridge::gamepad::SdlGamepadAxis::kTriggerLeft] = 32767;
  snapshot.axes[bridge::gamepad::SdlGamepadAxis::kTriggerRight] = 0;
  snapshot.buttons[bridge::gamepad::SdlGamepadButton::kA] = true;
  snapshot.buttons[bridge::gamepad::SdlGamepadButton::kBack] = true;
  snapshot.buttons[bridge::gamepad::SdlGamepadButton::kLeftShoulder] = true;

  bridge::gamepad::CommonGamepad gamepad;
  bridge::gamepad::convert_sdl_snapshot_to_common_gamepad(snapshot, gamepad);

  assert(gamepad.raw_lx > 0.49F && gamepad.raw_lx < 0.51F);
  assert(gamepad.raw_ly == -1.0F);
  assert(gamepad.raw_rx > -0.51F && gamepad.raw_rx < -0.49F);
  assert(gamepad.raw_ry == 1.0F);
  assert(gamepad.raw_lt == 1.0F);
  assert(gamepad.raw_rt == 0.0F);
  assert(gamepad.lt == 1.0F);
  assert(gamepad.rt == 0.0F);
  assert(gamepad.A.pressed);
  assert(gamepad.back.pressed);
  assert(gamepad.select.pressed);
  assert(gamepad.LB.pressed);
  assert(gamepad.L1.pressed);
  assert(gamepad.lx > 0.07F);
}

void TestDebugLineShowsSdlAndCommonValues() {
  bridge::gamepad::SdlGamepadSnapshot snapshot;
  snapshot.axes[bridge::gamepad::SdlGamepadAxis::kLeftX] = 16384;
  snapshot.axes[bridge::gamepad::SdlGamepadAxis::kLeftY] = -32768;
  snapshot.buttons[bridge::gamepad::SdlGamepadButton::kA] = true;
  snapshot.buttons[bridge::gamepad::SdlGamepadButton::kStart] = true;

  bridge::gamepad::CommonGamepad gamepad;
  bridge::gamepad::convert_sdl_snapshot_to_common_gamepad(snapshot, gamepad);

  const std::string line =
      bridge::gamepad::format_sdl_gamepad_debug_line(snapshot, gamepad);
  assert(line.find("[SdlGamepadRemote.read]") != std::string::npos);
  assert(line.find("raw_axes={lx=16384") != std::string::npos);
  assert(line.find("common_axes={lx=") != std::string::npos);
  assert(line.find("buttons={A=1") != std::string::npos);
  assert(line.find("start=1") != std::string::npos);
}

void TestUnavailableReaderReturnsNoSnapshot() {
  bridge::gamepad::SdlGamepadRemote remote({.controller_index = 9999});
  const auto start = remote.start();
  assert(!start.connected);
  assert(!remote.read().has_value());
  remote.stop();
}

}  // namespace

int main() {
  TestSnapshotConversion();
  TestDebugLineShowsSdlAndCommonValues();
  TestUnavailableReaderReturnsNoSnapshot();
  return 0;
}
