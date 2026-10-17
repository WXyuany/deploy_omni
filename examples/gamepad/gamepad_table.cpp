#include "gamepad_table.hpp"

#include <iomanip>
#include <sstream>

namespace bridge::demo {
namespace {

[[nodiscard]] const char* cyan(const bool color) noexcept {
  return color ? "\033[36m" : "";
}

[[nodiscard]] const char* green(const bool color) noexcept {
  return color ? "\033[32m" : "";
}

[[nodiscard]] const char* yellow(const bool color) noexcept {
  return color ? "\033[33m" : "";
}

[[nodiscard]] const char* reset(const bool color) noexcept {
  return color ? "\033[0m" : "";
}

[[nodiscard]] const char* screen_home(const bool color) noexcept {
  return color ? "\033[2J\033[H" : "";
}

void write_axis_row(std::ostringstream& out, const char* name, const int raw,
                    const float normalized, const float packed) {
  out << "| " << std::left << std::setw(13) << name << " | " << std::right
      << std::setw(8) << raw << " | " << std::setw(10) << normalized << " | "
      << std::setw(10) << packed << " |\n";
}

void write_button(std::ostringstream& out, const char* name, const bool pressed) {
  out << name << '=' << (pressed ? '1' : '0') << ' ';
}

}  // namespace

std::string format_gamepad_sender_table(
    const gamepad::CommonGamepad& gamepad,
    const gamepad::SdlGamepadSnapshot& snapshot,
    const std::uint64_t published,
    const std::uint64_t publish_rate_hz,
    const int controller_index,
    const bool color) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(3);
  out << screen_home(color);
  out << cyan(color) << "Gamepad DDS Sender" << reset(color) << '\n';
  out << "published=" << published << " rate=" << publish_rate_hz
      << "Hz controller=" << controller_index << " refresh=2Hz\n\n";

  out << green(color) << "Axes before wireless_remote pack" << reset(color)
      << '\n';
  out << "+---------------+----------+------------+------------+\n";
  out << "| Axis          | SDL raw  | normalized | packed     |\n";
  out << "+---------------+----------+------------+------------+\n";
  write_axis_row(out, "left_x/lx", snapshot.axes[gamepad::kLeftX],
                 gamepad.raw_lx, gamepad.lx);
  write_axis_row(out, "left_y/ly", snapshot.axes[gamepad::kLeftY],
                 gamepad.raw_ly, gamepad.ly);
  write_axis_row(out, "right_x/rx", snapshot.axes[gamepad::kRightX],
                 gamepad.raw_rx, gamepad.rx);
  write_axis_row(out, "right_y/ry", snapshot.axes[gamepad::kRightY],
                 gamepad.raw_ry, gamepad.ry);
  write_axis_row(out, "trigger_l/lt", snapshot.axes[gamepad::kTriggerLeft],
                 gamepad.raw_lt, gamepad.lt);
  write_axis_row(out, "trigger_r/rt", snapshot.axes[gamepad::kTriggerRight],
                 gamepad.raw_rt, gamepad.rt);
  out << "+---------------+----------+------------+------------+\n\n";

  out << yellow(color) << "Buttons" << reset(color) << '\n';
  write_button(out, "A", gamepad.A.pressed);
  write_button(out, "B", gamepad.B.pressed);
  write_button(out, "X", gamepad.X.pressed);
  write_button(out, "Y", gamepad.Y.pressed);
  write_button(out, "LB", gamepad.LB.pressed);
  write_button(out, "RB", gamepad.RB.pressed);
  write_button(out, "LT", gamepad.LT.pressed);
  write_button(out, "RT", gamepad.RT.pressed);
  write_button(out, "start", gamepad.start.pressed);
  write_button(out, "select", gamepad.select.pressed);
  write_button(out, "up", gamepad.up.pressed);
  write_button(out, "down", gamepad.down.pressed);
  write_button(out, "left", gamepad.left.pressed);
  write_button(out, "right", gamepad.right.pressed);
  out << '\n';
  return out.str();
}

}  // namespace bridge::demo
