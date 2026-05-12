#include "../../../include/bridge/extensions/hooks.hpp"

#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

namespace bridge::extensions {
namespace {

namespace Color {
constexpr const char* kReset = "\033[0m";
constexpr const char* kBold = "\033[1m";
constexpr const char* kCyan = "\033[36m";
constexpr const char* kGreen = "\033[32m";
constexpr const char* kYellow = "\033[33m";
constexpr const char* kBlue = "\033[34m";
constexpr const char* kMagenta = "\033[35m";
constexpr const char* kRed = "\033[31m";
constexpr const char* kWhite = "\033[37m";
constexpr const char* kGray = "\033[90m";
}  // namespace Color

[[nodiscard]] bool should_print(const std::uint64_t now_ns,
                                std::uint64_t* const last_print_ns,
                                const std::uint32_t rate_hz) {
  if (rate_hz == 0) {
    return true;
  }
  const std::uint64_t interval_ns = 1000000000ULL / rate_hz;
  if (*last_print_ns == 0 || now_ns <= *last_print_ns ||
      (now_ns - *last_print_ns) >= interval_ns) {
    *last_print_ns = now_ns;
    return true;
  }
  return false;
}

[[nodiscard]] std::string joint_name_at(const std::vector<std::string>& names,
                                        const std::size_t index) {
  if (index < names.size()) {
    return names[index];
  }
  return "joint_" + std::to_string(index);
}

void append_joint_rows(std::ostringstream* const stream,
                       const std::vector<std::string>& joint_names,
                       const std::vector<double>& position,
                       const std::vector<double>& velocity,
                       const char* const value_color) {
  const std::size_t joint_count = std::max(position.size(), velocity.size());
  for (std::size_t index = 0; index < joint_count; ++index) {
    const double pos = index < position.size() ? position[index] : 0.0;
    const double vel = index < velocity.size() ? velocity[index] : 0.0;
    *stream << Color::kGray << std::left << std::setw(4) << index << Color::kReset
            << Color::kCyan << std::left << std::setw(24)
            << joint_name_at(joint_names, index) << Color::kReset
            << value_color << std::right << std::setw(12) << pos << Color::kReset
            << value_color << std::right << std::setw(12) << vel << Color::kReset
            << '\n';
  }
}

[[nodiscard]] std::string format_command_screen(
    const TerminalObservationOptions& options,
    const bridge::core::CommandFrame& command,
    const HookInvocationContext& context) {
  std::ostringstream stream;
  stream << "\033[2J\033[H";
  stream << Color::kBold << Color::kYellow << "COMMAND MONITOR" << Color::kReset
         << "  cycle=" << context.cycle_index << " sequence=" << command.sequence
         << " mode=" << command.mode_label << '\n';
  stream << Color::kGray << std::left << std::setw(4) << "idx"
         << std::setw(24) << "joint"
         << std::right << std::setw(12) << "position"
         << std::setw(12) << "velocity" << Color::kReset << '\n';
  append_joint_rows(&stream, options.joint_names, command.joints.position,
                    command.joints.velocity, Color::kYellow);
  return stream.str();
}

[[nodiscard]] std::string format_state_screen(
    const TerminalObservationOptions& options,
    const bridge::core::StateFrame& state,
    const HookInvocationContext& context) {
  std::ostringstream stream;
  stream << "\033[2J\033[H";
  stream << Color::kBold << Color::kGreen << "STATE MONITOR" << Color::kReset
         << "  cycle=" << context.cycle_index << " sequence=" << state.sequence
         << " mode=" << state.mode_label << '\n';
  stream << Color::kGray << std::left << std::setw(4) << "idx"
         << std::setw(24) << "joint"
         << std::right << std::setw(12) << "position"
         << std::setw(12) << "velocity" << Color::kReset << '\n';
  append_joint_rows(&stream, options.joint_names, state.joints.position,
                    state.joints.velocity, Color::kGreen);

  if (state.imu.has_value()) {
    const auto& imu = *state.imu;
    stream << '\n' << Color::kBold << Color::kMagenta << "IMU" << Color::kReset
           << '\n';
    if (imu.orientation_xyzw.size() >= 4) {
      stream << Color::kBlue << "quat:" << Color::kReset << ' '
             << imu.orientation_xyzw[0] << ' ' << imu.orientation_xyzw[1] << ' '
             << imu.orientation_xyzw[2] << ' ' << imu.orientation_xyzw[3] << '\n';
    }
    if (imu.rpy_xyz.size() >= 3) {
      stream << Color::kBlue << "rpy :" << Color::kReset << ' ' << imu.rpy_xyz[0]
             << ' ' << imu.rpy_xyz[1] << ' ' << imu.rpy_xyz[2] << '\n';
    }
    if (imu.angular_velocity_xyz.size() >= 3) {
      stream << Color::kBlue << "gyro:" << Color::kReset << ' '
             << imu.angular_velocity_xyz[0] << ' ' << imu.angular_velocity_xyz[1]
             << ' ' << imu.angular_velocity_xyz[2] << '\n';
    }
    if (imu.linear_acceleration_xyz.size() >= 3) {
      stream << Color::kBlue << "acc :" << Color::kReset << ' '
             << imu.linear_acceleration_xyz[0] << ' ' << imu.linear_acceleration_xyz[1]
             << ' ' << imu.linear_acceleration_xyz[2] << '\n';
    }
  }
  return stream.str();
}

}  // namespace

TerminalObservationHook::TerminalObservationHook(TerminalObservationOptions options)
    : options_(std::move(options)) {}

void TerminalObservationHook::observe_command(
    const bridge::core::CommandFrame& command,
    const HookInvocationContext& context) {
  if (!options_.print_command ||
      !should_print(context.time_ns, &last_command_print_time_ns_, options_.rate_hz)) {
    return;
  }
  const std::string output = format_command_screen(options_, command, context);
  std::fputs(output.c_str(), stdout);
  std::fflush(stdout);
}

void TerminalObservationHook::observe_state(const bridge::core::StateFrame& state,
                                            const HookInvocationContext& context) {
  if (!options_.print_state ||
      !should_print(context.time_ns, &last_state_print_time_ns_, options_.rate_hz)) {
    return;
  }
  const std::string output = format_state_screen(options_, state, context);
  std::fputs(output.c_str(), stdout);
  std::fflush(stdout);
}

void TerminalObservationHook::observe_debug(std::string_view, const HookInvocationContext&) {}

const std::vector<ObservationRecord>& TerminalObservationHook::records() const noexcept {
  return records_;
}

std::unique_ptr<TerminalObservationHook> make_terminal_observation_hook(
    TerminalObservationOptions options) {
  return std::make_unique<TerminalObservationHook>(std::move(options));
}

}  // namespace bridge::extensions
