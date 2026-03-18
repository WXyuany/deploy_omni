#include "../../../include/bridge/extensions/hooks.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <utility>

namespace bridge::extensions {

namespace {

double clamp_alpha(const double alpha) {
  return std::clamp(alpha, 0.0, 1.0);
}

bool set_if_present(std::vector<double>& values,
                    const std::size_t index,
                    const double replacement) {
  if (index >= values.size()) {
    return false;
  }
  const bool changed = values[index] != replacement;
  values[index] = replacement;
  return changed;
}

std::string join_doubles(const std::vector<double>& values) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6);
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      stream << ',';
    }
    stream << values[index];
  }
  return stream.str();
}

}

LegacyKeplerHooks::LegacyKeplerHooks(LegacyKeplerHookOptions options)
    : options_(std::move(options)) {
  options_.torque_smoothing.alpha =
      clamp_alpha(options_.torque_smoothing.alpha);
}

const LegacyKeplerHookOptions& LegacyKeplerHooks::options() const noexcept {
  return options_;
}

CommandHookResult LegacyKeplerHooks::apply_joint_lock(
    const bridge::core::CommandFrame& command,
    const HookInvocationContext& context) const {
  static_cast<void>(context);

  CommandHookResult result{.command = command, .changed = false};
  for (const JointLockSetting& lock : options_.joint_locks) {
    result.changed =
        set_if_present(result.command.joints.position, lock.joint_index,
                       lock.locked_position) ||
        result.changed;
    result.changed =
        set_if_present(result.command.joints.velocity, lock.joint_index,
                       lock.locked_velocity) ||
        result.changed;
    result.changed =
        set_if_present(result.command.joints.effort, lock.joint_index,
                       lock.locked_effort) ||
        result.changed;
  }
  return result;
}

CommandHookResult LegacyKeplerHooks::smooth_command(
    const bridge::core::CommandFrame& command,
    const HookInvocationContext& context) {
  static_cast<void>(context);

  CommandHookResult result{.command = command, .changed = false};
  if (!options_.torque_smoothing.enabled ||
      command.mode != bridge::core::ControlMode::kTorque ||
      command.joints.effort.empty()) {
    return result;
  }

  if (!has_previous_effort_ || previous_effort_.size() != command.joints.effort.size()) {
    previous_effort_ = command.joints.effort;
    has_previous_effort_ = true;
    return result;
  }

  const double alpha = options_.torque_smoothing.alpha;
  for (std::size_t index = 0; index < command.joints.effort.size(); ++index) {
    const double smoothed = previous_effort_[index] +
                            alpha * (command.joints.effort[index] -
                                     previous_effort_[index]);
    result.changed = (smoothed != command.joints.effort[index]) || result.changed;
    result.command.joints.effort[index] = smoothed;
    previous_effort_[index] = smoothed;
  }
  return result;
}

void LegacyKeplerHooks::observe_command(
    const bridge::core::CommandFrame& command,
    const HookInvocationContext& context) {
  if (!options_.capture_command_csv) {
    return;
  }

  std::ostringstream stream;
  stream << "cycle=" << context.cycle_index << ",sequence=" << command.sequence
         << ",position=" << join_doubles(command.joints.position)
         << ",velocity=" << join_doubles(command.joints.velocity)
         << ",effort=" << join_doubles(command.joints.effort);
  records_.push_back({.kind = ObservationKind::kCommandCsv,
                      .sequence = command.sequence,
                      .payload = stream.str()});
}

void LegacyKeplerHooks::observe_state(const bridge::core::StateFrame& state,
                                      const HookInvocationContext& context) {
  if (!options_.capture_state_csv) {
    return;
  }

  std::ostringstream stream;
  stream << "cycle=" << context.cycle_index << ",sequence=" << state.sequence
         << ",position=" << join_doubles(state.joints.position)
         << ",velocity=" << join_doubles(state.joints.velocity)
         << ",effort=" << join_doubles(state.joints.effort);
  records_.push_back({.kind = ObservationKind::kStateCsv,
                      .sequence = state.sequence,
                      .payload = stream.str()});
}

void LegacyKeplerHooks::observe_debug(std::string_view message,
                                      const HookInvocationContext& context) {
  if (!options_.capture_debug_messages) {
    return;
  }

  std::ostringstream stream;
  stream << "cycle=" << context.cycle_index << ",message=" << message;
  records_.push_back({.kind = ObservationKind::kDebugMessage,
                      .sequence = context.time_ns,
                      .payload = stream.str()});
}

const std::vector<ObservationRecord>& LegacyKeplerHooks::records() const noexcept {
  return records_;
}

std::string LegacyKeplerHooks::select_mode_label(
    const bridge::core::ControlMode mode,
    std::string_view default_label) const {
  for (const ExtraModeLabel& entry : options_.extra_mode_labels) {
    if (entry.mode == mode) {
      return entry.label;
    }
  }
  return std::string(default_label);
}

void LegacyKeplerHooks::reset() {
  previous_effort_.clear();
  has_previous_effort_ = false;
  records_.clear();
}

std::unique_ptr<LegacyKeplerHooks> make_legacy_kepler_hooks(
    LegacyKeplerHookOptions options) {
  return std::make_unique<LegacyKeplerHooks>(std::move(options));
}

}
