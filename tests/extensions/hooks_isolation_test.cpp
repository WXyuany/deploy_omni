#include <cassert>
#include <string>
#include <vector>

#include "../../include/bridge/extensions/hooks.hpp"

namespace {

using bridge::core::CommandFrame;
using bridge::core::ControlMode;
using bridge::core::LifecycleState;
using bridge::core::StateFrame;
using bridge::extensions::ExtraModeLabel;
using bridge::extensions::HookInvocationContext;
using bridge::extensions::JointLockSetting;
using bridge::extensions::LegacyKeplerHookOptions;
using bridge::extensions::LegacyKeplerHooks;
using bridge::extensions::ObservationKind;
using bridge::extensions::TerminalObservationOptions;
using bridge::extensions::TorqueSmoothingSetting;

CommandFrame MakeCommand() {
  CommandFrame command;
  command.sequence = 7;
  command.source_time_ns = 100;
  command.receive_time_ns = 110;
  command.mode = ControlMode::kTorque;
  command.mode_label = "torque";
  command.valid = true;
  command.joints.position = {1.0, 2.0, 3.0};
  command.joints.velocity = {0.1, 0.2, 0.3};
  command.joints.effort = {4.0, 5.0, 6.0};
  return command;
}

StateFrame MakeState() {
  StateFrame state;
  state.sequence = 9;
  state.sample_time_ns = 200;
  state.publish_time_ns = 210;
  state.lifecycle_state = LifecycleState::kActive;
  state.active_mode = ControlMode::kTorque;
  state.mode_label = "torque";
  state.valid = true;
  state.joints.position = {0.5, 1.5, 2.5};
  state.joints.velocity = {0.01, 0.02, 0.03};
  state.joints.effort = {0.4, 0.5, 0.6};
  return state;
}

bool SameCommand(const CommandFrame& left, const CommandFrame& right) {
  return left.sequence == right.sequence &&
         left.source_time_ns == right.source_time_ns &&
         left.receive_time_ns == right.receive_time_ns &&
         left.mode == right.mode && left.mode_label == right.mode_label &&
         left.valid == right.valid && left.stale == right.stale &&
         left.degraded == right.degraded &&
         left.joints.position == right.joints.position &&
         left.joints.velocity == right.joints.velocity &&
         left.joints.effort == right.joints.effort &&
         left.joints.stiffness == right.joints.stiffness &&
         left.joints.damping == right.joints.damping;
}

void TestDefaultHooksStayDisabledAndDoNotChangeCoreFrames() {
  auto hooks = bridge::extensions::make_legacy_kepler_hooks();
  const CommandFrame command = MakeCommand();
  const HookInvocationContext context{.cycle_index = 3, .time_ns = 300};

  const auto locked = hooks->apply_joint_lock(command, context);
  assert(!locked.changed);
  assert(SameCommand(locked.command, command));

  const auto smoothed = hooks->smooth_command(command, context);
  assert(!smoothed.changed);
  assert(SameCommand(smoothed.command, command));

  hooks->observe_command(command, context);
  hooks->observe_state(MakeState(), context);
  hooks->observe_debug("suppressed debug", context);
  assert(hooks->records().empty());

  assert(hooks->select_mode_label(ControlMode::kTorque, "torque") == "torque");
  assert(hooks->options().joint_locks.empty());
  assert(!hooks->options().torque_smoothing.enabled);
}

void TestEnabledHooksRemainDeterministicAndExplicitlyOptIn() {
  LegacyKeplerHookOptions options;
  options.joint_locks.push_back(JointLockSetting{
      .joint_index = 1,
      .locked_position = 42.0,
      .locked_velocity = 0.0,
      .locked_effort = 0.0,
  });
  options.torque_smoothing = TorqueSmoothingSetting{.enabled = true, .alpha = 0.25};
  options.capture_command_csv = true;
  options.capture_state_csv = true;
  options.capture_debug_messages = true;
  options.extra_mode_labels.push_back(
      ExtraModeLabel{.mode = ControlMode::kTorque, .label = "legacy_torque"});

  LegacyKeplerHooks hooks(options);
  const HookInvocationContext context{.cycle_index = 5, .time_ns = 500};

  const auto locked = hooks.apply_joint_lock(MakeCommand(), context);
  assert(locked.changed);
  assert(locked.command.joints.position[1] == 42.0);
  assert(locked.command.joints.velocity[1] == 0.0);
  assert(locked.command.joints.effort[1] == 0.0);
  assert(locked.command.joints.position[0] == 1.0);
  assert(locked.command.joints.effort[2] == 6.0);

  const auto first_smoothed = hooks.smooth_command(locked.command, context);
  assert(!first_smoothed.changed);
  assert(SameCommand(first_smoothed.command, locked.command));

  CommandFrame next_command = locked.command;
  next_command.sequence = 8;
  next_command.joints.effort = {8.0, 4.0, 2.0};
  const auto second_smoothed = hooks.smooth_command(next_command, context);
  assert(second_smoothed.changed);
  assert(second_smoothed.command.joints.effort ==
         std::vector<double>({5.0, 1.0, 5.0}));

  hooks.observe_command(second_smoothed.command, context);
  hooks.observe_state(MakeState(), context);
  hooks.observe_debug("color print isolated", context);
  assert(hooks.records().size() == 3U);
  assert(hooks.records()[0].kind == ObservationKind::kCommandCsv);
  assert(hooks.records()[0].sequence == 8);
  assert(hooks.records()[1].kind == ObservationKind::kStateCsv);
  assert(hooks.records()[1].sequence == 9);
  assert(hooks.records()[2].kind == ObservationKind::kDebugMessage);
  assert(hooks.records()[2].payload == "cycle=5,message=color print isolated");

  assert(hooks.select_mode_label(ControlMode::kTorque, "torque") == "legacy_torque");
  assert(hooks.select_mode_label(ControlMode::kPosition, "position") == "position");

  hooks.reset();
  assert(hooks.records().empty());
  const auto reset_smoothed = hooks.smooth_command(next_command, context);
  assert(!reset_smoothed.changed);
  assert(SameCommand(reset_smoothed.command, next_command));
}

void TestTerminalObservationHookStaysCallable() {
  auto hook = bridge::extensions::make_terminal_observation_hook(
      TerminalObservationOptions{.print_command = true,
                                 .print_state = true,
                                 .rate_hz = 0,
                                 .joint_names = {"joint_a", "joint_b", "joint_c"}});
  const HookInvocationContext context{.cycle_index = 7, .time_ns = 700};
  hook->observe_command(MakeCommand(), context);
  hook->observe_state(MakeState(), context);
  hook->observe_debug("ignored", context);
  assert(hook->records().empty());
}

}

int main() {
  TestDefaultHooksStayDisabledAndDoNotChangeCoreFrames();
  TestEnabledHooksRemainDeterministicAndExplicitlyOptIn();
  TestTerminalObservationHookStaysCallable();
  return 0;
}
