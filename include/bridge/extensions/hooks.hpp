#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "../core/types.hpp"

namespace bridge::extensions {

struct HookInvocationContext {
  std::uint64_t cycle_index{0};
  std::uint64_t time_ns{0};
};

struct JointLockSetting {
  std::size_t joint_index{0};
  double locked_position{0.0};
  double locked_velocity{0.0};
  double locked_effort{0.0};
};

struct TorqueSmoothingSetting {
  bool enabled{false};
  double alpha{1.0};
};

struct ExtraModeLabel {
  bridge::core::ControlMode mode{bridge::core::ControlMode::kUnknown};
  std::string label;
};

enum class ObservationKind {
  kCommandCsv,
  kStateCsv,
  kDebugMessage,
};

struct ObservationRecord {
  ObservationKind kind{ObservationKind::kDebugMessage};
  std::uint64_t sequence{0};
  std::string payload;
};

struct CommandHookResult {
  bridge::core::CommandFrame command;
  bool changed{false};
};

struct LegacyKeplerHookOptions {
  std::vector<JointLockSetting> joint_locks;
  TorqueSmoothingSetting torque_smoothing;
  bool capture_command_csv{false};
  bool capture_state_csv{false};
  bool capture_debug_messages{false};
  std::vector<ExtraModeLabel> extra_mode_labels;
};

struct TerminalObservationOptions {
  bool print_command{false};
  bool print_state{false};
  std::uint32_t rate_hz{0};
  std::vector<std::string> joint_names;
};

class JointLockHook {
 public:
  virtual ~JointLockHook() = default;

  [[nodiscard]] virtual CommandHookResult apply_joint_lock(
      const bridge::core::CommandFrame& command,
      const HookInvocationContext& context) const = 0;
};

class TorqueSmoothingHook {
 public:
  virtual ~TorqueSmoothingHook() = default;

  [[nodiscard]] virtual CommandHookResult smooth_command(
      const bridge::core::CommandFrame& command,
      const HookInvocationContext& context) = 0;
};

class ObservationHook {
 public:
  virtual ~ObservationHook() = default;

  virtual void observe_command(const bridge::core::CommandFrame& command,
                               const HookInvocationContext& context) = 0;
  virtual void observe_state(const bridge::core::StateFrame& state,
                             const HookInvocationContext& context) = 0;
  virtual void observe_debug(std::string_view message,
                             const HookInvocationContext& context) = 0;

  [[nodiscard]] virtual const std::vector<ObservationRecord>& records()
      const noexcept = 0;
};

class ExtraModeHook {
 public:
  virtual ~ExtraModeHook() = default;

  [[nodiscard]] virtual std::string select_mode_label(
      bridge::core::ControlMode mode,
      std::string_view default_label) const = 0;
};

class LegacyKeplerHooks final : public JointLockHook,
                                public TorqueSmoothingHook,
                                public ObservationHook,
                                public ExtraModeHook {
 public:
  explicit LegacyKeplerHooks(LegacyKeplerHookOptions options = {});

  [[nodiscard]] const LegacyKeplerHookOptions& options() const noexcept;

  [[nodiscard]] CommandHookResult apply_joint_lock(
      const bridge::core::CommandFrame& command,
      const HookInvocationContext& context) const override;

  [[nodiscard]] CommandHookResult smooth_command(
      const bridge::core::CommandFrame& command,
      const HookInvocationContext& context) override;

  void observe_command(const bridge::core::CommandFrame& command,
                       const HookInvocationContext& context) override;
  void observe_state(const bridge::core::StateFrame& state,
                     const HookInvocationContext& context) override;
  void observe_debug(std::string_view message,
                     const HookInvocationContext& context) override;

  [[nodiscard]] const std::vector<ObservationRecord>& records()
      const noexcept override;

  [[nodiscard]] std::string select_mode_label(
      bridge::core::ControlMode mode,
      std::string_view default_label) const override;

  void reset();

 private:
  LegacyKeplerHookOptions options_;
  std::vector<double> previous_effort_;
  bool has_previous_effort_{false};
  std::vector<ObservationRecord> records_;
};

[[nodiscard]] std::unique_ptr<LegacyKeplerHooks> make_legacy_kepler_hooks(
    LegacyKeplerHookOptions options = {});

class TerminalObservationHook final : public ObservationHook {
 public:
  explicit TerminalObservationHook(TerminalObservationOptions options = {});

  void observe_command(const bridge::core::CommandFrame& command,
                       const HookInvocationContext& context) override;
  void observe_state(const bridge::core::StateFrame& state,
                     const HookInvocationContext& context) override;
  void observe_debug(std::string_view message,
                     const HookInvocationContext& context) override;

  [[nodiscard]] const std::vector<ObservationRecord>& records()
      const noexcept override;

 private:
  TerminalObservationOptions options_;
  std::vector<ObservationRecord> records_;
  std::uint64_t last_command_print_time_ns_{0};
  std::uint64_t last_state_print_time_ns_{0};
};

[[nodiscard]] std::unique_ptr<TerminalObservationHook> make_terminal_observation_hook(
    TerminalObservationOptions options = {});

}
