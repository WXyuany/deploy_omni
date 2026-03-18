#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "robot_sdk_adapter.hpp"
#include "types.hpp"

namespace bridge::core {

enum class RunnerCallStatus {
  kOk,
  kDegraded,
  kUnsupported,
  kRejected,
  kRetryableError,
  kFatalError,
};

struct RunnerCallResult {
  RunnerCallStatus status{RunnerCallStatus::kOk};
  std::string detail;
  std::optional<BridgeFault> fault;

  [[nodiscard]] bool succeeded() const noexcept {
    return status == RunnerCallStatus::kOk ||
           status == RunnerCallStatus::kDegraded;
  }
};

enum class StartupIdlePolicy {
  kRequireSafeCommand,
  kPreferSafeCommand,
  kNoCommandWrite,
};

struct BridgeRunnerFaultPolicy {
  bool fail_stop_on_transport_fault{true};
  bool fail_stop_on_command_stale{true};
  bool fail_stop_on_adapter_degraded{false};
  bool write_safe_command_on_fault{true};
  bool continue_publishing_in_fault{true};
};

struct BridgeRunnerConfig {
  std::uint64_t loop_period_ns{0};
  std::uint64_t command_stale_timeout_ns{0};
  StartupIdlePolicy startup_idle_policy{StartupIdlePolicy::kPreferSafeCommand};
  BridgeRunnerFaultPolicy fault_policy;
};

struct RunnerCommandBufferState {
  bool has_latest_command{false};
  bool has_active_command{false};
  bool latest_command_stale{false};
  std::uint64_t latest_command_sequence{0};
  std::uint64_t latest_command_receive_time_ns{0};
  std::uint64_t active_command_sequence{0};
  ControlMode active_mode{ControlMode::kUnknown};
};

struct BridgeRunnerStatus {
  LifecycleState lifecycle_state{LifecycleState::kCreated};
  bool stop_requested{false};
  bool first_state_received{false};
  bool first_command_received{false};
  bool startup_idle_active{true};
  bool command_stale{false};
  bool fail_stop_active{false};
  bool degraded{false};
  std::uint64_t cycle_count{0};
  std::uint64_t last_cycle_time_ns{0};
  std::uint64_t last_state_publish_time_ns{0};
  RunnerCommandBufferState command_buffer;
  std::optional<BridgeFault> latched_fault;
};

struct CommandSubmissionResult {
  RunnerCallResult result;
  bool replaced_previous{false};
  std::uint64_t stored_sequence{0};
};

class BridgeCommandSink {
 public:
  virtual ~BridgeCommandSink() = default;

  virtual CommandSubmissionResult submit_latest_command(CommandFrame command) = 0;
};

class BridgeRuntimeEventSink {
 public:
  virtual ~BridgeRuntimeEventSink() = default;

  virtual RunnerCallResult report_transport_fault(BridgeFault fault) = 0;
};

class BridgeCommandIngress {
 public:
  virtual ~BridgeCommandIngress() = default;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;

  virtual RunnerCallResult start(BridgeCommandSink& command_sink,
                                 BridgeRuntimeEventSink& event_sink) = 0;

  virtual RunnerCallResult stop() = 0;
};

class BridgeStateEgress {
 public:
  virtual ~BridgeStateEgress() = default;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;

  virtual RunnerCallResult start() = 0;

  virtual RunnerCallResult publish(const StateFrame& state) = 0;

  virtual RunnerCallResult stop() = 0;
};

struct BridgeRunnerDependencies {
  RobotSdkAdapter* adapter{nullptr};
  BridgeCommandIngress* command_ingress{nullptr};
  BridgeStateEgress* state_egress{nullptr};
};

class BridgeRunner final : public BridgeCommandSink,
                           public BridgeRuntimeEventSink {
 public:
  BridgeRunner(BridgeRunnerConfig config, BridgeRunnerDependencies dependencies);
  ~BridgeRunner();

  BridgeRunner(const BridgeRunner&) = delete;
  BridgeRunner& operator=(const BridgeRunner&) = delete;
  BridgeRunner(BridgeRunner&&) = delete;
  BridgeRunner& operator=(BridgeRunner&&) = delete;

  [[nodiscard]] const BridgeRunnerConfig& config() const noexcept;

  [[nodiscard]] BridgeRunnerStatus status() const;

  [[nodiscard]] RunnerCallResult initialize();

  [[nodiscard]] RunnerCallResult run();

  [[nodiscard]] RunnerCallResult request_stop();

  [[nodiscard]] RunnerCallResult shutdown();

  [[nodiscard]] CommandSubmissionResult submit_latest_command(
      CommandFrame command) override;

  [[nodiscard]] RunnerCallResult report_transport_fault(
      BridgeFault fault) override;

 private:
  class Impl;

  std::unique_ptr<Impl> impl_;
};

}
