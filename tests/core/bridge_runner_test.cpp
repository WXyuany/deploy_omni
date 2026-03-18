#include <cassert>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "../../include/bridge/core/bridge_runner.hpp"

namespace {

using bridge::core::AdapterCallResult;
using bridge::core::AdapterCallStatus;
using bridge::core::AdapterHealth;
using bridge::core::AdapterHealthResult;
using bridge::core::AdapterReadResult;
using bridge::core::BridgeCommandIngress;
using bridge::core::BridgeCommandSink;
using bridge::core::BridgeFault;
using bridge::core::BridgeRunner;
using bridge::core::BridgeRunnerConfig;
using bridge::core::BridgeRunnerStatus;
using bridge::core::BridgeRuntimeEventSink;
using bridge::core::BridgeStateEgress;
using bridge::core::CommandFrame;
using bridge::core::CommandSubmissionResult;
using bridge::core::ControlMode;
using bridge::core::FaultCategory;
using bridge::core::HealthState;
using bridge::core::LifecycleState;
using bridge::core::RobotSdkAdapter;
using bridge::core::RunnerCallResult;
using bridge::core::RunnerCallStatus;
using bridge::core::StartupIdlePolicy;
using bridge::core::StateFrame;

CommandFrame MakeCommand(const std::uint64_t sequence,
                         const std::uint64_t receive_time_ns,
                         const ControlMode mode) {
  CommandFrame command;
  command.sequence = sequence;
  command.source_time_ns = receive_time_ns;
  command.receive_time_ns = receive_time_ns;
  command.mode = mode;
  command.mode_label = "test_command";
  command.valid = true;
  command.joints.position = {1.0, 2.0};
  command.joints.velocity = {0.1, 0.2};
  command.joints.effort = {0.0, 0.0};
  return command;
}

StateFrame MakeState(const std::uint64_t sequence,
                     const std::uint64_t sample_time_ns,
                     const std::uint64_t publish_time_ns) {
  StateFrame state;
  state.sequence = sequence;
  state.sample_time_ns = sample_time_ns;
  state.publish_time_ns = publish_time_ns;
  state.valid = true;
  state.active_mode = ControlMode::kSafeIdle;
  state.mode_label = "robot_state";
  state.joints.position = {0.0, 0.0};
  state.joints.velocity = {0.0, 0.0};
  state.joints.effort = {0.0, 0.0};
  state.joints.acceleration = {0.0, 0.0};
  return state;
}

class FakeRobotSdkAdapter final : public RobotSdkAdapter {
 public:
  explicit FakeRobotSdkAdapter(std::vector<StateFrame> states)
      : states_(std::move(states)) {}

  [[nodiscard]] std::string_view name() const noexcept override {
    return "fake_runner_adapter";
  }

  AdapterCallResult initialize() override {
    initialize_count_ += 1;
    initialized_ = true;
    return {.status = AdapterCallStatus::kOk,
            .detail = "initialized",
            .fault = std::nullopt};
  }

  [[nodiscard]] AdapterReadResult read_state() override {
    read_count_ += 1;
    assert(read_index_ < states_.size());
    return {
        .result = {.status = AdapterCallStatus::kOk,
                   .detail = "read",
                   .fault = std::nullopt},
        .state = states_[read_index_++],
    };
  }

  AdapterCallResult write_command(const CommandFrame& command) override {
    write_command_count_ += 1;
    last_written_command_ = command;
    return {.status = AdapterCallStatus::kOk,
            .detail = "command written",
            .fault = std::nullopt};
  }

  AdapterCallResult write_safe_command() override {
    write_safe_command_count_ += 1;
    return {.status = AdapterCallStatus::kOk,
            .detail = "safe written",
            .fault = std::nullopt};
  }

  [[nodiscard]] AdapterHealthResult query_health() const override {
    query_health_count_ += 1;
    AdapterHealth health;
    health.state = HealthState::kHealthy;
    health.safe_command_supported = true;
    health.connection_alive = initialized_;
    health.degraded = false;
    return {
        .result = {.status = AdapterCallStatus::kOk,
                   .detail = "healthy",
                   .fault = std::nullopt},
        .health = std::move(health),
    };
  }

  AdapterCallResult shutdown() override {
    shutdown_count_ += 1;
    initialized_ = false;
    return {.status = AdapterCallStatus::kOk,
            .detail = "shutdown",
            .fault = std::nullopt};
  }

  [[nodiscard]] int initialize_count() const noexcept { return initialize_count_; }
  [[nodiscard]] int read_count() const noexcept { return read_count_; }
  [[nodiscard]] int write_command_count() const noexcept {
    return write_command_count_;
  }
  [[nodiscard]] int write_safe_command_count() const noexcept {
    return write_safe_command_count_;
  }
  [[nodiscard]] int query_health_count() const noexcept {
    return query_health_count_;
  }
  [[nodiscard]] int shutdown_count() const noexcept { return shutdown_count_; }
  [[nodiscard]] const std::optional<CommandFrame>& last_written_command() const noexcept {
    return last_written_command_;
  }

 private:
  std::vector<StateFrame> states_;
  std::size_t read_index_{0};
  bool initialized_{false};
  int initialize_count_{0};
  int read_count_{0};
  int write_command_count_{0};
  int write_safe_command_count_{0};
  mutable int query_health_count_{0};
  int shutdown_count_{0};
  std::optional<CommandFrame> last_written_command_;
};

class FakeCommandIngress final : public BridgeCommandIngress {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "fake_runner_ingress";
  }

  RunnerCallResult start(BridgeCommandSink& command_sink,
                         BridgeRuntimeEventSink& event_sink) override {
    start_count_ += 1;
    started_ = true;
    command_sink_ = &command_sink;
    event_sink_ = &event_sink;
    return {.status = RunnerCallStatus::kOk,
            .detail = "ingress started",
            .fault = std::nullopt};
  }

  RunnerCallResult stop() override {
    stop_count_ += 1;
    started_ = false;
    return {.status = RunnerCallStatus::kOk,
            .detail = "ingress stopped",
            .fault = std::nullopt};
  }

  CommandSubmissionResult submit_from_callback(CommandFrame command) {
    assert(started_);
    assert(command_sink_ != nullptr);
    callback_submit_count_ += 1;
    return command_sink_->submit_latest_command(std::move(command));
  }

  RunnerCallResult report_fault_from_callback(BridgeFault fault) {
    assert(started_);
    assert(event_sink_ != nullptr);
    return event_sink_->report_transport_fault(std::move(fault));
  }

  [[nodiscard]] bool started() const noexcept { return started_; }
  [[nodiscard]] int start_count() const noexcept { return start_count_; }
  [[nodiscard]] int stop_count() const noexcept { return stop_count_; }
  [[nodiscard]] int callback_submit_count() const noexcept {
    return callback_submit_count_;
  }

 private:
  BridgeCommandSink* command_sink_{nullptr};
  BridgeRuntimeEventSink* event_sink_{nullptr};
  bool started_{false};
  int start_count_{0};
  int stop_count_{0};
  int callback_submit_count_{0};
};

class FakeStateEgress final : public BridgeStateEgress {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "fake_runner_egress";
  }

  RunnerCallResult start() override {
    start_count_ += 1;
    started_ = true;
    return {.status = RunnerCallStatus::kOk,
            .detail = "egress started",
            .fault = std::nullopt};
  }

  RunnerCallResult publish(const StateFrame& state) override {
    assert(started_);
    publish_count_ += 1;
    published_states_.push_back(state);
    return {.status = RunnerCallStatus::kOk,
            .detail = "state published",
            .fault = std::nullopt};
  }

  RunnerCallResult stop() override {
    stop_count_ += 1;
    started_ = false;
    return {.status = RunnerCallStatus::kOk,
            .detail = "egress stopped",
            .fault = std::nullopt};
  }

  [[nodiscard]] int start_count() const noexcept { return start_count_; }
  [[nodiscard]] int publish_count() const noexcept { return publish_count_; }
  [[nodiscard]] int stop_count() const noexcept { return stop_count_; }
  [[nodiscard]] const std::vector<StateFrame>& published_states() const noexcept {
    return published_states_;
  }

 private:
  bool started_{false};
  int start_count_{0};
  int publish_count_{0};
  int stop_count_{0};
  std::vector<StateFrame> published_states_;
};

BridgeRunnerConfig MakeConfig() {
  BridgeRunnerConfig config;
  config.loop_period_ns = 10;
  config.command_stale_timeout_ns = 50;
  config.startup_idle_policy = StartupIdlePolicy::kPreferSafeCommand;
  config.fault_policy.fail_stop_on_transport_fault = true;
  config.fault_policy.fail_stop_on_command_stale = true;
  config.fault_policy.fail_stop_on_adapter_degraded = false;
  config.fault_policy.write_safe_command_on_fault = true;
  config.fault_policy.continue_publishing_in_fault = true;
  return config;
}

void TestHappyPathLifecycleProgression() {
  FakeRobotSdkAdapter adapter({MakeState(1, 100, 100)});
  FakeCommandIngress ingress;
  FakeStateEgress egress;
  BridgeRunner runner(MakeConfig(), {.adapter = &adapter,
                                     .command_ingress = &ingress,
                                     .state_egress = &egress});

  const RunnerCallResult initialize = runner.initialize();
  assert(initialize.succeeded());
  assert(adapter.initialize_count() == 1);
  assert(adapter.query_health_count() == 1);
  assert(ingress.started());
  assert(ingress.start_count() == 1);
  assert(egress.start_count() == 1);

  const BridgeRunnerStatus initialized = runner.status();
  assert(initialized.lifecycle_state == LifecycleState::kWaitingForState);
  assert(initialized.startup_idle_active);
  assert(!initialized.first_state_received);
  assert(!initialized.first_command_received);

  const CommandSubmissionResult submission =
      ingress.submit_from_callback(MakeCommand(7, 90, ControlMode::kTorque));
  assert(submission.result.succeeded());
  assert(submission.stored_sequence == 7);
  assert(!submission.replaced_previous);
  assert(adapter.write_command_count() == 0);
  assert(adapter.write_safe_command_count() == 0);
  assert(ingress.callback_submit_count() == 1);

  const RunnerCallResult run = runner.run();
  assert(run.succeeded());
  assert(adapter.read_count() == 1);
  assert(adapter.query_health_count() == 2);
  assert(adapter.write_command_count() == 1);
  assert(adapter.write_safe_command_count() == 0);
  assert(adapter.last_written_command().has_value());
  assert(adapter.last_written_command()->sequence == 7);
  assert(adapter.last_written_command()->mode == ControlMode::kTorque);

  const BridgeRunnerStatus active = runner.status();
  assert(active.lifecycle_state == LifecycleState::kActive);
  assert(active.first_state_received);
  assert(active.first_command_received);
  assert(!active.startup_idle_active);
  assert(!active.command_stale);
  assert(!active.fail_stop_active);
  assert(active.cycle_count == 1);
  assert(active.command_buffer.has_latest_command);
  assert(active.command_buffer.has_active_command);
  assert(active.command_buffer.active_command_sequence == 7);
  assert(active.command_buffer.active_mode == ControlMode::kTorque);
  assert(active.last_state_publish_time_ns == 100);

  assert(egress.publish_count() == 1);
  assert(egress.published_states().size() == 1U);
  const StateFrame& published = egress.published_states().front();
  assert(published.lifecycle_state == LifecycleState::kActive);
  assert(published.active_mode == ControlMode::kSafeIdle);
  assert(!published.command_stale);
  assert(!published.fault.has_value());

  const RunnerCallResult shutdown = runner.shutdown();
  assert(shutdown.succeeded());
  assert(ingress.stop_count() == 1);
  assert(egress.stop_count() == 1);
  assert(adapter.write_safe_command_count() == 1);
  assert(adapter.shutdown_count() == 1);
  assert(runner.status().lifecycle_state == LifecycleState::kStopped);
}

void TestStaleCommandTriggersFailStop() {
  FakeRobotSdkAdapter adapter({MakeState(1, 200, 200), MakeState(2, 300, 300)});
  FakeCommandIngress ingress;
  FakeStateEgress egress;
  BridgeRunner runner(MakeConfig(), {.adapter = &adapter,
                                     .command_ingress = &ingress,
                                     .state_egress = &egress});

  const RunnerCallResult initialize = runner.initialize();
  assert(initialize.succeeded());

  const CommandSubmissionResult submission =
      ingress.submit_from_callback(MakeCommand(9, 100, ControlMode::kVelocity));
  assert(submission.result.succeeded());
  assert(adapter.write_command_count() == 0);

  const RunnerCallResult first_run = runner.run();
  assert(first_run.succeeded());
  const BridgeRunnerStatus faulted = runner.status();
  assert(faulted.lifecycle_state == LifecycleState::kFaulted);
  assert(faulted.command_stale);
  assert(faulted.fail_stop_active);
  assert(faulted.degraded);
  assert(faulted.command_buffer.has_latest_command);
  assert(!faulted.command_buffer.has_active_command);
  assert(faulted.command_buffer.latest_command_stale);
  assert(faulted.latched_fault.has_value());
  assert(faulted.latched_fault->category == FaultCategory::kCommandStale);
  assert(faulted.latched_fault->latched);
  assert(adapter.write_command_count() == 0);
  assert(adapter.write_safe_command_count() == 1);
  assert(egress.publish_count() == 1);
  assert(egress.published_states().front().lifecycle_state == LifecycleState::kActive);
  assert(!egress.published_states().front().fault.has_value());

  const RunnerCallResult second_run = runner.run();
  assert(second_run.succeeded());
  assert(adapter.write_command_count() == 0);
  assert(adapter.write_safe_command_count() == 1);
  assert(egress.publish_count() == 2);
  const StateFrame& fault_published = egress.published_states().back();
  assert(fault_published.lifecycle_state == LifecycleState::kFaulted);
  assert(fault_published.command_stale);
  assert(fault_published.degraded);
  assert(fault_published.fault.has_value());
  assert(fault_published.fault->category == FaultCategory::kCommandStale);

  const RunnerCallResult shutdown = runner.shutdown();
  assert(shutdown.succeeded());
  assert(runner.status().lifecycle_state == LifecycleState::kStopped);
}

}

int main() {
  TestHappyPathLifecycleProgression();
  TestStaleCommandTriggersFailStop();
  return 0;
}
