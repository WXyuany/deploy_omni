#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "../../include/bridge/core/bridge_runner.hpp"
#include "../../include/bridge/extensions/hooks.hpp"
#include "../../include/bridge/observability/forwarding_logger.hpp"

namespace {

using bridge::core::AdapterCallResult;
using bridge::core::AdapterCallStatus;
using bridge::core::AdapterHealth;
using bridge::core::AdapterHealthResult;
using bridge::core::AdapterReadResult;
using bridge::core::BridgeFault;
using bridge::core::BridgeRunner;
using bridge::core::BridgeRunnerConfig;
using bridge::core::BridgeRunnerStatus;
using bridge::core::BridgeRuntimeEventSink;
using bridge::core::BridgeGamepadRemoteSource;
using bridge::core::BridgeStateEgress;
using bridge::core::BridgeCommandIngress;
using bridge::core::BridgeCommandSink;
using bridge::core::CommandFrame;
using bridge::core::CommandSubmissionResult;
using bridge::core::ControlMode;
using bridge::core::FaultCategory;
using bridge::core::HealthState;
using bridge::core::LifecycleState;
using bridge::core::RobotAuxStateSnapshot;
using bridge::core::RobotSdkAdapter;
using bridge::core::RunnerCallResult;
using bridge::core::RunnerCallStatus;
using bridge::core::StateFrame;
using bridge::gamepad::CommonGamepad;
using bridge::extensions::HookInvocationContext;
using bridge::extensions::ObservationHook;

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

  void push_read_result(AdapterReadResult read_result) {
    queued_read_results_.push_back(std::move(read_result));
  }

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
    if (!queued_read_results_.empty()) {
      AdapterReadResult result = std::move(queued_read_results_.front());
      queued_read_results_.erase(queued_read_results_.begin());
      return result;
    }
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

  [[nodiscard]] AdapterHealthResult query_health() const override {
    query_health_count_ += 1;
    AdapterHealth health;
    health.state = HealthState::kHealthy;
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
  [[nodiscard]] int write_command_count() const noexcept { return write_command_count_; }
  [[nodiscard]] int query_health_count() const noexcept { return query_health_count_; }
  [[nodiscard]] int shutdown_count() const noexcept { return shutdown_count_; }
  [[nodiscard]] const std::optional<CommandFrame>& last_written_command() const noexcept {
    return last_written_command_;
  }

 private:
  std::vector<StateFrame> states_;
  std::vector<AdapterReadResult> queued_read_results_;
  std::size_t read_index_{0};
  bool initialized_{false};
  int initialize_count_{0};
  int read_count_{0};
  int write_command_count_{0};
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

  RunnerCallResult publish(StateFrame& state,
                           const RobotAuxStateSnapshot& aux_state) override {
    (void)aux_state;
    assert(started_);
    state.lowstate_translate_duration_ns = 11'000U;
    state.dds_write_duration_ns = 22'000U;
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

class FakeGamepadRemoteSource final : public BridgeGamepadRemoteSource {
 public:
  void set_sample(CommonGamepad gamepad) { next_sample_ = std::move(gamepad); }

  RunnerCallResult start() override {
    start_count_ += 1;
    started_ = true;
    return {.status = RunnerCallStatus::kOk,
            .detail = "gamepad started",
            .fault = std::nullopt};
  }

  std::optional<CommonGamepad> read() override {
    read_count_ += 1;
    return next_sample_;
  }

  RunnerCallResult stop() override {
    stop_count_ += 1;
    started_ = false;
    return {.status = RunnerCallStatus::kOk,
            .detail = "gamepad stopped",
            .fault = std::nullopt};
  }

  [[nodiscard]] bool started() const noexcept { return started_; }
  [[nodiscard]] int start_count() const noexcept { return start_count_; }
  [[nodiscard]] int read_count() const noexcept { return read_count_; }
  [[nodiscard]] int stop_count() const noexcept { return stop_count_; }

 private:
  bool started_{false};
  int start_count_{0};
  int read_count_{0};
  int stop_count_{0};
  std::optional<CommonGamepad> next_sample_;
};

class FakeObservationHook final : public ObservationHook {
 public:
  void observe_command(const CommandFrame& command,
                       const HookInvocationContext& context) override {
    observed_command_sequences_.push_back(command.sequence);
    observed_command_cycles_.push_back(context.cycle_index);
  }

  void observe_state(const StateFrame& state,
                     const HookInvocationContext& context) override {
    observed_state_sequences_.push_back(state.sequence);
    observed_state_cycles_.push_back(context.cycle_index);
  }

  void observe_debug(std::string_view, const HookInvocationContext&) override {}

  [[nodiscard]] const std::vector<bridge::extensions::ObservationRecord>& records()
      const noexcept override {
    return records_;
  }

  [[nodiscard]] const std::vector<std::uint64_t>& observed_command_sequences() const noexcept {
    return observed_command_sequences_;
  }
  [[nodiscard]] const std::vector<std::uint64_t>& observed_state_sequences() const noexcept {
    return observed_state_sequences_;
  }
  [[nodiscard]] const std::vector<std::uint64_t>& observed_command_cycles() const noexcept {
    return observed_command_cycles_;
  }
  [[nodiscard]] const std::vector<std::uint64_t>& observed_state_cycles() const noexcept {
    return observed_state_cycles_;
  }

 private:
  std::vector<bridge::extensions::ObservationRecord> records_;
  std::vector<std::uint64_t> observed_command_sequences_;
  std::vector<std::uint64_t> observed_state_sequences_;
  std::vector<std::uint64_t> observed_command_cycles_;
  std::vector<std::uint64_t> observed_state_cycles_;
};

BridgeRunnerConfig MakeConfig() {
  BridgeRunnerConfig config;
  config.loop_period_ns = 10;
  config.command_stale_timeout_ns = 50;
  config.fault_policy.fail_stop_on_transport_fault = true;
  config.fault_policy.fail_stop_on_command_stale = true;
  config.fault_policy.fail_stop_on_adapter_degraded = false;
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
  assert(!initialized.first_state_received);
  assert(!initialized.first_command_received);

  const CommandSubmissionResult submission =
      ingress.submit_from_callback(MakeCommand(7, 90, ControlMode::kTorque));
  assert(submission.result.succeeded());
  assert(submission.stored_sequence == 7);
  assert(!submission.replaced_previous);
  assert(adapter.write_command_count() == 1);
  assert(adapter.last_written_command().has_value());
  assert(adapter.last_written_command()->sequence == 7);
  assert(adapter.last_written_command()->mode == ControlMode::kTorque);
  assert(ingress.callback_submit_count() == 1);

  const RunnerCallResult run = runner.run();
  assert(run.succeeded());
  assert(adapter.read_count() == 1);
  assert(adapter.query_health_count() == 2);
  assert(adapter.write_command_count() == 1);
  assert(adapter.last_written_command().has_value());
  assert(adapter.last_written_command()->sequence == 7);
  assert(adapter.last_written_command()->mode == ControlMode::kTorque);

  const BridgeRunnerStatus active = runner.status();
  assert(active.lifecycle_state == LifecycleState::kActive);
  assert(active.first_state_received);
  assert(active.first_command_received);
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
  assert(published.active_mode == ControlMode::kTorque);
  assert(!published.command_stale);
  assert(!published.fault.has_value());

  const RunnerCallResult shutdown = runner.shutdown();
  assert(shutdown.succeeded());
  assert(ingress.stop_count() == 1);
  assert(egress.stop_count() == 1);
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
  assert(adapter.write_command_count() == 1);

  const RunnerCallResult first_run = runner.run();
  assert(first_run.succeeded());
  const BridgeRunnerStatus faulted = runner.status();
  assert(faulted.lifecycle_state == LifecycleState::kFaulted);
  assert(faulted.command_stale);
  assert(faulted.fail_stop_active);
  assert(faulted.degraded);
  assert(faulted.command_buffer.has_latest_command);
  assert(faulted.command_buffer.has_active_command);
  assert(faulted.command_buffer.latest_command_stale);
  assert(faulted.latched_fault.has_value());
  assert(faulted.latched_fault->category == FaultCategory::kCommandStale);
  assert(faulted.latched_fault->latched);
  assert(adapter.write_command_count() == 1);
  assert(egress.publish_count() == 1);
  assert(egress.published_states().front().lifecycle_state == LifecycleState::kActive);
  assert(!egress.published_states().front().fault.has_value());

  const RunnerCallResult second_run = runner.run();
  assert(second_run.succeeded());
  assert(adapter.write_command_count() == 1);
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

void TestWaitingForInitialStateDoesNotFailStopRunner() {
  FakeRobotSdkAdapter adapter({MakeState(1, 100, 100)});
  adapter.push_read_result({
      .result = {.status = AdapterCallStatus::kRetryableError,
                 .detail = "waiting for /hardware/joint_state",
                 .fault = std::nullopt},
      .state = std::nullopt,
  });
  FakeCommandIngress ingress;
  FakeStateEgress egress;
  BridgeRunner runner(MakeConfig(), {.adapter = &adapter,
                                     .command_ingress = &ingress,
                                     .state_egress = &egress});

  const RunnerCallResult initialize = runner.initialize();
  assert(initialize.succeeded());

  const RunnerCallResult waiting_run = runner.run();
  assert(waiting_run.succeeded());
  assert(waiting_run.detail == "waiting for /hardware/joint_state");

  const BridgeRunnerStatus waiting = runner.status();
  assert(waiting.lifecycle_state == LifecycleState::kWaitingForState);
  assert(!waiting.first_state_received);
  assert(!waiting.fail_stop_active);
  assert(!waiting.latched_fault.has_value());
  assert(adapter.write_command_count() == 0);
  assert(egress.publish_count() == 0);

  const RunnerCallResult active_run = runner.run();
  assert(active_run.succeeded());
  assert(runner.status().lifecycle_state == LifecycleState::kActive);
  assert(runner.status().first_state_received);

  const RunnerCallResult shutdown = runner.shutdown();
  assert(shutdown.succeeded());
}

void TestRobotStateUnavailableSkipsPublishAndResumesWithoutFailStop() {
  FakeRobotSdkAdapter adapter({MakeState(2, 200, 200)});
  adapter.push_read_result({
      .result = {.status = AdapterCallStatus::kOk,
                 .detail = "read",
                 .fault = std::nullopt},
      .state = MakeState(1, 100, 100),
  });
  adapter.push_read_result({
      .result = {.status = AdapterCallStatus::kRetryableError,
                 .detail = "pm01 robot state not available",
                 .fault = std::nullopt},
      .state = std::nullopt,
  });
  FakeCommandIngress ingress;
  FakeStateEgress egress;
  BridgeRunner runner(MakeConfig(), {.adapter = &adapter,
                                     .command_ingress = &ingress,
                                     .state_egress = &egress});

  assert(runner.initialize().succeeded());

  const RunnerCallResult first_run = runner.run();
  assert(first_run.succeeded());
  assert(runner.status().lifecycle_state == LifecycleState::kActive);
  assert(egress.publish_count() == 1);

  const RunnerCallResult timeout_run = runner.run();
  assert(timeout_run.succeeded());
  assert(timeout_run.status == RunnerCallStatus::kDegraded);
  assert(timeout_run.detail == "pm01 robot state not available");
  assert(runner.status().lifecycle_state == LifecycleState::kActive);
  assert(!runner.status().fail_stop_active);
  assert(!runner.status().latched_fault.has_value());
  assert(egress.publish_count() == 1);

  const RunnerCallResult resumed_run = runner.run();
  assert(resumed_run.succeeded());
  assert(runner.status().lifecycle_state == LifecycleState::kActive);
  assert(!runner.status().fail_stop_active);
  assert(egress.publish_count() == 2);

  const RunnerCallResult shutdown = runner.shutdown();
  assert(shutdown.succeeded());
}

void TestG1NoFreshLowStateSkipsPublishAndResumesWithoutFailStop() {
  FakeRobotSdkAdapter adapter({MakeState(2, 200, 200)});
  adapter.push_read_result({
      .result = {.status = AdapterCallStatus::kOk,
                 .detail = "read",
                 .fault = std::nullopt},
      .state = MakeState(1, 100, 100),
  });
  adapter.push_read_result({
      .result = {.status = AdapterCallStatus::kRetryableError,
                 .detail = "g1 low-level state not yet available after read retries",
                 .fault = std::nullopt},
      .state = std::nullopt,
  });
  FakeCommandIngress ingress;
  FakeStateEgress egress;
  BridgeRunner runner(MakeConfig(), {.adapter = &adapter,
                                     .command_ingress = &ingress,
                                     .state_egress = &egress});

  assert(runner.initialize().succeeded());

  const RunnerCallResult first_run = runner.run();
  assert(first_run.succeeded());
  assert(runner.status().lifecycle_state == LifecycleState::kActive);
  assert(egress.publish_count() == 1);

  const RunnerCallResult no_fresh_run = runner.run();
  assert(no_fresh_run.succeeded());
  assert(no_fresh_run.status == RunnerCallStatus::kDegraded);
  assert(no_fresh_run.detail == "g1 low-level state not yet available after read retries");
  assert(runner.status().lifecycle_state == LifecycleState::kActive);
  assert(!runner.status().fail_stop_active);
  assert(!runner.status().latched_fault.has_value());
  assert(egress.publish_count() == 1);

  const RunnerCallResult resumed_run = runner.run();
  assert(resumed_run.succeeded());
  assert(runner.status().lifecycle_state == LifecycleState::kActive);
  assert(!runner.status().fail_stop_active);
  assert(egress.publish_count() == 2);

  const RunnerCallResult shutdown = runner.shutdown();
  assert(shutdown.succeeded());
}

void TestObservationHookSeesPublishedStateAndEffectiveCommand() {
  FakeRobotSdkAdapter adapter({MakeState(1, 100, 100)});
  FakeCommandIngress ingress;
  FakeStateEgress egress;
  FakeObservationHook observation;
  BridgeRunner runner(MakeConfig(), {.adapter = &adapter,
                                     .command_ingress = &ingress,
                                     .state_egress = &egress,
                                     .observation_hook = &observation});

  assert(runner.initialize().succeeded());
  assert(ingress.submit_from_callback(MakeCommand(11, 90, ControlMode::kTorque)).result.succeeded());
  assert(runner.run().succeeded());

  assert(observation.observed_state_sequences() == std::vector<std::uint64_t>({1}));
  assert(observation.observed_command_sequences() == std::vector<std::uint64_t>({11}));
  assert(observation.observed_state_cycles() == std::vector<std::uint64_t>({1}));
  assert(observation.observed_command_cycles() == std::vector<std::uint64_t>({1}));
}

void TestFreshGamepadRemoteOverlaysPublishedWirelessRemote() {
  StateFrame adapter_state = MakeState(1, 100, 100);
  adapter_state.wireless_remote[0] = 0x80U;
  FakeRobotSdkAdapter adapter({adapter_state});
  FakeCommandIngress ingress;
  FakeStateEgress egress;
  FakeGamepadRemoteSource gamepad_remote;
  CommonGamepad gamepad;
  gamepad.A.update(true);
  gamepad.lx = 1.0F;
  gamepad_remote.set_sample(gamepad);

  BridgeRunnerConfig config = MakeConfig();
  config.gamepad_poll_interval_ns = 1'000'000;
  config.gamepad_stale_timeout_ns = 1'000'000'000;
  BridgeRunner runner(config, {.adapter = &adapter,
                               .command_ingress = &ingress,
                               .state_egress = &egress,
                               .gamepad_remote = &gamepad_remote});

  assert(runner.initialize().succeeded());
  for (int attempt = 0; attempt < 100 && gamepad_remote.read_count() == 0; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  assert(gamepad_remote.started());
  assert(gamepad_remote.read_count() > 0);

  assert(runner.run().succeeded());

  assert(egress.publish_count() == 1);
  const StateFrame& published = egress.published_states().front();
  float lx = 0.0F;
  std::memcpy(&lx, published.wireless_remote.data() + 4, sizeof(lx));
  assert(published.wireless_remote[0] == 0xFEU);
  assert(published.wireless_remote[1] == 0xEFU);
  assert((published.wireless_remote[2] & 0x01U) != 0U);
  assert(lx == 1.0F);

  assert(runner.shutdown().succeeded());
  assert(gamepad_remote.stop_count() == 1);
}

void TestRunnerAnnotatesPublishTimingBreakdown() {
  StateFrame state = MakeState(1, 100, 100);
  state.receive_time_ns = 10;
  state.read_start_time_ns = 20;
  state.read_complete_time_ns = 30;
  state.lowstate_callback_store_duration_ns = 5;
  state.read_snapshot_duration_ns = 6;
  state.stateframe_build_duration_ns = 7;
  FakeRobotSdkAdapter adapter({state});
  FakeCommandIngress ingress;
  FakeStateEgress egress;
  BridgeRunner runner(MakeConfig(), {.adapter = &adapter,
                                     .command_ingress = &ingress,
                                     .state_egress = &egress});

  assert(runner.initialize().succeeded());
  assert(runner.run().succeeded());
  assert(egress.publish_count() == 1);

  const StateFrame& published = egress.published_states().front();
  assert(published.publish_start_time_ns != 0);
  assert(published.lowstate_translate_duration_ns == 11'000U);
  assert(published.dds_write_duration_ns == 22'000U);
}

void TestForwardingLogDisabledSkipsRunnerTelemetry() {
  std::remove("logs/g1_forwarding.log");
  std::remove("logs/g1_forwarding.1.log");
  std::remove("logs/g1_forwarding.2.log");
  std::remove("logs/g1_forwarding.3.log");

  StateFrame state = MakeState(1, 100, 100);
  state.receive_time_ns = 10;
  state.read_start_time_ns = 20;
  state.read_complete_time_ns = 30;
  FakeRobotSdkAdapter adapter({state});
  FakeCommandIngress ingress;
  FakeStateEgress egress;
  BridgeRunnerConfig config = MakeConfig();
  config.forwarding_log_enabled = false;
  BridgeRunner runner(config, {.adapter = &adapter,
                               .command_ingress = &ingress,
                               .state_egress = &egress});

  assert(runner.initialize().succeeded());
  assert(runner.run().succeeded());
  bridge::observability::forwarding_logger()->flush();

  std::ifstream log("logs/g1_forwarding.log");
  const std::string output((std::istreambuf_iterator<char>(log)),
                           std::istreambuf_iterator<char>());
  assert(output.find("[adapter_state_forward]") == std::string::npos);
  assert(output.find("[adapter_latency_breakdown]") == std::string::npos);
}

void TestStateAvailableNotificationWakesEventWaiter() {
  FakeRobotSdkAdapter adapter({MakeState(1, 100, 100)});
  FakeCommandIngress ingress;
  FakeStateEgress egress;
  BridgeRunner runner(MakeConfig(), {.adapter = &adapter,
                                     .command_ingress = &ingress,
                                     .state_egress = &egress});

  bool woke = false;
  std::thread waiter([&]() {
    woke = runner.wait_for_state_available_until(
        std::chrono::steady_clock::now() + std::chrono::seconds(1));
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  assert(!woke);
  runner.notify_state_available();
  waiter.join();
  assert(woke);
}

}

int main() {
  TestHappyPathLifecycleProgression();
  TestStaleCommandTriggersFailStop();
  TestWaitingForInitialStateDoesNotFailStopRunner();
  TestRobotStateUnavailableSkipsPublishAndResumesWithoutFailStop();
  TestG1NoFreshLowStateSkipsPublishAndResumesWithoutFailStop();
  TestObservationHookSeesPublishedStateAndEffectiveCommand();
  TestFreshGamepadRemoteOverlaysPublishedWirelessRemote();
  TestRunnerAnnotatesPublishTimingBreakdown();
  TestForwardingLogDisabledSkipsRunnerTelemetry();
  TestStateAvailableNotificationWakesEventWaiter();
  return 0;
}
