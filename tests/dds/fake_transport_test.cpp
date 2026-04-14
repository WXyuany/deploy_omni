#include <cassert>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "../../include/bridge/config/joint_map.hpp"
#include "../../include/bridge/dds/dds_command_ingress.hpp"
#include "../../include/bridge/dds/dds_state_egress.hpp"
#include "../../include/bridge/dds/dds_factory.hpp"
#include "../../include/bridge/dds/dds_translation.hpp"

namespace {

using bridge::config::JointFieldTransform;
using bridge::config::JointMap;
using bridge::config::JointMapEntry;
using bridge::config::JointMapValidationOptions;
using bridge::core::AdapterCallResult;
using bridge::core::AdapterCallStatus;
using bridge::core::AdapterHealth;
using bridge::core::AdapterHealthResult;
using bridge::core::AdapterReadResult;
using bridge::core::BridgeRunnerConfig;
using bridge::core::CommandFrame;
using bridge::core::ControlMode;
using bridge::core::HealthState;
using bridge::core::LifecycleState;
using bridge::core::RobotSdkAdapter;
using bridge::core::RunnerCallStatus;
using bridge::core::StartupIdlePolicy;
using bridge::core::StateFrame;
using bridge::dds::DdsCommandIngressDependencies;
using bridge::dds::DdsRuntime;
using bridge::dds::DdsStateEgress;
using bridge::dds::DdsStateEgressDependencies;
using bridge::dds::StateTranslation;
using bridge::dds::StatusTranslation;
using bridge::dds::CommandTranslation;
using bridge::dds::create_dds_assembly;
using bridge::dds::translate_hardware_state_to_bridge_status;
using bridge::dds::translate_hardware_state_to_lowstate;
using bridge::dds::translate_lowcmd_to_command;

constexpr double kTolerance = 1e-9;

void AssertNear(const double actual, const double expected) {
  assert(std::fabs(actual - expected) < kTolerance);
}

void AssertVectorNear(const std::vector<double>& actual,
                      const std::vector<double>& expected) {
  assert(actual.size() == expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    AssertNear(actual[index], expected[index]);
  }
}

JointMap MakeJointMap() {
  std::vector<JointMapEntry> entries;
  for (std::size_t index = 0; index < 2U; ++index) {
    JointMapEntry entry;
    entry.sim_index = index;
    entry.hardware_index = index;
    entry.sim_name = std::string("sim_") + std::to_string(index);
    entry.hardware_name = std::string("hw_") + std::to_string(index);
    entry.transforms.position = JointFieldTransform{};
    entry.transforms.velocity = JointFieldTransform{};
    entry.transforms.effort = JointFieldTransform{};
    entry.transforms.stiffness = JointFieldTransform{};
    entry.transforms.damping = JointFieldTransform{};
    entry.transforms.acceleration = JointFieldTransform{};
    entries.push_back(std::move(entry));
  }

  JointMapValidationOptions options;
  options.expected_sim_joint_count = 2U;
  options.expected_hardware_joint_count = 2U;
  const auto built = JointMap::build(std::move(entries), options);
  assert(built.succeeded());
  return *built.joint_map;
}

bridge_wire::LowCmd MakeCommandMessage() {
  bridge_wire::LowCmd message;
  message.seq_id(42U);
  message.source_time_ns(123456U);
  message.control_mode(5U);
  message.mode_label("hybrid");
  message.valid(true);
  message.stale(false);
  message.degraded(true);
  message.position({1.25, -2.5});
  message.velocity({0.5, -0.75});
  message.effort({3.0, -4.0});
  message.stiffness({10.0, 20.0});
  message.damping({1.0, 2.0});
  return message;
}

StateFrame MakeStateFrame() {
  StateFrame state;
  state.sequence = 77U;
  state.sample_time_ns = 9000U;
  state.publish_time_ns = 9015U;
  state.lifecycle_state = LifecycleState::kActive;
  state.active_mode = ControlMode::kHybrid;
  state.mode_label = "hybrid_tracking";
  state.valid = true;
  state.command_stale = true;
  state.joints.position = {0.1, -0.2};
  state.joints.velocity = {1.0, -1.5};
  state.joints.effort = {2.5, -3.5};
  state.joints.acceleration = {0.25, -0.5};
  state.imu = bridge::core::ImuSample{
      .sample_time_ns = 9005U,
      .valid = true,
      .stale = false,
      .orientation_xyzw = {0.0, 0.0, 0.70710678, 0.70710678},
      .angular_velocity_xyz = {0.01, 0.02, 0.03},
      .linear_acceleration_xyz = {0.0, 0.1, 9.81},
  };
  return state;
}

bridge::config::BridgeStartupConfig MakeStartupConfig() {
  bridge::config::BridgeStartupConfig config;
  config.robot_type = "kepler_k2";
  config.dds.domain_id = 0U;
  config.dds.topics.command = "bridge/command";
  config.dds.topics.state = "bridge/state";
  config.dds.topics.status = "bridge/status";
  config.adapter.kind = "kepler_k2";
  config.adapter.robot_name = "kepler_k2";
  config.adapter.command_mode = "torque_pd_passthrough";
  config.adapter.connection.transport = "udp";
  config.adapter.connection.interface_hint = "robot_lan0";
  config.adapter.startup.read_initial_state_timeout_ns = 1'000'000'000U;
  config.adapter.startup.require_healthy_hardware = true;
  config.adapter.safety.safe_command_profile = "zero_torque";
  config.adapter.safety.allow_safe_command_during_shutdown = true;
  config.runner.loop_period_ns = 4'000'000U;
  config.runner.command_stale_timeout_ns = 120'000'000U;
  config.runner.startup_idle_policy = StartupIdlePolicy::kPreferSafeCommand;
  return config;
}

class FakeRobotSdkAdapter final : public RobotSdkAdapter {
 public:
  explicit FakeRobotSdkAdapter(StateFrame state) : state_(std::move(state)) {}

  [[nodiscard]] std::string_view name() const noexcept override { return "fake_adapter"; }

  AdapterCallResult initialize() override {
    return {.status = AdapterCallStatus::kOk, .detail = "initialized", .fault = std::nullopt};
  }

  [[nodiscard]] AdapterReadResult read_state() override {
    return {
        .result = {.status = AdapterCallStatus::kOk, .detail = "read", .fault = std::nullopt},
        .state = state_,
    };
  }

  AdapterCallResult write_command(const CommandFrame& command) override {
    last_command_ = command;
    return {.status = AdapterCallStatus::kOk, .detail = "written", .fault = std::nullopt};
  }

  AdapterCallResult write_safe_command() override {
    return {.status = AdapterCallStatus::kOk, .detail = "safe", .fault = std::nullopt};
  }

  [[nodiscard]] AdapterHealthResult query_health() const override {
    return {
        .result = {.status = AdapterCallStatus::kOk, .detail = "healthy", .fault = std::nullopt},
        .health = {.state = HealthState::kHealthy,
                   .safe_command_supported = true,
                   .connection_alive = true,
                   .degraded = false,
                   .last_read_time_ns = state_.sample_time_ns,
                   .last_write_time_ns = state_.publish_time_ns,
                   .fault = std::nullopt},
    };
  }

  AdapterCallResult shutdown() override {
    return {.status = AdapterCallStatus::kOk, .detail = "shutdown", .fault = std::nullopt};
  }

  [[nodiscard]] const std::optional<CommandFrame>& last_command() const noexcept {
    return last_command_;
  }

 private:
  StateFrame state_;
  std::optional<CommandFrame> last_command_;
};

class RecordingCommandSink final : public bridge::core::BridgeCommandSink {
 public:
  bridge::core::CommandSubmissionResult submit_latest_command(CommandFrame command) override {
    last_command = std::move(command);
    return {
        .result = {.status = RunnerCallStatus::kOk, .detail = "stored", .fault = std::nullopt},
        .replaced_previous = false,
        .stored_sequence = last_command->sequence,
    };
  }

  std::optional<CommandFrame> last_command;
};

class RecordingEventSink final : public bridge::core::BridgeRuntimeEventSink {
 public:
  bridge::core::RunnerCallResult report_transport_fault(bridge::core::BridgeFault fault) override {
    last_fault = std::move(fault);
    return {.status = RunnerCallStatus::kOk, .detail = "reported", .fault = std::nullopt};
  }

  std::optional<bridge::core::BridgeFault> last_fault;
};

void TestDecodeTypedLowCmdMessage() {
  const CommandTranslation decoded =
      translate_lowcmd_to_command(MakeCommandMessage(), 123999U);
  assert(decoded.result.succeeded());
  assert(decoded.command.has_value());
  assert(decoded.command->sequence == 42U);
  assert(decoded.command->mode == ControlMode::kHybrid);
  assert(decoded.command->mode_label == "hybrid");
  AssertVectorNear(decoded.command->joints.position, {1.25, -2.5});
  AssertVectorNear(decoded.command->joints.velocity, {0.5, -0.75});
  AssertVectorNear(decoded.command->joints.effort, {3.0, -4.0});
}

void TestEncodeTypedStatePublications() {
  JointMap joint_map = MakeJointMap();
  const StateTranslation encoded_state =
      translate_hardware_state_to_lowstate(MakeStateFrame(), joint_map);
  assert(encoded_state.result.succeeded());
  assert(encoded_state.state.has_value());

  const bridge_wire::LowState& state_msg = *encoded_state.state;
  assert(state_msg.seq_id() == 77U);
  assert(state_msg.active_mode() == 5U);
  AssertVectorNear(state_msg.position(), {0.1, -0.2});

  const StatusTranslation encoded_status =
      translate_hardware_state_to_bridge_status(MakeStateFrame(), joint_map);
  assert(encoded_status.result.succeeded());
  assert(encoded_status.status.has_value());

  const bridge_wire::BridgeStatus& status_msg = *encoded_status.status;
  assert(status_msg.seq_id() == 77U);
  assert(status_msg.control_mode() == 5U);
  assert(status_msg.command_stale());
}

void TestDdsFactoryCreatesAssembly() {
  JointMap joint_map = MakeJointMap();
  auto assembly = create_dds_assembly(MakeStartupConfig(), joint_map);
  assert(assembly.runtime != nullptr);
  assert(assembly.runtime->command_topic != nullptr);
  assert(assembly.runtime->state_topic != nullptr);
  assert(assembly.runtime->status_topic != nullptr);
  assert(assembly.runtime->command_reader != nullptr);
  assert(assembly.runtime->state_writer != nullptr);
  assert(assembly.runtime->status_writer != nullptr);
  assert(assembly.command_ingress != nullptr);
  assert(assembly.state_egress != nullptr);
}

void TestDdsFactoryUsesConfiguredRuntimeTopicsAndDomain() {
  JointMap joint_map = MakeJointMap();
  auto config = MakeStartupConfig();
  config.dds.domain_id = 7U;
  config.dds.topics.command = "kepler_k2/low_level_command";
  config.dds.topics.state = "kepler_k2/low_level_state";
  config.dds.topics.status = "kepler_k2/bridge_status";

  auto assembly = create_dds_assembly(config, joint_map);
  assert(assembly.runtime != nullptr);
  assert(assembly.runtime->participant != nullptr);
  assert(assembly.runtime->participant->get_domain_id() == 7U);
  assert(assembly.runtime->command_topic != nullptr);
  assert(assembly.runtime->state_topic != nullptr);
  assert(assembly.runtime->status_topic != nullptr);
  assert(assembly.runtime->command_topic->get_name() == "kepler_k2/low_level_command");
  assert(assembly.runtime->state_topic->get_name() == "kepler_k2/low_level_state");
  assert(assembly.runtime->status_topic->get_name() == "kepler_k2/bridge_status");
}

void TestCommandIngressStartFailsWithoutReader() {
  bridge::dds::DdsCommandIngress ingress(DdsCommandIngressDependencies{
      .runtime = std::make_shared<DdsRuntime>(),
      .joint_map = MakeJointMap(),
  });
  RecordingCommandSink command_sink;
  RecordingEventSink event_sink;

  const auto result = ingress.start(command_sink, event_sink);
  assert(result.status == RunnerCallStatus::kFatalError);
}

void TestStateEgressStartFailsWithoutWriters() {
  DdsStateEgress egress(DdsStateEgressDependencies{
      .runtime = std::make_shared<DdsRuntime>(),
      .joint_map = MakeJointMap(),
  });

  const auto result = egress.start();
  assert(result.status == RunnerCallStatus::kFatalError);
}

void TestStateEgressRejectsInvalidPublishedState() {
  auto runtime = bridge::dds::make_dds_runtime();
  DdsStateEgress egress(DdsStateEgressDependencies{
      .runtime = runtime,
      .joint_map = MakeJointMap(),
  });
  assert(egress.start().succeeded());

  auto invalid_state = MakeStateFrame();
  invalid_state.imu = bridge::core::ImuSample{
      .sample_time_ns = 9005U,
      .valid = true,
      .stale = false,
      .orientation_xyzw = {1.0, 0.0, 0.0},
      .angular_velocity_xyz = {0.01, 0.02, 0.03},
      .linear_acceleration_xyz = {0.0, 0.1, 9.81},
  };

  const auto result = egress.publish(invalid_state);
  assert(result.status == RunnerCallStatus::kRejected);
}

}

int main() {
  TestDecodeTypedLowCmdMessage();
  TestEncodeTypedStatePublications();
  TestDdsFactoryCreatesAssembly();
  TestDdsFactoryUsesConfiguredRuntimeTopicsAndDomain();
  TestCommandIngressStartFailsWithoutReader();
  TestStateEgressStartFailsWithoutWriters();
  TestStateEgressRejectsInvalidPublishedState();
  return 0;
}
