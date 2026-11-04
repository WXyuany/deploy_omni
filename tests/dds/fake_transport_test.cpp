#include <cassert>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <unitree/dds_wrapper/common/crc.h>
#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"

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
using bridge::core::CommandFrame;
using bridge::core::ControlMode;
using bridge::core::HealthState;
using bridge::core::LifecycleState;
using bridge::core::RobotSdkAdapter;
using bridge::core::RunnerCallStatus;
using bridge::core::StateFrame;
using bridge::dds::CommandTranslation;
using bridge::dds::DdsCommandIngressDependencies;
using bridge::dds::DdsRuntime;
using bridge::dds::DdsStateEgress;
using bridge::dds::DdsStateEgressDependencies;
using bridge::dds::StateTranslation;
using bridge::dds::create_dds_assembly;
using bridge::dds::translate_hardware_state_to_lowstate;
using bridge::dds::translate_lowcmd_to_command;
using unitree_hg::msg::dds_::LowCmd_;
using unitree_hg::msg::dds_::LowState_;

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

LowCmd_ MakeCommandMessage() {
  LowCmd_ message;
  message.mode_machine(5U);
  auto reserve = message.reserve();
  reserve[0] = 42U;
  reserve[1] = 123456U;
  message.reserve(reserve);

  auto motors = message.motor_cmd();
  motors[0].q(1.25F);
  motors[0].dq(0.5F);
  motors[0].tau(3.0F);
  motors[0].kp(10.0F);
  motors[0].kd(1.0F);
  motors[1].q(-2.5F);
  motors[1].dq(-0.75F);
  motors[1].tau(-4.0F);
  motors[1].kp(20.0F);
  motors[1].kd(2.0F);
  message.motor_cmd(motors);
  message.crc(crc32_core(reinterpret_cast<uint32_t*>(&message),
                         (sizeof(message) >> 2) - 1));
  return message;
}

StateFrame MakeStateFrame() {
  StateFrame state;
  state.sequence = 77U;
  state.tick = 1234U;
  state.sample_time_ns = 9000U;
  state.publish_time_ns = 9015U;
  state.mode_pr = 8U;
  state.mode_machine = 5U;
  state.low_state_version = {3U, 4U};
  state.low_state_reserve = {101U, 102U, 103U, 104U};
  state.lifecycle_state = LifecycleState::kActive;
  state.active_mode = ControlMode::kHybrid;
  state.mode_label = "hybrid_tracking";
  state.valid = true;
  state.command_stale = true;
  state.joints.position = {0.1, -0.2};
  state.joints.velocity = {1.0, -1.5};
  state.joints.effort = {2.5, -3.5};
  state.joints.acceleration = {0.25, -0.5};
  state.joints.motor_mode = {6U, 7U};
  state.joints.motor_temperature = {{{31, 41}}, {{32, 42}}};
  state.joints.motor_voltage = {51.0, 52.0};
  state.joints.motor_sensor = {{{61U, 71U}}, {{62U, 72U}}};
  state.joints.motor_state = {81U, 82U};
  state.joints.motor_reserve = {{{91U, 101U, 111U, 121U}},
                                {{92U, 102U, 112U, 122U}}};
  state.wireless_remote[0] = 0x21U;
  state.wireless_remote[39] = 0x40U;
  state.imu = bridge::core::ImuSample{
      .sample_time_ns = 9005U,
      .valid = true,
      .stale = false,
      .orientation_xyzw = {0.0, 0.0, 0.70710678, 0.70710678},
      .rpy_xyz = {0.11, 0.12, 0.13},
      .angular_velocity_xyz = {0.01, 0.02, 0.03},
      .linear_acceleration_xyz = {0.0, 0.1, 9.81},
      .temperature = 55,
  };
  return state;
}

bridge::config::BridgeStartupConfig MakeStartupConfig() {
  bridge::config::BridgeStartupConfig config;
  config.robot_type = "kepler_k2";
  config.dds.domain_id = 0U;
  config.dds.topics.command = "rt/lowcmd";
  config.dds.topics.state = "rt/lowstate";
  config.dds.topics.status = "bridge/status";
  config.adapter.kind = "kepler_k2";
  config.adapter.robot_name = "kepler_k2";
  config.adapter.command_mode = "torque_pd_passthrough";
  config.adapter.connection.transport = "udp";
  config.adapter.connection.interface_hint = "robot_lan0";
  config.adapter.startup.read_initial_state_timeout_ns = 1'000'000'000U;
  config.adapter.startup.require_healthy_hardware = true;
  config.runner.loop_period_ns = 4'000'000U;
  config.runner.command_stale_timeout_ns = 120'000'000U;
  return config;
}

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

  void notify_state_available() override {}

  std::optional<bridge::core::BridgeFault> last_fault;
};

void TestDecodeTypedLowCmdMessage() {
  const CommandTranslation decoded =
      translate_lowcmd_to_command(MakeCommandMessage(), 123999U);
  assert(decoded.result.succeeded());
  assert(decoded.command.has_value());
  assert(decoded.command->sequence == 42U);
  assert(decoded.command->mode == ControlMode::kHybrid);
  assert(decoded.command->joints.position.size() == 35U);
  AssertNear(decoded.command->joints.position[0], 1.25);
  AssertNear(decoded.command->joints.position[1], -2.5);
  AssertNear(decoded.command->joints.effort[0], 3.0);
}

void TestEncodeTypedStatePublications() {
  JointMap joint_map = MakeJointMap();
  const StateTranslation encoded_state =
      translate_hardware_state_to_lowstate(MakeStateFrame(), joint_map);
  assert(encoded_state.result.succeeded());
  assert(encoded_state.state.has_value());

  const auto& state_msg = *encoded_state.state;
  assert(state_msg.tick() == 1234U);
  assert(state_msg.mode_pr() == 8U);
  assert(state_msg.mode_machine() == 5U);
  assert(state_msg.version()[0] == 3U);
  assert(state_msg.reserve()[2] == 103U);
  assert(state_msg.motor_state()[0].mode() == 6U);
  assert(state_msg.motor_state()[0].q() == 0.1F);
  assert(state_msg.motor_state()[1].q() == -0.2F);
  assert(state_msg.motor_state()[1].temperature()[1] == 42);
  assert(state_msg.motor_state()[1].vol() == 52.0F);
  assert(state_msg.motor_state()[1].sensor()[0] == 62U);
  assert(state_msg.motor_state()[1].motorstate() == 82U);
  assert(state_msg.motor_state()[1].reserve()[3] == 122U);
  assert(state_msg.wireless_remote()[0] == 0x21U);
  assert(state_msg.wireless_remote()[39] == 0x40U);
  assert(state_msg.imu_state().quaternion()[0] == 0.70710677F ||
         state_msg.imu_state().quaternion()[0] == 0.70710678F);
  assert(state_msg.imu_state().rpy()[0] == 0.11F);
  assert(state_msg.imu_state().rpy()[1] == 0.12F);
  assert(state_msg.imu_state().rpy()[2] == 0.13F);
  assert(state_msg.imu_state().temperature() == 55);
  auto crc_check = state_msg;
  assert(state_msg.crc() == crc32_core(reinterpret_cast<uint32_t*>(&crc_check),
                                       (sizeof(LowState_) >> 2) - 1));
}

void TestEncodeTypedStateKeepsZeroTick() {
  JointMap joint_map = MakeJointMap();
  auto state = MakeStateFrame();
  state.sequence = 88U;
  state.tick = 0U;

  const StateTranslation encoded_state =
      translate_hardware_state_to_lowstate(state, joint_map);
  assert(encoded_state.result.succeeded());
  assert(encoded_state.state.has_value());
  assert(encoded_state.state->tick() == 0U);
}

void TestEncodeTypedStateKeepsZeroModeMachine() {
  JointMap joint_map = MakeJointMap();
  auto state = MakeStateFrame();
  state.mode_machine = 0U;
  state.active_mode = ControlMode::kHybrid;

  const StateTranslation encoded_state =
      translate_hardware_state_to_lowstate(state, joint_map);
  assert(encoded_state.result.succeeded());
  assert(encoded_state.state.has_value());
  assert(encoded_state.state->mode_machine() == 0U);
}

void TestDdsFactoryCreatesAssembly() {
  JointMap joint_map = MakeJointMap();
  auto assembly = create_dds_assembly(MakeStartupConfig(), joint_map);
  assert(assembly.runtime != nullptr);
  assert(assembly.runtime->ready());
  assert(assembly.runtime->has_command_reader());
  assert(assembly.runtime->has_state_writer());
  assert(!assembly.runtime->has_status_writer());
  assert(assembly.command_ingress != nullptr);
  assert(assembly.state_egress != nullptr);
}

void TestDdsFactoryUsesConfiguredRuntimeTopicsAndDomain() {
  JointMap joint_map = MakeJointMap();
  auto config = MakeStartupConfig();
  config.dds.domain_id = 7U;
  config.dds.topics.command = "rt/robot/lowcmd";
  config.dds.topics.state = "rt/robot/lowstate";

  auto assembly = create_dds_assembly(config, joint_map);
  assert(assembly.runtime != nullptr);
  assert(assembly.runtime->config.domain_id == 7U);
  assert(assembly.runtime->config.command_topic == "rt/robot/lowcmd");
  assert(assembly.runtime->config.state_topic == "rt/robot/lowstate");
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
      .rpy_xyz = {0.0, 0.0, 0.0},
      .angular_velocity_xyz = {0.01, 0.02, 0.03},
      .linear_acceleration_xyz = {0.0, 0.1, 9.81},
  };

  const auto result = egress.publish(invalid_state, bridge::core::RobotAuxStateSnapshot{});
  assert(result.status == RunnerCallStatus::kRejected);
}

}  // namespace

int main() {
  TestDecodeTypedLowCmdMessage();
  TestEncodeTypedStatePublications();
  TestEncodeTypedStateKeepsZeroTick();
  TestEncodeTypedStateKeepsZeroModeMachine();
  TestDdsFactoryCreatesAssembly();
  TestDdsFactoryUsesConfiguredRuntimeTopicsAndDomain();
  TestCommandIngressStartFailsWithoutReader();
  TestStateEgressStartFailsWithoutWriters();
  TestStateEgressRejectsInvalidPublishedState();
  return 0;
}
