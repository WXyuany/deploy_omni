#include <cassert>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <cmath>
#include <string>
#include <thread>

#include <unistd.h>

#include "../../include/bridge/adapters/unitree_g1/unitree_g1_low_level_session.hpp"
#include "../../include/bridge/observability/forwarding_logger.hpp"
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

namespace {

using bridge::adapters::unitree_g1::LowLevelSessionConfig;
using bridge::adapters::unitree_g1::UnitreeG1LowLevelSession;
using bridge::adapters::unitree_g1::select_reader_loop_sleep_ns;
using bridge::core::AdapterCallStatus;
using bridge::core::CommandFrame;
using bridge::core::ControlMode;
using unitree_hg::msg::dds_::IMUState_;
using unitree_hg::msg::dds_::LowState_;

LowLevelSessionConfig MakeConfig() {
  LowLevelSessionConfig config;
  config.transport_mode = "mock";
  config.robot_id = "g1-test";
  config.transport_endpoint = "mock:robot_lan0";
  config.control_profile = "raw_low_level";
  config.command_topic = "rt/lowcmd";
  config.state_topic = "rt/lowstate";
  config.imu_topic = "rt/secondary_imu";
  config.initialize_timeout_ns = 10;
  config.read_timeout_ns = 20;
  config.read_retry_attempts = 0;
  config.read_retry_interval_ns = 0;
  config.write_timeout_ns = 30;
  config.stale_state_after_ns = 100;
  config.require_motion_release = false;
  config.hardware_slot_count = 29;
  return config;
}

LowLevelSessionConfig MakeStrictConfig() {
  auto config = MakeConfig();
  config.verify_low_state_crc = true;
  return config;
}

LowLevelSessionConfig MakeBlockedMotionConfig() {
  auto config = MakeConfig();
  config.require_motion_release = true;
  config.check_mode_hook = [](std::string& form, std::string& name) {
    form = "motion";
    name = "ai";
    return 0;
  };
  config.release_mode_hook = []() { return -1; };
  return config;
}

LowLevelSessionConfig MakeReleasedMotionConfig() {
  auto config = MakeConfig();
  config.require_motion_release = true;
  auto active = std::make_shared<bool>(true);
  config.check_mode_hook = [active](std::string& form, std::string& name) {
    form = *active ? "motion" : "";
    name = *active ? "ai" : "";
    return 0;
  };
  config.release_mode_hook = [active]() {
    *active = false;
    return 0;
  };
  return config;
}

LowLevelSessionConfig MakeStickyMotionConfig() {
  auto config = MakeConfig();
  config.require_motion_release = true;
  config.check_mode_hook = [](std::string& form, std::string& name) {
    form = "motion";
    name = "ai";
    return 0;
  };
  config.release_mode_hook = []() { return 0; };
  return config;
}

void AssertClose(const double actual, const double expected,
                 const double tolerance = 1e-6) {
  assert(std::fabs(actual - expected) <= tolerance);
}

CommandFrame MakeCommand() {
  CommandFrame command;
  command.valid = true;
  command.mode = ControlMode::kTorque;
  command.mode_label = "torque";
  command.mode_pr = 5;
  command.mode_machine = 99;
  command.joints.position.assign(29, 0.25);
  command.joints.velocity.assign(29, 0.5);
  command.joints.effort.assign(29, 0.75);
  command.joints.stiffness.assign(29, 10.0);
  command.joints.damping.assign(29, 1.5);
  command.joints.motor_mode.assign(29, 1U);
  command.joints.motor_mode[0] = 7U;
  command.joints.motor_mode[1] = 0U;
  return command;
}

void FillLowState(LowState_& state) {
  state.version() = {3U, 4U};
  state.mode_pr() = 7;
  state.mode_machine() = 19;
  state.tick() = 1234;
  state.reserve() = {101U, 102U, 103U, 104U};
  for (std::size_t i = 0; i < 29; ++i) {
    auto& motor = state.motor_state().at(i);
    motor.mode(static_cast<std::uint8_t>(i % 3));
    motor.q(static_cast<float>(i) + 0.5F);
    motor.dq(static_cast<float>(i) + 1.5F);
    motor.tau_est(static_cast<float>(i) + 2.5F);
    motor.ddq(static_cast<float>(i) + 3.5F);
    motor.temperature() = {static_cast<std::int16_t>(30 + i),
                           static_cast<std::int16_t>(40 + i)};
    motor.vol(static_cast<float>(50 + i));
    motor.sensor() = {static_cast<std::uint32_t>(60 + i),
                      static_cast<std::uint32_t>(70 + i)};
    motor.motorstate(static_cast<std::uint32_t>(80 + i));
    motor.reserve() = {static_cast<std::uint32_t>(90 + i),
                       static_cast<std::uint32_t>(100 + i),
                       static_cast<std::uint32_t>(110 + i),
                       static_cast<std::uint32_t>(120 + i)};
  }
  state.imu_state().quaternion() = {1.0F, 0.0F, 0.0F, 0.0F};
  state.imu_state().gyroscope() = {0.1F, 0.2F, 0.3F};
  state.imu_state().accelerometer() = {1.1F, 1.2F, 1.3F};
  state.imu_state().rpy() = {0.4F, 0.5F, 0.6F};
  state.imu_state().temperature(55);
  for (std::size_t i = 0; i < state.wireless_remote().size(); ++i) {
    state.wireless_remote().at(i) = static_cast<uint8_t>(i + 1);
  }
}

void TestReadStateUsesInjectedLowStateCache() {
  UnitreeG1LowLevelSession session(MakeConfig());
  assert(session.initialize().status == AdapterCallStatus::kOk);

  LowState_ state;
  FillLowState(state);
  session.handle_low_state_message(state);

  const auto read = session.read_state();
  assert(read.result.status == AdapterCallStatus::kOk);
  assert(read.state.has_value());
  assert(read.state->mode_pr == 7);
  assert(read.state->mode_machine == 19);
  assert(read.state->tick == 1234U);
  assert(read.state->low_state_version[0] == 3U);
  assert(read.state->low_state_version[1] == 4U);
  assert(read.state->low_state_reserve[2] == 103U);
  assert(read.state->position.size() == 29U);
  AssertClose(read.state->position.at(0), 0.5);
  AssertClose(read.state->velocity.at(1), 2.5);
  AssertClose(read.state->effort.at(2), 4.5);
  AssertClose(read.state->acceleration.at(3), 6.5);
  assert(read.state->motor_mode.at(2) == 2U);
  assert(read.state->motor_temperature.at(3)[0] == 33);
  assert(read.state->motor_temperature.at(3)[1] == 43);
  AssertClose(read.state->motor_voltage.at(4), 54.0);
  assert(read.state->motor_sensor.at(5)[0] == 65U);
  assert(read.state->motor_sensor.at(5)[1] == 75U);
  assert(read.state->motor_state.at(6) == 86U);
  assert(read.state->motor_reserve.at(7)[3] == 127U);
  assert(read.state->imu.has_value());
  AssertClose(read.state->imu->orientation_xyzw.at(3), 1.0);
  AssertClose(read.state->imu->angular_velocity_xyz.at(1), 0.2);
  AssertClose(read.state->imu->linear_acceleration_xyz.at(2), 1.3);
  assert(read.state->imu->temperature == 55);
  assert(read.state->wireless_remote.size() == state.wireless_remote().size());
  assert(read.state->wireless_remote.at(0) == 1U);
  assert(read.state->wireless_remote.at(39) == 40U);
}

void TestReadStateConsumesFreshLowStateOnce() {
  UnitreeG1LowLevelSession session(MakeConfig());
  assert(session.initialize().status == AdapterCallStatus::kOk);

  LowState_ state;
  FillLowState(state);
  session.handle_low_state_message(state);

  const auto first = session.read_state();
  assert(first.result.status == AdapterCallStatus::kOk);
  assert(first.state.has_value());
  assert(first.state->receive_generation == 1U);

  const auto second = session.read_state();
  assert(second.result.status == AdapterCallStatus::kRetryableError);
  assert(!second.state.has_value());

  session.handle_low_state_message(state);
  const auto third = session.read_state();
  assert(third.result.status == AdapterCallStatus::kOk);
  assert(third.state.has_value());
  assert(third.state->receive_generation == 2U);
}

void TestReadStateSkipsBufferedOlderStateWhenNewerLowStateArrives() {
  auto config = MakeConfig();
  config.read_timeout_ns = 1'000'000;
  config.stale_state_after_ns = 0;
  UnitreeG1LowLevelSession session(config);
  assert(session.initialize().status == AdapterCallStatus::kOk);

  LowState_ first_state;
  FillLowState(first_state);
  first_state.tick(100U);
  session.handle_low_state_message(first_state);

  std::this_thread::sleep_for(std::chrono::milliseconds(3));

  LowState_ second_state;
  FillLowState(second_state);
  second_state.tick(200U);
  session.handle_low_state_message(second_state);

  const auto read = session.read_state();
  assert(read.result.status == AdapterCallStatus::kOk);
  assert(read.state.has_value());
  assert(read.state->receive_generation == 2U);
  assert(read.state->tick == 200U);
}

void TestReadStateRequiresCachedLowState() {
  UnitreeG1LowLevelSession session(MakeConfig());
  assert(session.initialize().status == AdapterCallStatus::kOk);

  const auto read = session.read_state();
  assert(read.result.status == AdapterCallStatus::kRetryableError);
  assert(read.result.detail == "waiting for rt/lowstate");
  assert(!read.state.has_value());
}

void TestReadStateRetriesUntilLowStateArrives() {
  auto config = MakeConfig();
  config.read_retry_attempts = 5;
  config.read_retry_interval_ns = 1'000'000;
  UnitreeG1LowLevelSession session(config);
  assert(session.initialize().status == AdapterCallStatus::kOk);

  std::thread delayed_state([&session]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    LowState_ state;
    FillLowState(state);
    session.handle_low_state_message(state);
  });

  const auto read = session.read_state();
  delayed_state.join();
  assert(read.result.status == AdapterCallStatus::kOk);
  assert(read.state.has_value());
}

void TestReadStateDoesNotBlockOnConsumedLowStateRetryInterval() {
  auto config = MakeConfig();
  config.read_retry_attempts = 1;
  config.read_retry_interval_ns = 50'000'000;
  UnitreeG1LowLevelSession session(config);
  assert(session.initialize().status == AdapterCallStatus::kOk);

  LowState_ state;
  FillLowState(state);
  session.handle_low_state_message(state);
  assert(session.read_state().result.status == AdapterCallStatus::kOk);

  const auto start = std::chrono::steady_clock::now();
  const auto read = session.read_state();
  const auto elapsed = std::chrono::steady_clock::now() - start;
  assert(read.result.status == AdapterCallStatus::kRetryableError);
  assert(!read.state.has_value());
  assert(elapsed < std::chrono::milliseconds(10));
}

void TestExplicitImuTopicOverridesEmbeddedImu() {
  UnitreeG1LowLevelSession session(MakeConfig());
  assert(session.initialize().status == AdapterCallStatus::kOk);

  LowState_ state;
  FillLowState(state);
  session.handle_low_state_message(state);

  IMUState_ imu;
  imu.quaternion() = {0.0F, 1.0F, 0.0F, 0.0F};
  imu.gyroscope() = {9.1F, 9.2F, 9.3F};
  imu.accelerometer() = {8.1F, 8.2F, 8.3F};
  imu.rpy() = {7.1F, 7.2F, 7.3F};
  imu.temperature(66);
  session.handle_imu_message(imu);

  const auto read = session.read_state();
  assert(read.result.status == AdapterCallStatus::kOk);
  assert(read.state.has_value());
  assert(read.state->imu.has_value());
  AssertClose(read.state->imu->orientation_xyzw.at(0), 1.0);
  AssertClose(read.state->imu->angular_velocity_xyz.at(0), 9.1);
  AssertClose(read.state->imu->linear_acceleration_xyz.at(2), 8.3);
  assert(read.state->imu->temperature == 66);
}

void TestWriteCommandPacksLowCmdUsingCommandModeMachineAndMotorModes() {
  UnitreeG1LowLevelSession session(MakeConfig());
  assert(session.initialize().status == AdapterCallStatus::kOk);

  LowState_ state;
  FillLowState(state);
  state.mode_machine() = 33;
  session.handle_low_state_message(state);

  const auto write = session.write_command(MakeCommand());
  assert(write.status == AdapterCallStatus::kOk);
  assert(session.last_packed_command().has_value());
  const auto& low_cmd = *session.last_packed_command();
  assert(low_cmd.mode_pr() == 5);
  assert(low_cmd.mode_machine() == 99);
  assert(low_cmd.motor_cmd().at(0).mode() == 7);
  assert(low_cmd.motor_cmd().at(1).mode() == 0);
  AssertClose(low_cmd.motor_cmd().at(0).q(), 0.25);
  AssertClose(low_cmd.motor_cmd().at(0).dq(), 0.5);
  AssertClose(low_cmd.motor_cmd().at(0).tau(), 0.75);
  AssertClose(low_cmd.motor_cmd().at(0).kp(), 10.0);
  AssertClose(low_cmd.motor_cmd().at(0).kd(), 1.5);
  assert(low_cmd.crc() != 0U);
}

std::size_t CountOccurrences(const std::string& text, const std::string& needle) {
  std::size_t count = 0;
  std::size_t position = 0;
  while ((position = text.find(needle, position)) != std::string::npos) {
    ++count;
    position += needle.size();
  }
  return count;
}

void TestWriteCommandLogsOnlyAggregateTelemetry() {
  auto config = MakeConfig();
  config.write_timeout_ns = 500'000'000ULL;
  UnitreeG1LowLevelSession session(config);
  assert(session.initialize().status == AdapterCallStatus::kOk);

  std::remove("logs/g1_forwarding.log");
  std::remove("logs/g1_forwarding.1.log");
  std::remove("logs/g1_forwarding.2.log");
  std::remove("logs/g1_forwarding.3.log");

  auto command = MakeCommand();
  command.sequence = static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
  assert(session.write_command(command).status == AdapterCallStatus::kOk);
  assert(session.write_command(command).status == AdapterCallStatus::kOk);
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  assert(session.write_command(command).status == AdapterCallStatus::kOk);
  bridge::observability::forwarding_logger()->flush();

  std::ifstream log("logs/g1_forwarding.log");
  const std::string output((std::istreambuf_iterator<char>(log)),
                           std::istreambuf_iterator<char>());
  assert(output.find("[g1_cmd_sample]") == std::string::npos);
}

void TestForwardingLogDisabledSkipsSessionTelemetry() {
  auto config = MakeConfig();
  config.forwarding_log_enabled = false;
  config.write_timeout_ns = 500'000'000ULL;
  UnitreeG1LowLevelSession session(config);
  assert(session.initialize().status == AdapterCallStatus::kOk);

  std::remove("logs/g1_forwarding.log");
  std::remove("logs/g1_forwarding.1.log");
  std::remove("logs/g1_forwarding.2.log");
  std::remove("logs/g1_forwarding.3.log");

  auto command = MakeCommand();
  command.sequence = static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
  assert(session.write_command(command).status == AdapterCallStatus::kOk);
  assert(session.write_command(command).status == AdapterCallStatus::kOk);
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  assert(session.write_command(command).status == AdapterCallStatus::kOk);
  bridge::observability::forwarding_logger()->flush();

  std::ifstream log("logs/g1_forwarding.log");
  const std::string output((std::istreambuf_iterator<char>(log)),
                           std::istreambuf_iterator<char>());
  assert(output.find("[g1_cmd_forward]") == std::string::npos);
}

void TestReaderLoopSleepFollowsConfiguredReadPeriod() {
  assert(select_reader_loop_sleep_ns(2'000'000ULL) == 2'000'000ULL);
  assert(select_reader_loop_sleep_ns(1'000'000ULL) == 1'000'000ULL);
  assert(select_reader_loop_sleep_ns(0ULL) == 1'000'000ULL);
}

void TestInvalidCrcLowStateIsRejectedWhenVerificationEnabled() {
  UnitreeG1LowLevelSession session(MakeStrictConfig());
  assert(session.initialize().status == AdapterCallStatus::kOk);

  LowState_ state;
  FillLowState(state);
  state.crc() = 1U;
  session.handle_low_state_message(state);

  const auto read = session.read_state();
  assert(read.result.status == AdapterCallStatus::kRetryableError);
  assert(!read.state.has_value());
}

void TestInitializeFailsWhenMotionReleaseCannotBeSatisfied() {
  UnitreeG1LowLevelSession session(MakeBlockedMotionConfig());
  const auto init = session.initialize();
  assert(init.status == AdapterCallStatus::kRetryableError);
}

void TestInitializeSucceedsWhenMotionReleaseCompletes() {
  UnitreeG1LowLevelSession session(MakeReleasedMotionConfig());
  const auto init = session.initialize();
  assert(init.status == AdapterCallStatus::kOk);
}

void TestInitializeFailsWhenMotionModeStaysActiveAfterReleaseAttempt() {
  UnitreeG1LowLevelSession session(MakeStickyMotionConfig());
  const auto init = session.initialize();
  assert(init.status == AdapterCallStatus::kRetryableError);
}

void TestReadStateDoesNotReturnCachedStateWhenItAgesOut() {
  auto config = MakeConfig();
  config.stale_state_after_ns = 30;
  UnitreeG1LowLevelSession session(config);
  assert(session.initialize().status == AdapterCallStatus::kOk);

  LowState_ state;
  FillLowState(state);
  session.handle_low_state_message(state);
  assert(session.read_state().result.status == AdapterCallStatus::kOk);

  for (int i = 0; i < 10; ++i) {
    const auto read = session.read_state();
    assert(read.result.status == AdapterCallStatus::kRetryableError);
    assert(!read.state.has_value());
  }
}

void TestReadStateStopsReturningCachedLowStateAfterReceiveTimeout() {
  UnitreeG1LowLevelSession session(MakeConfig());
  assert(session.initialize().status == AdapterCallStatus::kOk);

  LowState_ state;
  FillLowState(state);
  session.handle_low_state_message(state);
  assert(session.read_state().result.status == AdapterCallStatus::kOk);

  std::this_thread::sleep_for(std::chrono::milliseconds(550));

  const auto timed_out = session.read_state();
  assert(timed_out.result.status == AdapterCallStatus::kRetryableError);
  assert(!timed_out.state.has_value());

  session.handle_low_state_message(state);
  const auto resumed = session.read_state();
  assert(resumed.result.status == AdapterCallStatus::kOk);
  assert(resumed.state.has_value());
}

void TestHealthReportsDegradedWhenCachedStateGoesStale() {
  auto config = MakeConfig();
  config.stale_state_after_ns = 30;
  UnitreeG1LowLevelSession session(config);
  assert(session.initialize().status == AdapterCallStatus::kOk);

  LowState_ state;
  FillLowState(state);
  session.handle_low_state_message(state);
  assert(session.read_state().result.status == AdapterCallStatus::kOk);

  for (int i = 0; i < 10; ++i) {
    const auto read = session.read_state();
    if (read.result.status == AdapterCallStatus::kRetryableError &&
        !read.state.has_value()) {
      const auto health = session.health();
      if (health.degraded) {
        assert(health.state == bridge::core::HealthState::kDegraded);
        assert(health.connection_alive);
        assert(health.fault.has_value());
        return;
      }
    }
  }

  assert(false && "expected degraded health after cached lowstate ages out");
}

}

int main() {
  TestReadStateUsesInjectedLowStateCache();
  TestReadStateConsumesFreshLowStateOnce();
  TestReadStateSkipsBufferedOlderStateWhenNewerLowStateArrives();
  TestReadStateRequiresCachedLowState();
  TestReadStateRetriesUntilLowStateArrives();
  TestReadStateDoesNotBlockOnConsumedLowStateRetryInterval();
  TestExplicitImuTopicOverridesEmbeddedImu();
  TestWriteCommandPacksLowCmdUsingCommandModeMachineAndMotorModes();
  TestWriteCommandLogsOnlyAggregateTelemetry();
  TestForwardingLogDisabledSkipsSessionTelemetry();
  TestReaderLoopSleepFollowsConfiguredReadPeriod();
  TestInvalidCrcLowStateIsRejectedWhenVerificationEnabled();
  TestInitializeFailsWhenMotionReleaseCannotBeSatisfied();
  TestInitializeSucceedsWhenMotionReleaseCompletes();
  TestInitializeFailsWhenMotionModeStaysActiveAfterReleaseAttempt();
  TestReadStateDoesNotReturnCachedStateWhenItAgesOut();
  TestReadStateStopsReturningCachedLowStateAfterReceiveTimeout();
  TestHealthReportsDegradedWhenCachedStateGoesStale();
  return 0;
}
