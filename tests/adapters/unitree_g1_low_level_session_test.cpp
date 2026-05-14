#include <cassert>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <cmath>
#include <string>
#include <thread>

#include <unistd.h>

#include "../../include/bridge/adapters/unitree_g1/unitree_g1_low_level_session.hpp"
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

namespace {

using bridge::adapters::unitree_g1::LowLevelSessionConfig;
using bridge::adapters::unitree_g1::UnitreeG1LowLevelSession;
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
  state.mode_pr() = 7;
  state.mode_machine() = 19;
  state.tick() = 1234;
  for (std::size_t i = 0; i < 29; ++i) {
    state.motor_state().at(i).q(static_cast<float>(i) + 0.5F);
    state.motor_state().at(i).dq(static_cast<float>(i) + 1.5F);
    state.motor_state().at(i).tau_est(static_cast<float>(i) + 2.5F);
    state.motor_state().at(i).ddq(static_cast<float>(i) + 3.5F);
  }
  state.imu_state().quaternion() = {1.0F, 0.0F, 0.0F, 0.0F};
  state.imu_state().gyroscope() = {0.1F, 0.2F, 0.3F};
  state.imu_state().accelerometer() = {1.1F, 1.2F, 1.3F};
  state.imu_state().rpy() = {0.4F, 0.5F, 0.6F};
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
  assert(read.state->position.size() == 29U);
  AssertClose(read.state->position.at(0), 0.5);
  AssertClose(read.state->velocity.at(1), 2.5);
  AssertClose(read.state->effort.at(2), 4.5);
  AssertClose(read.state->acceleration.at(3), 6.5);
  assert(read.state->imu.has_value());
  AssertClose(read.state->imu->orientation_xyzw.at(0), 1.0);
  AssertClose(read.state->imu->angular_velocity_xyz.at(1), 0.2);
  AssertClose(read.state->imu->linear_acceleration_xyz.at(2), 1.3);
  assert(read.state->wireless_remote.size() == state.wireless_remote().size());
  assert(read.state->wireless_remote.at(0) == 1U);
  assert(read.state->wireless_remote.at(39) == 40U);
}

void TestReadStateRequiresCachedLowState() {
  UnitreeG1LowLevelSession session(MakeConfig());
  assert(session.initialize().status == AdapterCallStatus::kOk);

  const auto read = session.read_state();
  assert(read.result.status == AdapterCallStatus::kRetryableError);
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
  session.handle_imu_message(imu);

  const auto read = session.read_state();
  assert(read.result.status == AdapterCallStatus::kOk);
  assert(read.state.has_value());
  assert(read.state->imu.has_value());
  AssertClose(read.state->imu->orientation_xyzw.at(1), 1.0);
  AssertClose(read.state->imu->angular_velocity_xyz.at(0), 9.1);
  AssertClose(read.state->imu->linear_acceleration_xyz.at(2), 8.3);
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

void TestWriteCommandPrintsCommandInfoOncePerSecond() {
  auto config = MakeConfig();
  config.write_timeout_ns = 500'000'000ULL;
  UnitreeG1LowLevelSession session(config);
  assert(session.initialize().status == AdapterCallStatus::kOk);

  const char* log_path = "/tmp/opencode/g1_write_command_print_test.log";
  std::remove(log_path);
  const int saved_stderr = dup(fileno(stderr));
  assert(saved_stderr >= 0);
  assert(freopen(log_path, "w", stderr) != nullptr);

  auto command = MakeCommand();
  assert(session.write_command(command).status == AdapterCallStatus::kOk);
  assert(session.write_command(command).status == AdapterCallStatus::kOk);
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  assert(session.write_command(command).status == AdapterCallStatus::kOk);
  std::fflush(stderr);

  assert(dup2(saved_stderr, fileno(stderr)) >= 0);
  close(saved_stderr);

  std::ifstream log(log_path);
  const std::string output((std::istreambuf_iterator<char>(log)),
                           std::istreambuf_iterator<char>());
  assert(CountOccurrences(output, "[UnitreeG1LowLevelSession.write_command] ========== Command ==========") == 2U);
  assert(output.find("sequence=0 mode_label=torque mode_pr=5 mode_machine=99 joints=29") !=
         std::string::npos);
  assert(output.find("idx        q        dq       tau        kp        kd") !=
         std::string::npos);
  assert(output.find("  0   0.2500   0.5000   0.7500  10.0000   1.5000") !=
         std::string::npos);
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

void TestReadStateReportsStaleWhenCachedStateAgesOut() {
  auto config = MakeConfig();
  config.stale_state_after_ns = 30;
  UnitreeG1LowLevelSession session(config);
  assert(session.initialize().status == AdapterCallStatus::kOk);

  LowState_ state;
  FillLowState(state);
  session.handle_low_state_message(state);
  assert(session.read_state().result.status == AdapterCallStatus::kOk);
  session.handle_low_state_message(state);

  for (int i = 0; i < 10; ++i) {
    const auto read = session.read_state();
    if (read.state.has_value() && read.state->stale) {
      assert(read.result.status == AdapterCallStatus::kDegraded);
      assert(read.state->degraded);
      return;
    }
  }

  assert(false && "expected stale/degraded state after cached lowstate ages out");
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
  session.handle_low_state_message(state);

  for (int i = 0; i < 10; ++i) {
    const auto read = session.read_state();
    if (read.state.has_value() && read.state->stale) {
      const auto degraded_read = session.read_state();
      assert(degraded_read.state.has_value());
      assert(degraded_read.state->stale);
      const auto health = session.health();
      assert(health.state == bridge::core::HealthState::kDegraded);
      assert(health.degraded);
      assert(health.connection_alive);
      assert(health.fault.has_value());
      return;
    }
  }

  assert(false && "expected degraded health after cached lowstate ages out");
}

}

int main() {
  TestReadStateUsesInjectedLowStateCache();
  TestReadStateRequiresCachedLowState();
  TestReadStateRetriesUntilLowStateArrives();
  TestExplicitImuTopicOverridesEmbeddedImu();
  TestWriteCommandPacksLowCmdUsingCommandModeMachineAndMotorModes();
  TestWriteCommandPrintsCommandInfoOncePerSecond();
  TestInvalidCrcLowStateIsRejectedWhenVerificationEnabled();
  TestInitializeFailsWhenMotionReleaseCannotBeSatisfied();
  TestInitializeSucceedsWhenMotionReleaseCompletes();
  TestInitializeFailsWhenMotionModeStaysActiveAfterReleaseAttempt();
  TestReadStateReportsStaleWhenCachedStateAgesOut();
  TestHealthReportsDegradedWhenCachedStateGoesStale();
  return 0;
}
