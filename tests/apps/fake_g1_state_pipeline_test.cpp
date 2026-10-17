#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include "../../include/bridge/adapters/unitree_g1/unitree_g1_low_level_session.hpp"
#include "../../include/bridge/config/joint_map.hpp"
#include "../../include/bridge/dds/dds_translation.hpp"

namespace {

using bridge::adapters::unitree_g1::LowLevelSessionConfig;
using bridge::adapters::unitree_g1::UnitreeG1LowLevelSession;
using bridge::config::JointFieldTransform;
using bridge::config::JointMap;
using bridge::config::JointMapEntry;
using bridge::config::JointMapValidationOptions;
using bridge::core::AdapterCallStatus;
using bridge::core::ControlMode;
using bridge::core::LifecycleState;
using bridge::core::StateFrame;
using bridge::dds::translate_hardware_state_to_lowstate;
using unitree_hg::msg::dds_::IMUState_;
using unitree_hg::msg::dds_::LowState_;

constexpr std::size_t kHardwareSlotCount = 29;
constexpr std::size_t kFrameCount = 500;

LowLevelSessionConfig MakeSessionConfig() {
  LowLevelSessionConfig config;
  config.transport_mode = "mock";
  config.robot_id = "g1-fake-pipeline";
  config.command_topic = "custom/mujoco/rt/lowcmd";
  config.state_topic = "custom/mujoco/rt/lowstate";
  config.imu_topic = "rt/secondary_imu";
  config.initialize_timeout_ns = 1;
  config.read_timeout_ns = 2'000'000;
  config.read_retry_attempts = 0;
  config.read_retry_interval_ns = 0;
  config.write_timeout_ns = 2'000'000;
  config.stale_state_after_ns = 200'000'000;
  config.require_motion_release = false;
  config.hardware_slot_count = kHardwareSlotCount;
  return config;
}

JointMap MakeIdentityJointMap() {
  std::vector<JointMapEntry> entries;
  entries.reserve(kHardwareSlotCount);
  for (std::size_t index = 0; index < kHardwareSlotCount; ++index) {
    JointMapEntry entry;
    entry.sim_index = index;
    entry.hardware_index = index;
    entry.sim_name = "sim_" + std::to_string(index);
    entry.hardware_name = "hw_" + std::to_string(index);
    entry.transforms.position = JointFieldTransform{};
    entry.transforms.velocity = JointFieldTransform{};
    entry.transforms.effort = JointFieldTransform{};
    entry.transforms.stiffness = JointFieldTransform{};
    entry.transforms.damping = JointFieldTransform{};
    entry.transforms.acceleration = JointFieldTransform{};
    entries.push_back(std::move(entry));
  }
  JointMapValidationOptions options;
  const auto built = JointMap::build(std::move(entries), options);
  assert(built.succeeded());
  return *built.joint_map;
}

IMUState_ MakeImu(const std::uint64_t sequence) {
  IMUState_ imu;
  const double phase = static_cast<double>(sequence) * 0.01;
  const float roll = static_cast<float>(0.1 * std::sin(phase));
  const float pitch = static_cast<float>(0.1 * std::cos(phase));
  const float yaw = static_cast<float>(0.2 * std::sin(phase * 0.5));
  imu.quaternion() = {1.0F, 0.0F, 0.0F, yaw};
  imu.rpy() = {roll, pitch, yaw};
  imu.gyroscope() = {0.01F, 0.02F, 0.03F};
  imu.accelerometer() = {0.0F, 0.0F, 9.81F};
  imu.temperature() = 42;
  return imu;
}

LowState_ MakeLowState(const std::uint64_t sequence, const IMUState_& imu) {
  LowState_ state;
  state.mode_pr() = 1;
  state.mode_machine() = 5;
  state.tick() = static_cast<std::uint32_t>(sequence);
  const double phase = static_cast<double>(sequence) * 0.01;
  for (std::size_t index = 0; index < kHardwareSlotCount; ++index) {
    auto& motor = state.motor_state().at(index);
    motor.q(static_cast<float>(0.2 * std::sin(phase + index * 0.05)));
    motor.dq(static_cast<float>(0.1 * std::cos(phase)));
    motor.tau_est(static_cast<float>(0.01 * index));
    motor.ddq(static_cast<float>(0.02 * std::sin(phase)));
  }
  state.imu_state() = imu;
  return state;
}

StateFrame MakeHardwareState(const bridge::adapters::unitree_g1::LowLevelSessionState& state,
                             const std::uint64_t sequence) {
  StateFrame frame;
  frame.sequence = sequence;
  frame.tick = state.tick;
  frame.sample_time_ns = state.sample_time_ns;
  frame.publish_time_ns = state.sample_time_ns;
  frame.mode_pr = state.mode_pr;
  frame.mode_machine = state.mode_machine;
  frame.lifecycle_state = LifecycleState::kActive;
  frame.active_mode = ControlMode::kTorque;
  frame.valid = state.valid;
  frame.stale = state.stale;
  frame.degraded = state.degraded;
  frame.joints.position = state.position;
  frame.joints.velocity = state.velocity;
  frame.joints.effort = state.effort;
  frame.joints.acceleration = state.acceleration;
  frame.joints.motor_mode = state.motor_mode;
  frame.joints.motor_temperature = state.motor_temperature;
  frame.joints.motor_voltage = state.motor_voltage;
  frame.joints.motor_sensor = state.motor_sensor;
  frame.joints.motor_state = state.motor_state;
  frame.joints.motor_reserve = state.motor_reserve;
  frame.wireless_remote = state.wireless_remote;
  frame.imu = state.imu;
  return frame;
}

}  // namespace

int main() {
  UnitreeG1LowLevelSession session(MakeSessionConfig());
  const JointMap joint_map = MakeIdentityJointMap();
  assert(session.initialize().status == AdapterCallStatus::kOk);

  std::size_t converted = 0;
  double previous_roll = 0.0;
  bool roll_changed = false;
  const auto started = std::chrono::steady_clock::now();

  for (std::uint64_t sequence = 0; sequence < kFrameCount; ++sequence) {
    const auto imu = MakeImu(sequence);
    const auto low_state = MakeLowState(sequence, imu);
    session.handle_imu_message(imu);
    session.handle_low_state_message(low_state);

    std::this_thread::sleep_for(std::chrono::microseconds(2500));

    const auto read = session.read_state();
    assert(read.result.status == AdapterCallStatus::kOk);
    assert(read.state.has_value());
    assert(read.state->imu.has_value());

    const StateFrame frame = MakeHardwareState(*read.state, sequence);
    const auto translated = translate_hardware_state_to_lowstate(frame, joint_map);
    assert(translated.result.succeeded());
    assert(translated.state.has_value());
    assert(translated.state->motor_state().at(0).q() == low_state.motor_state().at(0).q());

    const double roll = translated.state->imu_state().rpy()[0];
    if (converted > 0 && roll != previous_roll) {
      roll_changed = true;
    }
    previous_roll = roll;
    ++converted;
  }

  const auto elapsed = std::chrono::steady_clock::now() - started;
  const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  std::printf("fake_g1_state_pipeline converted=%zu elapsed_ms=%lld\n", converted,
              static_cast<long long>(elapsed_ms));

  assert(converted == kFrameCount);
  assert(roll_changed);
  assert(elapsed < std::chrono::seconds(3));
  assert(session.shutdown().status == AdapterCallStatus::kOk);
  return 0;
}
