#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "../../include/bridge/config/bridge_config.hpp"

namespace {

constexpr const char* kRobotStateTopic = "custom/mujoco/rt/lowstate";
constexpr const char* kRobotImuTopic = "rt/secondary_imu";
constexpr const char* kRobotCommandTopic = "custom/mujoco/rt/lowcmd";
constexpr std::size_t kMotorCount = 35;
constexpr std::uint64_t kDefaultRateHz = 500;

std::atomic<bool> g_stop{false};

void handle_signal(int) { g_stop.store(true); }

[[nodiscard]] std::uint64_t parse_rate(int argc, char** argv) {
  if (argc < 3) {
    return kDefaultRateHz;
  }
  const std::uint64_t value = std::stoull(argv[2]);
  return value == 0 ? kDefaultRateHz : value;
}

[[nodiscard]] unitree_hg::msg::dds_::IMUState_ make_imu(
    const std::uint64_t sequence) {
  unitree_hg::msg::dds_::IMUState_ imu;
  const double phase = static_cast<double>(sequence) * 0.01;
  const float roll = static_cast<float>(0.10 * std::sin(phase));
  const float pitch = static_cast<float>(0.05 * std::cos(phase * 0.7));
  const float yaw = static_cast<float>(0.25 * std::sin(phase * 0.5));
  imu.rpy() = {roll, pitch, yaw};
  imu.quaternion() = {static_cast<float>(std::cos(yaw * 0.5F)), 0.0F, 0.0F,
                      static_cast<float>(std::sin(yaw * 0.5F))};
  imu.gyroscope() = {static_cast<float>(0.10 * std::cos(phase)),
                     static_cast<float>(-0.035 * std::sin(phase * 0.7)),
                     static_cast<float>(0.125 * std::cos(phase * 0.5))};
  imu.accelerometer() = {static_cast<float>(0.20 * std::sin(phase * 0.3)),
                         static_cast<float>(0.10 * std::cos(phase * 0.4)),
                         9.81F};
  imu.temperature() = 42;
  return imu;
}

[[nodiscard]] unitree_hg::msg::dds_::LowState_ make_state(
    const std::uint64_t sequence,
    const unitree_hg::msg::dds_::IMUState_& imu) {
  unitree_hg::msg::dds_::LowState_ state;
  state.mode_pr() = 1;
  state.mode_machine() = 5;
  state.tick() = static_cast<std::uint32_t>(sequence);
  const double phase = static_cast<double>(sequence) * 0.01;
  for (std::size_t index = 0; index < kMotorCount; ++index) {
    const float position = static_cast<float>(0.2 * std::sin(phase + index * 0.05));
    state.motor_state().at(index).q(position);
    state.motor_state().at(index).dq(static_cast<float>(0.1 * std::cos(phase)));
    state.motor_state().at(index).tau_est(0.0F);
    state.motor_state().at(index).ddq(0.0F);
  }
  state.imu_state() = imu;
  state.crc(crc32_core(reinterpret_cast<uint32_t*>(&state),
                       (sizeof(state) >> 2) - 1));
  return state;
}

void init_channel_factory(const bridge::config::BridgeStartupConfig& config) {
  if (config.adapter.connection.interface_hint.empty()) {
    unitree::robot::ChannelFactory::Instance()->Init(
        static_cast<int32_t>(config.dds.domain_id));
    return;
  }
  unitree::robot::ChannelFactory::Instance()->Init(
      static_cast<int32_t>(config.dds.domain_id),
      config.adapter.connection.interface_hint);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "--help") {
    std::printf("Usage: %s [config-path] [rate-hz]\n", argv[0]);
    std::printf("Publishes fake G1 raw state to %s and IMU to %s; listens on %s.\n",
                kRobotStateTopic, kRobotImuTopic, kRobotCommandTopic);
    return 0;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  const std::string config_path = argc > 1 ? argv[1] : "config/g1.sample.yaml";
  const auto loaded = bridge::config::load_bridge_config(config_path);
  if (!loaded.succeeded()) {
    std::fprintf(stderr, "fake_state_publisher config load failed: %s\n",
                 loaded.detail.c_str());
    return 1;
  }

  const std::uint64_t rate_hz = parse_rate(argc, argv);
  init_channel_factory(*loaded.config);

  unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowState_> publisher(
      kRobotStateTopic);
  publisher.InitChannel();

  unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::IMUState_> imu_publisher(
      kRobotImuTopic);
  imu_publisher.InitChannel();

  std::atomic<std::uint64_t> command_count{0};
  unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_> command_subscriber(
      kRobotCommandTopic);
  command_subscriber.InitChannel(
      [&command_count](const void* message) {
        if (message != nullptr) {
          command_count.fetch_add(1, std::memory_order_relaxed);
        }
      },
      1);

  std::printf("fake_state_publisher domain=%u iface=%s state_topic=%s imu_topic=%s cmd_topic=%s rate=%lluHz\n",
              loaded.config->dds.domain_id,
              loaded.config->adapter.connection.interface_hint.empty()
                  ? "<default>"
                  : loaded.config->adapter.connection.interface_hint.c_str(),
              kRobotStateTopic, kRobotImuTopic, kRobotCommandTopic,
              static_cast<unsigned long long>(rate_hz));

  std::uint64_t sequence = 0;
  std::uint64_t published_in_window = 0;
  std::uint64_t commands_at_window_start = 0;
  auto next_deadline = std::chrono::steady_clock::now();
  auto window_start = next_deadline;
  const auto period = std::chrono::nanoseconds(1'000'000'000ULL / rate_hz);

  while (!g_stop.load()) {
    const auto imu = make_imu(sequence);
    const auto state = make_state(sequence++, imu);
    publisher.Write(state);
    imu_publisher.Write(imu);
    ++published_in_window;

    const auto now = std::chrono::steady_clock::now();
    if (now - window_start >= std::chrono::seconds(1)) {
      const std::uint64_t commands = command_count.load(std::memory_order_relaxed);
      std::printf("fake_state_publisher lowstate=%llu/s imu=%llu/s lowcmd_received=%llu/s\n",
                  static_cast<unsigned long long>(published_in_window),
                  static_cast<unsigned long long>(published_in_window),
                  static_cast<unsigned long long>(commands - commands_at_window_start));
      published_in_window = 0;
      commands_at_window_start = commands;
      window_start = now;
    }

    next_deadline += period;
    std::this_thread::sleep_until(next_deadline);
  }

  command_subscriber.CloseChannel();
  imu_publisher.CloseChannel();
  publisher.CloseChannel();
  return 0;
}
