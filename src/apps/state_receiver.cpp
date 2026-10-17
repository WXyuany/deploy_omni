#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "../../examples/gamepad/unitree_g1_gamepad.hpp"
#include "../../include/bridge/config/bridge_config.hpp"

namespace {

constexpr auto kPrintInterval = std::chrono::milliseconds(20);
constexpr std::uint64_t kRateWindowNs = 1'000'000'000ULL;

std::atomic<bool> g_stop{false};

void handle_signal(int) { g_stop.store(true); }

[[nodiscard]] std::uint64_t monotonic_now_ns() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

[[nodiscard]] bool init_channel_factory(
    const bridge::config::BridgeStartupConfig& config) {
  try {
    if (config.adapter.connection.interface_hint.empty()) {
      unitree::robot::ChannelFactory::Instance()->Init(
          static_cast<int32_t>(config.dds.domain_id));
    } else {
      unitree::robot::ChannelFactory::Instance()->Init(
          static_cast<int32_t>(config.dds.domain_id),
          config.adapter.connection.interface_hint);
    }
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "--help") {
    std::printf("Usage: %s [config-path]\n", argv[0]);
    std::printf("Receives external robot state from dds.topics.state in config.\n");
    std::printf("Prints the latest state at 50 Hz.\n");
    return 0;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  const std::string config_path =
      argc > 1 ? std::string(argv[1]) : std::string("config/g1.sample.yaml");
  const bridge::config::BridgeConfigLoadResult loaded =
      bridge::config::load_bridge_config(config_path);
  if (!loaded.succeeded()) {
    std::fprintf(stderr, "state_receiver config load failed: %s\n",
                 loaded.detail.c_str());
    return 1;
  }
  if (!init_channel_factory(*loaded.config)) {
    std::fprintf(stderr, "state_receiver failed to initialize DDS\n");
    return 2;
  }

  std::mutex mutex;
  unitree_hg::msg::dds_::LowState_ latest;
  std::uint64_t received_count = 0;

  auto subscriber = std::make_unique<
      unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
      loaded.config->dds.topics.state);
  subscriber->InitChannel(
      [&mutex, &latest, &received_count](const void* message) {
        if (message == nullptr) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        latest = *static_cast<const unitree_hg::msg::dds_::LowState_*>(message);
        ++received_count;
      },
      1);

  std::fprintf(stderr, "state_receiver external_state=%s domain=%u iface=%s\n",
               loaded.config->dds.topics.state.c_str(),
               loaded.config->dds.domain_id,
               loaded.config->adapter.connection.interface_hint.c_str());

  std::uint64_t last_count = 0;
  std::uint64_t displayed_rate = 0;
  std::uint64_t window_start_ns = monotonic_now_ns();
  std::uint64_t last_gamepad_print_ns = window_start_ns;
  bridge::adapters::unitree_g1::G1Gamepad gamepad;
  while (!g_stop.load()) {
    std::this_thread::sleep_for(kPrintInterval);
    std::lock_guard<std::mutex> lock(mutex);
    const std::uint64_t now_ns = monotonic_now_ns();
    if (now_ns - window_start_ns >= kRateWindowNs) {
      displayed_rate = received_count - last_count;
      last_count = received_count;
      window_start_ns = now_ns;
    }
    std::fprintf(stderr,
                 "received=%llu/s total=%llu mode_pr=%u mode_machine=%u q0=%.4f dq0=%.4f rpy=(%.3f, %.3f, %.3f)\n",
                 static_cast<unsigned long long>(displayed_rate),
                 static_cast<unsigned long long>(received_count), latest.mode_pr(),
                  latest.mode_machine(), latest.motor_state().at(0).q(),
                  latest.motor_state().at(0).dq(), latest.imu_state().rpy()[0],
                  latest.imu_state().rpy()[1], latest.imu_state().rpy()[2]);
    if (now_ns - last_gamepad_print_ns >= kRateWindowNs) {
      bridge::adapters::unitree_g1::update_gamepad_from_wireless_remote(
          gamepad, latest.wireless_remote());
      const std::string gamepad_status =
          bridge::adapters::unitree_g1::format_gamepad_status(gamepad);
      std::fprintf(stderr, "%s\n", gamepad_status.c_str());
      last_gamepad_print_ns = now_ns;
    }
  }

  subscriber->CloseChannel();
  return 0;
}
