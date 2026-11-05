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

#include "bridge/config/bridge_config.hpp"

namespace {

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
    std::printf(
        "Subscribes dds.topics.state and Prints received LowState frequency.\n");
    return 0;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  const std::string config_path =
      argc > 1 ? std::string(argv[1]) : std::string("config/g1.sample.yaml");
  const bridge::config::BridgeConfigLoadResult loaded =
      bridge::config::load_bridge_config(config_path);
  if (!loaded.succeeded()) {
    std::fprintf(stderr, "lowstate_frequency_receiver config load failed: %s\n",
                 loaded.detail.c_str());
    return 1;
  }

  if (!init_channel_factory(*loaded.config)) {
    std::fprintf(stderr,
                 "lowstate_frequency_receiver failed to initialize DDS\n");
    return 2;
  }

  std::mutex mutex;
  std::uint64_t received_count = 0;
  std::uint32_t last_tick = 0;
  std::uint8_t last_mode_pr = 0;
  std::uint8_t last_mode_machine = 0;

  const std::string topic = loaded.config->adapter.topics.state.empty()
                                ? std::string("rt/lowstate")
                                : loaded.config->adapter.topics.state;
  auto subscriber = std::make_unique<
      unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
      topic);
  subscriber->InitChannel(
      [&mutex, &received_count, &last_tick, &last_mode_pr,
       &last_mode_machine](const void* message) {
        if (message == nullptr) {
          return;
        }
        const auto& state =
            *static_cast<const unitree_hg::msg::dds_::LowState_*>(message);
        std::lock_guard<std::mutex> lock(mutex);
        ++received_count;
        last_tick = state.tick();
        last_mode_pr = state.mode_pr();
        last_mode_machine = state.mode_machine();
      },
      1);

  std::fprintf(stderr,
               "lowstate_frequency_receiver topic=%s domain=%u iface=%s\n",
               topic.c_str(),
               loaded.config->dds.domain_id,
               loaded.config->adapter.connection.interface_hint.c_str());

  std::uint64_t last_count = 0;
  std::uint64_t window_start_ns = monotonic_now_ns();
  while (!g_stop.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    std::lock_guard<std::mutex> lock(mutex);
    const std::uint64_t now_ns = monotonic_now_ns();
    if (now_ns - window_start_ns < kRateWindowNs) {
      continue;
    }

    const std::uint64_t count = received_count - last_count;
    std::fprintf(stderr,
                 "lowstate receive frequency: %llu Hz total=%llu tick=%u mode_pr=%u mode_machine=%u\n",
                 static_cast<unsigned long long>(count),
                 static_cast<unsigned long long>(received_count), last_tick,
                 last_mode_pr, last_mode_machine);
    last_count = received_count;
    window_start_ns = now_ns;
  }

  subscriber->CloseChannel();
  return 0;
}
