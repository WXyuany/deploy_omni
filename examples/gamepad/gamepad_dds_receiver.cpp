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

#include "unitree_g1_gamepad.hpp"
#include "bridge/config/bridge_config.hpp"

namespace {

std::atomic<bool> g_stop{false};

void handle_signal(int) { g_stop.store(true); }

[[nodiscard]] bool init_channel_factory(
    const bridge::config::BridgeStartupConfig& config) {
  try {
    if (config.adapter.connection.interface_hint.empty()) {
      unitree::robot::ChannelFactory::Instance()->Init(
          static_cast<int32_t>(config.dds.domain_id));
      return true;
    }
    unitree::robot::ChannelFactory::Instance()->Init(
        static_cast<int32_t>(config.dds.domain_id),
        config.adapter.connection.interface_hint);
    return true;
  } catch (...) {
    unitree::robot::ChannelFactory::Instance()->Init(
        static_cast<int32_t>(config.dds.domain_id));
    return true;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "--help") {
    std::printf("Usage: %s [config-path]\n", argv[0]);
    std::printf("Subscribes LowState from dds.topics.state, decodes wireless_remote, and prints the gamepad.\n");
    return 0;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  const std::string config_path = argc > 1 ? argv[1] : "config/g1.sample.yaml";
  const bridge::config::BridgeConfigLoadResult loaded =
      bridge::config::load_bridge_config(config_path);
  if (!loaded.succeeded()) {
    std::fprintf(stderr, "gamepad_dds_receiver config load failed: %s\n",
                 loaded.detail.c_str());
    return 1;
  }

  if (!init_channel_factory(*loaded.config)) {
    std::fprintf(stderr, "gamepad_dds_receiver failed to initialize DDS\n");
    return 2;
  }

  bridge::adapters::unitree_g1::G1Gamepad gamepad;
  std::mutex gamepad_mutex;
  auto subscriber = std::make_unique<
      unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
      loaded.config->dds.topics.state);
  subscriber->InitChannel(
      [&gamepad, &gamepad_mutex](const void* message) {
        if (message == nullptr) {
          return;
        }
        const auto& state =
            *static_cast<const unitree_hg::msg::dds_::LowState_*>(message);
        std::lock_guard<std::mutex> lock(gamepad_mutex);
        const std::string status =
            bridge::adapters::unitree_g1::format_wireless_remote_gamepad_status(
                gamepad, state.wireless_remote());
        std::printf("%s\n", status.c_str());
      },
      1);

  std::printf("gamepad_dds_receiver domain=%u iface=%s topic=%s\n",
              loaded.config->dds.domain_id,
              loaded.config->adapter.connection.interface_hint.empty()
                  ? "<default>"
                  : loaded.config->adapter.connection.interface_hint.c_str(),
              loaded.config->dds.topics.state.c_str());

  while (!g_stop.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  subscriber->CloseChannel();
  return 0;
}
