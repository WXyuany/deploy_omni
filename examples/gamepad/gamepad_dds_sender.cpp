#include <atomic>
#include <array>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include "gamepad_table.hpp"
#include "bridge/config/bridge_config.hpp"
#include "bridge/gamepad/common_gamepad.hpp"
#include "bridge/gamepad/sdl_gamepad.hpp"

namespace {

constexpr std::uint64_t kDefaultRateHz = 60;

std::atomic<bool> g_stop{false};

void handle_signal(int) { g_stop.store(true); }

[[nodiscard]] std::uint64_t parse_rate(int argc, char** argv) {
  if (argc < 3) {
    return kDefaultRateHz;
  }
  const std::uint64_t value = std::stoull(argv[2]);
  return value == 0 ? kDefaultRateHz : value;
}

[[nodiscard]] int parse_controller_index(
    int argc, char** argv, const bridge::config::BridgeStartupConfig& config) {
  if (argc < 4) {
    return config.gamepad.controller_index;
  }
  return std::stoi(argv[3]);
}

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

[[nodiscard]] unitree_hg::msg::dds_::LowState_ make_state(
    const std::uint64_t sequence,
    const std::array<std::uint8_t, 40>& wireless_remote) {
  unitree_hg::msg::dds_::LowState_ state;
  state.mode_pr() = 1;
  state.mode_machine() = 5;
  state.tick() = static_cast<std::uint32_t>(sequence);
  state.wireless_remote(wireless_remote);
  return state;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "--help") {
    std::printf("Usage: %s [config-path] [rate-hz] [controller-index]\n", argv[0]);
    std::printf(
        "Reads a real SDL gamepad, packs wireless_remote, and publishes LowState to dds.topics.state.\n");
    return 0;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  const std::string config_path = argc > 1 ? argv[1] : "config/g1.sample.yaml";
  const auto loaded = bridge::config::load_bridge_config(config_path);
  if (!loaded.succeeded()) {
    std::fprintf(stderr, "gamepad_dds_sender config load failed: %s\n",
                 loaded.detail.c_str());
    return 1;
  }

  const std::uint64_t rate_hz = parse_rate(argc, argv);
  const int controller_index = parse_controller_index(argc, argv, *loaded.config);

  bridge::gamepad::SdlGamepadRemote gamepad_remote(
      {.controller_index = controller_index});
  const bridge::gamepad::SdlGamepadStartResult started = gamepad_remote.start();
  if (!started.connected) {
    std::fprintf(stderr,
                 "gamepad_dds_sender failed: no SDL gamepad connected at index %d\n",
                 controller_index);
    return 2;
  }

  if (!init_channel_factory(*loaded.config)) {
    std::fprintf(stderr, "gamepad_dds_sender failed to initialize DDS\n");
    return 3;
  }
  unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowState_> publisher(
      loaded.config->dds.topics.state);
  publisher.InitChannel();

  std::printf("gamepad_dds_sender domain=%u iface=%s topic=%s rate=%lluHz controller=%d\n",
              loaded.config->dds.domain_id,
              loaded.config->adapter.connection.interface_hint.empty()
                  ? "<default>"
                  : loaded.config->adapter.connection.interface_hint.c_str(),
              loaded.config->dds.topics.state.c_str(),
              static_cast<unsigned long long>(rate_hz), controller_index);

  std::uint64_t sequence = 0;
  std::uint64_t published = 0;
  auto next_deadline = std::chrono::steady_clock::now();
  auto last_print = next_deadline - std::chrono::milliseconds(500);
  const auto period = std::chrono::nanoseconds(1'000'000'000ULL / rate_hz);
  const auto print_period = std::chrono::milliseconds(500);

  while (!g_stop.load()) {
    if (const std::optional<bridge::gamepad::CommonGamepad> gamepad =
            gamepad_remote.read()) {
      const std::array<std::uint8_t, 40> wireless_remote =
          bridge::gamepad::compress_to_wireless_remote(*gamepad);
      unitree_hg::msg::dds_::LowState_ state = make_state(sequence++, wireless_remote);
      publisher.Write(state);
      ++published;

      const auto now = std::chrono::steady_clock::now();
      if (now - last_print >= print_period) {
        if (const std::optional<bridge::gamepad::SdlGamepadSnapshot> snapshot =
                gamepad_remote.last_snapshot()) {
          const std::string table = bridge::demo::format_gamepad_sender_table(
              *gamepad, *snapshot, published, rate_hz, controller_index);
          std::printf("%s", table.c_str());
          std::fflush(stdout);
        }
        last_print = now;
      }
    }

    next_deadline += period;
    std::this_thread::sleep_until(next_deadline);
  }

  publisher.CloseChannel();
  gamepad_remote.stop();
  return 0;
}
