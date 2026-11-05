#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "bridge/config/bridge_config.hpp"

namespace {

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

void write_array(std::ostream& output, const char* name,
                 const std::array<std::uint32_t, 2>& values) {
  output << "\"" << name << "\":[" << values[0] << ',' << values[1] << ']';
}

void write_array4(std::ostream& output, const char* name,
                  const std::array<std::uint32_t, 4>& values) {
  output << "\"" << name << "\":[" << values[0] << ',' << values[1] << ','
         << values[2] << ',' << values[3] << ']';
}

void write_u8_array(std::ostream& output, const char* name,
                    const std::array<std::uint8_t, 40>& values) {
  output << "\"" << name << "\":[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      output << ',';
    }
    output << static_cast<unsigned>(values[index]);
  }
  output << ']';
}

void write_imu(std::ostream& output,
               const unitree_hg::msg::dds_::IMUState_& imu) {
  output << "\"imu_state\":{";
  output << "\"quaternion\":[" << imu.quaternion()[0] << ','
         << imu.quaternion()[1] << ',' << imu.quaternion()[2] << ','
         << imu.quaternion()[3] << "],";
  output << "\"gyroscope\":[" << imu.gyroscope()[0] << ','
         << imu.gyroscope()[1] << ',' << imu.gyroscope()[2] << "],";
  output << "\"accelerometer\":[" << imu.accelerometer()[0] << ','
         << imu.accelerometer()[1] << ',' << imu.accelerometer()[2] << "],";
  output << "\"rpy\":[" << imu.rpy()[0] << ',' << imu.rpy()[1] << ','
         << imu.rpy()[2] << "],";
  output << "\"temperature\":" << imu.temperature() << '}';
}

void write_motor(std::ostream& output,
                 const unitree_hg::msg::dds_::MotorState_& motor) {
  output << '{';
  output << "\"mode\":" << static_cast<unsigned>(motor.mode()) << ',';
  output << "\"q\":" << motor.q() << ',';
  output << "\"dq\":" << motor.dq() << ',';
  output << "\"ddq\":" << motor.ddq() << ',';
  output << "\"tau_est\":" << motor.tau_est() << ',';
  output << "\"temperature\":[" << motor.temperature()[0] << ','
         << motor.temperature()[1] << "],";
  output << "\"vol\":" << motor.vol() << ',';
  output << "\"sensor\":[" << motor.sensor()[0] << ',' << motor.sensor()[1]
         << "],";
  output << "\"motorstate\":" << motor.motorstate() << ',';
  write_array4(output, "reserve", motor.reserve());
  output << '}';
}

void write_lowstate_json_line(std::ostream& output, const char* direction,
                              const std::string& topic,
                              const unitree_hg::msg::dds_::LowState_& state,
                              const std::uint64_t receive_time_ns) {
  output << '{';
  output << "\"direction\":\"" << direction << "\",";
  output << "\"topic\":\"" << topic << "\",";
  output << "\"receive_time_ns\":" << receive_time_ns << ',';
  write_array(output, "version", state.version());
  output << ',';
  output << "\"mode_pr\":" << static_cast<unsigned>(state.mode_pr()) << ',';
  output << "\"mode_machine\":" << static_cast<unsigned>(state.mode_machine())
         << ',';
  output << "\"tick\":" << state.tick() << ',';
  write_array4(output, "reserve", state.reserve());
  output << ',';
  output << "\"motor_state\":[";
  for (std::size_t index = 0; index < state.motor_state().size(); ++index) {
    if (index != 0) {
      output << ',';
    }
    write_motor(output, state.motor_state()[index]);
  }
  output << "],";
  write_imu(output, state.imu_state());
  output << ',';
  write_u8_array(output, "wireless_remote", state.wireless_remote());
  output << ',';
  output << "\"crc\":" << state.crc();
  output << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "--help") {
    std::printf("Usage: %s [config-path] [output-jsonl]\n", argv[0]);
    std::printf("Records Unitree LowState fields before and after forwarding.\n");
    std::printf("Before topic: adapter.topics.state; after topic: dds.topics.state.\n");
    return 0;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  const std::string config_path =
      argc > 1 ? std::string(argv[1]) : std::string("config/g1.sample.yaml");
  const std::string output_path =
      argc > 2 ? std::string(argv[2]) : std::string("logs/lowstate_fields.jsonl");

  const bridge::config::BridgeConfigLoadResult loaded =
      bridge::config::load_bridge_config(config_path);
  if (!loaded.succeeded()) {
    std::fprintf(stderr, "lowstate_field_recorder config load failed: %s\n",
                 loaded.detail.c_str());
    return 1;
  }
  if (!init_channel_factory(*loaded.config)) {
    std::fprintf(stderr, "lowstate_field_recorder failed to initialize DDS\n");
    return 2;
  }

  std::filesystem::create_directories(std::filesystem::path(output_path).parent_path());
  std::ofstream output(output_path, std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    std::fprintf(stderr, "lowstate_field_recorder failed to open %s\n",
                 output_path.c_str());
    return 3;
  }

  const std::string before_topic = loaded.config->adapter.topics.state;
  const std::string after_topic = loaded.config->dds.topics.state;
  std::mutex output_mutex;

  auto before_subscriber = std::make_unique<
      unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
      before_topic);
  auto after_subscriber = std::make_unique<
      unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
      after_topic);

  before_subscriber->InitChannel(
      [&output, &output_mutex, before_topic](const void* message) {
        if (message == nullptr) {
          return;
        }
        const auto& state =
            *static_cast<const unitree_hg::msg::dds_::LowState_*>(message);
        std::lock_guard<std::mutex> lock(output_mutex);
        write_lowstate_json_line(output, "before", before_topic, state,
                                 monotonic_now_ns());
      },
      10);
  after_subscriber->InitChannel(
      [&output, &output_mutex, after_topic](const void* message) {
        if (message == nullptr) {
          return;
        }
        const auto& state =
            *static_cast<const unitree_hg::msg::dds_::LowState_*>(message);
        std::lock_guard<std::mutex> lock(output_mutex);
        write_lowstate_json_line(output, "after", after_topic, state,
                                 monotonic_now_ns());
      },
      10);

  std::fprintf(stderr,
               "lowstate_field_recorder before=%s after=%s output=%s\n",
               before_topic.c_str(), after_topic.c_str(), output_path.c_str());
  while (!g_stop.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::lock_guard<std::mutex> lock(output_mutex);
    output.flush();
  }

  before_subscriber->CloseChannel();
  after_subscriber->CloseChannel();
  return 0;
}
