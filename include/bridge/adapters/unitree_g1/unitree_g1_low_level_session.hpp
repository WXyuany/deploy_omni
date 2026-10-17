#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "../../core/types.hpp"
#include "../../observability/forwarding_stats.hpp"

namespace bridge::adapters::unitree_g1 {

struct LowLevelSessionConfig {
  std::string transport_mode;
  std::string robot_id;
  std::string transport_endpoint;
  std::string control_profile;
  std::string command_topic;
  std::string state_topic;
  std::string imu_topic;
  std::uint64_t initialize_timeout_ns{0};
  std::uint64_t read_timeout_ns{0};
  std::uint32_t read_retry_attempts{10};
  std::uint64_t read_retry_interval_ns{1000000000ULL};
  std::uint64_t write_timeout_ns{0};
  std::uint64_t stale_state_after_ns{0};
  bool require_motion_release{true};
  bool verify_low_state_crc{false};
  bool forwarding_log_enabled{true};
  std::size_t hardware_slot_count{0};
  std::function<void()> state_available_callback;
  std::function<int32_t(std::string&, std::string&)> check_mode_hook;
  std::function<int32_t()> release_mode_hook;
};

struct LowLevelSessionState {
  std::uint64_t sample_time_ns{0};
  std::uint64_t receive_time_ns{0};
  std::uint64_t receive_generation{0};
  std::uint64_t callback_store_duration_ns{0};
  std::uint64_t read_start_time_ns{0};
  std::uint64_t read_complete_time_ns{0};
  std::uint64_t read_snapshot_duration_ns{0};
  std::uint64_t stateframe_build_duration_ns{0};
  std::uint64_t callback_drops{0};
  std::uint32_t tick{0};
  std::array<std::uint32_t, 2> low_state_version{1U, 0U};
  std::array<std::uint32_t, 4> low_state_reserve{};
  std::uint8_t mode_pr{0};
  std::uint8_t mode_machine{0};
  bool valid{false};
  bool stale{false};
  bool degraded{false};
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<double> acceleration;
  std::vector<std::uint8_t> motor_mode;
  std::vector<std::array<std::int16_t, 2>> motor_temperature;
  std::vector<double> motor_voltage;
  std::vector<std::array<std::uint32_t, 2>> motor_sensor;
  std::vector<std::uint32_t> motor_state;
  std::vector<std::array<std::uint32_t, 4>> motor_reserve;
  std::array<std::uint8_t, 40> wireless_remote{};
  std::optional<bridge::core::ImuSample> imu;
  std::optional<bridge::core::BridgeFault> fault;
};

struct LowLevelSessionReadResult {
  bridge::core::AdapterCallResult result;
  std::optional<LowLevelSessionState> state;
};

[[nodiscard]] std::uint64_t select_reader_loop_sleep_ns(
    std::uint64_t configured_read_timeout_ns) noexcept;

class UnitreeG1LowLevelSession {
 public:
  explicit UnitreeG1LowLevelSession(LowLevelSessionConfig config);
  ~UnitreeG1LowLevelSession();

  [[nodiscard]] const LowLevelSessionConfig& config() const noexcept;
  void set_state_available_callback(std::function<void()> callback);
  [[nodiscard]] bridge::core::AdapterCallResult initialize();
  [[nodiscard]] LowLevelSessionReadResult read_state();
  void handle_low_state_message(const unitree_hg::msg::dds_::LowState_& state);
  void handle_imu_message(const unitree_hg::msg::dds_::IMUState_& imu);
  void handle_low_state_message(const void* message);
  void handle_imu_message(const void* message);
  [[nodiscard]] const std::optional<unitree_hg::msg::dds_::LowCmd_>& last_packed_command()
      const noexcept;
  [[nodiscard]] bridge::core::AdapterCallResult write_command(
      const bridge::core::CommandFrame& command);
  [[nodiscard]] bridge::core::AdapterHealth health() const;
  [[nodiscard]] bridge::core::AdapterCallResult shutdown();

 private:
  LowLevelSessionConfig config_;
  std::uint64_t state_time_ns_{0};
  std::uint64_t command_time_ns_{0};
  std::atomic<bool> initialized_{false};
  std::atomic<bool> released_motion_{false};
  std::atomic<bool> connection_alive_{false};
  std::atomic<std::uint8_t> mode_pr_{0};
  std::atomic<std::uint8_t> mode_machine_{0};
  std::uint64_t last_low_state_time_ns_{0};
  std::uint64_t last_low_state_receive_time_ns_{0};
  std::uint64_t last_low_state_callback_store_duration_ns_{0};
  std::uint64_t last_low_state_generation_{0};
  std::uint64_t last_consumed_low_state_generation_{0};
  std::uint64_t last_read_time_ns_{0};
  std::uint64_t last_write_time_ns_{0};
  std::chrono::steady_clock::time_point last_low_state_receive_time_{};
  std::optional<unitree_hg::msg::dds_::LowCmd_> last_packed_command_;
  std::optional<unitree_hg::msg::dds_::LowState_> latest_low_state_;
  std::optional<unitree_hg::msg::dds_::IMUState_> latest_imu_state_;
  bridge::observability::ForwardingStats lowstate_receive_stats_;
  bridge::observability::ForwardingStats state_read_stats_;
  bridge::observability::ForwardingStats command_write_stats_;
  std::optional<bridge::observability::ForwardingStatsSnapshot>
      pending_lowstate_receive_stats_;
  std::atomic<std::uint64_t> dropped_lowstate_callback_updates_{0};
  std::uint64_t last_logged_lowstate_callback_drops_{0};
  std::shared_ptr<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>
      lowcmd_publisher_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>
      lowstate_subscriber_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::IMUState_>>
      imu_subscriber_;
  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> motion_switcher_client_;

  void drain_pending_lowstate_receive_log();
  void handle_low_state_message(const unitree_hg::msg::dds_::LowState_& state,
                                bool drop_if_state_lock_is_busy);
  mutable std::mutex command_mutex_;
  mutable std::mutex state_mutex_;
  std::mutex telemetry_mutex_;
};

}  // namespace bridge::adapters::unitree_g1
