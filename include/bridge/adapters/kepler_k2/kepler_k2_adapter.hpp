#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "../../core/robot_sdk_adapter.hpp"
#include "../../core/types.hpp"

namespace bridge::adapters::kepler_k2 {

inline constexpr std::string_view kKeplerK2AdapterName{"kepler_k2"};

enum class ImuIntegrationMode {
  kDisabled,
  kAdapterOwned,
  kSharedMemory,
};

struct SharedMemoryImuConfig {
  std::string region_name;
  std::string mapping_name;
  std::uint64_t expected_period_ns{0};
  std::uint64_t stale_after_ns{0};
  bool required{false};
};

struct ImuIntegrationConfig {
  ImuIntegrationMode mode{ImuIntegrationMode::kDisabled};
  std::string frame_id;
  std::optional<SharedMemoryImuConfig> shared_memory;
};

struct SessionConfig {
  std::string robot_id;
  // 这是一个复合名称：前缀是通信方式，后缀是机器人类型。
  std::string transport_endpoint;
  std::string control_profile;
  std::uint64_t initialize_timeout_ns{0};
  std::uint64_t read_timeout_ns{0};
  std::uint64_t write_timeout_ns{0};
};

struct SafetyConfig {
  bool require_initial_state_before_motion{true};
  bool allow_command_when_imu_missing{true};
};

struct KeplerK2AdapterConfig {
  SessionConfig session;
  SafetyConfig safety;
  std::vector<std::string> joint_names;
  std::size_t hardware_slot_count{0};
  std::optional<ImuIntegrationConfig> imu;

  [[nodiscard]] std::size_t joint_count() const noexcept {
    return joint_names.size();
  }

  [[nodiscard]] std::size_t command_width() const noexcept {
    return hardware_slot_count == 0 ? joint_names.size() : hardware_slot_count;
  }

  [[nodiscard]] bool has_imu_config() const noexcept { return imu.has_value(); }
};

class KeplerK2Adapter final : public bridge::core::RobotSdkAdapter {
 public:
  explicit KeplerK2Adapter(KeplerK2AdapterConfig config);
  ~KeplerK2Adapter() override;

  KeplerK2Adapter(const KeplerK2Adapter&) = delete;
  KeplerK2Adapter& operator=(const KeplerK2Adapter&) = delete;
  KeplerK2Adapter(KeplerK2Adapter&&) noexcept;
  KeplerK2Adapter& operator=(KeplerK2Adapter&&) noexcept;

  [[nodiscard]] std::string_view name() const noexcept override {
    return kKeplerK2AdapterName;
  }

  [[nodiscard]] const KeplerK2AdapterConfig& config() const noexcept;

  bridge::core::AdapterCallResult initialize() override;
  [[nodiscard]] bridge::core::AdapterReadResult read_state() override;
  bridge::core::AdapterCallResult write_command(
      const bridge::core::CommandFrame& command) override;
  [[nodiscard]] bridge::core::AdapterHealthResult query_health() const override;
  bridge::core::AdapterCallResult shutdown() override;

 private:
  class Impl;

  std::unique_ptr<Impl> impl_;
};

}
