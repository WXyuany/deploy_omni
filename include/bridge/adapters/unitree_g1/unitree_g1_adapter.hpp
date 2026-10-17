#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "../../core/robot_sdk_adapter.hpp"
#include "unitree_g1_sdk.hpp"
#include "unitree_g1_sensor_session.hpp"

namespace bridge::adapters::unitree_g1 {

inline constexpr std::string_view kUnitreeG1AdapterName{"unitree_g1"};
inline constexpr std::size_t kUnitreeG1JointCount{29};

struct SessionConfig {
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
  bool require_motion_release{true};
  bool verify_low_state_crc{false};
  bool forwarding_log_enabled{true};
};

struct SafetyConfig {
  bool require_initial_state_before_motion{true};
};

struct UnitreeG1AdapterConfig {
  SessionConfig session;
  SafetyConfig safety;
  std::vector<std::string> joint_names;
  std::size_t hardware_slot_count{0};

  [[nodiscard]] std::size_t joint_count() const noexcept {
    return joint_names.size();
  }

  [[nodiscard]] std::size_t command_width() const noexcept {
    return hardware_slot_count == 0 ? joint_names.size() : hardware_slot_count;
  }
};

class UnitreeG1Adapter final : public bridge::core::RobotSdkAdapter {
 public:
  explicit UnitreeG1Adapter(UnitreeG1AdapterConfig config);
  ~UnitreeG1Adapter() override;

  UnitreeG1Adapter(const UnitreeG1Adapter&) = delete;
  UnitreeG1Adapter& operator=(const UnitreeG1Adapter&) = delete;
  UnitreeG1Adapter(UnitreeG1Adapter&&) noexcept;
  UnitreeG1Adapter& operator=(UnitreeG1Adapter&&) noexcept;

  [[nodiscard]] std::string_view name() const noexcept override {
    return kUnitreeG1AdapterName;
  }

  [[nodiscard]] bridge::core::RobotCapabilitySet capabilities() const noexcept override {
    using bridge::core::RobotCapability;
    return RobotCapability::kJointState |
           RobotCapability::kJointPositionCommand |
           RobotCapability::kJointTorqueCommand |
           RobotCapability::kImu |
           RobotCapability::kCamera |
           RobotCapability::kPointCloud |
           RobotCapability::kNavigation |
           RobotCapability::kAudio |
           RobotCapability::kLighting;
  }

  [[nodiscard]] const UnitreeG1AdapterConfig& config() const noexcept;
  [[nodiscard]] UnitreeG1SdkFacade& sdk() noexcept;
  [[nodiscard]] const UnitreeG1SdkFacade& sdk() const noexcept;
  [[nodiscard]] UnitreeG1SensorSession& sensors() noexcept;
  [[nodiscard]] const UnitreeG1SensorSession& sensors() const noexcept;

  void set_runtime_event_sink(bridge::core::BridgeRuntimeEventSink* event_sink) override;
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

}  // namespace bridge::adapters::unitree_g1
