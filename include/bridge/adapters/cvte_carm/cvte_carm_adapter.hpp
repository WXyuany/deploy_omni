#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "../../core/robot_sdk_adapter.hpp"

namespace bridge::adapters::cvte_carm {

inline constexpr std::string_view kCvteCarmA3AdapterName{"cvte_carm_a3"};
inline constexpr std::string_view kCvteCarmD3AdapterName{"cvte_carm_d3"};

enum class Model {
  kA3Single,
  kD3Dual,
};

enum class ArmSelection {
  kSingle,
  kLeft,
  kRight,
  kBoth,
};

struct SessionConfig {
  std::string robot_id;
  std::string server_ip;
  std::uint16_t server_port{8090};
  double connect_timeout_seconds{1.0};
  std::string control_profile{"position"};
};

struct SafetyConfig {
  bool require_initial_state_before_motion{true};
};

struct CvteCarmAdapterConfig {
  Model model{Model::kA3Single};
  ArmSelection arm_selection{ArmSelection::kSingle};
  SessionConfig session;
  SafetyConfig safety;
  std::vector<std::string> joint_names;
  std::size_t hardware_slot_count{0};
  std::size_t arm_joint_count{0};

  [[nodiscard]] std::size_t joint_count() const noexcept {
    return joint_names.size();
  }

  [[nodiscard]] std::size_t command_width() const noexcept {
    return hardware_slot_count == 0 ? joint_names.size() : hardware_slot_count;
  }
};

class CvteCarmAdapter final : public bridge::core::RobotSdkAdapter {
 public:
  explicit CvteCarmAdapter(CvteCarmAdapterConfig config);
  ~CvteCarmAdapter() override;

  CvteCarmAdapter(const CvteCarmAdapter&) = delete;
  CvteCarmAdapter& operator=(const CvteCarmAdapter&) = delete;
  CvteCarmAdapter(CvteCarmAdapter&&) noexcept;
  CvteCarmAdapter& operator=(CvteCarmAdapter&&) noexcept;

  [[nodiscard]] std::string_view name() const noexcept override;
  [[nodiscard]] bridge::core::RobotCapabilitySet capabilities() const noexcept override;
  [[nodiscard]] const CvteCarmAdapterConfig& config() const noexcept;

  bridge::core::AdapterCallResult initialize() override;
  bridge::core::AdapterCallResult set_ready();
  bridge::core::AdapterCallResult set_servo_enable(bool enabled);
  bridge::core::AdapterCallResult set_control_mode(int mode);
  bridge::core::AdapterCallResult emergency_stop();
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
