#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "../../core/robot_sdk_adapter.hpp"

namespace bridge::adapters::pm01 {

inline constexpr std::string_view kPm01AdapterName{"pm01"};
inline constexpr std::size_t kPm01JointCount{24};
inline constexpr std::size_t kPm01DdsMotorCount{35};

struct SessionConfig {
  std::string robot_id;
  std::string control_profile;
  std::uint64_t initialize_timeout_ns{0};
  std::uint64_t read_timeout_ns{0};
  std::uint64_t write_timeout_ns{0};
};

struct SafetyConfig {
  bool require_initial_state_before_motion{true};
};

struct MotionStateConfig {
  bool required{true};
  std::string required_task{"joint_bridge"};
  std::optional<std::string> initial_task;
};

struct Pm01AdapterConfig {
  SessionConfig session;
  SafetyConfig safety;
  MotionStateConfig motion_state;
  bool publish_default_imu_when_missing{true};
  std::vector<std::string> joint_names;
  std::vector<std::int64_t> dds_joint_indices;

  [[nodiscard]] std::size_t joint_count() const noexcept {
    return joint_names.size();
  }
};

class Pm01Adapter final : public bridge::core::RobotSdkAdapter {
 public:
  explicit Pm01Adapter(Pm01AdapterConfig config);
  ~Pm01Adapter() override;

  Pm01Adapter(const Pm01Adapter&) = delete;
  Pm01Adapter& operator=(const Pm01Adapter&) = delete;
  Pm01Adapter(Pm01Adapter&&) noexcept;
  Pm01Adapter& operator=(Pm01Adapter&&) noexcept;

  [[nodiscard]] std::string_view name() const noexcept override {
    return kPm01AdapterName;
  }

  [[nodiscard]] const Pm01AdapterConfig& config() const noexcept;

  bridge::core::AdapterCallResult initialize() override;
  void set_runtime_event_sink(bridge::core::BridgeRuntimeEventSink* event_sink) override;
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
