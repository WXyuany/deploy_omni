#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "../../core/robot_sdk_adapter.hpp"

namespace bridge::adapters::agibot_x2 {

inline constexpr std::string_view kAgibotX2AdapterName{"agibot_x2"};
inline constexpr std::size_t kAgibotX2JointCount{31};

struct AgibotX2TopicConfig {
  std::string command_prefix{"/aima/hal/joint"};
  std::string state_prefix{"/aima/hal/joint"};
  std::string head_group{"head"};
  std::string waist_group{"waist"};
  std::string arm_group{"arm"};
  std::string leg_group{"leg"};
};

struct AgibotX2SessionConfig {
  std::string robot_id;
  std::string transport{"ros2"};
  std::string interface_hint;
  std::uint64_t initialize_timeout_ns{1'000'000'000ULL};
  std::uint64_t read_timeout_ns{2'000'000ULL};
  std::uint64_t write_timeout_ns{2'000'000ULL};
};

struct AgibotX2SafetyConfig {
  bool require_initial_state_before_motion{true};
  bool require_zero_command_on_shutdown{true};
};

struct AgibotX2AdapterConfig {
  AgibotX2SessionConfig session;
  AgibotX2TopicConfig topics;
  AgibotX2SafetyConfig safety;
  std::vector<std::string> joint_names;

  [[nodiscard]] std::size_t joint_count() const noexcept {
    return joint_names.size();
  }
};

class AgibotX2Adapter final : public bridge::core::RobotSdkAdapter {
 public:
  explicit AgibotX2Adapter(AgibotX2AdapterConfig config);
  ~AgibotX2Adapter() override;

  AgibotX2Adapter(const AgibotX2Adapter&) = delete;
  AgibotX2Adapter& operator=(const AgibotX2Adapter&) = delete;
  AgibotX2Adapter(AgibotX2Adapter&&) noexcept;
  AgibotX2Adapter& operator=(AgibotX2Adapter&&) noexcept;

  [[nodiscard]] std::string_view name() const noexcept override {
    return kAgibotX2AdapterName;
  }
  [[nodiscard]] bridge::core::RobotCapabilitySet capabilities() const noexcept override;
  [[nodiscard]] const AgibotX2AdapterConfig& config() const noexcept;

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

}  // namespace bridge::adapters::agibot_x2
