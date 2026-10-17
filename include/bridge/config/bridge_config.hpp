#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "../core/bridge_runner.hpp"
#include "joint_map.hpp"

namespace bridge::config {

enum class BridgeConfigLoadStatus {
  kOk,
  kIoError,
  kRejected,
};

enum class BridgeConfigIssueCode {
  kIoError,
  kMissingField,
  kInvalidValue,
  kUnsupportedValue,
  kJointMapInvalid,
};

struct BridgeConfigIssue {
  BridgeConfigIssueCode code{BridgeConfigIssueCode::kInvalidValue};
  std::string message;
};

struct AdapterConnectionConfig {
  // 前缀是通信方式，例如 udp / dds / ros2。
  std::string transport;
  // 后缀是端点提示；与 transport 组合后会形成 transport_endpoint。
  std::string interface_hint;
};

struct AdapterStartupPolicyConfig {
  std::uint64_t read_initial_state_timeout_ns{0};
  bool require_healthy_hardware{true};
};

struct AdapterMotionStateConfig {
  bool required{true};
  std::string required_task{"joint_bridge"};
  std::optional<std::string> initial_task;
};

struct AdapterTopicConfig {
  std::string command;
  std::string state;
  std::string imu;
  std::string bms_state;
  std::string mainboard_state;
};

struct RobotAdapterConfig {
  std::string kind;
  std::string robot_name;
  std::string robot_model;
  std::string command_mode;
  AdapterConnectionConfig connection;
  AdapterStartupPolicyConfig startup;
  AdapterMotionStateConfig motion_state;
  AdapterTopicConfig topics;
  std::vector<std::string> joint_names;

  [[nodiscard]] std::size_t joint_count() const noexcept {
    return joint_names.size();
  }
};

struct DdsTopicNames {
  std::string command;
  std::string state;
  std::string imu_state;
  std::string bms_state;
  std::string mainboard_state;
  std::string fault_code;
  std::string status;
};

struct DdsConfig {
  std::uint32_t domain_id{0};
  DdsTopicNames topics{
      .command = "rt/lowcmd",
      .state = "rt/lowstate",
      .imu_state = "",
      .bms_state = "",
      .mainboard_state = "",
      .fault_code = "",
      .status = "",
  };
};

struct DebugPrintConfig {
  bool state{false};
  bool command{false};
  bool forwarding_log{true};
  std::uint32_t rate_hz{0};

  [[nodiscard]] bool enabled() const noexcept {
    return state || command;
  }
};

struct GamepadConfig {
  bool enabled{false};
  std::string backend{"sdl"};
  int controller_index{0};
  std::string mapping_path;
};

struct AssemblyComponentConfig {
  bool enabled{true};
  std::string type;
};

struct AssemblyModuleConfig {
  bool enabled{true};
  std::string name;
  std::string type;
  int controller_index{0};
  std::string mapping_path;
  int poll_interval_ms{5};
  int stale_timeout_ms{100};
};

struct AssemblyConfig {
  std::string profile;
  AssemblyComponentConfig adapter;
  AssemblyComponentConfig command_ingress;
  AssemblyComponentConfig state_egress;
  std::vector<AssemblyModuleConfig> modules;
};

struct BridgeStartupConfig {
  std::string source_path;
  std::string robot_type;
  core::BridgeRunnerConfig runner;
  AssemblyConfig assembly;
  DdsConfig dds;
  DebugPrintConfig debug_print;
  GamepadConfig gamepad;
  RobotAdapterConfig adapter;
  JointMapValidationOptions joint_map_validation;
  std::vector<JointMapEntry> joint_map_entries;
};

struct BridgeConfigLoadResult {
  BridgeConfigLoadStatus status{BridgeConfigLoadStatus::kRejected};
  std::string detail;
  std::vector<BridgeConfigIssue> issues;
  std::optional<BridgeStartupConfig> config;

  [[nodiscard]] bool succeeded() const noexcept {
    return status == BridgeConfigLoadStatus::kOk && config.has_value();
  }
};

[[nodiscard]] BridgeConfigLoadResult load_bridge_config(
    const std::string& file_path);

}
