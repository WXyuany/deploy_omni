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
  std::string transport;
  std::string interface_hint;
};

struct AdapterStartupPolicyConfig {
  std::uint64_t read_initial_state_timeout_ns{0};
  bool require_healthy_hardware{true};
};

struct AdapterSafetyPolicyConfig {
  std::string safe_command_profile;
  bool allow_safe_command_during_shutdown{true};
};

struct RobotAdapterConfig {
  std::string kind;
  std::string robot_name;
  std::string robot_model;
  std::string command_mode;
  AdapterConnectionConfig connection;
  AdapterStartupPolicyConfig startup;
  AdapterSafetyPolicyConfig safety;
  std::vector<std::string> joint_names;

  [[nodiscard]] std::size_t joint_count() const noexcept {
    return joint_names.size();
  }
};

struct DdsTopicNames {
  std::string command;
  std::string state;
  std::string status;
};

struct DdsConfig {
  std::uint32_t domain_id{0};
  DdsTopicNames topics{
      .command = "rt/lowcmd",
      .state = "rt/lowstate",
      .status = "",
  };
};

struct BridgeStartupConfig {
  std::string source_path;
  std::string robot_type;
  core::BridgeRunnerConfig runner;
  DdsConfig dds;
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
