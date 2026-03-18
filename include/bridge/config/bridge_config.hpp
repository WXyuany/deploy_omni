#pragma once

#include <optional>
#include <string>
#include <vector>

#include "../adapters/kepler_k2/kepler_k2_adapter.hpp"
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

struct BridgeStartupConfig {
  std::string source_path;
  core::BridgeRunnerConfig runner;
  adapters::kepler_k2::KeplerK2AdapterConfig adapter;
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
