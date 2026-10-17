#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "../core/types.hpp"

namespace bridge::config {

class JointMap;
struct JointMapBuildResult;

struct JointValueRange {
  std::optional<double> lower;
  std::optional<double> upper;

  [[nodiscard]] bool contains(double value) const noexcept {
    if (lower.has_value() && value < *lower) {
      return false;
    }
    if (upper.has_value() && value > *upper) {
      return false;
    }
    return true;
  }
};

struct JointFieldLimits {
  std::optional<JointValueRange> position;
  std::optional<JointValueRange> velocity;
  std::optional<JointValueRange> effort;
  std::optional<JointValueRange> stiffness;
  std::optional<JointValueRange> damping;
  std::optional<JointValueRange> acceleration;
};

struct JointFieldTransform {
  double sign{1.0};
  double offset{0.0};

  [[nodiscard]] bool is_identity() const noexcept {
    return sign == 1.0 && offset == 0.0;
  }
};

struct JointFieldTransforms {
  JointFieldTransform position;
  JointFieldTransform velocity;
  JointFieldTransform effort;
  JointFieldTransform stiffness;
  JointFieldTransform damping;
  JointFieldTransform acceleration;
};

struct JointMapEntry {
  std::size_t sim_index{0};
  std::size_t hardware_index{0};
  std::optional<std::string> sim_name;
  std::optional<std::string> hardware_name;
  std::optional<std::string> label;
  bool enabled{true};
  JointFieldTransforms transforms;
  JointFieldLimits sim_limits;
  JointFieldLimits hardware_limits;
};

enum class JointMapIssueCode {
  kInvalidEntry,
  kInvalidRange,
  kInvalidTransform,
  kDuplicateSimIndex,
  kDuplicateHardwareIndex,
  kDuplicateSimName,
  kDuplicateHardwareName,
  kMissingExpectedSimIndex,
  kMissingExpectedHardwareIndex,
  kUnexpectedSimIndex,
  kUnexpectedHardwareIndex,
  kInputWidthMismatch,
  kEntryDisabled,
  kOutOfRange,
};

enum class JointMapIssueSeverity {
  kWarning,
  kError,
};

struct JointMapIssue {
  JointMapIssueCode code{JointMapIssueCode::kInvalidEntry};
  JointMapIssueSeverity severity{JointMapIssueSeverity::kError};
  std::optional<std::size_t> entry_index;
  std::optional<std::size_t> sim_index;
  std::optional<std::size_t> hardware_index;
  std::string message;
};

struct JointMapValidationOptions {
  std::optional<std::size_t> expected_sim_joint_count;
  std::optional<std::size_t> expected_hardware_joint_count;
  bool require_dense_enabled_sim_indexes{true};
  bool require_dense_enabled_hardware_indexes{true};
  bool require_unique_enabled_sim_names{false};
  bool require_unique_enabled_hardware_names{false};
};

struct JointMapValidationReport {
  std::vector<JointMapIssue> issues;
  std::size_t enabled_entry_count{0};

  [[nodiscard]] bool ok() const noexcept {
    return issues.empty();
  }
};

struct CommandProjectionResult {
  std::optional<core::CommandFrame> command;
  std::vector<JointMapIssue> issues;

  [[nodiscard]] bool succeeded() const noexcept {
    return command.has_value() && issues.empty();
  }
};

struct StateProjectionResult {
  std::optional<core::StateFrame> state;
  std::vector<JointMapIssue> issues;

  [[nodiscard]] bool succeeded() const noexcept {
    return state.has_value() && issues.empty();
  }
};

struct VectorProjectionResult {
  std::vector<double> values;
  std::vector<JointMapIssue> issues;

  [[nodiscard]] bool succeeded() const noexcept {
    return issues.empty();
  }
};

class JointMap {
 public:
  JointMap(const JointMap&) = default;
  JointMap(JointMap&&) noexcept = default;
  JointMap& operator=(const JointMap&) = default;
  JointMap& operator=(JointMap&&) noexcept = default;
  ~JointMap() = default;

  [[nodiscard]] static JointMapValidationReport validate_entries(
      const std::vector<JointMapEntry>& entries,
      const JointMapValidationOptions& options = {});

  [[nodiscard]] static JointMapBuildResult build(
      std::vector<JointMapEntry> entries,
      const JointMapValidationOptions& options = {});

  [[nodiscard]] const std::vector<JointMapEntry>& entries() const noexcept;

  [[nodiscard]] std::size_t size() const noexcept;

  [[nodiscard]] std::size_t enabled_entry_count() const noexcept;

  [[nodiscard]] CommandProjectionResult project_command_sim_to_hardware(
      const core::CommandFrame& sim_command) const;

  [[nodiscard]] StateProjectionResult project_state_hardware_to_sim(
      const core::StateFrame& hardware_state) const;

  [[nodiscard]] VectorProjectionResult project_positions_sim_to_hardware(
      const std::vector<double>& sim_values) const;

  [[nodiscard]] VectorProjectionResult project_positions_hardware_to_sim(
      const std::vector<double>& hardware_values) const;

 private:
  explicit JointMap(std::vector<JointMapEntry> entries);

  std::vector<JointMapEntry> entries_;
};

struct JointMapBuildResult {
  std::optional<JointMap> joint_map;
  JointMapValidationReport validation;

  [[nodiscard]] bool succeeded() const noexcept {
    return joint_map.has_value() && validation.ok();
  }
};

}
