#include "../../../include/bridge/config/joint_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bridge::config {
namespace {

enum class ProjectionDirection {
  kSimToHardware,
  kHardwareToSim,
};

enum class JointField {
  kPosition,
  kVelocity,
  kEffort,
  kStiffness,
  kDamping,
  kAcceleration,
};

struct IndexedEntryRef {
  std::size_t entry_index;
  std::size_t index;
};

[[nodiscard]] std::string field_name(const JointField field) {
  switch (field) {
    case JointField::kPosition:
      return "position";
    case JointField::kVelocity:
      return "velocity";
    case JointField::kEffort:
      return "effort";
    case JointField::kStiffness:
      return "stiffness";
    case JointField::kDamping:
      return "damping";
    case JointField::kAcceleration:
      return "acceleration";
  }

  return "unknown";
}

[[nodiscard]] std::string format_range(const JointValueRange& range) {
  std::ostringstream stream;
  stream << "[";
  if (range.lower.has_value()) {
    stream << *range.lower;
  } else {
    stream << "-inf";
  }
  stream << ", ";
  if (range.upper.has_value()) {
    stream << *range.upper;
  } else {
    stream << "+inf";
  }
  stream << "]";
  return stream.str();
}

[[nodiscard]] std::string entry_name(const JointMapEntry& entry,
                                     const std::size_t entry_index) {
  if (entry.label.has_value() && !entry.label->empty()) {
    return *entry.label;
  }
  if (entry.sim_name.has_value() && !entry.sim_name->empty()) {
    return *entry.sim_name;
  }
  if (entry.hardware_name.has_value() && !entry.hardware_name->empty()) {
    return *entry.hardware_name;
  }

  return "entry[" + std::to_string(entry_index) + "]";
}

[[nodiscard]] const JointFieldTransform& select_transform(
    const JointFieldTransforms& transforms, const JointField field) {
  switch (field) {
    case JointField::kPosition:
      return transforms.position;
    case JointField::kVelocity:
      return transforms.velocity;
    case JointField::kEffort:
      return transforms.effort;
    case JointField::kStiffness:
      return transforms.stiffness;
    case JointField::kDamping:
      return transforms.damping;
    case JointField::kAcceleration:
      return transforms.acceleration;
  }

  return transforms.position;
}

[[nodiscard]] const std::optional<JointValueRange>& select_limit(
    const JointFieldLimits& limits, const JointField field) {
  switch (field) {
    case JointField::kPosition:
      return limits.position;
    case JointField::kVelocity:
      return limits.velocity;
    case JointField::kEffort:
      return limits.effort;
    case JointField::kStiffness:
      return limits.stiffness;
    case JointField::kDamping:
      return limits.damping;
    case JointField::kAcceleration:
      return limits.acceleration;
  }

  return limits.position;
}

void push_issue(std::vector<JointMapIssue>* issues, const JointMapIssueCode code,
                const JointMapIssueSeverity severity,
                const std::string& message,
                const std::optional<std::size_t> entry_index = std::nullopt,
                const std::optional<std::size_t> sim_index = std::nullopt,
                const std::optional<std::size_t> hardware_index = std::nullopt) {
  issues->push_back(JointMapIssue{
      .code = code,
      .severity = severity,
      .entry_index = entry_index,
      .sim_index = sim_index,
      .hardware_index = hardware_index,
      .message = message,
  });
}

[[nodiscard]] bool range_is_valid(const JointValueRange& range) noexcept {
  return !range.lower.has_value() || !range.upper.has_value() ||
         *range.lower <= *range.upper;
}

void validate_limit(std::vector<JointMapIssue>* issues,
                    const std::optional<JointValueRange>& range,
                    const std::size_t entry_index,
                    const JointMapEntry& entry, const JointField field,
                    const std::string& side_name) {
  if (!range.has_value() || range_is_valid(*range)) {
    return;
  }

  push_issue(
      issues, JointMapIssueCode::kInvalidRange, JointMapIssueSeverity::kError,
      entry_name(entry, entry_index) + " has invalid " + side_name + " " +
          field_name(field) + " range " + format_range(*range),
      entry_index, entry.sim_index, entry.hardware_index);
}

void validate_transform(std::vector<JointMapIssue>* issues,
                        const JointFieldTransform& transform,
                        const std::size_t entry_index,
                        const JointMapEntry& entry, const JointField field) {
  if (std::isfinite(transform.sign) && std::isfinite(transform.offset) &&
      transform.sign != 0.0) {
    return;
  }

  std::ostringstream stream;
  stream << entry_name(entry, entry_index) << " has invalid "
         << field_name(field) << " transform (sign=" << transform.sign
         << ", offset=" << transform.offset << ")";
  push_issue(issues, JointMapIssueCode::kInvalidTransform,
             JointMapIssueSeverity::kError, stream.str(), entry_index,
             entry.sim_index, entry.hardware_index);
}

void validate_name(std::vector<JointMapIssue>* issues,
                   const std::optional<std::string>& name,
                   const std::size_t entry_index, const JointMapEntry& entry,
                   const std::string& side_name) {
  if (!name.has_value() || !name->empty()) {
    return;
  }

  push_issue(issues, JointMapIssueCode::kInvalidEntry,
             JointMapIssueSeverity::kError,
             entry_name(entry, entry_index) + " has an empty " + side_name +
                 " name",
             entry_index, entry.sim_index, entry.hardware_index);
}

void validate_dense_indexes(
    std::vector<JointMapIssue>* issues, const std::vector<IndexedEntryRef>& refs,
    const std::optional<std::size_t> expected_count, const bool require_dense,
    const std::string& axis_name, const JointMapIssueCode missing_code,
    const JointMapIssueCode unexpected_code) {
  if (expected_count.has_value()) {
    for (const IndexedEntryRef& ref : refs) {
      if (ref.index >= *expected_count) {
        push_issue(issues, unexpected_code, JointMapIssueSeverity::kError,
                   axis_name + " index " + std::to_string(ref.index) +
                       " is outside expected width " +
                       std::to_string(*expected_count),
                   ref.entry_index,
                   axis_name == "sim" ? std::optional<std::size_t>(ref.index)
                                      : std::nullopt,
                   axis_name == "hardware"
                       ? std::optional<std::size_t>(ref.index)
                       : std::nullopt);
      }
    }
  }

  if (!require_dense) {
    return;
  }

  const std::size_t dense_count = expected_count.value_or(refs.size());
  std::vector<bool> present(dense_count, false);

  for (const IndexedEntryRef& ref : refs) {
    if (ref.index < dense_count) {
      present[ref.index] = true;
      continue;
    }

    if (!expected_count.has_value()) {
      push_issue(issues, unexpected_code, JointMapIssueSeverity::kError,
                 axis_name + " index " + std::to_string(ref.index) +
                     " exceeds dense enabled range [0, " +
                     std::to_string(dense_count) + ")",
                 ref.entry_index,
                 axis_name == "sim" ? std::optional<std::size_t>(ref.index)
                                    : std::nullopt,
                 axis_name == "hardware"
                     ? std::optional<std::size_t>(ref.index)
                     : std::nullopt);
    }
  }

  for (std::size_t expected_index = 0; expected_index < dense_count;
       ++expected_index) {
    if (present[expected_index]) {
      continue;
    }

    push_issue(issues, missing_code, JointMapIssueSeverity::kError,
               "missing enabled " + axis_name + " index " +
                   std::to_string(expected_index),
               std::nullopt,
               axis_name == "sim"
                   ? std::optional<std::size_t>(expected_index)
                   : std::nullopt,
               axis_name == "hardware"
                   ? std::optional<std::size_t>(expected_index)
                   : std::nullopt);
  }
}

[[nodiscard]] std::size_t projection_width(const std::vector<JointMapEntry>& entries,
                                           const bool use_sim_index) {
  std::size_t width = 0;
  for (const JointMapEntry& entry : entries) {
    const std::size_t index = use_sim_index ? entry.sim_index : entry.hardware_index;
    width = std::max(width, index + 1);
  }
  return width;
}

[[nodiscard]] double apply_forward_transform(const double value,
                                             const JointFieldTransform& transform) {
  return value * transform.sign + transform.offset;
}

[[nodiscard]] double apply_reverse_transform(const double value,
                                             const JointFieldTransform& transform) {
  return (value - transform.offset) / transform.sign;
}

void report_range_issue(std::vector<JointMapIssue>* issues,
                        const JointMapEntry& entry,
                        const std::size_t entry_index, const JointField field,
                        const std::string& side_name, const double value,
                        const JointValueRange& range) {
  std::ostringstream stream;
  stream << entry_name(entry, entry_index) << " " << side_name << " "
         << field_name(field) << " value " << value
         << " is outside configured range " << format_range(range);
  push_issue(issues, JointMapIssueCode::kOutOfRange,
             JointMapIssueSeverity::kWarning, stream.str(), entry_index,
             entry.sim_index, entry.hardware_index);
}

void report_width_mismatch(std::vector<JointMapIssue>* issues,
                           const std::string& vector_name,
                           const std::size_t actual_width,
                           const std::size_t expected_width) {
  push_issue(issues, JointMapIssueCode::kInputWidthMismatch,
             JointMapIssueSeverity::kError,
             vector_name + " width mismatch: expected " +
                 std::to_string(expected_width) + ", got " +
                 std::to_string(actual_width));
}

[[nodiscard]] bool validate_input_width(std::vector<JointMapIssue>* issues,
                                        const std::vector<double>& values,
                                        const std::size_t expected_width,
                                        const std::string& vector_name) {
  if (values.size() == expected_width) {
    return true;
  }

  report_width_mismatch(issues, vector_name, values.size(), expected_width);
  return false;
}

[[nodiscard]] bool block_field_width_ok(std::vector<JointMapIssue>* issues,
                                         const std::vector<double>& values,
                                         const std::size_t expected_width,
                                         const std::string& vector_name) {
  if (values.empty()) {
    return true;
  }

  return validate_input_width(issues, values, expected_width, vector_name);
}

[[nodiscard]] bool block_field_width_ok(std::vector<JointMapIssue>* issues,
                                        const std::vector<std::uint8_t>& values,
                                        const std::size_t expected_width,
                                        const std::string& vector_name) {
  if (values.empty()) {
    return true;
  }

  if (values.size() == expected_width) {
    return true;
  }

  report_width_mismatch(issues, vector_name, values.size(), expected_width);
  return false;
}

template <typename T>
[[nodiscard]] bool block_field_width_ok(std::vector<JointMapIssue>* issues,
                                        const std::vector<T>& values,
                                        const std::size_t expected_width,
                                        const std::string& vector_name) {
  if (values.empty()) {
    return true;
  }

  if (values.size() == expected_width) {
    return true;
  }

  report_width_mismatch(issues, vector_name, values.size(), expected_width);
  return false;
}

VectorProjectionResult project_vector_impl(const std::vector<JointMapEntry>& entries,
                                           const std::vector<double>& values,
                                           const ProjectionDirection direction,
                                           const JointField field,
                                           const std::string& vector_name) {
  VectorProjectionResult result;

  const bool from_sim = direction == ProjectionDirection::kSimToHardware;
  const std::size_t expected_input_width = projection_width(entries, from_sim);
  const std::size_t output_width = projection_width(entries, !from_sim);
  if (!validate_input_width(&result.issues, values, expected_input_width,
                            vector_name)) {
    return result;
  }

  result.values.assign(output_width, 0.0);

  for (std::size_t entry_index = 0; entry_index < entries.size(); ++entry_index) {
    const JointMapEntry& entry = entries[entry_index];
    if (!entry.enabled) {
      continue;
    }

    const std::size_t source_index = from_sim ? entry.sim_index : entry.hardware_index;
    const std::size_t destination_index =
        from_sim ? entry.hardware_index : entry.sim_index;
    const double source_value = values[source_index];

    const auto& source_limits =
        select_limit(from_sim ? entry.sim_limits : entry.hardware_limits, field);
    if (source_limits.has_value() && !source_limits->contains(source_value)) {
      report_range_issue(&result.issues, entry, entry_index, field,
                         from_sim ? "sim" : "hardware", source_value,
                         *source_limits);
    }

    const JointFieldTransform& transform = select_transform(entry.transforms, field);
    const double destination_value =
        from_sim ? apply_forward_transform(source_value, transform)
                 : apply_reverse_transform(source_value, transform);

    const auto& destination_limits =
        select_limit(from_sim ? entry.hardware_limits : entry.sim_limits, field);
    if (destination_limits.has_value() &&
        !destination_limits->contains(destination_value)) {
      report_range_issue(&result.issues, entry, entry_index, field,
                         from_sim ? "hardware" : "sim", destination_value,
                         *destination_limits);
    }

    result.values[destination_index] = destination_value;
  }

  return result;
}

[[nodiscard]] std::vector<std::uint8_t> project_motor_modes_sim_to_hardware(
    const std::vector<JointMapEntry>& entries,
    const std::vector<std::uint8_t>& values) {
  const std::size_t output_width = projection_width(entries, false);
  std::vector<std::uint8_t> projected(output_width, 0U);

  for (const JointMapEntry& entry : entries) {
    if (!entry.enabled) {
      continue;
    }
    projected[entry.hardware_index] = values[entry.sim_index];
  }

  return projected;
}

template <typename T>
[[nodiscard]] std::vector<T> project_values_by_index(
    const std::vector<JointMapEntry>& entries, const std::vector<T>& values,
    const ProjectionDirection direction, const T& default_value) {
  const bool from_sim = direction == ProjectionDirection::kSimToHardware;
  const std::size_t output_width = projection_width(entries, !from_sim);
  std::vector<T> projected(output_width, default_value);

  for (const JointMapEntry& entry : entries) {
    if (!entry.enabled) {
      continue;
    }
    const std::size_t source_index = from_sim ? entry.sim_index : entry.hardware_index;
    const std::size_t destination_index =
        from_sim ? entry.hardware_index : entry.sim_index;
    projected[destination_index] = values[source_index];
  }

  return projected;
}

}

JointMapValidationReport JointMap::validate_entries(
    const std::vector<JointMapEntry>& entries,
    const JointMapValidationOptions& options) {
  JointMapValidationReport report;

  std::unordered_map<std::size_t, std::size_t> sim_index_to_entry;
  std::unordered_map<std::size_t, std::size_t> hardware_index_to_entry;
  std::unordered_map<std::string, std::size_t> sim_name_to_entry;
  std::unordered_map<std::string, std::size_t> hardware_name_to_entry;
  std::vector<IndexedEntryRef> enabled_sim_indexes;
  std::vector<IndexedEntryRef> enabled_hardware_indexes;

  for (std::size_t entry_index = 0; entry_index < entries.size(); ++entry_index) {
    const JointMapEntry& entry = entries[entry_index];

    validate_name(&report.issues, entry.sim_name, entry_index, entry, "sim");
    validate_name(&report.issues, entry.hardware_name, entry_index, entry,
                  "hardware");

    validate_limit(&report.issues, entry.sim_limits.position, entry_index, entry,
                   JointField::kPosition, "sim");
    validate_limit(&report.issues, entry.sim_limits.velocity, entry_index, entry,
                   JointField::kVelocity, "sim");
    validate_limit(&report.issues, entry.sim_limits.effort, entry_index, entry,
                   JointField::kEffort, "sim");
    validate_limit(&report.issues, entry.sim_limits.stiffness, entry_index,
                   entry, JointField::kStiffness, "sim");
    validate_limit(&report.issues, entry.sim_limits.damping, entry_index, entry,
                   JointField::kDamping, "sim");
    validate_limit(&report.issues, entry.sim_limits.acceleration, entry_index,
                   entry, JointField::kAcceleration, "sim");
    validate_limit(&report.issues, entry.hardware_limits.position, entry_index,
                   entry, JointField::kPosition, "hardware");
    validate_limit(&report.issues, entry.hardware_limits.velocity, entry_index,
                   entry, JointField::kVelocity, "hardware");
    validate_limit(&report.issues, entry.hardware_limits.effort, entry_index,
                   entry, JointField::kEffort, "hardware");
    validate_limit(&report.issues, entry.hardware_limits.stiffness, entry_index,
                   entry, JointField::kStiffness, "hardware");
    validate_limit(&report.issues, entry.hardware_limits.damping, entry_index,
                   entry, JointField::kDamping, "hardware");
    validate_limit(&report.issues, entry.hardware_limits.acceleration,
                   entry_index, entry, JointField::kAcceleration, "hardware");

    validate_transform(&report.issues, entry.transforms.position, entry_index,
                       entry, JointField::kPosition);
    validate_transform(&report.issues, entry.transforms.velocity, entry_index,
                       entry, JointField::kVelocity);
    validate_transform(&report.issues, entry.transforms.effort, entry_index,
                       entry, JointField::kEffort);
    validate_transform(&report.issues, entry.transforms.stiffness, entry_index,
                       entry, JointField::kStiffness);
    validate_transform(&report.issues, entry.transforms.damping, entry_index,
                       entry, JointField::kDamping);
    validate_transform(&report.issues, entry.transforms.acceleration,
                       entry_index, entry, JointField::kAcceleration);

    if (!entry.enabled) {
      continue;
    }

    ++report.enabled_entry_count;
    enabled_sim_indexes.push_back(
        IndexedEntryRef{.entry_index = entry_index, .index = entry.sim_index});
    enabled_hardware_indexes.push_back(IndexedEntryRef{.entry_index = entry_index,
                                                       .index = entry.hardware_index});

    const auto [sim_it, sim_inserted] =
        sim_index_to_entry.emplace(entry.sim_index, entry_index);
    if (!sim_inserted) {
      push_issue(&report.issues, JointMapIssueCode::kDuplicateSimIndex,
                 JointMapIssueSeverity::kError,
                 entry_name(entry, entry_index) + " reuses sim index " +
                     std::to_string(entry.sim_index) + " already claimed by entry[" +
                     std::to_string(sim_it->second) + "]",
                 entry_index, entry.sim_index, entry.hardware_index);
    }

    const auto [hardware_it, hardware_inserted] =
        hardware_index_to_entry.emplace(entry.hardware_index, entry_index);
    if (!hardware_inserted) {
      push_issue(&report.issues, JointMapIssueCode::kDuplicateHardwareIndex,
                 JointMapIssueSeverity::kError,
                 entry_name(entry, entry_index) + " reuses hardware index " +
                     std::to_string(entry.hardware_index) +
                     " already claimed by entry[" +
                     std::to_string(hardware_it->second) + "]",
                 entry_index, entry.sim_index, entry.hardware_index);
    }

    if (options.require_unique_enabled_sim_names && entry.sim_name.has_value()) {
      const auto [name_it, inserted] =
          sim_name_to_entry.emplace(*entry.sim_name, entry_index);
      if (!inserted) {
        push_issue(&report.issues, JointMapIssueCode::kDuplicateSimName,
                   JointMapIssueSeverity::kError,
                   entry_name(entry, entry_index) + " reuses sim name '" +
                       *entry.sim_name + "' already claimed by entry[" +
                       std::to_string(name_it->second) + "]",
                   entry_index, entry.sim_index, entry.hardware_index);
      }
    }

    if (options.require_unique_enabled_hardware_names &&
        entry.hardware_name.has_value()) {
      const auto [name_it, inserted] =
          hardware_name_to_entry.emplace(*entry.hardware_name, entry_index);
      if (!inserted) {
        push_issue(&report.issues, JointMapIssueCode::kDuplicateHardwareName,
                   JointMapIssueSeverity::kError,
                   entry_name(entry, entry_index) +
                       " reuses hardware name '" + *entry.hardware_name +
                       "' already claimed by entry[" +
                       std::to_string(name_it->second) + "]",
                   entry_index, entry.sim_index, entry.hardware_index);
      }
    }
  }

  validate_dense_indexes(
      &report.issues, enabled_sim_indexes, options.expected_sim_joint_count,
      options.require_dense_enabled_sim_indexes, "sim",
      JointMapIssueCode::kMissingExpectedSimIndex,
      JointMapIssueCode::kUnexpectedSimIndex);
  validate_dense_indexes(
      &report.issues, enabled_hardware_indexes,
      options.expected_hardware_joint_count,
      options.require_dense_enabled_hardware_indexes, "hardware",
      JointMapIssueCode::kMissingExpectedHardwareIndex,
      JointMapIssueCode::kUnexpectedHardwareIndex);

  return report;
}

JointMapBuildResult JointMap::build(std::vector<JointMapEntry> entries,
                                    const JointMapValidationOptions& options) {
  JointMapBuildResult result;
  result.validation = validate_entries(entries, options);
  if (!result.validation.ok()) {
    return result;
  }

  auto joint_map = std::unique_ptr<JointMap>(new JointMap(std::move(entries)));
  result.joint_map = std::move(*joint_map);
  return result;
}

const std::vector<JointMapEntry>& JointMap::entries() const noexcept {
  return entries_;
}

std::size_t JointMap::size() const noexcept {
  return entries_.size();
}

std::size_t JointMap::enabled_entry_count() const noexcept {
  return static_cast<std::size_t>(std::count_if(
      entries_.begin(), entries_.end(),
      [](const JointMapEntry& entry) { return entry.enabled; }));
}

CommandProjectionResult JointMap::project_command_sim_to_hardware(
    const core::CommandFrame& sim_command) const {
  CommandProjectionResult result;

  const std::size_t sim_width = projection_width(entries_, true);
  bool width_ok = true;
  width_ok &= block_field_width_ok(&result.issues, sim_command.joints.position,
                                   sim_width, "command.position");
  width_ok &= block_field_width_ok(&result.issues, sim_command.joints.velocity,
                                   sim_width, "command.velocity");
  width_ok &= block_field_width_ok(&result.issues, sim_command.joints.effort,
                                   sim_width, "command.effort");
  width_ok &= block_field_width_ok(&result.issues, sim_command.joints.stiffness,
                                   sim_width, "command.stiffness");
  width_ok &= block_field_width_ok(&result.issues, sim_command.joints.damping,
                                    sim_width, "command.damping");
  width_ok &= block_field_width_ok(&result.issues, sim_command.joints.motor_mode,
                                   sim_width, "command.motor_mode");
  if (!width_ok) {
    return result;
  }

  core::CommandFrame hardware_command = sim_command;

  if (!sim_command.joints.position.empty()) {
    auto projected = project_vector_impl(entries_, sim_command.joints.position,
                                         ProjectionDirection::kSimToHardware,
                                         JointField::kPosition,
                                         "command.position");
    hardware_command.joints.position = std::move(projected.values);
    result.issues.insert(result.issues.end(), projected.issues.begin(),
                         projected.issues.end());
  } else {
    hardware_command.joints.position.clear();
  }

  if (!sim_command.joints.velocity.empty()) {
    auto projected = project_vector_impl(entries_, sim_command.joints.velocity,
                                         ProjectionDirection::kSimToHardware,
                                         JointField::kVelocity,
                                         "command.velocity");
    hardware_command.joints.velocity = std::move(projected.values);
    result.issues.insert(result.issues.end(), projected.issues.begin(),
                         projected.issues.end());
  } else {
    hardware_command.joints.velocity.clear();
  }

  if (!sim_command.joints.effort.empty()) {
    auto projected = project_vector_impl(entries_, sim_command.joints.effort,
                                         ProjectionDirection::kSimToHardware,
                                         JointField::kEffort,
                                         "command.effort");
    hardware_command.joints.effort = std::move(projected.values);
    result.issues.insert(result.issues.end(), projected.issues.begin(),
                         projected.issues.end());
  } else {
    hardware_command.joints.effort.clear();
  }

  if (!sim_command.joints.stiffness.empty()) {
    auto projected = project_vector_impl(entries_, sim_command.joints.stiffness,
                                         ProjectionDirection::kSimToHardware,
                                         JointField::kStiffness,
                                         "command.stiffness");
    hardware_command.joints.stiffness = std::move(projected.values);
    result.issues.insert(result.issues.end(), projected.issues.begin(),
                         projected.issues.end());
  } else {
    hardware_command.joints.stiffness.clear();
  }

  if (!sim_command.joints.damping.empty()) {
    auto projected = project_vector_impl(entries_, sim_command.joints.damping,
                                         ProjectionDirection::kSimToHardware,
                                         JointField::kDamping,
                                         "command.damping");
    hardware_command.joints.damping = std::move(projected.values);
    result.issues.insert(result.issues.end(), projected.issues.begin(),
                         projected.issues.end());
  } else {
    hardware_command.joints.damping.clear();
  }

  if (!sim_command.joints.motor_mode.empty()) {
    hardware_command.joints.motor_mode =
        project_motor_modes_sim_to_hardware(entries_, sim_command.joints.motor_mode);
  } else {
    hardware_command.joints.motor_mode.clear();
  }

  result.command = std::move(hardware_command);
  return result;
}

StateProjectionResult JointMap::project_state_hardware_to_sim(
    const core::StateFrame& hardware_state) const {
  StateProjectionResult result;

  const std::size_t hardware_width = projection_width(entries_, false);
  bool width_ok = true;
  width_ok &= block_field_width_ok(&result.issues, hardware_state.joints.position,
                                   hardware_width, "state.position");
  width_ok &= block_field_width_ok(&result.issues, hardware_state.joints.velocity,
                                   hardware_width, "state.velocity");
  width_ok &= block_field_width_ok(&result.issues, hardware_state.joints.effort,
                                   hardware_width, "state.effort");
  width_ok &= block_field_width_ok(&result.issues,
                                   hardware_state.joints.acceleration,
                                   hardware_width, "state.acceleration");
  width_ok &= block_field_width_ok(&result.issues, hardware_state.joints.motor_mode,
                                   hardware_width, "state.motor_mode");
  width_ok &= block_field_width_ok(&result.issues,
                                   hardware_state.joints.motor_temperature,
                                   hardware_width, "state.motor_temperature");
  width_ok &= block_field_width_ok(&result.issues,
                                   hardware_state.joints.motor_voltage,
                                   hardware_width, "state.motor_voltage");
  width_ok &= block_field_width_ok(&result.issues,
                                   hardware_state.joints.motor_sensor,
                                   hardware_width, "state.motor_sensor");
  width_ok &= block_field_width_ok(&result.issues,
                                   hardware_state.joints.motor_state,
                                   hardware_width, "state.motor_state");
  width_ok &= block_field_width_ok(&result.issues,
                                   hardware_state.joints.motor_reserve,
                                   hardware_width, "state.motor_reserve");
  if (!width_ok) {
    return result;
  }

  core::StateFrame sim_state = hardware_state;

  if (!hardware_state.joints.position.empty()) {
    auto projected = project_vector_impl(entries_, hardware_state.joints.position,
                                         ProjectionDirection::kHardwareToSim,
                                         JointField::kPosition,
                                         "state.position");
    sim_state.joints.position = std::move(projected.values);
    result.issues.insert(result.issues.end(), projected.issues.begin(),
                         projected.issues.end());
  } else {
    sim_state.joints.position.clear();
  }

  if (!hardware_state.joints.velocity.empty()) {
    auto projected = project_vector_impl(entries_, hardware_state.joints.velocity,
                                         ProjectionDirection::kHardwareToSim,
                                         JointField::kVelocity,
                                         "state.velocity");
    sim_state.joints.velocity = std::move(projected.values);
    result.issues.insert(result.issues.end(), projected.issues.begin(),
                         projected.issues.end());
  } else {
    sim_state.joints.velocity.clear();
  }

  if (!hardware_state.joints.effort.empty()) {
    auto projected = project_vector_impl(entries_, hardware_state.joints.effort,
                                         ProjectionDirection::kHardwareToSim,
                                         JointField::kEffort, "state.effort");
    sim_state.joints.effort = std::move(projected.values);
    result.issues.insert(result.issues.end(), projected.issues.begin(),
                         projected.issues.end());
  } else {
    sim_state.joints.effort.clear();
  }

  if (!hardware_state.joints.acceleration.empty()) {
    auto projected =
        project_vector_impl(entries_, hardware_state.joints.acceleration,
                            ProjectionDirection::kHardwareToSim,
                            JointField::kAcceleration, "state.acceleration");
    sim_state.joints.acceleration = std::move(projected.values);
    result.issues.insert(result.issues.end(), projected.issues.begin(),
                         projected.issues.end());
  } else {
    sim_state.joints.acceleration.clear();
  }

  if (!hardware_state.joints.motor_mode.empty()) {
    sim_state.joints.motor_mode = project_values_by_index<std::uint8_t>(
        entries_, hardware_state.joints.motor_mode,
        ProjectionDirection::kHardwareToSim, 0U);
  } else {
    sim_state.joints.motor_mode.clear();
  }

  if (!hardware_state.joints.motor_temperature.empty()) {
    sim_state.joints.motor_temperature =
        project_values_by_index<std::array<std::int16_t, 2>>(
            entries_, hardware_state.joints.motor_temperature,
            ProjectionDirection::kHardwareToSim, {0, 0});
  } else {
    sim_state.joints.motor_temperature.clear();
  }

  if (!hardware_state.joints.motor_voltage.empty()) {
    sim_state.joints.motor_voltage = project_values_by_index<double>(
        entries_, hardware_state.joints.motor_voltage,
        ProjectionDirection::kHardwareToSim, 0.0);
  } else {
    sim_state.joints.motor_voltage.clear();
  }

  if (!hardware_state.joints.motor_sensor.empty()) {
    sim_state.joints.motor_sensor =
        project_values_by_index<std::array<std::uint32_t, 2>>(
            entries_, hardware_state.joints.motor_sensor,
            ProjectionDirection::kHardwareToSim, {0U, 0U});
  } else {
    sim_state.joints.motor_sensor.clear();
  }

  if (!hardware_state.joints.motor_state.empty()) {
    sim_state.joints.motor_state = project_values_by_index<std::uint32_t>(
        entries_, hardware_state.joints.motor_state,
        ProjectionDirection::kHardwareToSim, 0U);
  } else {
    sim_state.joints.motor_state.clear();
  }

  if (!hardware_state.joints.motor_reserve.empty()) {
    sim_state.joints.motor_reserve =
        project_values_by_index<std::array<std::uint32_t, 4>>(
            entries_, hardware_state.joints.motor_reserve,
            ProjectionDirection::kHardwareToSim, {0U, 0U, 0U, 0U});
  } else {
    sim_state.joints.motor_reserve.clear();
  }

  result.state = std::move(sim_state);
  return result;
}

VectorProjectionResult JointMap::project_positions_sim_to_hardware(
    const std::vector<double>& sim_values) const {
  return project_vector_impl(entries_, sim_values, ProjectionDirection::kSimToHardware,
                             JointField::kPosition, "positions");
}

VectorProjectionResult JointMap::project_positions_hardware_to_sim(
    const std::vector<double>& hardware_values) const {
  return project_vector_impl(entries_, hardware_values,
                             ProjectionDirection::kHardwareToSim,
                             JointField::kPosition, "positions");
}

JointMap::JointMap(std::vector<JointMapEntry> entries)
    : entries_(std::move(entries)) {}

}
