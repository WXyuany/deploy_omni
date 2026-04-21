#include <cassert>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "../../include/bridge/config/joint_map.hpp"

namespace {

using bridge::config::CommandProjectionResult;
using bridge::config::JointFieldTransform;
using bridge::config::JointMap;
using bridge::config::JointMapBuildResult;
using bridge::config::JointMapEntry;
using bridge::config::JointMapIssue;
using bridge::config::JointMapIssueCode;
using bridge::config::JointMapIssueSeverity;
using bridge::config::JointMapValidationOptions;
using bridge::core::CommandFrame;
using bridge::core::StateFrame;

constexpr double kTolerance = 1e-9;

JointMapEntry MakeEntry(const std::size_t sim_index, const std::size_t hardware_index,
                        const std::string& sim_name,
                        const std::string& hardware_name) {
  JointMapEntry entry;
  entry.sim_index = sim_index;
  entry.hardware_index = hardware_index;
  entry.sim_name = sim_name;
  entry.hardware_name = hardware_name;
  entry.label = sim_name;
  entry.enabled = true;
  return entry;
}

std::vector<JointMapEntry> MakeProjectionEntries() {
  std::vector<JointMapEntry> entries;

  JointMapEntry first = MakeEntry(0, 2, "left_hip_yaw", "l_hip_yaw");
  first.transforms.position = JointFieldTransform{.sign = 1.0, .offset = 0.5};
  first.transforms.velocity = JointFieldTransform{.sign = -1.0, .offset = 0.0};
  first.transforms.effort = JointFieldTransform{.sign = 1.0, .offset = 1.0};
  first.transforms.stiffness = JointFieldTransform{.sign = 2.0, .offset = 0.0};
  first.transforms.damping = JointFieldTransform{.sign = 1.0, .offset = -0.5};
  first.transforms.acceleration =
      JointFieldTransform{.sign = -1.0, .offset = 0.25};
  entries.push_back(first);

  JointMapEntry second = MakeEntry(1, 0, "left_knee", "l_knee");
  second.transforms.position = JointFieldTransform{.sign = -1.0, .offset = 0.25};
  second.transforms.velocity = JointFieldTransform{.sign = 2.0, .offset = 1.0};
  second.transforms.effort = JointFieldTransform{.sign = -1.0, .offset = 0.0};
  second.transforms.stiffness = JointFieldTransform{.sign = 1.0, .offset = 0.0};
  second.transforms.damping = JointFieldTransform{.sign = 3.0, .offset = 0.0};
  second.transforms.acceleration =
      JointFieldTransform{.sign = 1.0, .offset = -0.5};
  entries.push_back(second);

  JointMapEntry third = MakeEntry(2, 1, "right_ankle_roll", "r_ankle_roll");
  third.transforms.position = JointFieldTransform{.sign = 0.5, .offset = -1.0};
  third.transforms.velocity = JointFieldTransform{.sign = 1.0, .offset = -2.0};
  third.transforms.effort = JointFieldTransform{.sign = 2.0, .offset = -3.0};
  third.transforms.stiffness = JointFieldTransform{.sign = 1.0, .offset = 0.75};
  third.transforms.damping = JointFieldTransform{.sign = -1.0, .offset = 0.0};
  third.transforms.acceleration = JointFieldTransform{.sign = 2.0, .offset = 1.0};
  entries.push_back(third);

  return entries;
}

std::vector<JointMapEntry> MakeSampleShapedEntries(const std::size_t count) {
  std::vector<JointMapEntry> entries;
  entries.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    entries.push_back(MakeEntry(index, index, "sim_joint_" + std::to_string(index),
                                "hw_joint_" + std::to_string(index)));
  }
  return entries;
}

std::vector<JointMapEntry> MakeEntriesWithDisabledMiddleSlot() {
  std::vector<JointMapEntry> entries;

  JointMapEntry first = MakeEntry(0, 0, "joint_0", "hw_0");
  first.transforms.position = JointFieldTransform{.sign = 1.0, .offset = 0.5};
  entries.push_back(first);

  JointMapEntry disabled = MakeEntry(1, 1, "joint_1", "hw_1");
  disabled.enabled = false;
  disabled.transforms.position = JointFieldTransform{.sign = -1.0, .offset = 3.0};
  entries.push_back(disabled);

  JointMapEntry third = MakeEntry(2, 2, "joint_2", "hw_2");
  third.transforms.position = JointFieldTransform{.sign = 2.0, .offset = -1.0};
  entries.push_back(third);

  return entries;
}

std::vector<JointMapEntry> MakeEntriesWithDisabledHighestSlot() {
  std::vector<JointMapEntry> entries;

  JointMapEntry first = MakeEntry(0, 0, "joint_0", "hw_0");
  first.transforms.position = JointFieldTransform{.sign = 1.0, .offset = 0.5};
  entries.push_back(first);

  JointMapEntry second = MakeEntry(1, 1, "joint_1", "hw_1");
  second.transforms.position = JointFieldTransform{.sign = 2.0, .offset = -1.0};
  entries.push_back(second);

  JointMapEntry disabled = MakeEntry(2, 2, "joint_2", "hw_2");
  disabled.enabled = false;
  disabled.transforms.position = JointFieldTransform{.sign = -1.0, .offset = 3.0};
  entries.push_back(disabled);

  return entries;
}

bool HasIssueCode(const std::vector<JointMapIssue>& issues,
                  const JointMapIssueCode code) {
  for (const JointMapIssue& issue : issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
}

bool HasIssueSeverity(const std::vector<JointMapIssue>& issues,
                      const JointMapIssueSeverity severity) {
  for (const JointMapIssue& issue : issues) {
    if (issue.severity == severity) {
      return true;
    }
  }
  return false;
}

void AssertNear(const double actual, const double expected) {
  assert(std::fabs(actual - expected) < kTolerance);
}

void AssertVectorNear(const std::vector<double>& actual,
                      const std::vector<double>& expected) {
  assert(actual.size() == expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    AssertNear(actual[index], expected[index]);
  }
}

void TestProjectionRoundTripBehavior() {
  JointMapBuildResult build_result = JointMap::build(MakeProjectionEntries());
  assert(build_result.succeeded());
  assert(build_result.joint_map.has_value());
  assert(build_result.joint_map->size() == 3U);
  assert(build_result.joint_map->enabled_entry_count() == 3U);

  const auto hardware_positions =
      build_result.joint_map->project_positions_sim_to_hardware({1.0, 2.0, 6.0});
  assert(hardware_positions.succeeded());
  AssertVectorNear(hardware_positions.values, {-1.75, 2.0, 1.5});

  const auto sim_positions =
      build_result.joint_map->project_positions_hardware_to_sim(
          hardware_positions.values);
  assert(sim_positions.succeeded());
  AssertVectorNear(sim_positions.values, {1.0, 2.0, 6.0});

  CommandFrame sim_command;
  sim_command.valid = true;
  sim_command.joints.position = {1.0, 2.0, 6.0};
  sim_command.joints.velocity = {4.0, 5.0, 6.0};
  sim_command.joints.effort = {7.0, 8.0, 9.0};
  sim_command.joints.stiffness = {1.0, 2.0, 3.0};
  sim_command.joints.damping = {1.0, 2.0, 3.0};

  const CommandProjectionResult command_projection =
      build_result.joint_map->project_command_sim_to_hardware(sim_command);
  assert(command_projection.succeeded());
  assert(command_projection.command.has_value());
  AssertVectorNear(command_projection.command->joints.position,
                   {-1.75, 2.0, 1.5});
  AssertVectorNear(command_projection.command->joints.velocity,
                   {11.0, 4.0, -4.0});
  AssertVectorNear(command_projection.command->joints.effort,
                   {-8.0, 15.0, 8.0});
  AssertVectorNear(command_projection.command->joints.stiffness,
                   {2.0, 3.75, 2.0});
  AssertVectorNear(command_projection.command->joints.damping,
                   {6.0, -3.0, 0.5});

  StateFrame hardware_state;
  hardware_state.valid = true;
  hardware_state.joints.position = {-1.75, 2.0, 1.5};
  hardware_state.joints.velocity = {11.0, 4.0, -4.0};
  hardware_state.joints.effort = {-8.0, 15.0, 8.0};
  hardware_state.joints.acceleration = {1.5, 7.0, -0.75};

  const auto state_projection =
      build_result.joint_map->project_state_hardware_to_sim(hardware_state);
  assert(state_projection.succeeded());
  assert(state_projection.state.has_value());
  AssertVectorNear(state_projection.state->joints.position, {1.0, 2.0, 6.0});
  AssertVectorNear(state_projection.state->joints.velocity, {4.0, 5.0, 6.0});
  AssertVectorNear(state_projection.state->joints.effort, {7.0, 8.0, 9.0});
  AssertVectorNear(state_projection.state->joints.acceleration,
                   {1.0, 2.0, 3.0});
}

void TestDuplicateIndexValidation() {
  std::vector<JointMapEntry> entries = MakeProjectionEntries();
  entries[2].sim_index = 1;

  const auto report = JointMap::validate_entries(entries);
  assert(!report.ok());
  assert(HasIssueCode(report.issues, JointMapIssueCode::kDuplicateSimIndex));
}

void TestWidthMismatchDetection() {
  JointMapBuildResult build_result = JointMap::build(MakeProjectionEntries());
  assert(build_result.succeeded());
  assert(build_result.joint_map.has_value());

  CommandFrame sim_command;
  sim_command.valid = true;
  sim_command.joints.position = {1.0, 2.0};

  const auto projection =
      build_result.joint_map->project_command_sim_to_hardware(sim_command);
  assert(!projection.succeeded());
  assert(!projection.command.has_value());
  assert(HasIssueCode(projection.issues, JointMapIssueCode::kInputWidthMismatch));
}

void TestStartupValidationFailureForInvalidSampleShapedMapping() {
  std::vector<JointMapEntry> entries = MakeSampleShapedEntries(12);
  entries[11].sim_index = 12;

  JointMapValidationOptions options;
  options.expected_sim_joint_count = 12;
  options.expected_hardware_joint_count = 12;
  options.require_dense_enabled_sim_indexes = true;
  options.require_dense_enabled_hardware_indexes = true;
  options.require_unique_enabled_sim_names = true;
  options.require_unique_enabled_hardware_names = true;

  const JointMapBuildResult build_result = JointMap::build(entries, options);
  assert(!build_result.succeeded());
  assert(!build_result.joint_map.has_value());
  assert(HasIssueCode(build_result.validation.issues,
                      JointMapIssueCode::kMissingExpectedSimIndex));
  assert(HasIssueCode(build_result.validation.issues,
                      JointMapIssueCode::kUnexpectedSimIndex));
}

void TestProjectionWarningsKeepProjectedFramesAvailable() {
  std::vector<JointMapEntry> entries = MakeProjectionEntries();
  entries[0].sim_limits.position = bridge::config::JointValueRange{
      .lower = -0.5,
      .upper = 0.5,
  };
  entries[0].hardware_limits.position = bridge::config::JointValueRange{
      .lower = -0.5,
      .upper = 0.5,
  };

  JointMapBuildResult build_result = JointMap::build(std::move(entries));
  assert(build_result.succeeded());
  assert(build_result.joint_map.has_value());

  CommandFrame sim_command;
  sim_command.valid = true;
  sim_command.joints.position = {1.0, 2.0, 6.0};

  const CommandProjectionResult command_projection =
      build_result.joint_map->project_command_sim_to_hardware(sim_command);
  assert(command_projection.command.has_value());
  assert(!command_projection.succeeded());
  assert(HasIssueCode(command_projection.issues, JointMapIssueCode::kOutOfRange));
  assert(HasIssueSeverity(command_projection.issues,
                          JointMapIssueSeverity::kWarning));
  AssertVectorNear(command_projection.command->joints.position,
                   {-1.75, 2.0, 1.5});

  StateFrame hardware_state;
  hardware_state.valid = true;
  hardware_state.joints.position = {-1.75, 2.0, 1.5};

  const auto state_projection =
      build_result.joint_map->project_state_hardware_to_sim(hardware_state);
  assert(state_projection.state.has_value());
  assert(!state_projection.succeeded());
  assert(HasIssueCode(state_projection.issues, JointMapIssueCode::kOutOfRange));
  assert(HasIssueSeverity(state_projection.issues,
                          JointMapIssueSeverity::kWarning));
  AssertVectorNear(state_projection.state->joints.position, {1.0, 2.0, 6.0});
}

void TestDisabledEntriesKeepSlotWidthAndProjectZeros() {
  JointMapValidationOptions options;
  options.require_dense_enabled_sim_indexes = false;
  options.require_dense_enabled_hardware_indexes = false;
  JointMapBuildResult build_result =
      JointMap::build(MakeEntriesWithDisabledHighestSlot(), options);
  assert(build_result.succeeded());
  assert(build_result.joint_map.has_value());
  assert(build_result.joint_map->enabled_entry_count() == 2U);

  const auto hardware_positions =
      build_result.joint_map->project_positions_sim_to_hardware({1.0, 4.0, 9.0});
  assert(hardware_positions.succeeded());
  AssertVectorNear(hardware_positions.values, {1.5, 7.0, 0.0});

  CommandFrame sim_command;
  sim_command.valid = true;
  sim_command.joints.position = {1.0, 4.0, 9.0};
  sim_command.joints.velocity = {2.0, 6.0, 8.0};
  sim_command.joints.effort = {3.0, 5.0, 7.0};
  sim_command.joints.stiffness = {4.0, 4.0, 6.0};
  sim_command.joints.damping = {5.0, 3.0, 5.0};

  const auto command_projection =
      build_result.joint_map->project_command_sim_to_hardware(sim_command);
  assert(command_projection.succeeded());
  assert(command_projection.command.has_value());
  AssertVectorNear(command_projection.command->joints.position, {1.5, 7.0, 0.0});
  AssertVectorNear(command_projection.command->joints.velocity, {2.0, 6.0, 0.0});
  AssertVectorNear(command_projection.command->joints.effort, {3.0, 5.0, 0.0});
  AssertVectorNear(command_projection.command->joints.stiffness, {4.0, 4.0, 0.0});
  AssertVectorNear(command_projection.command->joints.damping, {5.0, 3.0, 0.0});

  StateFrame hardware_state;
  hardware_state.valid = true;
  hardware_state.joints.position = {1.5, 7.0, 42.0};
  hardware_state.joints.velocity = {2.0, 6.0, 24.0};
  hardware_state.joints.effort = {3.0, 5.0, 12.0};
  hardware_state.joints.acceleration = {4.0, 8.0, 99.0};

  const auto state_projection =
      build_result.joint_map->project_state_hardware_to_sim(hardware_state);
  assert(state_projection.succeeded());
  assert(state_projection.state.has_value());
  AssertVectorNear(state_projection.state->joints.position, {1.0, 4.0, 0.0});
  AssertVectorNear(state_projection.state->joints.velocity, {2.0, 6.0, 0.0});
  AssertVectorNear(state_projection.state->joints.effort, {3.0, 5.0, 0.0});
  AssertVectorNear(state_projection.state->joints.acceleration, {4.0, 8.0, 0.0});
}

}

int main() {
  TestProjectionRoundTripBehavior();
  TestDuplicateIndexValidation();
  TestWidthMismatchDetection();
  TestStartupValidationFailureForInvalidSampleShapedMapping();
  TestProjectionWarningsKeepProjectedFramesAvailable();
  TestDisabledEntriesKeepSlotWidthAndProjectZeros();
  return 0;
}
