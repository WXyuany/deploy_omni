#include "../../../include/bridge/dds/dds_translation.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace bridge::dds {
namespace {

[[nodiscard]] core::BridgeFault make_fault(const core::FaultCategory category,
                                           const core::FaultSeverity severity,
                                           std::string message,
                                           const std::uint64_t sequence,
                                           const std::uint64_t timestamp_ns) {
  return {
      .category = category,
      .severity = severity,
      .lifecycle_state = core::LifecycleState::kCreated,
      .message = std::move(message),
      .latched = false,
      .sequence = sequence,
      .timestamp_ns = timestamp_ns,
  };
}

[[nodiscard]] TranslationResult make_result(
    const TranslationStatus status, std::string detail,
    std::optional<core::BridgeFault> fault = std::nullopt) {
  return {
      .status = status,
      .detail = std::move(detail),
      .fault = std::move(fault),
  };
}

[[nodiscard]] core::ControlMode decode_control_mode(const std::uint8_t code) noexcept {
  switch (code) {
    case 1U:
      return core::ControlMode::kSafeIdle;
    case 2U:
      return core::ControlMode::kTorque;
    case 3U:
      return core::ControlMode::kVelocity;
    case 4U:
      return core::ControlMode::kPosition;
    case 5U:
      return core::ControlMode::kHybrid;
    default:
      return core::ControlMode::kUnknown;
  }
}

[[nodiscard]] std::uint8_t encode_control_mode(const core::ControlMode mode) noexcept {
  switch (mode) {
    case core::ControlMode::kUnknown:
      return 0U;
    case core::ControlMode::kSafeIdle:
      return 1U;
    case core::ControlMode::kTorque:
      return 2U;
    case core::ControlMode::kVelocity:
      return 3U;
    case core::ControlMode::kPosition:
      return 4U;
    case core::ControlMode::kHybrid:
      return 5U;
  }
  return 0U;
}

[[nodiscard]] std::uint8_t encode_lifecycle_state(
    const core::LifecycleState state) noexcept {
  switch (state) {
    case core::LifecycleState::kCreated:
      return 1U;
    case core::LifecycleState::kInitializing:
      return 2U;
    case core::LifecycleState::kWaitingForState:
      return 3U;
    case core::LifecycleState::kActive:
      return 4U;
    case core::LifecycleState::kFaulted:
      return 5U;
    case core::LifecycleState::kStopping:
      return 6U;
    case core::LifecycleState::kStopped:
      return 7U;
  }
  return 0U;
}

[[nodiscard]] bool has_only_finite_values(const std::vector<double>& values) {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] std::optional<std::string> first_projection_error(
    const std::vector<config::JointMapIssue>& issues) {
  for (const config::JointMapIssue& issue : issues) {
    if (issue.severity == config::JointMapIssueSeverity::kError) {
      return issue.message;
    }
  }
  return std::nullopt;
}

[[nodiscard]] TranslationResult validate_lowcmd(const bridge_wire::LowCmd& message,
                                                const std::uint64_t receive_time_ns) {
  if (!message.valid()) {
    return make_result(TranslationStatus::kRejected,
                       "command message is marked invalid",
                       make_fault(core::FaultCategory::kInvalidCommand,
                                  core::FaultSeverity::kRecoverable,
                                  "command message is marked invalid",
                                  message.seq_id(),
                                  receive_time_ns != 0U ? receive_time_ns
                                                        : message.source_time_ns()));
  }
  if (decode_control_mode(message.control_mode()) == core::ControlMode::kUnknown) {
    return make_result(TranslationStatus::kUnsupported,
                       "unsupported command control mode",
                       make_fault(core::FaultCategory::kInvalidCommand,
                                  core::FaultSeverity::kRecoverable,
                                  "unsupported command control mode",
                                  message.seq_id(),
                                  receive_time_ns != 0U ? receive_time_ns
                                                        : message.source_time_ns()));
  }

  const std::size_t width = !message.position().empty()   ? message.position().size()
                            : !message.velocity().empty() ? message.velocity().size()
                            : !message.effort().empty()   ? message.effort().size()
                            : !message.stiffness().empty() ? message.stiffness().size()
                                                          : message.damping().size();
  const auto same_width = [width](const std::vector<double>& values) {
    return values.empty() || values.size() == width;
  };
  if (!same_width(message.position()) || !same_width(message.velocity()) ||
      !same_width(message.effort()) || !same_width(message.stiffness()) ||
      !same_width(message.damping())) {
    return make_result(TranslationStatus::kRejected,
                       "command message vectors must share the same width",
                       make_fault(core::FaultCategory::kInvalidCommand,
                                  core::FaultSeverity::kRecoverable,
                                  "command message vectors must share the same width",
                                  message.seq_id(),
                                  receive_time_ns != 0U ? receive_time_ns
                                                        : message.source_time_ns()));
  }
  if (!has_only_finite_values(message.position()) ||
      !has_only_finite_values(message.velocity()) ||
      !has_only_finite_values(message.effort()) ||
      !has_only_finite_values(message.stiffness()) ||
      !has_only_finite_values(message.damping())) {
    return make_result(TranslationStatus::kRejected,
                       "command message vectors must contain only finite values",
                       make_fault(core::FaultCategory::kInvalidCommand,
                                  core::FaultSeverity::kRecoverable,
                                  "command message vectors must contain only finite values",
                                  message.seq_id(),
                                  receive_time_ns != 0U ? receive_time_ns
                                                        : message.source_time_ns()));
  }
  return make_result(TranslationStatus::kOk, "lowcmd message validated");
}

[[nodiscard]] TranslationResult validate_state(const core::StateFrame& state) {
  const std::size_t width = state.joints.width();
  const auto same_width = [width](const std::vector<double>& values) {
    return values.empty() || values.size() == width;
  };
  if (!same_width(state.joints.position) || !same_width(state.joints.velocity) ||
      !same_width(state.joints.effort) || !same_width(state.joints.acceleration)) {
    return make_result(TranslationStatus::kRejected,
                       "state joint vectors must share the same width");
  }
  if (!has_only_finite_values(state.joints.position) ||
      !has_only_finite_values(state.joints.velocity) ||
      !has_only_finite_values(state.joints.effort) ||
      !has_only_finite_values(state.joints.acceleration)) {
    return make_result(TranslationStatus::kRejected,
                       "state joint vectors must contain only finite values");
  }
  if (state.imu.has_value()) {
    if (state.imu->orientation_xyzw.size() != 4U ||
        state.imu->angular_velocity_xyz.size() != 3U ||
        state.imu->linear_acceleration_xyz.size() != 3U) {
      return make_result(TranslationStatus::kRejected,
                         "imu vectors must have widths 4/3/3 when imu is present");
    }
    if (!has_only_finite_values(state.imu->orientation_xyzw) ||
        !has_only_finite_values(state.imu->angular_velocity_xyz) ||
        !has_only_finite_values(state.imu->linear_acceleration_xyz)) {
      return make_result(TranslationStatus::kRejected,
                         "imu vectors must contain only finite values");
    }
  }
  return make_result(TranslationStatus::kOk, "state validated");
}

[[nodiscard]] bridge_wire::LowState make_low_state_message(const core::StateFrame& state) {
  bridge_wire::LowState message;
  message.seq_id(state.sequence);
  message.sample_time_ns(state.sample_time_ns);
  message.publish_time_ns(state.publish_time_ns);
  message.lifecycle_state(encode_lifecycle_state(state.lifecycle_state));
  message.active_mode(encode_control_mode(state.active_mode));
  message.mode_label(state.mode_label);
  message.valid(state.valid);
  message.stale(state.stale);
  message.degraded(state.degraded);
  message.command_stale(state.command_stale);
  message.position(state.joints.position);
  message.velocity(state.joints.velocity);
  message.effort(state.joints.effort);
  message.acceleration(state.joints.acceleration);
  if (state.imu.has_value()) {
    message.imu_present(true);
    message.imu_valid(state.imu->valid);
    message.imu_stale(state.imu->stale);
    message.imu_sample_time_ns(state.imu->sample_time_ns);
    message.imu_orientation_xyzw(state.imu->orientation_xyzw);
    message.imu_angular_velocity_xyz(state.imu->angular_velocity_xyz);
    message.imu_linear_acceleration_xyz(state.imu->linear_acceleration_xyz);
  } else {
    message.imu_present(false);
    message.imu_valid(false);
    message.imu_stale(false);
    message.imu_sample_time_ns(0U);
    message.imu_orientation_xyzw({});
    message.imu_angular_velocity_xyz({});
    message.imu_linear_acceleration_xyz({});
  }
  return message;
}

[[nodiscard]] bridge_wire::BridgeStatus make_status_message(const core::StateFrame& state) {
  bridge_wire::BridgeStatus message;
  message.seq_id(state.sequence);
  message.sample_time_ns(state.sample_time_ns);
  message.publish_time_ns(state.publish_time_ns);
  message.lifecycle_state(encode_lifecycle_state(state.lifecycle_state));
  message.control_mode(encode_control_mode(state.active_mode));
  message.mode_label(state.mode_label);
  message.valid(state.valid);
  message.stale(state.stale);
  message.degraded(state.degraded);
  message.command_stale(state.command_stale);
  if (state.fault.has_value() && state.fault->has_fault()) {
    message.fault_present(true);
    message.fault_category(static_cast<std::uint8_t>(state.fault->category));
    message.fault_severity(static_cast<std::uint8_t>(state.fault->severity));
    message.fault_lifecycle_state(encode_lifecycle_state(state.fault->lifecycle_state));
    message.fault_latched(state.fault->latched);
    message.fault_sequence(state.fault->sequence);
    message.fault_timestamp_ns(state.fault->timestamp_ns);
    message.fault_message(state.fault->message);
  } else {
    message.fault_present(false);
    message.fault_category(0U);
    message.fault_severity(0U);
    message.fault_lifecycle_state(0U);
    message.fault_latched(false);
    message.fault_sequence(0U);
    message.fault_timestamp_ns(0U);
    message.fault_message("");
  }
  return message;
}

}

CommandTranslation translate_lowcmd_to_command(const bridge_wire::LowCmd& message,
                                               const std::uint64_t receive_time_ns) {
  const TranslationResult validation = validate_lowcmd(message, receive_time_ns);
  if (!validation.succeeded()) {
    return {.result = validation, .command = std::nullopt};
  }

  core::CommandFrame command;
  command.sequence = message.seq_id();
  command.source_time_ns = message.source_time_ns();
  command.receive_time_ns = receive_time_ns;
  command.mode = decode_control_mode(message.control_mode());
  command.mode_label = message.mode_label();
  command.valid = message.valid();
  command.stale = message.stale();
  command.degraded = message.degraded();
  command.joints.position = message.position();
  command.joints.velocity = message.velocity();
  command.joints.effort = message.effort();
  command.joints.stiffness = message.stiffness();
  command.joints.damping = message.damping();
  return {.result = make_result(TranslationStatus::kOk, "translated lowcmd to command"),
          .command = std::move(command)};
}

CommandTranslation translate_lowcmd_to_hardware_command(
    const bridge_wire::LowCmd& message, const std::uint64_t receive_time_ns,
    const config::JointMap& joint_map) {
  CommandTranslation translated = translate_lowcmd_to_command(message, receive_time_ns);
  if (!translated.result.succeeded() || !translated.command.has_value()) {
    return translated;
  }
  config::CommandProjectionResult projected =
      joint_map.project_command_sim_to_hardware(*translated.command);
  const std::optional<std::string> projection_error =
      first_projection_error(projected.issues);
  if (!projected.command.has_value() || projection_error.has_value()) {
    return {
        .result = make_result(TranslationStatus::kRejected,
                              projection_error.value_or("joint-map command projection failed")),
        .command = std::nullopt,
    };
  }
  return {.result = make_result(TranslationStatus::kOk, "translated lowcmd to hardware command"),
          .command = std::move(*projected.command)};
}

StateTranslation translate_state_to_lowstate(const core::StateFrame& state) {
  const TranslationResult validation = validate_state(state);
  if (!validation.succeeded()) {
    return {.result = validation, .state = std::nullopt};
  }
  return {.result = make_result(TranslationStatus::kOk, "translated state to lowstate"),
          .state = make_low_state_message(state)};
}

StateTranslation translate_hardware_state_to_lowstate(
    const core::StateFrame& hardware_state,
    const config::JointMap& joint_map) {
  config::StateProjectionResult projected =
      joint_map.project_state_hardware_to_sim(hardware_state);
  const std::optional<std::string> projection_error =
      first_projection_error(projected.issues);
  if (!projected.state.has_value() || projection_error.has_value()) {
    return {.result = make_result(TranslationStatus::kRejected,
                                  projection_error.value_or("joint-map state projection failed")),
            .state = std::nullopt};
  }
  return translate_state_to_lowstate(*projected.state);
}

StatusTranslation translate_state_to_bridge_status(const core::StateFrame& state) {
  const TranslationResult validation = validate_state(state);
  if (!validation.succeeded()) {
    return {.result = validation, .status = std::nullopt};
  }
  return {.result = make_result(TranslationStatus::kOk, "translated state to bridge status"),
          .status = make_status_message(state)};
}

StatusTranslation translate_hardware_state_to_bridge_status(
    const core::StateFrame& hardware_state,
    const config::JointMap& joint_map) {
  config::StateProjectionResult projected =
      joint_map.project_state_hardware_to_sim(hardware_state);
  const std::optional<std::string> projection_error =
      first_projection_error(projected.issues);
  if (!projected.state.has_value() || projection_error.has_value()) {
    return {.result = make_result(TranslationStatus::kRejected,
                                  projection_error.value_or("joint-map state projection failed")),
            .status = std::nullopt};
  }
  return translate_state_to_bridge_status(*projected.state);
}

}
