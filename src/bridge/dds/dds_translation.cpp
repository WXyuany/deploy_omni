#include "../../../include/bridge/dds/dds_translation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include <unitree/dds_wrapper/common/crc.h>
#include "unitree/idl/hg/IMUState_.hpp"
#include "unitree/idl/hg/MotorCmd_.hpp"
#include "unitree/idl/hg/MotorState_.hpp"

namespace bridge::dds {
namespace {

constexpr std::size_t kUnitreeMotorSlotCount = 35U;

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

[[nodiscard]] std::uint64_t infer_sequence(
    const unitree_hg::msg::dds_::LowCmd_& message) {
  return static_cast<std::uint64_t>(message.reserve()[0]);
}

[[nodiscard]] std::uint64_t infer_source_time_ns(
    const unitree_hg::msg::dds_::LowCmd_& message) {
  return static_cast<std::uint64_t>(message.reserve()[1]);
}

[[nodiscard]] std::size_t configured_sim_width(const config::JointMap& joint_map) {
  std::size_t width = 0;
  for (const auto& entry : joint_map.entries()) {
    if (!entry.enabled) {
      continue;
    }
    width = std::max(width, entry.sim_index + 1);
  }
  return width;
}

[[nodiscard]] bool validate_motor_commands_finite(
    const std::array<unitree_hg::msg::dds_::MotorCmd_, kUnitreeMotorSlotCount>& motors,
    const std::size_t width) {
  for (std::size_t index = 0; index < width; ++index) {
    if (!std::isfinite(motors[index].q()) || !std::isfinite(motors[index].dq()) ||
        !std::isfinite(motors[index].tau()) || !std::isfinite(motors[index].kp()) ||
        !std::isfinite(motors[index].kd())) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool validate_lowcmd_crc(const unitree_hg::msg::dds_::LowCmd_& message) {
  auto copy = message;
  return copy.crc() == crc32_core(reinterpret_cast<uint32_t*>(&copy),
                                  (sizeof(copy) >> 2) - 1);
}

[[nodiscard]] TranslationResult validate_lowcmd(
    const unitree_hg::msg::dds_::LowCmd_& message,
    const std::uint64_t receive_time_ns,
    const std::size_t width) {
  if (width > kUnitreeMotorSlotCount) {
    return make_result(TranslationStatus::kRejected,
                       "command width exceeds Unitree motor slot count",
                       make_fault(core::FaultCategory::kInvalidCommand,
                                  core::FaultSeverity::kRecoverable,
                                  "command width exceeds Unitree motor slot count",
                                  infer_sequence(message), receive_time_ns));
  }
  if (decode_control_mode(message.mode_machine()) == core::ControlMode::kUnknown) {
    return make_result(TranslationStatus::kUnsupported,
                       "unsupported command control mode",
                       make_fault(core::FaultCategory::kInvalidCommand,
                                  core::FaultSeverity::kRecoverable,
                                  "unsupported command control mode",
                                   infer_sequence(message), receive_time_ns));
  }
  if (!validate_lowcmd_crc(message)) {
    return make_result(TranslationStatus::kRejected,
                       "command message failed crc validation",
                       make_fault(core::FaultCategory::kInvalidCommand,
                                  core::FaultSeverity::kRecoverable,
                                  "command message failed crc validation",
                                  infer_sequence(message), receive_time_ns));
  }
  if (!validate_motor_commands_finite(message.motor_cmd(), width)) {
    return make_result(TranslationStatus::kRejected,
                       "command message contains non-finite motor values",
                       make_fault(core::FaultCategory::kInvalidCommand,
                                  core::FaultSeverity::kRecoverable,
                                  "command message contains non-finite motor values",
                                  infer_sequence(message), receive_time_ns));
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
        state.imu->rpy_xyz.size() != 3U ||
        state.imu->angular_velocity_xyz.size() != 3U ||
        state.imu->linear_acceleration_xyz.size() != 3U) {
      return make_result(TranslationStatus::kRejected,
                         "imu vectors must have widths 4/3/3/3 when imu is present");
    }
    if (!has_only_finite_values(state.imu->orientation_xyzw) ||
        !has_only_finite_values(state.imu->rpy_xyz) ||
        !has_only_finite_values(state.imu->angular_velocity_xyz) ||
        !has_only_finite_values(state.imu->linear_acceleration_xyz)) {
      return make_result(TranslationStatus::kRejected,
                         "imu vectors must contain only finite values");
    }
  }
  return make_result(TranslationStatus::kOk, "state validated");
}

[[nodiscard]] unitree_hg::msg::dds_::LowState_ make_low_state_message(
    const core::StateFrame& state) {
  unitree_hg::msg::dds_::LowState_ message;
  message.version({1U, 0U});
  message.mode_pr(state.mode_pr);
  const std::uint8_t encoded_mode = encode_control_mode(state.active_mode);
  message.mode_machine(state.mode_machine == 0U ? encoded_mode : state.mode_machine);
  message.tick(static_cast<std::uint32_t>(state.sequence));

  auto motors = message.motor_state();
  const std::size_t width = std::min<std::size_t>(state.joints.width(), kUnitreeMotorSlotCount);
  for (std::size_t index = 0; index < width; ++index) {
    motors[index].mode(encode_control_mode(state.active_mode));
    motors[index].q(static_cast<float>(state.joints.position[index]));
    motors[index].dq(static_cast<float>(state.joints.velocity[index]));
    motors[index].ddq(static_cast<float>(state.joints.acceleration[index]));
    motors[index].tau_est(static_cast<float>(state.joints.effort[index]));
  }
  message.motor_state(motors);

  unitree_hg::msg::dds_::IMUState_ imu_state;
  if (state.imu.has_value()) {
    std::array<float, 4> quaternion{1.0F, 0.0F, 0.0F, 0.0F};
    std::array<float, 3> rpy{0.0F, 0.0F, 0.0F};
    std::array<float, 3> gyroscope{0.0F, 0.0F, 0.0F};
    std::array<float, 3> accelerometer{0.0F, 0.0F, 0.0F};
    quaternion[0] = static_cast<float>(state.imu->orientation_xyzw[3]);
    quaternion[1] = static_cast<float>(state.imu->orientation_xyzw[0]);
    quaternion[2] = static_cast<float>(state.imu->orientation_xyzw[1]);
    quaternion[3] = static_cast<float>(state.imu->orientation_xyzw[2]);
    std::copy_n(state.imu->rpy_xyz.begin(), 3, rpy.begin());
    std::copy_n(state.imu->angular_velocity_xyz.begin(), 3, gyroscope.begin());
    std::copy_n(state.imu->linear_acceleration_xyz.begin(), 3, accelerometer.begin());
    imu_state.quaternion(quaternion);
    imu_state.rpy(rpy);
    imu_state.gyroscope(gyroscope);
    imu_state.accelerometer(accelerometer);
  }
  message.imu_state(imu_state);
  return message;
}

}  // namespace

CommandTranslation translate_lowcmd_to_command(
    const unitree_hg::msg::dds_::LowCmd_& message,
    const std::uint64_t receive_time_ns) {
  constexpr std::size_t width = kUnitreeMotorSlotCount;
  const TranslationResult validation = validate_lowcmd(message, receive_time_ns, width);
  if (!validation.succeeded()) {
    return {.result = validation, .command = std::nullopt};
  }

  core::CommandFrame command;
  command.sequence = infer_sequence(message);
  command.source_time_ns = infer_source_time_ns(message);
  command.receive_time_ns = receive_time_ns;
  command.mode_pr = message.mode_pr();
  command.mode_machine = message.mode_machine();
  command.mode = decode_control_mode(message.mode_machine());
  command.mode_label = "unitree_lowcmd";
  command.valid = true;
  command.stale = false;
  command.degraded = false;
  command.joints.position.resize(width);
  command.joints.velocity.resize(width);
  command.joints.effort.resize(width);
  command.joints.stiffness.resize(width);
  command.joints.damping.resize(width);
  for (std::size_t index = 0; index < width; ++index) {
    command.joints.position[index] = message.motor_cmd()[index].q();
    command.joints.velocity[index] = message.motor_cmd()[index].dq();
    command.joints.effort[index] = message.motor_cmd()[index].tau();
    command.joints.stiffness[index] = message.motor_cmd()[index].kp();
    command.joints.damping[index] = message.motor_cmd()[index].kd();
  }
  return {.result = make_result(TranslationStatus::kOk, "translated lowcmd to command"),
          .command = std::move(command)};
}

CommandTranslation translate_lowcmd_to_hardware_command(
    const unitree_hg::msg::dds_::LowCmd_& message,
    const std::uint64_t receive_time_ns,
    const config::JointMap& joint_map) {
  const std::size_t width = configured_sim_width(joint_map);
  const TranslationResult validation = validate_lowcmd(message, receive_time_ns, width);
  if (!validation.succeeded()) {
    return {.result = validation, .command = std::nullopt};
  }

  core::CommandFrame command;
  command.sequence = infer_sequence(message);
  command.source_time_ns = infer_source_time_ns(message);
  command.receive_time_ns = receive_time_ns;
  command.mode_pr = message.mode_pr();
  command.mode_machine = message.mode_machine();
  command.mode = decode_control_mode(message.mode_machine());
  command.mode_label = "unitree_lowcmd";
  command.valid = true;
  command.stale = false;
  command.degraded = false;
  command.joints.position.resize(width);
  command.joints.velocity.resize(width);
  command.joints.effort.resize(width);
  command.joints.stiffness.resize(width);
  command.joints.damping.resize(width);
  for (std::size_t index = 0; index < width; ++index) {
    command.joints.position[index] = message.motor_cmd()[index].q();
    command.joints.velocity[index] = message.motor_cmd()[index].dq();
    command.joints.effort[index] = message.motor_cmd()[index].tau();
    command.joints.stiffness[index] = message.motor_cmd()[index].kp();
    command.joints.damping[index] = message.motor_cmd()[index].kd();
  }

  config::CommandProjectionResult projected =
      joint_map.project_command_sim_to_hardware(command);
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

}  // namespace bridge::dds
