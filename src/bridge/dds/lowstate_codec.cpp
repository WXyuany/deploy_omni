#include "../../../include/bridge/dds/transport.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bridge::dds {
namespace {

constexpr std::byte kLowStateMagic[4]{std::byte{'L'}, std::byte{'S'},
                                      std::byte{'T'}, std::byte{'A'}};
constexpr std::byte kLowStatusMagic[4]{std::byte{'L'}, std::byte{'S'},
                                       std::byte{'T'}, std::byte{'S'}};
constexpr std::uint8_t kLowStateVersion = 1U;
constexpr std::size_t kStateHeaderSize = 24U;
constexpr std::size_t kStatusHeaderSize = 32U;

constexpr std::uint8_t kStateFlagValid = 1U << 0U;
constexpr std::uint8_t kStateFlagStale = 1U << 1U;
constexpr std::uint8_t kStateFlagDegraded = 1U << 2U;
constexpr std::uint8_t kStateFlagCommandStale = 1U << 3U;

constexpr std::uint8_t kStatusFlagValid = 1U << 0U;
constexpr std::uint8_t kStatusFlagStale = 1U << 1U;
constexpr std::uint8_t kStatusFlagDegraded = 1U << 2U;
constexpr std::uint8_t kStatusFlagCommandStale = 1U << 3U;
constexpr std::uint8_t kStatusFlagFaultPresent = 1U << 4U;
constexpr std::uint8_t kStatusFlagFaultLatched = 1U << 5U;

constexpr std::uint32_t kJointFieldPosition = 1U << 0U;
constexpr std::uint32_t kJointFieldVelocity = 1U << 1U;
constexpr std::uint32_t kJointFieldEffort = 1U << 2U;
constexpr std::uint32_t kJointFieldAcceleration = 1U << 3U;

constexpr std::uint8_t kImuFieldOrientation = 1U << 0U;
constexpr std::uint8_t kImuFieldAngularVelocity = 1U << 1U;
constexpr std::uint8_t kImuFieldLinearAcceleration = 1U << 2U;
constexpr std::uint8_t kImuFlagPresent = 1U << 0U;
constexpr std::uint8_t kImuFlagValid = 1U << 1U;
constexpr std::uint8_t kImuFlagStale = 1U << 2U;

constexpr std::string_view kStateTopicName = "bridge/state";
constexpr std::string_view kStateTypeName = "bridge.dds.LowState";
constexpr std::string_view kStatusTopicName = "bridge/status";
constexpr std::string_view kStatusTypeName = "bridge.dds.LowStatus";

[[nodiscard]] core::BridgeFault make_fault(const core::StateFrame& state,
                                           const core::FaultCategory category,
                                           const core::FaultSeverity severity,
                                           std::string message) {
  return {
      .category = category,
      .severity = severity,
      .lifecycle_state = state.lifecycle_state,
      .message = std::move(message),
      .latched = false,
      .sequence = state.sequence,
      .timestamp_ns = state.publish_time_ns != 0U ? state.publish_time_ns
                                                  : state.sample_time_ns,
  };
}

[[nodiscard]] StateEncodeResult make_result(
    const TransportCallStatus status, std::string detail,
    std::optional<core::BridgeFault> fault = std::nullopt,
    std::vector<DdsPublication> publications = {}) {
  return {
      .result = {
          .status = status,
          .detail = std::move(detail),
          .fault = std::move(fault),
      },
      .publications = std::move(publications),
  };
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

[[nodiscard]] std::uint8_t encode_control_mode(
    const core::ControlMode mode) noexcept {
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

[[nodiscard]] std::uint8_t encode_fault_category(
    const core::FaultCategory category) noexcept {
  return static_cast<std::uint8_t>(category);
}

[[nodiscard]] std::uint8_t encode_fault_severity(
    const core::FaultSeverity severity) noexcept {
  return static_cast<std::uint8_t>(severity);
}

[[nodiscard]] std::uint8_t make_state_flags(
    const core::StateFrame& state) noexcept {
  std::uint8_t flags = 0U;
  if (state.valid) {
    flags |= kStateFlagValid;
  }
  if (state.stale) {
    flags |= kStateFlagStale;
  }
  if (state.degraded) {
    flags |= kStateFlagDegraded;
  }
  if (state.command_stale) {
    flags |= kStateFlagCommandStale;
  }
  return flags;
}

[[nodiscard]] std::uint8_t make_status_flags(
    const core::StateFrame& state) noexcept {
  std::uint8_t flags = 0U;
  if (state.valid) {
    flags |= kStatusFlagValid;
  }
  if (state.stale) {
    flags |= kStatusFlagStale;
  }
  if (state.degraded) {
    flags |= kStatusFlagDegraded;
  }
  if (state.command_stale) {
    flags |= kStatusFlagCommandStale;
  }
  if (state.fault.has_value() && state.fault->has_fault()) {
    flags |= kStatusFlagFaultPresent;
    if (state.fault->latched) {
      flags |= kStatusFlagFaultLatched;
    }
  }
  return flags;
}

[[nodiscard]] std::uint8_t make_imu_flags(const core::ImuSample& imu) noexcept {
  std::uint8_t flags = kImuFlagPresent;
  if (imu.valid) {
    flags |= kImuFlagValid;
  }
  if (imu.stale) {
    flags |= kImuFlagStale;
  }
  return flags;
}

[[nodiscard]] bool has_only_finite_values(const std::vector<double>& values) {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] std::optional<std::string> validate_joint_block(
    const core::JointStateBlock& joints) {
  const std::size_t width = joints.width();
  if ((!joints.position.empty() && joints.position.size() != width) ||
      (!joints.velocity.empty() && joints.velocity.size() != width) ||
      (!joints.effort.empty() && joints.effort.size() != width) ||
      (!joints.acceleration.empty() && joints.acceleration.size() != width)) {
    return std::string{"state joint vectors must share the same width"};
  }
  if (!has_only_finite_values(joints.position) ||
      !has_only_finite_values(joints.velocity) ||
      !has_only_finite_values(joints.effort) ||
      !has_only_finite_values(joints.acceleration)) {
    return std::string{"state joint vectors must contain only finite values"};
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<std::string> validate_imu(
    const std::optional<core::ImuSample>& imu) {
  if (!imu.has_value()) {
    return std::nullopt;
  }
  if (!imu->orientation_xyzw.empty() && imu->orientation_xyzw.size() != 4U) {
    return std::string{"state imu orientation must have exactly four values"};
  }
  if (!imu->angular_velocity_xyz.empty() &&
      imu->angular_velocity_xyz.size() != 3U) {
    return std::string{"state imu angular velocity must have exactly three values"};
  }
  if (!imu->linear_acceleration_xyz.empty() &&
      imu->linear_acceleration_xyz.size() != 3U) {
    return std::string{"state imu linear acceleration must have exactly three values"};
  }
  if (!has_only_finite_values(imu->orientation_xyzw) ||
      !has_only_finite_values(imu->angular_velocity_xyz) ||
      !has_only_finite_values(imu->linear_acceleration_xyz)) {
    return std::string{"state imu vectors must contain only finite values"};
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<std::string> validate_state(
    const core::StateFrame& state) {
  if (const std::optional<std::string> joint_issue =
          validate_joint_block(state.joints);
      joint_issue.has_value()) {
    return joint_issue;
  }
  if (const std::optional<std::string> imu_issue = validate_imu(state.imu);
      imu_issue.has_value()) {
    return imu_issue;
  }
  if (state.mode_label.size() > std::numeric_limits<std::uint32_t>::max()) {
    return std::string{"state mode label is too large to encode"};
  }
  if (state.fault.has_value() &&
      state.fault->message.size() > std::numeric_limits<std::uint32_t>::max()) {
    return std::string{"state fault message is too large to encode"};
  }

  const std::size_t joint_width = state.joints.width();
  const std::size_t joint_field_count =
      static_cast<std::size_t>(!state.joints.position.empty()) +
      static_cast<std::size_t>(!state.joints.velocity.empty()) +
      static_cast<std::size_t>(!state.joints.effort.empty()) +
      static_cast<std::size_t>(!state.joints.acceleration.empty());
  if (joint_field_count != 0U &&
      joint_width > std::numeric_limits<std::size_t>::max() /
                        (joint_field_count * sizeof(double))) {
    return std::string{"state joint payload is too large to encode"};
  }

  if (state.imu.has_value()) {
    const std::size_t imu_value_count =
        state.imu->orientation_xyzw.size() + state.imu->angular_velocity_xyz.size() +
        state.imu->linear_acceleration_xyz.size();
    if (imu_value_count >
        std::numeric_limits<std::size_t>::max() / sizeof(double)) {
      return std::string{"state imu payload is too large to encode"};
    }
  }

  return std::nullopt;
}

void append_u8(std::vector<std::byte>* payload, const std::uint8_t value) {
  payload->push_back(static_cast<std::byte>(value));
}

void append_u32_le(std::vector<std::byte>* payload, std::uint32_t value) {
  for (std::size_t index = 0; index < sizeof(value); ++index) {
    append_u8(payload, static_cast<std::uint8_t>(value & 0xFFU));
    value >>= 8U;
  }
}

void append_u64_le(std::vector<std::byte>* payload, std::uint64_t value) {
  for (std::size_t index = 0; index < sizeof(value); ++index) {
    append_u8(payload, static_cast<std::uint8_t>(value & 0xFFU));
    value >>= 8U;
  }
}

void append_double_le(std::vector<std::byte>* payload, const double value) {
  std::uint64_t raw_bits = 0U;
  std::memcpy(&raw_bits, &value, sizeof(value));
  append_u64_le(payload, raw_bits);
}

void append_string(std::vector<std::byte>* payload, const std::string& value) {
  append_u32_le(payload, static_cast<std::uint32_t>(value.size()));
  for (const char character : value) {
    payload->push_back(static_cast<std::byte>(character));
  }
}

[[nodiscard]] std::uint32_t make_joint_field_mask(
    const core::JointStateBlock& joints) noexcept {
  std::uint32_t mask = 0U;
  if (!joints.position.empty()) {
    mask |= kJointFieldPosition;
  }
  if (!joints.velocity.empty()) {
    mask |= kJointFieldVelocity;
  }
  if (!joints.effort.empty()) {
    mask |= kJointFieldEffort;
  }
  if (!joints.acceleration.empty()) {
    mask |= kJointFieldAcceleration;
  }
  return mask;
}

void append_joint_values(std::vector<std::byte>* payload,
                         const std::vector<double>& values) {
  for (const double value : values) {
    append_double_le(payload, value);
  }
}

[[nodiscard]] std::uint8_t make_imu_field_mask(
    const core::ImuSample& imu) noexcept {
  std::uint8_t mask = 0U;
  if (!imu.orientation_xyzw.empty()) {
    mask |= kImuFieldOrientation;
  }
  if (!imu.angular_velocity_xyz.empty()) {
    mask |= kImuFieldAngularVelocity;
  }
  if (!imu.linear_acceleration_xyz.empty()) {
    mask |= kImuFieldLinearAcceleration;
  }
  return mask;
}

[[nodiscard]] DdsPublication encode_state_publication(
    const core::StateFrame& state, const bool degraded) {
  DdsPublication publication;
  publication.kind = DdsPublicationKind::kState;
  publication.endpoint = {
      .topic_name = std::string{kStateTopicName},
      .type_name = std::string{kStateTypeName},
  };
  publication.sequence = state.sequence;
  publication.timestamp_ns =
      state.publish_time_ns != 0U ? state.publish_time_ns : state.sample_time_ns;
  publication.degraded = degraded;

  std::vector<std::byte>& payload = publication.payload;
  payload.reserve(kStateHeaderSize +
                  (state.joints.position.size() + state.joints.velocity.size() +
                   state.joints.effort.size() + state.joints.acceleration.size() +
                   (state.imu.has_value()
                        ? state.imu->orientation_xyzw.size() +
                              state.imu->angular_velocity_xyz.size() +
                              state.imu->linear_acceleration_xyz.size()
                        : 0U)) *
                      sizeof(double));

  for (const std::byte byte : kLowStateMagic) {
    payload.push_back(byte);
  }
  append_u8(&payload, kLowStateVersion);
  append_u8(&payload, make_state_flags(state));
  append_u8(&payload, state.imu.has_value() ? make_imu_flags(*state.imu) : 0U);
  append_u8(&payload, state.imu.has_value() ? make_imu_field_mask(*state.imu) : 0U);
  append_u32_le(&payload, static_cast<std::uint32_t>(state.joints.width()));
  append_u32_le(&payload, make_joint_field_mask(state.joints));
  append_u64_le(&payload, state.sample_time_ns);

  append_joint_values(&payload, state.joints.position);
  append_joint_values(&payload, state.joints.velocity);
  append_joint_values(&payload, state.joints.effort);
  append_joint_values(&payload, state.joints.acceleration);

  if (state.imu.has_value()) {
    append_u64_le(&payload, state.imu->sample_time_ns);
    append_joint_values(&payload, state.imu->orientation_xyzw);
    append_joint_values(&payload, state.imu->angular_velocity_xyz);
    append_joint_values(&payload, state.imu->linear_acceleration_xyz);
  }

  return publication;
}

[[nodiscard]] DdsPublication encode_status_publication(
    const core::StateFrame& state, const bool degraded) {
  DdsPublication publication;
  publication.kind = DdsPublicationKind::kStatus;
  publication.endpoint = {
      .topic_name = std::string{kStatusTopicName},
      .type_name = std::string{kStatusTypeName},
  };
  publication.sequence = state.sequence;
  publication.timestamp_ns =
      state.publish_time_ns != 0U ? state.publish_time_ns : state.sample_time_ns;
  publication.degraded = degraded;

  std::vector<std::byte>& payload = publication.payload;
  const std::size_t fault_message_size =
      (state.fault.has_value() && state.fault->has_fault())
          ? state.fault->message.size()
          : 0U;
  payload.reserve(kStatusHeaderSize + sizeof(std::uint64_t) +
                  sizeof(std::uint32_t) + state.mode_label.size() +
                  sizeof(std::uint32_t) + fault_message_size);

  for (const std::byte byte : kLowStatusMagic) {
    payload.push_back(byte);
  }
  append_u8(&payload, kLowStateVersion);
  append_u8(&payload, encode_lifecycle_state(state.lifecycle_state));
  append_u8(&payload, encode_control_mode(state.active_mode));
  append_u8(&payload, make_status_flags(state));
  append_u64_le(&payload, state.sample_time_ns);
  append_u64_le(&payload, state.publish_time_ns);
  append_u64_le(&payload, state.fault.has_value() && state.fault->has_fault()
                              ? state.fault->timestamp_ns
                              : 0U);
  append_string(&payload, state.mode_label);

  if (state.fault.has_value() && state.fault->has_fault()) {
    append_u8(&payload, encode_fault_category(state.fault->category));
    append_u8(&payload, encode_fault_severity(state.fault->severity));
    append_u8(&payload, encode_lifecycle_state(state.fault->lifecycle_state));
    append_u8(&payload, 0U);
    append_u64_le(&payload, state.fault->sequence);
    append_string(&payload, state.fault->message);
  } else {
    append_u8(&payload, encode_fault_category(core::FaultCategory::kNone));
    append_u8(&payload, encode_fault_severity(core::FaultSeverity::kInfo));
    append_u8(&payload, encode_lifecycle_state(state.lifecycle_state));
    append_u8(&payload, 0U);
    append_u64_le(&payload, 0U);
    append_u32_le(&payload, 0U);
  }

  return publication;
}

class LowStateEncoder final : public DdsStateEncoder {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "lowstate_codec";
  }

  [[nodiscard]] StateEncodeResult encode(const core::StateFrame& state) override {
    if (const std::optional<std::string> issue = validate_state(state);
        issue.has_value()) {
      return make_result(
          TransportCallStatus::kRejected, *issue,
          make_fault(state, core::FaultCategory::kInternal,
                     core::FaultSeverity::kRecoverable, *issue));
    }

    const bool degraded = state.degraded ||
                          (state.fault.has_value() && state.fault->has_fault());
    std::vector<DdsPublication> publications;
    publications.reserve(2U);
    publications.push_back(encode_state_publication(state, degraded));
    publications.push_back(encode_status_publication(state, degraded));

    return make_result(degraded ? TransportCallStatus::kDegraded
                                : TransportCallStatus::kOk,
                       degraded ? "lowstate payload encoded with degraded status"
                                : "lowstate payload encoded",
                       degraded
                           ? std::optional<core::BridgeFault>(make_fault(
                                 state, core::FaultCategory::kTransport,
                                 core::FaultSeverity::kWarning,
                                 "state encoded for DDS publication with degraded status"))
                           : std::nullopt,
                       std::move(publications));
  }
};

}

[[nodiscard]] std::unique_ptr<DdsStateEncoder> make_lowstate_codec() {
  return std::unique_ptr<DdsStateEncoder>(new LowStateEncoder());
}

}
