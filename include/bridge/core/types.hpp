#pragma once

#include <cstdint>
#include <optional>
#include <array>
#include <string>
#include <vector>

namespace bridge::core {

enum class LifecycleState {
  kCreated,
  kInitializing,
  kWaitingForState,
  kActive,
  kFaulted,
  kStopping,
  kStopped,
};

enum class ControlMode {
  kUnknown,
  kSafeIdle,
  kTorque,
  kVelocity,
  kPosition,
  kHybrid,
};

enum class FaultCategory {
  kNone,
  kConfiguration,
  kTransport,
  kInvalidCommand,
  kCommandStale,
  kAdapterInitialize,
  kAdapterRead,
  kAdapterWrite,
  kAdapterHealth,
  kAdapterShutdown,
  kSafety,
  kInternal,
};

enum class FaultSeverity {
  kInfo,
  kWarning,
  kRecoverable,
  kFatal,
};

enum class AdapterCallStatus {
  kOk,
  kDegraded,
  kUnsupported,
  kRetryableError,
  kFatalError,
};

enum class HealthState {
  kUnknown,
  kHealthy,
  kDegraded,
  kFaulted,
};

enum class RobotCapability : std::uint32_t {
  kNone = 0U,
  kJointState = 1U << 0U,
  kJointPositionCommand = 1U << 1U,
  kJointVelocityCommand = 1U << 2U,
  kJointTorqueCommand = 1U << 3U,
  kCartesianMotion = 1U << 4U,
  kEndEffector = 1U << 5U,
  kKinematics = 1U << 6U,
  kTeach = 1U << 7U,
  kImu = 1U << 8U,
  kCamera = 1U << 9U,
  kPointCloud = 1U << 10U,
  kNavigation = 1U << 11U,
  kAudio = 1U << 12U,
  kLighting = 1U << 13U,
};

struct RobotCapabilitySet {
  std::uint32_t bits{0U};

  [[nodiscard]] constexpr bool contains(const RobotCapability capability) const noexcept {
    return (bits & static_cast<std::uint32_t>(capability)) != 0U;
  }
};

[[nodiscard]] constexpr RobotCapabilitySet operator|(
    const RobotCapability left, const RobotCapability right) noexcept {
  return {.bits = static_cast<std::uint32_t>(left) |
                    static_cast<std::uint32_t>(right)};
}

[[nodiscard]] constexpr RobotCapabilitySet operator|(
    const RobotCapabilitySet left, const RobotCapability right) noexcept {
  return {.bits = left.bits | static_cast<std::uint32_t>(right)};
}

[[nodiscard]] constexpr RobotCapabilitySet operator|(
    const RobotCapability left, const RobotCapabilitySet right) noexcept {
  return {.bits = static_cast<std::uint32_t>(left) | right.bits};
}

struct BridgeFault {
  FaultCategory category{FaultCategory::kNone};
  FaultSeverity severity{FaultSeverity::kInfo};
  LifecycleState lifecycle_state{LifecycleState::kCreated};
  std::string message;
  bool latched{false};
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};

  [[nodiscard]] bool has_fault() const noexcept {
    return category != FaultCategory::kNone;
  }
};

struct JointCommandBlock {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<double> stiffness;
  std::vector<double> damping;
  std::vector<std::uint8_t> motor_mode;

  [[nodiscard]] std::size_t width() const noexcept {
    if (!position.empty()) {
      return position.size();
    }
    if (!velocity.empty()) {
      return velocity.size();
    }
    if (!effort.empty()) {
      return effort.size();
    }
    if (!stiffness.empty()) {
      return stiffness.size();
    }
    if (!damping.empty()) {
      return damping.size();
    }
    return motor_mode.size();
  }
};

struct JointStateBlock {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<double> acceleration;
  std::vector<std::uint8_t> motor_mode;
  std::vector<std::array<std::int16_t, 2>> motor_temperature;
  std::vector<double> motor_voltage;
  std::vector<std::array<std::uint32_t, 2>> motor_sensor;
  std::vector<std::uint32_t> motor_state;
  std::vector<std::array<std::uint32_t, 4>> motor_reserve;

  [[nodiscard]] std::size_t width() const noexcept {
    if (!position.empty()) {
      return position.size();
    }
    if (!velocity.empty()) {
      return velocity.size();
    }
    if (!effort.empty()) {
      return effort.size();
    }
    if (!acceleration.empty()) {
      return acceleration.size();
    }
    if (!motor_mode.empty()) {
      return motor_mode.size();
    }
    if (!motor_temperature.empty()) {
      return motor_temperature.size();
    }
    if (!motor_voltage.empty()) {
      return motor_voltage.size();
    }
    if (!motor_sensor.empty()) {
      return motor_sensor.size();
    }
    if (!motor_state.empty()) {
      return motor_state.size();
    }
    return motor_reserve.size();
  }
};

struct ImuSample {
  std::uint64_t sample_time_ns{0};
  bool valid{false};
  bool stale{false};
  std::vector<double> orientation_xyzw;
  std::vector<double> rpy_xyz;
  std::vector<double> angular_velocity_xyz;
  std::vector<double> linear_acceleration_xyz;
  std::int16_t temperature{0};
};

struct BmsStateSample {
  std::uint8_t version_high{0};
  std::uint8_t version_low{0};
  std::uint8_t fn{0};
  std::array<std::uint16_t, 40> cell_voltage{};
  std::array<std::uint32_t, 3> bms_voltage{};
  std::int32_t current{0};
  std::uint8_t soc{0};
  std::uint8_t soh{0};
  std::array<std::int16_t, 12> temperature{};
  std::uint16_t cycle{0};
  std::uint16_t manufacturer_date{0};
  std::array<std::uint32_t, 5> bms_state{};
  std::array<std::uint32_t, 3> reserve{};
};

struct MainBoardStateSample {
  std::array<std::uint16_t, 6> fan_state{};
  std::array<std::int16_t, 6> temperature{};
  std::array<float, 6> value{};
  std::array<std::uint32_t, 6> state{};
};

struct FaultCodeSample {
  std::uint64_t sequence{0};
  std::uint64_t sample_time_ns{0};
  std::uint8_t level{0};
  std::uint8_t reserve{0};
  std::uint32_t code{0};
};

struct RobotAuxStateSnapshot {
  std::optional<ImuSample> imu;
  std::optional<BmsStateSample> bms;
  std::optional<MainBoardStateSample> mainboard;
  std::vector<FaultCodeSample> faults;

  [[nodiscard]] bool empty() const noexcept {
    return !imu.has_value() && !bms.has_value() && !mainboard.has_value() &&
           faults.empty();
  }
};

struct CommandFrame {
  std::uint64_t sequence{0};
  std::uint64_t source_time_ns{0};
  std::uint64_t receive_time_ns{0};
  std::uint8_t mode_pr{0};
  std::uint8_t mode_machine{0};
  std::optional<std::uint32_t> low_cmd_crc;
  ControlMode mode{ControlMode::kUnknown};
  std::string mode_label;
  bool valid{false};
  bool stale{false};
  bool degraded{false};
  JointCommandBlock joints;
};

struct StateFrame {
  std::uint64_t sequence{0};
  std::uint32_t tick{0};
  std::uint64_t sample_time_ns{0};
  std::uint64_t receive_time_ns{0};
  std::uint64_t publish_time_ns{0};
  std::uint64_t lowstate_callback_store_duration_ns{0};
  std::uint64_t read_start_time_ns{0};
  std::uint64_t read_complete_time_ns{0};
  std::uint64_t read_snapshot_duration_ns{0};
  std::uint64_t stateframe_build_duration_ns{0};
  std::uint64_t publish_start_time_ns{0};
  std::uint64_t publish_complete_time_ns{0};
  std::uint64_t lowstate_translate_duration_ns{0};
  std::uint64_t dds_write_duration_ns{0};
  std::uint64_t callback_drops{0};
  std::array<std::uint32_t, 2> low_state_version{1U, 0U};
  std::array<std::uint32_t, 4> low_state_reserve{};
  std::uint8_t mode_pr{0};
  std::uint8_t mode_machine{0};
  LifecycleState lifecycle_state{LifecycleState::kCreated};
  ControlMode active_mode{ControlMode::kUnknown};
  std::string mode_label;
  bool valid{false};
  bool stale{false};
  bool degraded{false};
  bool command_stale{false};
  JointStateBlock joints;
  std::array<std::uint8_t, 40> wireless_remote{};
  std::optional<ImuSample> imu;
  std::optional<BridgeFault> fault;
};

struct AdapterCallResult {
  AdapterCallStatus status{AdapterCallStatus::kOk};
  std::string detail;
  std::optional<BridgeFault> fault;

  [[nodiscard]] bool succeeded() const noexcept {
    return status == AdapterCallStatus::kOk ||
           status == AdapterCallStatus::kDegraded;
  }
};

struct AdapterHealth {
  HealthState state{HealthState::kUnknown};
  bool connection_alive{false};
  bool degraded{false};
  std::uint64_t last_read_time_ns{0};
  std::uint64_t last_write_time_ns{0};
  std::optional<BridgeFault> fault;
};

struct AdapterReadResult {
  AdapterCallResult result;
  std::optional<StateFrame> state;
  RobotAuxStateSnapshot aux_state;
};

struct AdapterHealthResult {
  AdapterCallResult result;
  AdapterHealth health;
};

}
