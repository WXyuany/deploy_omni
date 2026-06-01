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
  bool safe_command_supported{false};
  bool connection_alive{false};
  bool degraded{false};
  std::uint64_t last_read_time_ns{0};
  std::uint64_t last_write_time_ns{0};
  std::optional<BridgeFault> fault;
};

struct AdapterReadResult {
  AdapterCallResult result;
  std::optional<StateFrame> state;
};

struct AdapterHealthResult {
  AdapterCallResult result;
  AdapterHealth health;
};

}
