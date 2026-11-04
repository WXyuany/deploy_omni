#include "../../../../include/bridge/adapters/cvte_carm/cvte_carm_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include "arm_control_sdk/carm_cobot.h"
#include "arm_control_sdk/carm_dual.h"

namespace bridge::adapters::cvte_carm {
namespace {

using bridge::core::AdapterCallResult;
using bridge::core::AdapterCallStatus;
using bridge::core::AdapterHealthResult;
using bridge::core::AdapterReadResult;
using bridge::core::BridgeFault;
using bridge::core::ControlMode;
using bridge::core::FaultCategory;
using bridge::core::FaultSeverity;
using bridge::core::HealthState;
using bridge::core::LifecycleState;

struct VendorState {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  bool connected{false};
  bool servo_enabled{false};
  int controller_state{0};
  int control_mode{0};
};

[[nodiscard]] std::uint64_t now_ns() noexcept {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

[[nodiscard]] BridgeFault make_fault(FaultCategory category,
                                      FaultSeverity severity,
                                      LifecycleState lifecycle,
                                      std::string message,
                                      std::uint64_t sequence = 0) {
  return {.category = category,
          .severity = severity,
          .lifecycle_state = lifecycle,
          .message = std::move(message),
          .latched = false,
          .sequence = sequence,
          .timestamp_ns = now_ns()};
}

[[nodiscard]] AdapterCallResult status_from_vendor(int code,
                                                    std::string operation,
                                                    LifecycleState lifecycle,
                                                    std::uint64_t sequence = 0) {
  if (code > 0) {
    return {.status = AdapterCallStatus::kOk,
            .detail = std::move(operation) + " succeeded",
            .fault = std::nullopt};
  }
  return {.status = AdapterCallStatus::kRetryableError,
          .detail = std::move(operation) + " returned " + std::to_string(code),
          .fault = make_fault(FaultCategory::kTransport, FaultSeverity::kRecoverable,
                              lifecycle, "CVTE CArm operation failed", sequence)};
}

[[nodiscard]] AdapterCallResult unsupported(std::string detail,
                                            LifecycleState lifecycle,
                                            std::uint64_t sequence = 0) {
  return {.status = AdapterCallStatus::kUnsupported,
          .detail = std::move(detail),
          .fault = make_fault(FaultCategory::kInvalidCommand, FaultSeverity::kWarning,
                              lifecycle, "CVTE CArm capability is unsupported", sequence)};
}

class ArmBackend {
 public:
  virtual ~ArmBackend() = default;
  virtual int connect() = 0;
  virtual int disconnect() = 0;
  virtual bool is_connected() const = 0;
  virtual int set_ready() = 0;
  virtual int set_servo_enable(bool enabled) = 0;
  virtual int set_control_mode(int mode) = 0;
  virtual int emergency_stop() = 0;
  virtual int track_joint(const std::vector<double>& position) = 0;
  virtual VendorState read_state() = 0;
};

class SingleBackend final : public ArmBackend {
 public:
  explicit SingleBackend(const SessionConfig& config) : config_(config) {}

  int connect() override {
    ensure_object();
    return arm_->connect(config_.server_ip, config_.server_port,
                         config_.connect_timeout_seconds);
  }

  int disconnect() override { return arm_ == nullptr ? 1 : arm_->disconnect(); }

  bool is_connected() const override {
    return arm_ != nullptr && arm_->is_connected();
  }

  int set_ready() override { return arm_->set_ready(); }

  int set_servo_enable(bool enabled) override {
    return arm_->set_servo_enable(enabled);
  }

  int set_control_mode(int mode) override {
    return arm_->set_control_mode(mode);
  }

  int emergency_stop() override { return arm_->emergency_stop(); }

  int track_joint(const std::vector<double>& position) override {
    return arm_->track_joint(position);
  }

  VendorState read_state() override {
    const carm::ArmStatus status = arm_->get_status();
    return {.position = arm_->get_joint_pos(),
            .velocity = arm_->get_joint_vel(),
            .effort = arm_->get_joint_tau(),
            .connected = status.arm_is_connected,
            .servo_enabled = status.servo_status,
            .controller_state = status.state,
            .control_mode = status.fsm_state};
  }

 private:
  void ensure_object() {
    if (arm_ == nullptr) {
      arm_ = std::make_unique<carm::CArmSingleCol>(
          config_.server_ip, config_.server_port, config_.connect_timeout_seconds);
    }
  }

  SessionConfig config_;
  std::unique_ptr<carm::CArmSingleCol> arm_;
};

class DualBackend final : public ArmBackend {
 public:
  DualBackend(const SessionConfig& config, ArmSelection selection,
              std::size_t arm_joint_count)
      : config_(config), selection_(selection), arm_joint_count_(arm_joint_count) {}

  int connect() override {
    ensure_object();
    return arm_->connect(config_.server_ip, config_.server_port,
                         config_.connect_timeout_seconds);
  }

  int disconnect() override { return arm_ == nullptr ? 1 : arm_->disconnect(); }

  bool is_connected() const override {
    return arm_ != nullptr && arm_->is_connected();
  }

  int set_ready() override { return arm_->set_ready(); }

  int set_servo_enable(bool enabled) override {
    if (selection_ == ArmSelection::kLeft) {
      return arm_->set_left_servo_enable(enabled);
    }
    if (selection_ == ArmSelection::kRight) {
      return arm_->set_right_servo_enable(enabled);
    }
    return arm_->set_servo_enable(enabled);
  }

  int set_control_mode(int mode) override {
    if (selection_ == ArmSelection::kLeft) {
      return arm_->set_left_control_mode(mode);
    }
    if (selection_ == ArmSelection::kRight) {
      return arm_->set_right_control_mode(mode);
    }
    return arm_->set_control_mode(mode);
  }

  int emergency_stop() override { return arm_->emergency_stop(); }

  int track_joint(const std::vector<double>& position) override {
    if (selection_ == ArmSelection::kLeft) {
      return arm_->track_left_joint(position);
    }
    if (selection_ == ArmSelection::kRight) {
      return arm_->track_right_joint(position);
    }
    if (arm_joint_count_ == 0 || position.size() != arm_joint_count_ * 2U) {
      return -2;
    }
    const std::vector<double> left(position.begin(),
                                    position.begin() + arm_joint_count_);
    const std::vector<double> right(position.begin() + arm_joint_count_,
                                     position.end());
    return arm_->track_joint(left, right);
  }

  VendorState read_state() override {
    const carm::ArmStatus left = arm_->get_left_status();
    const carm::ArmStatus right = arm_->get_right_status();
    std::vector<double> left_position = arm_->get_left_joint_pos();
    const std::vector<double> right_position = arm_->get_right_joint_pos();
    std::vector<double> left_velocity = arm_->get_left_joint_vel();
    const std::vector<double> right_velocity = arm_->get_right_joint_vel();
    std::vector<double> left_effort = arm_->get_left_joint_tau();
    const std::vector<double> right_effort = arm_->get_right_joint_tau();

    if (selection_ == ArmSelection::kLeft) {
      return {.position = left_position,
              .velocity = left_velocity,
              .effort = left_effort,
              .connected = left.arm_is_connected,
              .servo_enabled = left.servo_status,
              .controller_state = left.state,
              .control_mode = left.fsm_state};
    }
    if (selection_ == ArmSelection::kRight) {
      return {.position = right_position,
              .velocity = right_velocity,
              .effort = right_effort,
              .connected = right.arm_is_connected,
              .servo_enabled = right.servo_status,
              .controller_state = right.state,
              .control_mode = right.fsm_state};
    }

    left_position.insert(left_position.end(), right_position.begin(), right_position.end());
    left_velocity.insert(left_velocity.end(), right_velocity.begin(), right_velocity.end());
    left_effort.insert(left_effort.end(), right_effort.begin(), right_effort.end());
    return {.position = left_position,
            .velocity = left_velocity,
            .effort = left_effort,
            .connected = left.arm_is_connected && right.arm_is_connected,
            .servo_enabled = left.servo_status && right.servo_status,
            .controller_state = std::min(left.state, right.state),
            .control_mode = left.fsm_state == right.fsm_state ? left.fsm_state : -1};
  }

 private:
  void ensure_object() {
    if (arm_ == nullptr) {
      arm_ = std::make_unique<carm::CArmDualBot>(
          config_.server_ip, config_.server_port, config_.connect_timeout_seconds);
    }
  }

  SessionConfig config_;
  ArmSelection selection_;
  std::size_t arm_joint_count_;
  std::unique_ptr<carm::CArmDualBot> arm_;
};

[[nodiscard]] std::unique_ptr<ArmBackend> make_backend(
    const CvteCarmAdapterConfig& config) {
  if (config.model == Model::kD3Dual) {
    return std::make_unique<DualBackend>(config.session, config.arm_selection,
                                         config.arm_joint_count);
  }
  return std::make_unique<SingleBackend>(config.session);
}

[[nodiscard]] bool vector_width_matches(const std::vector<double>& values,
                                        std::size_t width) noexcept {
  return values.empty() || values.size() == width;
}

[[nodiscard]] int sdk_mode(ControlMode mode) noexcept {
  switch (mode) {
    case ControlMode::kSafeIdle:
      return 0;
    case ControlMode::kPosition:
      return 1;
    case ControlMode::kHybrid:
      return 4;
    case ControlMode::kTorque:
    case ControlMode::kVelocity:
    case ControlMode::kUnknown:
      return -1;
  }
  return -1;
}

}

class CvteCarmAdapter::Impl {
 public:
  explicit Impl(CvteCarmAdapterConfig config)
      : config_(std::move(config)), backend_(make_backend(config_)) {}

  [[nodiscard]] const CvteCarmAdapterConfig& config() const noexcept { return config_; }

  [[nodiscard]] std::string_view name() const noexcept {
    return config_.model == Model::kD3Dual ? kCvteCarmD3AdapterName
                                           : kCvteCarmA3AdapterName;
  }

  [[nodiscard]] AdapterCallResult initialize() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) {
      return {.status = AdapterCallStatus::kOk,
              .detail = "CVTE CArm adapter already initialized",
              .fault = std::nullopt};
    }
    if (const auto validation = validate_config(); !validation.succeeded()) {
      lifecycle_ = LifecycleState::kFaulted;
      return validation;
    }
    lifecycle_ = LifecycleState::kInitializing;
    try {
      const int code = backend_->connect();
      if (code <= 0) {
        lifecycle_ = LifecycleState::kFaulted;
        return status_from_vendor(code, "connect", lifecycle_);
      }
    } catch (const std::exception& error) {
      lifecycle_ = LifecycleState::kFaulted;
      return {.status = AdapterCallStatus::kFatalError,
              .detail = std::string("CArm connect threw: ") + error.what(),
              .fault = make_fault(FaultCategory::kAdapterInitialize,
                                  FaultSeverity::kFatal, lifecycle_, error.what())};
    }
    initialized_ = true;
    connection_alive_ = true;
    lifecycle_ = LifecycleState::kWaitingForState;
    return {.status = AdapterCallStatus::kOk,
            .detail = "CVTE CArm adapter initialized",
            .fault = std::nullopt};
  }

  [[nodiscard]] AdapterCallResult set_ready() {
    return invoke_control("set_ready", [this] { return backend_->set_ready(); });
  }

  [[nodiscard]] AdapterCallResult set_servo_enable(bool enabled) {
    return invoke_control("set_servo_enable",
                          [this, enabled] { return backend_->set_servo_enable(enabled); });
  }

  [[nodiscard]] AdapterCallResult set_control_mode(int mode) {
    return invoke_control("set_control_mode",
                          [this, mode] { return backend_->set_control_mode(mode); });
  }

  [[nodiscard]] AdapterCallResult emergency_stop() {
    return invoke_control("emergency_stop", [this] { return backend_->emergency_stop(); });
  }

  [[nodiscard]] AdapterReadResult read_state() {
    std::lock_guard<std::mutex> lock(mutex_);
    AdapterReadResult result;
    if (!initialized_) {
      result.result = {.status = AdapterCallStatus::kRetryableError,
                       .detail = "CVTE CArm adapter must be initialized before read",
                       .fault = make_fault(FaultCategory::kAdapterRead,
                                           FaultSeverity::kRecoverable, lifecycle_,
                                           "adapter is not initialized", sequence_)};
      return result;
    }
    try {
      const VendorState vendor = backend_->read_state();
      const std::uint64_t timestamp = now_ns();
      last_read_time_ns_ = timestamp;
      connection_alive_ = vendor.connected && backend_->is_connected();
      if (!has_read_state_) {
        has_read_state_ = true;
        lifecycle_ = LifecycleState::kActive;
      }
      bridge::core::StateFrame state;
      state.sequence = ++sequence_;
      state.sample_time_ns = timestamp;
      state.receive_time_ns = timestamp;
      state.publish_time_ns = timestamp;
      state.lifecycle_state = lifecycle_;
      state.active_mode = vendor.control_mode == 1 ? ControlMode::kPosition
                                                    : ControlMode::kSafeIdle;
      state.mode_label = "carm_fsm_" + std::to_string(vendor.control_mode);
      state.valid = !vendor.position.empty();
      state.stale = !connection_alive_;
      state.degraded = !connection_alive_ || vendor.controller_state < 0;
      state.joints.position = vendor.position;
      state.joints.velocity = vendor.velocity;
      state.joints.effort = vendor.effort;
      result.state = std::move(state);
      result.result = {.status = result.state->degraded
                                   ? AdapterCallStatus::kDegraded
                                   : AdapterCallStatus::kOk,
                       .detail = "CVTE CArm state read",
                       .fault = std::nullopt};
      return result;
    } catch (const std::exception& error) {
      connection_alive_ = false;
      result.result = {.status = AdapterCallStatus::kRetryableError,
                       .detail = std::string("CArm state read threw: ") + error.what(),
                       .fault = make_fault(FaultCategory::kAdapterRead,
                                           FaultSeverity::kRecoverable, lifecycle_,
                                           error.what(), sequence_)};
      return result;
    }
  }

  [[nodiscard]] AdapterCallResult write_command(
      const bridge::core::CommandFrame& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      return {.status = AdapterCallStatus::kRetryableError,
              .detail = "CVTE CArm adapter must be initialized before command write",
              .fault = make_fault(FaultCategory::kAdapterWrite,
                                  FaultSeverity::kRecoverable, lifecycle_,
                                  "adapter is not initialized", command.sequence)};
    }
    if (!command.valid || command.stale) {
      return {.status = AdapterCallStatus::kRetryableError,
              .detail = command.stale ? "command frame is stale" : "command frame is invalid",
              .fault = make_fault(command.stale ? FaultCategory::kCommandStale
                                                : FaultCategory::kInvalidCommand,
                                  FaultSeverity::kRecoverable, lifecycle_,
                                  "command rejected", command.sequence)};
    }
    const std::size_t width = config_.joint_count();
    if (!vector_width_matches(command.joints.position, width) ||
        !command.joints.velocity.empty() || !command.joints.effort.empty() ||
        !command.joints.stiffness.empty() || !command.joints.damping.empty()) {
      return unsupported("CArm adapter accepts position commands only", lifecycle_,
                         command.sequence);
    }
    if (command.mode == ControlMode::kSafeIdle) {
      return {.status = AdapterCallStatus::kOk,
              .detail = "CArm safe idle command accepted",
              .fault = std::nullopt};
    }
    const int mode = sdk_mode(command.mode);
    if (mode != 1) {
      return unsupported("CArm adapter supports position mode for generic commands",
                         lifecycle_, command.sequence);
    }
    if (config_.safety.require_initial_state_before_motion && !has_read_state_) {
      return {.status = AdapterCallStatus::kRetryableError,
              .detail = "initial state read is required before CArm motion",
              .fault = make_fault(FaultCategory::kSafety, FaultSeverity::kRecoverable,
                                  lifecycle_, "initial state is unavailable",
                                  command.sequence)};
    }
    try {
      const int code = backend_->set_control_mode(mode);
      if (code <= 0) {
        return status_from_vendor(code, "set_control_mode", lifecycle_, command.sequence);
      }
      const int track_code = backend_->track_joint(command.joints.position);
      last_write_time_ns_ = now_ns();
      return status_from_vendor(track_code, "track_joint", lifecycle_, command.sequence);
    } catch (const std::exception& error) {
      return {.status = AdapterCallStatus::kRetryableError,
              .detail = std::string("CArm command threw: ") + error.what(),
              .fault = make_fault(FaultCategory::kAdapterWrite,
                                  FaultSeverity::kRecoverable, lifecycle_, error.what(),
                                  command.sequence)};
    }
  }

  [[nodiscard]] AdapterHealthResult query_health() const {
    std::lock_guard<std::mutex> lock(mutex_);
    AdapterHealthResult result;
    result.health.connection_alive = connection_alive_ && backend_->is_connected();
    result.health.last_read_time_ns = last_read_time_ns_;
    result.health.last_write_time_ns = last_write_time_ns_;
    result.health.degraded = !result.health.connection_alive;
    result.health.state = !initialized_
                              ? HealthState::kUnknown
                              : result.health.degraded ? HealthState::kDegraded
                                                       : HealthState::kHealthy;
    result.result = {.status = result.health.degraded ? AdapterCallStatus::kDegraded
                                                       : AdapterCallStatus::kOk,
                     .detail = "CVTE CArm health queried",
                     .fault = std::nullopt};
    return result;
  }

  [[nodiscard]] AdapterCallResult shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      lifecycle_ = LifecycleState::kStopped;
      return {.status = AdapterCallStatus::kOk,
              .detail = "CVTE CArm adapter already stopped",
              .fault = std::nullopt};
    }
    lifecycle_ = LifecycleState::kStopping;
    int code = 1;
    try {
      code = backend_->disconnect();
    } catch (const std::exception& error) {
      code = -1;
      connection_alive_ = false;
      lifecycle_ = LifecycleState::kStopped;
      initialized_ = false;
      return {.status = AdapterCallStatus::kRetryableError,
              .detail = std::string("CArm disconnect threw: ") + error.what(),
              .fault = make_fault(FaultCategory::kAdapterShutdown,
                                  FaultSeverity::kRecoverable, lifecycle_, error.what())};
    }
    initialized_ = false;
    connection_alive_ = false;
    has_read_state_ = false;
    lifecycle_ = LifecycleState::kStopped;
    return status_from_vendor(code, "disconnect", lifecycle_);
  }

 private:
  template <typename Operation>
  [[nodiscard]] AdapterCallResult invoke_control(const std::string& operation,
                                                  Operation&& callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      return {.status = AdapterCallStatus::kRetryableError,
              .detail = "CVTE CArm adapter must be initialized before " + operation,
              .fault = make_fault(FaultCategory::kAdapterWrite,
                                  FaultSeverity::kRecoverable, lifecycle_,
                                  "adapter is not initialized")};
    }
    try {
      const int code = callback();
      last_write_time_ns_ = now_ns();
      return status_from_vendor(code, operation, lifecycle_);
    } catch (const std::exception& error) {
      return {.status = AdapterCallStatus::kRetryableError,
              .detail = "CArm " + operation + " threw: " + error.what(),
              .fault = make_fault(FaultCategory::kAdapterWrite,
                                  FaultSeverity::kRecoverable, lifecycle_, error.what())};
    }
  }

  [[nodiscard]] AdapterCallResult validate_config() const {
    if (config_.session.server_ip.empty()) {
      return {.status = AdapterCallStatus::kFatalError,
              .detail = "CVTE CArm server_ip is required",
              .fault = make_fault(FaultCategory::kConfiguration, FaultSeverity::kFatal,
                                  LifecycleState::kInitializing, "server_ip is empty")};
    }
    if (config_.joint_names.empty()) {
      return {.status = AdapterCallStatus::kFatalError,
              .detail = "CVTE CArm joint_names must not be empty",
              .fault = make_fault(FaultCategory::kConfiguration, FaultSeverity::kFatal,
                                  LifecycleState::kInitializing, "joint_names is empty")};
    }
    std::unordered_set<std::string> names(config_.joint_names.begin(),
                                          config_.joint_names.end());
    if (names.size() != config_.joint_names.size()) {
      return {.status = AdapterCallStatus::kFatalError,
              .detail = "CVTE CArm joint_names must be unique",
              .fault = make_fault(FaultCategory::kConfiguration, FaultSeverity::kFatal,
                                  LifecycleState::kInitializing,
                                  "joint_names are duplicated")};
    }
    if (config_.model == Model::kA3Single &&
        config_.arm_selection != ArmSelection::kSingle) {
      return {.status = AdapterCallStatus::kFatalError,
              .detail = "A3 only supports single arm selection",
              .fault = make_fault(FaultCategory::kConfiguration, FaultSeverity::kFatal,
                                  LifecycleState::kInitializing,
                                  "invalid A3 arm selection")};
    }
    if (config_.model == Model::kD3Dual && config_.arm_selection == ArmSelection::kBoth &&
        config_.arm_joint_count == 0) {
      return {.status = AdapterCallStatus::kFatalError,
              .detail = "D3 both-arm mode requires arm_joint_count",
              .fault = make_fault(FaultCategory::kConfiguration, FaultSeverity::kFatal,
                                  LifecycleState::kInitializing,
                                  "arm_joint_count is zero")};
    }
    return {.status = AdapterCallStatus::kOk,
            .detail = "CVTE CArm config validated",
            .fault = std::nullopt};
  }

  CvteCarmAdapterConfig config_;
  std::unique_ptr<ArmBackend> backend_;
  mutable std::mutex mutex_;
  LifecycleState lifecycle_{LifecycleState::kCreated};
  bool initialized_{false};
  bool connection_alive_{false};
  bool has_read_state_{false};
  std::uint64_t sequence_{0};
  std::uint64_t last_read_time_ns_{0};
  std::uint64_t last_write_time_ns_{0};
};

CvteCarmAdapter::CvteCarmAdapter(CvteCarmAdapterConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

CvteCarmAdapter::~CvteCarmAdapter() = default;

CvteCarmAdapter::CvteCarmAdapter(CvteCarmAdapter&&) noexcept = default;

CvteCarmAdapter& CvteCarmAdapter::operator=(CvteCarmAdapter&&) noexcept = default;

std::string_view CvteCarmAdapter::name() const noexcept { return impl_->name(); }

bridge::core::RobotCapabilitySet CvteCarmAdapter::capabilities() const noexcept {
  using bridge::core::RobotCapability;
  return RobotCapability::kJointState |
         RobotCapability::kJointPositionCommand;
}

const CvteCarmAdapterConfig& CvteCarmAdapter::config() const noexcept {
  return impl_->config();
}

AdapterCallResult CvteCarmAdapter::initialize() { return impl_->initialize(); }

AdapterCallResult CvteCarmAdapter::set_ready() { return impl_->set_ready(); }

AdapterCallResult CvteCarmAdapter::set_servo_enable(bool enabled) {
  return impl_->set_servo_enable(enabled);
}

AdapterCallResult CvteCarmAdapter::set_control_mode(int mode) {
  return impl_->set_control_mode(mode);
}

AdapterCallResult CvteCarmAdapter::emergency_stop() { return impl_->emergency_stop(); }

AdapterReadResult CvteCarmAdapter::read_state() { return impl_->read_state(); }

AdapterCallResult CvteCarmAdapter::write_command(
    const bridge::core::CommandFrame& command) {
  return impl_->write_command(command);
}

AdapterHealthResult CvteCarmAdapter::query_health() const {
  return impl_->query_health();
}

AdapterCallResult CvteCarmAdapter::shutdown() { return impl_->shutdown(); }

}
