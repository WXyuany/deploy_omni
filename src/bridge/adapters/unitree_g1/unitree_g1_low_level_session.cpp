#include "../../../../include/bridge/adapters/unitree_g1/unitree_g1_low_level_session.hpp"
#include "../../../../include/bridge/adapters/unitree_g1/unitree_g1_low_level_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <utility>

namespace bridge::adapters::unitree_g1 {
namespace {

constexpr std::uint64_t kFallbackTickNs = 1'000'000;
constexpr char kDdsTransportPrefix[] = "dds:";
constexpr int kMaxMotionReleaseAttempts = 3;

[[nodiscard]] bridge::core::AdapterCallResult motion_release_failure(
    const std::string& detail, const std::uint64_t timestamp_ns) {
  return {
      .status = bridge::core::AdapterCallStatus::kRetryableError,
      .detail = detail,
      .fault = bridge::core::BridgeFault{
          .category = bridge::core::FaultCategory::kSafety,
          .severity = bridge::core::FaultSeverity::kRecoverable,
          .lifecycle_state = bridge::core::LifecycleState::kInitializing,
          .message = detail,
          .latched = false,
          .sequence = 0,
          .timestamp_ns = timestamp_ns,
      },
  };
}

[[nodiscard]] std::uint64_t tick_or_default(const std::uint64_t value) noexcept {
  return value == 0 ? kFallbackTickNs : value;
}

[[nodiscard]] bridge::core::BridgeFault make_fault(
    const bridge::core::FaultCategory category,
    const bridge::core::FaultSeverity severity,
    const bridge::core::LifecycleState lifecycle_state, std::string message,
    const std::uint64_t sequence, const std::uint64_t timestamp_ns,
    const bool latched = false) {
  return {
      .category = category,
      .severity = severity,
      .lifecycle_state = lifecycle_state,
      .message = std::move(message),
      .latched = latched,
      .sequence = sequence,
      .timestamp_ns = timestamp_ns,
  };
}

[[nodiscard]] std::vector<double> to_vector(const std::array<float, 3>& values) {
  return {static_cast<double>(values[0]), static_cast<double>(values[1]),
          static_cast<double>(values[2])};
}

[[nodiscard]] std::vector<double> to_vector(const std::array<float, 4>& values) {
  return {static_cast<double>(values[0]), static_cast<double>(values[1]),
          static_cast<double>(values[2]), static_cast<double>(values[3])};
}

[[nodiscard]] bridge::core::ImuSample to_imu_sample(
    const ::unitree_hg::msg::dds_::IMUState_& imu, const std::uint64_t sample_time_ns) {
  return {
      .sample_time_ns = sample_time_ns,
      .valid = true,
      .stale = false,
      .orientation_xyzw = to_vector(imu.quaternion()),
      .rpy_xyz = to_vector(imu.rpy()),
      .angular_velocity_xyz = to_vector(imu.gyroscope()),
      .linear_acceleration_xyz = to_vector(imu.accelerometer()),
  };
}

void print_command_info(const bridge::core::CommandFrame& command,
                        const std::uint8_t mode_machine,
                        const std::size_t hardware_slot_count) {
  const std::size_t width = std::min<std::size_t>(
      hardware_slot_count, command.joints.width());
  std::fprintf(stderr,
               "[UnitreeG1LowLevelSession.write_command] ========== Command =========="
               "\n");
  std::fprintf(stderr,
               "sequence=%llu mode_label=%s mode_pr=%u mode_machine=%u joints=%zu\n",
               static_cast<unsigned long long>(command.sequence),
               command.mode_label.c_str(), command.mode_pr, mode_machine, width);
  std::fprintf(stderr,
               "idx        q        dq       tau        kp        kd\n");
  for (std::size_t index = 0; index < width; ++index) {
    const double q = index < command.joints.position.size()
                         ? command.joints.position[index]
                         : 0.0;
    const double dq = index < command.joints.velocity.size()
                          ? command.joints.velocity[index]
                          : 0.0;
    const double tau = index < command.joints.effort.size()
                           ? command.joints.effort[index]
                           : 0.0;
    const double kp = index < command.joints.stiffness.size()
                          ? command.joints.stiffness[index]
                          : 0.0;
    const double kd = index < command.joints.damping.size()
                          ? command.joints.damping[index]
                          : 0.0;
    std::fprintf(stderr, "%3zu %8.4f %8.4f %8.4f %8.4f %8.4f\n", index,
                 q, dq, tau, kp, kd);
  }
  std::fprintf(stderr,
               "[UnitreeG1LowLevelSession.write_command] =============================="
               "\n");
}

}  // namespace

UnitreeG1LowLevelSession::UnitreeG1LowLevelSession(LowLevelSessionConfig config)
    : config_(std::move(config)),
      position_(config_.hardware_slot_count, 0.0),
      velocity_(config_.hardware_slot_count, 0.0),
      effort_(config_.hardware_slot_count, 0.0),
      acceleration_(config_.hardware_slot_count, 0.0) {}

UnitreeG1LowLevelSession::~UnitreeG1LowLevelSession() {
  stop_reader_ = true;
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
  stop_writer_ = true;
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }
}

const LowLevelSessionConfig& UnitreeG1LowLevelSession::config() const noexcept {
  return config_;
}

const std::optional<unitree_hg::msg::dds_::LowCmd_>&
UnitreeG1LowLevelSession::last_packed_command() const noexcept {
  return last_packed_command_;
}

bridge::core::AdapterCallResult UnitreeG1LowLevelSession::initialize() {
  synthetic_time_ns_ += tick_or_default(config_.initialize_timeout_ns);
  initialized_ = true;
  connection_alive_ = true;
  released_motion_ = !config_.require_motion_release;

  auto check_mode = [this](std::string& form, std::string& name) -> int32_t {
    if (config_.check_mode_hook) {
      return config_.check_mode_hook(form, name);
    }
    if (!motion_switcher_client_) {
      return 0;
    }
    return motion_switcher_client_->CheckMode(form, name);
  };

  auto release_mode = [this]() -> int32_t {
    if (config_.release_mode_hook) {
      return config_.release_mode_hook();
    }
    if (!motion_switcher_client_) {
      return 0;
    }
    return motion_switcher_client_->ReleaseMode();
  };

  if (config_.transport_mode == "unitree_dds") {
    if (config_.require_motion_release) {
      motion_switcher_client_ =
          std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
      motion_switcher_client_->SetTimeout(5.0f);
      motion_switcher_client_->Init();
    }

    lowcmd_publisher_ = std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(
        config_.command_topic);
    lowcmd_publisher_->InitChannel();

    lowstate_subscriber_ =
        std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
            config_.state_topic);
    lowstate_subscriber_->InitChannel(
        [this](const void* message) { handle_low_state_message(message); },
        1);

    if (!config_.imu_topic.empty()) {
      imu_subscriber_ =
          std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::IMUState_>>(
              config_.imu_topic);
      imu_subscriber_->InitChannel(
          [this](const void* message) { handle_imu_message(message); },
          1);
    }
  }

  if (config_.require_motion_release) {
    bool released = false;
    for (int attempt = 0; attempt < kMaxMotionReleaseAttempts; ++attempt) {
      std::string form;
      std::string name;
      check_mode(form, name);
      if (name.empty()) {
        released = true;
        break;
      }

      const int32_t release_result = release_mode();
      if (release_result != 0) {
        initialized_ = false;
        connection_alive_ = false;
        return motion_release_failure(
            "g1 low-level motion release prerequisite not satisfied",
            synthetic_time_ns_);
      }
    }

    if (!released) {
      initialized_ = false;
      connection_alive_ = false;
      return motion_release_failure(
          "g1 low-level motion mode remained active after release attempts",
          synthetic_time_ns_);
    }
    released_motion_ = true;
  }

  stop_reader_ = false;
  reader_thread_ = std::thread(&UnitreeG1LowLevelSession::reader_loop, this);

  return {
      .status = bridge::core::AdapterCallStatus::kOk,
      .detail = config_.require_motion_release
                    ? "g1 low-level session initialized after motion release prerequisite"
                    : "g1 low-level session initialized",
      .fault = std::nullopt,
  };
}

void UnitreeG1LowLevelSession::handle_low_state_message(const void* message) {
  if (message == nullptr) {
    return;
  }
  handle_low_state_message(*static_cast<const unitree_hg::msg::dds_::LowState_*>(message));
}

void UnitreeG1LowLevelSession::handle_imu_message(const void* message) {
  if (message == nullptr) {
    return;
  }
  handle_imu_message(*static_cast<const unitree_hg::msg::dds_::IMUState_*>(message));
}

LowLevelSessionReadResult UnitreeG1LowLevelSession::read_state() {
  LowLevelSessionReadResult result;
  if (!initialized_) {
    result.result = {
        .status = bridge::core::AdapterCallStatus::kRetryableError,
        .detail = "g1 low-level session must be initialized before read",
        .fault = make_fault(bridge::core::FaultCategory::kAdapterRead,
                            bridge::core::FaultSeverity::kRecoverable,
                            bridge::core::LifecycleState::kCreated,
                            "g1 low-level session must be initialized before read",
                            0, synthetic_time_ns_),
    };
    return result;
  }

  std::optional<LowLevelSessionState> state;
  {
    std::lock_guard<std::mutex> lock(state_buffer_mutex_);
    state = std::move(buffered_state_);
    buffered_state_.reset();
  }

  if (!state.has_value() && !latest_low_state_.has_value()) {
    for (std::uint32_t attempt = 0; attempt < config_.read_retry_attempts;
         ++attempt) {
      if (config_.read_retry_interval_ns > 0) {
        std::this_thread::sleep_for(
            std::chrono::nanoseconds(config_.read_retry_interval_ns));
      }
      {
        std::lock_guard<std::mutex> lock(state_buffer_mutex_);
        state = std::move(buffered_state_);
        buffered_state_.reset();
      }
      if (state.has_value() || latest_low_state_.has_value()) {
        break;
      }
    }
  }

  if (!state.has_value()) {
    synthetic_time_ns_ += tick_or_default(config_.read_timeout_ns);
    last_read_time_ns_ = synthetic_time_ns_;

    if (!latest_low_state_.has_value()) {
      result.result = {
          .status = bridge::core::AdapterCallStatus::kRetryableError,
          .detail = "g1 low-level state not yet available after read retries",
          .fault = make_fault(bridge::core::FaultCategory::kAdapterRead,
                              bridge::core::FaultSeverity::kRecoverable,
                              bridge::core::LifecycleState::kWaitingForState,
                              "g1 low-level state not yet available after read retries", 0,
                              synthetic_time_ns_),
      };
      return result;
    }

    LowLevelSessionState fallback;
    fallback.sample_time_ns = synthetic_time_ns_;
    fallback.tick = latest_low_state_->tick();
    fallback.mode_pr = latest_low_state_->mode_pr();
    fallback.mode_machine = latest_low_state_->mode_machine();
    fallback.valid = true;
    fallback.position.resize(config_.hardware_slot_count, 0.0);
    fallback.velocity.resize(config_.hardware_slot_count, 0.0);
    fallback.effort.resize(config_.hardware_slot_count, 0.0);
    fallback.acceleration.resize(config_.hardware_slot_count, 0.0);
    fallback.wireless_remote = latest_low_state_->wireless_remote();

    const std::size_t width = std::min<std::size_t>(
        config_.hardware_slot_count, latest_low_state_->motor_state().size());
    for (std::size_t index = 0; index < width; ++index) {
      const auto& motor = latest_low_state_->motor_state().at(index);
      fallback.position[index] = motor.q();
      fallback.velocity[index] = motor.dq();
      fallback.effort[index] = motor.tau_est();
      fallback.acceleration[index] = motor.ddq();
    }

    if (latest_imu_state_.has_value()) {
      fallback.imu = to_imu_sample(*latest_imu_state_, synthetic_time_ns_);
    } else {
      fallback.imu = to_imu_sample(latest_low_state_->imu_state(), synthetic_time_ns_);
    }

    if (config_.stale_state_after_ns > 0 &&
        synthetic_time_ns_ > last_low_state_time_ns_ + config_.stale_state_after_ns &&
        last_low_state_time_ns_ != 0) {
      fallback.stale = true;
      fallback.degraded = true;
      fallback.fault = make_fault(bridge::core::FaultCategory::kAdapterHealth,
                                  bridge::core::FaultSeverity::kRecoverable,
                                  bridge::core::LifecycleState::kActive,
                                  "g1 low-level state is stale", 0,
                                  synthetic_time_ns_);
    }

    state = std::move(fallback);
  }

  mode_pr_ = state->mode_pr;
  mode_machine_ = state->mode_machine;
  position_ = state->position;
  velocity_ = state->velocity;
  effort_ = state->effort;
  acceleration_ = state->acceleration;

  result.state = std::move(state);
  result.result = result.state->degraded
                      ? bridge::core::AdapterCallResult{
                            .status = bridge::core::AdapterCallStatus::kDegraded,
                            .detail = "g1 low-level state read in degraded mode",
                            .fault = result.state->fault,
                        }
                      : bridge::core::AdapterCallResult{
                            .status = bridge::core::AdapterCallStatus::kOk,
                            .detail = "g1 low-level state read",
                            .fault = std::nullopt,
                        };
  return result;
}

void UnitreeG1LowLevelSession::handle_low_state_message(
    const ::unitree_hg::msg::dds_::LowState_& state) {
  if (config_.verify_low_state_crc && !validate_low_state_crc(state)) {
    return;
  }
  latest_low_state_ = state;
  last_low_state_time_ns_ = synthetic_time_ns_;
  mode_pr_ = state.mode_pr();
  mode_machine_ = state.mode_machine();
}

void UnitreeG1LowLevelSession::handle_imu_message(
    const ::unitree_hg::msg::dds_::IMUState_& imu) {
  latest_imu_state_ = imu;
}

bridge::core::AdapterCallResult UnitreeG1LowLevelSession::write_command(
    const bridge::core::CommandFrame& command) {
  if (!initialized_) {
    return {
        .status = bridge::core::AdapterCallStatus::kRetryableError,
        .detail = "g1 low-level session must be initialized before write",
        .fault = make_fault(bridge::core::FaultCategory::kAdapterWrite,
                            bridge::core::FaultSeverity::kRecoverable,
                            bridge::core::LifecycleState::kCreated,
                            "g1 low-level session must be initialized before write",
                            command.sequence, synthetic_time_ns_),
    };
  }
  // if (!released_motion_) {
  //   return {
  //       .status = bridge::core::AdapterCallStatus::kRetryableError,
  //       .detail = "g1 low-level motion release prerequisite not satisfied",
  //       .fault = make_fault(bridge::core::FaultCategory::kSafety,
  //                           bridge::core::FaultSeverity::kRecoverable,
  //                           bridge::core::LifecycleState::kWaitingForState,
  //                           "g1 low-level motion release prerequisite not satisfied",
  //                           command.sequence, synthetic_time_ns_),
  //   };
  // }

  synthetic_time_ns_ += tick_or_default(config_.write_timeout_ns);
  last_write_time_ns_ = synthetic_time_ns_;
  mode_pr_ = command.mode_pr;
  const std::uint8_t effective_mode_machine =
      latest_low_state_.has_value() ? latest_low_state_->mode_machine() : mode_machine_;
  mode_machine_ = effective_mode_machine;
  const auto now = std::chrono::steady_clock::now();
  if (last_command_print_time_ == std::chrono::steady_clock::time_point{} ||
      now - last_command_print_time_ >= std::chrono::seconds(1)) {
    print_command_info(command, effective_mode_machine, config_.hardware_slot_count);
    last_command_print_time_ = now;
  }
  if (!command.joints.position.empty()) {
    position_ = command.joints.position;
  }
  if (!command.joints.velocity.empty()) {
    velocity_ = command.joints.velocity;
  }
  if (!command.joints.effort.empty()) {
    effort_ = command.joints.effort;
  }
  acceleration_.assign(position_.size(), 0.0);
  last_packed_command_ = build_low_cmd_from_command(command, config_.hardware_slot_count,
                                                    effective_mode_machine);

  if (lowcmd_publisher_ && last_packed_command_.has_value()) {
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      pending_cmd_ = *last_packed_command_;
    }
    if (!writer_started_.exchange(true)) {
      stop_writer_ = false;
      writer_thread_ = std::thread(&UnitreeG1LowLevelSession::writer_loop, this);
    }
  }

  return {
      .status = bridge::core::AdapterCallStatus::kOk,
      .detail = "g1 low-level command buffered for writer thread",
      .fault = std::nullopt,
  };
}

bridge::core::AdapterCallResult UnitreeG1LowLevelSession::write_safe_command() {
  if (!initialized_) {
    return {
        .status = bridge::core::AdapterCallStatus::kRetryableError,
        .detail = "g1 low-level session must be initialized before safe command",
        .fault = make_fault(bridge::core::FaultCategory::kSafety,
                            bridge::core::FaultSeverity::kRecoverable,
                            bridge::core::LifecycleState::kCreated,
                            "g1 low-level session must be initialized before safe command",
                            0, synthetic_time_ns_),
    };
  }

  synthetic_time_ns_ += tick_or_default(config_.write_timeout_ns);
  last_write_time_ns_ = synthetic_time_ns_;
  mode_pr_ = 0;
  effort_.assign(effort_.size(), 0.0);
  velocity_.assign(velocity_.size(), 0.0);
  acceleration_.assign(acceleration_.size(), 0.0);
  last_packed_command_ = build_safe_low_cmd(config_.hardware_slot_count, mode_machine_);

  if (lowcmd_publisher_ && last_packed_command_.has_value()) {
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      pending_cmd_ = *last_packed_command_;
    }
    if (!writer_started_.exchange(true)) {
      stop_writer_ = false;
      writer_thread_ = std::thread(&UnitreeG1LowLevelSession::writer_loop, this);
    }
  }

  return {
      .status = bridge::core::AdapterCallStatus::kOk,
      .detail = "g1 low-level safe command buffered for writer thread",
      .fault = std::nullopt,
  };
}

void UnitreeG1LowLevelSession::writer_loop() {
  while (!stop_writer_) {
    std::optional<unitree_hg::msg::dds_::LowCmd_> cmd;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      cmd = pending_cmd_;
    }
    if (cmd.has_value() && lowcmd_publisher_) {
      connection_alive_ = lowcmd_publisher_->Write(*cmd);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(2000));
  }
}

void UnitreeG1LowLevelSession::reader_loop() {
  while (!stop_reader_) {
    if (!latest_low_state_.has_value()) {
      std::this_thread::sleep_for(std::chrono::microseconds(1000));
      continue;
    }

    LowLevelSessionState state;
    state.sample_time_ns = synthetic_time_ns_;
    state.tick = latest_low_state_->tick();
    state.mode_pr = latest_low_state_->mode_pr();
    state.mode_machine = latest_low_state_->mode_machine();
    state.valid = true;
    state.position.resize(config_.hardware_slot_count, 0.0);
    state.velocity.resize(config_.hardware_slot_count, 0.0);
    state.effort.resize(config_.hardware_slot_count, 0.0);
    state.acceleration.resize(config_.hardware_slot_count, 0.0);
    state.wireless_remote = latest_low_state_->wireless_remote();

    const std::size_t width = std::min<std::size_t>(
        config_.hardware_slot_count, latest_low_state_->motor_state().size());
    for (std::size_t index = 0; index < width; ++index) {
      const auto& motor = latest_low_state_->motor_state().at(index);
      state.position[index] = motor.q();
      state.velocity[index] = motor.dq();
      state.effort[index] = motor.tau_est();
      state.acceleration[index] = motor.ddq();
    }

    if (latest_imu_state_.has_value()) {
      state.imu = to_imu_sample(*latest_imu_state_, synthetic_time_ns_);
    } else {
      state.imu = to_imu_sample(latest_low_state_->imu_state(), synthetic_time_ns_);
    }

    if (config_.stale_state_after_ns > 0 &&
        synthetic_time_ns_ > last_low_state_time_ns_ + config_.stale_state_after_ns &&
        last_low_state_time_ns_ != 0) {
      state.stale = true;
      state.degraded = true;
      state.fault = make_fault(bridge::core::FaultCategory::kAdapterHealth,
                               bridge::core::FaultSeverity::kRecoverable,
                               bridge::core::LifecycleState::kActive,
                               "g1 low-level state is stale", 0,
                               synthetic_time_ns_);
    }

    {
      std::lock_guard<std::mutex> lock(state_buffer_mutex_);
      buffered_state_ = std::move(state);
    }
    has_state_ = true;

    std::this_thread::sleep_for(std::chrono::microseconds(2000));
  }
}

bridge::core::AdapterHealth UnitreeG1LowLevelSession::health() const {
  const bool stale = config_.stale_state_after_ns > 0 && last_low_state_time_ns_ != 0 &&
                     synthetic_time_ns_ > last_low_state_time_ns_ + config_.stale_state_after_ns;
  return {
      .state = !connection_alive_ ? bridge::core::HealthState::kUnknown
                                  : stale ? bridge::core::HealthState::kDegraded
                                          : bridge::core::HealthState::kHealthy,
      .safe_command_supported = true,
      .connection_alive = connection_alive_,
      .degraded = stale,
      .last_read_time_ns = last_read_time_ns_,
      .last_write_time_ns = last_write_time_ns_,
      .fault = stale
                   ? std::optional<bridge::core::BridgeFault>(make_fault(
                         bridge::core::FaultCategory::kAdapterHealth,
                         bridge::core::FaultSeverity::kRecoverable,
                         bridge::core::LifecycleState::kActive,
                         "g1 low-level state is stale", 0, synthetic_time_ns_))
                   : std::nullopt,
  };
}

bridge::core::AdapterCallResult UnitreeG1LowLevelSession::shutdown() {
  stop_writer_ = true;
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }
  writer_started_ = false;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    pending_cmd_.reset();
  }

  stop_reader_ = true;
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
  {
    std::lock_guard<std::mutex> lock(state_buffer_mutex_);
    buffered_state_.reset();
  }
  initialized_ = false;
  connection_alive_ = false;
  has_state_ = false;
  last_packed_command_.reset();
  latest_low_state_.reset();
  latest_imu_state_.reset();
  last_low_state_time_ns_ = 0;
  if (imu_subscriber_) {
    imu_subscriber_->CloseChannel();
    imu_subscriber_.reset();
  }
  if (lowstate_subscriber_) {
    lowstate_subscriber_->CloseChannel();
    lowstate_subscriber_.reset();
  }
  if (lowcmd_publisher_) {
    lowcmd_publisher_->CloseChannel();
    lowcmd_publisher_.reset();
  }
  return {
      .status = bridge::core::AdapterCallStatus::kOk,
      .detail = "g1 low-level session shutdown complete",
      .fault = std::nullopt,
  };
}

}  // namespace bridge::adapters::unitree_g1
