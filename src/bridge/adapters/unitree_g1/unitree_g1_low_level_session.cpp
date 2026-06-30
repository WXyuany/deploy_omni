#include "../../../../include/bridge/adapters/unitree_g1/unitree_g1_low_level_session.hpp"
#include "../../../../include/bridge/adapters/unitree_g1/unitree_g1_low_level_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <sstream>
#include <utility>

#include "../../../../include/bridge/observability/forwarding_logger.hpp"

namespace bridge::adapters::unitree_g1 {
namespace {

constexpr std::uint64_t kFallbackTickNs = 1'000'000;
constexpr char kDdsTransportPrefix[] = "dds:";
constexpr int kMaxMotionReleaseAttempts = 3;
constexpr auto kLowStateReceiveTimeout = std::chrono::milliseconds(500);

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

[[nodiscard]] std::uint64_t monotonic_now_ns() noexcept {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

[[nodiscard]] std::string format_one_decimal(const double value) {
  std::ostringstream output;
  output.setf(std::ios::fixed);
  output.precision(1);
  output << value;
  return output.str();
}

[[nodiscard]] std::string format_two_decimal(const double value) {
  std::ostringstream output;
  output.setf(std::ios::fixed);
  output.precision(2);
  output << value;
  return output.str();
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

[[nodiscard]] bool low_state_receive_timed_out(
    const std::chrono::steady_clock::time_point receive_time) noexcept {
  return receive_time != std::chrono::steady_clock::time_point{} &&
         std::chrono::steady_clock::now() - receive_time > kLowStateReceiveTimeout;
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
      .orientation_xyzw = {static_cast<double>(imu.quaternion()[1]),
                           static_cast<double>(imu.quaternion()[2]),
                           static_cast<double>(imu.quaternion()[3]),
                           static_cast<double>(imu.quaternion()[0])},
      .rpy_xyz = to_vector(imu.rpy()),
      .angular_velocity_xyz = to_vector(imu.gyroscope()),
      .linear_acceleration_xyz = to_vector(imu.accelerometer()),
      .temperature = imu.temperature(),
  };
}

[[nodiscard]] LowLevelSessionState make_session_state_from_low_state(
    const ::unitree_hg::msg::dds_::LowState_& low_state,
    const std::optional<::unitree_hg::msg::dds_::IMUState_>& latest_imu_state,
    const std::size_t hardware_slot_count,
    const std::uint64_t sample_time_ns) {
  LowLevelSessionState state;
  state.sample_time_ns = sample_time_ns;
  state.low_state_version = low_state.version();
  state.low_state_reserve = low_state.reserve();
  state.tick = low_state.tick();
  state.mode_pr = low_state.mode_pr();
  state.mode_machine = low_state.mode_machine();
  state.valid = true;
  state.position.resize(hardware_slot_count, 0.0);
  state.velocity.resize(hardware_slot_count, 0.0);
  state.effort.resize(hardware_slot_count, 0.0);
  state.acceleration.resize(hardware_slot_count, 0.0);
  state.motor_mode.resize(hardware_slot_count, 0U);
  state.motor_temperature.resize(hardware_slot_count, {0, 0});
  state.motor_voltage.resize(hardware_slot_count, 0.0);
  state.motor_sensor.resize(hardware_slot_count, {0U, 0U});
  state.motor_state.resize(hardware_slot_count, 0U);
  state.motor_reserve.resize(hardware_slot_count, {0U, 0U, 0U, 0U});
  state.wireless_remote = low_state.wireless_remote();

  const std::size_t width =
      std::min<std::size_t>(hardware_slot_count, low_state.motor_state().size());
  for (std::size_t index = 0; index < width; ++index) {
    const auto& motor = low_state.motor_state().at(index);
    state.position[index] = motor.q();
    state.velocity[index] = motor.dq();
    state.effort[index] = motor.tau_est();
    state.acceleration[index] = motor.ddq();
    state.motor_mode[index] = motor.mode();
    state.motor_temperature[index] = motor.temperature();
    state.motor_voltage[index] = motor.vol();
    state.motor_sensor[index] = motor.sensor();
    state.motor_state[index] = motor.motorstate();
    state.motor_reserve[index] = motor.reserve();
  }

  state.imu = latest_imu_state.has_value()
                  ? to_imu_sample(*latest_imu_state, sample_time_ns)
                  : to_imu_sample(low_state.imu_state(), sample_time_ns);
  return state;
}

}  // namespace

UnitreeG1LowLevelSession::UnitreeG1LowLevelSession(LowLevelSessionConfig config)
    : config_(std::move(config)) {}

std::uint64_t select_reader_loop_sleep_ns(
    const std::uint64_t configured_read_timeout_ns) noexcept {
  return configured_read_timeout_ns == 0 ? kFallbackTickNs : configured_read_timeout_ns;
}

UnitreeG1LowLevelSession::~UnitreeG1LowLevelSession() {
  static_cast<void>(shutdown());
}

const LowLevelSessionConfig& UnitreeG1LowLevelSession::config() const noexcept {
  return config_;
}

void UnitreeG1LowLevelSession::set_state_available_callback(
    std::function<void()> callback) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  config_.state_available_callback = std::move(callback);
}

const std::optional<unitree_hg::msg::dds_::LowCmd_>&
UnitreeG1LowLevelSession::last_packed_command() const noexcept {
  return last_packed_command_;
}

bridge::core::AdapterCallResult UnitreeG1LowLevelSession::initialize() {
  state_time_ns_ += tick_or_default(config_.initialize_timeout_ns);
  command_time_ns_ = state_time_ns_;
  initialized_.store(true);
  connection_alive_ = true;
  released_motion_.store(!config_.require_motion_release);

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
        initialized_.store(false);
        connection_alive_ = false;
        return motion_release_failure(
            "g1 low-level motion release prerequisite not satisfied",
            state_time_ns_);
      }
    }

    if (!released) {
      initialized_.store(false);
      connection_alive_ = false;
      return motion_release_failure(
          "g1 low-level motion mode remained active after release attempts",
          state_time_ns_);
    }
    released_motion_.store(true);
  }

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
  handle_low_state_message(*static_cast<const unitree_hg::msg::dds_::LowState_*>(message), true);
}

void UnitreeG1LowLevelSession::handle_imu_message(const void* message) {
  if (message == nullptr) {
    return;
  }
  handle_imu_message(*static_cast<const unitree_hg::msg::dds_::IMUState_*>(message));
}

LowLevelSessionReadResult UnitreeG1LowLevelSession::read_state() {
  const std::uint64_t read_start_ns = monotonic_now_ns();
  LowLevelSessionReadResult result;
  if (!initialized_.load()) {
    result.result = {
        .status = bridge::core::AdapterCallStatus::kRetryableError,
        .detail = "g1 low-level session must be initialized before read",
        .fault = make_fault(bridge::core::FaultCategory::kAdapterRead,
                            bridge::core::FaultSeverity::kRecoverable,
                            bridge::core::LifecycleState::kCreated,
                            "g1 low-level session must be initialized before read",
                            0, state_time_ns_),
    };
    return result;
  }
  drain_pending_lowstate_receive_log();

  struct LowStateSnapshot {
    std::optional<unitree_hg::msg::dds_::LowState_> low_state;
    std::optional<unitree_hg::msg::dds_::IMUState_> imu_state;
    std::chrono::steady_clock::time_point receive_time;
    std::uint64_t receive_time_ns{0};
    std::uint64_t receive_generation{0};
    std::uint64_t state_time_ns{0};
    std::uint64_t consumed_generation{0};
    std::uint64_t callback_store_duration_ns{0};
  };
  std::uint64_t last_snapshot_duration_ns = 0;
  auto low_state_snapshot = [this, &last_snapshot_duration_ns]() {
    const std::uint64_t snapshot_start_ns = monotonic_now_ns();
    std::lock_guard<std::mutex> lock(state_mutex_);
    LowStateSnapshot snapshot{
        .low_state = latest_low_state_,
        .imu_state = latest_imu_state_,
        .receive_time = last_low_state_receive_time_,
        .receive_time_ns = last_low_state_receive_time_ns_,
        .receive_generation = last_low_state_generation_,
        .state_time_ns = last_low_state_time_ns_,
        .consumed_generation = last_consumed_low_state_generation_,
        .callback_store_duration_ns = last_low_state_callback_store_duration_ns_,
    };
    last_snapshot_duration_ns = monotonic_now_ns() - snapshot_start_ns;
    return snapshot;
  };
  auto snapshot_has_unconsumed_low_state = [](const LowStateSnapshot& snapshot) {
    return snapshot.low_state.has_value() && snapshot.receive_generation != 0 &&
           snapshot.receive_generation != snapshot.consumed_generation;
  };
  auto mark_consumed_low_state = [this](const std::uint64_t receive_generation) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_consumed_low_state_generation_ = receive_generation;
  };
  LowStateSnapshot snapshot = low_state_snapshot();

  if (!snapshot.low_state.has_value()) {
    for (std::uint32_t attempt = 0; attempt < config_.read_retry_attempts;
         ++attempt) {
      if (config_.read_retry_interval_ns > 0) {
        std::this_thread::sleep_for(
            std::chrono::nanoseconds(config_.read_retry_interval_ns));
      }
      snapshot = low_state_snapshot();
      if (snapshot_has_unconsumed_low_state(snapshot)) {
        break;
      }
    }
  }

  state_time_ns_ += tick_or_default(config_.read_timeout_ns);
  last_read_time_ns_ = state_time_ns_;

  snapshot = low_state_snapshot();
  if (!snapshot.low_state.has_value()) {
    const std::string detail = config_.state_topic.empty()
                                   ? "waiting for g1 low-level state"
                                   : "waiting for " + config_.state_topic;
    result.result = {
        .status = bridge::core::AdapterCallStatus::kRetryableError,
        .detail = detail,
        .fault = make_fault(bridge::core::FaultCategory::kAdapterRead,
                            bridge::core::FaultSeverity::kRecoverable,
                            bridge::core::LifecycleState::kWaitingForState,
                            detail, 0, state_time_ns_),
    };
    return result;
  }

  if (low_state_receive_timed_out(snapshot.receive_time)) {
    connection_alive_ = false;
    result.result = {
        .status = bridge::core::AdapterCallStatus::kRetryableError,
        .detail = "g1 low-level lowstate receive timeout",
        .fault = make_fault(bridge::core::FaultCategory::kAdapterRead,
                            bridge::core::FaultSeverity::kRecoverable,
                            bridge::core::LifecycleState::kActive,
                            "g1 low-level lowstate receive timeout", 0,
                            state_time_ns_),
    };
    return result;
  }

  if (!snapshot_has_unconsumed_low_state(snapshot)) {
    result.result = {
        .status = bridge::core::AdapterCallStatus::kRetryableError,
        .detail = "g1 low-level state not yet available after read retries",
        .fault = make_fault(bridge::core::FaultCategory::kAdapterRead,
                            bridge::core::FaultSeverity::kRecoverable,
                            bridge::core::LifecycleState::kWaitingForState,
                            "g1 low-level state not yet available after read retries", 0,
                            state_time_ns_),
    };
    return result;
  }

  const std::uint64_t state_build_start_ns = monotonic_now_ns();
  LowLevelSessionState state = make_session_state_from_low_state(
      *snapshot.low_state, snapshot.imu_state, config_.hardware_slot_count,
      state_time_ns_);
  const std::uint64_t state_build_end_ns = monotonic_now_ns();
  state.receive_time_ns = snapshot.receive_time_ns;
  state.receive_generation = snapshot.receive_generation;
  state.callback_store_duration_ns = snapshot.callback_store_duration_ns;
  state.read_start_time_ns = read_start_ns;
  state.read_complete_time_ns = state_build_end_ns;
  state.read_snapshot_duration_ns = last_snapshot_duration_ns;
  state.stateframe_build_duration_ns = state_build_end_ns - state_build_start_ns;
  state.callback_drops = dropped_lowstate_callback_updates_.load(std::memory_order_relaxed);

  if (config_.stale_state_after_ns > 0 &&
      state_time_ns_ > snapshot.state_time_ns + config_.stale_state_after_ns &&
      snapshot.state_time_ns != 0) {
    state.stale = true;
    state.degraded = true;
    state.fault = make_fault(bridge::core::FaultCategory::kAdapterHealth,
                                bridge::core::FaultSeverity::kRecoverable,
                                bridge::core::LifecycleState::kActive,
                                "g1 low-level state is stale", 0,
                                state_time_ns_);
  }

  mode_pr_.store(state.mode_pr);
  mode_machine_.store(state.mode_machine);
  mark_consumed_low_state(state.receive_generation);
  if (const auto stats = state_read_stats_.record(monotonic_now_ns(), state.receive_time_ns);
      config_.forwarding_log_enabled &&
      stats.has_value()) {
    bridge::observability::forwarding_logger()->info(
        "[g1_state_read] read_hz=" + format_two_decimal(stats->frequency_hz) +
        " robot_rx_to_read_avg_us=" + format_one_decimal(stats->avg_latency_us) +
        " robot_rx_to_read_max_us=" + format_one_decimal(stats->max_latency_us) +
        " interval_avg_us=" + format_one_decimal(stats->avg_interval_us));
  }

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
  handle_low_state_message(state, false);
}

void UnitreeG1LowLevelSession::handle_low_state_message(
    const ::unitree_hg::msg::dds_::LowState_& state,
    const bool drop_if_state_lock_is_busy) {
  if (config_.verify_low_state_crc && !validate_low_state_crc(state)) {
    return;
  }
  const auto receive_time = std::chrono::steady_clock::now();
  const std::uint64_t receive_time_ns = monotonic_now_ns();
  const std::uint64_t callback_store_start_ns = monotonic_now_ns();
  if (const auto stats = lowstate_receive_stats_.record(receive_time_ns); stats.has_value()) {
    std::unique_lock<std::mutex> telemetry_lock(telemetry_mutex_, std::try_to_lock);
    if (telemetry_lock.owns_lock()) {
      pending_lowstate_receive_stats_ = *stats;
    }
  }

  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    if (!drop_if_state_lock_is_busy) {
      lock.lock();
    }
  }
  if (!lock.owns_lock()) {
    dropped_lowstate_callback_updates_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  std::function<void()> state_available_callback;
  latest_low_state_ = state;
  last_low_state_time_ns_ = state_time_ns_;
  last_low_state_receive_time_ = receive_time;
  last_low_state_receive_time_ns_ = receive_time_ns;
  last_low_state_callback_store_duration_ns_ = monotonic_now_ns() - callback_store_start_ns;
  ++last_low_state_generation_;
  state_available_callback = config_.state_available_callback;
  lock.unlock();
  connection_alive_ = true;
  mode_pr_.store(state.mode_pr());
  mode_machine_.store(state.mode_machine());
  if (state_available_callback) {
    state_available_callback();
  }
}

void UnitreeG1LowLevelSession::handle_imu_message(
    const ::unitree_hg::msg::dds_::IMUState_& imu) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_imu_state_ = imu;
}

void UnitreeG1LowLevelSession::drain_pending_lowstate_receive_log() {
  if (!config_.forwarding_log_enabled) {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    pending_lowstate_receive_stats_.reset();
    return;
  }
  std::optional<bridge::observability::ForwardingStatsSnapshot> stats;
  const std::uint64_t dropped_updates =
      dropped_lowstate_callback_updates_.load(std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    stats = pending_lowstate_receive_stats_;
    pending_lowstate_receive_stats_.reset();
  }
  if (stats.has_value()) {
    std::string line =
        "[g1_state_rx] robot_lowstate_hz=" + format_two_decimal(stats->frequency_hz) +
        " interval_avg_us=" + format_one_decimal(stats->avg_interval_us) +
        " interval_min_us=" + format_one_decimal(stats->min_interval_us) +
        " interval_max_us=" + format_one_decimal(stats->max_interval_us);
    if (dropped_updates != last_logged_lowstate_callback_drops_) {
      line += " callback_drops=" + std::to_string(dropped_updates);
      last_logged_lowstate_callback_drops_ = dropped_updates;
    }
    bridge::observability::forwarding_logger()->info(line);
  }
}

bridge::core::AdapterCallResult UnitreeG1LowLevelSession::write_command(
    const bridge::core::CommandFrame& command) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!initialized_.load()) {
    return {
        .status = bridge::core::AdapterCallStatus::kRetryableError,
        .detail = "g1 low-level session must be initialized before write",
        .fault = make_fault(bridge::core::FaultCategory::kAdapterWrite,
                            bridge::core::FaultSeverity::kRecoverable,
                            bridge::core::LifecycleState::kCreated,
                            "g1 low-level session must be initialized before write",
                            command.sequence, command_time_ns_),
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

  command_time_ns_ += tick_or_default(config_.write_timeout_ns);
  last_write_time_ns_ = command_time_ns_;
  mode_pr_.store(command.mode_pr);
  const std::uint8_t effective_mode_machine = command.mode_machine;
  mode_machine_.store(effective_mode_machine);
  last_packed_command_ = build_low_cmd_from_command(command, config_.hardware_slot_count,
                                                    effective_mode_machine);

  const std::uint64_t write_begin_ns = monotonic_now_ns();
  if (lowcmd_publisher_ && last_packed_command_.has_value()) {
    connection_alive_ = lowcmd_publisher_->Write(*last_packed_command_);
  }
  const std::uint64_t write_end_ns = monotonic_now_ns();
  const std::uint64_t source_time_ns =
      command.receive_time_ns == 0 ? write_begin_ns : command.receive_time_ns;
  if (const auto stats = command_write_stats_.record(write_end_ns, source_time_ns);
      config_.forwarding_log_enabled &&
      stats.has_value()) {
    bridge::observability::forwarding_logger()->info(
        "[g1_cmd_forward] write_hz=" + format_two_decimal(stats->frequency_hz) +
        " ingress_to_write_avg_us=" + format_one_decimal(stats->avg_latency_us) +
        " ingress_to_write_max_us=" + format_one_decimal(stats->max_latency_us) +
        " interval_avg_us=" + format_one_decimal(stats->avg_interval_us) +
        " last_dds_write_us=" +
        format_one_decimal(static_cast<double>(write_end_ns - write_begin_ns) / 1000.0) +
        " connection_alive=" + (connection_alive_.load() ? "true" : "false"));
  }

  return {
      .status = bridge::core::AdapterCallStatus::kOk,
      .detail = "g1 low-level command written",
      .fault = std::nullopt,
  };
}

bridge::core::AdapterHealth UnitreeG1LowLevelSession::health() const {
  std::uint64_t state_time_ns = 0;
  std::uint64_t last_low_state_time_ns = 0;
  std::uint64_t last_read_time_ns = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_time_ns = state_time_ns_;
    last_low_state_time_ns = last_low_state_time_ns_;
    last_read_time_ns = last_read_time_ns_;
  }
  std::uint64_t last_write_time_ns = 0;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    last_write_time_ns = last_write_time_ns_;
  }
  const bool stale = config_.stale_state_after_ns > 0 && last_low_state_time_ns != 0 &&
                     state_time_ns > last_low_state_time_ns + config_.stale_state_after_ns;
  return {
      .state = !connection_alive_ ? bridge::core::HealthState::kUnknown
                                  : stale ? bridge::core::HealthState::kDegraded
                                          : bridge::core::HealthState::kHealthy,
      .connection_alive = connection_alive_,
      .degraded = stale,
      .last_read_time_ns = last_read_time_ns,
      .last_write_time_ns = last_write_time_ns,
      .fault = stale
                   ? std::optional<bridge::core::BridgeFault>(make_fault(
                         bridge::core::FaultCategory::kAdapterHealth,
                         bridge::core::FaultSeverity::kRecoverable,
                         bridge::core::LifecycleState::kActive,
                          "g1 low-level state is stale", 0, state_time_ns))
                   : std::nullopt,
  };
}

bridge::core::AdapterCallResult UnitreeG1LowLevelSession::shutdown() {
  initialized_.store(false);
  connection_alive_ = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    last_packed_command_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_low_state_.reset();
    latest_imu_state_.reset();
    last_low_state_time_ns_ = 0;
    last_low_state_receive_time_ = std::chrono::steady_clock::time_point{};
  }
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
