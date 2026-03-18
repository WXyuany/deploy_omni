#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "../../include/bridge/core/bridge_runner.hpp"
#include "../../include/bridge/dds/transport.hpp"

namespace {

using bridge::core::AdapterCallResult;
using bridge::core::AdapterCallStatus;
using bridge::core::AdapterHealth;
using bridge::core::AdapterHealthResult;
using bridge::core::AdapterReadResult;
using bridge::core::BridgeCommandIngress;
using bridge::core::BridgeCommandSink;
using bridge::core::BridgeFault;
using bridge::core::BridgeRunner;
using bridge::core::BridgeRunnerConfig;
using bridge::core::BridgeRuntimeEventSink;
using bridge::core::BridgeStateEgress;
using bridge::core::CommandFrame;
using bridge::core::CommandSubmissionResult;
using bridge::core::ControlMode;
using bridge::core::FaultCategory;
using bridge::core::FaultSeverity;
using bridge::core::HealthState;
using bridge::core::LifecycleState;
using bridge::core::RobotSdkAdapter;
using bridge::core::RunnerCallResult;
using bridge::core::RunnerCallStatus;
using bridge::core::StartupIdlePolicy;
using bridge::core::StateFrame;
using bridge::dds::CommandDecodeResult;
using bridge::dds::CommandDecodeStatus;
using bridge::dds::DdsCommandDecoder;
using bridge::dds::DdsCommandReader;
using bridge::dds::DdsCommandSample;
using bridge::dds::DdsCommandSampleSink;
using bridge::dds::DdsPublication;
using bridge::dds::DdsPublicationKind;
using bridge::dds::DdsPublicationWriter;
using bridge::dds::DdsStateEncoder;
using bridge::dds::StateEncodeResult;
using bridge::dds::TransportCallResult;
using bridge::dds::TransportCallStatus;
using bridge::dds::make_lowcmd_codec;
using bridge::dds::make_lowstate_codec;

constexpr double kTolerance = 1e-9;

constexpr std::uint32_t kCmdFieldPosition = 1U << 0U;
constexpr std::uint32_t kCmdFieldVelocity = 1U << 1U;
constexpr std::uint32_t kCmdFieldEffort = 1U << 2U;
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
constexpr std::uint32_t kStateJointFieldPosition = 1U << 0U;
constexpr std::uint32_t kStateJointFieldVelocity = 1U << 1U;
constexpr std::uint32_t kStateJointFieldEffort = 1U << 2U;
constexpr std::uint32_t kStateJointFieldAcceleration = 1U << 3U;
constexpr std::uint8_t kImuFieldOrientation = 1U << 0U;
constexpr std::uint8_t kImuFieldAngularVelocity = 1U << 1U;
constexpr std::uint8_t kImuFieldLinearAcceleration = 1U << 2U;
constexpr std::uint8_t kImuFlagPresent = 1U << 0U;
constexpr std::uint8_t kImuFlagValid = 1U << 1U;

void AppendU8(std::vector<std::byte>* payload, const std::uint8_t value) {
  payload->push_back(static_cast<std::byte>(value));
}

void AppendU32Le(std::vector<std::byte>* payload, std::uint32_t value) {
  for (std::size_t index = 0; index < sizeof(value); ++index) {
    AppendU8(payload, static_cast<std::uint8_t>(value & 0xFFU));
    value >>= 8U;
  }
}

void AppendU64Le(std::vector<std::byte>* payload, std::uint64_t value) {
  for (std::size_t index = 0; index < sizeof(value); ++index) {
    AppendU8(payload, static_cast<std::uint8_t>(value & 0xFFU));
    value >>= 8U;
  }
}

void AppendDoubleLe(std::vector<std::byte>* payload, const double value) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(value));
  AppendU64Le(payload, bits);
}

std::uint8_t ReadU8(const std::vector<std::byte>& payload,
                   const std::size_t offset) {
  return std::to_integer<std::uint8_t>(payload[offset]);
}

std::uint32_t ReadU32Le(const std::vector<std::byte>& payload,
                        const std::size_t offset) {
  return static_cast<std::uint32_t>(ReadU8(payload, offset + 0U)) |
         (static_cast<std::uint32_t>(ReadU8(payload, offset + 1U)) << 8U) |
         (static_cast<std::uint32_t>(ReadU8(payload, offset + 2U)) << 16U) |
         (static_cast<std::uint32_t>(ReadU8(payload, offset + 3U)) << 24U);
}

std::uint64_t ReadU64Le(const std::vector<std::byte>& payload,
                        const std::size_t offset) {
  std::uint64_t value = 0U;
  for (std::size_t index = 0; index < sizeof(std::uint64_t); ++index) {
    value |= static_cast<std::uint64_t>(ReadU8(payload, offset + index))
             << (index * 8U);
  }
  return value;
}

double ReadDoubleLe(const std::vector<std::byte>& payload,
                    const std::size_t offset) {
  const std::uint64_t bits = ReadU64Le(payload, offset);
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::string ReadString(const std::vector<std::byte>& payload, std::size_t* offset) {
  const std::uint32_t size = ReadU32Le(payload, *offset);
  *offset += sizeof(std::uint32_t);
  std::string value;
  value.reserve(size);
  for (std::uint32_t index = 0; index < size; ++index) {
    value.push_back(static_cast<char>(payload[*offset + index]));
  }
  *offset += size;
  return value;
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

std::uint8_t EncodeLifecycleState(const LifecycleState state) {
  switch (state) {
    case LifecycleState::kCreated:
      return 1U;
    case LifecycleState::kInitializing:
      return 2U;
    case LifecycleState::kWaitingForState:
      return 3U;
    case LifecycleState::kActive:
      return 4U;
    case LifecycleState::kFaulted:
      return 5U;
    case LifecycleState::kStopping:
      return 6U;
    case LifecycleState::kStopped:
      return 7U;
  }

  return 0U;
}

std::uint8_t EncodeControlMode(const ControlMode mode) {
  switch (mode) {
    case ControlMode::kUnknown:
      return 0U;
    case ControlMode::kSafeIdle:
      return 1U;
    case ControlMode::kTorque:
      return 2U;
    case ControlMode::kVelocity:
      return 3U;
    case ControlMode::kPosition:
      return 4U;
    case ControlMode::kHybrid:
      return 5U;
  }

  return 0U;
}

struct ParsedStatePublication {
  std::uint8_t version{0};
  std::uint8_t state_flags{0};
  std::uint8_t imu_flags{0};
  std::uint8_t imu_field_mask{0};
  std::uint32_t joint_count{0};
  std::uint32_t joint_field_mask{0};
  std::uint64_t sample_time_ns{0};
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<double> acceleration;
  std::uint64_t imu_sample_time_ns{0};
  std::vector<double> orientation_xyzw;
  std::vector<double> angular_velocity_xyz;
  std::vector<double> linear_acceleration_xyz;
};

struct ParsedStatusPublication {
  std::uint8_t version{0};
  std::uint8_t lifecycle_state{0};
  std::uint8_t control_mode{0};
  std::uint8_t status_flags{0};
  std::uint64_t sample_time_ns{0};
  std::uint64_t publish_time_ns{0};
  std::uint64_t fault_timestamp_ns{0};
  std::string mode_label;
  std::uint8_t fault_category{0};
  std::uint8_t fault_severity{0};
  std::uint8_t fault_lifecycle_state{0};
  std::uint64_t fault_sequence{0};
  std::string fault_message;
};

ParsedStatePublication ParseStatePublication(const DdsPublication& publication) {
  assert(publication.payload.size() >= 24U);
  const std::vector<std::byte>& payload = publication.payload;
  assert(payload[0] == std::byte{'L'});
  assert(payload[1] == std::byte{'S'});
  assert(payload[2] == std::byte{'T'});
  assert(payload[3] == std::byte{'A'});

  ParsedStatePublication parsed;
  parsed.version = ReadU8(payload, 4U);
  parsed.state_flags = ReadU8(payload, 5U);
  parsed.imu_flags = ReadU8(payload, 6U);
  parsed.imu_field_mask = ReadU8(payload, 7U);
  parsed.joint_count = ReadU32Le(payload, 8U);
  parsed.joint_field_mask = ReadU32Le(payload, 12U);
  parsed.sample_time_ns = ReadU64Le(payload, 16U);

  std::size_t offset = 24U;
  const auto read_block = [&](const bool present, std::vector<double>* out) {
    if (!present) {
      return;
    }
    out->reserve(parsed.joint_count);
    for (std::uint32_t index = 0; index < parsed.joint_count; ++index) {
      out->push_back(ReadDoubleLe(payload, offset));
      offset += sizeof(double);
    }
  };

  read_block((parsed.joint_field_mask & kStateJointFieldPosition) != 0U,
             &parsed.position);
  read_block((parsed.joint_field_mask & kStateJointFieldVelocity) != 0U,
             &parsed.velocity);
  read_block((parsed.joint_field_mask & kStateJointFieldEffort) != 0U,
             &parsed.effort);
  read_block((parsed.joint_field_mask & kStateJointFieldAcceleration) != 0U,
             &parsed.acceleration);

  if ((parsed.imu_flags & kImuFlagPresent) != 0U) {
    parsed.imu_sample_time_ns = ReadU64Le(payload, offset);
    offset += sizeof(std::uint64_t);

    const auto read_fixed = [&](const bool present, const std::size_t width,
                                std::vector<double>* out) {
      if (!present) {
        return;
      }
      out->reserve(width);
      for (std::size_t index = 0; index < width; ++index) {
        out->push_back(ReadDoubleLe(payload, offset));
        offset += sizeof(double);
      }
    };

    read_fixed((parsed.imu_field_mask & kImuFieldOrientation) != 0U, 4U,
               &parsed.orientation_xyzw);
    read_fixed((parsed.imu_field_mask & kImuFieldAngularVelocity) != 0U, 3U,
               &parsed.angular_velocity_xyz);
    read_fixed((parsed.imu_field_mask & kImuFieldLinearAcceleration) != 0U, 3U,
               &parsed.linear_acceleration_xyz);
  }

  assert(offset == payload.size());
  return parsed;
}

ParsedStatusPublication ParseStatusPublication(const DdsPublication& publication) {
  assert(publication.payload.size() >= 32U);
  const std::vector<std::byte>& payload = publication.payload;
  assert(payload[0] == std::byte{'L'});
  assert(payload[1] == std::byte{'S'});
  assert(payload[2] == std::byte{'T'});
  assert(payload[3] == std::byte{'S'});

  ParsedStatusPublication parsed;
  parsed.version = ReadU8(payload, 4U);
  parsed.lifecycle_state = ReadU8(payload, 5U);
  parsed.control_mode = ReadU8(payload, 6U);
  parsed.status_flags = ReadU8(payload, 7U);
  parsed.sample_time_ns = ReadU64Le(payload, 8U);
  parsed.publish_time_ns = ReadU64Le(payload, 16U);
  parsed.fault_timestamp_ns = ReadU64Le(payload, 24U);

  std::size_t offset = 32U;
  parsed.mode_label = ReadString(payload, &offset);
  parsed.fault_category = ReadU8(payload, offset + 0U);
  parsed.fault_severity = ReadU8(payload, offset + 1U);
  parsed.fault_lifecycle_state = ReadU8(payload, offset + 2U);
  offset += 4U;
  parsed.fault_sequence = ReadU64Le(payload, offset);
  offset += sizeof(std::uint64_t);
  parsed.fault_message = ReadString(payload, &offset);
  assert(offset == payload.size());
  return parsed;
}

DdsCommandSample MakeLowCmdSample() {
  DdsCommandSample sample;
  sample.endpoint = {
      .topic_name = "bridge/command",
      .type_name = "bridge.dds.LowCmd",
  };
  sample.sequence = 42U;
  sample.source_time_ns = 123456U;
  sample.receive_time_ns = 123999U;

  std::vector<std::byte>& payload = sample.payload;
  payload.reserve(16U + (3U * 2U * sizeof(double)));
  payload.push_back(std::byte{'L'});
  payload.push_back(std::byte{'C'});
  payload.push_back(std::byte{'M'});
  payload.push_back(std::byte{'D'});
  AppendU8(&payload, 1U);
  AppendU8(&payload, 5U);
  AppendU8(&payload, 2U);
  AppendU8(&payload, 0U);
  AppendU32Le(&payload, 2U);
  AppendU32Le(&payload, kCmdFieldPosition | kCmdFieldVelocity | kCmdFieldEffort);
  AppendDoubleLe(&payload, 1.25);
  AppendDoubleLe(&payload, -2.5);
  AppendDoubleLe(&payload, 0.5);
  AppendDoubleLe(&payload, -0.75);
  AppendDoubleLe(&payload, 3.0);
  AppendDoubleLe(&payload, -4.0);
  return sample;
}

StateFrame MakeStateFrame() {
  StateFrame state;
  state.sequence = 77U;
  state.sample_time_ns = 9000U;
  state.publish_time_ns = 9015U;
  state.lifecycle_state = LifecycleState::kActive;
  state.active_mode = ControlMode::kHybrid;
  state.mode_label = "hybrid_tracking";
  state.valid = true;
  state.stale = false;
  state.degraded = false;
  state.command_stale = true;
  state.joints.position = {0.1, -0.2};
  state.joints.velocity = {1.0, -1.5};
  state.joints.effort = {2.5, -3.5};
  state.joints.acceleration = {0.25, -0.5};
  state.imu = bridge::core::ImuSample{
      .sample_time_ns = 9005U,
      .valid = true,
      .stale = false,
      .orientation_xyzw = {0.0, 0.0, 0.70710678, 0.70710678},
      .angular_velocity_xyz = {0.01, 0.02, 0.03},
      .linear_acceleration_xyz = {0.0, 0.1, 9.81},
  };
  state.fault = BridgeFault{
      .category = FaultCategory::kAdapterHealth,
      .severity = FaultSeverity::kWarning,
      .lifecycle_state = LifecycleState::kFaulted,
      .message = "imu drift observed",
      .latched = true,
      .sequence = 5U,
      .timestamp_ns = 9010U,
  };
  return state;
}

class FakeDdsReader final : public DdsCommandReader {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "fake_dds_reader";
  }

  TransportCallResult start(DdsCommandSampleSink& sink) override {
    start_count_ += 1;
    sink_ = &sink;
    started_ = true;
    return {.status = TransportCallStatus::kOk,
            .detail = "reader started",
            .fault = std::nullopt};
  }

  TransportCallResult stop() override {
    stop_count_ += 1;
    started_ = false;
    return {.status = TransportCallStatus::kOk,
            .detail = "reader stopped",
            .fault = std::nullopt};
  }

  void Inject(DdsCommandSample sample) {
    assert(started_);
    assert(sink_ != nullptr);
    sink_->on_command_sample(std::move(sample));
  }

  [[nodiscard]] int start_count() const noexcept { return start_count_; }
  [[nodiscard]] int stop_count() const noexcept { return stop_count_; }

 private:
  DdsCommandSampleSink* sink_{nullptr};
  bool started_{false};
  int start_count_{0};
  int stop_count_{0};
};

class FakeDdsWriter final : public DdsPublicationWriter {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "fake_dds_writer";
  }

  TransportCallResult start() override {
    start_count_ += 1;
    started_ = true;
    return {.status = TransportCallStatus::kOk,
            .detail = "writer started",
            .fault = std::nullopt};
  }

  TransportCallResult publish(const DdsPublication& publication) override {
    assert(started_);
    publish_count_ += 1;
    publications_.push_back(publication);
    return {.status = TransportCallStatus::kOk,
            .detail = "publication captured",
            .fault = std::nullopt};
  }

  TransportCallResult stop() override {
    stop_count_ += 1;
    started_ = false;
    return {.status = TransportCallStatus::kOk,
            .detail = "writer stopped",
            .fault = std::nullopt};
  }

  [[nodiscard]] int start_count() const noexcept { return start_count_; }
  [[nodiscard]] int publish_count() const noexcept { return publish_count_; }
  [[nodiscard]] int stop_count() const noexcept { return stop_count_; }
  [[nodiscard]] const std::vector<DdsPublication>& publications() const noexcept {
    return publications_;
  }

 private:
  bool started_{false};
  int start_count_{0};
  int publish_count_{0};
  int stop_count_{0};
  std::vector<DdsPublication> publications_;
};

class FakeRobotSdkAdapter final : public RobotSdkAdapter {
 public:
  explicit FakeRobotSdkAdapter(StateFrame state) : state_(std::move(state)) {}

  [[nodiscard]] std::string_view name() const noexcept override {
    return "fake_dds_adapter";
  }

  AdapterCallResult initialize() override {
    initialized_ = true;
    initialize_count_ += 1;
    return {.status = AdapterCallStatus::kOk,
            .detail = "initialized",
            .fault = std::nullopt};
  }

  [[nodiscard]] AdapterReadResult read_state() override {
    read_count_ += 1;
    return {
        .result = {.status = AdapterCallStatus::kOk,
                   .detail = "read",
                   .fault = std::nullopt},
        .state = state_,
    };
  }

  AdapterCallResult write_command(const CommandFrame& command) override {
    write_command_count_ += 1;
    last_command_ = command;
    return {.status = AdapterCallStatus::kOk,
            .detail = "command written",
            .fault = std::nullopt};
  }

  AdapterCallResult write_safe_command() override {
    write_safe_command_count_ += 1;
    return {.status = AdapterCallStatus::kOk,
            .detail = "safe written",
            .fault = std::nullopt};
  }

  [[nodiscard]] AdapterHealthResult query_health() const override {
    AdapterHealth health;
    health.state = initialized_ ? HealthState::kHealthy : HealthState::kUnknown;
    health.safe_command_supported = true;
    health.connection_alive = initialized_;
    health.degraded = false;
    health.last_read_time_ns = state_.sample_time_ns;
    health.last_write_time_ns =
        last_command_.has_value() ? last_command_->receive_time_ns : 0U;
    return {
        .result = {.status = AdapterCallStatus::kOk,
                   .detail = "healthy",
                   .fault = std::nullopt},
        .health = std::move(health),
    };
  }

  AdapterCallResult shutdown() override {
    shutdown_count_ += 1;
    initialized_ = false;
    return {.status = AdapterCallStatus::kOk,
            .detail = "shutdown",
            .fault = std::nullopt};
  }

  [[nodiscard]] int write_command_count() const noexcept {
    return write_command_count_;
  }

  [[nodiscard]] int write_safe_command_count() const noexcept {
    return write_safe_command_count_;
  }

  [[nodiscard]] int read_count() const noexcept { return read_count_; }

  [[nodiscard]] const std::optional<CommandFrame>& last_command() const noexcept {
    return last_command_;
  }

 private:
  StateFrame state_;
  bool initialized_{false};
  int initialize_count_{0};
  int read_count_{0};
  int write_command_count_{0};
  int write_safe_command_count_{0};
  int shutdown_count_{0};
  std::optional<CommandFrame> last_command_;
};

class FakeDdsIngress final : public BridgeCommandIngress,
                             public DdsCommandSampleSink {
 public:
  FakeDdsIngress(DdsCommandReader& reader, DdsCommandDecoder& decoder)
      : reader_(reader), decoder_(decoder) {}

  [[nodiscard]] std::string_view name() const noexcept override {
    return "fake_dds_ingress";
  }

  RunnerCallResult start(BridgeCommandSink& command_sink,
                         BridgeRuntimeEventSink& event_sink) override {
    command_sink_ = &command_sink;
    event_sink_ = &event_sink;
    return bridge::dds::to_runner_call_result(reader_.start(*this));
  }

  RunnerCallResult stop() override {
    return bridge::dds::to_runner_call_result(reader_.stop());
  }

  void on_command_sample(DdsCommandSample sample) override {
    ++sample_count_;
    CommandDecodeResult result = decoder_.decode(sample);
    last_decode_status_ = result.status;
    if (result.accepted()) {
      last_submission_ = command_sink_->submit_latest_command(*result.command);
      return;
    }

    if (result.fault.has_value()) {
      last_fault_report_ = event_sink_->report_transport_fault(*result.fault);
    }
  }

  void on_transport_fault(TransportCallResult result) override {
    if (result.fault.has_value()) {
      last_fault_report_ = event_sink_->report_transport_fault(*result.fault);
    }
  }

  [[nodiscard]] int sample_count() const noexcept { return sample_count_; }
  [[nodiscard]] CommandDecodeStatus last_decode_status() const noexcept {
    return last_decode_status_;
  }
  [[nodiscard]] const std::optional<CommandSubmissionResult>&
  last_submission() const noexcept {
    return last_submission_;
  }

 private:
  DdsCommandReader& reader_;
  DdsCommandDecoder& decoder_;
  BridgeCommandSink* command_sink_{nullptr};
  BridgeRuntimeEventSink* event_sink_{nullptr};
  int sample_count_{0};
  CommandDecodeStatus last_decode_status_{CommandDecodeStatus::kRejected};
  std::optional<CommandSubmissionResult> last_submission_;
  std::optional<RunnerCallResult> last_fault_report_;
};

class FakeDdsEgress final : public BridgeStateEgress {
 public:
  FakeDdsEgress(DdsStateEncoder& encoder, DdsPublicationWriter& writer)
      : encoder_(encoder), writer_(writer) {}

  [[nodiscard]] std::string_view name() const noexcept override {
    return "fake_dds_egress";
  }

  RunnerCallResult start() override {
    return bridge::dds::to_runner_call_result(writer_.start());
  }

  RunnerCallResult publish(const StateFrame& state) override {
    StateEncodeResult encoded = encoder_.encode(state);
    if (!encoded.succeeded()) {
      return bridge::dds::to_runner_call_result(std::move(encoded.result));
    }

    RunnerCallStatus final_status = encoded.result.status == TransportCallStatus::kDegraded
                                        ? RunnerCallStatus::kDegraded
                                        : RunnerCallStatus::kOk;
    std::string detail = encoded.result.detail;
    std::optional<BridgeFault> fault = encoded.result.fault;
    for (const DdsPublication& publication : encoded.publications) {
      TransportCallResult result = writer_.publish(publication);
      if (!result.succeeded()) {
        return bridge::dds::to_runner_call_result(std::move(result));
      }
      if (result.status == TransportCallStatus::kDegraded) {
        final_status = RunnerCallStatus::kDegraded;
      }
      if (!result.detail.empty()) {
        detail = result.detail;
      }
      if (result.fault.has_value()) {
        fault = result.fault;
      }
    }
    return {.status = final_status, .detail = detail, .fault = std::move(fault)};
  }

  RunnerCallResult stop() override {
    return bridge::dds::to_runner_call_result(writer_.stop());
  }

 private:
  DdsStateEncoder& encoder_;
  DdsPublicationWriter& writer_;
};

BridgeRunnerConfig MakeRunnerConfig() {
  BridgeRunnerConfig config;
  config.loop_period_ns = 100U;
  config.command_stale_timeout_ns = 500U;
  config.startup_idle_policy = StartupIdlePolicy::kPreferSafeCommand;
  config.fault_policy.fail_stop_on_transport_fault = true;
  config.fault_policy.fail_stop_on_command_stale = true;
  config.fault_policy.fail_stop_on_adapter_degraded = false;
  config.fault_policy.write_safe_command_on_fault = true;
  config.fault_policy.continue_publishing_in_fault = true;
  return config;
}

void TestLowCmdCodecDecodesFakePayload() {
  std::unique_ptr<DdsCommandDecoder> decoder = make_lowcmd_codec();
  const DdsCommandSample sample = MakeLowCmdSample();

  const CommandDecodeResult result = decoder->decode(sample);
  assert(result.status == CommandDecodeStatus::kAccepted);
  assert(result.accepted());
  assert(result.detail == "lowcmd payload decoded");
  assert(result.command.has_value());

  const CommandFrame& command = *result.command;
  assert(command.sequence == 42U);
  assert(command.source_time_ns == 123456U);
  assert(command.receive_time_ns == 123999U);
  assert(command.valid);
  assert(!command.stale);
  assert(command.degraded);
  assert(command.mode == ControlMode::kHybrid);
  assert(command.mode_label == "hybrid");
  AssertVectorNear(command.joints.position, {1.25, -2.5});
  AssertVectorNear(command.joints.velocity, {0.5, -0.75});
  AssertVectorNear(command.joints.effort, {3.0, -4.0});
  assert(command.joints.stiffness.empty());
  assert(command.joints.damping.empty());
}

void TestLowStateCodecEncodesStateAndStatusPublications() {
  std::unique_ptr<DdsStateEncoder> encoder = make_lowstate_codec();
  const StateFrame state = MakeStateFrame();

  const StateEncodeResult result = encoder->encode(state);
  assert(result.result.status == TransportCallStatus::kDegraded);
  assert(result.succeeded());
  assert(result.result.fault.has_value());
  assert(result.publications.size() == 2U);

  const DdsPublication& state_publication = result.publications[0];
  const DdsPublication& status_publication = result.publications[1];

  assert(state_publication.kind == DdsPublicationKind::kState);
  assert(state_publication.endpoint.topic_name == "bridge/state");
  assert(state_publication.endpoint.type_name == "bridge.dds.LowState");
  assert(state_publication.sequence == state.sequence);
  assert(state_publication.timestamp_ns == state.publish_time_ns);
  assert(state_publication.degraded);

  assert(status_publication.kind == DdsPublicationKind::kStatus);
  assert(status_publication.endpoint.topic_name == "bridge/status");
  assert(status_publication.endpoint.type_name == "bridge.dds.LowStatus");
  assert(status_publication.sequence == state.sequence);
  assert(status_publication.timestamp_ns == state.publish_time_ns);
  assert(status_publication.degraded);

  const ParsedStatePublication parsed_state = ParseStatePublication(state_publication);
  assert(parsed_state.version == 1U);
  assert(parsed_state.state_flags == (kStateFlagValid | kStateFlagCommandStale));
  assert(parsed_state.imu_flags == (kImuFlagPresent | kImuFlagValid));
  assert(parsed_state.imu_field_mask ==
         (kImuFieldOrientation | kImuFieldAngularVelocity |
          kImuFieldLinearAcceleration));
  assert(parsed_state.joint_count == 2U);
  assert(parsed_state.joint_field_mask ==
         (kStateJointFieldPosition | kStateJointFieldVelocity |
          kStateJointFieldEffort | kStateJointFieldAcceleration));
  assert(parsed_state.sample_time_ns == state.sample_time_ns);
  AssertVectorNear(parsed_state.position, state.joints.position);
  AssertVectorNear(parsed_state.velocity, state.joints.velocity);
  AssertVectorNear(parsed_state.effort, state.joints.effort);
  AssertVectorNear(parsed_state.acceleration, state.joints.acceleration);
  assert(parsed_state.imu_sample_time_ns == state.imu->sample_time_ns);
  AssertVectorNear(parsed_state.orientation_xyzw, state.imu->orientation_xyzw);
  AssertVectorNear(parsed_state.angular_velocity_xyz,
                   state.imu->angular_velocity_xyz);
  AssertVectorNear(parsed_state.linear_acceleration_xyz,
                   state.imu->linear_acceleration_xyz);

  const ParsedStatusPublication parsed_status = ParseStatusPublication(status_publication);
  assert(parsed_status.version == 1U);
  assert(parsed_status.lifecycle_state == EncodeLifecycleState(state.lifecycle_state));
  assert(parsed_status.control_mode == EncodeControlMode(state.active_mode));
  assert(parsed_status.status_flags ==
         (kStatusFlagValid | kStatusFlagCommandStale | kStatusFlagFaultPresent |
          kStatusFlagFaultLatched));
  assert(parsed_status.sample_time_ns == state.sample_time_ns);
  assert(parsed_status.publish_time_ns == state.publish_time_ns);
  assert(parsed_status.fault_timestamp_ns == state.fault->timestamp_ns);
  assert(parsed_status.mode_label == state.mode_label);
  assert(parsed_status.fault_category ==
         static_cast<std::uint8_t>(state.fault->category));
  assert(parsed_status.fault_severity ==
         static_cast<std::uint8_t>(state.fault->severity));
  assert(parsed_status.fault_lifecycle_state ==
         EncodeLifecycleState(state.fault->lifecycle_state));
  assert(parsed_status.fault_sequence == state.fault->sequence);
  assert(parsed_status.fault_message == state.fault->message);
}

void TestRunnerCompatibleFakeTransportClosedLoop() {
  std::unique_ptr<DdsCommandDecoder> decoder = make_lowcmd_codec();
  std::unique_ptr<DdsStateEncoder> encoder = make_lowstate_codec();
  FakeDdsReader reader;
  FakeDdsWriter writer;
  FakeDdsIngress ingress(reader, *decoder);
  FakeDdsEgress egress(*encoder, writer);
  FakeRobotSdkAdapter adapter(MakeStateFrame());

  BridgeRunner runner(MakeRunnerConfig(),
                      {.adapter = &adapter,
                       .command_ingress = &ingress,
                       .state_egress = &egress});

  const RunnerCallResult initialize = runner.initialize();
  assert(initialize.succeeded());
  assert(reader.start_count() == 1);
  assert(writer.start_count() == 1);

  reader.Inject(MakeLowCmdSample());
  assert(ingress.sample_count() == 1);
  assert(ingress.last_decode_status() == CommandDecodeStatus::kAccepted);
  assert(ingress.last_submission().has_value());
  assert(ingress.last_submission()->result.succeeded());
  assert(ingress.last_submission()->stored_sequence == 42U);

  const RunnerCallResult run = runner.run();
  assert(run.succeeded());
  assert(adapter.read_count() == 1);
  assert(adapter.write_command_count() == 1);
  assert(adapter.write_safe_command_count() == 0);
  assert(adapter.last_command().has_value());
  assert(adapter.last_command()->sequence == 42U);
  assert(adapter.last_command()->mode == ControlMode::kHybrid);
  AssertVectorNear(adapter.last_command()->joints.position, {1.25, -2.5});
  AssertVectorNear(adapter.last_command()->joints.velocity, {0.5, -0.75});
  AssertVectorNear(adapter.last_command()->joints.effort, {3.0, -4.0});

  assert(writer.publish_count() == 2);
  assert(writer.publications().size() == 2U);
  assert(writer.publications()[0].kind == DdsPublicationKind::kState);
  assert(writer.publications()[1].kind == DdsPublicationKind::kStatus);

  const ParsedStatusPublication status = ParseStatusPublication(writer.publications()[1]);
  assert(status.mode_label == "hybrid_tracking");
  assert(status.control_mode == EncodeControlMode(ControlMode::kHybrid));

  const RunnerCallResult shutdown = runner.shutdown();
  assert(shutdown.succeeded());
  assert(reader.stop_count() == 1);
  assert(writer.stop_count() == 1);
}

}

int main() {
  TestLowCmdCodecDecodesFakePayload();
  TestLowStateCodecEncodesStateAndStatusPublications();
  TestRunnerCompatibleFakeTransportClosedLoop();
  return 0;
}
