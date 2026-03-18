#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "../core/bridge_runner.hpp"
#include "../core/types.hpp"

namespace bridge::dds {

enum class TransportCallStatus {
  kOk,
  kDegraded,
  kUnsupported,
  kRejected,
  kRetryableError,
  kFatalError,
};

struct TransportCallResult {
  TransportCallStatus status{TransportCallStatus::kOk};
  std::string detail;
  std::optional<core::BridgeFault> fault;

  [[nodiscard]] bool succeeded() const noexcept {
    return status == TransportCallStatus::kOk ||
           status == TransportCallStatus::kDegraded;
  }
};

[[nodiscard]] inline core::RunnerCallStatus to_runner_call_status(
    const TransportCallStatus status) noexcept {
  switch (status) {
    case TransportCallStatus::kOk:
      return core::RunnerCallStatus::kOk;
    case TransportCallStatus::kDegraded:
      return core::RunnerCallStatus::kDegraded;
    case TransportCallStatus::kUnsupported:
      return core::RunnerCallStatus::kUnsupported;
    case TransportCallStatus::kRejected:
      return core::RunnerCallStatus::kRejected;
    case TransportCallStatus::kRetryableError:
      return core::RunnerCallStatus::kRetryableError;
    case TransportCallStatus::kFatalError:
      return core::RunnerCallStatus::kFatalError;
  }

  return core::RunnerCallStatus::kFatalError;
}

[[nodiscard]] inline core::RunnerCallResult to_runner_call_result(
    TransportCallResult result) {
  return {
      .status = to_runner_call_status(result.status),
      .detail = std::move(result.detail),
      .fault = std::move(result.fault),
  };
}

enum class DdsPublicationKind {
  kState,
  kStatus,
};

struct DdsEndpointAddress {
  std::string topic_name;
  std::string type_name;
};

struct DdsCommandSample {
  DdsEndpointAddress endpoint;
  std::uint64_t sequence{0};
  std::uint64_t source_time_ns{0};
  std::uint64_t receive_time_ns{0};
  std::vector<std::byte> payload;
};

enum class CommandDecodeStatus {
  kAccepted,
  kRejected,
  kUnsupported,
  kRetryableError,
  kFatalError,
};

struct CommandDecodeResult {
  CommandDecodeStatus status{CommandDecodeStatus::kRejected};
  std::optional<core::CommandFrame> command;
  std::string detail;
  std::optional<core::BridgeFault> fault;

  [[nodiscard]] bool accepted() const noexcept {
    return status == CommandDecodeStatus::kAccepted && command.has_value();
  }
};

struct DdsPublication {
  DdsPublicationKind kind{DdsPublicationKind::kState};
  DdsEndpointAddress endpoint;
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};
  bool degraded{false};
  std::vector<std::byte> payload;
};

struct StateEncodeResult {
  TransportCallResult result;
  std::vector<DdsPublication> publications;

  [[nodiscard]] bool succeeded() const noexcept {
    return result.succeeded();
  }
};

class DdsCommandDecoder {
 public:
  virtual ~DdsCommandDecoder() = default;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;

  [[nodiscard]] virtual CommandDecodeResult decode(
      const DdsCommandSample& sample) = 0;
};

class DdsStateEncoder {
 public:
  virtual ~DdsStateEncoder() = default;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;

  [[nodiscard]] virtual StateEncodeResult encode(
      const core::StateFrame& state) = 0;
};

[[nodiscard]] std::unique_ptr<DdsCommandDecoder> make_lowcmd_codec();

[[nodiscard]] std::unique_ptr<DdsStateEncoder> make_lowstate_codec();

class DdsCommandSampleSink {
 public:
  virtual ~DdsCommandSampleSink() = default;

  virtual void on_command_sample(DdsCommandSample sample) = 0;

  virtual void on_transport_fault(TransportCallResult result) = 0;
};

class DdsCommandReader {
 public:
  virtual ~DdsCommandReader() = default;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;

  virtual TransportCallResult start(DdsCommandSampleSink& sink) = 0;

  virtual TransportCallResult stop() = 0;
};

class DdsPublicationWriter {
 public:
  virtual ~DdsPublicationWriter() = default;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;

  virtual TransportCallResult start() = 0;

  virtual TransportCallResult publish(const DdsPublication& publication) = 0;

  virtual TransportCallResult stop() = 0;
};

struct DdsCommandIngressDependencies {
  DdsCommandReader* reader{nullptr};
  DdsCommandDecoder* decoder{nullptr};
};

class DdsCommandIngress final : public core::BridgeCommandIngress,
                                public DdsCommandSampleSink {
 public:
  explicit DdsCommandIngress(DdsCommandIngressDependencies dependencies);
  ~DdsCommandIngress() override;

  DdsCommandIngress(const DdsCommandIngress&) = delete;
  DdsCommandIngress& operator=(const DdsCommandIngress&) = delete;
  DdsCommandIngress(DdsCommandIngress&&) = delete;
  DdsCommandIngress& operator=(DdsCommandIngress&&) = delete;

  [[nodiscard]] std::string_view name() const noexcept override;

  core::RunnerCallResult start(core::BridgeCommandSink& command_sink,
                               core::BridgeRuntimeEventSink& event_sink) override;

  core::RunnerCallResult stop() override;

  void on_command_sample(DdsCommandSample sample) override;

  void on_transport_fault(TransportCallResult result) override;

 private:
  class Impl;

  std::unique_ptr<Impl> impl_;
};

struct DdsStateEgressDependencies {
  DdsStateEncoder* encoder{nullptr};
  DdsPublicationWriter* writer{nullptr};
};

class DdsStateEgress final : public core::BridgeStateEgress {
 public:
  explicit DdsStateEgress(DdsStateEgressDependencies dependencies);
  ~DdsStateEgress() override;

  DdsStateEgress(const DdsStateEgress&) = delete;
  DdsStateEgress& operator=(const DdsStateEgress&) = delete;
  DdsStateEgress(DdsStateEgress&&) = delete;
  DdsStateEgress& operator=(DdsStateEgress&&) = delete;

  [[nodiscard]] std::string_view name() const noexcept override;

  core::RunnerCallResult start() override;

  core::RunnerCallResult publish(const core::StateFrame& state) override;

  core::RunnerCallResult stop() override;

 private:
  class Impl;

  std::unique_ptr<Impl> impl_;
};

}
