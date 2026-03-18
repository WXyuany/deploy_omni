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

constexpr std::byte kLowCmdMagic[4]{std::byte{'L'}, std::byte{'C'},
                                    std::byte{'M'}, std::byte{'D'}};
constexpr std::uint8_t kLowCmdVersion = 1U;
constexpr std::size_t kHeaderSize = 16U;

constexpr std::uint8_t kFlagStale = 1U << 0U;
constexpr std::uint8_t kFlagDegraded = 1U << 1U;
constexpr std::uint8_t kSupportedFlags = kFlagStale | kFlagDegraded;

constexpr std::uint32_t kFieldPosition = 1U << 0U;
constexpr std::uint32_t kFieldVelocity = 1U << 1U;
constexpr std::uint32_t kFieldEffort = 1U << 2U;
constexpr std::uint32_t kFieldStiffness = 1U << 3U;
constexpr std::uint32_t kFieldDamping = 1U << 4U;
constexpr std::uint32_t kSupportedFieldMask =
    kFieldPosition | kFieldVelocity | kFieldEffort | kFieldStiffness |
    kFieldDamping;

enum class LowCmdModeCode : std::uint8_t {
  kSafeIdle = 1U,
  kTorque = 2U,
  kVelocity = 3U,
  kPosition = 4U,
  kHybrid = 5U,
};

struct ParsedHeader {
  core::ControlMode mode{core::ControlMode::kUnknown};
  std::string_view mode_label;
  bool stale{false};
  bool degraded{false};
  std::uint32_t joint_count{0};
  std::uint32_t field_mask{0};
};

[[nodiscard]] core::BridgeFault make_fault(const DdsCommandSample& sample,
                                           const core::FaultCategory category,
                                           const core::FaultSeverity severity,
                                           std::string message) {
  return {
      .category = category,
      .severity = severity,
      .lifecycle_state = core::LifecycleState::kCreated,
      .message = std::move(message),
      .latched = false,
      .sequence = sample.sequence,
      .timestamp_ns = sample.receive_time_ns != 0U ? sample.receive_time_ns
                                                   : sample.source_time_ns,
  };
}

[[nodiscard]] CommandDecodeResult make_result(
    const CommandDecodeStatus status, std::string detail,
    std::optional<core::BridgeFault> fault = std::nullopt,
    std::optional<core::CommandFrame> command = std::nullopt) {
  return {
      .status = status,
      .command = std::move(command),
      .detail = std::move(detail),
      .fault = std::move(fault),
  };
}

[[nodiscard]] bool starts_with_magic(const std::vector<std::byte>& payload) {
  return payload.size() >= 4U && payload[0] == kLowCmdMagic[0] &&
         payload[1] == kLowCmdMagic[1] && payload[2] == kLowCmdMagic[2] &&
         payload[3] == kLowCmdMagic[3];
}

[[nodiscard]] std::uint8_t read_u8(const std::vector<std::byte>& payload,
                                   const std::size_t offset) {
  return std::to_integer<std::uint8_t>(payload[offset]);
}

[[nodiscard]] std::uint32_t read_u32_le(const std::vector<std::byte>& payload,
                                        const std::size_t offset) {
  return static_cast<std::uint32_t>(read_u8(payload, offset + 0U)) |
         (static_cast<std::uint32_t>(read_u8(payload, offset + 1U)) << 8U) |
         (static_cast<std::uint32_t>(read_u8(payload, offset + 2U)) << 16U) |
         (static_cast<std::uint32_t>(read_u8(payload, offset + 3U)) << 24U);
}

[[nodiscard]] std::uint64_t read_u64_le(const std::vector<std::byte>& payload,
                                        const std::size_t offset) {
  std::uint64_t value = 0U;
  for (std::size_t index = 0; index < sizeof(std::uint64_t); ++index) {
    value |= static_cast<std::uint64_t>(read_u8(payload, offset + index))
             << (index * 8U);
  }
  return value;
}

[[nodiscard]] std::optional<double> read_double_le(
    const std::vector<std::byte>& payload, const std::size_t offset) {
  const std::uint64_t raw_bits = read_u64_le(payload, offset);
  double value = 0.0;
  std::memcpy(&value, &raw_bits, sizeof(value));
  if (!std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

[[nodiscard]] std::optional<ParsedHeader> parse_header(
    const DdsCommandSample& sample) {
  const std::vector<std::byte>& payload = sample.payload;
  if (payload.size() < kHeaderSize) {
    return std::nullopt;
  }

  ParsedHeader header;
  const auto mode_code = static_cast<LowCmdModeCode>(read_u8(payload, 5U));
  switch (mode_code) {
    case LowCmdModeCode::kSafeIdle:
      header.mode = core::ControlMode::kSafeIdle;
      header.mode_label = "safe_idle";
      break;
    case LowCmdModeCode::kTorque:
      header.mode = core::ControlMode::kTorque;
      header.mode_label = "torque";
      break;
    case LowCmdModeCode::kVelocity:
      header.mode = core::ControlMode::kVelocity;
      header.mode_label = "velocity";
      break;
    case LowCmdModeCode::kPosition:
      header.mode = core::ControlMode::kPosition;
      header.mode_label = "position";
      break;
    case LowCmdModeCode::kHybrid:
      header.mode = core::ControlMode::kHybrid;
      header.mode_label = "hybrid";
      break;
  }

  const std::uint8_t flags = read_u8(payload, 6U);
  header.stale = (flags & kFlagStale) != 0U;
  header.degraded = (flags & kFlagDegraded) != 0U;
  header.joint_count = read_u32_le(payload, 8U);
  header.field_mask = read_u32_le(payload, 12U);
  return header;
}

[[nodiscard]] std::optional<std::string> validate_header(
    const DdsCommandSample& sample, const ParsedHeader& header) {
  const std::vector<std::byte>& payload = sample.payload;
  if (!starts_with_magic(payload)) {
    return std::string{"unsupported command payload magic"};
  }
  if (read_u8(payload, 4U) != kLowCmdVersion) {
    return std::string{"unsupported command payload version"};
  }
  if (header.mode == core::ControlMode::kUnknown) {
    return std::string{"unsupported command control mode"};
  }
  if ((read_u8(payload, 6U) & ~kSupportedFlags) != 0U) {
    return std::string{"unsupported command payload flags"};
  }
  if (payload[7] != std::byte{0}) {
    return std::string{"reserved command payload byte must be zero"};
  }
  if ((header.field_mask & ~kSupportedFieldMask) != 0U) {
    return std::string{"unsupported command field mask"};
  }
  if (header.field_mask == 0U && header.mode != core::ControlMode::kSafeIdle) {
    return std::string{"non-idle command requires at least one joint field"};
  }
  if (header.mode == core::ControlMode::kSafeIdle &&
      header.field_mask != 0U && header.joint_count == 0U) {
    return std::string{"safe-idle command field mask requires positive joint count"};
  }

  const std::size_t field_count =
      static_cast<std::size_t>((header.field_mask & kFieldPosition) != 0U) +
      static_cast<std::size_t>((header.field_mask & kFieldVelocity) != 0U) +
      static_cast<std::size_t>((header.field_mask & kFieldEffort) != 0U) +
      static_cast<std::size_t>((header.field_mask & kFieldStiffness) != 0U) +
      static_cast<std::size_t>((header.field_mask & kFieldDamping) != 0U);
  if (field_count != 0U &&
      header.joint_count >
          std::numeric_limits<std::size_t>::max() / (field_count * sizeof(double))) {
    return std::string{"command joint count is too large"};
  }
  return std::nullopt;
}

[[nodiscard]] bool append_block(const DdsCommandSample& sample,
                                const ParsedHeader& header,
                                const std::uint32_t field_bit,
                                std::size_t* offset,
                                std::vector<double>* values,
                                std::string* detail) {
  if ((header.field_mask & field_bit) == 0U) {
    return true;
  }

  values->clear();
  values->reserve(header.joint_count);
  for (std::uint32_t index = 0; index < header.joint_count; ++index) {
    if (*offset + sizeof(double) > sample.payload.size()) {
      *detail = "command payload ended before all joint values were decoded";
      return false;
    }
    std::optional<double> value = read_double_le(sample.payload, *offset);
    if (!value.has_value()) {
      *detail = "command payload contains a non-finite joint value";
      return false;
    }
    values->push_back(*value);
    *offset += sizeof(double);
  }
  return true;
}

class LowCmdCommandDecoder final : public DdsCommandDecoder {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "lowcmd_codec";
  }

  [[nodiscard]] CommandDecodeResult decode(const DdsCommandSample& sample) override {
    if (sample.payload.empty()) {
      return make_result(
          CommandDecodeStatus::kRejected, "command payload is empty",
          make_fault(sample, core::FaultCategory::kInvalidCommand,
                     core::FaultSeverity::kRecoverable,
                     "command payload is empty"));
    }
    if (sample.payload.size() < kHeaderSize) {
      return make_result(
          CommandDecodeStatus::kRejected,
          "command payload smaller than lowcmd header",
          make_fault(sample, core::FaultCategory::kInvalidCommand,
                     core::FaultSeverity::kRecoverable,
                     "command payload smaller than lowcmd header"));
    }

    const bool has_magic = starts_with_magic(sample.payload);
    if (!has_magic) {
      return make_result(
          CommandDecodeStatus::kUnsupported, "unsupported command payload magic",
          make_fault(sample, core::FaultCategory::kInvalidCommand,
                     core::FaultSeverity::kRecoverable,
                     "unsupported command payload magic"));
    }
    if (read_u8(sample.payload, 4U) != kLowCmdVersion) {
      return make_result(
          CommandDecodeStatus::kUnsupported,
          "unsupported command payload version",
          make_fault(sample, core::FaultCategory::kInvalidCommand,
                     core::FaultSeverity::kRecoverable,
                     "unsupported command payload version"));
    }

    const std::optional<ParsedHeader> header = parse_header(sample);
    if (!header.has_value()) {
      return make_result(
          CommandDecodeStatus::kFatalError,
          "internal error while parsing command payload header",
          make_fault(sample, core::FaultCategory::kInternal,
                     core::FaultSeverity::kFatal,
                     "internal error while parsing command payload header"));
    }
    if (const std::optional<std::string> issue = validate_header(sample, *header);
        issue.has_value()) {
      const bool unsupported = issue->rfind("unsupported", 0U) == 0U;
      return make_result(
          unsupported ? CommandDecodeStatus::kUnsupported
                      : CommandDecodeStatus::kRejected,
          *issue,
          make_fault(sample, core::FaultCategory::kInvalidCommand,
                     core::FaultSeverity::kRecoverable, *issue));
    }

    core::CommandFrame command;
    command.sequence = sample.sequence;
    command.source_time_ns = sample.source_time_ns;
    command.receive_time_ns = sample.receive_time_ns;
    command.mode = header->mode;
    command.mode_label = std::string{header->mode_label};
    command.valid = true;
    command.stale = header->stale;
    command.degraded = header->degraded;

    std::size_t offset = kHeaderSize;
    std::string detail;
    if (!append_block(sample, *header, kFieldPosition, &offset,
                      &command.joints.position, &detail) ||
        !append_block(sample, *header, kFieldVelocity, &offset,
                      &command.joints.velocity, &detail) ||
        !append_block(sample, *header, kFieldEffort, &offset,
                      &command.joints.effort, &detail) ||
        !append_block(sample, *header, kFieldStiffness, &offset,
                      &command.joints.stiffness, &detail) ||
        !append_block(sample, *header, kFieldDamping, &offset,
                      &command.joints.damping, &detail)) {
      return make_result(
          CommandDecodeStatus::kRejected, detail,
          make_fault(sample, core::FaultCategory::kInvalidCommand,
                     core::FaultSeverity::kRecoverable, detail));
    }

    if (offset != sample.payload.size()) {
      return make_result(
          CommandDecodeStatus::kRejected,
          "command payload contains trailing bytes",
          make_fault(sample, core::FaultCategory::kInvalidCommand,
                     core::FaultSeverity::kRecoverable,
                     "command payload contains trailing bytes"));
    }

    return make_result(CommandDecodeStatus::kAccepted,
                       "lowcmd payload decoded", std::nullopt,
                       std::move(command));
  }
};

}

[[nodiscard]] std::unique_ptr<DdsCommandDecoder> make_lowcmd_codec() {
  return std::unique_ptr<DdsCommandDecoder>(new LowCmdCommandDecoder());
}

}
