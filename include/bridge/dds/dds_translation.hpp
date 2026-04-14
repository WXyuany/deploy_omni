#pragma once

#include <optional>
#include <string>

#include "../../../idl/bridge/BridgeMessages.hpp"
#include "../config/joint_map.hpp"
#include "../core/types.hpp"

namespace bridge::dds {

enum class TranslationStatus {
  kOk,
  kRejected,
  kUnsupported,
};

struct TranslationResult {
  TranslationStatus status{TranslationStatus::kRejected};
  std::string detail;
  std::optional<core::BridgeFault> fault;

  [[nodiscard]] bool succeeded() const noexcept {
    return status == TranslationStatus::kOk;
  }
};

struct CommandTranslation {
  TranslationResult result;
  std::optional<core::CommandFrame> command;
};

struct StateTranslation {
  TranslationResult result;
  std::optional<bridge_wire::LowState> state;
};

struct StatusTranslation {
  TranslationResult result;
  std::optional<bridge_wire::BridgeStatus> status;
};

[[nodiscard]] CommandTranslation translate_lowcmd_to_command(
    const bridge_wire::LowCmd& message,
    std::uint64_t receive_time_ns);

[[nodiscard]] CommandTranslation translate_lowcmd_to_hardware_command(
    const bridge_wire::LowCmd& message,
    std::uint64_t receive_time_ns,
    const config::JointMap& joint_map);

[[nodiscard]] StateTranslation translate_state_to_lowstate(
    const core::StateFrame& state);

[[nodiscard]] StateTranslation translate_hardware_state_to_lowstate(
    const core::StateFrame& hardware_state,
    const config::JointMap& joint_map);

[[nodiscard]] StatusTranslation translate_state_to_bridge_status(
    const core::StateFrame& state);

[[nodiscard]] StatusTranslation translate_hardware_state_to_bridge_status(
    const core::StateFrame& hardware_state,
    const config::JointMap& joint_map);

}
