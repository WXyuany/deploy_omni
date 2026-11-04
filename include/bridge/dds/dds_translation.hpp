#pragma once

#include <optional>
#include <string>

#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/idl/hg/BmsState_.hpp"
#include "unitree/idl/hg/IMUState_.hpp"
#include "unitree/idl/hg/MainBoardState_.hpp"

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
  std::optional<unitree_hg::msg::dds_::LowState_> state;
};

struct ImuStateTranslation {
  TranslationResult result;
  std::optional<unitree_hg::msg::dds_::IMUState_> state;
};

struct BmsStateTranslation {
  TranslationResult result;
  std::optional<unitree_hg::msg::dds_::BmsState_> state;
};

struct MainBoardStateTranslation {
  TranslationResult result;
  std::optional<unitree_hg::msg::dds_::MainBoardState_> state;
};

[[nodiscard]] CommandTranslation translate_lowcmd_to_command(
    const unitree_hg::msg::dds_::LowCmd_& message,
    std::uint64_t receive_time_ns);

[[nodiscard]] CommandTranslation translate_lowcmd_to_hardware_command(
    const unitree_hg::msg::dds_::LowCmd_& message,
    std::uint64_t receive_time_ns,
    const config::JointMap& joint_map);

[[nodiscard]] StateTranslation translate_state_to_lowstate(
    const core::StateFrame& state);

[[nodiscard]] StateTranslation translate_hardware_state_to_lowstate(
    const core::StateFrame& hardware_state,
    const config::JointMap& joint_map);

[[nodiscard]] ImuStateTranslation translate_state_to_imu_state(
    const core::StateFrame& state);

[[nodiscard]] BmsStateTranslation translate_bms_sample_to_bms_state(
    const core::BmsStateSample& state);

[[nodiscard]] MainBoardStateTranslation translate_mainboard_sample_to_mainboard_state(
    const core::MainBoardStateSample& state);

}
