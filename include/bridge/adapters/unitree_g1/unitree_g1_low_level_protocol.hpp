#pragma once

#include <cstddef>
#include <cstdint>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include "../../core/types.hpp"

namespace bridge::adapters::unitree_g1 {

[[nodiscard]] bool validate_low_state_crc(
    const unitree_hg::msg::dds_::LowState_& state);

[[nodiscard]] unitree_hg::msg::dds_::LowCmd_ build_low_cmd_from_command(
    const bridge::core::CommandFrame& command, std::size_t hardware_slot_count,
    std::uint8_t mode_machine);

[[nodiscard]] unitree_hg::msg::dds_::LowCmd_ build_safe_low_cmd(
    std::size_t hardware_slot_count, std::uint8_t mode_machine);

}  // namespace bridge::adapters::unitree_g1
