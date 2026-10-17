#pragma once

#include <cstdint>
#include <string>

#include "bridge/gamepad/common_gamepad.hpp"
#include "bridge/gamepad/sdl_gamepad.hpp"

namespace bridge::demo {

[[nodiscard]] std::string format_gamepad_sender_table(
    const gamepad::CommonGamepad& gamepad,
    const gamepad::SdlGamepadSnapshot& snapshot,
    std::uint64_t published,
    std::uint64_t publish_rate_hz,
    int controller_index,
    bool color = true);

}  // namespace bridge::demo
