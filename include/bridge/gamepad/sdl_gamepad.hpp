#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "common_gamepad.hpp"

namespace bridge::gamepad {

enum SdlGamepadAxis : std::size_t {
  kLeftX = 0,
  kLeftY,
  kRightX,
  kRightY,
  kTriggerLeft,
  kTriggerRight,
  kSdlGamepadAxisCount,
};

enum SdlGamepadButton : std::size_t {
  kA = 0,
  kB,
  kX,
  kY,
  kBack,
  kGuide,
  kStart,
  kLeftStick,
  kRightStick,
  kLeftShoulder,
  kRightShoulder,
  kDpadUp,
  kDpadDown,
  kDpadLeft,
  kDpadRight,
  kSdlGamepadButtonCount,
};

struct SdlGamepadSnapshot {
  std::array<std::int16_t, kSdlGamepadAxisCount> axes{};
  std::array<bool, kSdlGamepadButtonCount> buttons{};
};

struct SdlGamepadRemoteOptions {
  int controller_index{0};
  std::string mapping_path;
};

struct SdlGamepadStartResult {
  bool connected{false};
};

void convert_sdl_snapshot_to_common_gamepad(const SdlGamepadSnapshot& snapshot,
                                            CommonGamepad& gamepad) noexcept;

[[nodiscard]] std::string format_sdl_gamepad_debug_line(
    const SdlGamepadSnapshot& snapshot, const CommonGamepad& gamepad);

class SdlGamepadRemote {
 public:
  explicit SdlGamepadRemote(SdlGamepadRemoteOptions options);
  SdlGamepadRemote(const SdlGamepadRemote&) = delete;
  SdlGamepadRemote& operator=(const SdlGamepadRemote&) = delete;
  ~SdlGamepadRemote();

  SdlGamepadStartResult start();
  [[nodiscard]] std::optional<CommonGamepad> read();
  [[nodiscard]] std::optional<SdlGamepadSnapshot> last_snapshot() const;
  void stop() noexcept;

 private:
  SdlGamepadRemoteOptions options_;
  CommonGamepad gamepad_;
  std::optional<SdlGamepadSnapshot> last_snapshot_;
  void* controller_{nullptr};
  bool sdl_initialized_{false};
};

}  // namespace bridge::gamepad
