#include "../../../include/bridge/gamepad/sdl_gamepad.hpp"

#include <SDL2/SDL.h>

#include <algorithm>
#include <cstdio>
#include <sstream>

namespace bridge::gamepad {
namespace {

float NormalizeStick(const std::int16_t value) noexcept {
  if (value >= 0) {
    return static_cast<float>(value) / 32767.0F;
  }
  return static_cast<float>(value) / 32768.0F;
}

float NormalizeTrigger(const std::int16_t value) noexcept {
  const float normalized = static_cast<float>(value) / 32767.0F;
  return std::clamp(normalized, 0.0F, 1.0F);
}

bool Button(const SdlGamepadSnapshot& snapshot, const SdlGamepadButton button) noexcept {
  return snapshot.buttons[button];
}

SDL_GameController* Controller(void* controller) noexcept {
  return static_cast<SDL_GameController*>(controller);
}

}  // namespace

void convert_sdl_snapshot_to_common_gamepad(const SdlGamepadSnapshot& snapshot,
                                            CommonGamepad& gamepad) noexcept {
  gamepad.raw_lx = NormalizeStick(snapshot.axes[kLeftX]);
  gamepad.raw_ly = NormalizeStick(snapshot.axes[kLeftY]);
  gamepad.raw_rx = NormalizeStick(snapshot.axes[kRightX]);
  gamepad.raw_ry = NormalizeStick(snapshot.axes[kRightY]);
  gamepad.raw_lt = NormalizeTrigger(snapshot.axes[kTriggerLeft]);
  gamepad.raw_rt = NormalizeTrigger(snapshot.axes[kTriggerRight]);

  gamepad.lt = gamepad.raw_lt;
  gamepad.rt = gamepad.raw_rt;

  gamepad.A.update(Button(snapshot, kA));
  gamepad.B.update(Button(snapshot, kB));
  gamepad.X.update(Button(snapshot, kX));
  gamepad.Y.update(Button(snapshot, kY));
  gamepad.back.update(Button(snapshot, kBack));
  gamepad.select.update(Button(snapshot, kBack));
  gamepad.start.update(Button(snapshot, kStart));
  gamepad.LB.update(Button(snapshot, kLeftShoulder));
  gamepad.L1.update(Button(snapshot, kLeftShoulder));
  gamepad.RB.update(Button(snapshot, kRightShoulder));
  gamepad.R1.update(Button(snapshot, kRightShoulder));
  gamepad.up.update(Button(snapshot, kDpadUp));
  gamepad.down.update(Button(snapshot, kDpadDown));
  gamepad.left.update(Button(snapshot, kDpadLeft));
  gamepad.right.update(Button(snapshot, kDpadRight));

  gamepad.update_filter();
}

std::string format_sdl_gamepad_debug_line(const SdlGamepadSnapshot& snapshot,
                                          const CommonGamepad& gamepad) {
  std::ostringstream out;
  out << "[SdlGamepadRemote.read] raw_axes={"
      << "lx=" << snapshot.axes[kLeftX]
      << " ly=" << snapshot.axes[kLeftY]
      << " rx=" << snapshot.axes[kRightX]
      << " ry=" << snapshot.axes[kRightY]
      << " lt=" << snapshot.axes[kTriggerLeft]
      << " rt=" << snapshot.axes[kTriggerRight] << "} common_axes={"
      << "lx=" << gamepad.lx
      << " ly=" << gamepad.ly
      << " rx=" << gamepad.rx
      << " ry=" << gamepad.ry
      << " lt=" << gamepad.lt
      << " rt=" << gamepad.rt << "} buttons={"
      << "A=" << gamepad.A.pressed
      << " B=" << gamepad.B.pressed
      << " X=" << gamepad.X.pressed
      << " Y=" << gamepad.Y.pressed
      << " LB=" << gamepad.LB.pressed
      << " RB=" << gamepad.RB.pressed
      << " back=" << gamepad.back.pressed
      << " start=" << gamepad.start.pressed
      << " up=" << gamepad.up.pressed
      << " down=" << gamepad.down.pressed
      << " left=" << gamepad.left.pressed
      << " right=" << gamepad.right.pressed << '}';
  return out.str();
}

SdlGamepadRemote::SdlGamepadRemote(SdlGamepadRemoteOptions options)
    : options_(options) {}

SdlGamepadRemote::~SdlGamepadRemote() { stop(); }

SdlGamepadStartResult SdlGamepadRemote::start() {
  if (!sdl_initialized_) {
    if (SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK | SDL_INIT_EVENTS) != 0) {
      return {};
    }
    if (!options_.mapping_path.empty()) {
      SDL_GameControllerAddMappingsFromFile(options_.mapping_path.c_str());
    }
    sdl_initialized_ = true;
  }

  if (controller_ != nullptr) {
    return {.connected = true};
  }

  int controller_count = 0;
  for (int index = 0; index < SDL_NumJoysticks(); ++index) {
    if (!SDL_IsGameController(index)) {
      continue;
    }
    if (controller_count++ == options_.controller_index) {
      controller_ = SDL_GameControllerOpen(index);
      break;
    }
  }

  return {.connected = controller_ != nullptr};
}

std::optional<CommonGamepad> SdlGamepadRemote::read() {
  if (controller_ == nullptr) {
    return std::nullopt;
  }

  SDL_GameControllerUpdate();

  SdlGamepadSnapshot snapshot;
  snapshot.axes[kLeftX] = SDL_GameControllerGetAxis(Controller(controller_), SDL_CONTROLLER_AXIS_LEFTX);
  snapshot.axes[kLeftY] = SDL_GameControllerGetAxis(Controller(controller_), SDL_CONTROLLER_AXIS_LEFTY);
  snapshot.axes[kRightX] = SDL_GameControllerGetAxis(Controller(controller_), SDL_CONTROLLER_AXIS_RIGHTX);
  snapshot.axes[kRightY] = SDL_GameControllerGetAxis(Controller(controller_), SDL_CONTROLLER_AXIS_RIGHTY);
  snapshot.axes[kTriggerLeft] = SDL_GameControllerGetAxis(Controller(controller_), SDL_CONTROLLER_AXIS_TRIGGERLEFT);
  snapshot.axes[kTriggerRight] = SDL_GameControllerGetAxis(Controller(controller_), SDL_CONTROLLER_AXIS_TRIGGERRIGHT);

  snapshot.buttons[kA] = SDL_GameControllerGetButton(Controller(controller_), SDL_CONTROLLER_BUTTON_A) != 0;
  snapshot.buttons[kB] = SDL_GameControllerGetButton(Controller(controller_), SDL_CONTROLLER_BUTTON_B) != 0;
  snapshot.buttons[kX] = SDL_GameControllerGetButton(Controller(controller_), SDL_CONTROLLER_BUTTON_X) != 0;
  snapshot.buttons[kY] = SDL_GameControllerGetButton(Controller(controller_), SDL_CONTROLLER_BUTTON_Y) != 0;
  snapshot.buttons[kBack] = SDL_GameControllerGetButton(Controller(controller_), SDL_CONTROLLER_BUTTON_BACK) != 0;
  snapshot.buttons[kStart] = SDL_GameControllerGetButton(Controller(controller_), SDL_CONTROLLER_BUTTON_START) != 0;
  snapshot.buttons[kLeftShoulder] = SDL_GameControllerGetButton(Controller(controller_), SDL_CONTROLLER_BUTTON_LEFTSHOULDER) != 0;
  snapshot.buttons[kRightShoulder] = SDL_GameControllerGetButton(Controller(controller_), SDL_CONTROLLER_BUTTON_RIGHTSHOULDER) != 0;
  snapshot.buttons[kDpadUp] = SDL_GameControllerGetButton(Controller(controller_), SDL_CONTROLLER_BUTTON_DPAD_UP) != 0;
  snapshot.buttons[kDpadDown] = SDL_GameControllerGetButton(Controller(controller_), SDL_CONTROLLER_BUTTON_DPAD_DOWN) != 0;
  snapshot.buttons[kDpadLeft] = SDL_GameControllerGetButton(Controller(controller_), SDL_CONTROLLER_BUTTON_DPAD_LEFT) != 0;
  snapshot.buttons[kDpadRight] = SDL_GameControllerGetButton(Controller(controller_), SDL_CONTROLLER_BUTTON_DPAD_RIGHT) != 0;

  last_snapshot_ = snapshot;
  convert_sdl_snapshot_to_common_gamepad(snapshot, gamepad_);
  // const std::string debug_line = format_sdl_gamepad_debug_line(snapshot, gamepad_);
  // std::fprintf(stderr, "%s\n", debug_line.c_str());
  return gamepad_;
}

std::optional<SdlGamepadSnapshot> SdlGamepadRemote::last_snapshot() const {
  return last_snapshot_;
}

void SdlGamepadRemote::stop() noexcept {
  last_snapshot_.reset();
  if (controller_ != nullptr) {
    SDL_GameControllerClose(Controller(controller_));
    controller_ = nullptr;
  }
  if (sdl_initialized_) {
    SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK | SDL_INIT_EVENTS);
    sdl_initialized_ = false;
  }
}

}  // namespace bridge::gamepad
