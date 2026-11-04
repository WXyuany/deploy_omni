#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace bridge::adapters::unitree_g1 {

inline constexpr std::int32_t kUnitreeG1SdkNotInitialized = -10001;
inline constexpr std::int32_t kUnitreeG1SdkInvalidArgument = -10002;

enum class UnitreeG1SdkCapability {
  kLocomotion,
  kArmAction,
  kAudio,
  kMotionSwitcher,
  kFrontCamera,
};

struct UnitreeG1SdkResult {
  std::int32_t code{0};
  std::string operation;
  std::string detail;

  [[nodiscard]] bool succeeded() const noexcept { return code == 0; }
};

class UnitreeG1SdkFacade final {
 public:
  UnitreeG1SdkFacade();
  ~UnitreeG1SdkFacade();

  UnitreeG1SdkFacade(const UnitreeG1SdkFacade&) = delete;
  UnitreeG1SdkFacade& operator=(const UnitreeG1SdkFacade&) = delete;
  UnitreeG1SdkFacade(UnitreeG1SdkFacade&&) noexcept;
  UnitreeG1SdkFacade& operator=(UnitreeG1SdkFacade&&) noexcept;

  [[nodiscard]] UnitreeG1SdkResult initialize();
  [[nodiscard]] UnitreeG1SdkResult shutdown();
  [[nodiscard]] bool initialized() const noexcept;
  [[nodiscard]] const std::vector<UnitreeG1SdkCapability>& capabilities() const noexcept;

  [[nodiscard]] UnitreeG1SdkResult get_fsm_id(int* value);
  [[nodiscard]] UnitreeG1SdkResult get_fsm_mode(int* value);
  [[nodiscard]] UnitreeG1SdkResult get_balance_mode(int* value);
  [[nodiscard]] UnitreeG1SdkResult get_swing_height(float* value);
  [[nodiscard]] UnitreeG1SdkResult get_stand_height(float* value);
  [[nodiscard]] UnitreeG1SdkResult get_phase(std::vector<float>* value);
  [[nodiscard]] UnitreeG1SdkResult set_fsm_id(int value);
  [[nodiscard]] UnitreeG1SdkResult set_balance_mode(int value);
  [[nodiscard]] UnitreeG1SdkResult set_swing_height(float value);
  [[nodiscard]] UnitreeG1SdkResult set_stand_height(float value);
  [[nodiscard]] UnitreeG1SdkResult set_velocity(float vx, float vy, float omega,
                                                float duration = 1.0F);
  [[nodiscard]] UnitreeG1SdkResult set_arm_task(int task_id);
  [[nodiscard]] UnitreeG1SdkResult set_speed_mode(int speed_mode);
  [[nodiscard]] UnitreeG1SdkResult damp();
  [[nodiscard]] UnitreeG1SdkResult start();
  [[nodiscard]] UnitreeG1SdkResult squat();
  [[nodiscard]] UnitreeG1SdkResult sit();
  [[nodiscard]] UnitreeG1SdkResult stand_up();
  [[nodiscard]] UnitreeG1SdkResult high_stand();
  [[nodiscard]] UnitreeG1SdkResult low_stand();
  [[nodiscard]] UnitreeG1SdkResult zero_torque();
  [[nodiscard]] UnitreeG1SdkResult stop_move();
  [[nodiscard]] UnitreeG1SdkResult move(float vx, float vy, float omega,
                                         bool continuous = false);
  [[nodiscard]] UnitreeG1SdkResult balance_stand();
  [[nodiscard]] UnitreeG1SdkResult continuous_gait(bool enabled);
  [[nodiscard]] UnitreeG1SdkResult switch_move_mode(bool continuous);
  [[nodiscard]] UnitreeG1SdkResult wave_hand(bool turn = false);
  [[nodiscard]] UnitreeG1SdkResult shake_hand(int stage = -1);

  [[nodiscard]] UnitreeG1SdkResult execute_arm_action(std::int32_t action_id);
  [[nodiscard]] UnitreeG1SdkResult get_arm_action_list(std::string* value);

  [[nodiscard]] UnitreeG1SdkResult tts(const std::string& text,
                                       std::int32_t speaker_id);
  [[nodiscard]] UnitreeG1SdkResult get_volume(std::uint8_t* value);
  [[nodiscard]] UnitreeG1SdkResult set_volume(std::uint8_t value);
  [[nodiscard]] UnitreeG1SdkResult play_stream(
      const std::string& app_name, const std::string& stream_id,
      const std::vector<std::uint8_t>& pcm_data);
  [[nodiscard]] UnitreeG1SdkResult stop_stream(const std::string& app_name);
  [[nodiscard]] UnitreeG1SdkResult set_rgb_led(std::uint8_t red,
                                               std::uint8_t green,
                                               std::uint8_t blue);

  [[nodiscard]] UnitreeG1SdkResult check_motion_mode(std::string* form,
                                                      std::string* name);
  [[nodiscard]] UnitreeG1SdkResult select_motion_mode(const std::string& name);
  [[nodiscard]] UnitreeG1SdkResult release_motion_mode();
  [[nodiscard]] UnitreeG1SdkResult set_silent(bool enabled);
  [[nodiscard]] UnitreeG1SdkResult get_silent(bool* enabled);
  [[nodiscard]] UnitreeG1SdkResult get_front_camera_jpeg(std::vector<std::uint8_t>* value);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bridge::adapters::unitree_g1
