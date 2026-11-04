#include "../../../../include/bridge/adapters/unitree_g1/unitree_g1_sdk.hpp"

#include <mutex>
#include <utility>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/g1/arm/g1_arm_action_client.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/go2/video/video_client.hpp>

namespace bridge::adapters::unitree_g1 {
namespace {

template <typename Callable>
UnitreeG1SdkResult invoke(const char* operation, Callable&& callable) {
  const std::int32_t code = callable();
  return {.code = code,
          .operation = operation,
          .detail = code == 0 ? "ok" : "Unitree SDK call failed"};
}

UnitreeG1SdkResult not_initialized(const char* operation) {
  return {.code = kUnitreeG1SdkNotInitialized,
          .operation = operation,
          .detail = "G1 SDK facade is not initialized"};
}

UnitreeG1SdkResult invalid_argument(const char* operation, const char* detail) {
  return {.code = kUnitreeG1SdkInvalidArgument, .operation = operation, .detail = detail};
}

}  // namespace

class UnitreeG1SdkFacade::Impl {
 public:
  std::unique_ptr<unitree::robot::g1::LocoClient> loco;
  std::unique_ptr<unitree::robot::g1::G1ArmActionClient> arm;
  std::unique_ptr<unitree::robot::g1::AudioClient> audio;
  std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> motion_switcher;
  std::unique_ptr<unitree::robot::go2::VideoClient> front_camera;
  bool initialized{false};
  const std::vector<UnitreeG1SdkCapability> capabilities{
      UnitreeG1SdkCapability::kLocomotion,
      UnitreeG1SdkCapability::kArmAction,
      UnitreeG1SdkCapability::kAudio,
      UnitreeG1SdkCapability::kMotionSwitcher,
      UnitreeG1SdkCapability::kFrontCamera,
  };

  template <typename Callable>
  UnitreeG1SdkResult call(const char* operation, Callable&& callable) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!initialized) {
      return not_initialized(operation);
    }
    return invoke(operation, std::forward<Callable>(callable));
  }

  template <typename Value, typename Callable>
  UnitreeG1SdkResult call_output(const char* operation, Value* output,
                                 Callable&& callable) {
    if (output == nullptr) {
      return invalid_argument(operation, "output is null");
    }
    return call(operation, [&] { return callable(*output); });
  }

  mutable std::mutex mutex;
};

UnitreeG1SdkFacade::UnitreeG1SdkFacade() : impl_(std::make_unique<Impl>()) {}
UnitreeG1SdkFacade::~UnitreeG1SdkFacade() = default;
UnitreeG1SdkFacade::UnitreeG1SdkFacade(UnitreeG1SdkFacade&&) noexcept = default;
UnitreeG1SdkFacade& UnitreeG1SdkFacade::operator=(UnitreeG1SdkFacade&&) noexcept = default;

UnitreeG1SdkResult UnitreeG1SdkFacade::initialize() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->initialized) {
    return {.code = 0, .operation = "initialize", .detail = "already initialized"};
  }
  impl_->loco = std::make_unique<unitree::robot::g1::LocoClient>();
  impl_->arm = std::make_unique<unitree::robot::g1::G1ArmActionClient>();
  impl_->audio = std::make_unique<unitree::robot::g1::AudioClient>();
  impl_->motion_switcher =
      std::make_unique<unitree::robot::b2::MotionSwitcherClient>();
  impl_->loco->SetTimeout(5.0F);
  impl_->arm->SetTimeout(5.0F);
  impl_->audio->SetTimeout(5.0F);
  impl_->motion_switcher->SetTimeout(5.0F);
  impl_->front_camera = std::make_unique<unitree::robot::go2::VideoClient>();
  impl_->front_camera->SetTimeout(5.0F);
  impl_->loco->Init();
  impl_->arm->Init();
  impl_->audio->Init();
  impl_->motion_switcher->Init();
  impl_->front_camera->Init();
  impl_->initialized = true;
  return {.code = 0, .operation = "initialize", .detail = "initialized"};
}

UnitreeG1SdkResult UnitreeG1SdkFacade::shutdown() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->initialized = false;
  impl_->motion_switcher.reset();
  impl_->front_camera.reset();
  impl_->audio.reset();
  impl_->arm.reset();
  impl_->loco.reset();
  return {.code = 0, .operation = "shutdown", .detail = "stopped"};
}

bool UnitreeG1SdkFacade::initialized() const noexcept {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->initialized;
}
const std::vector<UnitreeG1SdkCapability>& UnitreeG1SdkFacade::capabilities() const noexcept {
  return impl_->capabilities;
}

UnitreeG1SdkResult UnitreeG1SdkFacade::get_fsm_id(int* value) {
  return impl_->call_output("get_fsm_id", value,
                            [&](int& output) { return impl_->loco->GetFsmId(output); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::get_fsm_mode(int* value) {
  return impl_->call_output("get_fsm_mode", value,
                            [&](int& output) { return impl_->loco->GetFsmMode(output); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::get_balance_mode(int* value) {
  return impl_->call_output("get_balance_mode", value,
                            [&](int& output) { return impl_->loco->GetBalanceMode(output); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::get_swing_height(float* value) {
  return impl_->call_output("get_swing_height", value,
                            [&](float& output) { return impl_->loco->GetSwingHeight(output); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::get_stand_height(float* value) {
  return impl_->call_output("get_stand_height", value,
                            [&](float& output) { return impl_->loco->GetStandHeight(output); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::get_phase(std::vector<float>* value) {
  return impl_->call_output("get_phase", value,
                            [&](std::vector<float>& output) { return impl_->loco->GetPhase(output); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::set_fsm_id(int value) {
  return impl_->call("set_fsm_id", [&] { return impl_->loco->SetFsmId(value); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::set_balance_mode(int value) {
  return impl_->call("set_balance_mode", [&] { return impl_->loco->SetBalanceMode(value); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::set_swing_height(float value) {
  return impl_->call("set_swing_height", [&] { return impl_->loco->SetSwingHeight(value); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::set_stand_height(float value) {
  return impl_->call("set_stand_height", [&] { return impl_->loco->SetStandHeight(value); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::set_velocity(float vx, float vy, float omega, float duration) {
  return impl_->call("set_velocity", [&] { return impl_->loco->SetVelocity(vx, vy, omega, duration); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::set_arm_task(int task_id) {
  return impl_->call("set_arm_task", [&] { return impl_->loco->SetTaskId(task_id); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::set_speed_mode(int speed_mode) {
  return impl_->call("set_speed_mode", [&] { return impl_->loco->SetSpeedMode(speed_mode); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::damp() { return impl_->call("damp", [&] { return impl_->loco->Damp(); }); }
UnitreeG1SdkResult UnitreeG1SdkFacade::start() { return impl_->call("start", [&] { return impl_->loco->Start(); }); }
UnitreeG1SdkResult UnitreeG1SdkFacade::squat() { return impl_->call("squat", [&] { return impl_->loco->Squat(); }); }
UnitreeG1SdkResult UnitreeG1SdkFacade::sit() { return impl_->call("sit", [&] { return impl_->loco->Sit(); }); }
UnitreeG1SdkResult UnitreeG1SdkFacade::stand_up() { return impl_->call("stand_up", [&] { return impl_->loco->StandUp(); }); }
UnitreeG1SdkResult UnitreeG1SdkFacade::high_stand() { return impl_->call("high_stand", [&] { return impl_->loco->HighStand(); }); }
UnitreeG1SdkResult UnitreeG1SdkFacade::low_stand() { return impl_->call("low_stand", [&] { return impl_->loco->LowStand(); }); }
UnitreeG1SdkResult UnitreeG1SdkFacade::zero_torque() { return impl_->call("zero_torque", [&] { return impl_->loco->ZeroTorque(); }); }
UnitreeG1SdkResult UnitreeG1SdkFacade::stop_move() { return impl_->call("stop_move", [&] { return impl_->loco->StopMove(); }); }
UnitreeG1SdkResult UnitreeG1SdkFacade::move(float vx, float vy, float omega, bool continuous) {
  return impl_->call("move", [&] { return impl_->loco->Move(vx, vy, omega, continuous); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::balance_stand() { return impl_->call("balance_stand", [&] { return impl_->loco->BalanceStand(); }); }
UnitreeG1SdkResult UnitreeG1SdkFacade::continuous_gait(bool enabled) { return impl_->call("continuous_gait", [&] { return impl_->loco->ContinuousGait(enabled); }); }
UnitreeG1SdkResult UnitreeG1SdkFacade::switch_move_mode(bool continuous) { return impl_->call("switch_move_mode", [&] { return impl_->loco->SwitchMoveMode(continuous); }); }
UnitreeG1SdkResult UnitreeG1SdkFacade::wave_hand(bool turn) { return impl_->call("wave_hand", [&] { return impl_->loco->WaveHand(turn); }); }
UnitreeG1SdkResult UnitreeG1SdkFacade::shake_hand(int stage) { return impl_->call("shake_hand", [&] { return impl_->loco->ShakeHand(stage); }); }

UnitreeG1SdkResult UnitreeG1SdkFacade::execute_arm_action(std::int32_t action_id) {
  return impl_->call("execute_arm_action", [&] { return impl_->arm->ExecuteAction(action_id); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::get_arm_action_list(std::string* value) {
  return impl_->call_output("get_arm_action_list", value,
                            [&](std::string& output) { return impl_->arm->GetActionList(output); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::tts(const std::string& text, std::int32_t speaker_id) {
  return impl_->call("tts", [&] { return impl_->audio->TtsMaker(text, speaker_id); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::get_volume(std::uint8_t* value) {
  return impl_->call_output("get_volume", value,
                            [&](std::uint8_t& output) { return impl_->audio->GetVolume(output); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::set_volume(std::uint8_t value) {
  return impl_->call("set_volume", [&] { return impl_->audio->SetVolume(value); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::play_stream(const std::string& app_name, const std::string& stream_id,
                                                   const std::vector<std::uint8_t>& pcm_data) {
  return impl_->call("play_stream", [&] { return impl_->audio->PlayStream(app_name, stream_id, pcm_data); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::stop_stream(const std::string& app_name) {
  return impl_->call("stop_stream", [&] { return impl_->audio->PlayStop(app_name); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::set_rgb_led(std::uint8_t red, std::uint8_t green, std::uint8_t blue) {
  return impl_->call("set_rgb_led", [&] { return impl_->audio->LedControl(red, green, blue); });
}

UnitreeG1SdkResult UnitreeG1SdkFacade::check_motion_mode(std::string* form, std::string* name) {
  if (form == nullptr || name == nullptr) {
    return invalid_argument("check_motion_mode", "output is null");
  }
  return impl_->call("check_motion_mode", [&] { return impl_->motion_switcher->CheckMode(*form, *name); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::select_motion_mode(const std::string& name) {
  return impl_->call("select_motion_mode", [&] { return impl_->motion_switcher->SelectMode(name); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::release_motion_mode() {
  return impl_->call("release_motion_mode", [&] { return impl_->motion_switcher->ReleaseMode(); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::set_silent(bool enabled) {
  return impl_->call("set_silent", [&] { return impl_->motion_switcher->SetSilent(enabled); });
}
UnitreeG1SdkResult UnitreeG1SdkFacade::get_silent(bool* enabled) {
  return impl_->call_output("get_silent", enabled,
                            [&](bool& output) { return impl_->motion_switcher->GetSilent(output); });
}

UnitreeG1SdkResult UnitreeG1SdkFacade::get_front_camera_jpeg(
    std::vector<std::uint8_t>* value) {
  return impl_->call_output("get_front_camera_jpeg", value,
                            [&](std::vector<std::uint8_t>& output) {
                              return impl_->front_camera->GetImageSample(output);
                            });
}

}  // namespace bridge::adapters::unitree_g1
