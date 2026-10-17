#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>

#include "../../include/bridge/adapters/unitree_g1/unitree_g1_sdk.hpp"

namespace {

using bridge::adapters::unitree_g1::UnitreeG1SdkFacade;
using bridge::adapters::unitree_g1::UnitreeG1SdkResult;

bool report(const UnitreeG1SdkResult& result) {
  std::cout << result.operation << ": code=" << result.code
            << " detail=" << result.detail << '\n';
  return result.succeeded();
}

}  // namespace

int main() {
  unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");

  UnitreeG1SdkFacade sdk;
  bool passed = report(sdk.initialize());

  std::string form;
  std::string mode;
  passed = report(sdk.check_motion_mode(&form, &mode)) && passed;

  bool silent = false;
  passed = report(sdk.get_silent(&silent)) && passed;
  const bool original_silent = silent;
  passed = report(sdk.set_silent(!original_silent)) && passed;
  passed = report(sdk.set_silent(original_silent)) && passed;

  std::string action_list;
  passed = report(sdk.get_arm_action_list(&action_list)) && passed;
  std::cout << "arm_action_list_bytes=" << action_list.size() << '\n';

  std::uint8_t volume = 0;
  passed = report(sdk.get_volume(&volume)) && passed;
  const std::uint8_t original_volume = volume;
  const std::uint8_t test_volume = original_volume == 40U ? 41U : 40U;
  passed = report(sdk.set_volume(test_volume)) && passed;
  std::uint8_t observed_volume = 0;
  passed = report(sdk.get_volume(&observed_volume)) && passed;
  passed = (observed_volume == test_volume) && passed;
  passed = report(sdk.set_volume(original_volume)) && passed;

  passed = report(sdk.tts("G1 SDK interface test", 0)) && passed;
  passed = report(sdk.set_rgb_led(0U, 255U, 0U)) && passed;
  passed = report(sdk.set_rgb_led(0U, 0U, 0U)) && passed;

  const std::vector<std::uint8_t> silence(320U, 0U);
  passed = report(sdk.play_stream("deploy_omni_audio_test", "silence", silence)) && passed;
  passed = report(sdk.stop_stream("deploy_omni_audio_test")) && passed;

  std::vector<std::uint8_t> jpeg;
  passed = report(sdk.get_front_camera_jpeg(&jpeg)) && passed;
  const bool jpeg_signature = jpeg.size() >= 2U && jpeg[0] == 0xFFU && jpeg[1] == 0xD8U;
  std::cout << "front_camera_jpeg_bytes=" << jpeg.size()
            << " jpeg_signature=" << (jpeg_signature ? "yes" : "no") << '\n';
  passed = jpeg_signature && passed;

  passed = report(sdk.shutdown()) && passed;
  return passed ? 0 : 1;
}
