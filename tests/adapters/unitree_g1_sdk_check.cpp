#include <array>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/b2/robot_state/robot_state_client.hpp>

#include "bridge/adapters/unitree_g1/unitree_g1_sdk.hpp"

namespace {
using bridge::adapters::unitree_g1::UnitreeG1SdkFacade;
using bridge::adapters::unitree_g1::UnitreeG1SdkResult;
volatile std::sig_atomic_t interrupted = 0;
void stop(int) { interrupted = 1; }

struct Options {
  std::string test{"all"};
  std::string interface{"eth0"};
  std::filesystem::path output{"logs/front_camera.jpg"};
};

bool report(const UnitreeG1SdkResult& result) {
  std::cout << result.operation << ": code=" << result.code
            << " detail=" << result.detail << std::endl;
  return result.succeeded();
}

void pause(int milliseconds) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
  while (!interrupted && std::chrono::steady_clock::now() < end)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

bool metadata(UnitreeG1SdkFacade& sdk, const Options&) {
  std::string form, mode, actions;
  bool passed = report(sdk.check_motion_mode(&form, &mode));
  passed = report(sdk.get_arm_action_list(&actions)) && passed;
  std::cout << "form=" << form << " mode=" << mode
            << " arm_action_list_bytes=" << actions.size() << std::endl;
  return passed;
}

bool device(UnitreeG1SdkFacade&, const Options&) {
  unitree::robot::b2::RobotStateClient client;
  client.SetTimeout(5.0F);
  client.Init();
  std::vector<unitree::robot::b2::ServiceState> services;
  const auto code = client.ServiceList(services);
  std::cout << "ServiceList code=" << code << " services=" << services.size() << std::endl;
  for (const auto& service : services)
    std::cout << "service=" << service.name << " status=" << service.status
              << " protect=" << service.protect << std::endl;
  return code == 0 && !services.empty();
}

bool audio(UnitreeG1SdkFacade& sdk, const Options&) {
  std::uint8_t original = 0;
  if (!report(sdk.get_volume(&original))) return false;
  const std::uint8_t test_volume = 30;
  bool passed = true;
  try {
    passed = report(sdk.set_volume(test_volume));
    std::uint8_t observed = 0;
    passed = report(sdk.get_volume(&observed)) && passed;
    passed = observed == test_volume && passed;
    std::cout << "volume original=" << static_cast<unsigned>(original)
              << " observed=" << static_cast<unsigned>(observed) << std::endl;
    if (passed && !interrupted) {
      passed = report(sdk.tts("G1 audio verification", 0));
      pause(4000);
    }
    if (passed && !interrupted) {
      passed = report(sdk.play_stream("deploy_omni_audio_test", "silence",
                                     std::vector<std::uint8_t>(320, 0)));
      passed = report(sdk.stop_stream("deploy_omni_audio_test")) && passed;
    }
  } catch (...) {
    report(sdk.set_volume(original));
    throw;
  }
  passed = report(sdk.set_volume(original)) && passed;
  std::uint8_t restored = 0;
  passed = report(sdk.get_volume(&restored)) && passed;
  return restored == original && passed;
}

bool led(UnitreeG1SdkFacade& sdk, const Options&) {
  const std::array<std::array<std::uint8_t, 3>, 3> colors{{
      {{255, 0, 0}}, {{0, 255, 0}}, {{0, 0, 255}}}};
  bool passed = true;
  try {
    for (const auto& color : colors) {
      if (interrupted) break;
      std::cout << "LED rgb=" << static_cast<unsigned>(color[0]) << ','
                << static_cast<unsigned>(color[1]) << ','
                << static_cast<unsigned>(color[2]) << std::endl;
      passed = report(sdk.set_rgb_led(color[0], color[1], color[2])) && passed;
      if (!passed) break;
      pause(1000);
    }
  } catch (...) {
    report(sdk.set_rgb_led(0, 0, 0));
    throw;
  }
  return report(sdk.set_rgb_led(0, 0, 0)) && passed;
}

bool camera(UnitreeG1SdkFacade& sdk, const Options& options) {
  std::vector<std::uint8_t> jpeg;
  if (!report(sdk.get_front_camera_jpeg(&jpeg))) return false;
  const bool signature = jpeg.size() >= 4 && jpeg[0] == 0xFF && jpeg[1] == 0xD8 &&
                         jpeg[jpeg.size() - 2] == 0xFF && jpeg.back() == 0xD9;
  std::cout << "front_camera_jpeg_bytes=" << jpeg.size()
            << " jpeg_signature=" << (signature ? "yes" : "no") << std::endl;
  if (!signature) return false;
  if (options.output.has_parent_path())
    std::filesystem::create_directories(options.output.parent_path());
  std::ofstream output(options.output, std::ios::binary);
  output.write(reinterpret_cast<const char*>(jpeg.data()), jpeg.size());
  output.close();
  if (!output) throw std::runtime_error("Cannot save JPEG: " + options.output.string());
  std::cout << "jpeg_saved=" << options.output << std::endl;
  return true;
}

using Check = bool (*)(UnitreeG1SdkFacade&, const Options&);
struct Test { const char* name; Check run; };
const std::array<Test, 5> tests{{{"metadata", metadata}, {"device", device},
                               {"audio", audio}, {"led", led}, {"camera", camera}}};

void help() {
  std::cout << "Usage: unitree_g1_sdk_check [--test all|metadata|device|audio|led|camera]\n"
               "  [--interface eth0] [--output logs/front_camera.jpg]\n"
               "No locomotion. Audio changes/restores volume; LED cycles RGB then off.\n"
               "Exit: 0=selected API/data checks passed; 1=failed; 2=invalid args; 130=interrupted.\n";
}

Options parse(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (i + 1 == argc) throw std::invalid_argument("Missing value: " + key);
    const std::string value = argv[++i];
    if (value.empty()) throw std::invalid_argument("Empty value: " + key);
    if (key == "--test") options.test = value;
    else if (key == "--interface") options.interface = value;
    else if (key == "--output") options.output = value;
    else throw std::invalid_argument("Unknown option: " + key);
  }
  bool known = options.test == "all";
  for (const auto& test : tests) known = known || options.test == test.name;
  if (!known) throw std::invalid_argument("Unknown test: " + options.test);
  return options;
}
}  // namespace

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == "--help") { help(); return 0; }
  Options options;
  try { options = parse(argc, argv); }
  catch (const std::invalid_argument& error) {
    std::cerr << error.what() << '\n'; help(); return 2;
  }
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  try {
    unitree::robot::ChannelFactory::Instance()->Init(0, options.interface);
    UnitreeG1SdkFacade sdk;
    if (!report(sdk.initialize())) return 1;
    bool passed = true;
    for (const auto& test : tests) {
      if (interrupted) break;
      if (options.test != "all" && options.test != test.name) continue;
      const bool ok = test.run(sdk, options);
      std::cout << test.name << ": " << (ok && !interrupted ? "CHECK_OK" : "FAIL") << std::endl;
      passed = ok && passed;
    }
    passed = report(sdk.shutdown()) && passed;
    return interrupted ? 130 : passed ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << std::endl;
    return 1;
  }
}
