#include <chrono>
#include <cstdio>
#include <utility>
#include <string>
#include <thread>
#include <vector>

#include "../../include/bridge/adapters/agibot_x2/agibot_x2_adapter.hpp"

int main() {
  using namespace bridge::adapters::agibot_x2;
  AgibotX2AdapterConfig config;
  config.joint_names.reserve(kAgibotX2JointCount);
  const char* groups[] = {"head", "waist", "arm", "leg"};
  for (const char* group : groups) {
    (void)group;
  }
  for (std::size_t i = 0; i < kAgibotX2JointCount; ++i) {
    config.joint_names.push_back("joint_" + std::to_string(i));
  }
  AgibotX2Adapter adapter(std::move(config));
  const auto initialized = adapter.initialize();
  if (!initialized.succeeded()) {
    std::fprintf(stderr, "x2 state probe initialize: %s\n", initialized.detail.c_str());
    return 2;
  }
  for (int i = 0; i < 100; ++i) {
    const auto state = adapter.read_state();
    if (state.state.has_value()) {
      std::printf("received %zu X2 joints\n", state.state->joints.position.size());
      (void)adapter.shutdown();
      return state.state->valid ? 0 : 3;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  (void)adapter.shutdown();
  std::fprintf(stderr, "x2 state probe timed out waiting for four joint groups\n");
  return 4;
}
