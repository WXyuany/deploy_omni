#include <cassert>
#include <string>

#include "bridge/build_config.hpp"
#include "../../include/bridge/app/robot_assembly_factory.hpp"
#include "../../include/bridge/config/bridge_config.hpp"

#ifndef BRIDGE_BUILD_UNITREE_G1
#error "BRIDGE_BUILD_UNITREE_G1 must be defined for G1 assembly tests"
#endif

int main() {
  const auto loaded =
      bridge::config::load_bridge_config("../config/g1.sample.yaml");
  assert(loaded.succeeded());
  assert(loaded.config.has_value());

  auto assembled = bridge::app::create_robot_assembly(*loaded.config);
  assert(assembled.succeeded());
  assert(assembled.assembly.has_value());
  assert(assembled.assembly->adapter != nullptr);
  assert(assembled.assembly->command_ingress != nullptr);
  assert(assembled.assembly->state_egress != nullptr);
  assert(assembled.assembly->gamepad_remote == nullptr);
  assert(assembled.assembly->runtime != nullptr);
  assert(assembled.assembly->adapter->name() == std::string_view{"unitree_g1"});
  assert(assembled.assembly->runtime->config.command_topic == "bridge/command");
  assert(assembled.assembly->runtime->config.state_topic == "bridge/state");
  assert(assembled.assembly->runtime->config.network_interface.empty() ||
          assembled.assembly->runtime->config.network_interface == "robot_lan0" ||
          assembled.assembly->runtime->config.network_interface == "wlo1" ||
          assembled.assembly->runtime->config.network_interface == "wlp0s20f3");

#if BRIDGE_BUILD_GAMEPAD_SDL
  auto gamepad_config = *loaded.config;
  gamepad_config.gamepad.enabled = true;
  gamepad_config.gamepad.backend = "sdl";
  gamepad_config.gamepad.controller_index = 0;
  auto gamepad_assembled = bridge::app::create_robot_assembly(gamepad_config);
  assert(gamepad_assembled.succeeded());
  assert(gamepad_assembled.assembly.has_value());
  assert(gamepad_assembled.assembly->gamepad_remote != nullptr);
  assert(gamepad_assembled.assembly->runner_dependencies().gamepad_remote ==
         gamepad_assembled.assembly->gamepad_remote.get());
#endif

  return 0;
}
