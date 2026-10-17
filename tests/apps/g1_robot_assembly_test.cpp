#include <cassert>
#include <string>

#include "bridge/build_config.hpp"
#include "../../include/bridge/adapters/unitree_g1/unitree_g1_adapter.hpp"
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
  const auto* g1_adapter = dynamic_cast<const bridge::adapters::unitree_g1::UnitreeG1Adapter*>(
      assembled.assembly->adapter.get());
  assert(g1_adapter != nullptr);
  assert(g1_adapter->config().session.command_topic == "custom/mujoco/rt/lowcmd");
  assert(g1_adapter->config().session.state_topic == "custom/mujoco/rt/lowstate");
  assert(g1_adapter->config().session.imu_topic == "rt/secondary_imu");
  assert(assembled.assembly->runtime->config.command_topic == "bridge/rt/lowcmd");
  assert(assembled.assembly->runtime->config.state_topic == "bridge/rt/lowstate");
  assert(assembled.assembly->runtime->config.network_interface.empty() ||
          assembled.assembly->runtime->config.network_interface == "eth0");

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
