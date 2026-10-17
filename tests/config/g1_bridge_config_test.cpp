#include <cassert>

#include "../../include/bridge/config/bridge_config.hpp"

int main() {
  const auto result = bridge::config::load_bridge_config("../config/g1.sample.yaml");
  assert(result.succeeded());
  assert(result.config.has_value());

  const auto& config = *result.config;
  assert(config.source_path == "../config/g1.sample.yaml");
  assert(config.robot_type == "unitree_g1");
  assert(config.runner.loop_period_ns == 1'000'000U);
  assert(config.dds.topics.command == "bridge/rt/lowcmd");
  assert(config.dds.topics.state == "bridge/rt/lowstate");
  assert(config.adapter.kind == "unitree_g1");
  assert(config.adapter.robot_model == "g1");
  assert(config.adapter.command_mode == "raw_low_level");
  assert(config.adapter.connection.transport == "dds");
  assert(config.adapter.connection.interface_hint == "eth0");
  assert(config.adapter.topics.command == "custom/mujoco/rt/lowcmd");
  assert(config.adapter.topics.state == "custom/mujoco/rt/lowstate");
  assert(config.adapter.topics.imu == "rt/secondary_imu");
  assert(!config.gamepad.enabled);
  assert(config.gamepad.backend == "sdl");
  assert(config.gamepad.controller_index == 0);
  assert(config.gamepad.mapping_path == "config/gamecontrollerdb.txt");
  assert(config.runner.gamepad_poll_interval_ns == 5'000'000U);
  assert(config.runner.gamepad_stale_timeout_ns == 100'000'000U);
  assert(config.adapter.joint_names.size() == 29U);
  assert(config.joint_map_entries.size() == 29U);

  return 0;
}
