#include <cassert>

#include "../../include/bridge/config/bridge_config.hpp"

int main() {
  const auto result = bridge::config::load_bridge_config("../config/g1.sample.yaml");
  assert(result.succeeded());
  assert(result.config.has_value());

  const auto& config = *result.config;
  assert(config.source_path == "../config/g1.sample.yaml");
  assert(config.robot_type == "unitree_g1");
  assert(config.dds.topics.command == "bridge/command");
  assert(config.dds.topics.state == "bridge/state");
  assert(config.adapter.kind == "unitree_g1");
  assert(config.adapter.robot_model == "g1");
  assert(config.adapter.command_mode == "raw_low_level");
  assert(config.adapter.connection.transport == "dds");
  assert(config.adapter.connection.interface_hint == "wlo1");
  assert(config.adapter.joint_names.size() == 29U);
  assert(config.joint_map_entries.size() == 29U);

  return 0;
}
