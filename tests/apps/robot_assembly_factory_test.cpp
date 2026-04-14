#include <cassert>
#include <string>

#include "../../include/bridge/app/robot_assembly_factory.hpp"
#include "../../include/bridge/config/bridge_config.hpp"
#include "../../include/bridge/core/bridge_runner.hpp"

namespace {

bridge::config::BridgeConfigLoadResult LoadSampleConfig() {
#ifdef BRIDGE_REPO_ROOT
  return bridge::config::load_bridge_config(
      std::string(BRIDGE_REPO_ROOT) + "/config/kepler_k2.sample.yaml");
#else
  bridge::config::BridgeConfigLoadResult result =
      bridge::config::load_bridge_config("../config/kepler_k2.sample.yaml");
  if (result.succeeded()) {
    return result;
  }
  return bridge::config::load_bridge_config("config/kepler_k2.sample.yaml");
#endif
}

void TestCreatesKeplerAssemblyFromSampleConfig() {
  const bridge::config::BridgeConfigLoadResult loaded = LoadSampleConfig();
  assert(loaded.succeeded());
  assert(loaded.config.has_value());

  bridge::app::RobotAssemblyCreateResult assembled =
      bridge::app::create_robot_assembly(*loaded.config);
  assert(assembled.succeeded());
  assert(assembled.assembly.has_value());
  assert(assembled.assembly->adapter != nullptr);
  assert(assembled.assembly->command_ingress != nullptr);
  assert(assembled.assembly->state_egress != nullptr);
  assert(assembled.assembly->adapter->name() == std::string_view{"kepler_k2"});

  bridge::core::BridgeRunner runner(loaded.config->runner,
                                    assembled.assembly->runner_dependencies());
  assert(runner.initialize().succeeded());
  assert(runner.run().succeeded());
  assert(runner.request_stop().succeeded());
  assert(runner.shutdown().succeeded());
}

void TestRejectsUnsupportedRobotType() {
  bridge::config::BridgeStartupConfig config;
  config.robot_type = "unknown_robot";
  config.adapter.kind = "unknown_robot";

  const bridge::app::RobotAssemblyCreateResult assembled =
      bridge::app::create_robot_assembly(config);
  assert(!assembled.succeeded());
  assert(assembled.status ==
         bridge::app::RobotAssemblyCreateStatus::kUnsupportedRobot);
}

}

int main() {
  TestCreatesKeplerAssemblyFromSampleConfig();
  TestRejectsUnsupportedRobotType();
  return 0;
}
