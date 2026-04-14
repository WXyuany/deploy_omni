#include <string>

#include "../../include/bridge/app/robot_assembly_factory.hpp"
#include "../../include/bridge/config/bridge_config.hpp"
#include "../../include/bridge/core/bridge_runner.hpp"

namespace {

[[nodiscard]] int to_exit_code(const bridge::core::RunnerCallResult& result,
                               const int success_code,
                               const int failure_code) {
  return result.succeeded() ? success_code : failure_code;
}

}

int main(int argc, char** argv) {
  const std::string config_path =
      argc > 1 ? std::string(argv[1]) : std::string("config/kepler_k2.sample.yaml");
  const bridge::config::BridgeConfigLoadResult loaded =
      bridge::config::load_bridge_config(config_path);
  if (!loaded.succeeded()) {
    return loaded.status == bridge::config::BridgeConfigLoadStatus::kIoError ? 1 : 2;
  }

  bridge::app::RobotAssemblyCreateResult assembly =
      bridge::app::create_robot_assembly(*loaded.config);
  if (!assembly.succeeded()) {
    return assembly.status == bridge::app::RobotAssemblyCreateStatus::kUnsupportedRobot
               ? 3
               : 4;
  }

  bridge::core::BridgeRunner runner(loaded.config->runner,
                                    assembly.assembly->runner_dependencies());

  const bridge::core::RunnerCallResult initialize = runner.initialize();
  if (!initialize.succeeded()) {
    return to_exit_code(initialize, 0, 5);
  }

  const bridge::core::RunnerCallResult run = runner.run();
  const bridge::core::RunnerCallResult stop_request = runner.request_stop();
  const bridge::core::RunnerCallResult shutdown = runner.shutdown();

  if (!run.succeeded()) {
    return to_exit_code(run, 0, 6);
  }
  if (!stop_request.succeeded()) {
    return to_exit_code(stop_request, 0, 7);
  }
  return to_exit_code(shutdown, 0, 8);
}
