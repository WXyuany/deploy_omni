#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "../config/bridge_config.hpp"
#include "../config/joint_map.hpp"
#include "../core/bridge_runner.hpp"
#include "../dds/dds_facade.hpp"
#include "../extensions/hooks.hpp"

namespace bridge::app {

struct RobotAssembly {
  config::JointMap joint_map;
  std::shared_ptr<dds::DdsRuntime> runtime;
  std::unique_ptr<core::RobotSdkAdapter> adapter;
  std::unique_ptr<core::BridgeCommandIngress> command_ingress;
  std::unique_ptr<core::BridgeStateEgress> state_egress;
  std::unique_ptr<extensions::ObservationHook> observation_hook;
  std::unique_ptr<core::BridgeGamepadRemoteSource> gamepad_remote;

  [[nodiscard]] core::BridgeRunnerDependencies runner_dependencies() const noexcept {
    return {
        .adapter = adapter.get(),
        .command_ingress = command_ingress.get(),
        .state_egress = state_egress.get(),
        .observation_hook = observation_hook.get(),
        .gamepad_remote = gamepad_remote.get(),
    };
  }
};

enum class RobotAssemblyCreateStatus {
  kOk,
  kInvalidConfig,
  kUnsupportedRobot,
};

struct RobotAssemblyCreateResult {
  RobotAssemblyCreateStatus status{RobotAssemblyCreateStatus::kInvalidConfig};
  std::string detail;
  std::optional<RobotAssembly> assembly;

  [[nodiscard]] bool succeeded() const noexcept {
    return status == RobotAssemblyCreateStatus::kOk && assembly.has_value();
  }
};

[[nodiscard]] RobotAssemblyCreateResult create_robot_assembly(
    const config::BridgeStartupConfig& startup_config);

[[nodiscard]] std::vector<std::string_view> registered_robot_types();

}
