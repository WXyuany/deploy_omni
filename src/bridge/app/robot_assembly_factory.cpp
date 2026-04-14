#include "../../../include/bridge/app/robot_assembly_factory.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "../../../include/bridge/adapters/kepler_k2/kepler_k2_adapter.hpp"
#include "../../../include/bridge/dds/dds_factory.hpp"

namespace bridge::app {
namespace {

[[nodiscard]] adapters::kepler_k2::KeplerK2AdapterConfig
to_kepler_k2_adapter_config(const config::BridgeStartupConfig& startup_config) {
  adapters::kepler_k2::KeplerK2AdapterConfig config;
  config.session.robot_id = startup_config.adapter.robot_name;
  config.session.transport_address = startup_config.adapter.connection.transport +
                                     ":" +
                                     startup_config.adapter.connection.interface_hint;
  config.session.control_profile = startup_config.adapter.command_mode;
  config.session.initialize_timeout_ns =
      startup_config.adapter.startup.read_initial_state_timeout_ns;
  config.session.read_timeout_ns = startup_config.runner.loop_period_ns;
  config.session.write_timeout_ns = startup_config.runner.loop_period_ns;
  config.safety.safe_command_supported =
      startup_config.runner.startup_idle_policy != core::StartupIdlePolicy::kNoCommandWrite &&
      startup_config.adapter.safety.allow_safe_command_during_shutdown &&
      startup_config.adapter.safety.safe_command_profile == "zero_torque";
  config.safety.require_initial_state_before_motion =
      startup_config.adapter.startup.require_healthy_hardware;
  config.safety.allow_command_when_imu_missing = true;
  config.joint_names = startup_config.adapter.joint_names;
  return config;
}

[[nodiscard]] RobotAssemblyCreateResult create_kepler_k2_assembly(
    const config::BridgeStartupConfig& startup_config) {
  const config::JointMapBuildResult joint_map_result = config::JointMap::build(
      startup_config.joint_map_entries, startup_config.joint_map_validation);
  if (!joint_map_result.succeeded() || !joint_map_result.joint_map.has_value()) {
    return {
        .status = RobotAssemblyCreateStatus::kInvalidConfig,
        .detail = "joint-map build failed during kepler_k2 assembly",
        .assembly = std::nullopt,
    };
  }

  RobotAssembly assembly{
      .joint_map = std::move(*joint_map_result.joint_map),
      .adapter = std::make_unique<adapters::kepler_k2::KeplerK2Adapter>(
          to_kepler_k2_adapter_config(startup_config)),
      .command_ingress = nullptr,
      .state_egress = nullptr,
  };

  dds::DdsAssembly dds_assembly =
      dds::create_dds_assembly(startup_config, assembly.joint_map);
  assembly.runtime = dds_assembly.runtime;
  assembly.command_ingress = std::move(dds_assembly.command_ingress);
  assembly.state_egress = std::move(dds_assembly.state_egress);

  return {
      .status = RobotAssemblyCreateStatus::kOk,
      .detail = "assembled kepler_k2 robot runtime",
      .assembly = std::move(assembly),
  };
}

struct RobotFactoryEntry {
  std::string_view robot_type;
  RobotAssemblyCreateResult (*create)(const config::BridgeStartupConfig&);
};

[[nodiscard]] const std::vector<RobotFactoryEntry>& factory_entries() {
  static const std::vector<RobotFactoryEntry> entries{{
      .robot_type = "kepler_k2",
      .create = &create_kepler_k2_assembly,
  }};
  return entries;
}

}

RobotAssemblyCreateResult create_robot_assembly(
    const config::BridgeStartupConfig& startup_config) {
  for (const RobotFactoryEntry& entry : factory_entries()) {
    if (entry.robot_type == startup_config.robot_type) {
      return entry.create(startup_config);
    }
  }

  return {
      .status = RobotAssemblyCreateStatus::kUnsupportedRobot,
      .detail = "no registered robot factory for type '" +
                startup_config.robot_type + "'",
      .assembly = std::nullopt,
  };
}

std::vector<std::string_view> registered_robot_types() {
  std::vector<std::string_view> result;
  result.reserve(factory_entries().size());
  for (const RobotFactoryEntry& entry : factory_entries()) {
    result.push_back(entry.robot_type);
  }
  return result;
}

}
