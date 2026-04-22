#include "../../../include/bridge/app/robot_assembly_factory.hpp"

#include <algorithm>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#if defined(BRIDGE_BUILD_KEPLER_K2)
#include "../../../include/bridge/adapters/kepler_k2/kepler_k2_adapter.hpp"
#endif
#if defined(BRIDGE_BUILD_PM01)
#include "../../../include/bridge/adapters/pm01/pm01_adapter.hpp"
#endif
#include "../../../include/bridge/dds/dds_factory.hpp"

namespace bridge::app {
namespace {

[[nodiscard]] std::size_t configured_hardware_width(
    const std::vector<config::JointMapEntry>& entries) {
  std::size_t width = 0;
  for (const config::JointMapEntry& entry : entries) {
    width = std::max(width, entry.hardware_index + 1);
  }
  return width;
}

#if defined(BRIDGE_BUILD_KEPLER_K2)
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
  config.hardware_slot_count =
      configured_hardware_width(startup_config.joint_map_entries);
  return config;
}
#endif

#if defined(BRIDGE_BUILD_PM01)
struct Pm01AdapterConfigBuildResult {
  std::optional<adapters::pm01::Pm01AdapterConfig> config;
  std::string detail;

  [[nodiscard]] bool succeeded() const noexcept {
    return config.has_value();
  }
};

[[nodiscard]] Pm01AdapterConfigBuildResult to_pm01_adapter_config(
    const config::BridgeStartupConfig& startup_config) {
  if (startup_config.adapter.joint_names.size() !=
      adapters::pm01::kPm01JointCount) {
    return {
        .config = std::nullopt,
        .detail = "pm01 requires exactly 24 enabled joint names in adapter config",
    };
  }

  adapters::pm01::Pm01AdapterConfig config;
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
  config.joint_names = startup_config.adapter.joint_names;
  config.dds_joint_indices.reserve(config.joint_names.size());

  for (const std::string& joint_name : config.joint_names) {
    const auto entry_it = std::find_if(
        startup_config.joint_map_entries.begin(),
        startup_config.joint_map_entries.end(),
        [&joint_name](const config::JointMapEntry& entry) {
          return entry.enabled && entry.hardware_name.has_value() &&
                 *entry.hardware_name == joint_name;
        });
    if (entry_it == startup_config.joint_map_entries.end()) {
      return {
          .config = std::nullopt,
          .detail = "pm01 joint mapping is missing enabled hardware_name '" +
                    joint_name + "'",
      };
    }
    if (entry_it->hardware_index >= adapters::pm01::kPm01DdsMotorCount) {
      return {
          .config = std::nullopt,
          .detail = "pm01 hardware_index out of DDS range for joint '" +
                    joint_name + "'",
      };
    }
    config.dds_joint_indices.push_back(
        static_cast<std::int64_t>(entry_it->hardware_index));
  }

  return {
      .config = std::move(config),
      .detail = "built pm01 adapter config from startup config",
  };
}
#endif

#if defined(BRIDGE_BUILD_KEPLER_K2)
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
#endif

#if defined(BRIDGE_BUILD_PM01)
[[nodiscard]] RobotAssemblyCreateResult create_pm01_assembly(
    const config::BridgeStartupConfig& startup_config) {
  const config::JointMapBuildResult joint_map_result = config::JointMap::build(
      startup_config.joint_map_entries, startup_config.joint_map_validation);
  if (!joint_map_result.succeeded() || !joint_map_result.joint_map.has_value()) {
    return {
        .status = RobotAssemblyCreateStatus::kInvalidConfig,
        .detail = "joint-map build failed during pm01 assembly",
        .assembly = std::nullopt,
    };
  }

  Pm01AdapterConfigBuildResult adapter_config_result =
      to_pm01_adapter_config(startup_config);
  if (!adapter_config_result.succeeded()) {
    return {
        .status = RobotAssemblyCreateStatus::kInvalidConfig,
        .detail = adapter_config_result.detail,
        .assembly = std::nullopt,
    };
  }

  RobotAssembly assembly{
      .joint_map = std::move(*joint_map_result.joint_map),
      .adapter = std::make_unique<adapters::pm01::Pm01Adapter>(
          std::move(*adapter_config_result.config)),
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
      .detail = "assembled pm01 robot runtime",
      .assembly = std::move(assembly),
  };
}
#endif

struct RobotFactoryEntry {
  std::string_view robot_type;
  RobotAssemblyCreateResult (*create)(const config::BridgeStartupConfig&);
};

[[nodiscard]] const std::vector<RobotFactoryEntry>& factory_entries() {
  static const std::vector<RobotFactoryEntry> entries{
#if defined(BRIDGE_BUILD_KEPLER_K2)
      {.robot_type = "kepler_k2", .create = &create_kepler_k2_assembly},
#endif
#if defined(BRIDGE_BUILD_PM01)
      {.robot_type = "pm01", .create = &create_pm01_assembly},
#endif
  };
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
