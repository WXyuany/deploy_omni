#include "../../../include/bridge/app/robot_assembly_factory.hpp"
#include "bridge/build_config.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#if BRIDGE_BUILD_KEPLER_K2
#include "../../../include/bridge/adapters/kepler_k2/kepler_k2_adapter.hpp"
#endif
#if BRIDGE_BUILD_PM01
#include "../../../include/bridge/adapters/pm01/pm01_adapter.hpp"
#endif
#if BRIDGE_BUILD_UNITREE_G1
#include "../../../include/bridge/adapters/unitree_g1/unitree_g1_adapter.hpp"
#endif
#if BRIDGE_BUILD_CVTE_CARM
#include "../../../include/bridge/adapters/cvte_carm/cvte_carm_adapter.hpp"
#endif
#include "../../../include/bridge/dds/dds_factory.hpp"
#include "../../../include/bridge/extensions/hooks.hpp"
#if BRIDGE_BUILD_GAMEPAD_SDL
#include "../../../include/bridge/gamepad/sdl_gamepad.hpp"
#endif

namespace bridge::app {
namespace {

[[nodiscard]] std::unique_ptr<extensions::ObservationHook>
create_observation_hook(const config::BridgeStartupConfig& startup_config) {
  if (!startup_config.debug_print.enabled()) {
    return nullptr;
  }
  return extensions::make_terminal_observation_hook({
      .print_command = startup_config.debug_print.command,
      .print_state = startup_config.debug_print.state,
      .rate_hz = startup_config.debug_print.rate_hz,
      .joint_names = startup_config.adapter.joint_names,
  });
}

#if BRIDGE_BUILD_GAMEPAD_SDL
class SdlBridgeGamepadRemoteSource final : public core::BridgeGamepadRemoteSource {
 public:
  explicit SdlBridgeGamepadRemoteSource(gamepad::SdlGamepadRemoteOptions options)
      : remote_(options) {}

  core::RunnerCallResult start() override {
    const gamepad::SdlGamepadStartResult result = remote_.start();
    return {.status = result.connected ? core::RunnerCallStatus::kOk
                                       : core::RunnerCallStatus::kDegraded,
            .detail = result.connected ? "gamepad remote connected"
                                       : "gamepad remote not connected",
            .fault = std::nullopt};
  }

  std::optional<gamepad::CommonGamepad> read() override { return remote_.read(); }

  core::RunnerCallResult stop() override {
    remote_.stop();
    return {.status = core::RunnerCallStatus::kOk,
            .detail = "gamepad remote stopped",
            .fault = std::nullopt};
  }

 private:
  gamepad::SdlGamepadRemote remote_;
};
#endif

[[nodiscard]] std::unique_ptr<core::BridgeGamepadRemoteSource>
create_gamepad_remote(const config::BridgeStartupConfig& startup_config) {
  if (!startup_config.gamepad.enabled) {
    return nullptr;
  }
#if BRIDGE_BUILD_GAMEPAD_SDL
  return std::make_unique<SdlBridgeGamepadRemoteSource>(
      gamepad::SdlGamepadRemoteOptions{
          .controller_index = startup_config.gamepad.controller_index,
          .mapping_path = startup_config.gamepad.mapping_path});
#else
  (void)startup_config;
  return nullptr;
#endif
}

[[nodiscard]] std::size_t configured_hardware_width(
    const std::vector<config::JointMapEntry>& entries) {
  std::size_t width = 0;
  for (const config::JointMapEntry& entry : entries) {
    width = std::max(width, entry.hardware_index + 1);
  }
  return width;
}

class NoopCommandIngress final : public core::BridgeCommandIngress {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "noop_command_ingress";
  }

  core::RunnerCallResult start(core::BridgeCommandSink& command_sink,
                               core::BridgeRuntimeEventSink& event_sink) override {
    (void)command_sink;
    (void)event_sink;
    return {.status = core::RunnerCallStatus::kOk,
            .detail = "noop command ingress started",
            .fault = std::nullopt};
  }

  core::RunnerCallResult stop() override {
    return {.status = core::RunnerCallStatus::kOk,
            .detail = "noop command ingress stopped",
            .fault = std::nullopt};
  }
};

class NoopStateEgress final : public core::BridgeStateEgress {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "noop_state_egress";
  }

  core::RunnerCallResult start() override {
    return {.status = core::RunnerCallStatus::kOk,
            .detail = "noop state egress started",
            .fault = std::nullopt};
  }

  core::RunnerCallResult publish(
      core::StateFrame& state,
      const core::RobotAuxStateSnapshot& aux_state) override {
    (void)state;
    (void)aux_state;
    return {.status = core::RunnerCallStatus::kOk,
            .detail = "noop state egress published",
            .fault = std::nullopt};
  }

  core::RunnerCallResult stop() override {
    return {.status = core::RunnerCallStatus::kOk,
            .detail = "noop state egress stopped",
            .fault = std::nullopt};
  }
};

void attach_configured_bridge_components(
    const config::BridgeStartupConfig& startup_config, RobotAssembly* assembly) {
  const bool use_command_ingress =
      startup_config.assembly.command_ingress.enabled &&
      startup_config.assembly.command_ingress.type == "dds";
  const bool use_state_egress = startup_config.assembly.state_egress.enabled &&
                                startup_config.assembly.state_egress.type == "dds";

  if (use_command_ingress || use_state_egress) {
    dds::DdsAssembly dds_assembly =
        dds::create_dds_assembly(startup_config, assembly->joint_map);
    assembly->runtime = dds_assembly.runtime;
    if (use_command_ingress) {
      assembly->command_ingress = std::move(dds_assembly.command_ingress);
    }
    if (use_state_egress) {
      assembly->state_egress = std::move(dds_assembly.state_egress);
    }
  }

  if (!assembly->command_ingress) {
    assembly->command_ingress = std::make_unique<NoopCommandIngress>();
  }
  if (!assembly->state_egress) {
    assembly->state_egress = std::make_unique<NoopStateEgress>();
  }
}

#if BRIDGE_BUILD_KEPLER_K2
[[nodiscard]] adapters::kepler_k2::KeplerK2AdapterConfig
to_kepler_k2_adapter_config(const config::BridgeStartupConfig& startup_config) {
  adapters::kepler_k2::KeplerK2AdapterConfig config;
  config.session.robot_id = startup_config.adapter.robot_name;
  config.session.transport_endpoint = startup_config.adapter.connection.transport +
                                     ":" +
                                     startup_config.adapter.connection.interface_hint;
  config.session.control_profile = startup_config.adapter.command_mode;
  config.session.initialize_timeout_ns =
      startup_config.adapter.startup.read_initial_state_timeout_ns;
  config.session.read_timeout_ns = startup_config.runner.loop_period_ns;
  config.session.write_timeout_ns = startup_config.runner.loop_period_ns;
  config.safety.require_initial_state_before_motion =
      startup_config.adapter.startup.require_healthy_hardware;
  config.safety.allow_command_when_imu_missing = true;
  config.joint_names = startup_config.adapter.joint_names;
  config.hardware_slot_count =
      configured_hardware_width(startup_config.joint_map_entries);
  return config;
}
#endif

#if BRIDGE_BUILD_UNITREE_G1
struct UnitreeG1AdapterConfigBuildResult {
  std::optional<adapters::unitree_g1::UnitreeG1AdapterConfig> config;
  std::string detail;

  [[nodiscard]] bool succeeded() const noexcept {
    return config.has_value();
  }
};

[[nodiscard]] UnitreeG1AdapterConfigBuildResult to_unitree_g1_adapter_config(
    const config::BridgeStartupConfig& startup_config) {
  if (startup_config.adapter.joint_names.size() !=
      adapters::unitree_g1::kUnitreeG1JointCount) {
    return {
        .config = std::nullopt,
        .detail = "unitree_g1 requires exactly 29 enabled joint names in adapter config",
    };
  }

  adapters::unitree_g1::UnitreeG1AdapterConfig config;
  config.session.transport_mode = "unitree_dds";
  config.session.robot_id = startup_config.adapter.robot_name;
  config.session.transport_endpoint = startup_config.adapter.connection.transport +
                                     ":" +
                                     startup_config.adapter.connection.interface_hint;
  config.session.control_profile = startup_config.adapter.command_mode;
  config.session.command_topic = startup_config.adapter.topics.command;
  config.session.state_topic = startup_config.adapter.topics.state;
  config.session.imu_topic = startup_config.adapter.topics.imu;
  config.session.initialize_timeout_ns =
      startup_config.adapter.startup.read_initial_state_timeout_ns;
  config.session.read_timeout_ns = startup_config.runner.loop_period_ns;
  config.session.write_timeout_ns = startup_config.runner.loop_period_ns;
  config.session.forwarding_log_enabled = startup_config.debug_print.forwarding_log;
  config.safety.require_initial_state_before_motion =
      startup_config.adapter.startup.require_healthy_hardware;
  config.joint_names = startup_config.adapter.joint_names;
  config.hardware_slot_count =
      configured_hardware_width(startup_config.joint_map_entries);

  return {
      .config = std::move(config),
      .detail = "built unitree_g1 adapter config from startup config",
  };
}
#endif

#if BRIDGE_BUILD_PM01
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
  config.session.control_profile = startup_config.adapter.command_mode;
  config.session.initialize_timeout_ns =
      startup_config.adapter.startup.read_initial_state_timeout_ns;
  config.session.read_timeout_ns = startup_config.runner.loop_period_ns;
  config.session.write_timeout_ns = startup_config.runner.loop_period_ns;
  config.safety.require_initial_state_before_motion =
      startup_config.adapter.startup.require_healthy_hardware;
  config.motion_state.required = startup_config.adapter.motion_state.required;
  config.motion_state.required_task =
      startup_config.adapter.motion_state.required_task;
  config.motion_state.initial_task =
      startup_config.adapter.motion_state.initial_task;
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

#if BRIDGE_BUILD_CVTE_CARM
struct CvteCarmAdapterConfigBuildResult {
  std::optional<adapters::cvte_carm::CvteCarmAdapterConfig> config;
  std::string detail;

  [[nodiscard]] bool succeeded() const noexcept { return config.has_value(); }
};

[[nodiscard]] CvteCarmAdapterConfigBuildResult to_cvte_carm_adapter_config(
    const config::BridgeStartupConfig& startup_config) {
  adapters::cvte_carm::CvteCarmAdapterConfig config;
  if (startup_config.robot_type == "cvte_carm_a3") {
    config.model = adapters::cvte_carm::Model::kA3Single;
    config.arm_selection = adapters::cvte_carm::ArmSelection::kSingle;
  } else if (startup_config.robot_type == "cvte_carm_d3" ||
             startup_config.robot_type == "cvte_carm_d3_left" ||
             startup_config.robot_type == "cvte_carm_d3_right") {
    config.model = adapters::cvte_carm::Model::kD3Dual;
    config.arm_selection = startup_config.robot_type == "cvte_carm_d3_left"
                               ? adapters::cvte_carm::ArmSelection::kLeft
                               : startup_config.robot_type == "cvte_carm_d3_right"
                                     ? adapters::cvte_carm::ArmSelection::kRight
                                     : adapters::cvte_carm::ArmSelection::kBoth;
  } else {
    return {.config = std::nullopt,
            .detail = "unsupported CVTE CArm robot type '" + startup_config.robot_type + "'"};
  }

  config.session.robot_id = startup_config.adapter.robot_name;
  config.session.server_ip = startup_config.adapter.connection.interface_hint;
  config.session.control_profile = startup_config.adapter.command_mode;
  config.safety.require_initial_state_before_motion =
      startup_config.adapter.startup.require_healthy_hardware;
  config.joint_names = startup_config.adapter.joint_names;
  config.hardware_slot_count = configured_hardware_width(startup_config.joint_map_entries);
  if (config.model == adapters::cvte_carm::Model::kD3Dual) {
    if (config.joint_names.empty()) {
      return {.config = std::nullopt,
              .detail = "cvte_carm_d3 requires a non-empty joint mapping"};
    }
    if (config.arm_selection == adapters::cvte_carm::ArmSelection::kBoth) {
      if (config.joint_names.size() % 2 != 0) {
        return {.config = std::nullopt,
                .detail = "cvte_carm_d3 both-arm mode requires an even joint mapping width"};
      }
      config.arm_joint_count = config.joint_names.size() / 2U;
    } else {
      config.arm_joint_count = config.joint_names.size();
    }
  }
  return {.config = std::move(config),
          .detail = "built CVTE CArm adapter config from startup config"};
}

[[nodiscard]] RobotAssemblyCreateResult create_cvte_carm_assembly(
    const config::BridgeStartupConfig& startup_config) {
  const config::JointMapBuildResult joint_map_result = config::JointMap::build(
      startup_config.joint_map_entries, startup_config.joint_map_validation);
  if (!joint_map_result.succeeded() || !joint_map_result.joint_map.has_value()) {
    return {.status = RobotAssemblyCreateStatus::kInvalidConfig,
            .detail = "joint-map build failed during CVTE CArm assembly",
            .assembly = std::nullopt};
  }

  const CvteCarmAdapterConfigBuildResult adapter_config_result =
      to_cvte_carm_adapter_config(startup_config);
  if (!adapter_config_result.succeeded()) {
    return {.status = RobotAssemblyCreateStatus::kInvalidConfig,
            .detail = adapter_config_result.detail,
            .assembly = std::nullopt};
  }

  RobotAssembly assembly{
      .joint_map = std::move(*joint_map_result.joint_map),
      .adapter = std::make_unique<adapters::cvte_carm::CvteCarmAdapter>(
          std::move(*adapter_config_result.config)),
      .command_ingress = nullptr,
      .state_egress = nullptr,
      .observation_hook = create_observation_hook(startup_config),
      .gamepad_remote = create_gamepad_remote(startup_config),
  };

  attach_configured_bridge_components(startup_config, &assembly);
  return {.status = RobotAssemblyCreateStatus::kOk,
          .detail = "assembled CVTE CArm robot runtime",
          .assembly = std::move(assembly)};
}
#endif

#if BRIDGE_BUILD_UNITREE_G1
[[nodiscard]] RobotAssemblyCreateResult create_unitree_g1_assembly(
    const config::BridgeStartupConfig& startup_config) {
  const config::JointMapBuildResult joint_map_result = config::JointMap::build(
      startup_config.joint_map_entries, startup_config.joint_map_validation);
  if (!joint_map_result.succeeded() || !joint_map_result.joint_map.has_value()) {
    return {
        .status = RobotAssemblyCreateStatus::kInvalidConfig,
        .detail = "joint-map build failed during unitree_g1 assembly",
        .assembly = std::nullopt,
    };
  }

  UnitreeG1AdapterConfigBuildResult adapter_config_result =
      to_unitree_g1_adapter_config(startup_config);
  if (!adapter_config_result.succeeded()) {
    return {
        .status = RobotAssemblyCreateStatus::kInvalidConfig,
        .detail = adapter_config_result.detail,
        .assembly = std::nullopt,
    };
  }

  RobotAssembly assembly{
      .joint_map = std::move(*joint_map_result.joint_map),
      .adapter = std::make_unique<adapters::unitree_g1::UnitreeG1Adapter>(
          std::move(*adapter_config_result.config)),
      .command_ingress = nullptr,
      .state_egress = nullptr,
      .observation_hook = create_observation_hook(startup_config),
      .gamepad_remote = create_gamepad_remote(startup_config),
  };

  attach_configured_bridge_components(startup_config, &assembly);

  return {
      .status = RobotAssemblyCreateStatus::kOk,
      .detail = "assembled unitree_g1 robot runtime",
      .assembly = std::move(assembly),
  };
}
#endif

#if BRIDGE_BUILD_KEPLER_K2
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
      .observation_hook = create_observation_hook(startup_config),
      .gamepad_remote = create_gamepad_remote(startup_config),
  };

  attach_configured_bridge_components(startup_config, &assembly);

  return {
      .status = RobotAssemblyCreateStatus::kOk,
      .detail = "assembled kepler_k2 robot runtime",
      .assembly = std::move(assembly),
  };
}
#endif

#if BRIDGE_BUILD_PM01
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
      .observation_hook = create_observation_hook(startup_config),
      .gamepad_remote = create_gamepad_remote(startup_config),
  };

  attach_configured_bridge_components(startup_config, &assembly);

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
#if BRIDGE_BUILD_KEPLER_K2
      {.robot_type = "kepler_k2", .create = &create_kepler_k2_assembly},
#endif
#if BRIDGE_BUILD_PM01
      {.robot_type = "pm01", .create = &create_pm01_assembly},
#endif
#if BRIDGE_BUILD_UNITREE_G1
      {.robot_type = "unitree_g1", .create = &create_unitree_g1_assembly},
#endif
#if BRIDGE_BUILD_CVTE_CARM
      {.robot_type = "cvte_carm_a3", .create = &create_cvte_carm_assembly},
      {.robot_type = "cvte_carm_d3", .create = &create_cvte_carm_assembly},
      {.robot_type = "cvte_carm_d3_left", .create = &create_cvte_carm_assembly},
      {.robot_type = "cvte_carm_d3_right", .create = &create_cvte_carm_assembly},
#endif
  };
  return entries;
}

}

RobotAssemblyCreateResult create_robot_assembly(
    const config::BridgeStartupConfig& startup_config) {
#if !BRIDGE_BUILD_GAMEPAD_SDL
  if (startup_config.gamepad.enabled && startup_config.gamepad.backend == "sdl") {
    return {
        .status = RobotAssemblyCreateStatus::kInvalidConfig,
        .detail = "gamepad backend sdl requested but SDL2 support is not built",
        .assembly = std::nullopt,
    };
  }
#endif
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
