#include <algorithm>
#include <cassert>
#include <fstream>
#include <string>
#include <vector>

#include "bridge/build_config.hpp"
#include "../../include/bridge/app/robot_assembly_factory.hpp"
#include "../../include/bridge/adapters/pm01/pm01_adapter.hpp"
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

std::string SamplePath(const std::string& relative_path) {
#ifdef BRIDGE_REPO_ROOT
  return std::string(BRIDGE_REPO_ROOT) + "/" + relative_path;
#else
  return "../" + relative_path;
#endif
}

std::string WriteSampleConfigWithGamepadEnabled(const std::string& relative_path,
                                                const std::string& output_path) {
  std::ifstream input(SamplePath(relative_path));
  std::ofstream output(output_path);
  std::string line;
  bool in_gamepad = false;
  while (std::getline(input, line)) {
    if (line == "gamepad:") {
      in_gamepad = true;
      output << line << '\n';
      continue;
    }
    if (in_gamepad && line == "  enabled: false") {
      output << "  enabled: true\n";
      in_gamepad = false;
      continue;
    }
    output << line << '\n';
  }
  return output_path;
}

std::string WriteSampleConfigWithAssemblyGamepadModule(
    const std::string& relative_path,
    const std::string& output_path) {
  std::ifstream input(SamplePath(relative_path));
  std::ofstream output(output_path);
  output << input.rdbuf();
  output << "assembly:\n"
         << "  modules:\n"
         << "    - name: gamepad\n"
         << "      type: sdl\n"
         << "      enabled: true\n"
         << "      controller_index: 0\n"
         << "      mapping_path: config/gamecontrollerdb.txt\n";
  return output_path;
}

std::string WritePm01Config(const std::string& path) {
  static const std::vector<std::string> kJointNames{
      "l_hip_yaw",       "l_hip_roll",      "l_hip_pitch",
      "l_knee",          "l_ankle_pitch",   "l_ankle_roll",
      "r_hip_yaw",       "r_hip_roll",      "r_hip_pitch",
      "r_knee",          "r_ankle_pitch",   "r_ankle_roll",
      "waist_yaw",       "waist_pitch",     "waist_roll",
      "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw",
      "l_elbow",         "l_wrist_yaw",     "r_shoulder_pitch",
      "r_shoulder_roll", "r_shoulder_yaw",  "r_elbow"};

  std::ofstream output(path);
  output
      << "schema_version: 1\n"
      << "bridge:\n"
      << "  robot_name: pm01\n"
      << "  loop_rate_hz: 250\n"
      << "  stale_command_timeout_ms: 120\n"
      << "  fault_policy:\n"
      << "    mode: fail_stop\n"
      << "    publish_degraded_state_while_faulted: true\n"
      << "    halt_command_writes_while_faulted: true\n"
      << "dds:\n"
      << "  domain_id: 0\n"
      << "  topics:\n"
      << "    command:\n"
      << "      name: rt/lowcmd\n"
      << "    state:\n"
      << "      name: rt/lowstate\n"
      << "adapter:\n"
      << "  kind: pm01\n"
      << "  robot_model: pm01\n"
      << "  command_mode: joint_bridge\n"
      << "  connection:\n"
      << "    transport: ros2\n"
      << "  startup:\n"
      << "    read_initial_state_timeout_ms: 1000\n"
      << "    require_healthy_hardware: true\n"
      << "joint_mapping:\n"
      << "  validation:\n"
      << "    expected_sim_joint_count: 24\n"
      << "    expected_hardware_joint_count: 24\n"
      << "    require_dense_enabled_sim_indexes: true\n"
      << "    require_dense_enabled_hardware_indexes: true\n"
      << "    require_unique_enabled_sim_names: true\n"
      << "    require_unique_enabled_hardware_names: true\n"
      << "  entries:\n";

  for (std::size_t index = 0; index < kJointNames.size(); ++index) {
    output << "    - sim_index: " << index << "\n"
           << "      hardware_index: " << index << "\n"
           << "      sim_name: sim_" << kJointNames[index] << "\n"
           << "      hardware_name: " << kJointNames[index] << "\n"
           << "      label: " << kJointNames[index] << "\n"
           << "      enabled: true\n"
           << "      transforms:\n"
           << "        position: {sign: 1.0, offset: 0.0}\n"
           << "        velocity: {sign: 1.0, offset: 0.0}\n"
           << "        effort: {sign: 1.0, offset: 0.0}\n"
           << "        stiffness: {sign: 1.0, offset: 0.0}\n"
           << "        damping: {sign: 1.0, offset: 0.0}\n"
           << "        acceleration: {sign: 1.0, offset: 0.0}\n"
           << "      sim_limits:\n"
           << "        position: {lower: -1.0, upper: 1.0}\n"
           << "      hardware_limits:\n"
           << "        position: {lower: -1.0, upper: 1.0}\n";
  }

  return path;
}

std::string WritePm01ConfigWithoutAdapterKind(const std::string& path) {
  const std::string source_path = WritePm01Config(path + ".source");
  std::ifstream input(source_path);
  std::ofstream output(path);
  std::string line;
  while (std::getline(input, line)) {
    if (line == "  kind: pm01") {
      continue;
    }
    output << line << '\n';
  }
  return path;
}

void TestCreatesKeplerAssemblyFromSampleConfig() {
#if BRIDGE_BUILD_KEPLER_K2
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
  assert(assembled.assembly->gamepad_remote == nullptr);
  assert(assembled.assembly->adapter->name() == std::string_view{"kepler_k2"});

  bridge::core::BridgeRunner runner(loaded.config->runner,
                                    assembled.assembly->runner_dependencies());
  assert(runner.initialize().succeeded());
  assert(runner.run().succeeded());
  assert(runner.request_stop().succeeded());
  assert(runner.shutdown().succeeded());
#endif
}

void TestCreatesGamepadRemoteWhenEnabled() {
#if BRIDGE_BUILD_KEPLER_K2 && BRIDGE_BUILD_GAMEPAD_SDL
  const std::string path = WriteSampleConfigWithAssemblyGamepadModule(
      "config/kepler_k2.sample.yaml", "kepler_gamepad_enabled_test.yaml");
  bridge::config::BridgeConfigLoadResult loaded =
      bridge::config::load_bridge_config(path);
  assert(loaded.succeeded());
  assert(loaded.config.has_value());

  bridge::app::RobotAssemblyCreateResult assembled =
      bridge::app::create_robot_assembly(*loaded.config);
  assert(assembled.succeeded());
  assert(assembled.assembly.has_value());
  assert(assembled.assembly->gamepad_remote != nullptr);
  assert(assembled.assembly->runner_dependencies().gamepad_remote ==
         assembled.assembly->gamepad_remote.get());
#endif
}

void TestCreatesPm01GamepadRemoteWhenConfigEnabled() {
#if BRIDGE_BUILD_PM01 && BRIDGE_BUILD_GAMEPAD_SDL
  const std::string path = WriteSampleConfigWithAssemblyGamepadModule(
      "config/pm01.sample.yaml", "pm01_gamepad_enabled_test.yaml");
  const bridge::config::BridgeConfigLoadResult loaded =
      bridge::config::load_bridge_config(path);
  assert(loaded.succeeded());
  assert(loaded.config.has_value());

  bridge::app::RobotAssemblyCreateResult assembled =
      bridge::app::create_robot_assembly(*loaded.config);
  assert(assembled.succeeded());
  assert(assembled.assembly.has_value());
  assert(assembled.assembly->gamepad_remote != nullptr);
  assert(assembled.assembly->runner_dependencies().gamepad_remote ==
         assembled.assembly->gamepad_remote.get());
#endif
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

void TestRegisteredRobotTypesIncludesPm01() {
  const std::vector<std::string_view> types =
      bridge::app::registered_robot_types();
#if BRIDGE_BUILD_PM01
  assert(std::find(types.begin(), types.end(), std::string_view{"pm01"}) !=
         types.end());
#else
  assert(std::find(types.begin(), types.end(), std::string_view{"pm01"}) ==
         types.end());
#endif
}

void TestCreatesPm01AssemblyFromConfig() {
#if BRIDGE_BUILD_PM01
  const std::string path = WritePm01Config("pm01_robot_assembly_test.yaml");
  const bridge::config::BridgeConfigLoadResult loaded =
      bridge::config::load_bridge_config(path);
  assert(loaded.succeeded());
  assert(loaded.config.has_value());

  bridge::app::RobotAssemblyCreateResult assembled =
      bridge::app::create_robot_assembly(*loaded.config);
  assert(assembled.succeeded());
  assert(assembled.assembly.has_value());
  assert(assembled.assembly->adapter != nullptr);
  assert(assembled.assembly->adapter->name() == std::string_view{"pm01"});
#endif
}

void TestPm01AssemblyPropagatesMotionStateConfig() {
#if BRIDGE_BUILD_PM01
  const std::string path = WritePm01Config("pm01_robot_assembly_motion_state_test.yaml");
  {
    std::ofstream output(path, std::ios::app);
    output << "adapter:\n"
           << "  motion_state:\n"
           << "    required: true\n"
           << "    required_task: passive\n"
           << "    initial_task: idle\n";
  }

  const bridge::config::BridgeConfigLoadResult loaded =
      bridge::config::load_bridge_config(path);
  assert(loaded.succeeded());
  assert(loaded.config.has_value());

  bridge::app::RobotAssemblyCreateResult assembled =
      bridge::app::create_robot_assembly(*loaded.config);
  assert(assembled.succeeded());
  assert(assembled.assembly.has_value());
  assert(assembled.assembly->adapter != nullptr);

  const auto* pm01 = dynamic_cast<bridge::adapters::pm01::Pm01Adapter*>(
      assembled.assembly->adapter.get());
  assert(pm01 != nullptr);
  assert(pm01->config().motion_state.required);
  assert(pm01->config().motion_state.required_task == "passive");
  assert(pm01->config().motion_state.initial_task.has_value());
  assert(*pm01->config().motion_state.initial_task == "idle");
#endif
}

void TestAssemblyCanDisableDdsIngressAndEgress() {
#if BRIDGE_BUILD_PM01
  const std::string path = WritePm01Config("pm01_robot_assembly_components_test.yaml");
  {
    std::ofstream output(path, std::ios::app);
    output << "assembly:\n"
           << "  command_ingress:\n"
           << "    enabled: false\n"
           << "    type: none\n"
           << "  state_egress:\n"
           << "    enabled: false\n"
           << "    type: none\n";
  }

  const bridge::config::BridgeConfigLoadResult loaded =
      bridge::config::load_bridge_config(path);
  assert(loaded.succeeded());
  assert(loaded.config.has_value());

  bridge::app::RobotAssemblyCreateResult assembled =
      bridge::app::create_robot_assembly(*loaded.config);
  assert(assembled.succeeded());
  assert(assembled.assembly.has_value());
  assert(assembled.assembly->adapter != nullptr);
  assert(assembled.assembly->command_ingress != nullptr);
  assert(assembled.assembly->state_egress != nullptr);
#endif
}

void TestAssemblyProfileCreatesPm01AdapterWithoutAdapterKind() {
#if BRIDGE_BUILD_PM01
  const std::string path = WritePm01ConfigWithoutAdapterKind(
      "pm01_robot_assembly_profile_test.yaml");
  {
    std::ofstream output(path, std::ios::app);
    output << "assembly:\n"
           << "  profile: pm01\n";
  }

  const bridge::config::BridgeConfigLoadResult loaded =
      bridge::config::load_bridge_config(path);
  assert(loaded.succeeded());
  assert(loaded.config.has_value());

  bridge::app::RobotAssemblyCreateResult assembled =
      bridge::app::create_robot_assembly(*loaded.config);
  assert(assembled.succeeded());
  assert(assembled.assembly.has_value());
  assert(assembled.assembly->adapter != nullptr);
  assert(assembled.assembly->adapter->name() == std::string_view{"pm01"});
#endif
}

void TestRegisteredRobotTypesIncludesKeplerK2() {
  const std::vector<std::string_view> types =
      bridge::app::registered_robot_types();
#if BRIDGE_BUILD_KEPLER_K2
  assert(std::find(types.begin(), types.end(), std::string_view{"kepler_k2"}) !=
         types.end());
#else
  assert(std::find(types.begin(), types.end(), std::string_view{"kepler_k2"}) ==
         types.end());
#endif
}

}

int main() {
  TestCreatesKeplerAssemblyFromSampleConfig();
  TestCreatesGamepadRemoteWhenEnabled();
  TestCreatesPm01GamepadRemoteWhenConfigEnabled();
  TestRejectsUnsupportedRobotType();
  TestRegisteredRobotTypesIncludesKeplerK2();
  TestRegisteredRobotTypesIncludesPm01();
  TestCreatesPm01AssemblyFromConfig();
  TestPm01AssemblyPropagatesMotionStateConfig();
  TestAssemblyCanDisableDdsIngressAndEgress();
  TestAssemblyProfileCreatesPm01AdapterWithoutAdapterKind();
  return 0;
}
