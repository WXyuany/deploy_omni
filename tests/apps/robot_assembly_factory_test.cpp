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
      << "  startup_idle_policy:\n"
      << "    mode: zero_torque\n"
      << "    require_first_valid_command: true\n"
      << "    use_adapter_safe_command: true\n"
      << "  fault_policy:\n"
      << "    mode: fail_stop\n"
      << "    send_safe_command_on_fault: true\n"
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
      << "    transport: dds\n"
      << "    interface_hint: pm01\n"
      << "  startup:\n"
      << "    read_initial_state_timeout_ms: 1000\n"
      << "    require_healthy_hardware: true\n"
      << "  safety:\n"
      << "    safe_command_profile: zero_torque\n"
      << "    allow_safe_command_during_shutdown: true\n"
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
  assert(assembled.assembly->adapter->name() == std::string_view{"kepler_k2"});

  bridge::core::BridgeRunner runner(loaded.config->runner,
                                    assembled.assembly->runner_dependencies());
  assert(runner.initialize().succeeded());
  assert(runner.run().succeeded());
  assert(runner.request_stop().succeeded());
  assert(runner.shutdown().succeeded());
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

void TestPm01AssemblyPropagatesBackendField() {
#if BRIDGE_BUILD_PM01
  const std::string path = WritePm01Config("pm01_robot_assembly_backend_test.yaml");
  {
    std::ofstream output(path, std::ios::app);
    output << "adapter:\n"
           << "  backend: ros2\n";
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
  assert(pm01->config().backend == "ros2");
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
  TestRejectsUnsupportedRobotType();
  TestRegisteredRobotTypesIncludesKeplerK2();
  TestRegisteredRobotTypesIncludesPm01();
  TestCreatesPm01AssemblyFromConfig();
  TestPm01AssemblyPropagatesBackendField();
  TestPm01AssemblyPropagatesMotionStateConfig();
  return 0;
}
