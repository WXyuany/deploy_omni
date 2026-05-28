#include <cassert>
#include <fstream>
#include <string>
#include <vector>

#include "../../include/bridge/config/bridge_config.hpp"

namespace {

using bridge::config::BridgeConfigIssueCode;
using bridge::config::BridgeConfigLoadStatus;
using bridge::config::JointMap;
using bridge::config::JointMapBuildResult;
using bridge::config::load_bridge_config;

bool HasIssueCode(const bridge::config::BridgeConfigLoadResult& result,
                  const BridgeConfigIssueCode code) {
  for (const auto& issue : result.issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
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

std::string WriteSimplifiedKeplerConfig(const std::string& path,
                                       const bool include_loop_rate = true,
                                       const bool include_command_topic = true,
                                       const bool include_state_topic = true) {
  std::ofstream output(path);
  output << "robot_kind: kepler_k2\n";
  if (include_loop_rate) {
    output << "loop_rate_hz: 1000\n";
  }
  output << "stale_command_timeout_ms: 120\n"
         << "dds:\n";
  if (include_command_topic) {
    output << "  command: /lowcmd\n";
  }
  if (include_state_topic) {
    output << "  state: /lowstate\n";
  }
  output << "  status: /bridge_status\n"
         << "joint_mapping:\n"
         << "  - name: left_hip_yaw\n"
         << "    index: 0\n"
         << "    sign: 1.0\n"
         << "    offset: 0.0\n"
         << "    enable: true\n";
  return path;
}

std::string WriteSimplifiedPm01Config(const std::string& path,
                                      const std::optional<std::string>& transport = std::nullopt,
                                      const std::optional<std::string>& interface_hint = std::nullopt) {
  std::ofstream output(path);
  output << "robot_name: pm01\n"
         << "api_rate_hz: 250\n";
  if (transport.has_value()) {
    output << "transport: " << *transport << "\n";
  }
  if (interface_hint.has_value()) {
    output << "interface_hint: " << *interface_hint << "\n";
  }
  output << "dds:\n"
         << "  command_topic: rt/lowcmd\n"
         << "  state_topic: rt/lowstate\n"
         << "joints:\n";
  for (int index = 0; index < 24; ++index) {
    output << "  - { index: " << index
           << ", enable: true, sign: 1.0, offset: 0.0 }\n";
  }
  return path;
}

void TestLoadsSampleBridgeConfig() {
  const auto result = load_bridge_config("../config/kepler_k2.sample.yaml");
  assert(result.succeeded());
  assert(result.status == BridgeConfigLoadStatus::kOk);
  assert(result.config.has_value());

  const auto& config = *result.config;
  assert(config.source_path == "../config/kepler_k2.sample.yaml");
  assert(config.robot_type == "kepler_k2");
  assert(config.runner.loop_period_ns == 1'000'000U);
  assert(config.runner.command_stale_timeout_ns == 120'000'000U);
  assert(config.dds.domain_id == 0U);
  assert(config.dds.topics.command == "rt/lowcmd");
  assert(config.dds.topics.state == "rt/lowstate");
  assert(config.dds.topics.status.empty());
  assert(!config.debug_print.state);
  assert(!config.debug_print.command);
  assert(config.debug_print.rate_hz == 2U);
  assert(config.adapter.kind == "kepler_k2");
  assert(config.adapter.robot_name == "kepler_k2");
  assert(config.adapter.robot_model == "k2");
  assert(config.adapter.command_mode == "torque_pd_passthrough");
  assert(config.adapter.connection.transport == "udp");
  assert(config.adapter.connection.interface_hint == "robot_lan0");
  assert(!config.gamepad.enabled);
  assert(config.gamepad.backend == "sdl");
  assert(config.gamepad.controller_index == 0);
  assert(config.gamepad.mapping_path ==
         "docs/joy_stick/include/hardware/gamecontrollerdb.txt");
  assert(config.runner.gamepad_poll_interval_ns == 5'000'000U);
  assert(config.runner.gamepad_stale_timeout_ns == 100'000'000U);
  assert(config.adapter.startup.read_initial_state_timeout_ns == 1'000'000'000U);
  assert(config.adapter.startup.require_healthy_hardware);
  assert(config.adapter.safety.safe_command_profile == "zero_torque");
  assert(config.adapter.safety.allow_safe_command_during_shutdown);
  assert(config.adapter.joint_names.size() == 12U);
  assert(config.adapter.joint_names.front().find("joint_") == 0 ||
        config.adapter.joint_names.front() == "l_hip_yaw");
  assert(config.adapter.joint_names.back().find("joint_") == 0 ||
        config.adapter.joint_names.back() == "r_ankle_roll");
  assert(config.joint_map_entries.size() == 12U);

  const JointMapBuildResult joint_map =
      JointMap::build(config.joint_map_entries, config.joint_map_validation);
  assert(joint_map.succeeded());
}

void TestLoadsPm01SampleBridgeConfig() {
  const auto result = load_bridge_config("../config/pm01.sample.yaml");
  assert(result.succeeded());
  assert(result.status == BridgeConfigLoadStatus::kOk);
  assert(result.config.has_value());

  const auto& config = *result.config;
  assert(config.source_path == "../config/pm01.sample.yaml");
  assert(config.robot_type == "pm01");
  assert(config.dds.topics.command == "rt/lowcmd");
  assert(config.dds.topics.state == "rt/lowstate");
  assert(config.adapter.kind == "pm01");
  assert(config.adapter.robot_name == "pm01");
  assert(config.adapter.robot_model == "pm01");
  assert(config.adapter.command_mode == "joint_bridge");
  assert(config.adapter.connection.transport == "ros2");
  assert(config.adapter.connection.interface_hint == "pm01");
  assert(!config.adapter.motion_state.required);
  assert(config.adapter.motion_state.required_task == "joint_bridge");
  assert(!config.adapter.motion_state.initial_task.has_value());
  assert(!config.gamepad.enabled);
  assert(config.gamepad.backend == "sdl");
  assert(config.gamepad.controller_index == 0);
  assert(config.gamepad.mapping_path ==
         "docs/joy_stick/include/hardware/gamecontrollerdb.txt");
  assert(config.runner.gamepad_poll_interval_ns == 5'000'000U);
  assert(config.runner.gamepad_stale_timeout_ns == 100'000'000U);
  assert(!config.debug_print.state);
  assert(config.debug_print.command);
  assert(config.debug_print.rate_hz == 2U);
  assert(config.adapter.joint_names.size() == 24U);
  assert(config.adapter.joint_names.front() == "l_hip_yaw");
  assert(config.adapter.joint_names.back() == "r_elbow");
  assert(config.joint_map_entries.size() == 24U);

  const JointMapBuildResult joint_map =
      JointMap::build(config.joint_map_entries, config.joint_map_validation);
  assert(joint_map.succeeded());
}

void TestLoadsUnitreeG1SampleBridgeConfig() {
  const auto result = load_bridge_config("../config/g1.sample.yaml");
  assert(result.succeeded());
  assert(result.status == BridgeConfigLoadStatus::kOk);
  assert(result.config.has_value());

  const auto& config = *result.config;
  assert(config.source_path == "../config/g1.sample.yaml");
  assert(config.robot_type == "unitree_g1");
  assert(config.runner.loop_period_ns == 1'250'000U);
  assert(config.dds.topics.command == "bridge/rt/lowcmd");
  assert(config.dds.topics.state == "bridge/rt/lowstate");
  assert(config.adapter.kind == "unitree_g1");
  assert(config.adapter.robot_name == "unitree_g1");
  assert(config.adapter.robot_model == "g1");
  assert(config.adapter.command_mode == "raw_low_level");
  assert(config.adapter.connection.transport == "dds");
  assert(config.adapter.connection.interface_hint == "enx9c69d3018e32");
  assert(config.adapter.topics.command == "custom/mujoco/rt/lowcmd");
  assert(config.adapter.topics.state == "custom/mujoco/rt/lowstate");
  assert(config.adapter.topics.imu == "rt/secondary_imu");
  assert(config.gamepad.enabled);
  assert(config.gamepad.mapping_path ==
         "docs/joy_stick/include/hardware/gamecontrollerdb.txt");
  assert(config.runner.gamepad_poll_interval_ns == 5'000'000U);
  assert(config.runner.gamepad_stale_timeout_ns == 100'000'000U);
  assert(config.adapter.safety.safe_command_profile == "zero_torque");
  assert(config.adapter.joint_names.size() == 29U);
  assert(config.joint_map_entries.size() == 29U);

  const JointMapBuildResult joint_map =
      JointMap::build(config.joint_map_entries, config.joint_map_validation);
  assert(joint_map.succeeded());
}

void TestLoadsDebugPrintBlock() {
  const std::string path = "bridge_config_debug_print_test.yaml";
  WritePm01Config(path);
  {
    std::ofstream output(path, std::ios::app);
    output << "debug_print:\n"
           << "  state: true\n"
           << "  cmd: false\n"
           << "  rate_hz: 2\n";
  }

  const auto result = load_bridge_config(path);
  assert(result.succeeded());
  assert(result.config.has_value());
  assert(result.config->debug_print.state);
  assert(!result.config->debug_print.command);
  assert(result.config->debug_print.rate_hz == 2U);
}

void TestLoadsPm01MotionStateConfig() {
  const std::string path = WritePm01Config("bridge_config_pm01_motion_state_test.yaml");
  {
    std::ofstream output(path, std::ios::app);
    output << "adapter:\n"
           << "  motion_state:\n"
           << "    required: true\n"
           << "    required_task: passive\n"
           << "    initial_task: idle\n";
  }

  const auto result = load_bridge_config(path);
  assert(result.succeeded());
  assert(result.config.has_value());
  assert(result.config->adapter.motion_state.required);
  assert(result.config->adapter.motion_state.required_task == "passive");
  assert(result.config->adapter.motion_state.initial_task.has_value());
  assert(*result.config->adapter.motion_state.initial_task == "idle");
}

void TestLoadsSimplifiedPm01InterfaceHintOverride() {
  const auto result = load_bridge_config(
      WriteSimplifiedPm01Config("bridge_config_simplified_pm01_interface_hint_test.yaml",
                                std::string("ros2"), std::string("pm01_lab")));
  assert(result.succeeded());
  assert(result.config.has_value());
  assert(result.config->adapter.connection.transport == "ros2");
  assert(result.config->adapter.connection.interface_hint == "pm01_lab");
}

void TestLoadsSimplifiedPm01InterfaceHintDefault() {
  const auto result = load_bridge_config(
      WriteSimplifiedPm01Config("bridge_config_simplified_pm01_default_hint_test.yaml",
                                std::string("ros2"), std::nullopt));
  assert(result.succeeded());
  assert(result.config.has_value());
  assert(result.config->adapter.connection.transport == "ros2");
  assert(result.config->adapter.connection.interface_hint == "pm01");
}

void TestLoadsSimplifiedKeplerConfig() {
  const std::string path = "bridge_config_simplified_k2_test.yaml";
  WriteSimplifiedKeplerConfig(path);
  {
    std::ofstream output(path, std::ios::app);
    output << "  - name: left_knee\n"
           << "    index: 1\n"
           << "    sign: -1.0\n"
           << "    offset: 0.5\n"
           << "    enable: false\n"
           << "  - name: right_hip_yaw\n"
           << "    index: 2\n"
           << "    sign: 1.0\n"
           << "    offset: 0.0\n"
           << "    enable: true\n";
  }

  const auto result = load_bridge_config(path);
  assert(result.succeeded());
  assert(result.config.has_value());
  assert(result.config->robot_type == "kepler_k2");
  assert(result.config->runner.loop_period_ns == 1'000'000U);
  assert(result.config->dds.topics.command == "/lowcmd");
  assert(result.config->dds.topics.state == "/lowstate");
  assert(result.config->dds.topics.status == "/bridge_status");
  assert(result.config->adapter.kind == "kepler_k2");
  assert(result.config->adapter.command_mode == "torque_pd_passthrough");
  assert(result.config->adapter.connection.transport == "udp");
  assert(result.config->adapter.connection.interface_hint == "robot_lan0");
  assert(result.config->adapter.joint_names.size() == 2U);
  assert(result.config->adapter.joint_names[0] == "left_hip_yaw");
  assert(result.config->adapter.joint_names[1] == "right_hip_yaw");
  assert(result.config->joint_map_entries.size() == 3U);
  assert(result.config->joint_map_entries[1].enabled == false);
  assert(result.config->joint_map_entries[1].transforms.position.sign == -1.0);
  assert(result.config->joint_map_entries[1].transforms.position.offset == 0.5);
}

void TestRejectsSimplifiedConfigWithoutLoopRate() {
  const auto result = load_bridge_config(
      WriteSimplifiedKeplerConfig("bridge_config_missing_loop_rate_test.yaml",
                                  false, true, true));
  assert(!result.succeeded());
  assert(result.status == BridgeConfigLoadStatus::kRejected);
  assert(HasIssueCode(result, BridgeConfigIssueCode::kMissingField));
}

void TestRejectsSimplifiedConfigWithoutCommandTopic() {
  const auto result = load_bridge_config(
      WriteSimplifiedKeplerConfig("bridge_config_missing_command_topic_test.yaml",
                                  true, false, true));
  assert(!result.succeeded());
  assert(result.status == BridgeConfigLoadStatus::kRejected);
  assert(HasIssueCode(result, BridgeConfigIssueCode::kMissingField));
}

void TestRejectsSimplifiedConfigWithoutStateTopic() {
  const auto result = load_bridge_config(
      WriteSimplifiedKeplerConfig("bridge_config_missing_state_topic_test.yaml",
                                  true, true, false));
  assert(!result.succeeded());
  assert(result.status == BridgeConfigLoadStatus::kRejected);
  assert(HasIssueCode(result, BridgeConfigIssueCode::kMissingField));
}

void TestLoadsPm01BackendField() {
  const std::string path = "bridge_config_pm01_backend_test.yaml";
  {
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
        << "  backend: ros2\n"
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
        << "    expected_sim_joint_count: 1\n"
        << "    expected_hardware_joint_count: 1\n"
        << "    require_dense_enabled_sim_indexes: true\n"
        << "    require_dense_enabled_hardware_indexes: true\n"
        << "    require_unique_enabled_sim_names: true\n"
        << "    require_unique_enabled_hardware_names: true\n"
        << "  entries:\n"
        << "    - sim_index: 0\n"
        << "      hardware_index: 0\n"
        << "      sim_name: joint_0\n"
        << "      hardware_name: joint_0\n"
        << "      label: joint_0\n"
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

  const auto result = load_bridge_config(path);
  assert(result.succeeded());
  assert(result.config.has_value());
  assert(result.config->adapter.backend == "ros2");
}

void TestRejectsUnsupportedFaultPolicyMode() {
  const std::string path = "bridge_config_invalid_test.yaml";
  {
    std::ofstream output(path);
    output
        << "schema_version: 1\n"
        << "bridge:\n"
        << "  robot_name: kepler_k2\n"
        << "  loop_rate_hz: 250\n"
        << "  stale_command_timeout_ms: 120\n"
        << "  startup_idle_policy:\n"
        << "    mode: zero_torque\n"
        << "    require_first_valid_command: true\n"
        << "    use_adapter_safe_command: true\n"
        << "  fault_policy:\n"
        << "    mode: continue\n"
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
        << "  kind: kepler_k2\n"
        << "  robot_model: k2\n"
        << "  command_mode: torque_pd_passthrough\n"
        << "  connection:\n"
        << "    transport: udp\n"
        << "    interface_hint: robot_lan0\n"
        << "  startup:\n"
        << "    read_initial_state_timeout_ms: 1000\n"
        << "    require_healthy_hardware: true\n"
        << "  safety:\n"
        << "    safe_command_profile: zero_torque\n"
        << "    allow_safe_command_during_shutdown: true\n"
        << "joint_mapping:\n"
        << "  validation:\n"
        << "    expected_sim_joint_count: 1\n"
        << "    expected_hardware_joint_count: 1\n"
        << "    require_dense_enabled_sim_indexes: true\n"
        << "    require_dense_enabled_hardware_indexes: true\n"
        << "    require_unique_enabled_sim_names: true\n"
        << "    require_unique_enabled_hardware_names: true\n"
        << "  entries:\n"
        << "    - sim_index: 0\n"
        << "      hardware_index: 0\n"
        << "      sim_name: left_hip_yaw\n"
        << "      hardware_name: l_hip_yaw\n"
        << "      label: left hip yaw\n"
        << "      enabled: true\n"
        << "      transforms:\n"
        << "        position: {sign: 1.0, offset: 0.0}\n"
        << "        velocity: {sign: 1.0, offset: 0.0}\n"
        << "        effort: {sign: 1.0, offset: 0.0}\n"
        << "        stiffness: {sign: 1.0, offset: 0.0}\n"
        << "        damping: {sign: 1.0, offset: 0.0}\n"
        << "        acceleration: {sign: 1.0, offset: 0.0}\n"
        << "      sim_limits:\n"
        << "        position: {lower: -0.8, upper: 0.8}\n"
        << "      hardware_limits:\n"
        << "        position: {lower: -0.8, upper: 0.8}\n";
  }

  const auto result = load_bridge_config(path);
  assert(!result.succeeded());
  assert(result.status == BridgeConfigLoadStatus::kRejected);
  assert(HasIssueCode(result, BridgeConfigIssueCode::kUnsupportedValue));
}

void TestLoadsConfigWhenDisabledEntryIsOutsideEnabledHardwareWidth() {
  const std::string path = "bridge_config_disabled_entry_test.yaml";
  {
    std::ofstream output(path);
    output
        << "schema_version: 1\n"
        << "bridge:\n"
        << "  robot_name: kepler_k2\n"
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
        << "  kind: kepler_k2\n"
        << "  robot_model: k2\n"
        << "  command_mode: torque_pd_passthrough\n"
        << "  connection:\n"
        << "    transport: udp\n"
        << "    interface_hint: robot_lan0\n"
        << "  startup:\n"
        << "    read_initial_state_timeout_ms: 1000\n"
        << "    require_healthy_hardware: true\n"
        << "  safety:\n"
        << "    safe_command_profile: zero_torque\n"
        << "    allow_safe_command_during_shutdown: true\n"
        << "joint_mapping:\n"
        << "  validation:\n"
        << "    expected_sim_joint_count: 1\n"
        << "    expected_hardware_joint_count: 1\n"
        << "    require_dense_enabled_sim_indexes: true\n"
        << "    require_dense_enabled_hardware_indexes: true\n"
        << "    require_unique_enabled_sim_names: true\n"
        << "    require_unique_enabled_hardware_names: true\n"
        << "  entries:\n"
        << "    - sim_index: 0\n"
        << "      hardware_index: 0\n"
        << "      sim_name: left_hip_yaw\n"
        << "      hardware_name: l_hip_yaw\n"
        << "      label: left hip yaw\n"
        << "      enabled: true\n"
        << "      transforms:\n"
        << "        position: {sign: 1.0, offset: 0.0}\n"
        << "        velocity: {sign: 1.0, offset: 0.0}\n"
        << "        effort: {sign: 1.0, offset: 0.0}\n"
        << "        stiffness: {sign: 1.0, offset: 0.0}\n"
        << "        damping: {sign: 1.0, offset: 0.0}\n"
        << "        acceleration: {sign: 1.0, offset: 0.0}\n"
        << "      sim_limits:\n"
        << "        position: {lower: -0.8, upper: 0.8}\n"
        << "      hardware_limits:\n"
        << "        position: {lower: -0.8, upper: 0.8}\n"
        << "    - sim_index: 1\n"
        << "      hardware_index: 1\n"
        << "      sim_name: right_hip_yaw\n"
        << "      hardware_name: r_hip_yaw\n"
        << "      label: right hip yaw\n"
        << "      enabled: false\n"
        << "      transforms:\n"
        << "        position: {sign: 1.0, offset: 0.0}\n"
        << "        velocity: {sign: 1.0, offset: 0.0}\n"
        << "        effort: {sign: 1.0, offset: 0.0}\n"
        << "        stiffness: {sign: 1.0, offset: 0.0}\n"
        << "        damping: {sign: 1.0, offset: 0.0}\n"
        << "        acceleration: {sign: 1.0, offset: 0.0}\n"
        << "      sim_limits:\n"
        << "        position: {lower: -0.8, upper: 0.8}\n"
        << "      hardware_limits:\n"
        << "        position: {lower: -0.8, upper: 0.8}\n";
  }

  const auto result = load_bridge_config(path);
  assert(result.succeeded());
  assert(result.config.has_value());
  assert(result.config->robot_type == "kepler_k2");
  assert(result.config->adapter.joint_names.size() == 1U);
  assert(result.config->adapter.joint_names[0] == "l_hip_yaw");
  assert(result.config->joint_map_entries.size() == 2U);
}

void TestLoadsUnknownRobotKindAsFactoryInput() {
  const std::string path = "bridge_config_unknown_robot_test.yaml";
  {
    std::ofstream output(path);
    output
        << "schema_version: 1\n"
        << "bridge:\n"
        << "  robot_name: demo_robot\n"
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
        << "  kind: demo_robot\n"
        << "  robot_model: prototype\n"
        << "  command_mode: torque_pd_passthrough\n"
        << "  connection:\n"
        << "    transport: udp\n"
        << "    interface_hint: loopback0\n"
        << "  startup:\n"
        << "    read_initial_state_timeout_ms: 1000\n"
        << "    require_healthy_hardware: true\n"
        << "  safety:\n"
        << "    safe_command_profile: zero_torque\n"
        << "    allow_safe_command_during_shutdown: true\n"
        << "joint_mapping:\n"
        << "  validation:\n"
        << "    expected_sim_joint_count: 1\n"
        << "    expected_hardware_joint_count: 1\n"
        << "    require_dense_enabled_sim_indexes: true\n"
        << "    require_dense_enabled_hardware_indexes: true\n"
        << "    require_unique_enabled_sim_names: true\n"
        << "    require_unique_enabled_hardware_names: true\n"
        << "  entries:\n"
        << "    - sim_index: 0\n"
        << "      hardware_index: 0\n"
        << "      sim_name: joint0\n"
        << "      hardware_name: motor0\n"
        << "      label: joint 0\n"
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

  const auto result = load_bridge_config(path);
  assert(result.succeeded());
  assert(result.config.has_value());
  assert(result.config->robot_type == "demo_robot");
  assert(result.config->adapter.kind == "demo_robot");
  assert(result.config->adapter.robot_model == "prototype");
}

void TestLoadsPm01ConfigAsFactoryInput() {
  const auto result = load_bridge_config(WritePm01Config("bridge_config_pm01_test.yaml"));
  assert(result.succeeded());
  assert(result.status == BridgeConfigLoadStatus::kOk);
  assert(result.config.has_value());
  assert(result.config->robot_type == "pm01");
  assert(result.config->adapter.kind == "pm01");
  assert(result.config->adapter.robot_model == "pm01");
  assert(result.config->adapter.command_mode == "joint_bridge");
  assert(result.config->adapter.joint_names.size() == 24U);
  assert(result.config->adapter.joint_names.front() == "l_hip_yaw");
  assert(result.config->adapter.joint_names.back() == "r_elbow");
  assert(result.config->joint_map_entries.size() == 24U);

  const JointMapBuildResult joint_map =
      JointMap::build(result.config->joint_map_entries,
                      result.config->joint_map_validation);
  assert(joint_map.succeeded());
}

void TestRejectsNegativeDdsDomainId() {
  const std::string path = "bridge_config_negative_domain_test.yaml";
  {
    std::ofstream output(path);
    output
        << "schema_version: 1\n"
        << "bridge:\n"
        << "  robot_name: kepler_k2\n"
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
        << "  domain_id: -1\n"
        << "  topics:\n"
      << "    command:\n"
      << "      name: rt/lowcmd\n"
      << "    state:\n"
      << "      name: rt/lowstate\n"
        << "adapter:\n"
        << "  kind: kepler_k2\n"
        << "  robot_model: k2\n"
        << "  command_mode: torque_pd_passthrough\n"
        << "  connection:\n"
        << "    transport: udp\n"
        << "    interface_hint: robot_lan0\n"
        << "  startup:\n"
        << "    read_initial_state_timeout_ms: 1000\n"
        << "    require_healthy_hardware: true\n"
        << "  safety:\n"
        << "    safe_command_profile: zero_torque\n"
        << "    allow_safe_command_during_shutdown: true\n"
        << "joint_mapping:\n"
        << "  validation:\n"
        << "    expected_sim_joint_count: 1\n"
        << "    expected_hardware_joint_count: 1\n"
        << "    require_dense_enabled_sim_indexes: true\n"
        << "    require_dense_enabled_hardware_indexes: true\n"
        << "    require_unique_enabled_sim_names: true\n"
        << "    require_unique_enabled_hardware_names: true\n"
        << "  entries:\n"
        << "    - sim_index: 0\n"
        << "      hardware_index: 0\n"
        << "      sim_name: joint0\n"
        << "      hardware_name: motor0\n"
        << "      label: joint 0\n"
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

  const auto result = load_bridge_config(path);
  assert(!result.succeeded());
  assert(result.status == BridgeConfigLoadStatus::kRejected);
  assert(HasIssueCode(result, BridgeConfigIssueCode::kInvalidValue));
}

void TestLoadsGamepadConfig() {
  const std::string path = WritePm01Config("bridge_config_gamepad_test.yaml");
  {
    std::ofstream output(path, std::ios::app);
    output << "gamepad:\n"
           << "  enabled: true\n"
           << "  backend: sdl\n"
           << "  controller_index: 2\n"
           << "  mapping_path: docs/joy_stick/include/hardware/gamecontrollerdb.txt\n"
           << "  poll_interval_ms: 7\n"
           << "  stale_timeout_ms: 123\n";
  }

  const auto result = load_bridge_config(path);
  assert(result.succeeded());
  assert(result.config.has_value());
  assert(result.config->gamepad.enabled);
  assert(result.config->gamepad.backend == "sdl");
  assert(result.config->gamepad.controller_index == 2);
  assert(result.config->gamepad.mapping_path ==
         "docs/joy_stick/include/hardware/gamecontrollerdb.txt");
  assert(result.config->runner.gamepad_poll_interval_ns == 7'000'000U);
  assert(result.config->runner.gamepad_stale_timeout_ns == 123'000'000U);
}

}

int main() {
  TestLoadsSampleBridgeConfig();
  TestLoadsPm01SampleBridgeConfig();
  TestLoadsUnitreeG1SampleBridgeConfig();
  TestLoadsDebugPrintBlock();
  TestLoadsPm01MotionStateConfig();
  TestLoadsSimplifiedPm01InterfaceHintOverride();
  TestLoadsSimplifiedPm01InterfaceHintDefault();
  TestLoadsSimplifiedKeplerConfig();
  TestRejectsSimplifiedConfigWithoutLoopRate();
  TestRejectsSimplifiedConfigWithoutCommandTopic();
  TestRejectsSimplifiedConfigWithoutStateTopic();
  TestLoadsPm01BackendField();
  TestRejectsUnsupportedFaultPolicyMode();
  TestLoadsConfigWhenDisabledEntryIsOutsideEnabledHardwareWidth();
  TestLoadsUnknownRobotKindAsFactoryInput();
  TestLoadsPm01ConfigAsFactoryInput();
  TestRejectsNegativeDdsDomainId();
  TestLoadsGamepadConfig();
  return 0;
}
