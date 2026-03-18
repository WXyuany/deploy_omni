#include <cassert>
#include <fstream>
#include <string>

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

void TestLoadsSampleBridgeConfig() {
  const auto result = load_bridge_config("../config/kepler_k2.sample.yaml");
  assert(result.succeeded());
  assert(result.status == BridgeConfigLoadStatus::kOk);
  assert(result.config.has_value());

  const auto& config = *result.config;
  assert(config.source_path == "../config/kepler_k2.sample.yaml");
  assert(config.runner.loop_period_ns == 4'000'000U);
  assert(config.runner.command_stale_timeout_ns == 120'000'000U);
  assert(config.adapter.session.robot_id == "kepler_k2");
  assert(config.adapter.session.transport_address == "udp:robot_lan0");
  assert(config.adapter.session.control_profile == "torque_pd_passthrough");
  assert(config.adapter.session.initialize_timeout_ns == 1'000'000'000U);
  assert(config.adapter.session.read_timeout_ns == 4'000'000U);
  assert(config.adapter.session.write_timeout_ns == 4'000'000U);
  assert(config.adapter.safety.safe_command_supported);
  assert(config.adapter.safety.require_initial_state_before_motion);
  assert(config.adapter.safety.allow_command_when_imu_missing);
  assert(config.adapter.joint_names.size() == 12U);
  assert(config.adapter.joint_names.front() == "l_hip_yaw");
  assert(config.adapter.joint_names.back() == "r_ankle_roll");
  assert(config.joint_map_entries.size() == 12U);

  const JointMapBuildResult joint_map =
      JointMap::build(config.joint_map_entries, config.joint_map_validation);
  assert(joint_map.succeeded());
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
  assert(result.config->adapter.joint_names.size() == 1U);
  assert(result.config->adapter.joint_names[0] == "l_hip_yaw");
  assert(result.config->joint_map_entries.size() == 2U);
}

}

int main() {
  TestLoadsSampleBridgeConfig();
  TestRejectsUnsupportedFaultPolicyMode();
  TestLoadsConfigWhenDisabledEntryIsOutsideEnabledHardwareWidth();
  return 0;
}
