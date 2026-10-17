#include "../../../include/bridge/dds/dds_factory.hpp"

#include <memory>

namespace bridge::dds {

DdsAssembly create_dds_assembly(const config::BridgeStartupConfig& startup_config,
                                const config::JointMap& joint_map) {
  auto runtime = make_dds_runtime(DdsRuntimeConfig{
      .domain_id = startup_config.dds.domain_id,
      .command_topic = startup_config.dds.topics.command,
      .state_topic = startup_config.dds.topics.state,
      .imu_state_topic = startup_config.dds.topics.imu_state,
      .bms_state_topic = startup_config.dds.topics.bms_state,
      .mainboard_state_topic = startup_config.dds.topics.mainboard_state,
      .fault_code_topic = startup_config.dds.topics.fault_code,
      .status_topic = startup_config.dds.topics.status,
      .network_interface = startup_config.adapter.connection.interface_hint,
  });

  DdsAssembly assembly;
  assembly.runtime = runtime;
  assembly.command_ingress = std::make_unique<DdsCommandIngress>(
      DdsCommandIngressDependencies{
          .runtime = runtime,
          .joint_map = joint_map,
      });
  assembly.state_egress = std::make_unique<DdsStateEgress>(
      DdsStateEgressDependencies{
          .runtime = runtime,
          .joint_map = joint_map,
      });
  return assembly;
}

}
