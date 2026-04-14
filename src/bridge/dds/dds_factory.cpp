#include "../../../include/bridge/dds/dds_factory.hpp"

#include <memory>

namespace bridge::dds {

DdsAssembly create_dds_assembly(const config::BridgeStartupConfig& startup_config,
                                const config::JointMap& joint_map) {
  auto runtime = make_dds_runtime(DdsRuntimeConfig{
      .domain_id = startup_config.dds.domain_id,
      .command_topic = startup_config.dds.topics.command,
      .state_topic = startup_config.dds.topics.state,
      .status_topic = startup_config.dds.topics.status,
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
