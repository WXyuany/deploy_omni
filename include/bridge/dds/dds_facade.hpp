#pragma once

#include <memory>

#include "../config/bridge_config.hpp"
#include "../config/joint_map.hpp"
#include "../core/bridge_runner.hpp"
#include "dds_command_ingress.hpp"
#include "dds_runtime.hpp"
#include "dds_state_egress.hpp"

namespace bridge::dds {

struct DdsAssembly {
  std::shared_ptr<DdsRuntime> runtime;
  std::unique_ptr<core::BridgeCommandIngress> command_ingress;
  std::unique_ptr<core::BridgeStateEgress> state_egress;
};

}
