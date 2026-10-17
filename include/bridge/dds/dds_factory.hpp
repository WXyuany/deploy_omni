#pragma once

#include "dds_facade.hpp"

namespace bridge::dds {

[[nodiscard]] DdsAssembly create_dds_assembly(
    const config::BridgeStartupConfig& startup_config,
    const config::JointMap& joint_map);

}
