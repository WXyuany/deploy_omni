#include <cassert>

#include "../../include/bridge/adapters/agibot_x2/agibot_x2_adapter.hpp"

int main() {
  using namespace bridge::adapters::agibot_x2;
  AgibotX2AdapterConfig config;
  config.joint_names.resize(kAgibotX2JointCount, "joint");
  AgibotX2Adapter adapter(config);
  assert(adapter.name() == kAgibotX2AdapterName);
  assert(adapter.config().joint_count() == kAgibotX2JointCount);
  assert(adapter.capabilities().contains(bridge::core::RobotCapability::kJointState));
  const auto initialized = adapter.initialize();
#if defined(BRIDGE_AGIBOT_X2_ROS2_AVAILABLE) && BRIDGE_AGIBOT_X2_ROS2_AVAILABLE
  (void)initialized;
#else
  assert(!initialized.succeeded());
#endif
  assert(adapter.shutdown().succeeded());
  return 0;
}
