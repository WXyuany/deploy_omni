#include <cassert>

#include "../../include/bridge/adapters/unitree_g1/unitree_g1_sdk.hpp"

int main() {
  bridge::adapters::unitree_g1::UnitreeG1SdkFacade sdk;
  assert(!sdk.initialized());
  assert(sdk.capabilities().size() == 5U);

  int fsm_id = 0;
  const auto rejected = sdk.get_fsm_id(&fsm_id);
  assert(!rejected.succeeded());
  assert(rejected.code == bridge::adapters::unitree_g1::kUnitreeG1SdkNotInitialized);

  assert(sdk.shutdown().succeeded());
  assert(!sdk.initialized());
  return 0;
}
