#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"

int main() {
  unitree_hg::msg::dds_::LowCmd_ command;
  command.mode_machine(2U);

  unitree_hg::msg::dds_::LowState_ state;
  state.tick(1U);

  return (command.mode_machine() == 2U && state.tick() == 1U) ? 0 : 1;
}
