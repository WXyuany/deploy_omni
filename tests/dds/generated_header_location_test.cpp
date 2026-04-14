#include "../../idl/bridge/BridgeMessages.hpp"

int main() {
  bridge_wire::LowCmd command;
  command.seq_id(1U);
  command.mode_label("probe");
  return command.seq_id() == 1U ? 0 : 1;
}
