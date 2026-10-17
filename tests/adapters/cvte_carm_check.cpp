#include "bridge/adapters/cvte_carm/cvte_carm_adapter.hpp"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

void print_usage(const char* program) {
  std::cout << "Usage: " << program
            << " --model a3|d3 [--ip ADDRESS] [--port PORT]\n"
            << "Connects, reads CArm state, and disconnects without motion.\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string model = "a3";
  std::string ip = "10.42.0.101";
  int port = 8090;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help") {
      print_usage(argv[0]);
      return 0;
    }
    if (argument == "--model" && index + 1 < argc) {
      model = argv[++index];
      continue;
    }
    if (argument == "--ip" && index + 1 < argc) {
      ip = argv[++index];
      continue;
    }
    if (argument == "--port" && index + 1 < argc) {
      port = std::atoi(argv[++index]);
      continue;
    }
    std::cerr << "Unknown or incomplete argument: " << argument << '\n';
    print_usage(argv[0]);
    return 2;
  }

  using bridge::adapters::cvte_carm::CvteCarmAdapter;
  using bridge::adapters::cvte_carm::CvteCarmAdapterConfig;
  using bridge::adapters::cvte_carm::Model;
  using bridge::adapters::cvte_carm::ArmSelection;

  const bool is_a3 = model == "a3";
  const bool is_d3 = model == "d3";
  if (!is_a3 && !is_d3) {
    std::cerr << "--model must be a3 or d3\n";
    return 2;
  }

  CvteCarmAdapterConfig config;
  config.model = is_a3 ? Model::kA3Single : Model::kD3Dual;
  config.arm_selection = is_a3 ? ArmSelection::kSingle : ArmSelection::kBoth;
  config.session.server_ip = ip;
  config.session.server_port = static_cast<std::uint16_t>(port);
  config.session.robot_id = model;
  const std::size_t joint_count = is_a3 ? 6U : 14U;
  config.arm_joint_count = is_a3 ? 0U : joint_count / 2U;
  config.joint_names.reserve(joint_count);
  for (std::size_t index = 0; index < joint_count; ++index) {
    config.joint_names.push_back("joint_" + std::to_string(index));
  }

  CvteCarmAdapter adapter(std::move(config));
  const auto initialized = adapter.initialize();
  std::cout << "initialize: " << initialized.detail << '\n';
  if (!initialized.succeeded()) {
    return 1;
  }

  const auto state = adapter.read_state();
  std::cout << "read_state: " << state.result.detail << '\n';
  if (state.state.has_value()) {
    std::cout << "joint_count=" << state.state->joints.position.size()
              << " valid=" << state.state->valid
              << " degraded=" << state.state->degraded << '\n';
  }
  const auto health = adapter.query_health();
  std::cout << "health: " << health.result.detail
            << " connected=" << health.health.connection_alive << '\n';
  const auto shutdown = adapter.shutdown();
  std::cout << "shutdown: " << shutdown.detail << '\n';
  const bool healthy_state = state.result.succeeded() && state.state.has_value() &&
      state.state->valid && !state.state->stale && !state.state->degraded &&
      state.state->joints.position.size() == joint_count;
  return healthy_state && health.result.succeeded() && health.health.connection_alive &&
      shutdown.succeeded() ? 0 : 1;
}
