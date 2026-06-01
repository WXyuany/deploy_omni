#include <chrono>
#include <cstdio>
#include <cstdint>
#include <string>
#include <string_view>
#include <thread>

#include "../../include/bridge/app/robot_assembly_factory.hpp"
#include "../../include/bridge/config/bridge_config.hpp"
#include "../../include/bridge/core/bridge_runner.hpp"

namespace {

struct RunnerFrequencyLogState {
  std::uint64_t window_start_ns{0};
  std::uint64_t cycle_count{0};
};

[[nodiscard]] int to_exit_code(const bridge::core::RunnerCallResult& result,
                               const int success_code,
                               const int failure_code) {
  return result.succeeded() ? success_code : failure_code;
}

[[nodiscard]] std::uint64_t monotonic_now_ns() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

[[nodiscard]] bool is_waiting_detail(const std::string& detail) {
  return std::string_view(detail).rfind("waiting for ", 0) == 0;
}

void print_failure(const char* const stage, const std::string& detail) {
  if (detail.empty()) {
    std::fprintf(stderr, "adapter_bridge %s failed\n", stage);
    return;
  }
  std::fprintf(stderr, "adapter_bridge %s failed: %s\n", stage, detail.c_str());
}

[[nodiscard]] std::optional<std::uint64_t> update_runner_frequency_log_state(
    RunnerFrequencyLogState& state, const std::uint64_t now_ns) {
  if (state.window_start_ns == 0) {
    state.window_start_ns = now_ns;
  }

  ++state.cycle_count;
  if (now_ns - state.window_start_ns < 1'000'000'000ULL) {
    return std::nullopt;
  }

  const std::uint64_t count = state.cycle_count;
  state.window_start_ns = now_ns;
  state.cycle_count = 0;
  return count;
}

[[nodiscard]] bridge::core::RunnerCallResult run_bridge_loop(
    bridge::core::BridgeRunner& runner, const std::uint64_t loop_period_ns) {
  auto next_deadline = std::chrono::steady_clock::now();
  std::uint64_t wait_cycles_until_log = 0;
  RunnerFrequencyLogState frequency_state;
  const std::uint64_t wait_log_interval_cycles =
      loop_period_ns == 0 ? 250 : std::max<std::uint64_t>(1, 1000000000ULL / loop_period_ns);
  const bool event_triggered_lowstate =
      runner.config().lowstate_trigger == bridge::core::LowStateTriggerMode::kEvent;

  while (true) {
    const bridge::core::RunnerCallResult result = runner.run();
    if (!result.succeeded()) {
      return result;
    }

    if (is_waiting_detail(result.detail)) {
      if (wait_cycles_until_log == 0) {
        std::fprintf(stderr, "%s\n", result.detail.c_str());
        wait_cycles_until_log = wait_log_interval_cycles;
      } else {
        --wait_cycles_until_log;
      }
    } else {
      wait_cycles_until_log = 0;
    }

    const auto current_status = runner.status();
    if (const auto count = update_runner_frequency_log_state(
            frequency_state, monotonic_now_ns());
        count.has_value()) {
      std::fprintf(stderr, "runner frequency: %llu Hz\n",
                   static_cast<unsigned long long>(*count));
    }

    if (event_triggered_lowstate) {
      (void)runner.wait_for_state_available_until(
          std::chrono::steady_clock::now() + std::chrono::hours(24));
      continue;
    }

    if (loop_period_ns == 0) {
      continue;
    }

    next_deadline += std::chrono::nanoseconds(loop_period_ns);
    std::this_thread::sleep_until(next_deadline);
  }
}

}

int main(int argc, char** argv) {
  const std::string config_path =
      argc > 1 ? std::string(argv[1]) : std::string("config/kepler_k2.sample.yaml");
  const bridge::config::BridgeConfigLoadResult loaded =
      bridge::config::load_bridge_config(config_path);
  if (!loaded.succeeded()) {
    print_failure("config load", loaded.detail);
    return loaded.status == bridge::config::BridgeConfigLoadStatus::kIoError ? 1 : 2;
  }

  bridge::app::RobotAssemblyCreateResult assembly =
      bridge::app::create_robot_assembly(*loaded.config);
  if (!assembly.succeeded()) {
    print_failure("robot assembly", assembly.detail);
    return assembly.status == bridge::app::RobotAssemblyCreateStatus::kUnsupportedRobot
               ? 3
               : 4;
  }

  bridge::core::BridgeRunner runner(loaded.config->runner,
                                    assembly.assembly->runner_dependencies());
  assembly.assembly->adapter->set_runtime_event_sink(&runner);

  const bridge::core::RunnerCallResult initialize = runner.initialize();
  if (!initialize.succeeded()) {
    print_failure("initialize", initialize.detail);
    return to_exit_code(initialize, 0, 5);
  }

  const bridge::core::RunnerCallResult run =
      run_bridge_loop(runner, loaded.config->runner.loop_period_ns);
  const bridge::core::RunnerCallResult stop_request = runner.request_stop();
  const bridge::core::RunnerCallResult shutdown = runner.shutdown();

  if (!run.succeeded()) {
    print_failure("run", run.detail);
    return to_exit_code(run, 0, 6);
  }
  if (!stop_request.succeeded()) {
    print_failure("stop request", stop_request.detail);
    return to_exit_code(stop_request, 0, 7);
  }
  if (!shutdown.succeeded()) {
    print_failure("shutdown", shutdown.detail);
  }
  return to_exit_code(shutdown, 0, 8);
}
