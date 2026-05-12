#include <cassert>
#include <cstdint>
#include <optional>

struct RunnerFrequencyLogState {
  std::uint64_t window_start_ns{0};
  std::uint64_t cycle_count{0};
};

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

int main() {
  RunnerFrequencyLogState state;

  const auto first = update_runner_frequency_log_state(state, 100);
  assert(!first.has_value());

  const auto second = update_runner_frequency_log_state(state, 500'000'000ULL);
  assert(!second.has_value());

  const auto third =
      update_runner_frequency_log_state(state, 1'000'000'100ULL);
  assert(third.has_value());
  assert(*third == 3ULL);

  const auto fourth = update_runner_frequency_log_state(state, 1'200'000'000ULL);
  assert(!fourth.has_value());

  const auto fifth = update_runner_frequency_log_state(state, 2'100'000'100ULL);
  assert(fifth.has_value());
  assert(*fifth == 2ULL);

  return 0;
}
