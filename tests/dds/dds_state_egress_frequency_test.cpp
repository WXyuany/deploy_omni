#include <cassert>
#include <cstdint>
#include <optional>

namespace {

struct DdsStateEgressFrequencyLogState {
  std::uint64_t window_start_ns{0};
  std::uint64_t publish_count{0};
};

[[nodiscard]] std::optional<std::uint64_t> update_dds_state_egress_frequency_log(
    DdsStateEgressFrequencyLogState& state, const std::uint64_t now_ns) {
  if (state.window_start_ns == 0) {
    state.window_start_ns = now_ns;
  }

  ++state.publish_count;
  if (now_ns - state.window_start_ns < 1'000'000'000ULL) {
    return std::nullopt;
  }

  const std::uint64_t count = state.publish_count;
  state.window_start_ns = now_ns;
  state.publish_count = 0;
  return count;
}

}

int main() {
  DdsStateEgressFrequencyLogState state;

  assert(!update_dds_state_egress_frequency_log(state, 100).has_value());
  assert(!update_dds_state_egress_frequency_log(state, 500'000'000ULL)
              .has_value());

  const auto first_frequency =
      update_dds_state_egress_frequency_log(state, 1'000'000'100ULL);
  assert(first_frequency.has_value());
  assert(*first_frequency == 3ULL);

  assert(!update_dds_state_egress_frequency_log(state, 1'200'000'000ULL)
              .has_value());

  const auto second_frequency =
      update_dds_state_egress_frequency_log(state, 2'100'000'100ULL);
  assert(second_frequency.has_value());
  assert(*second_frequency == 2ULL);

  return 0;
}
