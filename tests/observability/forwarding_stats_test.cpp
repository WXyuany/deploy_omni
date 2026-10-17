#include <cassert>
#include <cstdint>

#include <cmath>

#include "bridge/observability/forwarding_stats.hpp"

namespace {

[[nodiscard]] bool near(const double left, const double right) {
  return std::fabs(left - right) < 0.000001;
}

}

int main() {
  bridge::observability::ForwardingStats stats;

  assert(!stats.record(1'000ULL, 900ULL).has_value());
  assert(!stats.record(500'001'000ULL, 500'000'000ULL).has_value());

  const auto first = stats.record(1'000'001'000ULL, 999'999'000ULL);
  assert(first.has_value());
  assert(first->frequency_hz == 3.0);
  assert(first->event_count == 3ULL);
  assert(first->avg_interval_us == 500'000.0);
  assert(first->min_interval_us == 500'000.0);
  assert(first->max_interval_us == 500'000.0);
  assert(near(first->avg_latency_us, 1.0333333333333334));
  assert(first->max_latency_us == 2.0);

  assert(!stats.record(1'200'001'000ULL, 1'200'000'500ULL)
              .has_value());
  const auto second = stats.record(2'100'001'000ULL, 2'100'000'700ULL);
  assert(second.has_value());
  assert(second->event_count == 2ULL);
  assert(second->frequency_hz == 2.0);
  assert(second->avg_interval_us == 900'000.0);
  assert(second->avg_latency_us == 0.4);
  assert(second->max_latency_us == 0.5);

  return 0;
}
