#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>

namespace bridge::observability {

struct ForwardingStatsSnapshot {
  std::uint64_t event_count{0};
  double frequency_hz{0.0};
  double avg_interval_us{0.0};
  double min_interval_us{0.0};
  double max_interval_us{0.0};
  double avg_latency_us{0.0};
  double max_latency_us{0.0};
};

class ForwardingStats {
 public:
  [[nodiscard]] std::optional<ForwardingStatsSnapshot> record(
      const std::uint64_t event_time_ns,
      const std::uint64_t source_time_ns = 0) noexcept {
    if (window_start_ns_ == 0) {
      window_start_ns_ = event_time_ns;
    }

    if (last_event_time_ns_ != 0 && event_time_ns >= last_event_time_ns_) {
      const std::uint64_t interval_ns = event_time_ns - last_event_time_ns_;
      interval_count_ += 1;
      interval_sum_ns_ += interval_ns;
      min_interval_ns_ = std::min(min_interval_ns_, interval_ns);
      max_interval_ns_ = std::max(max_interval_ns_, interval_ns);
    }
    last_event_time_ns_ = event_time_ns;

    event_count_ += 1;
    if (source_time_ns != 0 && event_time_ns >= source_time_ns) {
      const std::uint64_t latency_ns = event_time_ns - source_time_ns;
      latency_count_ += 1;
      latency_sum_ns_ += latency_ns;
      max_latency_ns_ = std::max(max_latency_ns_, latency_ns);
    }

    if (event_time_ns - window_start_ns_ < kWindowNs) {
      return std::nullopt;
    }

    ForwardingStatsSnapshot snapshot;
    snapshot.event_count = event_count_;
    snapshot.frequency_hz = static_cast<double>(event_count_);
    if (interval_count_ != 0) {
      snapshot.avg_interval_us = ns_to_us(interval_sum_ns_ / static_cast<double>(interval_count_));
      snapshot.min_interval_us = ns_to_us(static_cast<double>(min_interval_ns_));
      snapshot.max_interval_us = ns_to_us(static_cast<double>(max_interval_ns_));
    }
    if (latency_count_ != 0) {
      snapshot.avg_latency_us = ns_to_us(latency_sum_ns_ / static_cast<double>(latency_count_));
      snapshot.max_latency_us = ns_to_us(static_cast<double>(max_latency_ns_));
    }

    reset_for_next_window(event_time_ns);
    return snapshot;
  }

 private:
  static constexpr std::uint64_t kWindowNs = 1'000'000'000ULL;

  [[nodiscard]] static double ns_to_us(const double value_ns) noexcept {
    return value_ns / 1000.0;
  }

  void reset_for_next_window(const std::uint64_t event_time_ns) noexcept {
    window_start_ns_ = event_time_ns;
    last_event_time_ns_ = 0;
    event_count_ = 0;
    interval_count_ = 0;
    interval_sum_ns_ = 0;
    min_interval_ns_ = std::numeric_limits<std::uint64_t>::max();
    max_interval_ns_ = 0;
    latency_count_ = 0;
    latency_sum_ns_ = 0;
    max_latency_ns_ = 0;
  }

  std::uint64_t window_start_ns_{0};
  std::uint64_t last_event_time_ns_{0};
  std::uint64_t event_count_{0};
  std::uint64_t interval_count_{0};
  std::uint64_t interval_sum_ns_{0};
  std::uint64_t min_interval_ns_{std::numeric_limits<std::uint64_t>::max()};
  std::uint64_t max_interval_ns_{0};
  std::uint64_t latency_count_{0};
  std::uint64_t latency_sum_ns_{0};
  std::uint64_t max_latency_ns_{0};
};

}  // namespace bridge::observability
