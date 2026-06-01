#include "../../../include/bridge/dds/dds_state_egress.hpp"

#include <cstdint>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "unitree/robot/channel/channel_publisher.hpp"

namespace bridge::dds {
namespace {

[[nodiscard]] std::uint64_t now_ns() noexcept {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

[[nodiscard]] core::RunnerCallResult fatal_result(std::string detail) {
  return {
      .status = core::RunnerCallStatus::kFatalError,
      .detail = std::move(detail),
      .fault = std::nullopt,
  };
}

[[nodiscard]] core::RunnerCallResult rejected_result(
    std::string detail,
    std::optional<core::BridgeFault> fault = std::nullopt) {
  return {
      .status = core::RunnerCallStatus::kRejected,
      .detail = std::move(detail),
      .fault = std::move(fault),
  };
}

}  // namespace

class DdsStateEgress::Impl final {
 public:
  using Publisher = unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowState_>;

  bool start(const std::string& topic_name) {
    publisher_ = std::make_unique<Publisher>(topic_name);
    publisher_->InitChannel();
    return true;
  }

  bool publish(const unitree_hg::msg::dds_::LowState_& state) {
    return publisher_ && publisher_->Write(state);
  }

  void stop() {
    if (publisher_) {
      publisher_->CloseChannel();
      publisher_.reset();
    }
  }

 private:
  std::unique_ptr<Publisher> publisher_;
};

DdsStateEgress::DdsStateEgress(DdsStateEgressDependencies dependencies)
    : dependencies_(std::move(dependencies)), impl_(std::make_unique<Impl>()) {}

DdsStateEgress::~DdsStateEgress() = default;

std::string_view DdsStateEgress::name() const noexcept {
  return "dds_state_egress";
}

core::RunnerCallResult DdsStateEgress::start() {
  if (!dependencies_.runtime || !dependencies_.runtime->has_state_writer()) {
    return fatal_result("dds state egress requires initialized runtime");
  }
  if (!impl_->start(dependencies_.runtime->config.state_topic)) {
    return fatal_result("failed to start cyclone state publisher");
  }
  return {.status = core::RunnerCallStatus::kOk,
          .detail = "dds state egress started",
          .fault = std::nullopt};
}

core::RunnerCallResult DdsStateEgress::publish(core::StateFrame& state) {
  const std::uint64_t translate_start_ns = now_ns();
  const StateTranslation lowstate =
      translate_hardware_state_to_lowstate(state, dependencies_.joint_map);
  state.lowstate_translate_duration_ns = now_ns() - translate_start_ns;
  if (!lowstate.result.succeeded()) {
    return rejected_result(lowstate.result.detail, lowstate.result.fault);
  }
  const std::uint64_t write_start_ns = now_ns();
  const bool published = impl_->publish(*lowstate.state);
  state.dds_write_duration_ns = now_ns() - write_start_ns;
  if (!published) {
    return fatal_result("failed to publish cyclone lowstate sample");
  }
  return {.status = core::RunnerCallStatus::kOk,
          .detail = "dds lowstate published",
          .fault = std::nullopt};
}

core::RunnerCallResult DdsStateEgress::stop() {
  impl_->stop();
  return {.status = core::RunnerCallStatus::kOk,
          .detail = "dds state egress stopped",
          .fault = std::nullopt};
}

}  // namespace bridge::dds
