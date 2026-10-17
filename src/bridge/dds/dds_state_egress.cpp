#include "../../../include/bridge/dds/dds_state_egress.hpp"

#include <cstdint>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/idl/hg/BmsState_.hpp"
#include "unitree/idl/hg/IMUState_.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/idl/hg/MainBoardState_.hpp"

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
  template <typename Message>
  class OptionalPublisher {
   public:
    bool start(const std::string& topic_name) {
      if (topic_name.empty()) {
        return true;
      }
      publisher_ =
          std::make_unique<unitree::robot::ChannelPublisher<Message>>(topic_name);
      publisher_->InitChannel();
      return true;
    }

    bool enabled() const noexcept {
      return static_cast<bool>(publisher_);
    }

    bool publish(const Message& message) {
      return !publisher_ || publisher_->Write(message);
    }

    void stop() {
      if (publisher_) {
        publisher_->CloseChannel();
        publisher_.reset();
      }
    }

   private:
    std::unique_ptr<unitree::robot::ChannelPublisher<Message>> publisher_;
  };

  bool start(const DdsRuntimeConfig& config) {
    return lowstate_publisher_.start(config.state_topic) &&
           imu_publisher_.start(config.imu_state_topic) &&
           bms_publisher_.start(config.bms_state_topic) &&
           mainboard_publisher_.start(config.mainboard_state_topic);
  }

  bool publish(const unitree_hg::msg::dds_::LowState_& state) {
    return lowstate_publisher_.publish(state);
  }

  bool publish_imu(const unitree_hg::msg::dds_::IMUState_& state) {
    return imu_publisher_.publish(state);
  }

  bool publish_bms(const unitree_hg::msg::dds_::BmsState_& state) {
    return bms_publisher_.publish(state);
  }

  bool publish_mainboard(const unitree_hg::msg::dds_::MainBoardState_& state) {
    return mainboard_publisher_.publish(state);
  }

  bool imu_enabled() const noexcept {
    return imu_publisher_.enabled();
  }

  bool bms_enabled() const noexcept {
    return bms_publisher_.enabled();
  }

  bool mainboard_enabled() const noexcept {
    return mainboard_publisher_.enabled();
  }

  void stop() {
    mainboard_publisher_.stop();
    bms_publisher_.stop();
    imu_publisher_.stop();
    lowstate_publisher_.stop();
  }

 private:
  OptionalPublisher<unitree_hg::msg::dds_::LowState_> lowstate_publisher_;
  OptionalPublisher<unitree_hg::msg::dds_::IMUState_> imu_publisher_;
  OptionalPublisher<unitree_hg::msg::dds_::BmsState_> bms_publisher_;
  OptionalPublisher<unitree_hg::msg::dds_::MainBoardState_> mainboard_publisher_;
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
  if (!impl_->start(dependencies_.runtime->config)) {
    return fatal_result("failed to start cyclone state publisher");
  }
  return {.status = core::RunnerCallStatus::kOk,
          .detail = "dds state egress started",
          .fault = std::nullopt};
}

core::RunnerCallResult DdsStateEgress::publish(
    core::StateFrame& state, const core::RobotAuxStateSnapshot& aux_state) {
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
  if (impl_->imu_enabled()) {
    core::StateFrame imu_source = state;
    if (aux_state.imu.has_value()) {
      imu_source.imu = aux_state.imu;
    }
    const ImuStateTranslation imu = translate_state_to_imu_state(imu_source);
    if (!imu.result.succeeded()) {
      return rejected_result(imu.result.detail, imu.result.fault);
    }
    if (!impl_->publish_imu(*imu.state)) {
      return fatal_result("failed to publish cyclone imu_state sample");
    }
  }
  if (impl_->bms_enabled() && aux_state.bms.has_value()) {
    const BmsStateTranslation bms =
        translate_bms_sample_to_bms_state(*aux_state.bms);
    if (!bms.result.succeeded()) {
      return rejected_result(bms.result.detail, bms.result.fault);
    }
    if (!impl_->publish_bms(*bms.state)) {
      return fatal_result("failed to publish cyclone bms_state sample");
    }
  }
  if (impl_->mainboard_enabled() && aux_state.mainboard.has_value()) {
    const MainBoardStateTranslation mainboard =
        translate_mainboard_sample_to_mainboard_state(*aux_state.mainboard);
    if (!mainboard.result.succeeded()) {
      return rejected_result(mainboard.result.detail, mainboard.result.fault);
    }
    if (!impl_->publish_mainboard(*mainboard.state)) {
      return fatal_result("failed to publish cyclone mainboard_state sample");
    }
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
