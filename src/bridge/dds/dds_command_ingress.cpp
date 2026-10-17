#include "../../../include/bridge/dds/dds_command_ingress.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "unitree/robot/channel/channel_subscriber.hpp"

namespace bridge::dds {
namespace {

[[nodiscard]] std::uint64_t now_ns() {
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

}  // namespace

class DdsCommandIngress::Impl final {
 public:
  using Subscriber = unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>;

  explicit Impl(DdsCommandIngress& owner) : owner_(owner) {}

  bool start(const std::string& topic_name) {
    subscriber_ = std::make_unique<Subscriber>(topic_name);
    subscriber_->InitChannel(
        [this](const void* message) {
          if (message == nullptr) {
            return;
          }
          owner_.handle_command(
              *static_cast<const unitree_hg::msg::dds_::LowCmd_*>(message), now_ns());
        },
        10);
    return true;
  }

  void stop() {
    if (subscriber_) {
      subscriber_->CloseChannel();
      subscriber_.reset();
    }
  }

 private:
  DdsCommandIngress& owner_;
  std::unique_ptr<Subscriber> subscriber_;
};

DdsCommandIngress::DdsCommandIngress(DdsCommandIngressDependencies dependencies)
    : dependencies_(std::move(dependencies)), impl_(std::make_unique<Impl>(*this)) {}

DdsCommandIngress::~DdsCommandIngress() = default;

std::string_view DdsCommandIngress::name() const noexcept {
  return "dds_command_ingress";
}

core::RunnerCallResult DdsCommandIngress::start(
    core::BridgeCommandSink& command_sink,
    core::BridgeRuntimeEventSink& event_sink) {
  command_sink_ = &command_sink;
  event_sink_ = &event_sink;
  if (!dependencies_.runtime || !dependencies_.runtime->has_command_reader()) {
    return fatal_result("dds command ingress requires initialized runtime");
  }
  if (!impl_->start(dependencies_.runtime->config.command_topic)) {
    return fatal_result("failed to start cyclone command subscriber");
  }
  return {.status = core::RunnerCallStatus::kOk,
          .detail = "dds command ingress started",
          .fault = std::nullopt};
}

core::RunnerCallResult DdsCommandIngress::stop() {
  impl_->stop();
  return {.status = core::RunnerCallStatus::kOk,
          .detail = "dds command ingress stopped",
          .fault = std::nullopt};
}

void DdsCommandIngress::handle_command(const unitree_hg::msg::dds_::LowCmd_& message,
                                       const std::uint64_t receive_time_ns) {
  if (command_sink_ == nullptr) {
    return;
  }
  //std::cout<<"receive dds low cmd!!!!!!!!!!!!"<<std::endl;
  const CommandTranslation translated = translate_lowcmd_to_hardware_command(
      message, receive_time_ns, dependencies_.joint_map);
  if (!translated.result.succeeded()) {
    std::fprintf(stderr,
                 "dds command ingress rejected LowCmd: detail=%s mode_machine=%u crc=%u\n",
                 translated.result.detail.c_str(), message.mode_machine(),
                 message.crc());
    if (event_sink_ != nullptr && translated.result.fault.has_value()) {
      [[maybe_unused]] const auto ignored =
          event_sink_->report_transport_fault(*translated.result.fault);
    }
    return;
  }

  const core::CommandSubmissionResult submission =
      command_sink_->submit_latest_command(*translated.command);
  if (!submission.result.succeeded()) {
    std::fprintf(stderr,
                 "dds command ingress submission rejected: detail=%s sequence=%llu mode=%u\n",
                 submission.result.detail.c_str(),
                 static_cast<unsigned long long>(translated.command->sequence),
                 message.mode_machine());
  }
}

}  // namespace bridge::dds
