#include "../../../include/bridge/dds/dds_command_ingress.hpp"

#include <chrono>
#include <optional>
#include <string>
#include <utility>

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

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

}

DdsCommandIngress::DdsCommandIngress(DdsCommandIngressDependencies dependencies)
    : dependencies_(std::move(dependencies)) {}

DdsCommandIngress::~DdsCommandIngress() = default;

std::string_view DdsCommandIngress::name() const noexcept {
  return "dds_command_ingress";
}

core::RunnerCallResult DdsCommandIngress::start(
    core::BridgeCommandSink& command_sink,
    core::BridgeRuntimeEventSink& event_sink) {
  command_sink_ = &command_sink;
  event_sink_ = &event_sink;
  if (!dependencies_.runtime || !dependencies_.runtime->command_reader) {
    return fatal_result("dds command ingress requires runtime command reader");
  }
  const auto code = dependencies_.runtime->command_reader->set_listener(this);
  if (code != eprosima::fastdds::dds::RETCODE_OK) {
    return fatal_result("failed to attach dds command reader listener");
  }
  return {
      .status = core::RunnerCallStatus::kOk,
      .detail = "dds command ingress started",
      .fault = std::nullopt,
  };
}

core::RunnerCallResult DdsCommandIngress::stop() {
  if (dependencies_.runtime && dependencies_.runtime->command_reader) {
    dependencies_.runtime->command_reader->set_listener(nullptr);
  }
  return {
      .status = core::RunnerCallStatus::kOk,
      .detail = "dds command ingress stopped",
      .fault = std::nullopt,
  };
}

void DdsCommandIngress::on_data_available(eprosima::fastdds::dds::DataReader* reader) {
  if (reader == nullptr) {
    return;
  }

  bridge_wire::LowCmd message;
  eprosima::fastdds::dds::SampleInfo info;
  while (reader->take_next_sample(&message, &info) == eprosima::fastdds::dds::RETCODE_OK) {
    if (!info.valid_data) {
      continue;
    }
    handle_command(message, now_ns());
  }
}

void DdsCommandIngress::handle_command(const bridge_wire::LowCmd& message,
                                       const std::uint64_t receive_time_ns) {
  if (command_sink_ == nullptr) {
    return;
  }

  const CommandTranslation translated = translate_lowcmd_to_hardware_command(
      message, receive_time_ns, dependencies_.joint_map);
  if (!translated.result.succeeded()) {
    if (event_sink_ != nullptr && translated.result.fault.has_value()) {
      [[maybe_unused]] const auto ignored =
          event_sink_->report_transport_fault(*translated.result.fault);
    }
    return;
  }

  [[maybe_unused]] const auto ignored =
      command_sink_->submit_latest_command(*translated.command);
}

}
