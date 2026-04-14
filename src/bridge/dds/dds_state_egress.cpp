#include "../../../include/bridge/dds/dds_state_egress.hpp"

#include <optional>
#include <string>
#include <utility>

#include <fastdds/dds/publisher/DataWriter.hpp>

namespace bridge::dds {
namespace {

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

}

DdsStateEgress::DdsStateEgress(DdsStateEgressDependencies dependencies)
    : dependencies_(std::move(dependencies)) {}

DdsStateEgress::~DdsStateEgress() = default;

std::string_view DdsStateEgress::name() const noexcept {
  return "dds_state_egress";
}

core::RunnerCallResult DdsStateEgress::start() {
  if (!dependencies_.runtime || !dependencies_.runtime->state_writer ||
      !dependencies_.runtime->status_writer) {
    return fatal_result("dds state egress requires runtime state/status writers");
  }
  return {
      .status = core::RunnerCallStatus::kOk,
      .detail = "dds state egress started",
      .fault = std::nullopt,
  };
}

core::RunnerCallResult DdsStateEgress::publish(const core::StateFrame& state) {
  const StateTranslation lowstate =
      translate_hardware_state_to_lowstate(state, dependencies_.joint_map);
  if (!lowstate.result.succeeded()) {
    return rejected_result(lowstate.result.detail, lowstate.result.fault);
  }

  const StatusTranslation status =
      translate_hardware_state_to_bridge_status(state, dependencies_.joint_map);
  if (!status.result.succeeded()) {
    return rejected_result(status.result.detail, status.result.fault);
  }

  if (dependencies_.runtime->state_writer->write(
          const_cast<bridge_wire::LowState*>(&*lowstate.state)) !=
      eprosima::fastdds::dds::RETCODE_OK) {
    return fatal_result("failed to publish bridge/state sample");
  }
  if (dependencies_.runtime->status_writer->write(
          const_cast<bridge_wire::BridgeStatus*>(&*status.status)) !=
      eprosima::fastdds::dds::RETCODE_OK) {
    return fatal_result("failed to publish bridge/status sample");
  }

  return {
      .status = core::RunnerCallStatus::kOk,
      .detail = "dds state/status published",
      .fault = std::nullopt,
  };
}

core::RunnerCallResult DdsStateEgress::stop() {
  return {
      .status = core::RunnerCallStatus::kOk,
      .detail = "dds state egress stopped",
      .fault = std::nullopt,
  };
}

}
