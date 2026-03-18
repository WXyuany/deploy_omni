#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "../../include/bridge/adapters/kepler_k2/kepler_k2_adapter.hpp"
#include "../../include/bridge/config/bridge_config.hpp"
#include "../../include/bridge/config/joint_map.hpp"
#include "../../include/bridge/core/bridge_runner.hpp"
#include "../../include/bridge/dds/transport.hpp"

namespace {

[[nodiscard]] bridge::core::BridgeFault make_mapping_fault(
    const std::string& message) {
  return {
      .category = bridge::core::FaultCategory::kInvalidCommand,
      .severity = bridge::core::FaultSeverity::kRecoverable,
      .lifecycle_state = bridge::core::LifecycleState::kCreated,
      .message = message,
      .latched = false,
      .sequence = 0,
      .timestamp_ns = 0,
  };
}

[[nodiscard]] std::optional<std::string> first_projection_error(
    const std::vector<bridge::config::JointMapIssue>& issues) {
  for (const bridge::config::JointMapIssue& issue : issues) {
    if (issue.severity == bridge::config::JointMapIssueSeverity::kError) {
      return issue.message;
    }
  }
  return std::nullopt;
}

class LocalCodecBackedIngress final : public bridge::core::BridgeCommandIngress,
                                      public bridge::dds::DdsCommandSampleSink {
 public:
  LocalCodecBackedIngress(bridge::dds::DdsCommandDecoder& decoder,
                          const bridge::config::JointMap& joint_map)
      : decoder_(decoder), joint_map_(joint_map) {}

  [[nodiscard]] std::string_view name() const noexcept override {
    return "local_codec_backed_ingress";
  }

  bridge::core::RunnerCallResult start(
      bridge::core::BridgeCommandSink& command_sink,
      bridge::core::BridgeRuntimeEventSink& event_sink) override {
    command_sink_ = &command_sink;
    event_sink_ = &event_sink;
    started_ = true;
    return {
        .status = bridge::core::RunnerCallStatus::kOk,
        .detail = std::string("assembled ingress with decoder ") +
                  std::string(decoder_.name()),
        .fault = std::nullopt,
    };
  }

  bridge::core::RunnerCallResult stop() override {
    started_ = false;
    command_sink_ = nullptr;
    event_sink_ = nullptr;
    return {
        .status = bridge::core::RunnerCallStatus::kOk,
        .detail = "local ingress stopped",
        .fault = std::nullopt,
    };
  }

  void on_command_sample(bridge::dds::DdsCommandSample sample) override {
    if (!started_ || command_sink_ == nullptr || event_sink_ == nullptr) {
      return;
    }

    bridge::dds::CommandDecodeResult decoded = decoder_.decode(sample);
    if (!decoded.accepted()) {
      if (decoded.fault.has_value()) {
        event_sink_->report_transport_fault(*decoded.fault);
      }
      return;
    }

    bridge::config::CommandProjectionResult projected =
        joint_map_.project_command_sim_to_hardware(*decoded.command);
    const std::optional<std::string> projection_error =
        first_projection_error(projected.issues);
    if (!projected.command.has_value() || projection_error.has_value()) {
      event_sink_->report_transport_fault(
          make_mapping_fault(projection_error.value_or(
              std::string("joint-map command projection failed"))));
      return;
    }

    command_sink_->submit_latest_command(std::move(*projected.command));
  }

  void on_transport_fault(bridge::dds::TransportCallResult result) override {
    if (event_sink_ == nullptr) {
      return;
    }
    if (result.fault.has_value()) {
      event_sink_->report_transport_fault(std::move(*result.fault));
    }
  }

 private:
  bridge::dds::DdsCommandDecoder& decoder_;
  const bridge::config::JointMap& joint_map_;
  bridge::core::BridgeCommandSink* command_sink_{nullptr};
  bridge::core::BridgeRuntimeEventSink* event_sink_{nullptr};
  bool started_{false};
};

class LocalCodecBackedEgress final : public bridge::core::BridgeStateEgress {
 public:
  LocalCodecBackedEgress(bridge::dds::DdsStateEncoder& encoder,
                         const bridge::config::JointMap& joint_map)
      : encoder_(encoder), joint_map_(joint_map) {}

  [[nodiscard]] std::string_view name() const noexcept override {
    return "local_codec_backed_egress";
  }

  bridge::core::RunnerCallResult start() override {
    started_ = true;
    publications_.clear();
    return {
        .status = bridge::core::RunnerCallStatus::kOk,
        .detail = std::string("assembled egress with encoder ") +
                  std::string(encoder_.name()),
        .fault = std::nullopt,
    };
  }

  bridge::core::RunnerCallResult publish(
      const bridge::core::StateFrame& state) override {
    if (!started_) {
      return {
          .status = bridge::core::RunnerCallStatus::kRejected,
          .detail = "local egress must be started before publish",
          .fault = std::nullopt,
      };
    }

    bridge::config::StateProjectionResult projected =
        joint_map_.project_state_hardware_to_sim(state);
    const std::optional<std::string> projection_error =
        first_projection_error(projected.issues);
    if (!projected.state.has_value() || projection_error.has_value()) {
      return {
          .status = bridge::core::RunnerCallStatus::kRejected,
          .detail = projection_error.value_or("joint-map state projection failed"),
          .fault = std::nullopt,
      };
    }

    bridge::dds::StateEncodeResult encoded = encoder_.encode(*projected.state);
    publications_ = std::move(encoded.publications);
    return bridge::dds::to_runner_call_result(std::move(encoded.result));
  }

  bridge::core::RunnerCallResult stop() override {
    started_ = false;
    publications_.clear();
    return {
        .status = bridge::core::RunnerCallStatus::kOk,
        .detail = "local egress stopped",
        .fault = std::nullopt,
    };
  }

 private:
  bridge::dds::DdsStateEncoder& encoder_;
  const bridge::config::JointMap& joint_map_;
  bool started_{false};
  std::vector<bridge::dds::DdsPublication> publications_;
};

[[nodiscard]] int to_exit_code(const bridge::core::RunnerCallResult& result,
                               const int success_code,
                               const int failure_code) {
  return result.succeeded() ? success_code : failure_code;
}

}

int main(int argc, char** argv) {
  const std::string config_path =
      argc > 1 ? std::string(argv[1]) : std::string("config/kepler_k2.sample.yaml");
  const bridge::config::BridgeConfigLoadResult loaded =
      bridge::config::load_bridge_config(config_path);
  if (!loaded.succeeded()) {
    return loaded.status == bridge::config::BridgeConfigLoadStatus::kIoError ? 1 : 2;
  }

  const bridge::config::JointMapBuildResult joint_map_result =
      bridge::config::JointMap::build(loaded.config->joint_map_entries,
                                      loaded.config->joint_map_validation);
  if (!joint_map_result.succeeded() || !joint_map_result.joint_map.has_value()) {
    return 3;
  }

  std::unique_ptr<bridge::dds::DdsCommandDecoder> command_decoder =
      bridge::dds::make_lowcmd_codec();
  std::unique_ptr<bridge::dds::DdsStateEncoder> state_encoder =
      bridge::dds::make_lowstate_codec();
  bridge::adapters::kepler_k2::KeplerK2Adapter adapter(loaded.config->adapter);
  LocalCodecBackedIngress ingress(*command_decoder, *joint_map_result.joint_map);
  LocalCodecBackedEgress egress(*state_encoder, *joint_map_result.joint_map);

  bridge::core::BridgeRunner runner(
      loaded.config->runner,
      {.adapter = &adapter, .command_ingress = &ingress, .state_egress = &egress});

  const bridge::core::RunnerCallResult initialize = runner.initialize();
  if (!initialize.succeeded()) {
    return to_exit_code(initialize, 0, 1);
  }

  const bridge::core::RunnerCallResult run = runner.run();
  const bridge::core::RunnerCallResult stop_request = runner.request_stop();
  const bridge::core::RunnerCallResult shutdown = runner.shutdown();

  if (!run.succeeded()) {
    return to_exit_code(run, 0, 2);
  }
  if (!stop_request.succeeded()) {
    return to_exit_code(stop_request, 0, 3);
  }
  return to_exit_code(shutdown, 0, 4);
}
