#pragma once

#include <cstdint>
#include <memory>
#include <string_view>

#include "unitree/idl/hg/LowCmd_.hpp"

#include "dds_runtime.hpp"
#include "dds_translation.hpp"
#include "../config/joint_map.hpp"
#include "../core/bridge_runner.hpp"

namespace bridge::dds {

struct DdsCommandIngressDependencies {
  std::shared_ptr<DdsRuntime> runtime;
  config::JointMap joint_map;
};

class DdsCommandIngress final : public core::BridgeCommandIngress {
 public:
  explicit DdsCommandIngress(DdsCommandIngressDependencies dependencies);
  ~DdsCommandIngress() override;

  DdsCommandIngress(const DdsCommandIngress&) = delete;
  DdsCommandIngress& operator=(const DdsCommandIngress&) = delete;
  DdsCommandIngress(DdsCommandIngress&&) = delete;
  DdsCommandIngress& operator=(DdsCommandIngress&&) = delete;

  [[nodiscard]] std::string_view name() const noexcept override;

  core::RunnerCallResult start(core::BridgeCommandSink& command_sink,
                               core::BridgeRuntimeEventSink& event_sink) override;

  core::RunnerCallResult stop() override;

 private:
  class Impl;

  void handle_command(const unitree_hg::msg::dds_::LowCmd_& message,
                      std::uint64_t receive_time_ns);

  DdsCommandIngressDependencies dependencies_;
  core::BridgeCommandSink* command_sink_{nullptr};
  core::BridgeRuntimeEventSink* event_sink_{nullptr};
  std::unique_ptr<Impl> impl_;
};

}
