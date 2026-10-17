#pragma once

#include <memory>
#include <string_view>

#include "dds_runtime.hpp"
#include "dds_translation.hpp"
#include "../config/joint_map.hpp"
#include "../core/bridge_runner.hpp"

namespace bridge::dds {

struct DdsStateEgressDependencies {
  std::shared_ptr<DdsRuntime> runtime;
  config::JointMap joint_map;
};

class DdsStateEgress final : public core::BridgeStateEgress {
 public:
  explicit DdsStateEgress(DdsStateEgressDependencies dependencies);
  ~DdsStateEgress() override;

  DdsStateEgress(const DdsStateEgress&) = delete;
  DdsStateEgress& operator=(const DdsStateEgress&) = delete;
  DdsStateEgress(DdsStateEgress&&) = delete;
  DdsStateEgress& operator=(DdsStateEgress&&) = delete;

  [[nodiscard]] std::string_view name() const noexcept override;

  core::RunnerCallResult start() override;

  core::RunnerCallResult publish(
      core::StateFrame& state,
      const core::RobotAuxStateSnapshot& aux_state) override;

  core::RunnerCallResult stop() override;

 private:
  class Impl;

  DdsStateEgressDependencies dependencies_;
  std::unique_ptr<Impl> impl_;
};

}
