#include <cassert>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "bridge/build_config.hpp"
#include "../../include/bridge/adapters/pm01/pm01_adapter.hpp"

namespace {

using bridge::adapters::pm01::Pm01Adapter;
using bridge::adapters::pm01::Pm01AdapterConfig;
using bridge::core::AdapterCallStatus;
using bridge::core::FaultCategory;


template <typename Adapter, typename = void>
struct HasPublicUpdateMotionTask : std::false_type {};

template <typename Adapter>
struct HasPublicUpdateMotionTask<
    Adapter,
    std::void_t<decltype(std::declval<Adapter&>().update_motion_task(
        std::declval<std::optional<std::string>>()))>> : std::true_type {};

template <typename Config, typename = void>
struct HasPublicBackendField : std::false_type {};

template <typename Config>
struct HasPublicBackendField<Config, std::void_t<decltype(std::declval<Config&>().backend)>>
    : std::true_type {};

template <typename Config, typename = void>
struct HasPublicTransportEndpointField : std::false_type {};

template <typename Config>
struct HasPublicTransportEndpointField<
    Config,
    std::void_t<decltype(std::declval<Config&>().session.transport_endpoint)>>
    : std::true_type {};

static_assert(!HasPublicUpdateMotionTask<Pm01Adapter>::value);
static_assert(!HasPublicBackendField<Pm01AdapterConfig>::value);
static_assert(!HasPublicTransportEndpointField<Pm01AdapterConfig>::value);

Pm01AdapterConfig MakeBaseConfig() {
  Pm01AdapterConfig config;
  config.session.robot_id = "pm01-test";
  config.session.control_profile = "joint_bridge";
  config.session.initialize_timeout_ns = 10;
  config.session.read_timeout_ns = 20;
  config.session.write_timeout_ns = 30;
  config.motion_state.required = true;
  config.motion_state.required_task = "joint_bridge";
  config.publish_default_imu_when_missing = true;
  config.joint_names.resize(24);
  for (std::size_t index = 0; index < config.joint_names.size(); ++index) {
    config.joint_names[index] = "joint_" + std::to_string(index);
    config.dds_joint_indices.push_back(static_cast<std::int64_t>(index));
  }
  return config;
}

void TestConfigValidationFailsWhenDdsJointIndicesWidthIsWrong() {
  Pm01AdapterConfig config = MakeBaseConfig();
  config.dds_joint_indices.pop_back();

  Pm01Adapter adapter(config);
  const auto init = adapter.initialize();
  assert(init.status == AdapterCallStatus::kFatalError);
  assert(init.fault.has_value());
  assert(init.fault->category == FaultCategory::kConfiguration);
}

void TestConfigValidationFailsWhenJointNamesWidthIsWrong() {
  Pm01AdapterConfig config = MakeBaseConfig();
  config.joint_names.pop_back();

  Pm01Adapter adapter(config);
  const auto init = adapter.initialize();
  assert(init.status == AdapterCallStatus::kFatalError);
  assert(init.fault.has_value());
  assert(init.fault->category == FaultCategory::kConfiguration);
}

#if !BRIDGE_PM01_REAL_BACKEND_AVAILABLE
void TestPm01RequiresRos2BackendSupport() {
  Pm01Adapter adapter(MakeBaseConfig());
  const auto init = adapter.initialize();
  assert(init.status == AdapterCallStatus::kFatalError);
  assert(init.fault.has_value());
  assert(init.fault->category == FaultCategory::kConfiguration);
}
#endif

#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
void TestRos2AdapterCanInitializeAndShutdown() {
  Pm01Adapter adapter(MakeBaseConfig());
  const auto init = adapter.initialize();
  assert(init.status == AdapterCallStatus::kOk);

  const auto health = adapter.query_health();
  assert(health.result.status == AdapterCallStatus::kOk);
  assert(health.health.connection_alive);

  const auto shutdown = adapter.shutdown();
  assert(shutdown.status == AdapterCallStatus::kOk);
}

void TestRos2AdapterCanBeDestroyedWithoutExplicitShutdown() {
  Pm01Adapter adapter(MakeBaseConfig());
  const auto init = adapter.initialize();
  assert(init.status == AdapterCallStatus::kOk);
}
#endif

}  // namespace

int main() {
  TestConfigValidationFailsWhenDdsJointIndicesWidthIsWrong();
  TestConfigValidationFailsWhenJointNamesWidthIsWrong();
#if !BRIDGE_PM01_REAL_BACKEND_AVAILABLE
  TestPm01RequiresRos2BackendSupport();
#endif
#if BRIDGE_PM01_REAL_BACKEND_AVAILABLE
  TestRos2AdapterCanInitializeAndShutdown();
  TestRos2AdapterCanBeDestroyedWithoutExplicitShutdown();
#endif
  return 0;
}
