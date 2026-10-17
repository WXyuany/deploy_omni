#pragma once

#include "../../../include/bridge/dds/dds_runtime.hpp"

namespace bridge::dds {

struct DdsRuntimeImpl {
  bool initialized{false};
  bool command_channel_available{false};
  bool state_channel_available{false};
};

[[nodiscard]] DdsRuntimeImpl& runtime_impl(DdsRuntime& runtime) noexcept;
[[nodiscard]] const DdsRuntimeImpl& runtime_impl(const DdsRuntime& runtime) noexcept;

}
