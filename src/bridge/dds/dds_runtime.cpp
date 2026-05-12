#include "dds_runtime_internal.hpp"

#include <utility>

#include "unitree/robot/channel/channel_factory.hpp"

namespace bridge::dds {

DdsRuntime::DdsRuntime() : impl_(std::make_unique<DdsRuntimeImpl>()) {}

DdsRuntime::~DdsRuntime() = default;

DdsRuntime::DdsRuntime(DdsRuntime&&) noexcept = default;

DdsRuntime& DdsRuntime::operator=(DdsRuntime&&) noexcept = default;

bool DdsRuntime::ready() const noexcept {
  return impl_ && impl_->initialized;
}

bool DdsRuntime::has_command_reader() const noexcept {
  return impl_ && impl_->initialized && impl_->command_channel_available;
}

bool DdsRuntime::has_state_writer() const noexcept {
  return impl_ && impl_->initialized && impl_->state_channel_available;
}

bool DdsRuntime::has_status_writer() const noexcept {
  return false;
}

DdsRuntimeImpl& runtime_impl(DdsRuntime& runtime) noexcept {
  return *runtime.impl_;
}

const DdsRuntimeImpl& runtime_impl(const DdsRuntime& runtime) noexcept {
  return *runtime.impl_;
}

std::shared_ptr<DdsRuntime> make_dds_runtime(DdsRuntimeConfig config) {
  auto runtime = std::make_shared<DdsRuntime>();
  runtime->config = std::move(config);
  DdsRuntimeImpl& impl = runtime_impl(*runtime);

  try {
    if (runtime->config.network_interface.empty()) {
      unitree::robot::ChannelFactory::Instance()->Init(
          static_cast<int32_t>(runtime->config.domain_id));
    } else {
      unitree::robot::ChannelFactory::Instance()->Init(
          static_cast<int32_t>(runtime->config.domain_id),
          runtime->config.network_interface);
    }
  } catch (...) {
    runtime->config.network_interface.clear();
    unitree::robot::ChannelFactory::Instance()->Init(
        static_cast<int32_t>(runtime->config.domain_id));
  }
  impl.initialized = true;
  impl.command_channel_available = true;
  impl.state_channel_available = true;
  return runtime;
}

}  // namespace bridge::dds
