#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace bridge::dds {

struct DdsRuntimeConfig {
  std::uint32_t domain_id{0};
  std::string command_topic{"bridge/command"};
  std::string state_topic{"bridge/state"};
  std::string imu_state_topic;
  std::string bms_state_topic;
  std::string mainboard_state_topic;
  std::string fault_code_topic;
  std::string status_topic{"bridge/status"};
  std::string network_interface;
};

struct DdsRuntimeImpl;

class DdsRuntime {
 public:
  DdsRuntime();
  ~DdsRuntime();

  DdsRuntime(const DdsRuntime&) = delete;
  DdsRuntime& operator=(const DdsRuntime&) = delete;
  DdsRuntime(DdsRuntime&&) noexcept;
  DdsRuntime& operator=(DdsRuntime&&) noexcept;

  [[nodiscard]] bool ready() const noexcept;
  [[nodiscard]] bool has_command_reader() const noexcept;
  [[nodiscard]] bool has_state_writer() const noexcept;
  [[nodiscard]] bool has_status_writer() const noexcept;

  DdsRuntimeConfig config;

 private:
  std::unique_ptr<DdsRuntimeImpl> impl_;

  friend DdsRuntimeImpl& runtime_impl(DdsRuntime& runtime) noexcept;
  friend const DdsRuntimeImpl& runtime_impl(const DdsRuntime& runtime) noexcept;
};

[[nodiscard]] std::shared_ptr<DdsRuntime> make_dds_runtime(
    DdsRuntimeConfig config = {});

}
