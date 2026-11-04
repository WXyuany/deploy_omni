#pragma once

#include <string_view>

#include "types.hpp"

namespace bridge::core {

class BridgeRuntimeEventSink;

class RobotSdkAdapter {
 public:
  virtual ~RobotSdkAdapter() = default;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;

  [[nodiscard]] virtual RobotCapabilitySet capabilities() const noexcept {
    return {};
  }

  virtual AdapterCallResult initialize() = 0;

  virtual void set_runtime_event_sink(BridgeRuntimeEventSink* event_sink) {
    (void)event_sink;
  }

  [[nodiscard]] virtual AdapterReadResult read_state() = 0;

  virtual AdapterCallResult write_command(const CommandFrame& command) = 0;

  [[nodiscard]] virtual AdapterHealthResult query_health() const = 0;

  virtual AdapterCallResult shutdown() = 0;
};

}
