#pragma once

#include <string_view>

#include "types.hpp"

namespace bridge::core {

class RobotSdkAdapter {
 public:
  virtual ~RobotSdkAdapter() = default;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;

  virtual AdapterCallResult initialize() = 0;

  [[nodiscard]] virtual AdapterReadResult read_state() = 0;

  virtual AdapterCallResult write_command(const CommandFrame& command) = 0;

  virtual AdapterCallResult write_safe_command() = 0;

  [[nodiscard]] virtual AdapterHealthResult query_health() const = 0;

  virtual AdapterCallResult shutdown() = 0;
};

}
