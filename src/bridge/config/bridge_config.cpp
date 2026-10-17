#include "../../../include/bridge/config/bridge_config.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bridge::config {
namespace {

struct ParsedLine {
  std::size_t indent{0};
  std::string text;
};

enum class TopSection {
  kNone,
  kBridge,
  kDds,
  kDebugPrint,
  kGamepad,
  kAssembly,
  kAdapter,
  kJointMapping,
};

enum class BridgeSubsection {
  kNone,
  kFaultPolicy,
};

enum class DdsSubsection {
  kNone,
  kTopics,
};

enum class DdsTopicSelection {
  kNone,
  kCommand,
  kState,
  kImuState,
  kBmsState,
  kMainboardState,
  kFaultCode,
  kStatus,
};

enum class AdapterSubsection {
  kNone,
  kConnection,
  kStartup,
  kMotionState,
  kTopics,
};

enum class AssemblySubsection {
  kNone,
  kAdapter,
  kCommandIngress,
  kStateEgress,
  kModules,
};

enum class JointMappingSubsection {
  kNone,
  kValidation,
  kEntries,
};

enum class EntrySubsection {
  kNone,
  kTransforms,
  kSimLimits,
  kHardwareLimits,
};

struct RawBridgeConfig {
  std::optional<int> schema_version;
  std::optional<std::string> robot_name;
  std::optional<int> loop_rate_hz;
  std::optional<std::string> lowstate_trigger;
  std::optional<int> stale_command_timeout_ms;
  std::optional<std::string> fault_mode;
  std::optional<bool> publish_degraded_state_while_faulted;
  std::optional<bool> halt_command_writes_while_faulted;
  std::optional<std::string> adapter_kind;
  std::optional<std::string> robot_model;
  std::optional<std::string> command_mode;
  std::optional<int> dds_domain_id;
  std::optional<std::string> dds_command_topic;
  std::optional<std::string> dds_state_topic;
  std::optional<std::string> dds_imu_state_topic;
  std::optional<std::string> dds_bms_state_topic;
  std::optional<std::string> dds_mainboard_state_topic;
  std::optional<std::string> dds_fault_code_topic;
  std::optional<std::string> dds_status_topic;
  std::optional<bool> debug_print_state;
  std::optional<bool> debug_print_command;
  std::optional<bool> debug_print_forwarding_log;
  std::optional<int> debug_print_rate_hz;
  std::optional<bool> gamepad_enabled;
  std::optional<std::string> gamepad_backend;
  std::optional<int> gamepad_controller_index;
  std::optional<std::string> gamepad_mapping_path;
  std::optional<int> gamepad_poll_interval_ms;
  std::optional<int> gamepad_stale_timeout_ms;
  std::optional<std::string> assembly_profile;
  std::optional<bool> assembly_adapter_enabled;
  std::optional<std::string> assembly_adapter_type;
  std::optional<bool> assembly_command_ingress_enabled;
  std::optional<std::string> assembly_command_ingress_type;
  std::optional<bool> assembly_state_egress_enabled;
  std::optional<std::string> assembly_state_egress_type;
  std::vector<AssemblyModuleConfig> assembly_modules;
  std::optional<std::string> connection_transport;
  std::optional<std::string> connection_interface_hint;
  std::optional<int> read_initial_state_timeout_ms;
  std::optional<bool> require_healthy_hardware;
  std::optional<bool> motion_state_required;
  std::optional<std::string> motion_state_required_task;
  std::optional<std::string> motion_state_initial_task;
  std::optional<std::string> adapter_command_topic;
  std::optional<std::string> adapter_state_topic;
  std::optional<std::string> adapter_imu_topic;
  std::optional<std::string> adapter_bms_state_topic;
  std::optional<std::string> adapter_mainboard_state_topic;
  JointMapValidationOptions joint_map_validation;
  std::vector<JointMapEntry> joint_map_entries;
};

[[nodiscard]] std::size_t configured_hardware_width(
    const std::vector<JointMapEntry>& entries) {
  std::size_t width = 0;
  for (const JointMapEntry& entry : entries) {
    width = std::max(width, entry.hardware_index + 1);
  }
  return width;
}

[[nodiscard]] std::size_t configured_sim_width(
    const std::vector<JointMapEntry>& entries) {
  std::size_t width = 0;
  for (const JointMapEntry& entry : entries) {
    width = std::max(width, entry.sim_index + 1);
  }
  return width;
}

void apply_default_joint_transform(JointMapEntry* entry, const double sign,
                                   const double offset) {
  entry->transforms.position = {.sign = sign, .offset = offset};
  entry->transforms.velocity = {.sign = sign, .offset = 0.0};
  entry->transforms.effort = {.sign = sign, .offset = 0.0};
  entry->transforms.stiffness = {.sign = 1.0, .offset = 0.0};
  entry->transforms.damping = {.sign = 1.0, .offset = 0.0};
  entry->transforms.acceleration = {.sign = sign, .offset = 0.0};
}

[[nodiscard]] std::string trim(std::string value) {
  const auto not_space = [](const unsigned char character) {
    return !std::isspace(character);
  };

  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(),
              value.end());
  return value;
}

[[nodiscard]] bool starts_with(const std::string& text,
                               const std::string_view prefix) {
  return text.rfind(prefix, 0) == 0;
}

[[nodiscard]] std::optional<std::pair<std::string, std::string>> split_key_value(
    const std::string& text) {
  const std::size_t delimiter = text.find(':');
  if (delimiter == std::string::npos) {
    return std::nullopt;
  }

  return std::pair<std::string, std::string>{trim(text.substr(0, delimiter)),
                                             trim(text.substr(delimiter + 1))};
}

[[nodiscard]] std::optional<bool> parse_bool(const std::string& text) {
  if (text == "true") {
    return true;
  }
  if (text == "false") {
    return false;
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<int> parse_int(const std::string& text) {
  try {
    std::size_t consumed = 0;
    const int value = std::stoi(text, &consumed);
    if (consumed != text.size()) {
      return std::nullopt;
    }
    return value;
  } catch (...) {
    return std::nullopt;
  }
}

[[nodiscard]] std::optional<double> parse_double(const std::string& text) {
  try {
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != text.size()) {
      return std::nullopt;
    }
    return value;
  } catch (...) {
    return std::nullopt;
  }
}

[[nodiscard]] std::optional<std::unordered_map<std::string, std::string>>
parse_inline_map(const std::string& text) {
  if (text.size() < 2 || text.front() != '{' || text.back() != '}') {
    return std::nullopt;
  }

  std::unordered_map<std::string, std::string> result;
  std::stringstream stream(text.substr(1, text.size() - 2));
  std::string item;
  while (std::getline(stream, item, ',')) {
    const auto key_value = split_key_value(trim(item));
    if (!key_value.has_value()) {
      return std::nullopt;
    }
    result.emplace(key_value->first, key_value->second);
  }
  return result;
}

void push_issue(std::vector<BridgeConfigIssue>* issues,
                const BridgeConfigIssueCode code, std::string message) {
  issues->push_back({.code = code, .message = std::move(message)});
}

[[nodiscard]] std::string first_issue_detail(
    const std::vector<BridgeConfigIssue>& issues,
    const std::string& fallback) {
  if (issues.empty()) {
    return fallback;
  }
  return issues.front().message;
}

template <typename T>
void assign_required(std::optional<T>* destination, const std::string& field_name,
                     const std::string& raw_value,
                     std::vector<BridgeConfigIssue>* issues,
                     const BridgeConfigIssueCode invalid_code) {
  static_assert(sizeof(T) == 0, "assign_required must be specialized");
}

template <>
void assign_required<int>(std::optional<int>* destination,
                          const std::string& field_name,
                          const std::string& raw_value,
                          std::vector<BridgeConfigIssue>* issues,
                          const BridgeConfigIssueCode invalid_code) {
  const std::optional<int> parsed = parse_int(raw_value);
  if (!parsed.has_value()) {
    push_issue(issues, invalid_code,
               field_name + " must be an integer value");
    return;
  }
  *destination = *parsed;
}

template <>
void assign_required<bool>(std::optional<bool>* destination,
                           const std::string& field_name,
                           const std::string& raw_value,
                           std::vector<BridgeConfigIssue>* issues,
                           const BridgeConfigIssueCode invalid_code) {
  const std::optional<bool> parsed = parse_bool(raw_value);
  if (!parsed.has_value()) {
    push_issue(issues, invalid_code,
               field_name + " must be true or false");
    return;
  }
  *destination = *parsed;
}

void assign_bool_value(bool* destination, const std::string& field_name,
                       const std::string& raw_value,
                       std::vector<BridgeConfigIssue>* issues,
                       const BridgeConfigIssueCode invalid_code) {
  const std::optional<bool> parsed = parse_bool(raw_value);
  if (!parsed.has_value()) {
    push_issue(issues, invalid_code,
               field_name + " must be true or false");
    return;
  }
  *destination = *parsed;
}

void assign_optional_size(std::optional<std::size_t>* destination,
                          const std::string& field_name,
                          const std::string& raw_value,
                          std::vector<BridgeConfigIssue>* issues,
                          const BridgeConfigIssueCode invalid_code) {
  const std::optional<int> parsed = parse_int(raw_value);
  if (!parsed.has_value() || *parsed < 0) {
    push_issue(issues, invalid_code,
               field_name + " must be a non-negative integer value");
    return;
  }
  *destination = static_cast<std::size_t>(*parsed);
}

[[nodiscard]] std::optional<JointFieldTransform> parse_transform(
    const std::string& raw_value) {
  const auto values = parse_inline_map(raw_value);
  if (!values.has_value()) {
    return std::nullopt;
  }
  const auto sign_it = values->find("sign");
  const auto offset_it = values->find("offset");
  if (sign_it == values->end() || offset_it == values->end()) {
    return std::nullopt;
  }

  const std::optional<double> sign = parse_double(sign_it->second);
  const std::optional<double> offset = parse_double(offset_it->second);
  if (!sign.has_value() || !offset.has_value()) {
    return std::nullopt;
  }

  return JointFieldTransform{.sign = *sign, .offset = *offset};
}

[[nodiscard]] std::optional<JointValueRange> parse_range(
    const std::string& raw_value) {
  const auto values = parse_inline_map(raw_value);
  if (!values.has_value()) {
    return std::nullopt;
  }
  const auto lower_it = values->find("lower");
  const auto upper_it = values->find("upper");
  if (lower_it == values->end() || upper_it == values->end()) {
    return std::nullopt;
  }

  const std::optional<double> lower = parse_double(lower_it->second);
  const std::optional<double> upper = parse_double(upper_it->second);
  if (!lower.has_value() || !upper.has_value()) {
    return std::nullopt;
  }

  return JointValueRange{.lower = *lower, .upper = *upper};
}

void assign_transform_field(JointFieldTransforms* transforms,
                            const std::string& key,
                            const JointFieldTransform& transform) {
  if (key == "position") {
    transforms->position = transform;
  } else if (key == "velocity") {
    transforms->velocity = transform;
  } else if (key == "effort") {
    transforms->effort = transform;
  } else if (key == "stiffness") {
    transforms->stiffness = transform;
  } else if (key == "damping") {
    transforms->damping = transform;
  } else if (key == "acceleration") {
    transforms->acceleration = transform;
  }
}

void assign_limit_field(JointFieldLimits* limits, const std::string& key,
                        const JointValueRange& range) {
  if (key == "position") {
    limits->position = range;
  } else if (key == "velocity") {
    limits->velocity = range;
  } else if (key == "effort") {
    limits->effort = range;
  } else if (key == "stiffness") {
    limits->stiffness = range;
  } else if (key == "damping") {
    limits->damping = range;
  } else if (key == "acceleration") {
    limits->acceleration = range;
  }
}

[[nodiscard]] std::optional<std::vector<ParsedLine>> load_lines(
    const std::string& file_path, std::vector<BridgeConfigIssue>* issues) {
  std::ifstream input(file_path);
  if (!input.is_open()) {
    push_issue(issues, BridgeConfigIssueCode::kIoError,
               "unable to open config file: " + file_path);
    return std::nullopt;
  }

  std::vector<ParsedLine> lines;
  std::string line;
  while (std::getline(input, line)) {
    const std::size_t first = line.find_first_not_of(' ');
    if (first == std::string::npos) {
      continue;
    }

    std::string text = trim(line.substr(first));
    if (text.empty()) {
      continue;
    }
    lines.push_back({.indent = first, .text = std::move(text)});
  }
  return lines;
}

[[nodiscard]] bool parse_lines(const std::vector<ParsedLine>& lines,
                               RawBridgeConfig* raw,
                               std::vector<BridgeConfigIssue>* issues) {
  TopSection top = TopSection::kNone;
  BridgeSubsection bridge_subsection = BridgeSubsection::kNone;
  DdsSubsection dds_subsection = DdsSubsection::kNone;
  DdsTopicSelection dds_topic_selection = DdsTopicSelection::kNone;
  AssemblySubsection assembly_subsection = AssemblySubsection::kNone;
  AdapterSubsection adapter_subsection = AdapterSubsection::kNone;
  JointMappingSubsection joint_mapping_subsection = JointMappingSubsection::kNone;
  EntrySubsection entry_subsection = EntrySubsection::kNone;
  JointMapEntry* current_entry = nullptr;
  AssemblyModuleConfig* current_assembly_module = nullptr;

  for (const ParsedLine& line : lines) {
    if (line.indent == 0) {
      bridge_subsection = BridgeSubsection::kNone;
      dds_subsection = DdsSubsection::kNone;
      dds_topic_selection = DdsTopicSelection::kNone;
      assembly_subsection = AssemblySubsection::kNone;
      adapter_subsection = AdapterSubsection::kNone;
      joint_mapping_subsection = JointMappingSubsection::kNone;
      entry_subsection = EntrySubsection::kNone;
      current_entry = nullptr;
      current_assembly_module = nullptr;

      if (line.text == "bridge:") {
        top = TopSection::kBridge;
        continue;
      }
      if (line.text == "dds:") {
        top = TopSection::kDds;
        continue;
      }
      if (line.text == "debug_print:") {
        top = TopSection::kDebugPrint;
        continue;
      }
      if (line.text == "gamepad:") {
        top = TopSection::kGamepad;
        continue;
      }
      if (line.text == "assembly:") {
        top = TopSection::kAssembly;
        continue;
      }
      if (line.text == "adapter:") {
        top = TopSection::kAdapter;
        continue;
      }
      if (line.text == "joint_mapping:") {
        top = TopSection::kJointMapping;
        continue;
      }

      if (starts_with(trim(line.text), "#")) {
        continue;
      }

      const auto key_value = split_key_value(line.text);
      if (key_value.has_value() && key_value->first == "schema_version") {
        assign_required<int>(&raw->schema_version, "schema_version",
                             key_value->second, issues,
                             BridgeConfigIssueCode::kInvalidValue);
        continue;
      }
      if (key_value.has_value() && key_value->first == "robot_name") {
        raw->robot_name = key_value->second;
        continue;
      }
      if (key_value.has_value() && (key_value->first == "api_rate_hz" || key_value->first == "loop_rate_hz")) {
        assign_required<int>(&raw->loop_rate_hz, key_value->first, key_value->second,
                             issues, BridgeConfigIssueCode::kInvalidValue);
        continue;
      }
      if (key_value.has_value() && key_value->first == "transport") {
        raw->connection_transport = key_value->second;
        continue;
      }
      if (key_value.has_value() && key_value->first == "interface_hint") {
        raw->connection_interface_hint = key_value->second;
        continue;
      }
      if (key_value.has_value() && key_value->first == "robot_kind") {
        raw->adapter_kind = key_value->second;
        if (!raw->robot_name.has_value()) {
          raw->robot_name = key_value->second;
        }
        continue;
      }
      if (key_value.has_value() && key_value->first == "stale_command_timeout_ms") {
        assign_required<int>(&raw->stale_command_timeout_ms,
                             "stale_command_timeout_ms", key_value->second, issues,
                             BridgeConfigIssueCode::kInvalidValue);
        continue;
      }
      if (key_value.has_value() && key_value->first == "dds") {
        top = TopSection::kDds;
        continue;
      }
      if (key_value.has_value() && key_value->first == "joints") {
        top = TopSection::kJointMapping;
        joint_mapping_subsection = JointMappingSubsection::kEntries;
        continue;
      }

      push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
                 "unsupported top-level config line: " + line.text);
      continue;
    }

    if (top == TopSection::kBridge) {
      if (line.indent == 2) {
        bridge_subsection = BridgeSubsection::kNone;
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid bridge line: " + line.text);
          continue;
        }

        if (key_value->second.empty()) {
          if (key_value->first == "fault_policy") {
            bridge_subsection = BridgeSubsection::kFaultPolicy;
            continue;
          }
        }

        if (key_value->first == "robot_name") {
          raw->robot_name = key_value->second;
        } else if (key_value->first == "loop_rate_hz") {
          assign_required<int>(&raw->loop_rate_hz, "bridge.loop_rate_hz",
                               key_value->second, issues,
                               BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "lowstate_trigger") {
          raw->lowstate_trigger = key_value->second;
        } else if (key_value->first == "stale_command_timeout_ms") {
          assign_required<int>(&raw->stale_command_timeout_ms,
                               "bridge.stale_command_timeout_ms",
                               key_value->second, issues,
                               BridgeConfigIssueCode::kInvalidValue);
        }
        continue;
      }

      if (line.indent == 4 &&
          bridge_subsection != BridgeSubsection::kNone) {
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid bridge subsection line: " + line.text);
          continue;
        }

        if (bridge_subsection == BridgeSubsection::kFaultPolicy) {
          if (key_value->first == "mode") {
            raw->fault_mode = key_value->second;
          } else if (key_value->first == "publish_degraded_state_while_faulted") {
            assign_required<bool>(&raw->publish_degraded_state_while_faulted,
                                  "bridge.fault_policy.publish_degraded_state_while_faulted",
                                  key_value->second, issues,
                                  BridgeConfigIssueCode::kInvalidValue);
          } else if (key_value->first == "halt_command_writes_while_faulted") {
            assign_required<bool>(&raw->halt_command_writes_while_faulted,
                                  "bridge.fault_policy.halt_command_writes_while_faulted",
                                  key_value->second, issues,
                                  BridgeConfigIssueCode::kInvalidValue);
          }
        }
      }
      continue;
    }

    if (top == TopSection::kDds) {
      if (line.indent == 2) {
        dds_subsection = DdsSubsection::kNone;
        dds_topic_selection = DdsTopicSelection::kNone;
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid dds line: " + line.text);
          continue;
        }

        if (key_value->second.empty() && key_value->first == "topics") {
          dds_subsection = DdsSubsection::kTopics;
          continue;
        }

        if (key_value->first == "domain_id" && !key_value->second.empty()) {
          assign_required<int>(&raw->dds_domain_id, "dds.domain_id",
                               key_value->second, issues,
                               BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "command_topic" && !key_value->second.empty()) {
          raw->dds_command_topic = key_value->second;
        } else if (key_value->first == "state_topic" && !key_value->second.empty()) {
          raw->dds_state_topic = key_value->second;
        } else if (key_value->first == "imu_state" && !key_value->second.empty()) {
          raw->dds_imu_state_topic = key_value->second;
        } else if (key_value->first == "bms_state" && !key_value->second.empty()) {
          raw->dds_bms_state_topic = key_value->second;
        } else if (key_value->first == "mainboard_state" && !key_value->second.empty()) {
          raw->dds_mainboard_state_topic = key_value->second;
        } else if (key_value->first == "fault_code" && !key_value->second.empty()) {
          raw->dds_fault_code_topic = key_value->second;
        } else if (key_value->first == "command" && !key_value->second.empty()) {
          raw->dds_command_topic = key_value->second;
        } else if (key_value->first == "state" && !key_value->second.empty()) {
          raw->dds_state_topic = key_value->second;
        } else if (key_value->first == "status" && !key_value->second.empty()) {
          raw->dds_status_topic = key_value->second;
        }
        continue;
      }

      if (line.indent == 4 && dds_subsection == DdsSubsection::kTopics) {
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid dds.topics line: " + line.text);
          continue;
        }

        if (key_value->first == "command") {
          dds_topic_selection = DdsTopicSelection::kCommand;
        } else if (key_value->first == "state") {
          dds_topic_selection = DdsTopicSelection::kState;
        } else if (key_value->first == "imu_state") {
          dds_topic_selection = DdsTopicSelection::kImuState;
        } else if (key_value->first == "bms_state") {
          dds_topic_selection = DdsTopicSelection::kBmsState;
        } else if (key_value->first == "mainboard_state") {
          dds_topic_selection = DdsTopicSelection::kMainboardState;
        } else if (key_value->first == "fault_code") {
          dds_topic_selection = DdsTopicSelection::kFaultCode;
        } else if (key_value->first == "status") {
          dds_topic_selection = DdsTopicSelection::kStatus;
        }
        continue;
      }

      if (line.indent == 6 && dds_subsection == DdsSubsection::kTopics) {
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid dds topic detail line: " + line.text);
          continue;
        }

        if (key_value->first == "name") {
          if (dds_topic_selection == DdsTopicSelection::kCommand) {
            raw->dds_command_topic = key_value->second;
          } else if (dds_topic_selection == DdsTopicSelection::kState) {
            raw->dds_state_topic = key_value->second;
          } else if (dds_topic_selection == DdsTopicSelection::kImuState) {
            raw->dds_imu_state_topic = key_value->second;
          } else if (dds_topic_selection == DdsTopicSelection::kBmsState) {
            raw->dds_bms_state_topic = key_value->second;
          } else if (dds_topic_selection == DdsTopicSelection::kMainboardState) {
            raw->dds_mainboard_state_topic = key_value->second;
          } else if (dds_topic_selection == DdsTopicSelection::kFaultCode) {
            raw->dds_fault_code_topic = key_value->second;
          } else if (dds_topic_selection == DdsTopicSelection::kStatus) {
            raw->dds_status_topic = key_value->second;
          }
        }
        continue;
      }

      continue;
    }

    if (top == TopSection::kDebugPrint) {
      if (line.indent == 2) {
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid debug_print line: " + line.text);
          continue;
        }

        if (key_value->first == "state") {
          assign_required<bool>(&raw->debug_print_state, "debug_print.state",
                                key_value->second, issues,
                                BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "cmd") {
          assign_required<bool>(&raw->debug_print_command, "debug_print.cmd",
                                key_value->second, issues,
                                BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "forwarding_log") {
          assign_required<bool>(&raw->debug_print_forwarding_log,
                                "debug_print.forwarding_log", key_value->second,
                                issues, BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "rate_hz") {
          assign_required<int>(&raw->debug_print_rate_hz,
                               "debug_print.rate_hz", key_value->second,
                               issues, BridgeConfigIssueCode::kInvalidValue);
        }
      }
      continue;
    }

    if (top == TopSection::kGamepad) {
      if (line.indent == 2) {
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid gamepad line: " + line.text);
          continue;
        }

        if (key_value->first == "enabled") {
          assign_required<bool>(&raw->gamepad_enabled, "gamepad.enabled",
                                key_value->second, issues,
                                BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "backend") {
          raw->gamepad_backend = key_value->second;
        } else if (key_value->first == "controller_index") {
          assign_required<int>(&raw->gamepad_controller_index,
                               "gamepad.controller_index", key_value->second,
                               issues, BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "mapping_path") {
          raw->gamepad_mapping_path = key_value->second;
        } else if (key_value->first == "poll_interval_ms") {
          assign_required<int>(&raw->gamepad_poll_interval_ms,
                               "gamepad.poll_interval_ms", key_value->second,
                               issues, BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "stale_timeout_ms") {
          assign_required<int>(&raw->gamepad_stale_timeout_ms,
                               "gamepad.stale_timeout_ms", key_value->second,
                               issues, BridgeConfigIssueCode::kInvalidValue);
        }
      }
      continue;
    }

    if (top == TopSection::kAssembly) {
      if (line.indent == 2) {
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid assembly line: " + line.text);
          continue;
        }

        assembly_subsection = AssemblySubsection::kNone;
        if (key_value->first == "profile") {
          raw->assembly_profile = key_value->second;
        } else if (key_value->second.empty() && key_value->first == "adapter") {
          assembly_subsection = AssemblySubsection::kAdapter;
        } else if (key_value->second.empty() && key_value->first == "command_ingress") {
          assembly_subsection = AssemblySubsection::kCommandIngress;
        } else if (key_value->second.empty() && key_value->first == "state_egress") {
          assembly_subsection = AssemblySubsection::kStateEgress;
        } else if (key_value->second.empty() && key_value->first == "modules") {
          assembly_subsection = AssemblySubsection::kModules;
        }
        continue;
      }

      if (line.indent == 4 && assembly_subsection == AssemblySubsection::kModules &&
          starts_with(line.text, "- ")) {
        raw->assembly_modules.emplace_back();
        current_assembly_module = &raw->assembly_modules.back();
        const auto key_value = split_key_value(line.text.substr(2));
        if (key_value.has_value()) {
          if (key_value->first == "name") {
            current_assembly_module->name = key_value->second;
          } else if (key_value->first == "type") {
            current_assembly_module->type = key_value->second;
          } else if (key_value->first == "enabled") {
            const std::optional<bool> enabled = parse_bool(key_value->second);
            if (enabled.has_value()) {
              current_assembly_module->enabled = *enabled;
            } else {
              push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                         "assembly.modules.enabled must be true or false");
            }
          }
        }
        continue;
      }

      if (line.indent == 6 && assembly_subsection == AssemblySubsection::kModules &&
          current_assembly_module != nullptr) {
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid assembly module line: " + line.text);
          continue;
        }

        if (key_value->first == "name") {
          current_assembly_module->name = key_value->second;
        } else if (key_value->first == "type") {
          current_assembly_module->type = key_value->second;
        } else if (key_value->first == "enabled") {
          const std::optional<bool> enabled = parse_bool(key_value->second);
          if (enabled.has_value()) {
            current_assembly_module->enabled = *enabled;
          } else {
            push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                       "assembly.modules.enabled must be true or false");
          }
        } else if (key_value->first == "controller_index") {
          const std::optional<int> controller_index = parse_int(key_value->second);
          if (controller_index.has_value()) {
            current_assembly_module->controller_index = *controller_index;
          } else {
            push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                       "assembly.modules.controller_index must be an integer value");
          }
        } else if (key_value->first == "mapping_path") {
          current_assembly_module->mapping_path = key_value->second;
        } else if (key_value->first == "poll_interval_ms") {
          const std::optional<int> poll_interval_ms = parse_int(key_value->second);
          if (poll_interval_ms.has_value()) {
            current_assembly_module->poll_interval_ms = *poll_interval_ms;
          } else {
            push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                       "assembly.modules.poll_interval_ms must be an integer value");
          }
        } else if (key_value->first == "stale_timeout_ms") {
          const std::optional<int> stale_timeout_ms = parse_int(key_value->second);
          if (stale_timeout_ms.has_value()) {
            current_assembly_module->stale_timeout_ms = *stale_timeout_ms;
          } else {
            push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                       "assembly.modules.stale_timeout_ms must be an integer value");
          }
        }
        continue;
      }

      if (line.indent == 4 && assembly_subsection != AssemblySubsection::kNone &&
          assembly_subsection != AssemblySubsection::kModules) {
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid assembly component line: " + line.text);
          continue;
        }

        std::optional<bool>* enabled = nullptr;
        std::optional<std::string>* type = nullptr;
        if (assembly_subsection == AssemblySubsection::kAdapter) {
          enabled = &raw->assembly_adapter_enabled;
          type = &raw->assembly_adapter_type;
        } else if (assembly_subsection == AssemblySubsection::kCommandIngress) {
          enabled = &raw->assembly_command_ingress_enabled;
          type = &raw->assembly_command_ingress_type;
        } else if (assembly_subsection == AssemblySubsection::kStateEgress) {
          enabled = &raw->assembly_state_egress_enabled;
          type = &raw->assembly_state_egress_type;
        }

        if (key_value->first == "enabled" && enabled != nullptr) {
          assign_required<bool>(enabled, "assembly.enabled", key_value->second,
                                issues, BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "type" && type != nullptr) {
          *type = key_value->second;
        }
        continue;
      }

      continue;
    }

    if (top == TopSection::kAdapter) {
      if (line.indent == 2) {
        adapter_subsection = AdapterSubsection::kNone;
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid adapter line: " + line.text);
          continue;
        }

        if (key_value->second.empty()) {
          if (key_value->first == "connection") {
            adapter_subsection = AdapterSubsection::kConnection;
            continue;
          }
          if (key_value->first == "startup") {
            adapter_subsection = AdapterSubsection::kStartup;
            continue;
          }
          if (key_value->first == "motion_state") {
            adapter_subsection = AdapterSubsection::kMotionState;
            continue;
          }
          if (key_value->first == "topics") {
            adapter_subsection = AdapterSubsection::kTopics;
            continue;
          }
        }

        if (key_value->first == "kind") {
          raw->adapter_kind = key_value->second;
        } else if (key_value->first == "robot_model") {
          raw->robot_model = key_value->second;
        } else if (key_value->first == "command_mode") {
          raw->command_mode = key_value->second;
        }
        continue;
      }

      if (line.indent == 4 && adapter_subsection != AdapterSubsection::kNone) {
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid adapter subsection line: " + line.text);
          continue;
        }

        if (adapter_subsection == AdapterSubsection::kConnection) {
          if (key_value->first == "transport") {
            raw->connection_transport = key_value->second;
          } else if (key_value->first == "interface_hint") {
            raw->connection_interface_hint = key_value->second;
          }
        }

        if (adapter_subsection == AdapterSubsection::kStartup) {
          if (key_value->first == "read_initial_state_timeout_ms") {
            assign_required<int>(&raw->read_initial_state_timeout_ms,
                                 "adapter.startup.read_initial_state_timeout_ms",
                                 key_value->second, issues,
                                 BridgeConfigIssueCode::kInvalidValue);
          } else if (key_value->first == "require_healthy_hardware") {
            assign_required<bool>(&raw->require_healthy_hardware,
                                  "adapter.startup.require_healthy_hardware",
                                  key_value->second, issues,
                                  BridgeConfigIssueCode::kInvalidValue);
          }
        }

        if (adapter_subsection == AdapterSubsection::kMotionState) {
          if (key_value->first == "required") {
            assign_required<bool>(&raw->motion_state_required,
                                  "adapter.motion_state.required",
                                  key_value->second, issues,
                                  BridgeConfigIssueCode::kInvalidValue);
          } else if (key_value->first == "required_task") {
            raw->motion_state_required_task = key_value->second;
          } else if (key_value->first == "initial_task") {
            raw->motion_state_initial_task = key_value->second;
          }
        }

        if (adapter_subsection == AdapterSubsection::kTopics) {
          if (key_value->first == "command") {
            raw->adapter_command_topic = key_value->second;
          } else if (key_value->first == "state") {
            raw->adapter_state_topic = key_value->second;
          } else if (key_value->first == "imu") {
            raw->adapter_imu_topic = key_value->second;
          } else if (key_value->first == "bms_state") {
            raw->adapter_bms_state_topic = key_value->second;
          } else if (key_value->first == "mainboard_state") {
            raw->adapter_mainboard_state_topic = key_value->second;
          }
        }
      }
      continue;
    }

    if (top == TopSection::kJointMapping) {
      if (line.indent == 2 && starts_with(line.text, "- ")) {
        joint_mapping_subsection = JointMappingSubsection::kEntries;
        raw->joint_map_entries.emplace_back();
        current_entry = &raw->joint_map_entries.back();
        entry_subsection = EntrySubsection::kNone;
        current_entry->enabled = true;

        std::string entry_content = trim(line.text.substr(2));

        if (starts_with(entry_content, "{")) {
          const auto inline_map = parse_inline_map(entry_content);
          if (inline_map.has_value()) {
            for (const auto& [key, value] : *inline_map) {
              if (key == "index") {
                const std::optional<int> idx = parse_int(value);
                if (idx.has_value() && *idx >= 0) {
                  current_entry->sim_index = static_cast<std::size_t>(*idx);
                  current_entry->hardware_index = static_cast<std::size_t>(*idx);
                }
              } else if (key == "enable") {
                const std::optional<bool> en = parse_bool(value);
                if (en.has_value()) {
                  current_entry->enabled = *en;
                }
              } else if (key == "sign") {
                const std::optional<double> s = parse_double(value);
                if (s.has_value()) {
                  apply_default_joint_transform(current_entry, *s,
                                                current_entry->transforms.position.offset);
                }
              } else if (key == "offset") {
                const std::optional<double> o = parse_double(value);
                if (o.has_value()) {
                  apply_default_joint_transform(current_entry,
                                                current_entry->transforms.position.sign,
                                                *o);
                }
              } else if (key == "name") {
                current_entry->sim_name = value;
                current_entry->hardware_name = value;
                current_entry->label = value;
              }
            }
          }
          continue;
        }

        const auto key_value = split_key_value(entry_content);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid simplified joint_mapping entry line: " + line.text);
          continue;
        }
        if (key_value->first == "name") {
          current_entry->sim_name = key_value->second;
          current_entry->hardware_name = key_value->second;
          current_entry->label = key_value->second;
        }
        continue;
      }

      if (line.indent == 2) {
        entry_subsection = EntrySubsection::kNone;
        current_entry = nullptr;
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid joint_mapping line: " + line.text);
          continue;
        }

        if (key_value->second.empty() && key_value->first == "validation") {
          joint_mapping_subsection = JointMappingSubsection::kValidation;
          continue;
        }
        if (key_value->second.empty() && key_value->first == "entries") {
          joint_mapping_subsection = JointMappingSubsection::kEntries;
          continue;
        }
      }

      if (line.indent == 4 &&
          joint_mapping_subsection == JointMappingSubsection::kValidation) {
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid joint_mapping.validation line: " + line.text);
          continue;
        }

        if (key_value->first == "expected_sim_joint_count") {
          assign_optional_size(&raw->joint_map_validation.expected_sim_joint_count,
                               "joint_mapping.validation.expected_sim_joint_count",
                               key_value->second, issues,
                               BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "expected_hardware_joint_count") {
          assign_optional_size(
              &raw->joint_map_validation.expected_hardware_joint_count,
              "joint_mapping.validation.expected_hardware_joint_count",
              key_value->second, issues,
              BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "require_dense_enabled_sim_indexes") {
          assign_bool_value(
              &raw->joint_map_validation.require_dense_enabled_sim_indexes,
              "joint_mapping.validation.require_dense_enabled_sim_indexes",
              key_value->second, issues,
              BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "require_dense_enabled_hardware_indexes") {
          assign_bool_value(
              &raw->joint_map_validation.require_dense_enabled_hardware_indexes,
              "joint_mapping.validation.require_dense_enabled_hardware_indexes",
              key_value->second, issues,
              BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "require_unique_enabled_sim_names") {
          assign_bool_value(
              &raw->joint_map_validation.require_unique_enabled_sim_names,
              "joint_mapping.validation.require_unique_enabled_sim_names",
              key_value->second, issues,
              BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "require_unique_enabled_hardware_names") {
          assign_bool_value(
              &raw->joint_map_validation.require_unique_enabled_hardware_names,
              "joint_mapping.validation.require_unique_enabled_hardware_names",
              key_value->second, issues,
              BridgeConfigIssueCode::kInvalidValue);
        }
        continue;
      }

      if (joint_mapping_subsection == JointMappingSubsection::kEntries &&
          line.indent == 4 && starts_with(line.text, "- ")) {
        raw->joint_map_entries.emplace_back();
        current_entry = &raw->joint_map_entries.back();
        entry_subsection = EntrySubsection::kNone;
        const std::string first_field = trim(line.text.substr(2));
        const auto key_value = split_key_value(first_field);
        if (!key_value.has_value() || current_entry == nullptr) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid joint_mapping entry line: " + line.text);
          continue;
        }
        if (key_value->first == "sim_index") {
          const std::optional<int> value = parse_int(key_value->second);
          if (!value.has_value() || *value < 0) {
            push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                       "joint_mapping.entries.sim_index must be a non-negative integer");
          } else {
            current_entry->sim_index = static_cast<std::size_t>(*value);
          }
        }
        continue;
      }

      if (joint_mapping_subsection == JointMappingSubsection::kEntries &&
          current_entry != nullptr && line.indent == 6) {
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid joint_mapping entry field: " + line.text);
          continue;
        }

        if (key_value->second.empty()) {
          if (key_value->first == "transforms") {
            entry_subsection = EntrySubsection::kTransforms;
            continue;
          }
          if (key_value->first == "sim_limits") {
            entry_subsection = EntrySubsection::kSimLimits;
            continue;
          }
          if (key_value->first == "hardware_limits") {
            entry_subsection = EntrySubsection::kHardwareLimits;
            continue;
          }
        }

        entry_subsection = EntrySubsection::kNone;
        if (key_value->first == "hardware_index") {
          const std::optional<int> value = parse_int(key_value->second);
          if (!value.has_value() || *value < 0) {
            push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                       "joint_mapping.entries.hardware_index must be a non-negative integer");
          } else {
            current_entry->hardware_index = static_cast<std::size_t>(*value);
          }
        } else if (key_value->first == "sim_name") {
          current_entry->sim_name = key_value->second;
        } else if (key_value->first == "hardware_name") {
          current_entry->hardware_name = key_value->second;
        } else if (key_value->first == "label") {
          current_entry->label = key_value->second;
        } else if (key_value->first == "enabled") {
          const std::optional<bool> value = parse_bool(key_value->second);
          if (!value.has_value()) {
            push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                       "joint_mapping.entries.enabled must be true or false");
          } else {
            current_entry->enabled = *value;
          }
        }
        continue;
      }

      if (joint_mapping_subsection == JointMappingSubsection::kEntries &&
          current_entry != nullptr && line.indent == 8 &&
          entry_subsection != EntrySubsection::kNone) {
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid joint_mapping nested field: " + line.text);
          continue;
        }

        if (entry_subsection == EntrySubsection::kTransforms) {
          const std::optional<JointFieldTransform> transform =
              parse_transform(key_value->second);
          if (!transform.has_value()) {
            push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                       "invalid transform for joint_mapping field " +
                           key_value->first);
          } else {
            assign_transform_field(&current_entry->transforms, key_value->first,
                                   *transform);
          }
        }

        if (entry_subsection == EntrySubsection::kSimLimits ||
            entry_subsection == EntrySubsection::kHardwareLimits) {
          const std::optional<JointValueRange> range = parse_range(key_value->second);
          if (!range.has_value()) {
            push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                       "invalid range for joint_mapping field " +
                           key_value->first);
          } else if (entry_subsection == EntrySubsection::kSimLimits) {
            assign_limit_field(&current_entry->sim_limits, key_value->first,
                               *range);
          } else {
            assign_limit_field(&current_entry->hardware_limits, key_value->first,
                               *range);
          }
        }
      }

      if (joint_mapping_subsection == JointMappingSubsection::kEntries &&
          current_entry != nullptr && line.indent == 4) {
        const auto key_value = split_key_value(line.text);
        if (!key_value.has_value()) {
          push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                     "invalid simplified joint_mapping field: " + line.text);
          continue;
        }

        if (key_value->first == "name") {
          current_entry->sim_name = key_value->second;
          current_entry->hardware_name = key_value->second;
          current_entry->label = key_value->second;
        } else if (key_value->first == "index") {
          const std::optional<int> value = parse_int(key_value->second);
          if (!value.has_value() || *value < 0) {
            push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                       "joint_mapping.index must be a non-negative integer");
          } else {
            current_entry->sim_index = static_cast<std::size_t>(*value);
            current_entry->hardware_index = static_cast<std::size_t>(*value);
          }
        } else if (key_value->first == "sign") {
          const std::optional<double> sign = parse_double(key_value->second);
          if (!sign.has_value()) {
            push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                       "joint_mapping.sign must be a numeric value");
          } else {
            apply_default_joint_transform(current_entry, *sign,
                                          current_entry->transforms.position.offset);
          }
        } else if (key_value->first == "offset") {
          const std::optional<double> offset = parse_double(key_value->second);
          if (!offset.has_value()) {
            push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                       "joint_mapping.offset must be a numeric value");
          } else {
            apply_default_joint_transform(current_entry,
                                          current_entry->transforms.position.sign,
                                          *offset);
          }
        } else if (key_value->first == "enable") {
          const std::optional<bool> value = parse_bool(key_value->second);
          if (!value.has_value()) {
            push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
                       "joint_mapping.enable must be true or false");
          } else {
            current_entry->enabled = *value;
          }
        }
        continue;
      }
    }
  }

  return issues->empty();
}

[[nodiscard]] std::optional<std::uint64_t> hz_to_period_ns(const int hz) {
  if (hz <= 0) {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(1'000'000'000ULL / hz);
}

[[nodiscard]] std::optional<std::uint64_t> ms_to_ns(const int milliseconds) {
  if (milliseconds < 0) {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(milliseconds) * 1'000'000ULL;
}

[[nodiscard]] std::size_t enabled_hardware_width(
    const std::vector<JointMapEntry>& entries) {
  std::size_t width = 0;
  for (const JointMapEntry& entry : entries) {
    if (!entry.enabled) {
      continue;
    }
    width = std::max(width, entry.hardware_index + 1);
  }
  return width;
}

[[nodiscard]] std::optional<BridgeStartupConfig> finalize_config(
    const RawBridgeConfig& raw, const std::string& file_path,
    std::vector<BridgeConfigIssue>* issues) {
  const bool simplified_config = !raw.schema_version.has_value();

  if (!raw.schema_version.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "schema_version is required");
  } else if (raw.schema_version.has_value() && *raw.schema_version != 1) {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "schema_version must be 1");
  }

  if (!raw.robot_name.has_value() || raw.robot_name->empty()) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               simplified_config ? "robot_kind is required"
                                 : "bridge.robot_name is required");
  }
  if (!raw.loop_rate_hz.has_value()) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               simplified_config ? "api_rate_hz or loop_rate_hz is required"
                                 : "bridge.loop_rate_hz is required");
  }
  if (!raw.stale_command_timeout_ms.has_value()) {
    if (!simplified_config) {
      push_issue(issues, BridgeConfigIssueCode::kMissingField,
                 "bridge.stale_command_timeout_ms is required");
    }
  }
  if (!raw.fault_mode.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "bridge.fault_policy.mode is required");
  }
  if (!raw.publish_degraded_state_while_faulted.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "bridge.fault_policy.publish_degraded_state_while_faulted is required");
  }
  if (!raw.halt_command_writes_while_faulted.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "bridge.fault_policy.halt_command_writes_while_faulted is required");
  }
  const bool has_profile_adapter_kind =
      raw.assembly_profile.has_value() && !raw.assembly_profile->empty();
  if (!raw.adapter_kind.has_value() || raw.adapter_kind->empty()) {
    if (!simplified_config) {
      if (!has_profile_adapter_kind) {
        push_issue(issues, BridgeConfigIssueCode::kMissingField,
                   "adapter.kind or assembly.profile is required");
      }
    }
  }
  if ((!raw.command_mode.has_value() || raw.command_mode->empty()) &&
      !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "adapter.command_mode is required");
  }
  if (!raw.dds_domain_id.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "dds.domain_id is required");
  }
  if (!raw.dds_command_topic.has_value() || raw.dds_command_topic->empty()) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               simplified_config ? "dds.command_topic or dds.command is required"
                                 : "dds.topics.command.name is required");
  }
  if (!raw.dds_state_topic.has_value() || raw.dds_state_topic->empty()) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               simplified_config ? "dds.state_topic or dds.state is required"
                                 : "dds.topics.state.name is required");
  }
  if ((!raw.connection_transport.has_value() || raw.connection_transport->empty()) &&
      !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "adapter.connection.transport is required");
  }
  const std::string configured_adapter_kind = raw.adapter_kind.has_value()
      ? *raw.adapter_kind
      : raw.assembly_profile.value_or(raw.robot_name.value_or(""));
  if ((!raw.connection_interface_hint.has_value() ||
       raw.connection_interface_hint->empty()) &&
      !simplified_config && configured_adapter_kind != "pm01") {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "adapter.connection.interface_hint is required");
  }
  if (!raw.read_initial_state_timeout_ms.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "adapter.startup.read_initial_state_timeout_ms is required");
  }
  if (!raw.require_healthy_hardware.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "adapter.startup.require_healthy_hardware is required");
  }
  if (raw.joint_map_entries.empty()) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "joint_mapping.entries must contain at least one entry");
  }

  if (!issues->empty()) {
    return std::nullopt;
  }

  const std::string adapter_kind = raw.adapter_kind.has_value()
      ? *raw.adapter_kind
      : raw.assembly_profile.value_or(raw.robot_name.value_or("kepler_k2"));
  const std::string robot_name = raw.robot_name.value_or(adapter_kind);
  const std::string command_mode = raw.command_mode.value_or(
      adapter_kind == "pm01" ? "joint_bridge" : "torque_pd_passthrough");
  const int dds_domain_id = raw.dds_domain_id.value_or(0);
  const std::string dds_status_topic = raw.dds_status_topic.value_or("");
  const bool debug_print_state = raw.debug_print_state.value_or(false);
  const bool debug_print_command = raw.debug_print_command.value_or(false);
  const bool debug_print_forwarding_log =
      raw.debug_print_forwarding_log.value_or(true);
  const int debug_print_rate_hz = raw.debug_print_rate_hz.value_or(0);
  bool gamepad_enabled = raw.gamepad_enabled.value_or(false);
  std::string gamepad_backend = raw.gamepad_backend.value_or("sdl");
  int gamepad_controller_index = raw.gamepad_controller_index.value_or(0);
  std::string gamepad_mapping_path = raw.gamepad_mapping_path.value_or("");
  int gamepad_poll_interval_ms = raw.gamepad_poll_interval_ms.value_or(5);
  int gamepad_stale_timeout_ms = raw.gamepad_stale_timeout_ms.value_or(100);
  const bool motion_state_required = raw.motion_state_required.value_or(true);
  const std::string motion_state_required_task =
      raw.motion_state_required_task.value_or("joint_bridge");
  const std::optional<std::string> motion_state_initial_task =
      raw.motion_state_initial_task.has_value() && raw.motion_state_initial_task->empty()
          ? std::nullopt
          : raw.motion_state_initial_task;
  const std::string connection_transport =
      raw.connection_transport.value_or(adapter_kind == "pm01" ? "dds" : "udp");
  const std::string connection_interface_hint = raw.connection_interface_hint.value_or(
      adapter_kind == "pm01" ? "" : "robot_lan0");
  const int read_initial_state_timeout_ms =
      raw.read_initial_state_timeout_ms.value_or(1000);
  const bool require_healthy_hardware =
      raw.require_healthy_hardware.value_or(true);
  const std::string fault_mode = raw.fault_mode.value_or("fail_stop");
  const bool publish_degraded_state_while_faulted =
      raw.publish_degraded_state_while_faulted.value_or(true);
  const bool halt_command_writes_while_faulted =
      raw.halt_command_writes_while_faulted.value_or(true);
  const std::string lowstate_trigger = raw.lowstate_trigger.value_or("fixed_rate");
  const std::string assembly_profile = raw.assembly_profile.value_or(adapter_kind);
  const bool assembly_adapter_enabled = raw.assembly_adapter_enabled.value_or(true);
  const std::string assembly_adapter_type = raw.assembly_adapter_type.value_or(adapter_kind);
  const bool assembly_command_ingress_enabled =
      raw.assembly_command_ingress_enabled.value_or(true);
  const std::string assembly_command_ingress_type =
      raw.assembly_command_ingress_type.value_or(
          assembly_command_ingress_enabled ? "dds" : "none");
  const bool assembly_state_egress_enabled =
      raw.assembly_state_egress_enabled.value_or(true);
  const std::string assembly_state_egress_type =
      raw.assembly_state_egress_type.value_or(
          assembly_state_egress_enabled ? "dds" : "none");

  for (const AssemblyModuleConfig& module : raw.assembly_modules) {
    if (module.name != "gamepad") {
      push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
                 "assembly.modules only supports gamepad module");
      continue;
    }

    const std::string module_type = module.type.empty() ? "sdl" : module.type;
    if (module_type != "sdl") {
      push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
                 "assembly.modules.gamepad.type must be sdl");
      continue;
    }

    gamepad_enabled = module.enabled;
    gamepad_backend = module_type;
    gamepad_controller_index = module.controller_index;
    gamepad_mapping_path = module.mapping_path;
    gamepad_poll_interval_ms = module.poll_interval_ms;
    gamepad_stale_timeout_ms = module.stale_timeout_ms;
  }

  if (fault_mode != "fail_stop") {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "bridge.fault_policy.mode must be fail_stop");
  }
  if (!halt_command_writes_while_faulted) {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "bridge.fault_policy.halt_command_writes_while_faulted must be true");
  }
  if (lowstate_trigger != "fixed_rate" && lowstate_trigger != "event") {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "bridge.lowstate_trigger must be fixed_rate or event");
  }
  if (!assembly_profile.empty() && assembly_profile != adapter_kind) {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "assembly.profile must match adapter.kind");
  }
  if (!assembly_adapter_enabled) {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "assembly.adapter.enabled must be true");
  }
  if (assembly_adapter_type != adapter_kind) {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "assembly.adapter.type must match adapter.kind");
  }
  if (assembly_command_ingress_type != "dds" && assembly_command_ingress_type != "none") {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "assembly.command_ingress.type must be dds or none");
  }
  if (assembly_state_egress_type != "dds" && assembly_state_egress_type != "none") {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "assembly.state_egress.type must be dds or none");
  }

  const std::optional<std::uint64_t> loop_period_ns = hz_to_period_ns(*raw.loop_rate_hz);
  if (!loop_period_ns.has_value()) {
    push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
               "api_rate_hz must be greater than zero");
  }
  const std::optional<std::uint64_t> stale_timeout_ns =
      ms_to_ns(raw.stale_command_timeout_ms.value_or(120));
  if (!stale_timeout_ns.has_value()) {
    push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
               "stale_command_timeout_ms must be non-negative");
  }
  const std::optional<std::uint64_t> initialize_timeout_ns =
      ms_to_ns(read_initial_state_timeout_ms);
  if (!initialize_timeout_ns.has_value()) {
    push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
               "adapter.startup.read_initial_state_timeout_ms must be non-negative");
  }
  if (dds_domain_id < 0) {
    push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
               "dds.domain_id must be non-negative");
  }
  if (debug_print_rate_hz < 0) {
    push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
               "debug_print.rate_hz must be non-negative");
  }
  if (gamepad_enabled && gamepad_backend != "sdl") {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "gamepad.backend must be sdl");
  }
  if (gamepad_controller_index < 0) {
    push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
               "gamepad.controller_index must be non-negative");
  }
  const std::optional<std::uint64_t> gamepad_poll_interval_ns =
      ms_to_ns(gamepad_poll_interval_ms);
  if (!gamepad_poll_interval_ns.has_value()) {
    push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
               "gamepad.poll_interval_ms must be non-negative");
  }
  const std::optional<std::uint64_t> gamepad_stale_timeout_ns =
      ms_to_ns(gamepad_stale_timeout_ms);
  if (!gamepad_stale_timeout_ns.has_value()) {
    push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
               "gamepad.stale_timeout_ms must be non-negative");
  }
  if (motion_state_required && motion_state_required_task.empty()) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "adapter.motion_state.required_task is required when motion gate is enabled");
  }

  if (!issues->empty()) {
    return std::nullopt;
  }

  JointMapValidationOptions joint_map_validation = raw.joint_map_validation;
  if (simplified_config) {
    joint_map_validation.require_dense_enabled_sim_indexes = false;
    joint_map_validation.require_dense_enabled_hardware_indexes = false;
    joint_map_validation.require_unique_enabled_sim_names = true;
    joint_map_validation.require_unique_enabled_hardware_names = true;
    if (!joint_map_validation.expected_sim_joint_count.has_value()) {
      joint_map_validation.expected_sim_joint_count =
          configured_sim_width(raw.joint_map_entries);
    }
    if (!joint_map_validation.expected_hardware_joint_count.has_value()) {
      joint_map_validation.expected_hardware_joint_count =
          configured_hardware_width(raw.joint_map_entries);
    }
  }

  std::vector<std::pair<std::size_t, std::string>> enabled_joint_names;
  enabled_joint_names.reserve(raw.joint_map_entries.size());
  for (const JointMapEntry& entry : raw.joint_map_entries) {
    if (!entry.enabled) {
      continue;
    }
    std::string joint_name;
    if (entry.hardware_name.has_value() && !entry.hardware_name->empty()) {
      joint_name = *entry.hardware_name;
    } else if (entry.sim_name.has_value() && !entry.sim_name->empty()) {
      joint_name = *entry.sim_name;
    } else if (entry.label.has_value() && !entry.label->empty()) {
      joint_name = *entry.label;
    }
    if (joint_name.empty()) {
      joint_name = "joint_" + std::to_string(entry.hardware_index);
    }
    if (!joint_name.empty()) {
      enabled_joint_names.emplace_back(entry.hardware_index, joint_name);
    }
  }

  std::sort(enabled_joint_names.begin(), enabled_joint_names.end(),
            [](const auto& left, const auto& right) {
              return left.first < right.first;
            });

  std::vector<std::string> joint_names;
  joint_names.reserve(enabled_joint_names.size());
  for (const auto& [_, joint_name] : enabled_joint_names) {
    joint_names.push_back(joint_name);
  }

  const JointMapBuildResult joint_map =
      JointMap::build(raw.joint_map_entries, joint_map_validation);
  if (!joint_map.succeeded()) {
    for (const JointMapIssue& issue : joint_map.validation.issues) {
      push_issue(issues, BridgeConfigIssueCode::kJointMapInvalid, issue.message);
    }
  }

  if (!issues->empty()) {
    return std::nullopt;
  }

  BridgeStartupConfig config;
  config.source_path = file_path;
  config.robot_type = adapter_kind;
  config.runner.loop_period_ns = *loop_period_ns;
  config.runner.lowstate_trigger = lowstate_trigger == "event"
                                        ? core::LowStateTriggerMode::kEvent
                                        : core::LowStateTriggerMode::kFixedRate;
  config.runner.forwarding_log_enabled = debug_print_forwarding_log;
  config.runner.command_stale_timeout_ns = *stale_timeout_ns;
  config.runner.gamepad_poll_interval_ns = *gamepad_poll_interval_ns;
  config.runner.gamepad_stale_timeout_ns = *gamepad_stale_timeout_ns;
  config.assembly.profile = assembly_profile;
  config.assembly.adapter.enabled = assembly_adapter_enabled;
  config.assembly.adapter.type = assembly_adapter_type;
  config.assembly.command_ingress.enabled = assembly_command_ingress_enabled;
  config.assembly.command_ingress.type = assembly_command_ingress_type;
  config.assembly.state_egress.enabled = assembly_state_egress_enabled;
  config.assembly.state_egress.type = assembly_state_egress_type;
  config.assembly.modules = raw.assembly_modules;
  config.dds.domain_id = static_cast<std::uint32_t>(dds_domain_id);
  config.dds.topics.command = *raw.dds_command_topic;
  config.dds.topics.state = *raw.dds_state_topic;
  config.dds.topics.imu_state = raw.dds_imu_state_topic.value_or("");
  config.dds.topics.bms_state = raw.dds_bms_state_topic.value_or("");
  config.dds.topics.mainboard_state = raw.dds_mainboard_state_topic.value_or("");
  config.dds.topics.fault_code = raw.dds_fault_code_topic.value_or("");
  config.dds.topics.status = dds_status_topic;
  config.debug_print.state = debug_print_state;
  config.debug_print.command = debug_print_command;
  config.debug_print.forwarding_log = debug_print_forwarding_log;
  config.debug_print.rate_hz = static_cast<std::uint32_t>(debug_print_rate_hz);
  config.gamepad.enabled = gamepad_enabled;
  config.gamepad.backend = gamepad_backend;
  config.gamepad.controller_index = gamepad_controller_index;
  config.gamepad.mapping_path = gamepad_mapping_path;
  config.runner.fault_policy.fail_stop_on_transport_fault = true;
  config.runner.fault_policy.fail_stop_on_command_stale = true;
  config.runner.fault_policy.fail_stop_on_adapter_degraded = false;
  config.runner.fault_policy.continue_publishing_in_fault =
      publish_degraded_state_while_faulted;

  config.adapter.kind = adapter_kind;
  config.adapter.robot_name = robot_name;
  // Simplified config: derive robot_model from robot_name if not provided
  std::string robot_model = raw.robot_model.value_or("");
  if (robot_model.empty() && !adapter_kind.empty()) {
    if (adapter_kind == "kepler_k2") {
      robot_model = "k2";
    } else if (adapter_kind == "pm01") {
      robot_model = "pm01";
    } else {
      robot_model = adapter_kind;
    }
  }
  config.adapter.robot_model = robot_model;
  config.adapter.command_mode = command_mode;
  config.adapter.connection.transport = connection_transport;
  config.adapter.connection.interface_hint = connection_interface_hint;
  config.adapter.startup.read_initial_state_timeout_ns = *initialize_timeout_ns;
  config.adapter.startup.require_healthy_hardware = require_healthy_hardware;
  config.adapter.motion_state.required = motion_state_required;
  config.adapter.motion_state.required_task = motion_state_required_task;
  config.adapter.motion_state.initial_task = motion_state_initial_task;
  config.adapter.topics.command = raw.adapter_command_topic.value_or("");
  config.adapter.topics.state = raw.adapter_state_topic.value_or("");
  config.adapter.topics.imu = raw.adapter_imu_topic.value_or("");
  config.adapter.topics.bms_state = raw.adapter_bms_state_topic.value_or("");
  config.adapter.topics.mainboard_state =
      raw.adapter_mainboard_state_topic.value_or("");
  config.adapter.joint_names = std::move(joint_names);

  config.joint_map_validation = joint_map_validation;
  config.joint_map_entries = raw.joint_map_entries;
  return config;
}

}

BridgeConfigLoadResult load_bridge_config(const std::string& file_path) {
  BridgeConfigLoadResult result;
  result.status = BridgeConfigLoadStatus::kRejected;

  const std::optional<std::vector<ParsedLine>> lines =
      load_lines(file_path, &result.issues);
  if (!lines.has_value()) {
    result.status = BridgeConfigLoadStatus::kIoError;
    result.detail = first_issue_detail(result.issues, "config file load failed");
    return result;
  }

  RawBridgeConfig raw;
  const bool parsed_ok = parse_lines(*lines, &raw, &result.issues);
  std::optional<BridgeStartupConfig> config =
      finalize_config(raw, file_path, &result.issues);
  if (!parsed_ok || !config.has_value()) {
    result.status = BridgeConfigLoadStatus::kRejected;
    result.detail = first_issue_detail(result.issues, "config validation failed");
    return result;
  }

  result.status = BridgeConfigLoadStatus::kOk;
  result.detail = "bridge startup config loaded";
  result.config = std::move(config);
  return result;
}

}
