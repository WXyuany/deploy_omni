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
  kAdapter,
  kJointMapping,
};

enum class BridgeSubsection {
  kNone,
  kStartupIdlePolicy,
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
  kStatus,
};

enum class AdapterSubsection {
  kNone,
  kConnection,
  kStartup,
  kSafety,
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
  std::optional<int> stale_command_timeout_ms;
  std::optional<std::string> startup_idle_mode;
  std::optional<bool> require_first_valid_command;
  std::optional<bool> use_adapter_safe_command;
  std::optional<std::string> fault_mode;
  std::optional<bool> send_safe_command_on_fault;
  std::optional<bool> publish_degraded_state_while_faulted;
  std::optional<bool> halt_command_writes_while_faulted;
  std::optional<std::string> adapter_kind;
  std::optional<std::string> robot_model;
  std::optional<std::string> adapter_backend;
  std::optional<std::string> command_mode;
  std::optional<int> dds_domain_id;
  std::optional<std::string> dds_command_topic;
  std::optional<std::string> dds_state_topic;
  std::optional<std::string> dds_status_topic;
  std::optional<std::string> connection_transport;
  std::optional<std::string> connection_interface_hint;
  std::optional<int> read_initial_state_timeout_ms;
  std::optional<bool> require_healthy_hardware;
  std::optional<std::string> safe_command_profile;
  std::optional<bool> allow_safe_command_during_shutdown;
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
  AdapterSubsection adapter_subsection = AdapterSubsection::kNone;
  JointMappingSubsection joint_mapping_subsection = JointMappingSubsection::kNone;
  EntrySubsection entry_subsection = EntrySubsection::kNone;
  JointMapEntry* current_entry = nullptr;

  for (const ParsedLine& line : lines) {
    if (line.indent == 0) {
      bridge_subsection = BridgeSubsection::kNone;
      dds_subsection = DdsSubsection::kNone;
      dds_topic_selection = DdsTopicSelection::kNone;
      adapter_subsection = AdapterSubsection::kNone;
      joint_mapping_subsection = JointMappingSubsection::kNone;
      entry_subsection = EntrySubsection::kNone;
      current_entry = nullptr;

      if (line.text == "bridge:") {
        top = TopSection::kBridge;
        continue;
      }
      if (line.text == "dds:") {
        top = TopSection::kDds;
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

      const auto key_value = split_key_value(line.text);
      if (key_value.has_value() && key_value->first == "schema_version") {
        assign_required<int>(&raw->schema_version, "schema_version",
                             key_value->second, issues,
                             BridgeConfigIssueCode::kInvalidValue);
        continue;
      }
      if (key_value.has_value() && key_value->first == "robot_kind") {
        raw->adapter_kind = key_value->second;
        if (!raw->robot_name.has_value()) {
          raw->robot_name = key_value->second;
        }
        continue;
      }
      if (key_value.has_value() && key_value->first == "loop_rate_hz") {
        assign_required<int>(&raw->loop_rate_hz, "loop_rate_hz", key_value->second,
                             issues, BridgeConfigIssueCode::kInvalidValue);
        continue;
      }
      if (key_value.has_value() && key_value->first == "stale_command_timeout_ms") {
        assign_required<int>(&raw->stale_command_timeout_ms,
                             "stale_command_timeout_ms", key_value->second, issues,
                             BridgeConfigIssueCode::kInvalidValue);
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
          if (key_value->first == "startup_idle_policy") {
            bridge_subsection = BridgeSubsection::kStartupIdlePolicy;
            continue;
          }
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

        if (bridge_subsection == BridgeSubsection::kStartupIdlePolicy) {
          if (key_value->first == "mode") {
            raw->startup_idle_mode = key_value->second;
          } else if (key_value->first == "require_first_valid_command") {
            assign_required<bool>(&raw->require_first_valid_command,
                                  "bridge.startup_idle_policy.require_first_valid_command",
                                  key_value->second, issues,
                                  BridgeConfigIssueCode::kInvalidValue);
          } else if (key_value->first == "use_adapter_safe_command") {
            assign_required<bool>(&raw->use_adapter_safe_command,
                                  "bridge.startup_idle_policy.use_adapter_safe_command",
                                  key_value->second, issues,
                                  BridgeConfigIssueCode::kInvalidValue);
          }
        }

        if (bridge_subsection == BridgeSubsection::kFaultPolicy) {
          if (key_value->first == "mode") {
            raw->fault_mode = key_value->second;
          } else if (key_value->first == "send_safe_command_on_fault") {
            assign_required<bool>(&raw->send_safe_command_on_fault,
                                  "bridge.fault_policy.send_safe_command_on_fault",
                                  key_value->second, issues,
                                  BridgeConfigIssueCode::kInvalidValue);
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

        if (key_value->first == "domain_id") {
          assign_required<int>(&raw->dds_domain_id, "dds.domain_id",
                               key_value->second, issues,
                               BridgeConfigIssueCode::kInvalidValue);
        } else if (key_value->first == "command") {
          raw->dds_command_topic = key_value->second;
        } else if (key_value->first == "state") {
          raw->dds_state_topic = key_value->second;
        } else if (key_value->first == "status") {
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
          } else if (dds_topic_selection == DdsTopicSelection::kStatus) {
            raw->dds_status_topic = key_value->second;
          }
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
          if (key_value->first == "safety") {
            adapter_subsection = AdapterSubsection::kSafety;
            continue;
          }
        }

        if (key_value->first == "kind") {
          raw->adapter_kind = key_value->second;
        } else if (key_value->first == "robot_model") {
          raw->robot_model = key_value->second;
        } else if (key_value->first == "backend") {
          raw->adapter_backend = key_value->second;
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

        if (adapter_subsection == AdapterSubsection::kSafety) {
          if (key_value->first == "safe_command_profile") {
            raw->safe_command_profile = key_value->second;
          } else if (key_value->first == "allow_safe_command_during_shutdown") {
            assign_required<bool>(&raw->allow_safe_command_during_shutdown,
                                  "adapter.safety.allow_safe_command_during_shutdown",
                                  key_value->second, issues,
                                  BridgeConfigIssueCode::kInvalidValue);
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
        const auto key_value = split_key_value(trim(line.text.substr(2)));
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
               "bridge.loop_rate_hz is required");
  }
  if (!raw.stale_command_timeout_ms.has_value()) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "bridge.stale_command_timeout_ms is required");
  }
  if (!raw.startup_idle_mode.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "bridge.startup_idle_policy.mode is required");
  }
  if (!raw.require_first_valid_command.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "bridge.startup_idle_policy.require_first_valid_command is required");
  }
  if (!raw.use_adapter_safe_command.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "bridge.startup_idle_policy.use_adapter_safe_command is required");
  }
  if (!raw.fault_mode.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "bridge.fault_policy.mode is required");
  }
  if (!raw.send_safe_command_on_fault.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "bridge.fault_policy.send_safe_command_on_fault is required");
  }
  if (!raw.publish_degraded_state_while_faulted.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "bridge.fault_policy.publish_degraded_state_while_faulted is required");
  }
  if (!raw.halt_command_writes_while_faulted.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "bridge.fault_policy.halt_command_writes_while_faulted is required");
  }
  if (!raw.adapter_kind.has_value() || raw.adapter_kind->empty()) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "adapter.kind is required");
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
               "dds.topics.command.name is required");
  }
  if (!raw.dds_state_topic.has_value() || raw.dds_state_topic->empty()) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "dds.topics.state.name is required");
  }
  if ((!raw.connection_transport.has_value() || raw.connection_transport->empty()) &&
      !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "adapter.connection.transport is required");
  }
  if ((!raw.connection_interface_hint.has_value() ||
       raw.connection_interface_hint->empty()) &&
      !simplified_config) {
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
  if ((!raw.safe_command_profile.has_value() || raw.safe_command_profile->empty()) &&
      !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "adapter.safety.safe_command_profile is required");
  }
  if (!raw.allow_safe_command_during_shutdown.has_value() && !simplified_config) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "adapter.safety.allow_safe_command_during_shutdown is required");
  }
  if (raw.joint_map_entries.empty()) {
    push_issue(issues, BridgeConfigIssueCode::kMissingField,
               "joint_mapping.entries must contain at least one entry");
  }

  if (!issues->empty()) {
    return std::nullopt;
  }

  const std::string adapter_kind = *raw.adapter_kind;
  const std::string robot_name = raw.robot_name.value_or(adapter_kind);
  const std::string adapter_backend = raw.adapter_backend.value_or("");
  const std::string command_mode = raw.command_mode.value_or(
      adapter_kind == "pm01" ? "joint_bridge" : "torque_pd_passthrough");
  const int dds_domain_id = raw.dds_domain_id.value_or(0);
  const std::string dds_status_topic = raw.dds_status_topic.value_or("");
  const std::string connection_transport =
      raw.connection_transport.value_or(adapter_kind == "pm01" ? "dds" : "udp");
  const std::string connection_interface_hint = raw.connection_interface_hint.value_or(
      adapter_kind == "pm01" ? "pm01" : "robot_lan0");
  const int read_initial_state_timeout_ms =
      raw.read_initial_state_timeout_ms.value_or(1000);
  const bool require_healthy_hardware =
      raw.require_healthy_hardware.value_or(true);
  const std::string safe_command_profile =
      raw.safe_command_profile.value_or("zero_torque");
  const bool allow_safe_command_during_shutdown =
      raw.allow_safe_command_during_shutdown.value_or(true);
  const std::string startup_idle_mode =
      raw.startup_idle_mode.value_or("zero_torque");
  const bool require_first_valid_command =
      raw.require_first_valid_command.value_or(true);
  const bool use_adapter_safe_command =
      raw.use_adapter_safe_command.value_or(true);
  const std::string fault_mode = raw.fault_mode.value_or("fail_stop");
  const bool send_safe_command_on_fault =
      raw.send_safe_command_on_fault.value_or(true);
  const bool publish_degraded_state_while_faulted =
      raw.publish_degraded_state_while_faulted.value_or(true);
  const bool halt_command_writes_while_faulted =
      raw.halt_command_writes_while_faulted.value_or(true);

  if (startup_idle_mode != "zero_torque") {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "bridge.startup_idle_policy.mode must be zero_torque");
  }
  if (fault_mode != "fail_stop") {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "bridge.fault_policy.mode must be fail_stop");
  }
  if (!require_first_valid_command) {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "bridge.startup_idle_policy.require_first_valid_command must be true");
  }
  if (!halt_command_writes_while_faulted) {
    push_issue(issues, BridgeConfigIssueCode::kUnsupportedValue,
               "bridge.fault_policy.halt_command_writes_while_faulted must be true");
  }

  const std::optional<std::uint64_t> loop_period_ns = hz_to_period_ns(*raw.loop_rate_hz);
  if (!loop_period_ns.has_value()) {
    push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
               "bridge.loop_rate_hz must be greater than zero");
  }
  const std::optional<std::uint64_t> stale_timeout_ns =
      ms_to_ns(*raw.stale_command_timeout_ms);
  if (!stale_timeout_ns.has_value()) {
    push_issue(issues, BridgeConfigIssueCode::kInvalidValue,
               "bridge.stale_command_timeout_ms must be non-negative");
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
    if (!entry.hardware_name.has_value() || entry.hardware_name->empty()) {
      push_issue(issues, BridgeConfigIssueCode::kMissingField,
                 "each enabled joint_mapping entry must define hardware_name");
      continue;
    }
    enabled_joint_names.emplace_back(entry.hardware_index, *entry.hardware_name);
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
  config.runner.command_stale_timeout_ns = *stale_timeout_ns;
  config.dds.domain_id = static_cast<std::uint32_t>(dds_domain_id);
  config.dds.topics.command = *raw.dds_command_topic;
  config.dds.topics.state = *raw.dds_state_topic;
  config.dds.topics.status = dds_status_topic;
  config.runner.startup_idle_policy =
      use_adapter_safe_command ? core::StartupIdlePolicy::kPreferSafeCommand
                               : core::StartupIdlePolicy::kNoCommandWrite;
  config.runner.fault_policy.fail_stop_on_transport_fault = true;
  config.runner.fault_policy.fail_stop_on_command_stale = true;
  config.runner.fault_policy.fail_stop_on_adapter_degraded = false;
  config.runner.fault_policy.write_safe_command_on_fault =
      send_safe_command_on_fault;
  config.runner.fault_policy.continue_publishing_in_fault =
      publish_degraded_state_while_faulted;

  config.adapter.kind = adapter_kind;
  config.adapter.robot_name = robot_name;
  config.adapter.robot_model = raw.robot_model.value_or("");
  config.adapter.backend = adapter_backend;
  config.adapter.command_mode = command_mode;
  config.adapter.connection.transport = connection_transport;
  config.adapter.connection.interface_hint = connection_interface_hint;
  config.adapter.startup.read_initial_state_timeout_ns = *initialize_timeout_ns;
  config.adapter.startup.require_healthy_hardware = require_healthy_hardware;
  config.adapter.safety.safe_command_profile = safe_command_profile;
  config.adapter.safety.allow_safe_command_during_shutdown =
      allow_safe_command_during_shutdown;
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
