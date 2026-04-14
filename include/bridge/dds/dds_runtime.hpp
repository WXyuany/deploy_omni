#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

namespace bridge::dds {

struct DdsRuntimeConfig {
  std::uint32_t domain_id{0};
  std::string command_topic{"bridge/command"};
  std::string state_topic{"bridge/state"};
  std::string status_topic{"bridge/status"};
};

struct DdsRuntime {
  std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant;
  std::shared_ptr<eprosima::fastdds::dds::Publisher> publisher;
  std::shared_ptr<eprosima::fastdds::dds::Subscriber> subscriber;
  std::shared_ptr<eprosima::fastdds::dds::Topic> command_topic;
  std::shared_ptr<eprosima::fastdds::dds::Topic> state_topic;
  std::shared_ptr<eprosima::fastdds::dds::Topic> status_topic;
  std::shared_ptr<eprosima::fastdds::dds::DataReader> command_reader;
  std::shared_ptr<eprosima::fastdds::dds::DataWriter> state_writer;
  std::shared_ptr<eprosima::fastdds::dds::DataWriter> status_writer;
  eprosima::fastdds::dds::TypeSupport command_type;
  eprosima::fastdds::dds::TypeSupport state_type;
  eprosima::fastdds::dds::TypeSupport status_type;
  DdsRuntimeConfig config;
};

[[nodiscard]] std::shared_ptr<DdsRuntime> make_dds_runtime(
    DdsRuntimeConfig config = {});

}
