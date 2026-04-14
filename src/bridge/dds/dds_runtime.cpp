#include "../../../include/bridge/dds/dds_runtime.hpp"

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include "../../../idl/bridge/BridgeMessagesPubSubTypes.hpp"

namespace bridge::dds {
namespace {

template <typename T, typename Deleter>
[[nodiscard]] std::shared_ptr<T> share_participant_child(
    T* child,
    eprosima::fastdds::dds::DomainParticipant* participant,
    Deleter deleter) {
  if (child == nullptr) {
    return nullptr;
  }
  return std::shared_ptr<T>(child, [participant, deleter](T* ptr) {
    if (participant != nullptr && ptr != nullptr) {
      deleter(participant, ptr);
    }
  });
}

template <typename T, typename Deleter>
[[nodiscard]] std::shared_ptr<T> share_publisher_child(
    T* child,
    eprosima::fastdds::dds::Publisher* publisher,
    Deleter deleter) {
  if (child == nullptr) {
    return nullptr;
  }
  return std::shared_ptr<T>(child, [publisher, deleter](T* ptr) {
    if (publisher != nullptr && ptr != nullptr) {
      deleter(publisher, ptr);
    }
  });
}

template <typename T, typename Deleter>
[[nodiscard]] std::shared_ptr<T> share_subscriber_child(
    T* child,
    eprosima::fastdds::dds::Subscriber* subscriber,
    Deleter deleter) {
  if (child == nullptr) {
    return nullptr;
  }
  return std::shared_ptr<T>(child, [subscriber, deleter](T* ptr) {
    if (subscriber != nullptr && ptr != nullptr) {
      deleter(subscriber, ptr);
    }
  });
}

}

std::shared_ptr<DdsRuntime> make_dds_runtime(DdsRuntimeConfig config) {
  using namespace eprosima::fastdds::dds;

  auto runtime = std::make_shared<DdsRuntime>();
  runtime->config = std::move(config);
  DomainParticipantQos participant_qos;
  participant_qos.name("bridge_dds_runtime_participant");

  runtime->participant = std::shared_ptr<DomainParticipant>(
      DomainParticipantFactory::get_instance()->create_participant(
          runtime->config.domain_id, participant_qos),
      [](DomainParticipant* participant) {
        if (participant != nullptr) {
          DomainParticipantFactory::get_instance()->delete_participant(participant);
        }
      });

  if (runtime->participant == nullptr) {
    return runtime;
  }

  runtime->command_type = TypeSupport(new bridge_wire::LowCmdPubSubType());
  runtime->state_type = TypeSupport(new bridge_wire::LowStatePubSubType());
  runtime->status_type = TypeSupport(new bridge_wire::BridgeStatusPubSubType());

  runtime->command_type.register_type(runtime->participant.get());
  runtime->state_type.register_type(runtime->participant.get());
  runtime->status_type.register_type(runtime->participant.get());

  runtime->publisher = share_participant_child(
      runtime->participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr),
      runtime->participant.get(),
      [](DomainParticipant* participant, Publisher* publisher) {
        participant->delete_publisher(publisher);
      });
  runtime->subscriber = share_participant_child(
      runtime->participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr),
      runtime->participant.get(),
      [](DomainParticipant* participant, Subscriber* subscriber) {
        participant->delete_subscriber(subscriber);
      });

  runtime->command_topic = share_participant_child(
      runtime->participant->create_topic(
          runtime->config.command_topic, runtime->command_type.get_type_name(),
          TOPIC_QOS_DEFAULT),
      runtime->participant.get(),
      [](DomainParticipant* participant, Topic* topic) {
        participant->delete_topic(topic);
      });
  runtime->state_topic = share_participant_child(
      runtime->participant->create_topic(
          runtime->config.state_topic, runtime->state_type.get_type_name(),
          TOPIC_QOS_DEFAULT),
      runtime->participant.get(),
      [](DomainParticipant* participant, Topic* topic) {
        participant->delete_topic(topic);
      });
  runtime->status_topic = share_participant_child(
      runtime->participant->create_topic(
          runtime->config.status_topic, runtime->status_type.get_type_name(),
          TOPIC_QOS_DEFAULT),
      runtime->participant.get(),
      [](DomainParticipant* participant, Topic* topic) {
        participant->delete_topic(topic);
      });

  if (runtime->subscriber && runtime->command_topic) {
    runtime->command_reader = share_subscriber_child(
        runtime->subscriber->create_datareader(
            runtime->command_topic.get(), DATAREADER_QOS_DEFAULT, nullptr),
        runtime->subscriber.get(),
        [](Subscriber* subscriber, DataReader* reader) {
          subscriber->delete_datareader(reader);
        });
  }
  if (runtime->publisher && runtime->state_topic) {
    runtime->state_writer = share_publisher_child(
        runtime->publisher->create_datawriter(
            runtime->state_topic.get(), DATAWRITER_QOS_DEFAULT, nullptr),
        runtime->publisher.get(),
        [](Publisher* publisher, DataWriter* writer) {
          publisher->delete_datawriter(writer);
        });
  }
  if (runtime->publisher && runtime->status_topic) {
    runtime->status_writer = share_publisher_child(
        runtime->publisher->create_datawriter(
            runtime->status_topic.get(), DATAWRITER_QOS_DEFAULT, nullptr),
        runtime->publisher.get(),
        [](Publisher* publisher, DataWriter* writer) {
          publisher->delete_datawriter(writer);
        });
  }

  return runtime;
}

}
