#include "../../../../include/bridge/adapters/unitree_g1/unitree_g1_sensor_session.hpp"

#include <utility>

#include <unitree/robot/channel/channel_factory.hpp>

namespace bridge::adapters::unitree_g1 {

UnitreeG1SensorSession::UnitreeG1SensorSession(UnitreeG1SensorSessionConfig config)
    : config_(std::move(config)) {}

UnitreeG1SensorSession::~UnitreeG1SensorSession() { shutdown(); }

bool UnitreeG1SensorSession::initialize() {
  if (initialized_.exchange(true)) {
    return true;
  }
  if (config_.transport_interface.empty() || config_.odometry_topic.empty() ||
      config_.pointcloud_topic.empty() || config_.queue_length < 1) {
    initialized_.store(false);
    return false;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, config_.transport_interface);
  odometry_subscriber_ = std::make_shared<unitree::robot::ChannelSubscriber<Odometry>>(
      config_.odometry_topic);
  pointcloud_subscriber_ = std::make_shared<unitree::robot::ChannelSubscriber<PointCloud>>(
      config_.pointcloud_topic);
  odometry_subscriber_->InitChannel(
      [this](const void* message) { handle_odometry(message); }, config_.queue_length);
  pointcloud_subscriber_->InitChannel(
      [this](const void* message) { handle_pointcloud(message); }, config_.queue_length);
  return true;
}

bool UnitreeG1SensorSession::initialized() const noexcept { return initialized_.load(); }

UnitreeG1SensorSnapshot UnitreeG1SensorSession::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

void UnitreeG1SensorSession::shutdown() noexcept {
  if (!initialized_.exchange(false)) {
    return;
  }
  if (odometry_subscriber_) {
    odometry_subscriber_->CloseChannel();
  }
  if (pointcloud_subscriber_) {
    pointcloud_subscriber_->CloseChannel();
  }
  odometry_subscriber_.reset();
  pointcloud_subscriber_.reset();
}

void UnitreeG1SensorSession::handle_odometry(const void* message) {
  if (message == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.odometry = *static_cast<const Odometry*>(message);
  ++snapshot_.odometry_samples;
}

void UnitreeG1SensorSession::handle_pointcloud(const void* message) {
  if (message == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.pointcloud = *static_cast<const PointCloud*>(message);
  ++snapshot_.pointcloud_samples;
}

}  // namespace bridge::adapters::unitree_g1
