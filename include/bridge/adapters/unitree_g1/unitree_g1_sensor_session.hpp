#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <unitree/idl/ros2/Odometry_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace bridge::adapters::unitree_g1 {

struct UnitreeG1SensorSessionConfig {
  std::string transport_interface{"eth0"};
  std::string odometry_topic{"rt/lio_sam_ros2/mapping/re_location_odometry"};
  std::string pointcloud_topic{"rt/rslidar_points"};
  std::int64_t queue_length{8};
};

struct UnitreeG1SensorSnapshot {
  std::optional<nav_msgs::msg::dds_::Odometry_> odometry;
  std::optional<sensor_msgs::msg::dds_::PointCloud2_> pointcloud;
  std::uint64_t odometry_samples{0};
  std::uint64_t pointcloud_samples{0};
};

class UnitreeG1SensorSession final {
 public:
  explicit UnitreeG1SensorSession(UnitreeG1SensorSessionConfig config = {});
  ~UnitreeG1SensorSession();

  UnitreeG1SensorSession(const UnitreeG1SensorSession&) = delete;
  UnitreeG1SensorSession& operator=(const UnitreeG1SensorSession&) = delete;

  [[nodiscard]] bool initialize();
  [[nodiscard]] bool initialized() const noexcept;
  [[nodiscard]] UnitreeG1SensorSnapshot snapshot() const;
  void shutdown() noexcept;

 private:
  using Odometry = nav_msgs::msg::dds_::Odometry_;
  using PointCloud = sensor_msgs::msg::dds_::PointCloud2_;

  void handle_odometry(const void* message);
  void handle_pointcloud(const void* message);

  UnitreeG1SensorSessionConfig config_;
  std::atomic<bool> initialized_{false};
  mutable std::mutex mutex_;
  UnitreeG1SensorSnapshot snapshot_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<Odometry>> odometry_subscriber_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<PointCloud>> pointcloud_subscriber_;
};

}  // namespace bridge::adapters::unitree_g1
