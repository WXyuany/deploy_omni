#include <cassert>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <unitree/robot/channel/channel_publisher.hpp>

#include "../../include/bridge/adapters/unitree_g1/unitree_g1_sensor_session.hpp"

int main() {
  bridge::adapters::unitree_g1::UnitreeG1SensorSession session;
  assert(!session.initialized());
  const auto snapshot = session.snapshot();
  assert(snapshot.odometry_samples == 0U);
  assert(snapshot.pointcloud_samples == 0U);

  bridge::adapters::unitree_g1::UnitreeG1SensorSessionConfig invalid_config;
  invalid_config.transport_interface.clear();
  invalid_config.odometry_topic = "rt/odom";
  invalid_config.pointcloud_topic = "rt/rslidar_points";
  bridge::adapters::unitree_g1::UnitreeG1SensorSession invalid(invalid_config);
  assert(!invalid.initialize());
  // Synthetic messages on loopback only; no robot or sensor driver is involved.
  using namespace bridge::adapters::unitree_g1;
  UnitreeG1SensorSessionConfig config;
  config.transport_interface = "lo";
  const auto prefix = "deploy_omni/test/sensor/" + std::to_string(getpid());
  config.pointcloud_topic = prefix + "/cloud";
  config.odometry_topic = prefix + "/odom";
  UnitreeG1SensorSession reader(config);
  assert(reader.initialize());
  using Cloud = sensor_msgs::msg::dds_::PointCloud2_;
  using Odom = nav_msgs::msg::dds_::Odometry_;
  unitree::robot::ChannelPublisher<Cloud> cloud_writer(config.pointcloud_topic);
  unitree::robot::ChannelPublisher<Odom> odom_writer(config.odometry_topic);
  cloud_writer.InitChannel();
  odom_writer.InitChannel();
  Cloud cloud;
  cloud.header().frame_id("test_lidar");
  cloud.width(2); cloud.height(1); cloud.point_step(12); cloud.row_step(24);
  cloud.data(std::vector<std::uint8_t>(24, 0));
  Odom odom;
  odom.header().frame_id("test_map");
  odom.child_frame_id("test_base");
  odom.pose().pose().position().x(1.25);
  odom.pose().pose().orientation().w(1.0);
  UnitreeG1SensorSnapshot received;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  do {
    cloud_writer.Write(cloud);
    odom_writer.Write(odom);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    received = reader.snapshot();
  } while ((!received.pointcloud || !received.odometry) &&
           std::chrono::steady_clock::now() < deadline);
  assert(received.pointcloud_samples > 0 && received.odometry_samples > 0);
  assert(received.pointcloud->header().frame_id() == "test_lidar");
  assert(received.pointcloud->width() == 2 && received.pointcloud->data().size() == 24);
  assert(received.odometry->pose().pose().position().x() == 1.25);
  assert(received.odometry->child_frame_id() == "test_base");
  reader.shutdown();
  assert(!reader.initialized());
  return 0;
}
