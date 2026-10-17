#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/ros2/Odometry_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/idl/ros2/PoseStamped_.hpp>
#include <unitree/idl/go2/LidarState_.hpp>
#include <unitree/robot/b2/robot_state/robot_state_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/video/video_client.hpp>

namespace {

using LowState = unitree_hg::msg::dds_::LowState_;
using Odometry = nav_msgs::msg::dds_::Odometry_;
using PointCloud = sensor_msgs::msg::dds_::PointCloud2_;
using PoseStamped = geometry_msgs::msg::dds_::PoseStamped_;
using LidarState = unitree_go::msg::dds_::LidarState_;

template <typename Message>
class CandidateTopicProbe {
 public:
  CandidateTopicProbe(std::string topic, std::atomic<std::uint64_t>* count)
      : topic_(std::move(topic)), count_(count), subscriber_(topic_) {
    subscriber_.InitChannel(
        [this](const void*) { count_->fetch_add(1, std::memory_order_relaxed); }, 8);
  }

  [[nodiscard]] const std::string& topic() const noexcept { return topic_; }

 private:
  std::string topic_;
  std::atomic<std::uint64_t>* count_;
  unitree::robot::ChannelSubscriber<Message> subscriber_;
};

void print_result(const char* name, bool passed, const std::string& detail) {
  std::cout << name << ": " << (passed ? "PASS" : "FAIL") << " " << detail << '\n';
}

}  // namespace

int main() {
  unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");

  std::atomic<std::uint64_t> lowstate_count{0};
  LowState latest_state;
  std::mutex state_mutex;
  unitree::robot::ChannelSubscriber<LowState> lowstate_subscriber("rt/lowstate");
  lowstate_subscriber.InitChannel(
      [&](const void* message) {
        if (message == nullptr) {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(state_mutex);
          latest_state = *static_cast<const LowState*>(message);
        }
        lowstate_count.fetch_add(1, std::memory_order_relaxed);
      },
      32);
  unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher(
      "rt/lowcmd");
  lowcmd_publisher.InitChannel();

  std::atomic<std::uint64_t> odometry_count{0};
  std::atomic<std::uint64_t> pointcloud_count{0};
  std::atomic<std::uint64_t> lidar_state_count{0};
  std::atomic<std::uint64_t> slam_pose_count{0};

  // These are the topic names used by Unitree's published SLAM examples.
  CandidateTopicProbe<Odometry> odom_slam(
      "rt/lio_sam_ros2/mapping/re_location_odometry", &odometry_count);
  CandidateTopicProbe<Odometry> odom_rt("rt/odom", &odometry_count);
  CandidateTopicProbe<Odometry> odom_plain("odom", &odometry_count);
  CandidateTopicProbe<Odometry> odom_utlidar("rt/utlidar/odometry", &odometry_count);
  CandidateTopicProbe<PointCloud> cloud_rslidar("rt/rslidar_points", &pointcloud_count);
  CandidateTopicProbe<PointCloud> cloud_rt("rt/pointcloud", &pointcloud_count);
  CandidateTopicProbe<PointCloud> cloud_utlidar("rt/utlidar/cloud", &pointcloud_count);
  CandidateTopicProbe<LidarState> lidar_rt("rt/lidar", &lidar_state_count);
  CandidateTopicProbe<LidarState> lidar_utlidar("rt/utlidar/lidar_state", &lidar_state_count);
  CandidateTopicProbe<PoseStamped> pose_rt("rt/slam/pose", &slam_pose_count);
  CandidateTopicProbe<PoseStamped> pose_utlidar("rt/utlidar/robot_pose", &slam_pose_count);

  unitree::robot::b2::RobotStateClient robot_state;
  robot_state.SetTimeout(5.0F);
  robot_state.Init();
  std::vector<unitree::robot::b2::ServiceState> services;
  const auto service_code = robot_state.ServiceList(services);
  int32_t low_power_status = -1;
  const auto low_power_code = robot_state.LowPowerStatus(low_power_status);
  std::cout << "device_status_service: code=" << service_code
            << " services=" << services.size() << " low_power_code=" << low_power_code
            << " low_power_status=" << low_power_status << '\n';
  for (const auto& service : services) {
    std::cout << "  service name=" << service.name << " status=" << service.status
              << " protect=" << service.protect << '\n';
  }

  int32_t lidar_switch_status = -1;
  const auto lidar_switch_code =
      robot_state.ServiceSwitch("lidar_driver", 1, lidar_switch_status);
  std::cout << "lidar_driver_start: code=" << lidar_switch_code
            << " status=" << lidar_switch_status << '\n';

  std::vector<std::uint8_t> camera_sample;
  unitree::robot::go2::VideoClient video_client;
  video_client.SetTimeout(2.0F);
  video_client.Init();
  const auto camera_code = video_client.GetImageSample(camera_sample);
  bool camera_jpeg = camera_sample.size() >= 2 && camera_sample[0] == 0xFFU &&
                     camera_sample[1] == 0xD8U;
  std::cout << "realsense_front_camera: code=" << camera_code
            << " jpeg_bytes=" << camera_sample.size()
            << " jpeg_signature=" << (camera_jpeg ? "yes" : "no") << '\n';

  lowstate_count.store(0, std::memory_order_relaxed);
  const auto sample_window_start = std::chrono::steady_clock::now();
  std::this_thread::sleep_for(std::chrono::seconds(3));

  int32_t lidar_stop_status = -1;
  const auto lidar_stop_code =
      robot_state.ServiceSwitch("lidar_driver", 0, lidar_stop_status);
  std::cout << "lidar_driver_restore: code=" << lidar_stop_code
            << " status=" << lidar_stop_status << '\n';
  const auto sample_window_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - sample_window_start).count();

  LowState state_snapshot;
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    state_snapshot = latest_state;
  }

  const auto lowstate_samples = lowstate_count.load(std::memory_order_relaxed);
  const auto lowstate_hz = static_cast<double>(lowstate_samples) / sample_window_seconds;
  const auto& imu = state_snapshot.imu_state();
  const auto& motor = state_snapshot.motor_state();
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "dds_lowstate: samples=" << lowstate_samples << " rate_hz=" << lowstate_hz
            << " tick=" << state_snapshot.tick() << " motors=" << motor.size()
            << " crc=0x" << std::hex << state_snapshot.crc() << std::dec << '\n';
  std::cout << "device_state: mode_pr=" << static_cast<unsigned>(state_snapshot.mode_pr())
            << " mode_machine=" << static_cast<unsigned>(state_snapshot.mode_machine())
            << " imu_rpy=" << imu.rpy()[0] << "," << imu.rpy()[1] << "," << imu.rpy()[2]
            << " imu_accel=" << imu.accelerometer()[0] << "," << imu.accelerometer()[1]
            << "," << imu.accelerometer()[2] << " motor0_q=" << motor[0].q()
            << " motor0_temp=" << motor[0].temperature()[0] << '\n';

  std::cout << "odometry_candidates: samples="
            << odometry_count.load(std::memory_order_relaxed)
            << " topics=[rt/lio_sam_ros2/mapping/re_location_odometry,rt/odom,odom,rt/utlidar/odometry]"
            << '\n';
  std::cout << "lidar_candidates: samples="
            << (pointcloud_count.load(std::memory_order_relaxed) +
                lidar_state_count.load(std::memory_order_relaxed))
            << " topics=[rt/rslidar_points,rt/pointcloud,rt/utlidar/cloud,rt/lidar,rt/utlidar/lidar_state]"
            << '\n';
  std::cout << "slam_candidates: samples="
            << slam_pose_count.load(std::memory_order_relaxed)
            << " topics=[rt/slam/pose,rt/utlidar/robot_pose]" << '\n';

  print_result("dds_communication", lowstate_samples > 0,
               "rt/lowstate received; rt/lowcmd writer created, no command written");
  print_result("device_status_service", service_code == 0,
               "RobotStateClient::ServiceList");
  print_result("low_power_status", low_power_code == 0,
               "RobotStateClient::LowPowerStatus (G1 firmware may not expose this API)");
  print_result("realsense_camera", camera_code == 0 && camera_jpeg,
               "go2::VideoClient::GetImageSample (front videohub)");
  print_result("odometry_service", odometry_count.load(std::memory_order_relaxed) > 0,
               "candidate DDS topics");
  print_result("lidar_service",
               pointcloud_count.load(std::memory_order_relaxed) > 0 ||
                   lidar_state_count.load(std::memory_order_relaxed) > 0,
               "official rt/rslidar_points plus compatibility topics");
  print_result("slam_navigation_service", slam_pose_count.load(std::memory_order_relaxed) > 0,
               "candidate DDS topics");
  return lowstate_samples > 0 && service_code == 0 ? 0 : 1;
}
