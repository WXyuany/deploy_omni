#include "bridge/adapters/unitree_g1/unitree_g1_sensor_session.hpp"

#include <charconv>
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using namespace bridge::adapters::unitree_g1;
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }

struct Options {
  UnitreeG1SensorSessionConfig session;
  std::string stream{"pointcloud"};
  int duration{10};
};

void help() {
  std::cout << "Usage: unitree_g1_sensor_reader [--interface eth0]\n"
               "  [--stream pointcloud|odometry|both] [--duration SECONDS]\n"
               "  [--cloud-topic rt/rslidar_points]\n"
               "  [--odom-topic rt/lio_sam_ros2/mapping/re_location_odometry]\n"
               "Reads DDS PointCloud2/Odometry; never starts services or sends motion.\n"
               "Default: pointcloud, 10 seconds. Ctrl+C stops reading.\n"
               "Exit: 0=selected streams received; 2=missing samples; 1=error; 64=invalid arguments.\n";
}

Options parse(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (i + 1 == argc) throw std::invalid_argument("Missing value: " + key);
    const std::string value = argv[++i];
    if (value.empty()) throw std::invalid_argument("Empty value: " + key);
    if (key == "--interface") options.session.transport_interface = value;
    else if (key == "--cloud-topic") options.session.pointcloud_topic = value;
    else if (key == "--odom-topic") options.session.odometry_topic = value;
    else if (key == "--stream") options.stream = value;
    else if (key == "--duration") {
      const auto result = std::from_chars(value.data(), value.data() + value.size(), options.duration);
      if (result.ec != std::errc{} || result.ptr != value.data() + value.size() ||
          options.duration < 1 || options.duration > 86400)
        throw std::invalid_argument("--duration must be an integer from 1 to 86400");
    } else throw std::invalid_argument("Unknown option: " + key);
  }
  if (options.stream != "pointcloud" && options.stream != "odometry" && options.stream != "both")
    throw std::invalid_argument("--stream must be pointcloud, odometry or both");
  return options;
}

void print(const UnitreeG1SensorSnapshot& snapshot, const Options& options) {
  if (options.stream != "odometry") {
    std::cout << "pointcloud samples=" << snapshot.pointcloud_samples;
    if (snapshot.pointcloud) {
      const auto& cloud = *snapshot.pointcloud;
      std::cout << " frame=" << cloud.header().frame_id()
                << " width=" << cloud.width() << " height=" << cloud.height()
                << " point_step=" << cloud.point_step() << " row_step=" << cloud.row_step()
                << " bigendian=" << cloud.is_bigendian() << " bytes=" << cloud.data().size()
                << " fields=";
      for (const auto& field : cloud.fields())
        std::cout << field.name() << ':' << field.offset() << ':'
                  << static_cast<int>(field.datatype()) << ':' << field.count() << ' ';
    }
    std::cout << '\n';
  }
  if (options.stream != "pointcloud") {
    std::cout << "odometry samples=" << snapshot.odometry_samples;
    if (snapshot.odometry) {
      const auto& odom = *snapshot.odometry;
      const auto& position = odom.pose().pose().position();
      const auto& rotation = odom.pose().pose().orientation();
      std::cout << " frame=" << odom.header().frame_id() << " child=" << odom.child_frame_id()
                << " xyz=" << position.x() << ',' << position.y() << ',' << position.z()
                << " quaternion_xyzw=" << rotation.x() << ',' << rotation.y() << ','
                << rotation.z() << ',' << rotation.w();
    }
    std::cout << '\n';
  }
  std::cout << std::flush;
}
}  // namespace

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == "--help") { help(); return 0; }
  Options options;
  try { options = parse(argc, argv); }
  catch (const std::invalid_argument& error) {
    std::cerr << error.what() << '\n'; help(); return 64;
  }
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  try {
    UnitreeG1SensorSession session(options.session);
    if (!session.initialize()) throw std::runtime_error("Cannot initialize sensor subscriptions");
    std::cout << "DDS domain=0 interface=" << options.session.transport_interface
              << " cloud_topic=" << options.session.pointcloud_topic
              << " odom_topic=" << options.session.odometry_topic << '\n';
    using Clock = std::chrono::steady_clock;
    const auto until = Clock::now() + std::chrono::seconds(options.duration);
    auto next_print = Clock::now();
    while (!stopped && Clock::now() < until) {
      if (Clock::now() >= next_print) {
        print(session.snapshot(), options);
        next_print = Clock::now() + std::chrono::seconds(1);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    const auto last = session.snapshot();
    print(last, options);
    session.shutdown();
    const bool received = (options.stream == "odometry" || last.pointcloud_samples > 0) &&
                          (options.stream == "pointcloud" || last.odometry_samples > 0);
    if (!received) std::cerr << "No samples on a selected stream; check driver/service, topic and interface.\n";
    return received ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
