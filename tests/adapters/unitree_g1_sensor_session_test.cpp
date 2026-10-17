#include <cassert>

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
  return 0;
}
