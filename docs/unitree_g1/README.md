# Unitree G1 接入文档

本目录记录本项目对 Unitree G1 的接入边界、调用方式和真机验证结果。

## 从哪里开始

- [SDK 与传感器使用指南](../unitree_g1_sdk_usage.md)：高层 SDK、前置相机、SLAM/LiDAR 订阅和烟测命令。
- [高层运控操作手册](high-level-motion.md)：独立编译、离线演练、模式转换、WASD、动作与中断。
- [状态与命令字段](state-and-command-fields.md)：低层 `LowState_` / `LowCmd_` 映射。
- [低层接入计划](2026-05-06-001-feat-g1-low-level-guide-plan.md)：低层控制的设计背景。

## 架构边界

G1 接入分为三条数据面：

| 数据面 | 入口 | 用途 |
| --- | --- | --- |
| 运动控制 SDK | 独立 `UnitreeG1SdkFacade`，或已初始化的 `UnitreeG1Adapter::sdk()` | 移动、手臂动作、音频、灯光、运动模式 |
| 低层 DDS | `UnitreeG1LowLevelSession` | `rt/lowstate`、`rt/lowcmd`、IMU、关节状态 |
| 传感器 DDS / 视频服务 | `UnitreeG1Adapter::sensors()`、`sdk().get_front_camera_jpeg()` | SLAM 里程计、MID360 点云、前置 RealSense JPEG |

图像和点云不写入通用 `StateFrame`。它们保留各自的消息类型和生命周期，避免高频二进制数据污染运动状态通道。
单独调用高层服务时优先使用独立 facade。默认低层 adapter 会释放运动服务，不能为了调用高层运控而直接初始化它；详见高层运控手册。

## 已确认接口

### 前置相机

G1 当前运行 `video_hub_pc4`，硬件为 Intel RealSense D435i。通过 `go2::VideoClient::GetImageSample` 可以取得 JPEG；项目对外接口是：

```cpp
std::vector<std::uint8_t> jpeg;
const auto result = adapter.sdk().get_front_camera_jpeg(&jpeg);
```

`result.code == 0` 且 JPEG 头为 `FF D8` 时表示本次读取成功。该接口只返回 JPEG，不承诺深度图、相机内参或 RealSense 原始帧。

### SLAM 里程计与 MID360 点云

项目使用 Unitree 官方 `unitree_slam` 示例确认的 DDS topic：

| 数据 | DDS 类型 | Topic |
| --- | --- | --- |
| SLAM 里程计 | `nav_msgs::msg::dds_::Odometry_` | `rt/lio_sam_ros2/mapping/re_location_odometry` |
| MID360 点云 | `sensor_msgs::msg::dds_::PointCloud2_` | `rt/rslidar_points` |

```cpp
auto& sensors = adapter.sensors();
if (sensors.initialize()) {
  const auto snapshot = sensors.snapshot();
  // snapshot.odometry_samples
  // snapshot.pointcloud_samples
}
```

`initialize()` 只创建 reader；数据是否可用由样本计数和 `std::optional` 状态判断。topic 没有发布者时，空快照是正常结果。

## 真机验证结论

本次 G1 验证于 2026-10-14 在 ARM64 板端完成，未发送运动、关节、建图或导航命令：

| 项目 | 结果 |
| --- | --- |
| `rt/lowstate` DDS | 通过，约 1042 Hz，35 电机和 IMU 可读 |
| 音频、灯光服务 | 通过，SDK 返回码均为 0 |
| 前置 RealSense JPEG | 通过，返回合法 JPEG，约 118 KB |
| `ServiceList` | 通过，返回 29 项服务 |
| `rt/lio_sam_ros2/mapping/re_location_odometry` | 0 帧 |
| `rt/rslidar_points` | 0 帧 |
| `lidar_driver` | 状态为 1，启动调用返回 3104 |
| `unitree_slam` | 状态为 1 |

因此，当前机器人上相机数据面可直接使用；点云和 SLAM 里程计的数据接口已经接入，但当前系统镜像没有启动对应服务，不能从 DDS 收到样本。重新验证时应先看 `RobotStateClient::ServiceList`，再看上述两个官方 topic，不要从 `rt/odom` 或 `rt/pointcloud` 等猜测名称开始排查。

## 构建与测试

普通构建不会触碰真实机器人。硬件测试需要 ARM64 Linux 和 `BUILD_UNITREE_G1_HARDWARE_TESTS=ON`：

```bash
cmake -S . -B build-g1-robot \
  -DBRIDGE_ROBOT=unitree_g1 \
  -DBUILD_UNITREE_G1_HARDWARE_TESTS=ON
cmake --build build-g1-robot --target \
  unitree_g1_sdk_robot_smoke unitree_g1_robot_services_smoke
```

官方参考：

- [G1 Developer 文档](https://support.unitree.com/home/zh/G1_developer)
- [Unitree SLAM 示例](https://github.com/unitreerobotics/unitree_slam)
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)
