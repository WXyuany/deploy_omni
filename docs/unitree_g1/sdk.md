# Unitree G1 SDK 接口使用指南

本文档说明本项目对 Unitree SDK2 的 G1 高层接口封装，以及与低层关节 DDS 控制的边界。

官方入口：<https://support.unitree.com/home/zh/G1_developer>

## 能力范围

当前 `UnitreeG1SdkFacade` 只封装仓库内 SDK2 中可以确认属于 G1 或已经用于 G1 低层会话的 client：

| 能力 | SDK client | 项目接口 |
| --- | --- | --- |
| 运动状态与移动 | `unitree::robot::g1::LocoClient` | `adapter.sdk().*` |
| 手臂动作 | `unitree::robot::g1::G1ArmActionClient` | `execute_arm_action`、`get_arm_action_list` |
| 语音与音频 | `unitree::robot::g1::AudioClient` | `tts`、`play_stream`、音量、RGB 灯 |
| 运动模式切换 | `unitree::robot::b2::MotionSwitcherClient` | `check_motion_mode`、`release_motion_mode` |
| 前置 RealSense 图像 | `unitree::robot::go2::VideoClient` | `get_front_camera_jpeg` |
| 低层关节控制 | `LowCmd_` / `LowState_` | `UnitreeG1LowLevelSession` |
| IMU | `IMUState_` | `UnitreeG1LowLevelSession` |

G1 固件的前置 `video_hub_pc4` 使用 SDK2 中与 Go2 兼容的 `VideoClient` 请求接口；本项目已将它作为明确的 G1 前置相机能力封装。该接口返回 JPEG 图像，不代表已经提供 RealSense 深度、点云或相机内参。

SLAM/LiDAR 数据使用独立的 DDS 传感器会话。默认 topic 与 Unitree 官方 `unitree_slam` 示例一致：

| 数据 | DDS 类型 | 默认 topic |
| --- | --- | --- |
| SLAM 里程计 | `nav_msgs::msg::dds_::Odometry_` | `rt/lio_sam_ros2/mapping/re_location_odometry` |
| MID360 点云 | `sensor_msgs::msg::dds_::PointCloud2_` | `rt/rslidar_points` |

对应封装是 `UnitreeG1SensorSession`，直接构造该 session 时可通过
`UnitreeG1SensorSessionConfig` 覆盖 topic；通过 `adapter.sensors()` 使用时采用上表默认值。
它只订阅数据，不发送建图或导航命令。

## 生命周期

`UnitreeG1Adapter::initialize()` 在真实 DDS 模式下初始化低层 session、传感器 session 和 SDK facade；首帧状态由后续读取流程检查，初始化成功不表示已有有效状态。`shutdown()` 关闭三者。高层应用可以独立初始化 facade，无需构造 adapter；未初始化的 facade 返回失败结果。

## 编译和头文件

高层程序可独立使用 `UnitreeG1SdkFacade`，先初始化 DDS，再初始化 facade。不要为了调用高层接口而初始化默认低层 adapter：低层 session 的 `require_motion_release` 默认开启，会释放高层依赖的运动服务。

`BRIDGE_ROBOT=unitree_g1` 选择对应 adapter；交互 demo 额外需要 `BUILD_UNITREE_G1_MOTION_DEMO=ON`。
Demo 的 SDK 后端集中在一个文件中，以便单独设置 0.3 秒 RPC 超时；其余交互策略不依赖 DDS，可以离线测试。

## 调用示例

以下只查询运动状态，不自动起立、行走或切换模式：

```cpp
#include <unitree/robot/channel/channel_factory.hpp>
#include "bridge/adapters/unitree_g1/unitree_g1_sdk.hpp"

unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
bridge::adapters::unitree_g1::UnitreeG1SdkFacade sdk;
const auto initialized = sdk.initialize();
if (!initialized.succeeded()) return;
int fsm = -1;
const auto result = sdk.get_fsm_id(&fsm);
// 检查 result.code；fsm 是高层状态机编号，不是 LowState.mode_machine。
static_cast<void>(sdk.shutdown());
```

其它接口沿用同一返回值约定；在上面的 `sdk.initialize()` 成功后、`sdk.shutdown()` 之前按需调用：

```cpp
std::string actions;
const auto action_list = sdk.get_arm_action_list(&actions);
// 动作编号从本机返回列表与配套官方文档核对后，再调用 execute_arm_action。

const auto speech = sdk.tts("hello", 0);
const auto led = sdk.set_rgb_led(0, 255, 0);

std::vector<std::uint8_t> jpeg;
const auto camera = sdk.get_front_camera_jpeg(&jpeg);

std::string form;
std::string mode;
const auto current_mode = sdk.check_motion_mode(&form, &mode);
// release_motion_mode() 仅用于明确的低层接管流程，高层运控不调用。
```

生产代码应按业务需要记录每个结果；示例中的 `action_list`、`speech`、`led`、`camera` 和 `current_mode` 都不能忽略。

读取传感器：

```cpp
#include "bridge/adapters/unitree_g1/unitree_g1_sensor_session.hpp"

auto& sensors = adapter.sensors();
const auto snapshot = sensors.snapshot();
if (snapshot.odometry.has_value()) {
  const auto& pose = snapshot.odometry->pose().pose().position();
  // pose.x(), pose.y(), pose.z()
}
if (snapshot.pointcloud.has_value()) {
  const auto& cloud = *snapshot.pointcloud;
  // cloud.width(), cloud.height(), cloud.fields(), cloud.data()
}
// adapter.shutdown() 统一关闭传感器会话。
```

`initialize()` 成功只表示 DDS reader 已创建；必须检查 `odometry_samples` 和 `pointcloud_samples`。如果对应机器人服务没有运行，快照会保持空。

高层动作接口和低层关节控制不能并发争抢同一运动控制服务。使用低层 `LowCmd_` 前，应先确认运动模式并按官方要求释放占用服务。

## 交互式高层运控 Demo

完整命令、状态前置条件、离线演练和真机操作步骤见 [G1 高层运控操作手册](high-level-motion.md)。
程序默认只读；使用 `--enable-motion` 才接受控制键。`Start()` 是切换到 FSM 500，`StopMove()` 只停止行走速度，不是通用动作取消或硬件急停。

## 接口索引

运动控制：`get_fsm_id`、`get_fsm_mode`、`get_balance_mode`、`get_swing_height`、`get_stand_height`、`get_phase`、`set_fsm_id`、`set_balance_mode`、`set_swing_height`、`set_stand_height`、`set_velocity`、`set_arm_task`、`set_speed_mode`、`damp`、`start`、`squat`、`sit`、`stand_up`、`high_stand`、`low_stand`、`zero_torque`、`stop_move`、`move`、`balance_stand`、`continuous_gait`、`switch_move_mode`、`wave_hand`、`shake_hand`。

手臂动作：`get_arm_action_list`、`execute_arm_action`。

语音与灯光：`tts`、`get_volume`、`set_volume`、`play_stream`、`stop_stream`、`set_rgb_led`。

前置相机：`get_front_camera_jpeg`。

运动模式：`check_motion_mode`、`select_motion_mode`、`release_motion_mode`、`set_silent`、`get_silent`。

## 返回值

所有方法返回 `UnitreeG1SdkResult`。`code == 0` 表示 SDK 调用成功；`kUnitreeG1SdkNotInitialized` 表示 facade 未初始化；`kUnitreeG1SdkInvalidArgument` 表示调用方参数非法；其他非零值是 Unitree SDK 返回码。业务层必须检查 `succeeded()` 并记录 `operation`、`code`、`detail`。

## 真机接口检查

硬件测试默认关闭，避免普通构建触碰真实机器人。连接 G1 后，在 ARM64 Linux 环境执行：

```bash
cmake -S . -B build-g1-robot \
  -DBRIDGE_ROBOT=unitree_g1 \
  -DBUILD_UNITREE_G1_HARDWARE_TESTS=ON
cmake --build build-g1-robot --target unitree_g1_sdk_check
./build-g1-robot/unitree_g1_sdk_check
```

接口检查覆盖服务方法，并额外调用 `get_front_camera_jpeg` 检查返回值和 JPEG 文件头。音量和静音会尝试恢复到原值，RGB 测试后设为黑色（不是恢复原颜色）；不调用 Loco、AGV、低层关节写入、手臂动作执行或模式选择/释放。

## 真机服务矩阵

服务矩阵测试默认关闭，连接 G1 后在 ARM64 Linux 环境执行：

```bash
cmake -S . -B build-g1-robot \
  -DBRIDGE_ROBOT=unitree_g1 \
  -DBUILD_UNITREE_G1_HARDWARE_TESTS=ON
cmake --build build-g1-robot --target unitree_g1_services_check
./build-g1-robot/unitree_g1_services_check
```

该测试只读订阅 `rt/lowstate`，创建 `rt/lowcmd` writer 但不写入命令；同时调用 `RobotStateClient::ServiceList` 和 `LowPowerStatus`，并探测官方 SLAM/LiDAR topic。`ServiceList` 的 `status=0` 表示服务在线；`status=1` 表示当前未运行。`LowPowerStatus` 返回 `3104` 时是本次调用失败；仅凭该返回码不判定全部 G1 固件是否支持此 B2 兼容 API，不影响 `ServiceList` 结果。测试会尝试启动 `lidar_driver` 后恢复原状态，但不会启动 `unitree_slam` 或发送导航指令。

以下为迁移前工作区记录的 G1 真机结果，本次发布整理未重新连接设备：

| 类别 | 结果 | 证据 |
| --- | --- | --- |
| DDS 通信 | 通过 | `rt/lowstate` 3 秒收到 3125 帧，约 1041.6 Hz；35 电机、IMU、tick、CRC 可读；`rt/lowcmd` writer 创建成功，未发送控制命令 |
| 设备状态服务 | 通过 | `ServiceList` 返回码 0，共 29 项；`state_estimator`、`video_hub_pc4`、`audio_player_service` 在线 |
| 音频灯光服务 | 通过 | TTS、音量读写、静音读写、RGB、PCM 播放/停止均返回 0，音量与静音尝试恢复，RGB 最后设为黑色 |
| 时间同步 | 通过 | chrony 同步源 `192.168.123.161`，`Leap status: Normal`，实测偏差约微秒级 |
| 前置 RealSense | 通过 | `/sys` 标识为 Intel RealSense Depth Camera 435i；`get_front_camera_jpeg` 返回合法 JPEG，本次 118,501 字节 |


## 雷达点云与里程计调用

项目提供独立的点云和里程计订阅接口，完整读取命令、topic 配置与 C++ 调用见 [雷达与里程计](lidar.md)。安装并启动 MID360 驱动后可读取点云；SLAM 里程计另需启动对应定位服务。
