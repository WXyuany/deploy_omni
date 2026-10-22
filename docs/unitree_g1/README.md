# Unitree G1 使用手册

[首页](../../README.md) · [通用开发指南](../development.md)

[连接与编译](#连接与编译) · [一行命令验证](#一行命令验证) · [高层运控](#高层运控) · [关节与 IMU](#关节与-imu) · [相机音频与灯光](#相机音频与灯光) · [点云与里程计](#点云与里程计) · [SDK 接口](#sdk-接口)

已有状态及部分服务真机记录。本文使用项目内置 SDK2 的 G1 `LocoClient`。

## 连接与编译

在控制电脑上 SSH 登录当前这台机器人：

| 连接方式 | 命令 |
|---|---|
| 网线直连 | `ssh unitree@192.168.123.164` |
| 电脑与机器人均连接 `MEETINGROOM_WIFI` | `ssh unitree@172.16.201.79` |

登录密码：`123`。进入机器人上的项目目录后执行：

```bash
bash scripts/build_robot.sh g1
./build-g1-only/unitree_g1_motion_demo --check --interface eth0
```

也可以在 Linux 上位机运行；用 `ip -br address` 查看网卡，将 `eth0` 换成连接机器人的网卡名。macOS 构建方式见 [Docker](../development.md#docker)。

正常输出应包含 `CheckMode code=0`、`GetFsmId code=0`、`LowState fresh/CRC/health checks: PASS`。
程序默认只读，`--interface` 填网卡名，DDS domain 为 0。

## 一行命令验证

在 Linux 项目根目录运行。脚本自动增量编译所需程序，默认网卡 `eth0`；其他网卡在命令后加 `--interface 网卡名`。

| 验证功能 | 一行命令 | 结果怎么看 |
|---|---|---|
| 状态与运控就绪检查，只读 | `python3 scripts/verify_robot.py g1 state` | FSM 查询返回 0，LowState 新鲜、CRC 和健康检查通过 |
| 设备服务列表，只读 | `python3 scripts/verify_robot.py g1 device` | `ServiceList code=0`，服务数量大于 0；`status=0` 在线 |
| 前置相机 | `python3 scripts/verify_robot.py g1 camera` | JPEG 首尾标记有效，打印 `jpeg_saved`；打开保存的图片查看 |
| 音频 | `python3 scripts/verify_robot.py g1 audio` | 音量读回一致，TTS／播放接口成功；现场听到英文测试语音，结束恢复原音量 |
| 灯光 | `python3 scripts/verify_robot.py g1 led` | API 返回 0，现场依次看到红、绿、蓝各约 1 秒，最后熄灭 |
| 雷达点云 | `python3 scripts/verify_robot.py g1 pointcloud` | 10 秒内 `pointcloud samples` 大于 0，查看点云字段和字节数 |
| SLAM 里程计 | `python3 scripts/verify_robot.py g1 odometry` | 10 秒内 `odometry samples` 大于 0，显示 XYZ 和四元数 |
| 高层键盘运控 | `python3 scripts/verify_robot.py g1 motion` | 按下方流程人工操作，核对实际动作与停车效果 |
| 离线回归 | `python3 scripts/verify_robot.py g1 offline` | CTest 显示 `100% tests passed`，不连接机器人 |

点云、里程计先启动对应驱动／定位服务；采样时间可加 `--seconds 30`。
每次结果摘要存入 `logs/verification/` 下的独立目录，相机图片保存在同一目录。
`PASS` 表示本项数据／自动检查通过，`API_OK` 表示接口调用通过、声光效果需现场核对，`COMPLETED` 仅表示交互运控已结束；失败为非零退出码。

## 高层运控

### 启动与操作顺序

```bash
# 离线熟悉按键
./build-g1-only/unitree_g1_motion_demo --dry-run

# 连接机器人后，接受键盘动作
./build-g1-only/unitree_g1_motion_demo --enable-motion --interface eth0
```

动作键均为单个小写字母，短按一次，无需回车或 `!`。启动后不会自动改变姿态。
操作时准备防跌倒保护和原厂遥控器；零力矩、阻尼均不保证站立支撑。

1. 按 `i` 查看状态。若 FSM=0，在可靠支撑下按 `p` 请求阻尼。
2. 确认 FSM=1，按 `u` 起立，等待并目视确认起立完成。
3. 确认 FSM=4，按 `t` 请求运动状态 FSM=500。
4. 确认 FSM=500，按 `b` 请求平衡站立，等待站稳。
5. 确认 FSM=500、balance=0，按 `r` 解锁一次移动，再短按 `w/s/a/d/j/l`。
6. 每次移动持续 0.5 秒并尝试停车，下一次移动需要重新按 `r`。

已处于 FSM=500、balance=0 时从步骤 5 开始。状态不符或请求失败时停止该步骤，按厂商流程恢复。
姿态请求间隔为 3 秒，动作是否完成需观察反馈。

### 键盘速查

| 键 | 功能／官方 API | 条件 |
|---|---|---|
| `z` | 零力矩 `ZeroTorque()`，FSM=0 | 可靠支撑 |
| `p` | 阻尼 `Damp()`，FSM=1 | 可靠支撑 |
| `u` | 起立 `StandUp()`，FSM=4 | 当前 FSM 为 1/2/3/4 |
| `t` | 运动状态 `Start()`，FSM=500 | 当前 FSM 为 4/500 |
| `b` | 平衡站立 `BalanceStand()`，balance=0 | FSM=500 |
| `c/v` | 下蹲／坐下 `Squat()/Sit()`，FSM=2/3 | 当前 FSM 为 4/500 |
| `o/n` | 高／低站姿 `HighStand()/LowStand()` | FSM=500、balance=0 |
| `h` | 挥手 `WaveHand(false)` | FSM=500、balance=0 |
| `k` | 伸手握手 `ShakeHand(0)` | FSM=500、balance=0 |
| `m` | 收手 `ShakeHand(1)` | FSM=500 |
| `r` | 解锁一次移动，只查询状态 | FSM=500、balance=0、健康且无未结束握手 |
| `w/s` | 前／后 `SetVelocity(±0.10,0,0,0.5)`，m/s | 先按 `r` |
| `a/d` | 自身坐标左／右 `SetVelocity(0,±0.08,0,0.5)`，m/s | 先按 `r` |
| `j/l` | 左／右转 `SetVelocity(0,0,±0.15,0.5)`，rad/s | 先按 `r` |
| 空格 / `x` | `StopMove()`，取消解锁并尝试收回本程序伸出的握手 | 随时 |
| `q` / Ctrl+C | 尝试停车并退出 | 随时 |
| `i` / `?` | 状态／帮助，取消未使用的移动解锁 | 不发送动作 |

速度脉冲不等于精确位移。`StopMove()` 仅停止行走速度，不能取消起立、坐下等动作，也不代替硬件急停。
程序不提供避障；不要与低层 `adapter_bridge` 或其他运控客户端同时下发。

### 自动保护

- 移动后取消解锁；错误回复尝试停车，不自动重发动作。
- 行走前检查 FSM、平衡模式、CRC 和健康状态；LowState 超过 500 ms 无有效新 tick 时尝试停车。
- 准入阈值：roll/pitch 绝对值不超过 0.75 rad，前 29 槽位位置／速度／力矩有限，`motorstate=0`，两路温度分别不超过 80/100。
- 丢弃粘贴、积压和连发动作，停车与退出优先。
- 输入断开或收到 SIGINT/TERM/HUP/QUIT/TSTP 时尝试清理；Ctrl+Z 会退出。强制终止、断电或断网时清理请求可能无法到达。

### 状态与错误码

| 结果 | 含义 |
|---|---|
| `GetBalanceMode code=-1`，FSM 非 500 | 程序未发出平衡模式查询 |
| `7301` | LocoState 不可用，检查运动服务与状态 |
| `7302` / `7303` | 无效 FSM ID／task ID |
| `-10003` / `-10004` | 本地因退出信号／查询期间的新输入取消动作 |

`LowState.mode_machine` 与高层 FSM 是不同字段。非零回复先查原因。

## 关节与 IMU

`rt/lowstate` 包含关节、IMU、tick、CRC 和遥控器数据。观察真实 topic 的频率：

```bash
cp config/g1.sample.yaml config/g1.local.yaml
# 修改本地配置：dds.topics.state.name=rt/lowstate
# adapter.connection.interface_hint 改为实际网卡
./build-g1-only/lowstate_frequency_receiver config/g1.local.yaml
./build-g1-only/lowstate_field_recorder --help
```

[g1.sample.yaml](../../config/g1.sample.yaml) 默认连接模拟 topic：上层 `bridge/rt/*`、下层 `custom/mujoco/rt/*`。
真实低层控制需配置设备 topic 和关节映射。默认低层 adapter 会释放官方运动服务，高层应用应独立使用 SDK facade。

<details>
<summary>低层状态与命令字段对照</summary>

### 状态字段

路径：`LowState_ → UnitreeG1LowLevelSession → UnitreeG1Adapter → StateFrame`。

| Bridge 字段 | Unitree 来源 |
|---|---|
| `sample_time_ns / publish_time_ns` | 本地采样时间，映射时两者相同 |
| `low_state_version / low_state_reserve` | `version() / reserve()` |
| `tick / mode_pr / mode_machine` | 同名字段 |
| `valid / stale / degraded` | 转换成功、缓存超时及降级标志 |
| `joints.position / velocity / effort / acceleration` | `motor_state()[i].q() / dq() / tau_est() / ddq()` |
| `joints.motor_mode / motor_temperature / motor_voltage` | `mode() / temperature() / vol()` |
| `joints.motor_sensor / motor_state / motor_reserve` | `sensor() / motorstate() / reserve()` |
| `wireless_remote[40]` | 原样透传 `wireless_remote()` |
| `imu.orientation_xyzw` | `quaternion()` 从 wxyz 重排为 xyzw |
| `imu.rpy_xyz / angular_velocity_xyz / linear_acceleration_xyz` | `rpy() / gyroscope() / accelerometer()` |
| `imu.temperature` | `temperature()` |

配置独立 IMU topic 且收到最新样本时优先使用，否则使用 `LowState_.imu_state()`。

### 命令字段

路径：`DDS LowCmd_ → CommandFrame → 关节映射 → UnitreeG1LowLevelSession → LowCmd_`。

| Unitree 字段 | Bridge 来源 | 缺省值 |
|---|---|---|
| `mode_pr() / mode_machine()` | `CommandFrame` 同名字段 | 按帧配置 |
| `crc()` | 存在时沿用 `low_cmd_crc`，否则计算 | 计算 |
| `motor_cmd()[i].mode()` | `joints.motor_mode[i]` | `1` |
| `motor_cmd()[i].q() / dq() / tau()` | `joints.position / velocity / effort` | `0.0` |
| `motor_cmd()[i].kp() / kd()` | `joints.stiffness / damping` | `0.0` |

填充前清零所有电机命令槽位。`write_safe_command()` 使用 `mode_pr=0`、已保存的 `mode_machine`，所有电机命令及 mode 置零，并重算 CRC。

实现：[低层会话](../../src/bridge/adapters/unitree_g1/unitree_g1_low_level_session.cpp)、[状态映射](../../src/bridge/adapters/unitree_g1/unitree_g1_adapter.cpp)、[DDS 转换](../../src/bridge/dds/dds_translation.cpp)、[命令打包](../../src/bridge/adapters/unitree_g1/unitree_g1_low_level_protocol.cpp)。

</details>

<details>
<summary>手柄 DDS 数据示例</summary>

示例验证 `wireless_remote[40]` 的发送与解码，不控制机器人。需要 SDL2：

```bash
sudo apt-get install -y libsdl2-dev
bash scripts/build_demo.sh --test
```

在两个终端分别运行接收端和发送端，发送端插入手柄：

```bash
./build-g1-only/gamepad_dds_receiver config/g1.sample.yaml
./build-g1-only/gamepad_dds_sender config/g1.sample.yaml 5 0
```

`5` 是发送频率 Hz，`0` 是手柄索引。输出应随左右摇杆、扳机和按键变化。
SDL 映射异常时，在配置的 gamepad 模块中设置 `mapping_path: config/gamecontrollerdb.txt`；设备条目可从 [SDL_GameControllerDB](https://github.com/gabomdq/SDL_GameControllerDB) 获取。

</details>

## 相机、音频与灯光

使用独立的 `UnitreeG1SdkFacade`，初始化 DDS 后调用 `sdk.initialize()`，用 `result.succeeded()` 检查结果，厂商成功码为 `0`。

| 能力 | 方法 |
|---|---|
| TTS | `tts(text, speaker_id)` |
| 音量 | `get_volume / set_volume` |
| PCM 音频 | `play_stream / stop_stream` |
| RGB 灯 | `set_rgb_led(r,g,b)` |
| 前置相机 | `get_front_camera_jpeg(&bytes)` |

前置相机接口返回 JPEG，使用 G1 固件兼容的 `VideoClient`，不包含深度图或相机内参。

```cpp
#include <unitree/robot/channel/channel_factory.hpp>
#include "bridge/adapters/unitree_g1/unitree_g1_sdk.hpp"

unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
bridge::adapters::unitree_g1::UnitreeG1SdkFacade sdk;
if (!sdk.initialize().succeeded()) return;

int fsm = -1;
const auto state = sdk.get_fsm_id(&fsm);
const auto speech = sdk.tts("hello", 0);
const auto led = sdk.set_rgb_led(0, 255, 0);
std::vector<std::uint8_t> jpeg;
const auto camera = sdk.get_front_camera_jpeg(&jpeg);
// 分别检查 state/speech/led/camera 的 succeeded()，并检查 JPEG 非空。
static_cast<void>(sdk.shutdown());
```

## 点云与里程计

`UnitreeG1SensorSession` 只订阅数据。先启动 MID360 驱动发布 `PointCloud2`；里程计还需 SLAM／定位服务发布 `Odometry`。
Livox 自定义消息需要转换为 `PointCloud2`。

| 数据 | 默认原始 DDS topic | 类型 |
|---|---|---|
| 点云 | `rt/rslidar_points` | `sensor_msgs::msg::dds_::PointCloud2_` |
| SLAM 里程计 | `rt/lio_sam_ros2/mapping/re_location_odometry` | `nav_msgs::msg::dds_::Odometry_` |

```bash
# 默认读取点云 10 秒
./build-g1-only/unitree_g1_sensor_reader --interface eth0

# 读取里程计
./build-g1-only/unitree_g1_sensor_reader --interface eth0 --stream odometry

# 同时读取，按实际发布者修改 topic
./build-g1-only/unitree_g1_sensor_reader \
  --interface eth0 --stream both --duration 30 \
  --cloud-topic rt/rslidar_points \
  --odom-topic rt/lio_sam_ros2/mapping/re_location_odometry
```

输出点云布局、字段、坐标系、字节数和里程计 XYZ／四元数。`samples` 为累计帧数。
退出码：`0` 选中流均收到消息，`2` 有流未收到消息，`1` 运行错误，`64` 参数错误。
没有数据时检查发布进程、网卡、domain、topic、类型与 QoS；ROS2 `/xxx` 在默认 DDS 命名映射下通常对应 `rt/xxx`。

### C++ 读取

头文件：[unitree_g1_sensor_session.hpp](../../include/bridge/adapters/unitree_g1/unitree_g1_sensor_session.hpp)。自定义目标链接 `bridge_adapter_unitree_g1`。

```cpp
#include "bridge/adapters/unitree_g1/unitree_g1_sensor_session.hpp"

using namespace bridge::adapters::unitree_g1;
UnitreeG1SensorSessionConfig config;
config.transport_interface = "eth0";
UnitreeG1SensorSession sensors(config);
if (!sensors.initialize()) return;

// 在业务循环中读取，后台 DDS 回调更新数据。
const auto snapshot = sensors.snapshot();
if (snapshot.pointcloud) {
  const auto& cloud = *snapshot.pointcloud;
  // header(), fields(), width(), height(), point_step(), row_step(), data()
}
if (snapshot.odometry) {
  const auto& pose = snapshot.odometry->pose().pose();
  // pose.position(): XYZ；pose.orientation(): XYZW
}
sensors.shutdown();  // 业务循环结束后调用
```

初始化建立 reader，需检查 optional 数据及 `pointcloud_samples / odometry_samples`。
点云按字段偏移、类型、跨度和字节序解析。多个组件共用 `ChannelFactory` 时使用相同网卡和 domain。

## SDK 接口

公开头文件：[unitree_g1_sdk.hpp](../../include/bridge/adapters/unitree_g1/unitree_g1_sdk.hpp)。所有方法返回 `UnitreeG1SdkResult`，记录 `operation / code / detail`。

| 分类 | 接口 |
|---|---|
| 状态查询 | `get_fsm_id`、`get_fsm_mode`、`get_balance_mode`、`get_swing_height`、`get_stand_height`、`get_phase` |
| 状态与参数设置 | `set_fsm_id`、`set_balance_mode`、`set_swing_height`、`set_stand_height`、`set_velocity`、`set_arm_task`、`set_speed_mode` |
| 姿态 | `damp`、`start`、`squat`、`sit`、`stand_up`、`high_stand`、`low_stand`、`zero_torque`、`balance_stand` |
| 行走 | `stop_move`、`move`、`continuous_gait`、`switch_move_mode` |
| 手臂 | `wave_hand`、`shake_hand`、`get_arm_action_list`、`execute_arm_action` |
| 音频、灯光、相机 | `tts`、`get_volume`、`set_volume`、`play_stream`、`stop_stream`、`set_rgb_led`、`get_front_camera_jpeg` |
| 运动服务管理 | `check_motion_mode`、`select_motion_mode`、`release_motion_mode`、`set_silent`、`get_silent` |

未初始化返回 `kUnitreeG1SdkNotInitialized`，参数错误返回 `kUnitreeG1SdkInvalidArgument`，其他非零值为 SDK 返回码。
手臂动作编号通过 `get_arm_action_list` 获取。`release_motion_mode()` 用于低层接管，高层运控不调用。

<details>
<summary>硬件接口检查与已有记录</summary>

```bash
cmake -S . -B build-g1-check \
  -DBRIDGE_ROBOT=unitree_g1 -DBUILD_UNITREE_G1_HARDWARE_TESTS=ON
cmake --build build-g1-check \
  --target unitree_g1_sdk_check unitree_g1_services_check -j2
./build-g1-check/unitree_g1_sdk_check
./build-g1-check/unitree_g1_services_check
```

SDK 检查支持 `--test device|audio|led|camera|metadata` 单项选择；音频结束恢复原音量，RGB 测试后设为黑色，相机保存 JPEG。
服务检查只读订阅 LowState、查询设备服务并探测传感器 topic，不启停驱动或 SLAM。两者不执行运控。

| 已有真机记录 | 结果 |
|---|---|
| DDS、关节与 IMU | LowState 约 1041.6 Hz，35 电机槽位、IMU、tick、CRC 可读 |
| 设备服务 | `ServiceList` 返回 0，共 29 项；`status=0` 在线，`1` 未运行 |
| 音频与灯光 | TTS、音量、静音、RGB、PCM 播放／停止返回 0 |
| 时间同步 | chrony 同步源 `192.168.123.161`，`Leap status: Normal` |
| 前置相机 | Intel RealSense D435i，JPEG 返回成功 |

`LowPowerStatus` 的已有返回码为 `3104`，该次查询失败，不影响服务列表查询。

</details>

### 源码与官方资料

- [键盘程序](../../examples/unitree_g1/unitree_g1_motion_demo.cpp)、[运控策略](../../examples/unitree_g1/unitree_g1_motion_control.hpp)、[策略测试](../../tests/demo/unitree_g1_motion_control_test.cpp)、[终端测试](../../tests/demo/unitree_g1_motion_terminal_test.py)。
- [官方开发文档](https://support.unitree.com/home/zh/G1_developer)、[官方 G1 LocoClient](https://github.com/unitreerobotics/unitree_sdk2/blob/master/include/unitree/robot/g1/loco/g1_loco_client.hpp)。
- 固定 SDK 版本的 [API 编号](https://github.com/Zhijie-He/robot_shared/blob/a801170c41b1b0b527468752b32502a438da3667/library/unitree_sdk2/include/unitree/robot/g1/loco/g1_loco_api.hpp) 与 [错误码](https://github.com/Zhijie-He/robot_shared/blob/a801170c41b1b0b527468752b32502a438da3667/library/unitree_sdk2/include/unitree/robot/g1/loco/g1_loco_error.hpp)。
