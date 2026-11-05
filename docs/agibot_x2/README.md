# Agibot X2 使用手册

[首页](../../README.md) · [开发指南](../development.md)

[连接与编译](#连接与编译) · [一行命令验证](#一行命令验证) · [传感器](#传感器) · [灯光与音频](#灯光与音频) · [关节运动方案](#关节运动方案) · [适配器接口](#适配器接口)

| 设备 | 接口状态 |
|---|---|
| Agibot X2 | ROS2 Humble、`aimdk_msgs`、31 个本体关节，提供相机、深度、雷达、IMU、音频、灯光、SLAM 与导航接口 |

## 连接与编译

在可访问 X2 的 Linux 主机上确认网络：

```bash
ssh agi@10.0.1.41
source /opt/ros/humble/setup.bash
source /agibot/software/common/share/aimdk_msgs/local_setup.bash
```

本地只编译 X2 适配器：

```bash
bash scripts/build_robot.sh agibot_x2
```

如果在 X2 系统上编译，`aimdk_msgs` 的头文件和类型支持库位于机器人软件环境；不要把 X2 消息替换为 Unitree DDS 消息。

## 一行命令验证

```bash
python3 scripts/x2_ros2_probe.py --json
```

结果应列出 `head`、`waist`、`arm`、`leg` 四组状态和命令话题，消息类型分别为 `aimdk_msgs/msg/JointStateArray` 与 `aimdk_msgs/msg/JointCommandArray`。

读取完整关节状态：

```bash
ros2 service call /aimdk_5Fmsgs/srv/GetAllJointState \
  aimdk_msgs/srv/GetAllJointState '{}'
```

读取系统状态：

```bash
ros2 service call /aimdk_5Fmsgs/srv/GetSystemState \
  aimdk_msgs/srv/GetSystemState '{}'
```

## 传感器

| 能力 | 话题 | 类型 |
|---|---|---|
| 胸部／躯干 IMU | `/aima/hal/imu/chest/state`、`/aima/hal/imu/torso/state` | `sensor_msgs/msg/Imu` |
| 前胸雷达点云 | `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` | `sensor_msgs/msg/PointCloud2` |
| 前胸雷达 IMU | `/aima/hal/sensor/lidar_chest_front/imu` | `sensor_msgs/msg/Imu` |
| RGB 相机 | `/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed` | `sensor_msgs/msg/CompressedImage` |
| 后置 RGB | `/aima/hal/sensor/rgb_head_rear/rgb_image/compressed` | `sensor_msgs/msg/CompressedImage` |
| RGBD 深度 | `/aima/hal/sensor/rgbd_head_front/depth_image` | `sensor_msgs/msg/Image` |
| 音频采集 | `/aima/hal/audio/capture` | `aimdk_msgs/msg/AudioCapture` |

使用 `ros2 topic info -v <topic>` 检查发布者、消息类型和 QoS，再用 `ros2 topic echo --once` 读取样本。

## 灯光与音频

### 灯光

官方示例位于机器人端 `/agibot/software/aimdk/src/py_examples/py_examples/play_lights.py`。服务类型为 `aimdk_msgs/srv/SetPmuLed`：

```bash
python3 /agibot/software/aimdk/src/py_examples/py_examples/play_lights.py \
  1 0 0 255 9 false
ros2 topic echo --once /aima/hal/pmu/led_state \
  --qos-reliability best_effort --qos-durability transient_local
```

模式 `0/1/2/3` 分别表示常亮、呼吸、闪烁、流水。优先级较低时会返回优先级不足；恢复灯光时使用更高优先级，必要时设置 `reset_priority=true`。

### 音频

文件播放示例：

```bash
python3 /agibot/software/aimdk/src/py_examples/py_examples/play_audio.py <audio-file>
```

PCM 流播放示例：

```bash
python3 /agibot/software/aimdk/src/py_examples/py_examples/play_audio_stream.py \
  --ros-args -p raw_audio_path:=<raw-pcm-file>
```

流播放使用 `/aima/hal/audio/playback`，先调用 `RequestAudioFocus`，结束后调用 `AbandonAudioFocus`。状态可通过 `/aima/hal/audio/play_state` 和 `GetAudioStatus` 查询。

## 关节运动方案

四组命令话题为：

```text
/aima/hal/joint/head/command
/aima/hal/joint/waist/command
/aima/hal/joint/arm/command
/aima/hal/joint/leg/command
```

消息类型为 `aimdk_msgs/msg/JointCommandArray`。每项填写实际关节名以及 `position`、`velocity`、`effort`、`stiffness`、`damping`。

按官方 Demo 的顺序执行：

1. 初始化 ROS2 节点，等待四组状态首帧。
2. 读取当前位置和关节名顺序。
3. 选择单组、单关节，发送小幅度、低速度的位置目标。
4. 读取状态确认目标位置和错误码。
5. 发送保持当前位置的安全命令，销毁节点。

关节命令没有自动超时保护，退出路径必须显式发送安全值。命令 QoS 使用 `BEST_EFFORT + TRANSIENT_LOCAL`。

## 适配器接口

公开头文件：[agibot_x2_adapter.hpp](../../include/bridge/adapters/agibot_x2/agibot_x2_adapter.hpp)。适配器统一提供：

- `initialize()`：建立 ROS2 会话并订阅四组状态。
- `read_state()`：返回 31 个本体关节的位置、速度和力矩。
- `write_command()`：按四组发布 `JointCommandArray`。
- `query_health()`：返回连接和生命周期状态。
- `shutdown()`：停止 ROS2 executor，并按配置发送安全值。

相机、雷达、IMU、音频、灯光、SLAM 和导航使用 X2 原生 ROS2 接口；接口名称和消息类型以本手册的实机话题为准。

## 排错

| 情况 | 检查 |
|---|---|
| `aimdk_msgs` 找不到 | 先 source `/opt/ros/humble/setup.bash` 和 `/agibot/software/common/share/aimdk_msgs/local_setup.bash` |
| 收不到关节状态 | 检查网络、四组 topic、QoS 和发布者数量 |
| 灯光优先级不足 | 使用不低于当前占用的 priority，必要时 reset priority |
| 音频无法播放 | 检查音频文件路径、格式和音频焦点；PCM 流确认采样率与采样格式 |
| SLAM 话题无样本 | 检查 SLAM 进程、当前地图和对应话题发布者 |
