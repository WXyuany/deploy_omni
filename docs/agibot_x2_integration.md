# Agibot X2 接入说明

## 已确认的数据面

X2（灵犀 X2）使用 ROS 2 `aimdk_msgs` 消息，不使用 Unitree `rt/lowcmd` / `rt/lowstate`。
官方关节话题按组发布：

| 组 | 状态 | 命令 | 数量 |
| --- | --- | --- | ---: |
| head | `/aima/hal/joint/head/state` | `/aima/hal/joint/head/command` | 2 |
| waist | `/aima/hal/joint/waist/state` | `/aima/hal/joint/waist/command` | 3 |
| arm | `/aima/hal/joint/arm/state` | `/aima/hal/joint/arm/command` | 14 |
| leg | `/aima/hal/joint/leg/state` | `/aima/hal/joint/leg/command` | 12 |

关节状态类型为 `aimdk_msgs/msg/JointStateArray`，命令类型为 `aimdk_msgs/msg/JointCommandArray`。状态里的关节名使用机器人实际名称，例如 `head_yaw_joint`、`left_knee_joint`；字段包含 `name`、`position`、`velocity`、`effort`、`error_code`，命令额外包含 `stiffness`、`damping`。实机观察到状态 QoS 为 `BEST_EFFORT + TRANSIENT_LOCAL`，命令 QoS 为 `BEST_EFFORT + TRANSIENT_LOCAL`。

四组本体关节共 31 个；手部是独立的 `hand` 话题组，不计入当前本体适配器的 31 关节布局。

## 当前代码边界

仓库已注册 `agibot_x2` 构建选择、31 关节样例、适配器能力声明和只读 ROS2 探测器。由于当前开发机没有 X2 的 `rclcpp`/`aimdk_msgs` 消息包，适配器在离线构建中会明确返回“ROS2 backend not compiled”，不会伪造连接或下发运动命令。

可执行的 `agibot_x2_state_probe` 会初始化四组订阅，等待四组状态首帧，并打印收到的 31 关节数量；它不发布普通运动命令，退出时仅发送全零安全消息（ROS2 backend 可用时）。

在 X2 开发计算单元上 source `/agibot/software/common/share/aimdk_msgs/local_setup.bash` 和 ROS 2 Humble 环境后，用 `aimdk_msgs` 生成的 C++ 类型编译四组 typed publisher/subscriber；不要把 X2 消息强行复用 Unitree DDS codec。

## 上机顺序

```bash
source /opt/ros/humble/setup.bash
source /agibot/software/common/share/aimdk_msgs/local_setup.bash
python3 scripts/x2_ros2_probe.py --json
ros2 topic echo /aima/hal/joint/leg/state --qos-durability transient_local --qos-reliability best_effort
```

探测器只读 topic/service 名称，不发布消息、不调用服务。确认四组状态均有发布者并读取到首帧后，才进行低速控制测试。官方文档还特别说明底层关节命令没有自动超时保护，退出时必须显式发送安全值；因此 X2 adapter 的命令 backend 完成前不能把 `adapter_bridge` 当作真机控制程序运行。

## 灯光测试

官方示例：`/agibot/software/aimdk/src/py_examples/py_examples/play_lights.py`。服务类型为 `aimdk_msgs/srv/SetPmuLed`，状态话题为 `/aima/hal/pmu/led_state`。

```bash
source /opt/ros/humble/setup.bash
source /agibot/software/common/share/aimdk_msgs/local_setup.bash
python3 /agibot/software/aimdk/src/py_examples/py_examples/play_lights.py 1 0 0 255 9 false
ros2 topic echo --once /aima/hal/pmu/led_state \
  --qos-reliability best_effort --qos-durability transient_local
python3 /agibot/software/aimdk/src/py_examples/py_examples/play_lights.py 0 255 0 0 10 true
```

模式 `0/1/2/3` 分别表示常亮、呼吸、闪烁、流水。优先级较低时服务会返回优先级不足；恢复灯光时使用不低于当前值的优先级，必要时将 `reset_priority` 设为 `true`。

## 音频测试

文件播放使用 `/aimdk_5Fmsgs/srv/PlayAudioFile`，官方示例为 `/agibot/software/aimdk/src/py_examples/py_examples/play_audio.py`：

```bash
python3 /agibot/software/aimdk/src/py_examples/py_examples/play_audio.py <audio-file>
```

PCM 流播放使用 `/aima/hal/audio/playback` 和 `aimdk_msgs/msg/AudioPlayback`，官方示例为 `play_audio_stream.py`。示例会先调用 `RequestAudioFocus`，发布带 `AudioInfo`/`AudioData` 的 PCM 数据，结束时调用 `AbandonAudioFocus`。播放状态可由 `/aima/hal/audio/play_state` 和 `GetAudioStatus` 查询。

## 关节运动测试方案

四组命令话题分别为 `/aima/hal/joint/head/command`、`waist/command`、`arm/command`、`leg/command`，消息类型为 `aimdk_msgs/msg/JointCommandArray`。按官方 Demo 的执行顺序：等待状态首帧，读取当前位置，使用小幅度低速位置目标测试单组关节，读取状态确认，再发送保持当前位置的安全命令。每个命令项填写实际关节名以及 `position`、`velocity`、`effort`、`stiffness`、`damping`；退出前显式发送安全值。
