# Adapter Any Robot 使用手册

这份手册只描述当前仓库里**实际生效**的构建与启动方式。

- 根程序入口：`adapter_bridge`
- 根构建脚本：`scripts/build_robot.sh`
- 根配置文件示例：
  - `config/kepler_k2.sample.yaml`
  - `config/pm01.sample.yaml`
  - `config/g1.sample.yaml`
  - `config/cvte_carm_a3.sample.yaml`
  - `config/cvte_carm_d3.sample.yaml`
  - `config/agibot_x2.sample.yaml`
- G1 SDK 接口指南：`docs/unitree_g1_sdk_usage.md`
- 统一适配器架构：`docs/adapter_architecture.md`

> 注意：仓库里有一些历史文档和示例代码仍然提到 `Fast DDS`、`kepler_k2_bridge` 或 standalone `pm01_bridge`。这些内容可以作为背景参考，但**当前仓库主入口仍然是 `adapter_bridge`**。

---

## 1. 支持的机器人

当前主程序 `adapter_bridge` 支持：

- `kepler_k2`
- `pm01`
- `unitree_g1`
- `cvte_carm_a3`
- `cvte_carm_d3`
- `agibot_x2`

机器人注册与装配逻辑在：

- `src/bridge/app/robot_assembly_factory.cpp`

统一 adapter 的边界、capability 规则和新机器人接入流程见：

- `docs/adapter_architecture.md`
- `docs/cvte_carm_integration.md`

---

## 2. 通用依赖

`adapter_bridge` 的公共运行时依赖下面这些库；选择单个机器人只会编译该机器人适配器，但公共 DDS runtime 仍由主程序统一使用：

### 2.1 Unitree SDK2

需要存在：

- `library/unitree_sdk2/lib/<arch>/libunitree_sdk2.a`

### 2.2 CycloneDDS

需要存在：

- `library/unitree_sdk2/thirdparty/lib/<arch>/libddsc.so`
- `library/unitree_sdk2/thirdparty/lib/<arch>/libddscxx.so`

### 2.3 C++17 和 Threads

根工程使用：

- C++17
- `Threads`

这些依赖由根 `CMakeLists.txt` 统一检查和链接。

---

## 3. 不同机器人所需依赖

## 3.1 Kepler K2

### 编译依赖

Kepler K2 在当前根工程里**没有额外的 ROS2 编译依赖**，只依赖上面的通用依赖：

- Unitree SDK2
- CycloneDDS
- Threads

### 运行说明

当前仓库里的 K2 主链路可以构建和跑通 `adapter_bridge`，但**真实 `motor_api` 后端并没有在当前根工程中编译接通**。

这意味着：

- 当前 K2 路径可以用于主程序联调、配置解析、runner 闭环和占位链路验证
- 如果你想直接上真机，需要额外确认 real backend 是否已接回当前根工程

---

## 3.2 PM01

### 编译依赖

PM01 分两种情况：

#### 情况 A：只编译 PM01 adapter 占位 / 非 ROS2 路径

只需要通用依赖：

- Unitree SDK2
- CycloneDDS
- Threads

#### 情况 B：走 PM01 真机 ROS2 backend

除了通用依赖，还需要：

- `rclcpp`
- `interface_protocol`

只有当这两个包都被 `find_package(...)` 找到时，PM01 的 ROS2 real backend 才会被编进来。

### 运行依赖（ROS2 backend）

如果你要让 `config/pm01.sample.yaml` 真正走 PM01 ROS2 backend，还需要：

- 系统中有可用 ROS2 环境
- `interface_protocol` 消息包可被找到
- 运行时存在这些 ROS2 topic：
  - `/hardware/joint_state`
  - `/hardware/imu_info`
  - `/motion/motion_state`
  - `/hardware/joint_command`

### 当前 sample 的意图

当前 `config/pm01.sample.yaml` 已显式写成：

- `transport: ros2`
- `interface_hint: pm01`

因此它的目标就是生成：

- `transport_endpoint = "ros2:pm01"`

从而命中 PM01 的 ROS2 backend 选择条件。

## 3.3 CVTE CArm

CVTE CArm adapter 使用独立的 `arm_control_sdk` 网络 SDK，不依赖 Unitree 控制协议。当前已接入：

- `cvte_carm_a3`：单臂 `CArmSingleCol`
- `cvte_carm_d3`：双臂 `CArmDualBot`
- `cvte_carm_d3_left` / `cvte_carm_d3_right`：D3 单侧视图

默认的 CArm 选择是 SDK-only 构建，只生成 `cvte_carm_sdk_smoke`，不检查 Unitree SDK 和 CycloneDDS。若现场已经准备好公共 DDS 依赖并需要 `adapter_bridge`，再显式打开 `-DBUILD_BRIDGE_DDS=ON`。

编译时需要将 SDK 解压到工作区，或通过 `BRIDGE_CVTE_SDK_DIR` 指定目录：

```bash
./scripts/build_robot.sh cvte_carm
```

脚本默认只构建只读烟测 `cvte_carm_sdk_smoke`，不会拉入其他机器人依赖。也可以手动指定 SDK：

```bash
BRIDGE_CVTE_SDK_DIR=/opt/cvte/arm_control_sdk \
  ./scripts/build_robot.sh cvte_carm build-cvte
```

SDK-only 烟测不需要配置文件，直接传入型号和控制器 IP：

```bash
./build-cvte-only/cvte_carm_sdk_smoke --model a3 --ip <A3控制器IP>
./build-cvte-only/cvte_carm_sdk_smoke --model d3 --ip <D3控制器IP>
```

若要运行 `adapter_bridge`，需要重新配置 CMake 并显式设置 `-DBUILD_BRIDGE_DDS=ON`；这时再把配置中的 `interface_hint` 改成控制器 IP。

CArm 当前已接入关节状态、位置跟随和显式安全控制；速度、力矩、刚度和阻尼通用命令会明确返回 `kUnsupported`。笛卡尔、末端、运动学和示教等 SDK 接口仍通过 CArm adapter 的扩展入口逐项增加，不把厂商 SDK 类型泄漏到核心层。完整 SDK 版本、网络、型号和现场安全顺序见 `docs/cvte_carm_integration.md`。

## 3.4 Agibot X2

X2 使用 ROS2 `aimdk_msgs` 消息，按 `head`、`waist`、`arm`、`leg` 四组提供 31 个本体关节，话题前缀为 `/aima/hal/joint/<group>/{state,command}`。它不复用 Unitree DDS 低层话题。机器人端的 ROS2 环境和 `aimdk_msgs` 包位于 `/opt/ros/humble` 与 `/agibot/software/common`。

```bash
./scripts/build_robot.sh agibot_x2
source /path/to/x2/ros2/setup.bash
python3 scripts/x2_ros2_probe.py --json
```

探测器不会发布命令。X2 底层关节命令没有自动超时保护，未完成状态首帧、QoS 和安全值下发验证前，不得把 `adapter_bridge` 用于带动力控制。

### 3.4.1 灯光测试方案

灯光服务为 `/aimdk_5Fmsgs/srv/SetPmuLed`，类型为 `aimdk_msgs/srv/SetPmuLed`。参数如下：`led_strip_mode=0` 常亮、`1` 呼吸、`2` 闪烁、`3` 流水；颜色由 `r/g/b` 指定，优先级由 `priority` 指定。官方 Python 示例位于机器人端 `/agibot/software/aimdk/src/py_examples/py_examples/play_lights.py`。

```bash
source /opt/ros/humble/setup.bash
source /agibot/software/common/share/aimdk_msgs/local_setup.bash

# 呼吸蓝灯，优先级 9
python3 /agibot/software/aimdk/src/py_examples/py_examples/play_lights.py \
  1 0 0 255 9 false

# 读取灯带状态
ros2 topic echo --once /aima/hal/pmu/led_state \
  --qos-reliability best_effort --qos-durability transient_local

# 恢复红色常亮；reset_priority=true 用于清理较高优先级占用
python3 /agibot/software/aimdk/src/py_examples/py_examples/play_lights.py \
  0 255 0 0 10 true
```

灯光状态话题类型为 `aimdk_msgs/msg/PmuLedState`，使用 `BEST_EFFORT + TRANSIENT_LOCAL` QoS。恢复状态时优先级必须不低于当前占用，必要时使用 `reset_priority=true`。

### 3.4.2 音频测试方案

音频文件服务为 `/aimdk_5Fmsgs/srv/PlayAudioFile`，类型为 `aimdk_msgs/srv/PlayAudioFile`；原始 PCM 播放话题为 `/aima/hal/audio/playback`，类型为 `aimdk_msgs/msg/AudioPlayback`。官方示例位于机器人端：

- `/agibot/software/aimdk/src/py_examples/py_examples/play_audio.py`
- `/agibot/software/aimdk/src/py_examples/py_examples/play_audio_stream.py`

文件播放示例：

```bash
source /opt/ros/humble/setup.bash
source /agibot/software/common/share/aimdk_msgs/local_setup.bash
python3 /agibot/software/aimdk/src/py_examples/py_examples/play_audio.py <audio-file>
```

原始流播放必须先申请音频焦点，发布 `AudioPlayback`，结束后释放焦点。官方示例默认使用 16 kHz、S16LE、单声道 PCM，并通过 `RequestAudioFocus` / `AbandonAudioFocus` 管理播放占用：

```bash
python3 /agibot/software/aimdk/src/py_examples/py_examples/play_audio_stream.py \
  --ros-args -p raw_audio_path:=<raw-pcm-file>
```

播放状态可通过以下接口确认：

```bash
ros2 topic echo --once /aima/hal/audio/play_state
ros2 service call /aimdk_5Fmsgs/srv/GetAudioStatus \
  aimdk_msgs/srv/GetAudioStatus \
  "{header: {header: {stamp: {sec: 0, nanosec: 0}}}, text: '', trace_id: 'x2_audio_check'}"
```

### 3.4.3 关节运动测试方案

关节命令按四组发送到：

- `/aima/hal/joint/head/command`
- `/aima/hal/joint/waist/command`
- `/aima/hal/joint/arm/command`
- `/aima/hal/joint/leg/command`

消息类型为 `aimdk_msgs/msg/JointCommandArray`。每个 `JointCommand` 应填写实际关节名、目标 `position`，并按官方 Demo 的控制模式设置 `velocity`、`effort`、`stiffness`、`damping`。正式动作前先读取四组状态并确认关节名顺序：

```bash
source /opt/ros/humble/setup.bash
source /agibot/software/common/share/aimdk_msgs/local_setup.bash
ros2 service call /aimdk_5Fmsgs/srv/GetAllJointState \
  aimdk_msgs/srv/GetAllJointState '{}'
```

推荐按照官方控制 Demo 的顺序执行：初始化 ROS2 节点 → 等待四组状态首帧 → 读取当前位置 → 以小幅度、低速度位置目标发送单组命令 → 读取状态确认到位 → 发送保持当前位置的安全命令 → 销毁节点。关节命令没有自动超时保护，Demo 的退出路径必须显式发送安全值；命令 QoS 使用 `BEST_EFFORT + TRANSIENT_LOCAL`。

---

## 4. 编译方法

## 4.1 推荐方式：使用脚本

根工程提供的统一脚本：

```bash
./scripts/build_robot.sh <pm01|kepler_k2|unitree_g1|cvte_carm|all> [build-dir]
```

### 只编译 PM01

```bash
./scripts/build_robot.sh pm01
```

默认输出目录：

- `build-pm01-only/`

### 只编译 Kepler K2

```bash
./scripts/build_robot.sh kepler_k2
```

或：

```bash
./scripts/build_robot.sh k2
```

默认输出目录：

- `build-k2-only/`

### 编译默认适配器集合

```bash
./scripts/build_robot.sh all
```

默认输出目录：

- `build/`

`all` 包含 K2、PM01 和 Unitree G1。CArm 使用独立的本地二进制 SDK，不包含在 `all` 中，应单独执行 `./scripts/build_robot.sh cvte_carm`。

### 只编译 Unitree G1

```bash
./scripts/build_robot.sh unitree_g1
```

或：

```bash
./scripts/build_robot.sh g1
```

默认输出目录：

- `build-g1-only/`

### 只编译 CVTE CArm

```bash
./scripts/build_robot.sh cvte_carm
```

默认输出目录：

- `build-cvte-only/`

该入口同时生成 `cvte_carm_sdk_smoke`，用于连接、读取状态、健康检查和断开，不会自动使能伺服或发送运动命令。

除 CArm 入口会额外构建 `cvte_carm_sdk_smoke` 外，脚本的主构建目标是 `adapter_bridge`。

---

## 4.2 手动使用 CMake

### 编译默认适配器集合

```bash
cmake -S . -B build \
  -DBRIDGE_ROBOT=all

cmake --build build --target adapter_bridge
```

### 只编译 PM01

```bash
cmake -S . -B build-pm01-only \
  -DBRIDGE_ROBOT=pm01

cmake --build build-pm01-only --target adapter_bridge
```

### 只编译 Kepler K2

```bash
cmake -S . -B build-k2-only \
  -DBRIDGE_ROBOT=kepler_k2

cmake --build build-k2-only --target adapter_bridge
```

### 只编译 CVTE CArm

```bash
cmake -S . -B build-cvte-only \
  -DBRIDGE_ROBOT=cvte_carm \
  -DBUILD_BRIDGE_DDS=OFF \
  -DBUILD_TESTING=OFF \
  -DBUILD_GAMEPAD_SDL=OFF \
  -DBRIDGE_CVTE_SDK_DIR="$PWD/arm_control_sdk"

cmake --build build-cvte-only --target cvte_carm_sdk_smoke
```

---

## 5. 启动方法

根程序入口在：

- `src/apps/adapter_bridge_main.cpp`

可执行文件名：

- `adapter_bridge`

## 5.1 默认启动

如果不传配置路径，程序默认加载：

- `config/kepler_k2.sample.yaml`

例如：

```bash
./build/adapter_bridge
```

---

## 5.2 使用 Kepler K2 配置启动

```bash
./build/adapter_bridge config/kepler_k2.sample.yaml
```

如果你是单独编译 K2：

```bash
./build-k2-only/adapter_bridge config/kepler_k2.sample.yaml
```

---

## 5.3 使用 PM01 配置启动

```bash
./build/adapter_bridge config/pm01.sample.yaml
```

如果你是单独编译 PM01：

```bash
./build-pm01-only/adapter_bridge config/pm01.sample.yaml
```

---

## 6. PM01 启动前检查

如果你要让 PM01 真正走 ROS2 backend，建议启动前确认：

### 6.1 编译时已启用 real backend

也就是 CMake 找到了：

- `rclcpp`
- `interface_protocol`

否则即使配置写成 ROS2，也会在初始化时报错：

- `pm01 ROS2 backend requested but not compiled in`

### 6.2 配置正确

至少要保证：

- `robot_name: pm01`
- `transport: ros2`
- `interface_hint: pm01`

### 6.3 ROS2 topic 存在

需要以下 topic：

- 输入：
  - `/hardware/joint_state`
  - `/hardware/imu_info`
  - `/motion/motion_state`
- 输出：
  - `/hardware/joint_command`

如果缺少这些 topic，PM01 虽然可能初始化成功，但不会达到和 old example 一样的真机效果。

---

## 7. 建议的典型流程

## 7.1 Kepler K2

```bash
./scripts/build_robot.sh kepler_k2
./build-k2-only/adapter_bridge config/kepler_k2.sample.yaml
```

## 7.2 PM01

```bash
./scripts/build_robot.sh pm01
./build-pm01-only/adapter_bridge config/pm01.sample.yaml
```

如果 PM01 需要真机 ROS2 backend，请先确保 ROS2 和 `interface_protocol` 已经在编译环境中被正确发现。

---

## 8. 常见问题

### 8.1 运行 PM01 时提示 `pm01 ROS2 backend requested but not compiled in`

原因：

- 构建时没有找到 `rclcpp`
- 或没有找到 `interface_protocol`

处理方式：

- 先准备好 ROS2 环境
- 确保 `interface_protocol` 可以被 CMake 找到
- 然后重新构建 PM01

### 8.2 K2 可以编译，但不代表已经接通真机 backend

当前根工程里的 Kepler K2 真实 `motor_api` 路径还不是完整成品。文档里如果出现旧的 `kepler_k2_bridge` 或历史 K2 bridge 示例，请把它们当成参考，不要当成当前主入口。

### 8.3 旧文档为什么和现在不一样

仓库里还有一些历史文档或示例：

- 部分内容仍然写 Fast DDS
- 部分内容仍然写 `kepler_k2_bridge`
- `docs/pm01_example/` 是 standalone PM01 示例，不是根工程主程序名

对于当前仓库主入口，优先以这些文件为准：

- `scripts/build_robot.sh`
- `CMakeLists.txt`
- `src/apps/adapter_bridge_main.cpp`
- `config/kepler_k2.sample.yaml`
- `config/pm01.sample.yaml`

---

## 9. 相关文件

- 根构建脚本：`scripts/build_robot.sh`
- 根构建配置：`CMakeLists.txt`
- 根程序入口：`src/apps/adapter_bridge_main.cpp`
- K2 配置示例：`config/kepler_k2.sample.yaml`
- PM01 配置示例：`config/pm01.sample.yaml`
- 机器人装配工厂：`src/bridge/app/robot_assembly_factory.cpp`
- 配置解析：`src/bridge/config/bridge_config.cpp`
