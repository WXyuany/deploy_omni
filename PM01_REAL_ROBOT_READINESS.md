# PM01 真机就绪审查与部署指南

> 审查日期：2026-07-13  
> 审查对象：当前工作区（包含未提交修改）  
> 真机基线：`/home/xsc/pm01_0528` 当前可用代码  
> 目标机器人：众擎 EngineAI PM01-EDU（24 关节）

## 1. 结论

**当前结论：NO-GO，不满足直接上真机闭环控制的条件。**

项目已经具备 PM01 ROS 2 backend、24 关节映射、DDS 命令输入、ROS 2
`/hardware/joint_command` 输出和状态回传的基本链路，并能在 Ubuntu 22.04、ROS 2
Humble、x86_64 环境编译出真实 PM01 backend。但是，在任何带动力测试前，至少必须解决
第 2 节的 P0 阻断项，并重新完成测试和无动力联调。

当前允许做的工作：

- 在开发机或机器人计算机上编译。
- 加载并校验配置。
- 机器人急停保持按下时检查 ROS 2 发现、Topic 类型和状态订阅。
- 使用录制数据或隔离网络验证 DDS 输入和状态转发。

当前不允许做的工作：

- 直接发送带非零 `kp`、`kd`、速度或位置变化的闭环命令。
- 把 `config/pm01.sample.yaml` 原样作为生产配置。
- 依赖 `Ctrl+C` 完成安全停机。
- 在没有机械支撑、急停人员和关节限位核验时使能关节。

### 与可用真机代码的对照摘要

`~/pm01_0528` 中有两条不同路径：

1. `src/pm01_bridge` 是已经使用的 ROS 2 ↔ Unitree DDS 桥，与本项目 PM01 adapter 的职责
   基本相同。
2. `engineai_robotics_native_sdk/.../dds_joint_bridge_runner` 是运控内部 Native SDK runner，
   它额外提供 500 ms 命令 watchdog 和纯阻尼回退，不能和根目录 ROS bridge 的安全能力
   混为一谈。

已经确认一致的部分：

- `JointCommand.msg`、`JointState.msg`、`ImuInfo.msg`、`MotionState.msg` 字段一致。
- ROS 2 Topic 一致：`/hardware/joint_state`、`/hardware/imu_info`、
  `/motion/motion_state`、`/hardware/joint_command`。
- JointState、IMU 和 JointCommand 均使用 depth 3、best-effort、volatile QoS；MotionState
  使用 depth 1。
- 两边都把 DDS `q/dq/kp/kd` 写入 ROS `position/velocity/stiffness/damping`，并把
  `feed_forward_torque` 与 `torque` 置零，`parallel_parser_type` 使用 0。
- 两边的 DDS motor slot 都是 35 个，PM01-EDU 使用 `0..23` 共 24 个槽位。

已经确认不一致的部分：

| 项目 | 当前项目 | 可用真机基线 | 影响 |
|---|---|---|---|
| DDS command Topic | `bridge/rt/lowcmd` | `rt/lowcmd` | 无法收到现有真机控制端命令 |
| DDS state Topic | `bridge/rt/lowstate` | `rt/lowstate` | 现有真机控制端无法收到状态 |
| DDS domain | `0` | `0` | 一致 |
| ROS domain | 运行环境决定 | arm 脚本明确 `69` | 当前运行时必须导出 `ROS_DOMAIN_ID=69` |
| 状态目标频率 | event 模式，配置值 `250` | `800 Hz` | event 路径随状态到达，但配置和诊断基准不一致 |
| motion gate | `required: false` | `require_motion_state: true` | 当前会绕过 `joint_bridge` 门禁 |
| 命令超时 | `120 ms`，fail-stop 写入拦截未实现 | Native runner `500 ms` 后纯阻尼 | 当前不具备基线 Native runner 的安全回退 |
| 24 关节语义 | 含 waist pitch/roll、wrist，无 head | 单 waist yaw、双 elbow yaw、head yaw | 限位、诊断和未来映射会绑定错误关节 |

## 2. 真机阻断项

### P0-1：DDS Topic 与可用真机基线不一致

当前示例使用 `bridge/rt/lowcmd` 和 `bridge/rt/lowstate`，而 `~/pm01_0528` 的已用配置和
代码均使用 `rt/lowcmd`、`rt/lowstate`：

- `config/pm01.sample.yaml:13-19`
- `/home/xsc/pm01_0528/src/pm01_bridge/config/pm01_bridge_params.yaml:4-6`
- `/home/xsc/pm01_0528/src/pm01_bridge/src/pm01_bridge.cpp:231-245`

**上机前要求：** 若继续兼容现有真机控制端，必须改成 `rt/lowcmd` 与 `rt/lowstate`；如要
保留 `bridge/` 前缀，则命令发布端和状态订阅端必须同步修改并重新联调。

### P0-2：24 关节模型与可用真机基线不一致

真机基线的 24 关节顺序是：左右腿各 6、腰 yaw 1、左右臂各 5、头 yaw 1。当前配置却把
slot 0/2 的 hip yaw/pitch 名称互换，并包含不存在于该基线布局中的 waist pitch、
waist roll、left wrist yaw，同时缺少双侧 elbow yaw 和 head yaw：

- `config/pm01.sample.yaml:73-467`
- `/home/xsc/pm01_0528/src/pm01_bridge/src/pm01_bridge.cpp:89-121`
- `/home/xsc/pm01_0528/engineai_robotics_native_sdk/src/runner/dds_joint_bridge/include/dds_joint_bridge/dds_joint_bridge_runner.h:31-44`

当前所有 `sim_index` 与 `hardware_index` 恰好都是恒等映射，因此数组数值暂时仍按 slot
原样通过；但关节名称、限位和诊断语义是错误的。一旦填写真实逐关节限位或引入非恒等
mapping，就会作用到错误关节。

**上机前要求：** 按第 8 节的真机基线顺序重写完整 24 关节映射，并通过只读 JointState
逐槽确认。

### P0-3：关节越界只告警，仍会下发

`JointMap` 在值超出 `sim_limits` 或 `hardware_limits` 时只生成 warning，随后仍把原值写入
目标向量：

- `src/bridge/config/joint_map.cpp:396-418`
- `src/bridge/dds/dds_translation.cpp:91-98`
- `src/bridge/dds/dds_translation.cpp:390-402`

当前 PM01 示例对 24 个关节统一使用 `[-1.0, 1.0]` 的位置范围，并且没有给速度、力矩、
刚度和阻尼配置真实硬件限值。该范围明显是占位配置，不能替代 PM01 每个关节的厂商限位。

**上机前要求：** 使用 PM01 对应版本的关节限位；越界必须拒绝或饱和，且需要覆盖
position、velocity、effort/torque、stiffness 和 damping。

### P0-4：配置宣称故障后停止写入，但实际没有执行该策略

配置要求 `halt_command_writes_while_faulted: true`，但解析结果没有映射到 runner 的任何
策略字段。DDS 回调收到新命令后直接调用 `submit_latest_command()`，该函数也没有检查
`fail_stop_active`，所以 fail-stop 激活后仍可能继续调用 adapter 的 `write_command()`：

- `include/bridge/core/bridge_runner.hpp:45-50`
- `src/bridge/config/bridge_config.cpp:1687-1691`
- `src/bridge/dds/dds_command_ingress.cpp:96-124`
- `src/bridge/core/bridge_runner.cpp:416-475`
- `src/bridge/adapters/pm01/pm01_adapter.cpp:535-558`

此外，当前 runner 没有在命令陈旧、DDS 失联或进程退出时发送明确的零力矩/阻尼保持/
厂商安全模式命令。**上机前要求：** 真正实现并测试故障后的写入拦截；同时和众擎确认
`/hardware/joint_command` 的 watchdog 语义，并实现经过确认的安全停机策略。不能自行
假设“停止发布”等价于“电机卸载”。

### P0-5：主程序没有 SIGINT/SIGTERM 优雅停机路径

`adapter_bridge` 的主循环为无限循环，`runner.request_stop()` 和 `runner.shutdown()` 只有在
`run_bridge_loop()` 返回错误后才执行；主程序没有注册 SIGINT/SIGTERM：

- `src/apps/adapter_bridge_main.cpp:61-109`
- `src/apps/adapter_bridge_main.cpp:142-158`

因此正常 `Ctrl+C` 或 systemd 停止不能保证执行 adapter shutdown。

**上机前要求：** 增加信号处理和可中断的 event wait，使退出流程始终执行
`request_stop()` 与 `shutdown()`，并验证多次启动/停止。

### P0-6：PM01 ROS executor shutdown 可复现阻塞

真实 PM01 backend 编译开启后，`pm01_adapter_test` 在 shutdown 测试中超时。系统调用跟踪
显示主线程阻塞在等待 ROS executor 线程退出；`ros_executor_->cancel()` 没有使 join 正常
完成。相关实现：

- `src/bridge/adapters/pm01/pm01_adapter.cpp:170-177`
- `src/bridge/adapters/pm01/pm01_adapter.cpp:590-616`
- `tests/adapters/pm01_adapter_test.cpp:98-116`

**上机前要求：** 修复并通过真实 backend 下的重复 initialize/shutdown 和析构测试。

### P0-7：示例关闭了运动状态门禁

`config/pm01.sample.yaml` 当前配置为：

```yaml
motion_state:
  required: false
  required_task: joint_bridge
```

这会绕过 `/motion/motion_state` 的 `joint_bridge` 检查。代码只有在 `required: true` 时才阻止
命令下发：`src/bridge/adapters/pm01/pm01_adapter.cpp:508-533`。

**上机前要求：** 设置 `required: true`。厂商公开流程也要求先进入 `pd-stand` 和
`joint_bridge`，再启动实机控制。

## 3. 其他高风险问题

1. `read_initial_state_timeout_ms` 当前只被转换成 adapter 的内部时间增量，没有形成真实的
   墙钟超时。event 模式会一次等待状态最长 24 小时：
   `src/apps/adapter_bridge_main.cpp:96-99`。
2. PM01 ROS 2 Topic 在代码中硬编码，配置中的 adapter Topic 不控制这些名称：
   `/hardware/joint_state`、`/hardware/imu_info`、`/hardware/power_info`、
   `/hardware/alert`、`/motion/motion_state` 和 `/hardware/joint_command`，见
   `src/bridge/adapters/pm01/pm01_adapter.cpp:371-412`。
3. PM01 状态仅接受三个数组都恰好为 24 的 `JointState`。不匹配的消息会被静默丢弃，
   见 `src/bridge/adapters/pm01/pm01_adapter.cpp:619-623`。
4. 外部 DDS 初始化绑定指定网卡失败后，会静默清空网卡并退回默认接口，见
   `src/bridge/dds/dds_runtime.cpp:46-59`。真机应 fail closed，避免命令流量出现在错误网卡。
5. `build_robot.sh pm01` 构建后错误地提示运行 `config/g1.sample.yaml`，见
   `scripts/build_robot.sh:64-65`。PM01 必须显式传入 PM01 配置。
6. DDS LowCmd 的未知 `mode_machine` 校验被注释，runner 的提前校验也被注释；PM01 adapter
   最终会在 `write_command()` 拒绝 unknown mode，但此时命令已进入 runner buffer，并会触发
   adapter failure/fail-stop。见 `src/bridge/dds/dds_translation.cpp:132-160`、
   `src/bridge/core/bridge_runner.cpp:416-458` 和
   `src/bridge/adapters/pm01/pm01_adapter.cpp:270-287`。
7. 每帧命令都会向 stderr 打印，250 Hz 下可能产生大量日志并干扰实时性，见
   `src/bridge/adapters/pm01/pm01_adapter.cpp:500-506,553-557`。

## 4. 已验证结果

本次在以下环境执行：

- Ubuntu 22.04
- x86_64
- GCC 11.4
- CMake 3.22.1
- ROS 2 Humble

PM01 专用构建成功：

```bash
source /opt/ros/humble/setup.bash
./scripts/build_robot.sh pm01 build-pm01-readiness
```

生成的 `build-pm01-readiness/generated/bridge/build_config.hpp` 确认：

```text
BRIDGE_BUILD_PM01=1
BRIDGE_PM01_REAL_BACKEND_AVAILABLE=1
```

完整测试结果：14/16 通过。

```text
g1_bridge_config_test  FAILED：样例 loop_rate_hz=1000，测试仍期望 800
pm01_adapter_test      TIMEOUT：ROS executor shutdown 阻塞
```

PM01 相关的配置、关节映射、DDS 翻译、fake transport 和 assembly 测试均通过；这些单元测试
不能替代真机通信和制动验证。

本次 CMake 配置输出显示 ROS 2 默认使用 `rmw_fastrtps_cpp`；本机没有安装
`rmw_cyclonedds_cpp`，因此本次没有完成厂商推荐 Cyclone DDS RMW 下的运行验证。

## 5. 机器人端环境准备

厂商公开的 ROS 2 工作区要求开发主机使用 Ubuntu 22.04、ROS 2 Humble、GCC >= 11、
CMake >= 3.22、Python >= 3.10，并使用 Cyclone DDS。公开资料没有证明所有 PM01 机内
计算机都具备相同环境，因此先在目标计算机执行：

```bash
uname -m
lsb_release -a
gcc --version
cmake --version
printenv ROS_DISTRO
ip -brief link
ip -brief address
```

安装依赖（Ubuntu 22.04）：

```bash
sudo apt update
sudo apt install -y build-essential cmake git \
  ros-humble-desktop ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosidl-default-generators \
  ros-humble-std-msgs ros-humble-geometry-msgs
```

仓库包含 x86_64 和 aarch64 版本的 Unitree SDK/Cyclone DDS 静态或动态库，但这不等于已经
完成 aarch64 真机验证。`uname -m` 必须是 `x86_64` 或 `aarch64`，其他架构需要补齐依赖。

厂商 ROS 2 工作区公开要求：从机器人背部 Ethernet 接入 `192.168.0.0/24` 内网，不使用
USB 网卡。按机器人实际网段设置静态地址，并先只做只读发现测试。

## 6. 编译

在目标计算机克隆完整仓库并初始化子模块：

```bash
git clone --recurse-submodules <项目仓库地址> adapter_any_robot
cd adapter_any_robot
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=69
./scripts/build_robot.sh pm01 build-pm01-only
```

不要只看可执行文件是否生成，还要确认真实 backend：

```bash
grep BRIDGE_PM01_REAL_BACKEND_AVAILABLE \
  build-pm01-only/generated/bridge/build_config.hpp
ldd build-pm01-only/adapter_bridge | grep -E 'not found|rclcpp|interface_protocol'
```

期望值必须是 `BRIDGE_PM01_REAL_BACKEND_AVAILABLE 1`，且 `ldd` 不得出现 `not found`。

构建并运行测试：

```bash
cmake --build build-pm01-only --parallel
ctest --test-dir build-pm01-only --output-on-failure --timeout 10
```

当前提交预期仍会出现第 4 节的两个失败；在 P0 问题修复前，不应继续带动力测试。

## 7. ROS 2 与网络验证

每个运行终端都设置：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=69
export ROS_LOCALHOST_ONLY=0
```

机器人上电但保持急停，检查 Topic 和类型：

```bash
ros2 topic list
ros2 topic type /hardware/joint_state
ros2 topic type /hardware/joint_command
ros2 topic type /motion/motion_state
ros2 topic hz /hardware/joint_state
ros2 topic echo /motion/motion_state --once
```

必须确认：

- `/hardware/joint_state` 类型与本项目生成的
  `interface_protocol/msg/JointState` 完全一致。
- position、velocity、torque 均为 24 个元素，顺序与配置一致。
- `/motion/motion_state.current_motion_task` 能进入 `joint_bridge`。
- 控制进程和机器人 ROS 2 graph 使用同一 `ROS_DOMAIN_ID` 和兼容 RMW。

注意：配置中的 `dds.domain_id` 控制本项目的外部 Unitree LowCmd/LowState DDS 通道；
`ROS_DOMAIN_ID` 控制 PM01 ROS 2 backend。二者用途不同，必须分别与各自的对端一致。

## 8. 配置文件写法

先复制完整的 24 关节模板：

```bash
cp config/pm01.sample.yaml config/pm01.robot.yaml
```

将文件头部调整为下面的形式；`<ROBOT_ETHERNET_IFACE>` 使用 `ip -brief link` 查到的背部
以太网接口名，例如 `enp3s0`，不要照抄示例名称：

```yaml
schema_version: 1

bridge:
  robot_name: pm01
  # 名义周期与真机基线一致；event 模式实际频率由 /hardware/joint_state 到达率决定。
  loop_rate_hz: 800
  lowstate_trigger: event
  # 对齐 Native runner 的 watchdog 时间；当前项目仍需实现对应的纯阻尼回退。
  stale_command_timeout_ms: 500
  fault_policy:
    mode: fail_stop
    publish_degraded_state_while_faulted: true
    halt_command_writes_while_faulted: true

dds:
  # 外部 LowCmd/LowState DDS domain，不会设置 ROS_DOMAIN_ID。
  # ~/pm01_0528 当前真机基线使用 0；ROS 2 仍单独使用 ROS_DOMAIN_ID=69。
  domain_id: 0
  topics:
    command:
      name: rt/lowcmd
    state:
      name: rt/lowstate
    imu_state:
      name: bridge/rt/secondary_imu
    bms_state:
      name: bridge/rt/lf/bmsstate
    mainboard_state:
      name: bridge/rt/lf/mainboardstate

debug_print:
  state: false
  cmd: false
  forwarding_log: false
  rate_hz: 1

assembly:
  profile: pm01
  adapter:
    type: pm01
  command_ingress:
    enabled: true
    type: dds
  state_egress:
    enabled: true
    type: dds
  modules:
    - name: gamepad
      enabled: false
      type: sdl
      controller_index: 0
      mapping_path: docs/joy_stick/include/hardware/gamecontrollerdb.txt
      poll_interval_ms: 5
      stale_timeout_ms: 100

adapter:
  kind: pm01
  robot_model: pm01
  command_mode: joint_bridge
  motion_state:
    required: true
    required_task: joint_bridge
  connection:
    transport: ros2
    # 该字段实际用于外部 DDS ChannelFactory 绑定网卡。
    interface_hint: <ROBOT_ETHERNET_IFACE>
  startup:
    read_initial_state_timeout_ms: 1000
    require_healthy_hardware: true
```

保留 `config/pm01.sample.yaml` 中完整的 `joint_mapping`，但必须逐项替换：

- `hardware_index`：按当前 PM01 固件的 24 关节顺序确认。
- `hardware_name`：必须和厂商 JointState 的关节顺序一致。
- `transforms.sign/offset`：通过只读状态与机械零位核验。
- `hardware_limits`：填入每个关节的真实位置、速度、力矩、刚度和阻尼范围。
- `sim_limits`：与命令生产端的单位和范围一致。

本次已用真机基线的 slot/ROS 数组顺序如下。建议 `sim_index`、`hardware_index` 都先保持
恒等映射，`sim_name`、`hardware_name` 使用同一语义名称：

| index | 推荐名称 | 真机基线名称 |
|---:|---|---|
| 0 | `l_hip_pitch` | `J00_HIP_PITCH_L` |
| 1 | `l_hip_roll` | `J01_HIP_ROLL_L` |
| 2 | `l_hip_yaw` | `J02_HIP_YAW_L` |
| 3 | `l_knee_pitch` | `J03_KNEE_PITCH_L` |
| 4 | `l_ankle_pitch` | `J04_ANKLE_PITCH_L` |
| 5 | `l_ankle_roll` | `J05_ANKLE_ROLL_L` |
| 6 | `r_hip_pitch` | `J06_HIP_PITCH_R` |
| 7 | `r_hip_roll` | `J07_HIP_ROLL_R` |
| 8 | `r_hip_yaw` | `J08_HIP_YAW_R` |
| 9 | `r_knee_pitch` | `J09_KNEE_PITCH_R` |
| 10 | `r_ankle_pitch` | `J10_ANKLE_PITCH_R` |
| 11 | `r_ankle_roll` | `J11_ANKLE_ROLL_R` |
| 12 | `waist_yaw` | `J12_WAIST_YAW` |
| 13 | `l_shoulder_pitch` | `J13_SHOULDER_PITCH_L` |
| 14 | `l_shoulder_roll` | `J14_SHOULDER_ROLL_L` |
| 15 | `l_shoulder_yaw` | `J15_SHOULDER_YAW_L` |
| 16 | `l_elbow_pitch` | `J16_ELBOW_PITCH_L` |
| 17 | `l_elbow_yaw` | `J17_ELBOW_YAW_L` |
| 18 | `r_shoulder_pitch` | `J18_SHOULDER_PITCH_R` |
| 19 | `r_shoulder_roll` | `J19_SHOULDER_ROLL_R` |
| 20 | `r_shoulder_yaw` | `J20_SHOULDER_YAW_R` |
| 21 | `r_elbow_pitch` | `J21_ELBOW_PITCH_R` |
| 22 | `r_elbow_yaw` | `J22_ELBOW_YAW_R` |
| 23 | `head_yaw` | `J23_HEAD_YAW` |

`~/pm01_0528` 两条桥接路径在桥层使用的 sign 全是 `+1`、offset 全是 `0`。这说明桥的
输入输出采用已归一化关节坐标；Native SDK 电机层另有自己的 sign。不要把电机层 sign
直接复制到本项目，必须用只读状态和小幅测试确认桥层坐标约定。

在越界处理修复前，即使限位填写正确，也不能依赖当前代码阻断危险命令。

## 9. 运行

仅在 P0 项全部关闭、配置评审完成、机器人有可靠支撑且现场急停人员就位后执行：

```bash
cd adapter_any_robot
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=69
export ROS_LOCALHOST_ONLY=0
./build-pm01-only/adapter_bridge config/pm01.robot.yaml
```

启动顺序：

1. 清空机器人周围至少 2 m，安装机械支撑，确认电量和急停有效。
2. 急停保持按下，上电并检查只读 ROS 2 Topic。
3. 启动 bridge，但外部 DDS 命令发布器保持关闭。
4. 确认持续收到 24 关节状态，无 `waiting for /hardware/joint_state`。
5. 由机器人官方流程进入 `pd-stand`，再进入 `joint_bridge`。
6. 先发送零增量、低刚度且经过限幅的保持命令。
7. 单关节、小幅度、低速测试；随时准备触发硬件急停。
8. 验证断开 DDS、停止发布、SIGTERM 和网络掉线均进入经厂商确认的安全状态。

## 10. 上机放行标准

只有同时满足以下条件，结论才能从 NO-GO 改为有条件 GO：

- [ ] P0-1 至 P0-7 已修复并有自动化测试。
- [ ] `ctest` 100% 通过，PM01 initialize/shutdown 可重复至少 100 次。
- [ ] 使用目标机器人相同 CPU 架构完成编译和动态库检查。
- [ ] 本地 `interface_protocol` 与机器人实际 Topic type/hash 完全一致。
- [ ] 24 关节顺序、方向、零偏和所有命令限位已双人复核。
- [ ] ROS 2 使用 Cyclone DDS，domain、网卡和 multicast 通信验证通过。
- [ ] 外部 LowCmd DDS 的 domain、Topic、网卡与命令发布端一致。
- [ ] 实测 `/hardware/joint_state` 和 DDS `rt/lowstate` 均稳定达到目标频率；不能仅依据
  `loop_rate_hz: 800` 判定。
- [ ] 机器人硬件急停、软件 watchdog 和进程退出安全状态均实测通过。
- [ ] 无动力联调、机械支撑测试、单关节测试依次通过。
- [ ] 已保存首轮测试日志、配置版本、Git commit 和机器人固件版本。

## 11. 厂商公开依据

- [EngineAI ROS 2 工作区：环境与 DDS 配置](https://github.com/engineai-robotics/engineai_ros2_workspace/blob/ebd638e31709a038d3208517693d33174dbacb46/README_CN.md#L31-L60)
- [EngineAI ROS 2 工作区：机器人以太网连接](https://github.com/engineai-robotics/engineai_ros2_workspace/blob/ebd638e31709a038d3208517693d33174dbacb46/README_CN.md#L81-L85)
- [EngineAI interface_protocol Topic 说明](https://github.com/engineai-robotics/engineai_ros2_workspace/blob/ebd638e31709a038d3208517693d33174dbacb46/src/interface_protocol/README.md#L8-L18)
- [EngineAI 真机 joint_bridge 流程](https://github.com/engineai-robotics/engineai_ros2_workspace/blob/ebd638e31709a038d3208517693d33174dbacb46/README_CN.md#L111-L135)
- [EngineAI Native SDK PM01 目标](https://github.com/engineai-robotics/engineai_robotics_native_sdk/blob/70bac3c0c47212adad4161e55b06f5a0705841b7/README_CN.md#L107-L124)
- [PM01 官方支持页](https://www.engineai.com.cn/product-support-pm01.html)
- [PM01 官方手册 PDF](https://cn-engineai-1304599088.cos.ap-guangzhou.myqcloud.com/uploads/upload/files/20251031/c15bec68ae58e2f5cd2901aad04e0750.pdf)

公开资料不能替代购买版本随附的固件说明、关节参数和售后安全规程。Topic 类型、关节顺序、
watchdog 行为和安全卸载命令必须以当前机器人固件和众擎技术支持确认为准。
