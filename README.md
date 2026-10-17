# deploy_omni

机器人 SDK 适配与 DDS 转发框架。按机器人单独编译，统一连接、关节状态、关节命令和故障处理；相机、音频、夹爪等能力保留各厂商接口。

## 先选你的机器人

| 设备 | 从这里开始 | 当前状态 |
|---|---|---|
| 宇树 G1 | [连接、状态、运控、相机、雷达](docs/unitree_g1/README.md) | 已有状态及部分服务真机记录；新版键盘运控通过离线测试，待真机验证 |
| Kepler K2 | [编译与模拟验证](docs/kepler_k2/README.md) | 等待接入官方api |
| CVTE CArm A3 单臂 | [6 关节＋夹爪操作](docs/cvte_carm/single-arm.md) | 已做单臂关节与夹爪现场测试；本次整理后的版本为离线复核 |
| CVTE CArm D3 双臂 | [左右臂接口与接入](docs/cvte_carm/dual-arm.md) | SDK 适配已实现，尚未真机测试；按左右臂分别检查 |

> 夹爪不计入机械臂关节数：已测试的 A3 是 **6 个关节＋1 个夹爪**。D3 配置样例按每侧 7 关节编写，以设备实际返回为准。

CArm 二次开发程序运行在**上位机**：上位机程序 → SDK → WebSocket（A3 示例 `10.42.0.101:8090`）→ 控制器 → 机械臂本体。连接、状态、运动、夹爪及 ROS / ROS2 扩展方式见 [上位机架构与开发能力](docs/cvte_carm/README.md#上位机与控制器的架构链路)。

## 三步开始

以下命令在 **Linux x86_64 / ARM64** 的项目根目录运行。macOS 用户先看 [Docker 构建](docs/build.md#macos-与-docker)。

### 1. 下载并安装构建工具

```bash
git clone --recurse-submodules https://github.com/WXyuany/deploy_omni.git
cd deploy_omni
sudo apt-get update
sudo apt-get install -y build-essential cmake git python3 python3-venv
```

已经 clone 但缺少 SDK 目录时：`git submodule update --init --recursive`。
CArm 的厂商 SDK 不随本仓库分发，见 [SDK 安装](docs/cvte_carm/README.md)。

### 2. 只编译需要的机器人

```bash
bash scripts/build_robot.sh g1       # G1
bash scripts/build_robot.sh k2       # K2 模拟后端
# CArm 单臂和双臂共用一次构建：换成实际 SDK 目录
BRIDGE_CVTE_SDK_DIR=/opt/cvte/arm_control_sdk bash scripts/build_robot.sh carm
```

每次只选择一行。编译不会启动机器人；默认并行数为 2，可用 `BUILD_JOBS=4` 调整。

### 3. 先读取状态

在 G1 内部运行时，先按 [SSH 登录说明](docs/unitree_g1/README.md#1-ssh-登录机器人) 通过网线或 `MEETINGROOM_WIFI` 登录。

```bash
# G1：eth0 换成连接机器人的网卡；只读，不起立、不行走
./build-g1-only/unitree_g1_motion_demo --check --interface eth0

# CArm A3：在控制电脑运行，不需要 SSH 登录机械臂
./build-cvte-only/cvte_carm_check --model a3 --ip 10.42.0.101

# CArm D3：替换实际控制器 IP
./build-cvte-only/cvte_carm_check --model d3 --ip 10.42.0.101
```

K2 的检查命令和限制见 [K2 文档](docs/kepler_k2/README.md)。状态正常后，再按各机器人文档执行运动验证。

## G1 键盘速查

启动方式及姿态转换条件见 [完整运控手册](docs/unitree_g1/high-level-motion.md)。默认只读，显式 `--enable-motion` 才接受动作。

| 小写键 | 功能 |
|---|---|
| `z` / `p` / `u` | 零力矩／阻尼／起立 |
| `t` / `b` | 请求运动状态 FSM=500／平衡站立 |
| `r` 后 `w/s/a/d` | 解锁一次前后左右移动，0.5 秒低速脉冲 |
| `r` 后 `j/l` | 解锁一次左右转向 |
| `c/v`、`o/n` | 下蹲／坐下、高站姿／低站姿 |
| `h/k/m` | 挥手／伸手握手／收手 |
| 空格或 `x`、`q` | 停车、尝试停车后退出 |

动作键短按一次即可，无需 `!` 或回车。零力矩、阻尼需要可靠支撑；软件停车不是硬件急停。

## 目录与技术资料

```text
cmake/                 构建选项、依赖、模块和测试目标
config/                各机器人配置样例
include/bridge/        核心契约、适配器、DDS 的公开头文件
src/bridge/            对应实现；按 core / adapters / dds 等分层
src/apps/              桥接主程序及状态工具
examples/              unitree_g1 / cvte_carm / gamepad 示例
scripts/               构建与仓库检查工具
tests/                 离线测试及显式启用的硬件接口检查
docs/                  按机型组织的操作和技术文档
third_party/           固定版本的共享依赖、ROS 消息包
```

- [构建、依赖与测试](docs/build.md)
- [统一接口与扩展方法](docs/architecture.md)
- [配置、DDS 方向和排错](docs/runtime.md)
- [G1 雷达点云与里程计](docs/unitree_g1/lidar.md)
- [G1 SDK 接口索引](docs/unitree_g1/sdk.md)
- [CArm SDK 与单／双臂接口对照](docs/cvte_carm/README.md)
- [验证记录与待验收项](docs/validation.md)
- [第三方依赖与许可说明](THIRD_PARTY.md)

PM01 作为现有附加适配器保留，见 [说明](docs/pm01.md)。各设备的“编译通过”“数据可读”“动作已验收”分开记录，不互相替代。
