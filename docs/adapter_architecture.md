# Adapter Any Robot 架构说明

本文档定义本项目接入新机器人时必须遵守的边界。目标是复用稳定的运行时骨架，同时允许不同厂商保留自己的 SDK、通信协议和专有能力。

## 核心原则

统一的是运行时契约，不是厂商 SDK 的全部 API：

- 核心层统一生命周期、关节状态、关节命令、健康状态、错误和时间戳。
- adapter 层负责把具体厂商 SDK 转换为核心契约。
- 厂商能力层负责末端、笛卡尔运动、运动学、示教、音频、灯光、导航和专用传感器。
- 传感器数据独立于关节 `StateFrame`，避免把图像、点云或导航消息强行压缩成关节字段。
- 上层只能通过 capability 判断是否调用专有能力，不能通过机器人名字散落条件分支。

## 运行时分层

```text
上层输入/输出
  DDS、ROS2、测试程序、其他传输
          |
          v
BridgeCommandIngress / BridgeStateEgress
          |
          v
BridgeRunner
  生命周期、超时、故障策略、状态发布节拍
          |
          v
RobotSdkAdapter
  initialize / read_state / write_command / query_health / shutdown
          |
          +--> 通用 StateFrame / CommandFrame
          |
          +--> 厂商 SDK facade
          |      CArm、Unitree、其他控制器协议
          |
          +--> 可选 SensorSession
                 相机、点云、IMU、里程计、导航
```

`BridgeRunner` 不应该知道 CArm、Unitree 或其他厂商类名。`RobotAssemblyFactory` 只负责按配置装配对应 adapter、关节映射和输入输出组件。

## 构建选择

每次构建必须明确选择机器人，使用 CMake 的 `BRIDGE_ROBOT`：

```bash
cmake -S . -B build-g1 -DBRIDGE_ROBOT=unitree_g1
cmake --build build-g1 --target adapter_bridge
```

可选值为 `kepler_k2`、`pm01`、`unitree_g1`、`cvte_carm` 和 `all`。`all` 只构建仓库内默认可用的 K2、PM01、G1；CArm 依赖本地二进制 SDK，必须显式选择 `cvte_carm` 并设置 `BRIDGE_CVTE_SDK_DIR`。CArm 默认使用 `BUILD_BRIDGE_DDS=OFF`，只构建 SDK adapter 和烟测；需要公共 DDS 主程序时再显式打开它。未设置 `BRIDGE_ROBOT` 时 CMake 会直接停止配置，避免误把其他机器人的 SDK 加入当前环境。

日常构建优先使用 `scripts/build_robot.sh`，它会为每种机器人使用独立的默认构建目录，避免不同 SDK 的 CMake 缓存互相污染。

## 核心契约

### `RobotSdkAdapter`

位置：`include/bridge/core/robot_sdk_adapter.hpp`

每个 adapter 必须实现：

| 方法 | 责任 |
| --- | --- |
| `name()` | 返回稳定的注册名称 |
| `capabilities()` | 声明当前 adapter 支持的共性和扩展能力 |
| `initialize()` | 创建连接、订阅或 SDK 会话，不发送危险运动命令 |
| `read_state()` | 读取一次一致的状态快照 |
| `write_command()` | 校验并发送通用命令，明确返回 unsupported 或错误 |
| `query_health()` | 返回连接、最近读写时间和健康状态 |
| `shutdown()` | 幂等关闭所有连接和后台资源 |

方法不得把厂商异常泄漏到 runner；厂商异常必须转换成 `AdapterCallResult` 和 `BridgeFault`。

### Capability

`RobotCapabilitySet` 是轻量位掩码，位于 `include/bridge/core/types.hpp`。当前能力包括：

- 关节状态、关节位置/速度/力矩命令
- 笛卡尔运动、末端执行器、运动学、示教
- IMU、相机、点云、导航、音频和灯光

能力声明只表示接口存在，不表示设备当前在线。运行时仍然必须检查 `AdapterCallResult`、`StateFrame.valid` 和 `AdapterHealth`。

例如，当前 CArm adapter 只声明关节状态和位置跟随；Unitree G1 还声明 IMU、相机、点云、导航、音频和灯光能力。CArm 的笛卡尔、末端、运动学和示教仍属于厂商扩展待接入项。CArm 的速度/力矩通用命令当前会返回 `kUnsupported`，不能仅因为设备有 MIT 模式就假装核心已经支持该命令。

当前适配器能力矩阵：

| Adapter | 已声明能力 |
| --- | --- |
| CArm A3/D3 | 关节状态、关节位置跟随 |
| Unitree G1 | 关节状态、位置/力矩命令、IMU、相机、点云、导航、音频、灯光 |
| 其他 adapter | 继承核心默认空集合，新增能力必须在实现和测试完成后声明 |

## 目录约定

```text
include/bridge/core/                 稳定核心契约和 runner
include/bridge/adapters/<robot>/     厂商 adapter 的公开配置和 facade
src/bridge/adapters/<robot>/         厂商 SDK 实现和协议转换
include/bridge/app/                  工厂和装配
src/bridge/app/                      注册机器人类型
config/<robot>.sample.yaml           可运行配置样例
tests/adapters/<robot>_test.cpp      离线适配器测试
tests/adapters/<robot>_smoke.cpp     真机烟测
docs/<robot>_integration.md          现场接入和 SDK 文档
```

厂商 SDK 头文件只能出现在对应 adapter 的 `.cpp` 或厂商 facade 内部。核心层、`BridgeRunner`、DDS codec 和其他机器人不得直接包含厂商头文件。

## 新机器人接入流程

1. 明确 SDK 传输方式、连接生命周期、状态读取频率和控制模式。
2. 在 `include/bridge/adapters/<robot>/` 定义配置和 adapter，不修改其他机器人代码。
3. 将 SDK 异常、错误码和超时转换为 `AdapterCallResult`。
4. 把能稳定共享的关节字段映射到 `StateFrame`，专有数据放到 facade 或独立 `SensorSession`。
5. 声明 capability；不支持的通用命令必须返回 `kUnsupported`。
6. 在 `RobotAssemblyFactory` 注册一个稳定的配置类型。
7. 添加离线测试、配置样例和只读真机烟测。
8. 先读状态和健康，再做伺服、低速运动、末端和急停测试。

## 两类测试

### 离线测试

不连接设备，验证：

- 配置解析和 joint map
- 状态/命令宽度校验
- 生命周期幂等性
- 错误码和 unsupported 行为
- 工厂注册和装配

### 真机烟测

真机测试必须单独编译或显式启用，顺序固定为：

```text
网络连通 -> connect -> 只读状态 -> 健康检查
  -> 安全负责人确认 -> ready/控制模式 -> 低速运动
  -> 末端 -> 急停 -> 关闭伺服 -> disconnect
```

烟测程序不能在构造时自动移动设备，也不能把 `set_ready` 或伺服使能隐藏在普通 `initialize()` 中。

## CArm 和 Unitree 的边界

| 内容 | CArm | Unitree G1 |
| --- | --- | --- |
| 控制器协议 | 网络 SDK/WebSocket 后端 | SDK2、DDS 和低层 topic |
| 通用状态 | CArm 关节位置/速度/力矩 | LowState 关节、IMU 和辅助状态 |
| 通用命令 | 当前位置跟随 | 低层关节命令和 G1 SDK 能力 |
| 专有扩展 | 末端、运动学、示教、轨迹 | 步态、音频、灯光、相机、SLAM/LiDAR |
| 传感器 | 独立 ROS2/相机会话 | 独立 G1 sensor session |

两者都复用 `RobotSdkAdapter` 和 `BridgeRunner`，但不共享厂商传输实现。

## 文档维护规则

每个机器人至少维护一份文档，内容包括：

- 官方资料和 SDK 版本
- 构建依赖和控制器连接方式
- 配置字段和 joint map
- capability 矩阵
- 只读、运动和急停测试结果
- 设备固件、IP、ROS 话题等现场变量
- 已知不支持能力和风险边界

公共架构变化更新本文档；单个机器人变化只更新对应机器人文档，不把厂商细节写进公共核心说明。
