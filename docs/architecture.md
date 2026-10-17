# 技术架构与二次开发

[首页](../README.md)

## 统一哪些部分

核心只统一生命周期、关节状态／命令、健康与故障。厂商专有能力保留独立接口：G1 用 SDK facade，CArm 的未封装能力用厂商 SDK。图像和点云不会塞入通用关节数组。

```text
配置 → RobotAssemblyFactory
              ├── BridgeCommandIngress → BridgeRunner → RobotSdkAdapter → 厂商 SDK
              └── BridgeStateEgress    ←             ←                  ← 设备

独立应用 → G1 SDK facade / SensorSession / CArm SDK
```

| 模块 | 路径 | 责任 |
|---|---|---|
| 核心 | `include/bridge/core`、`src/bridge/core` | 帧、生命周期、超时与故障处理 |
| 配置 | `include/bridge/config`、`src/bridge/config` | YAML 解析、关节映射与校验 |
| 适配器 | `include/bridge/adapters/<robot>`、`src/bridge/adapters/<robot>` | 厂商连接与协议转换 |
| DDS | `include/bridge/dds`、`src/bridge/dds` | CycloneDDS、序列化与消息方向 |
| 装配 | `src/bridge/app` | 按配置创建适配器与输入输出 |
| 应用 | `src/apps`、`examples` | 可执行工具、操作示例 |
| 构建 | `cmake` | Options、Dependencies、Core、Adapters、Applications、Tests |

## 通用契约

公开入口：[RobotSdkAdapter](../include/bridge/core/robot_sdk_adapter.hpp)、[类型](../include/bridge/core/types.hpp)。

| 方法／数据 | 使用方式 |
|---|---|
| `initialize()` | 建立会话；检查结果再读取状态 |
| `read_state()` | 同时检查返回结果、optional state、valid/stale/degraded |
| `write_command(CommandFrame)` | 按能力、模式、维度和时间戳构造；不支持则返回错误 |
| `query_health()` | 检查连接与健康，不能用 capability 代替在线状态 |
| `shutdown()` | 关闭会话；不假定所有厂商都会自动停止或关闭伺服 |
| `StateFrame` | 关节、IMU、时间戳及可用性等字段 |
| `CommandFrame` | 模式、关节目标、增益等字段；适配器只接受已实现的子集 |
| `RobotCapabilitySet` | 查询实现声明，不是设备已验收证据 |

G1 的默认低层 adapter 会释放官方运动服务；高层运控使用独立 client，不初始化低层 adapter。
CArm adapter 的初始化不调用 ready/伺服，`write_command` 支持位置跟随，不支持通用速度／力矩命令。
K2 当前使用模拟会话。导航能力位和点云订阅存在，也不表示 G1 驱动或导航控制已经可用。

## 构建选择

`BRIDGE_ROBOT` 选择适配器，CArm 默认不构建公共 DDS；使用独立 SDK 时无需安装其他机器人环境。
自有 CMake 目标按需链接 `bridge_core`、`bridge_adapter_unitree_g1`、`bridge_adapter_cvte_carm` 等目标。
SDK 类型留在对应厂商实现或专用接口中，公共核心不依赖 CArm、G1 的运动方法名。

## 添加设备

1. 在 `include/bridge/adapters/<name>` 定义配置与 adapter，在 `src/bridge/adapters/<name>` 实现。
2. 实现五个生命周期／读写／健康方法，将厂商异常和错误码转换为结构化结果。
3. 明确关节顺序、维度、单位、超时和不支持的命令；不自动截断未知硬件数据。
4. 在 `src/bridge/app/robot_assembly_factory.cpp` 注册配置类型。
5. 在 `cmake/Options.cmake`、`cmake/Adapters.cmake` 增加独立构建选择。
6. 添加配置、离线测试、只读工具和机型文档；真机测试不加入普通 CTest。
7. 按连接、状态、受控运动、停止、断连恢复顺序验证，并记录固件与 SDK 版本。

## 维护要求

API／topic／按键发生变化时，同步修改示例、测试、对应机型文档和首页入口。
现场测试结论写入 `docs/validation.md`，标明设备、固件和验证范围；没有测试的项目保持“待验证”。
提交只包含源码、配置、正式文档和必要依赖引用；本地 SDK、构建结果与开发记录由 `.gitignore` 排除。
