# Bridge 模块边界

本文档是独立 bridge 框架的职责边界真值来源（source of truth），对应 `.sisyphus/plans/robot-sdk-bridge-framework.md` 中描述的方案。

## 边界规则

- 该框架是一个独立的 bridge 应用，而不是 `BaseRobotBackend` 的扩展。
- 核心闭环范围固定为：`DDS 命令接入 -> 校验后的中间命令帧 -> SDK 写入 -> SDK 状态读取 -> 中间状态帧 -> DDS 发布`。
- core 层拥有生命周期、循环/线程、看门狗、故障处理、命令/状态流、映射以及启动策略。
- adapter 代码只拥有机器人 SDK 的初始化/读取/写入/关闭，不拥有任何 DDS 相关职责。
- 启动配置只加载一次、校验一次，然后冻结。
- 在收到第一条有效命令之前，系统必须保持零力矩或 adapter-safe idle。
- 故障策略是 fail-stop：停止正常命令写入，如果支持则发送一条 safe command，并继续发布 degraded state。
- joint lock、torque smoothing、CSV 日志、彩色打印、额外模式等 legacy 增强功能必须放在 MVP 核心之外，只能通过显式 optional hooks 接入。

## 目标模块与职责归属

| 目标模块 | 标签 | 拥有的职责 | 不得拥有的职责 |
| --- | --- | --- | --- |
| `bridge_app_main` | `app` | 进程入口、CLI/env 交接、配置路径选择、runner + transport + adapter 的装配、退出码选择 | 控制循环逻辑、DDS codec 细节、SDK I/O、映射数学 |
| `bridge_runner` | `core` | Bridge 生命周期状态机（`Created -> Initializing -> WaitingForState -> Active -> Faulted -> Stopping -> Stopped`）、启动/关闭顺序、周期性 read/publish 循环、latest-command buffer 消费、看门狗、首条命令前的 safe-idle 策略、故障状态迁移 | DDS runtime 建立、生成的 DDS message 类型、SDK 专有 API、文件解析、可选 legacy 行为 |
| `dds_ingress` | `transport` | DDS 订阅建立、生成的 DDS command message 接收、命令翻译到 `CommandFrame`、sim->hardware joint 投影、把结果交给 runner 拥有的 latest-command buffer | SDK 写入、周期循环所有权、生命周期状态机 |
| `dds_egress` | `transport` | DDS publisher 建立、hardware->sim joint 投影、生成的 DDS state/status message 填充与发布 | SDK 读取、runner 生命周期决策 |
| `intermediate_frames` | `core` | 机器人无关的 `CommandFrame`、`StateFrame`、状态/故障枚举，以及闭环所需的时间戳/序列号字段 | DDS wire type、SDK handle、机器人专有 joint 命名假设 |
| `joint_mapping` | `core` | transport-level joints 与 intermediate frames 之间的声明式映射、顺序/索引归一化、帧校验所需的 limit、命令/状态字段投影规则 | DDS transport 构造、SDK 调用、生命周期线程控制 |
| `config_loader` | `config` | 启动期文件加载、schema 校验、配置归一化、为 runner/transport/adapter 接线生成冻结后的配置对象 | 运行期热更新、循环执行、SDK I/O、运行时故障恢复策略修改 |
| `robot_sdk_adapter` | `adapter` | SDK 初始化、硬件/会话建立、safe command 能力、命令写入、状态读取、健康检查、关闭 | DDS 概念、transport 消息类型、runner 线程策略、adapter 专属配置之外的配置文件解析 |
| `imu_source` | `IMU source` | 当状态发布需要 IMU 时提供可选的机器人无关 IMU 接口、带时间戳的 IMU 采集、adapter-owned 或外部 IMU 源绑定 | 命令接入、runner 生命周期控制、DDS 订阅所有权 |
| `optional_hooks` | `optional extension` | 面向日志、遥测镜像、joint lock、torque smoothing、额外模式、debug probe 的显式后 MVP 扩展缝隙 | 必需的 bridge 安全行为、基线命令/状态主链路、core 内部隐藏副作用 |

## 按关注点划分的职责

### 启动阶段职责归属

1. `bridge_app_main` 选择配置来源并构造所有模块。
2. `config_loader` 解析并校验完整 bridge 配置，然后返回一个不可变配置对象。
3. `bridge_app_main` 构建 `dds_ingress`、`dds_egress`、`joint_mapping`、`robot_sdk_adapter`、可选的 `imu_source` 与 `optional_hooks`，然后把它们注入到 `bridge_runner`。
4. `bridge_runner` 拥有运行时启动顺序：
   - 切换到 `Initializing`
   - 初始化 adapter
   - 建立 transport 端点
   - 获取第一帧可读机器人状态
   - 按需发布初始 degraded/waiting 状态
   - 只有在状态与 transport 都准备完成后，才从 `WaitingForState` 进入 `Active`
5. 在第一条有效命令被消费之前，`bridge_runner` 必须让 adapter 保持零力矩或 adapter-safe idle。

### 关闭阶段职责归属

- 当收到 stop 请求或发生致命故障后，`bridge_runner` 拥有关机顺序。
- `bridge_runner` 会停止正常命令消费、如支持则请求发送一条 adapter safe command、停止周期循环，并经由 `Stopping` 进入 `Stopped`。
- `robot_sdk_adapter` 拥有最终的 SDK teardown。
- `bridge_app_main` 只拥有进程级退出码和 runner 停止后的最终析构顺序。

### 循环与线程职责归属

- 整个系统只能由 `bridge_runner` 拥有一个周期性运行循环。
- 该周期循环执行：adapter 状态读取 -> 可选 IMU 读取 -> 状态帧组装 -> DDS egress 发布 -> 看门狗/故障评估 -> 按 runner 策略应用下一周期命令。
- `dds_ingress` 可以运行 DDS callback 线程，但这些 callback 只能负责解码并更新 runner 拥有的 latest-command buffer。
- DDS callback 线程绝不能直接调用机器人 SDK。
- `optional_hooks` 可以观察事件，但不能成为任何必需运行循环的拥有者。

## 命令与状态流边界

### 命令流

1. `dds_ingress` 接收一条 DDS 命令消息。
2. `dds_ingress` 接收生成的 DDS command message，并拒绝格式错误的 transport 输入。
3. `dds_ingress` 使用 `joint_mapping` 与 `intermediate_frames` 的规则构建一个适合 bridge core 的、校验后的 hardware-order `CommandFrame`。
4. `bridge_runner` 保存最新的有效命令帧，并决定它何时在周期循环中生效。
5. `robot_sdk_adapter` 将当前生效的 `CommandFrame` 翻译成 SDK-native command 调用。
6. 如果还没有收到有效命令，`bridge_runner` 应发送 safe idle，而不是推断为 hold-position 行为。

### 状态流

1. `bridge_runner` 触发一次周期性读取。
2. `robot_sdk_adapter` 从 SDK 读取机器人状态。
3. 若有配置，可选的 `imu_source` 提供 IMU 数据。
4. `dds_egress` 将 SDK 状态投影成机器人无关/仿真顺序的状态，并填充生成的 DDS state/status message。
5. `dds_egress` 发布这些 typed DDS 输出消息。

## 配置职责边界

- `config_loader` 是唯一允许解析 bridge 配置文件的模块。
- `joint_mapping` 接收的是归一化后的映射配置，而不是原始文件文本。
- `robot_sdk_adapter` 在校验后只接收属于它自己的 adapter 配置子集。
- `bridge_runner` 接收冻结后的运行时配置值，例如 loop period、看门狗阈值、startup idle 策略、fault 行为标志等。
- 启动后任何模块都不得修改 live config。

## 故障边界

| 故障类别 | 主要拥有者 | 边界行为 |
| --- | --- | --- |
| DDS 解码 / 订阅故障 | `dds_ingress` + `bridge_runner` | transport 报告失败；runner 决定保持 degraded 还是进入 `Faulted` |
| 映射/校验后的非法命令内容 | `bridge_runner` | 拒绝该命令，保持最近的 safe/accepted 行为，维持 SDK 隔离 |
| SDK 初始化/读取/写入/健康故障 | `robot_sdk_adapter` 负责上报，`bridge_runner` 拥有策略 | adapter 报告故障原因；runner 进入 `Faulted`，停止正常写入，如支持则请求 safe command |
| 配置/schema 故障 | `config_loader` | 在运行前直接启动失败 |
| 可选功能故障 | `optional_hooks` | 不能打断 core loop；除非显式配置为 fatal，否则 hook 错误必须隔离 |
| IMU 采集故障 | `imu_source` + `bridge_runner` | 按策略发布 degraded state 或省略 IMU 字段；不得让 IMU 细节泄漏到命令链路 |

硬边界在于：adapter 只负责暴露能力与故障，但只有 `bridge_runner` 才能决定生命周期迁移和 fail-stop 行为。

## 扩展边界

- `optional_hooks` 是唯一允许承载非 MVP 行为的扩展缝隙。
- hooks 只能在 runner/app 显式选定的扩展点观察或变换数据。
- hooks 不得偷偷改变必需的安全行为、故障策略、启动顺序或 transport 所有权。
- legacy extras 都应归入这里，除非以后被正式提升为 core requirement。

## Legacy 职责迁移映射

| Legacy 概念职责 | 目标模块 | 标签 | 说明 |
| --- | --- | --- | --- |
| `main()` 启动与 ROS/DDS 进程拉起 | `bridge_app_main` | `app` | 保持轻量，只做模块装配 |
| 单体化生命周期标志与 bridge 状态迁移 | `bridge_runner` | `core` | 运行时状态机的唯一权威来源 |
| DDS subscriber callback 接入 | `dds_ingress` | `transport` | callback 只写 latest command buffer |
| DDS publisher 接线和对外状态发布 | `dds_egress` | `transport` | outbound codec/publish 与 core 逻辑解耦 |
| 与 K2 数据布局耦合的共享命令结构 | `intermediate_frames` | `core` | 用机器人无关的 frames 替换机器人专有结构 |
| joint index remap / name remap 逻辑 | `joint_mapping` | `core` | 用声明式映射，避免硬编码进 adapter |
| YAML/JSON 参数解析与默认值 | `config_loader` | `config` | 只解析一次、校验一次、冻结 |
| `motor_api` 会话初始化、命令发送、状态读取、关闭 | `robot_sdk_adapter` | `adapter` | adapter 是唯一面向 SDK 的层 |
| IMU 轮询或 SDK 专有惯导快照处理 | `imu_source` | `IMU source` | 独立出来，允许 IMU 缺失或替换 |
| joint lock / torque smoothing / CSV 日志 / debug 打印 / extra modes | `optional_hooks` | `optional extension` | 不进入 MVP 核心路径，必须显式 opt-in |

## 下游任务不可打破的缝隙

- 下游类型设计必须把 `intermediate_frames` 视为 transport 与 adapter 之间唯一共享的命令/状态契约。
- 下游 runner 设计必须把所有运行时控制决策都留在 `bridge_runner` 内部。
- 下游 adapter 设计必须维持 DDS-free 接口。
- 下游 config 设计必须保留“一次加载 / 一次校验 / 冻结”的行为。
- 下游扩展设计必须通过 `optional_hooks` 接入，绝不能重新打开 core ownership。
