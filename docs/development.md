# 开发指南

[首页](../README.md) · [G1](unitree_g1/README.md) · [K2](kepler_k2/README.md) · [CArm](cvte_carm/README.md)

[构建](#构建) · [Docker](#docker) · [统一接口](#统一接口) · [配置与运行](#配置与运行) · [新增设备](#新增设备) · [测试](#测试)

## 构建

需要 Linux x86_64 / ARM64、CMake ≥ 3.16、C++17 编译器和 Python 3。SDK 必须与运行主机的操作系统和 CPU 架构匹配。

```bash
git submodule update --init --recursive
sudo apt-get install -y build-essential cmake git python3 python3-venv
```

按机型构建，避免引入其他机器人的依赖：

| 命令 | 输出目录 |
|---|---|
| `bash scripts/build_robot.sh g1` | `build-g1-only` |
| `bash scripts/build_robot.sh k2` | `build-k2-only` |
| `bash scripts/build_robot.sh carm` | `build-cvte-only` |
| `bash scripts/build_robot.sh pm01`（[PM01](pm01.md)） | `build-pm01-only` |

脚本的第二个参数指定构建目录，`BUILD_JOBS` 设置并行数，默认 2。CArm 的外部 SDK 安装见设备手册。

手工配置示例：

```bash
cmake -S . -B build-g1 \
  -DBRIDGE_ROBOT=unitree_g1 \
  -DBUILD_UNITREE_G1_MOTION_DEMO=ON \
  -DBUILD_GAMEPAD_SDL=OFF \
  -DBUILD_TESTING=ON
cmake --build build-g1 -j2
ctest --test-dir build-g1 --output-on-failure
```

| CMake 选项 | 用途 |
|---|---|
| `BRIDGE_ROBOT` | 必选：`unitree_g1`、`kepler_k2`、`cvte_carm` 或 `pm01` |
| `BUILD_BRIDGE_DDS` | 通用 DDS 桥接；CArm 默认 OFF，其余默认 ON |
| `BRIDGE_CVTE_SDK_DIR` | CArm SDK 根目录，包含 include、lib |
| `BUILD_UNITREE_G1_MOTION_DEMO` | G1 高层键盘程序；手工 CMake 默认 OFF，构建脚本开启 |
| `BUILD_UNITREE_G1_HARDWARE_TESTS` | 生成 G1 硬件接口检查程序，默认 OFF，不加入普通 CTest |
| `BUILD_GAMEPAD_SDL` | SDL 手柄支持，需 `libsdl2-dev`；构建脚本默认关闭 |
| `BUILD_TESTING` | 生成离线测试 |

`all` 是历史兼容选项，包含 K2、PM01、G1，不包含 CArm。切换机型时使用独立构建目录。

## Docker

macOS 可用 Linux 容器编译并运行模拟测试：

```bash
docker run --rm -it \
  -v "$PWD:/workspace" -w /workspace ubuntu:22.04 bash
```

在容器内安装上面的构建工具，再执行对应机型的构建命令。项目外的 SDK 需要额外挂载。
Apple Silicon 默认使用 Linux ARM64 镜像，Intel 使用 x86_64；真机通信优先在机器人或 Linux 上位机运行，DDS 组播不会因目录挂载自动接通。

## 统一接口

```text
配置 → RobotAssemblyFactory
          ├── BridgeCommandIngress → BridgeRunner → RobotSdkAdapter → 厂商 SDK
          └── BridgeStateEgress    ←             ←                  ← 设备

独立应用 → 机型专用 SDK 接口
```

公开契约：[RobotSdkAdapter](../include/bridge/core/robot_sdk_adapter.hpp)、[数据类型](../include/bridge/core/types.hpp)。

| 接口 | 用途 |
|---|---|
| `initialize()` | 建立会话 |
| `read_state()` | 获取状态；检查返回结果、optional state 和 `valid/stale/degraded` |
| `write_command(CommandFrame)` | 提交适配器支持的关节命令 |
| `query_health()` | 查询连接与健康 |
| `shutdown()` | 关闭会话；具体停车、伺服行为由机型决定 |
| `StateFrame` / `CommandFrame` | 关节状态／命令、时间戳、IMU 等通用字段 |
| `RobotCapabilitySet` | 声明适配器支持的能力 |

厂商扩展接口保留在对应机型模块中。自定义程序按需链接 `bridge_core`、`bridge_adapter_unitree_g1` 或 `bridge_adapter_cvte_carm` 等 CMake 目标。

## 配置与运行

SDK 示例可独立运行。需要通用关节转发时，复制配置模板、修改本地副本，再启动：

```bash
./build-<机型>/adapter_bridge config/<机型>.local.yaml
```

各设备手册说明模板用途和是否适用于真实硬件。

| 配置字段 | 方向／作用 |
|---|---|
| `dds.topics.command` | 上层程序 → bridge |
| `dds.topics.state` | bridge → 上层程序 |
| `adapter.topics.command` | bridge → 设备 |
| `adapter.topics.state` | 设备 → bridge |
| `adapter.connection.interface_hint` | 设备连接信息，按适配器解释 |
| `joint_mapping.entries` | 关节索引、名称、符号和偏移 |

配置解析实现在 [bridge_config.cpp](../src/bridge/config/bridge_config.cpp)。样例中的旧格式 `joints` 仍可解析。

| 问题 | 检查顺序 |
|---|---|
| 缺少共享库 | 初始化子模块或配置外部 SDK 路径 |
| `wrong ELF class` / `file in wrong format` | SDK 的系统、CPU 架构是否匹配 |
| 更换机型后配置失败 | 使用新构建目录 |
| 收不到状态 | 网卡／IP → 发布进程 → domain、topic、消息类型 → QoS → 样本计数 |
| 命令返回 `unsupported` | 查看适配器支持范围，专有能力调用厂商 SDK |

## 新增设备

1. 在 `include/bridge/adapters/<name>` 和 `src/bridge/adapters/<name>` 定义配置与实现。
2. 实现连接、读写、健康和关闭接口，将厂商错误转为结构化结果。
3. 明确关节顺序、单位、超时及支持的控制模式。
4. 在 `src/bridge/app/robot_assembly_factory.cpp` 注册，在 `cmake/Options.cmake`、`cmake/Adapters.cmake` 添加构建选项。
5. 添加配置、测试和 `docs/<name>/README.md`，在首页添加入口。

设备命令、接口和实测范围集中维护在对应设备手册，避免复制到多个页面。

## 测试

```bash
cmake --build build-g1 -j2
ctest --test-dir build-g1 --output-on-failure
python3 scripts/check_repository.py
```

普通 CTest 运行离线测试；硬件检查需显式构建并人工运行。提交只包含源码、配置、正式文档与必要资源，厂商 SDK 和构建产物不提交。
