# 开发指南

[首页](../README.md) · [X2](agibot_x2/README.md) · [G1](unitree_g1/README.md) · [CArm](cvte_carm/README.md)

[构建](#构建) · [统一接口](#统一接口) · [配置与运行](#配置与运行) · [新增设备](#新增设备) · [测试](#测试)

## 构建

需要 Linux x86_64 / ARM64、CMake ≥ 3.16、C++17 编译器和 Python 3。SDK 必须与运行主机的操作系统和 CPU 架构匹配。

```bash
git submodule update --init --recursive
sudo apt-get install -y build-essential cmake git python3 python3-venv
```

按机型构建，避免引入其他机器人的依赖：

| 命令 | 输出目录 |
|---|---|
| `bash scripts/build_robot.sh agibot_x2` | `build-agibot-x2-only` |
| `bash scripts/build_robot.sh g1` | `build-g1-only` |
| `bash scripts/build_robot.sh k2` | `build-k2-only` |
| `bash scripts/build_robot.sh carm` | `build-cvte-only` |
| `bash scripts/build_robot.sh pm01` | `build-pm01-only` |

手工配置 X2：

```bash
cmake -S . -B build-x2 \
  -DBRIDGE_ROBOT=agibot_x2 \
  -DBUILD_BRIDGE_DDS=OFF \
  -DBUILD_GAMEPAD_SDL=OFF \
  -DBUILD_TESTING=ON
cmake --build build-x2 -j2
ctest --test-dir build-x2 --output-on-failure
```

## 统一接口

```text
配置 → RobotAssemblyFactory
          ├── BridgeCommandIngress → BridgeRunner → RobotSdkAdapter → 厂商 SDK / ROS2
          └── BridgeStateEgress    ←             ←                  ← 设备
```

公开契约：[RobotSdkAdapter](../include/bridge/core/robot_sdk_adapter.hpp)、[数据类型](../include/bridge/core/types.hpp)。

| 接口 | 用途 |
|---|---|
| `initialize()` | 建立会话 |
| `read_state()` | 获取状态并检查有效性 |
| `write_command(CommandFrame)` | 提交适配器支持的关节命令 |
| `query_health()` | 查询连接与健康 |
| `shutdown()` | 关闭会话并执行适配器安全收尾 |

## 配置与运行

复制对应配置模板，修改本地副本，再启动：

```bash
./build-<机型>/adapter_bridge config/<机型>.local.yaml
```

配置解析实现在 [bridge_config.cpp](../src/bridge/config/bridge_config.cpp)。切换机型时使用独立构建目录。

## 新增设备

1. 在 `include/bridge/adapters/<name>` 和 `src/bridge/adapters/<name>` 定义配置与实现。
2. 实现连接、读写、健康和关闭接口，将厂商错误转为结构化结果。
3. 明确关节顺序、单位、超时、QoS 及控制模式。
4. 在 `src/bridge/app/robot_assembly_factory.cpp` 注册并加入 CMake 构建选项。
5. 添加配置、测试和 `docs/<name>/README.md`，在首页添加入口。

设备命令、接口和验证步骤集中维护在对应设备手册，避免复制到多个页面。

## 测试

普通 CTest 运行离线测试：

```bash
cmake --build build-x2 --target agibot_x2_adapter_test
ctest --test-dir build-x2 --output-on-failure
```

X2 真机 ROS2 检查命令和灯光、音频、关节运动方案见 [X2 设备手册](agibot_x2/README.md)。提交只包含源码、配置、正式文档和必要资源，不提交构建目录、缓存和临时日志。
