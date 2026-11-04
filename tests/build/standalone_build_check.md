# Standalone build check

`adapter_bridge` is currently wired as a thin process entrypoint that loads startup config once, freezes runtime values, selects a registered robot factory, and then assembles these existing pieces:

- `bridge::config::load_bridge_config()`
- `bridge::app::create_robot_assembly()`
- `bridge::core::BridgeRunner`
- `idl/bridge/BridgeMessages.idl` and generated Fast DDS artifacts in the same folder
- `bridge::dds::create_dds_assembly()`
- `bridge::dds::make_dds_runtime()`
- `bridge::adapters::kepler_k2::KeplerK2Adapter`
- `bridge::config::JointMap`

The app now reads `config/kepler_k2.sample.yaml` by default, validates the current sample schema locally inside `bridge_config`, freezes runner, adapter, and joint-map subsets, and lets the app factory map `adapter.kind` to the correct runtime assembly before wiring begins.

The current repository state now provides a hand-written DDS facade/runtime layer. `RobotAssemblyFactory` obtains a DDS assembly from `bridge::dds::create_dds_assembly()`, and that factory owns:

- generated Fast DDS type support registration
- participant / publisher / subscriber ownership via `DdsRuntime`
- a command ingress that translates generated `LowCmd` messages into runner-facing `CommandFrame`
- a state egress that projects runner state into generated `LowState` / `BridgeStatus` publications

This intentionally keeps process-level wiring thin: the app loads config, selects the robot assembly factory, and chooses an exit code, while `BridgeRunner` owns initialization, the periodic read/publish cycle, startup safe-idle behavior, stop request handling, and shutdown. DDS generated types stay in `bridge/dds`, not in `bridge/core`.

Build verification command:

`cmake -S . -B build -DBRIDGE_ROBOT=kepler_k2 && cmake --build build --target adapter_bridge -j && ctest --test-dir build --output-on-failure`

## `scripts/build_robot.sh` 使用说明

这个脚本用于快速配置并编译通用入口程序 `adapter_bridge`，避免每次手写较长的 CMake 命令。

### 命令格式

`./scripts/build_robot.sh <pm01|kepler_k2|unitree_g1|cvte_carm|all> [build-dir]`

- 第一个参数表示要启用的机器人类型：
  - `pm01`：只编译 PM01 相关适配器
  - `kepler_k2` 或 `k2`：只编译 Kepler K2 相关适配器
  - `unitree_g1` 或 `g1`：只编译 Unitree G1 相关适配器
  - `cvte_carm` 或 `carm`：只编译 CVTE CArm 适配器和 SDK 烟测
  - `all`：编译 K2、PM01 和 Unitree G1；CArm 需要单独选择
- 第二个参数可选，用于指定构建目录。

### 默认构建目录

- `./scripts/build_robot.sh pm01` → `build-pm01-only`
- `./scripts/build_robot.sh kepler_k2` → `build-k2-only`
- `./scripts/build_robot.sh unitree_g1` → `build-g1-only`
- `./scripts/build_robot.sh cvte_carm` → `build-cvte-only`
- `./scripts/build_robot.sh all` → `build`

### 常用示例

只编译 PM01：

`./scripts/build_robot.sh pm01`

只编译 Kepler K2：

`./scripts/build_robot.sh kepler_k2`

或：

`./scripts/build_robot.sh k2`

同时编译两个机器人：

`./scripts/build_robot.sh all`

指定自定义构建目录：

`./scripts/build_robot.sh pm01 my-build-dir`

### 脚本实际执行的操作

脚本内部会执行两步：

1. 调用 `cmake -S ... -B ...` 配置构建目录，并设置 `BRIDGE_ROBOT`。
2. 调用 `cmake --build ...`，通常只编译 `adapter_bridge`；CArm SDK-only 模式只编译 `cvte_carm_sdk_smoke`。
