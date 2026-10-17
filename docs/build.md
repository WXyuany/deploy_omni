# 构建与测试

[返回首页](../README.md)

## 环境

- Linux x86_64 或 aarch64；CMake ≥ 3.16、C++17 编译器、Git、Python 3。
- G1、K2、PM01 的公共 DDS 依赖来自 `third_party/robot_shared` 子模块。
- CArm 默认 SDK-only，不需要共享 DDS 依赖；厂商 SDK 的 CPU 架构必须与运行主机一致。
- SDK 是 Linux 二进制。Apple Silicon 的 ARM64 与 Linux ARM64 不是同一个系统 ABI。

```bash
git submodule update --init --recursive
sudo apt-get install -y build-essential cmake git python3 python3-venv
```

## 一条命令选机型

| 命令 | 输出目录 | 主要目标 |
|---|---|---|
| `bash scripts/build_robot.sh g1` | `build-g1-only` | 键盘运控、状态工具、adapter_bridge |
| `bash scripts/build_robot.sh k2` | `build-k2-only` | adapter_bridge，K2 模拟实现 |
| `bash scripts/build_robot.sh carm` | `build-cvte-only` | cvte_carm_sdk_smoke，单臂／双臂共用 |
| `bash scripts/build_robot.sh pm01` | `build-pm01-only` | adapter_bridge，须准备 ROS2 依赖 |

脚本的第二个参数可指定构建目录。CArm 通过环境变量 `BRIDGE_CVTE_SDK_DIR` 指定 SDK；默认寻找根目录 `arm_control_sdk`。`BUILD_JOBS` 控制并行数。

## 手工 CMake

G1 示例；不混用不同机器人构建缓存：

```bash
cmake -S . -B build-g1 \
  -DBRIDGE_ROBOT=unitree_g1 \
  -DBUILD_GAMEPAD_SDL=OFF \
  -DBUILD_UNITREE_G1_MOTION_DEMO=ON \
  -DBUILD_TESTING=ON
cmake --build build-g1 -j2
ctest --test-dir build-g1 --output-on-failure
```

| 选项 | 用途 |
|---|---|
| `BRIDGE_ROBOT` | `unitree_g1`、`kepler_k2`、`cvte_carm`、`pm01`；必须选择 |
| `BUILD_BRIDGE_DDS` | CArm 默认为 OFF，其他机器人为 ON |
| `BRIDGE_CVTE_SDK_DIR` | CVTE SDK 根目录，含 include、lib |
| `BUILD_UNITREE_G1_MOTION_DEMO` | 生成 G1 高层键盘程序，默认 OFF |
| `BUILD_UNITREE_G1_HARDWARE_TESTS` | 生成历史硬件诊断程序，默认 OFF；不会自动运行 |
| `BUILD_GAMEPAD_SDL` | SDL 手柄支持；需要 `libsdl2-dev`；简易构建默认关闭 |
| `BUILD_TESTING` | 生成离线测试；运行 CTest 不执行硬件烟测 |

`all` 为历史兼容集合，仅包括 K2、PM01、G1，不包括 CArm；通常应按机型构建。
G1 的 `--check` / `--dry-run` / `--enable-motion` 不依赖 SDL，也不需要启动 `adapter_bridge`。

## macOS 与 Docker

使用 Linux 容器编译、运行模拟测试。首次下载镜像和安装工具需要联网：

```bash
docker run --rm -it \
  -v "$PWD:/workspace" -w /workspace ubuntu:22.04 bash
```

容器内：

```bash
apt-get update
apt-get install -y build-essential cmake git python3 python3-venv
bash scripts/build_robot.sh g1
./build-g1-only/unitree_g1_motion_demo --dry-run
```

Apple Silicon 默认拉取 ARM64 镜像，Intel 默认 x86_64。容器内编译产物留在挂载的项目目录。
CArm SDK 如果在项目外，需要额外挂载 SDK 目录并传入容器内的路径。
Docker Desktop 的 DDS 组播网络不等同于机器人直连网卡；真实 G1 通信优先在机器人或 Linux 控制电脑运行。
发布到机器人时重新编译，避免跨 Ubuntu/glibc 版本直接复制二进制。

## 常见失败

| 现象 | 处理 |
|---|---|
| 缺少 libddsc.so / libunitree_sdk2.a | 初始化子模块，确认 CPU 架构与 SDK 目录 |
| 缺少 CArm SDK | 设置 `BRIDGE_CVTE_SDK_DIR`，检查 `lib/cmake/arm_control_sdk` |
| wrong ELF class / file in wrong format | 使用相同 OS、CPU 架构的 SDK |
| 更换机器人后缓存冲突 | 新建独立构建目录，重新选择 `BRIDGE_ROBOT` |
| SDL 找不到 | 键盘示例用 `BUILD_GAMEPAD_SDL=OFF`；手柄另装 `libsdl2-dev` |
| Python 提示没有 carm | 激活虚拟环境并安装官方 SDK 包，见 CArm 文档 |
