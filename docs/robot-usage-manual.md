# Adapter Any Robot 使用手册

这份手册只描述当前仓库里**实际生效**的构建与启动方式。

- 根程序入口：`adapter_bridge`
- 根构建脚本：`scripts/build_robot.sh`
- 根配置文件示例：
  - `config/kepler_k2.sample.yaml`
  - `config/pm01.sample.yaml`
  - `config/g1.sample.yaml`

> 注意：仓库里有一些历史文档和示例代码仍然提到 `Fast DDS`、`kepler_k2_bridge` 或 standalone `pm01_bridge`。这些内容可以作为背景参考，但**当前仓库主入口仍然是 `adapter_bridge`**。

---

## 1. 支持的机器人

当前主程序 `adapter_bridge` 支持：

- `kepler_k2`
- `pm01`
- `unitree_g1`

机器人注册与装配逻辑在：

- `src/bridge/app/robot_assembly_factory.cpp`

---

## 2. 通用依赖

无论构建哪种机器人，当前根工程都会依赖下面这些库：

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

---

## 4. 编译方法

## 4.1 推荐方式：使用脚本

根工程提供的统一脚本：

```bash
./scripts/build_robot.sh <pm01|kepler_k2|unitree_g1|all> [build-dir]
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

### 同时编译全部机器人

```bash
./scripts/build_robot.sh all
```

默认输出目录：

- `build/`

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

这个脚本最终构建的目标是：

- `adapter_bridge`

---

## 4.2 手动使用 CMake

### 同时编译全部机器人

```bash
cmake -S . -B build \
  -DBUILD_ROBOT_KEPLER_K2=ON \
  -DBUILD_ROBOT_PM01=ON

cmake --build build --target adapter_bridge
```

### 只编译 PM01

```bash
cmake -S . -B build-pm01-only \
  -DBUILD_ROBOT_KEPLER_K2=OFF \
  -DBUILD_ROBOT_PM01=ON

cmake --build build-pm01-only --target adapter_bridge
```

### 只编译 Kepler K2

```bash
cmake -S . -B build-k2-only \
  -DBUILD_ROBOT_KEPLER_K2=ON \
  -DBUILD_ROBOT_PM01=OFF

cmake --build build-k2-only --target adapter_bridge
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
