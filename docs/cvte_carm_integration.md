# CVTE CArm 接入准备手册

本文档用于两款 CVTE 机械臂接入前的技术准备和现场测试。它记录了截至 2026-10-14 从官方仓库和公开页面确认的内容；具体机械臂型号、控制器 IP、SDK 压缩包版本和相机配置，必须在设备到场后再次确认。

## 官方资料

- [carm_demo](https://github.com/cvte-robotics/carm_demo)：C++、Python、ROS1、ROS2 SDK 使用示例。
- [cvte-robotics 组织仓库](https://github.com/cvte-robotics)：包含 `pycarm`、`carm_tool`、`carm_grasp`、`carm-lerobot`、`carm-openclaw` 等相关项目。
- [carm_grasp](https://github.com/cvte-robotics/carm_grasp)：基于眼在手 RGB-D 相机的标定和抓取范例。
- [CArm 用户文档中心](https://cvte-bot.feishu.cn/wiki/GyGbwKeWMiqEfDk80QXc9zeCnjf)：SDK 下载、产品手册和控制器说明的官方入口。
- [CVTE 智能柔性机械臂](https://www.cvte.com/product/flexiblemanipulator)：官网产品信息。当前页面显示 A3 产品亮点为 700 mm 臂展、4 kg 最大负载、5.5 kg 自重和力位混合控制。

本次访问飞书文档中心成功到达站点，但被重定向到飞书登录页；当前环境没有 CVTE 飞书登录态，因此无法读取文档正文或下载 SDK。不能据此推断文档内容缺失。明天在已登录的浏览器或公司网络中打开该页面，下载与实际控制器匹配的 SDK 压缩包，并以压缩包内头文件和版本号为准。

当前工作区已放入一份 amd64 SDK：版本 `1.0.1`，包含 `CArmSingleCol`、`CArmDualBot`、`libarm_control_sdk.so`、`libmlog.so` 和 CMake package。该二进制包不纳入源码版本管理；构建时通过 `BRIDGE_CVTE_SDK_DIR` 指向它。

## 已确认的 SDK 结构

### 三种调用层

| 层 | 官方实现 | 适用场景 | 本项目接入建议 |
| --- | --- | --- | --- |
| C++ SDK | `arm_control_sdk`，头文件为 `carm_cobot.h`、`carm_dual.h`、`data_type_def.h` | 实时控制、已有 C++ 工程 | 作为 CArm adapter 的唯一厂商控制依赖 |
| Python SDK | `carm`，基于 WebSocket，`pip install carm` | 快速验证、脚本、标定和视觉实验 | 用于现场原型和 `carm_grasp`，不直接进入核心 C++ 闭环 |
| ROS1/ROS2 | `carm_ros`、`carm_ros2` | 话题化控制和视觉生态 | 仅在现场 ROS 版本和话题确认后启用 |

官方 SDK 通过网络与机械臂控制器通信，不是 DDS 接口。不要把 CArm 控制器当作 Unitree DDS 设备处理，也不要把 CArm 的控制消息直接塞进现有 Unitree 数据类型。

### 单臂、双臂和上半身

- `CArmSingleCol`：一个控制器只管理一个机械臂的单臂套装。
- `CArmDualBot`：一个控制器同时管理左右两臂的人型双臂套装。
- 两个独立单臂即使组成“双臂”，仍然使用两个 `CArmSingleCol`，通过两个控制器 IP 区分。
- `pycarm` 还导出了 `CArmBust`，用于左右臂加两自由度腰部的上半身设备；是否适用必须以实际型号和 SDK 头文件为准。
- 机械臂自由度是动态配置，不能在通用层写死为 6、7 或某个固定数量。

## C++ SDK 使用方式

官方 C++ SDK 是预编译包，目录中包含 `include`、`lib`、CMake package 和 `setup.bash`。在控制电脑上：

```bash
unzip arm_control_sdk.<arch>.<version>.zip
source arm_control_sdk/setup.bash
```

最小 CMake 依赖形式如下：

```cmake
find_package(arm_control_sdk REQUIRED)
add_executable(carm_probe carm_probe.cpp)
target_include_directories(carm_probe PRIVATE ${arm_control_sdk_INCLUDE_DIRS})
target_link_libraries(carm_probe PRIVATE ${arm_control_sdk_LIBRARIES})
```

官方示例位于 `carm_demo/cpp_test_demo`，编译方式为：

```bash
cd cpp_test_demo
mkdir build && cd build
cmake ..
make -j
./carm_demo
```

本项目启用 CArm adapter 的构建方式：

```bash
cmake -S . -B build-cvte \
  -DBRIDGE_ROBOT=cvte_carm \
  -DBUILD_BRIDGE_DDS=OFF \
  -DBUILD_TESTING=OFF \
  -DBRIDGE_CVTE_SDK_DIR="$PWD/arm_control_sdk"
cmake --build build-cvte --target cvte_carm_sdk_smoke
```

`BRIDGE_ROBOT=cvte_carm` 是单独的构建选择，不要与其他 `BUILD_ROBOT_*` 开关混用。默认关闭 `BUILD_BRIDGE_DDS`，因此不会要求 Unitree SDK；只有明确需要公共 `adapter_bridge` 时才打开它。若使用旧脚本或旧缓存，先删除对应构建目录；兼容旧开关时可临时使用 `BRIDGE_ROBOT=legacy`。

在 amd64 控制电脑上运行只读烟测：

```bash
./build-cvte/cvte_carm_sdk_smoke --model a3 --ip <A3控制器IP>
./build-cvte/cvte_carm_sdk_smoke --model d3 --ip <D3控制器IP>
```

烟测只连接、读取状态、查询健康状态并断开，不调用 `set_ready`、伺服使能或运动接口。

现场先把 `test_carm_api.cpp` 当作连通性和 API 参考，不要直接运行会移动机械臂的示例。示例默认控制器地址为 `10.42.0.101`，实际地址以现场配置为准。

## API 能力和安全调用顺序

`CArmSingleCol` 和 `CArmDualBot` 覆盖的能力可以按以下边界理解：

1. 连接：`connect`、`disconnect`、`is_connected`。
2. 基础控制：`set_ready`、`set_servo_enable`、`set_control_mode`、`emergency_stop`。
3. 状态：版本、配置、状态、关节位置/速度/力矩、末端笛卡尔位姿、外力矩、规划目标值。
4. 末端：末端类型、自由度、连接状态、位置/速度/力矩和 `set_eeff`。旧的 `gripper`/`hand` 方法已经标记为 deprecated，应优先使用 `eeff` 接口。
5. 运动：`track_joint`、`track_pose`、`move_joint`、`move_pose`、直线运动和轨迹运动。
6. 配置：速度等级、工具号、工具坐标、碰撞配置。
7. 运动学：正解、逆解及数组版本。
8. 示教和回调：轨迹录制/检查、错误回调、完成回调、关节/位姿/规划值/外力回调。

首轮测试严格按以下顺序执行：

```text
connect
  -> is_connected / get_version / get_config / get_status
  -> set_ready
  -> set_control_mode（先使用现场确认的安全模式）
  -> 读取关节和末端状态
  -> 确认急停、保护停止和安全范围后才 set_servo_enable(true)
  -> 低速、短距离的单个运动验证
  -> 末端执行器验证
  -> emergency_stop 验证
  -> set_servo_enable(false) / disconnect
```

所有命令返回值、错误回调和完成回调都要记录。Python 包的命令类方法约定 `1` 为成功、`<1` 为失败；C++ 具体返回值和枚举必须以现场 SDK 头文件为准，不要跨语言猜测返回码含义。

## Python 和“小龙虾”相关仓库

### `pycarm`

`pycarm` 提供与 C++ 对齐的 `CArmSingleCol`、`CArmDualBot` 和 `CArmBust`。最小只读探测可以写成：

```python
from carm import CArmSingleCol

arm = CArmSingleCol("10.42.0.101")
if not arm.is_connected():
    raise RuntimeError("CArm controller is unreachable")

print(arm.get_joint_pos())
print(arm.get_cart_pose())
print(arm.get_status())
arm.disconnect()
```

`carm-openclaw` 是把 Python SDK 包装成 OpenClaw skill 的分发仓库，本身不是新的机器人协议。它适合验证“上层工具调用机械臂”的链路，底层仍然是 `pycarm` 和控制器 WebSocket。

### `carm_tool`

`carm_tool` 是独立末端工具驱动集合，包含夹爪、手柄和电动螺丝刀等示例，使用 CMake 编译并通过 `setup.bash` 声明环境。它应作为末端工具能力单独接入，不要与机械臂本体的关节状态混为一个固定接口。

## ROS 接口准备

官方 `carm_ros`/`carm_ros2` 示例把 C++ SDK 的部分能力映射成话题。ROS2 示例中已确认的接口包括：

| 方向 | 话题 | 典型消息 |
| --- | --- | --- |
| 订阅 | `connect`、`ready`、`emergency_stop` | `std_msgs/ String`、`std_msgs/ Bool` |
| 订阅 | `move_joint`、`move_line_joint`、`move_tracking_joint` | `sensor_msgs/ JointState` |
| 订阅 | `move_pose`、`move_line_pose`、`move_tracking_pose` | `geometry_msgs/ Pose` |
| 订阅 | `set_control_mode`、`set_servo_enable`、`set_speed_level`、`set_collision_config` | `std_msgs` 系列 |
| 订阅 | `set_gripper` | `sensor_msgs/ JointState` |
| 发布 | `real_joint_state` | `sensor_msgs/ JointState` |
| 发布 | `flange_cart_state` | `geometry_msgs/ PoseStamped` |
| 发布 | `arm_state`、`task_completion`、`carm_error` | `std_msgs` 系列 |

这些是官方 demo 的默认相对话题名，消息队列、命名空间、实际关节名和是否启用夹爪，必须用现场 launch 文件和 `ros2 topic list/info/echo` 复核。ROS 节点示例会在构造时调用 `set_ready`，所以不能未经审查直接用于生产启动。

## 眼在手抓取准备

`carm_grasp` 使用 ROS2、CARM 和 RGB-D 相机，官方示例覆盖 Intel RealSense D405 与 Orbbec Gemini 305：

| 相机 | README 参考彩色话题 | README 参考深度话题 |
| --- | --- | --- |
| D405 | `/realsense/d405/color/image_rect_raw` | `/realsense/d405/aligned_depth_to_color/image_raw` |
| Gemini 305 | `/gemini305/color/image_raw` | `/gemini305/depth/image_raw` |

实际部署前执行：

```bash
source /opt/ros/<ros2-distro>/setup.bash
export ROS_DOMAIN_ID=1  # 以现场网络规划为准
ros2 topic list | grep -E 'color|depth|camera_info'
ros2 topic info <color_topic>
ros2 topic info <depth_topic>
```

抓取前置条件：相机固定在末端、相机 Z 轴与末端 Z 轴夹角控制在文档建议范围内、准备 AprilTag 标定板和夹爪标定工具。完整流程是：

1. 录制相机标定轨迹，采集标定板图像，生成相机内参 `cam_params.json`。
2. 录制始终保持标定板在视野内且平移/旋转充分的轨迹，同时采集图像和末端位姿。
3. 求解 `AX=XB`，生成末端到相机的 `calib_handeye.json`，再用验证脚本检查 TF 和点云对齐。
4. 3D 抓取时额外标定夹爪几何模型。
5. 最后才创建和测试 2D/3D 抓取模板。

`carm_grasp` 的 `CamNode` 使用 `message_filters.ApproximateTimeSynchronizer` 同步彩色和深度图，默认队列深度为 1、时间容差约 50 ms，并支持 `SYSTEM_DEFAULT`、`RELIABLE`、`BEST_EFFORT` QoS。相机数据接不通时，先检查 QoS 和实际话题，再检查 Open3D 点云转换。

## 在本项目中的接入边界

本项目将 CArm 隔离在 `bridge/adapters/cvte_carm/`，对外只暴露本项目自己的接口：

- `CvteCarmAdapter`：封装当前已接入的连接、状态、位置跟随和安全控制，不把 CArm SDK 对象泄漏到核心层。
- CArm 专有能力 facade：后续按现场需要封装末端、运动学、示教和回调；厂商头文件只出现在 adapter 内部。
- `CvteCarmSensorSession`：若现场使用 ROS2 相机或其他传感器，独立订阅图像、深度、CameraInfo 和点云；图像/点云不塞入通用关节 `StateFrame`。
- `CvteCarmConfig`：保存 IP、端口、arm index、关节数量、控制模式、工具和话题配置，不把 `10.42.0.101` 写死在代码里。

当前已实现 `CvteCarmAdapter`，工厂注册了以下配置类型：

- `cvte_carm_a3`：单臂 A3，内部使用 `CArmSingleCol`。
- `cvte_carm_d3`：双臂 D3，内部使用 `CArmDualBot`，状态按左臂关节后右臂关节拼接。
- `cvte_carm_d3_left` / `cvte_carm_d3_right`：只转发 D3 的一侧。

当前 adapter 已接入关节状态、位置向量到 CArm `track_joint` 的映射，以及 `set_ready`、伺服、控制模式和急停等显式安全入口；速度、力矩、刚度和阻尼命令会返回 `kUnsupported`。末端、笛卡尔、运动学、示教和轨迹等 SDK 专有能力仍应通过厂商扩展 facade 逐项实现，并在完成后再加入 capability 声明。

接口设计上应使用能力和配置驱动：单臂/双臂/上半身分别实现对应能力，动态读取关节数量和末端自由度，统一返回结构化错误。运动命令和传感器订阅分开生命周期管理，便于在没有相机或没有 ROS 的控制环境中运行本体 SDK。
