# G1 雷达点云与里程计

[返回 G1](README.md) · [SDK 接口](sdk.md)

本项目通过 `UnitreeG1SensorSession` 订阅点云和里程计，提供独立读取程序与 C++ 接口。

## 1. 启动数据来源

- **点云**：安装并启动 MID360 驱动，使其发布 `PointCloud2` 数据。
- **里程计**：启动使用雷达数据的 SLAM／定位服务，使其发布 `Odometry` 数据。
- 读取程序与发布者使用相同 DDS domain（本程序为 0）、网卡、topic 和消息类型。

仅安装驱动不会自动产生 SLAM 里程计。程序负责读取数据，不启动驱动、建图或导航动作。
驱动的原生 Livox 自定义消息需要发布或转换为 `sensor_msgs/PointCloud2` 后才能使用本接口。

## 2. 编译并读取

```bash
bash scripts/build_robot.sh g1

# 点云：默认读取 10 秒，每秒打印最新一帧摘要
./build-g1-only/unitree_g1_sensor_reader --interface eth0

# 里程计
./build-g1-only/unitree_g1_sensor_reader --interface eth0 --stream odometry

# 同时读取点云和里程计，持续 60 秒
./build-g1-only/unitree_g1_sensor_reader --interface eth0 --stream both --duration 60
```

`eth0` 换成连接机器人的实际网卡。Ctrl+C 结束读取，不改变机器人运动状态。

默认接口：

| 数据 | 原始 DDS topic | 消息类型 |
|---|---|---|
| 点云 | `rt/rslidar_points` | `sensor_msgs::msg::dds_::PointCloud2_` |
| SLAM 里程计 | `rt/lio_sam_ros2/mapping/re_location_odometry` | `nav_msgs::msg::dds_::Odometry_` |

驱动配置不同可直接改 topic；下例仅演示参数，名称须以实际发布者为准：

```bash
./build-g1-only/unitree_g1_sensor_reader \
  --interface eth0 --stream both \
  --cloud-topic rt/livox/lidar \
  --odom-topic rt/odom --duration 30
```

ROS2 中显示的 `/xxx` 在默认 ROS DDS 命名映射下通常对应 `rt/xxx`，不要直接照抄 ROS 话题名而忽略 DDS 前缀。

## 3. 看懂输出

点云输出包含：累计接收帧数、坐标系、宽高、点／行字节跨度、字节序、数据长度，以及 `字段名:偏移:类型:数量`。
里程计输出包含：累计接收帧数、父／子坐标系、位置 XYZ 和四元数 XYZW。

`samples` 是累计帧数，不是频率。程序显示的是最新快照，帧数不再增加表示没有新数据。
退出码：`0` 表示选中的数据流均收到过消息，`2` 表示至少一个选中数据流没有消息，`1` 表示运行错误，`64` 表示参数错误。
收到消息不等于定位精度或点云内容已验收。

## 4. C++ 调用

头文件：[unitree_g1_sensor_session.hpp](../../include/bridge/adapters/unitree_g1/unitree_g1_sensor_session.hpp)。
自定义 CMake 目标链接 `bridge_adapter_unitree_g1`。不需要初始化低层运动 adapter。

```cpp
#include "bridge/adapters/unitree_g1/unitree_g1_sensor_session.hpp"

using namespace bridge::adapters::unitree_g1;
UnitreeG1SensorSessionConfig config;
config.transport_interface = "eth0";
config.pointcloud_topic = "rt/rslidar_points";
config.odometry_topic = "rt/lio_sam_ros2/mapping/re_location_odometry";
UnitreeG1SensorSession sensors(config);
if (!sensors.initialize()) return;

// 在业务循环中调用；数据通过后台 DDS 回调更新。
const auto snapshot = sensors.snapshot();
if (snapshot.pointcloud) {
  const auto& cloud = *snapshot.pointcloud;
  // cloud.header(): 时间戳、坐标系
  // cloud.fields(): 字段名、偏移、数据类型和数量
  // cloud.width()/height()/point_step()/row_step(): 布局
  // cloud.is_bigendian(): 字节序
  // cloud.data(): 原始点云字节数组
}
if (snapshot.odometry) {
  const auto& odom = *snapshot.odometry;
  const auto& xyz = odom.pose().pose().position();
  const auto& rotation = odom.pose().pose().orientation();
  // xyz.x()/y()/z(); rotation.x()/y()/z()/w()
  // odom.twist(): 速度及协方差；odom.header(): 时间戳、坐标系
}
sensors.shutdown();  // 在业务循环结束后调用
```

`initialize()` 建立订阅；`snapshot()` 返回线程安全的数据副本，刚启动时 optional 可以为空。
解析 XYZ、强度等字段时按 `fields` 的偏移和类型、点／行跨度及字节序读取，不假定每个点固定为三个 float。
多个组件共用 SDK 的 `ChannelFactory` 单例时，应使用相同网卡和 domain。
