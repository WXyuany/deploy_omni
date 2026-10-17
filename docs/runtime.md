# 配置与运行排错

[首页](../README.md) · [架构](architecture.md)

## 两类入口

- 高层服务／SDK 示例：G1 `unitree_g1_motion_demo`、CArm 只读和运动示例；不需要启动通用桥接器。
- 通用关节转发：`adapter_bridge <config.yaml>`；根据配置装配 command ingress、adapter、state egress。

模板不是完整现场配置。修改本地副本 `config/<name>.local.yaml`，保留示例作为参考。

## 数据方向

| 配置 | 方向 |
|---|---|
| `dds.topics.command` | 上层程序 → bridge 的命令入口 |
| `dds.topics.state` | bridge → 上层程序的状态出口 |
| `adapter.topics.command` | bridge → 机器人底层，G1 使用 |
| `adapter.topics.state` | 机器人底层 → bridge，G1 使用 |
| `adapter.connection.interface_hint` | G1 为网卡；CArm 配置装配使用控制器 IP |
| `joint_mapping.entries` | sim_index 与 hardware_index、名称、符号和偏移 |

`config/g1.sample.yaml` 的上下游 topic 刻意区分：`bridge/rt/*` 与 `custom/mujoco/rt/*`。
真实低层接管需按设备实际 topic、关节槽位和服务释放流程重新检查，不能把高层键盘流程和低层接管同时启动。
K2 与 CArm 的样例兼容旧格式 `joints`；字段解析以 `src/bridge/config/bridge_config.cpp` 为准。

## 运行前检查

1. 机器人型号、SDK 架构和构建目录一致。
2. 网卡／IP、domain、topic、消息类型与发布端一致；ROS topic 与原始 DDS topic 可能有 `rt/` 差异。
3. 关节数量与顺序来自设备配置，夹爪独立；左右臂不能互换。
4. 只读程序有新样本，返回结果与 `valid/stale/degraded` 均正常。
5. 场地和急停就绪后，再启动所需控制程序；避免多个客户端同时下发。

## 常见问题

| 现象 | 定位顺序 |
|---|---|
| 编译成功，运行没有状态 | 网卡 → 发布进程 → domain/topic/type → QoS → 样本计数 |
| G1 的 FSM 查询 7301 | 官方含义是 LocoState 不可用；检查运动服务和状态，不直接认定接口不支持 |
| G1 点云／里程计计数为 0 | 检查 MID360 驱动和 SLAM 发布者；订阅端不会生成传感器数据 |
| CArm 返回空状态 | IP/端口、实际型号、SDK 版本与每侧连接状态 |
| K2 motor_api 报未编译 | 当前实现明确不包含真实后端，不能靠配置开启 |
| 状态可读但动作没发生 | 目标与当前位置、模式／伺服、错误返回、固件限制；不要只看函数调用完成 |
| 通用命令返回 unsupported | 查询能力和 adapter 的支持子集，改用厂商专用接口 |

通用 `adapter_bridge` 的进程终止与机器人实际停车不是同一回事。G1 键盘 demo 的停车策略见专用手册；其他程序按设备急停与关闭流程处理。
