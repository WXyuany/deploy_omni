# Unitree G1

[首页](../../README.md) · [运控手册](high-level-motion.md) · [SDK 索引](sdk.md) · [低层字段](state-and-command-fields.md)

## 1. 编译

在 Linux 项目目录：

```bash
bash scripts/build_robot.sh g1
```

高层键盘程序、状态工具位于 `build-g1-only/`。默认不启动机器人、不发送运动。

## 2. 接通网络，只读检查

机器人或 Linux 控制电脑的网卡应能访问机器人所在网络。用 `ip -br address` 确认网卡名；下文用 `eth0` 举例。

```bash
./build-g1-only/unitree_g1_motion_demo --check --interface eth0
```

期望 `GetFsmId code=0` 和 `LowState fresh/CRC/health checks: PASS`。
FSM 不是 `LowState.mode_machine`；读取失败先排查网卡、DDS domain、机器人服务，不反复发送动作。

## 3. 人工运控

先离线熟悉按键：

```bash
./build-g1-only/unitree_g1_motion_demo --dry-run
```

连接机器人后，按 [运控手册](high-level-motion.md) 的状态转换顺序操作：

```bash
./build-g1-only/unitree_g1_motion_demo --enable-motion --interface eth0
```

动作均为单个小写字母；WASD 每次需 `r` 解锁，只执行 0.5 秒低速脉冲。
高层 demo 不释放官方运动服务。**不要同时启动默认低层 `adapter_bridge`**，它的低层接管会释放高层所需服务。

## 4. 关节、IMU 与 DDS

`rt/lowstate` 包含关节与 IMU，具体映射见 [字段说明](state-and-command-fields.md)。
`lowstate_frequency_receiver` 读取配置的 `dds.topics.state`；`lowstate_field_recorder --help` 查看记录前后 topic 的选项。
配置模板 `config/g1.sample.yaml` 面向低层模拟 topic，不是可直接启动的真实机器人控制配置。

如需观察真实 `rt/lowstate` 频率，复制模板，将其中 `dds.topics.state.name` 改成 `rt/lowstate`，
将 `adapter.connection.interface_hint` 改成实际网卡，再只运行接收工具：

```bash
cp config/g1.sample.yaml config/g1.local.yaml
# 按上面的说明编辑 config/g1.local.yaml
./build-g1-only/lowstate_frequency_receiver config/g1.local.yaml
```

## 5. 音频、灯光、前置相机

使用独立 `UnitreeG1SdkFacade`，先初始化 DDS，再 `sdk.initialize()`；完整签名见 [SDK 指南](sdk.md)。

| 用途 | 方法 | 结果检查 |
|---|---|---|
| TTS | `tts(text, speaker_id)` | `succeeded()` / code |
| 音量 | `get_volume / set_volume` | 保存原值，按需恢复 |
| RGB 灯 | `set_rgb_led(r,g,b)` | code；观察实际颜色 |
| 相机 | `get_front_camera_jpeg(&bytes)` | code 与非空 JPEG 数据 |
| 模式查询 | `check_motion_mode / get_fsm_id` | code；不主动释放模式 |

当前前置相机接口返回 JPEG，不提供深度图、相机内参或 RealSense 原始帧。
厂商成功码为 `0`，但不代表运动已经结束。

## 6. 点云与里程计

`UnitreeG1SensorSession` 只订阅，默认 topic：

| 数据 | topic |
|---|---|
| 点云 | `rt/rslidar_points` |
| SLAM 里程计 | `rt/lio_sam_ros2/mapping/re_location_odometry` |

传感器硬件存在不等于驱动和 SLAM 发布者正在运行。当前设备此前未安装／运行 MID360 驱动，未取得点云与 SLAM 里程计；不将空快照标为通过。
`initialize()` 只创建 reader，实际以 `pointcloud_samples`、`odometry_samples` 与 optional 数据判断。
驱动安装及 SLAM 导航控制不由本项目自动执行。
