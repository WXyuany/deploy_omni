# Unitree G1 状态与命令字段

本文档记录当前桥接程序从 Unitree G1 读取哪些状态数据，以及向 G1 下发哪些命令字段。

## G1 回传到桥接程序的状态数据

G1 adapter 会读取 Unitree `LowState_`，并转换成项目内部的 `bridge::core::StateFrame`。

数据路径：

```text
Unitree LowState_
  -> UnitreeG1LowLevelSession::read_state()
  -> LowLevelSessionState
  -> UnitreeG1Adapter::read_state()
  -> bridge::core::StateFrame
  -> DDS LowState_ 发布路径
```

### 帧级元数据

| Bridge 字段 | 来源 |
|---|---|
| `sample_time_ns` | adapter 本地采样时间 |
| `publish_time_ns` | 映射到 `StateFrame` 时与 `sample_time_ns` 相同 |
| `low_state_version` | `LowState_.version()` |
| `low_state_reserve` | `LowState_.reserve()` |
| `tick` | `LowState_.tick()` |
| `mode_pr` | `LowState_.mode_pr()` |
| `mode_machine` | `LowState_.mode_machine()` |
| `valid` | 成功转换后置为 true |
| `stale` | 缓存的 low state 超过超时时间后置为 true |
| `degraded` | 状态过期或降级时置为 true |

### 每个电机的状态数据

对每个硬件电机槽位，adapter 会把 `LowState_.motor_state()[i]` 映射到 `StateFrame::joints`：

| Bridge 关节状态字段 | Unitree 来源字段 |
|---|---|
| `position[i]` | `motor_state()[i].q()` |
| `velocity[i]` | `motor_state()[i].dq()` |
| `effort[i]` | `motor_state()[i].tau_est()` |
| `acceleration[i]` | `motor_state()[i].ddq()` |
| `motor_mode[i]` | `motor_state()[i].mode()` |
| `motor_temperature[i]` | `motor_state()[i].temperature()` |
| `motor_voltage[i]` | `motor_state()[i].vol()` |
| `motor_sensor[i]` | `motor_state()[i].sensor()` |
| `motor_state[i]` | `motor_state()[i].motorstate()` |
| `motor_reserve[i]` | `motor_state()[i].reserve()` |

### 遥控器数据

| Bridge 字段 | 来源 |
|---|---|
| `wireless_remote[40]` | `LowState_.wireless_remote()` |

`wireless_remote[40]` 会原样透传。

### IMU 数据

如果配置了独立 IMU topic，并且已经收到最新 IMU sample，则使用独立 IMU 数据；否则使用 `LowState_.imu_state()`。

| Bridge IMU 字段 | Unitree 来源字段 |
|---|---|
| `orientation_xyzw` | `IMUState_.quaternion()`，从 Unitree 的 `wxyz` 顺序重排为 bridge 的 `xyzw` 顺序 |
| `rpy_xyz` | `IMUState_.rpy()` |
| `angular_velocity_xyz` | `IMUState_.gyroscope()` |
| `linear_acceleration_xyz` | `IMUState_.accelerometer()` |
| `temperature` | `IMUState_.temperature()` |

## 桥接程序下发到 G1 的命令数据

G1 adapter 接收 Unitree `LowCmd_`，转换成项目内部的 `bridge::core::CommandFrame`，经过 joint map 投影后，再打包成硬件侧 `LowCmd_` 下发给机器人。

数据路径：

```text
DDS LowCmd_
  -> translate_lowcmd_to_hardware_command()
  -> bridge::core::CommandFrame
  -> UnitreeG1LowLevelSession::write_command()
  -> build_low_cmd_from_command()
  -> lowcmd_publisher_->Write()
```

### 帧级命令字段

| 下发给机器人的 Unitree 命令字段 | Bridge 来源字段 |
|---|---|
| `LowCmd_.mode_pr()` | `CommandFrame::mode_pr` |
| `LowCmd_.mode_machine()` | `CommandFrame::mode_machine` |
| `LowCmd_.crc()` | 如果 `CommandFrame::low_cmd_crc` 存在则沿用，否则重新计算 |

### 每个电机的命令字段

对每个命令电机槽位，adapter 会把 `CommandFrame::joints` 映射到 `LowCmd_.motor_cmd()[i]`：

| Unitree 电机命令字段 | Bridge 命令来源字段 | 来源向量为空时的默认值 |
|---|---|---|
| `motor_cmd()[i].mode()` | `joints.motor_mode[i]` | `1` |
| `motor_cmd()[i].q()` | `joints.position[i]` | `0.0` |
| `motor_cmd()[i].dq()` | `joints.velocity[i]` | `0.0` |
| `motor_cmd()[i].tau()` | `joints.effort[i]` | `0.0` |
| `motor_cmd()[i].kp()` | `joints.stiffness[i]` | `0.0` |
| `motor_cmd()[i].kd()` | `joints.damping[i]` | `0.0` |

填充命令槽位前，所有 `motor_cmd` 会先被清零。

### 安全命令

`write_safe_command()` 会构造一个安全 `LowCmd_`：

| Unitree 命令字段 | 值 |
|---|---|
| `mode_pr()` | `0` |
| `mode_machine()` | 当前保存的 `mode_machine` |
| 每个 `motor_cmd().mode()` | `0` |
| 每个 `motor_cmd().q()` | `0.0` |
| 每个 `motor_cmd().dq()` | `0.0` |
| 每个 `motor_cmd().tau()` | `0.0` |
| 每个 `motor_cmd().kp()` | `0.0` |
| 每个 `motor_cmd().kd()` | `0.0` |
| `crc()` | 重新计算 |

## 代码位置

- 状态提取：`src/bridge/adapters/unitree_g1/unitree_g1_low_level_session.cpp`
- `StateFrame` 映射：`src/bridge/adapters/unitree_g1/unitree_g1_adapter.cpp`
- 命令转换：`src/bridge/dds/dds_translation.cpp`
- 命令打包：`src/bridge/adapters/unitree_g1/unitree_g1_low_level_protocol.cpp`
- 命令发送：`src/bridge/adapters/unitree_g1/unitree_g1_low_level_session.cpp`
