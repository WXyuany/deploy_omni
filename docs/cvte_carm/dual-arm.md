# D3 双臂：接入与接口

[首页](../../README.md) · [SDK 安装与接口](README.md)

**尚未做 D3 真机测试。** 代码使用同一套 SDK 的 `CArmDualBot`，配置样例按左右各 7 关节、合计 14 关节编写。现场必须读取每侧 `arm_dof` 和关节数组确认。

## 第一步：编译并只读连接

与 A3 共用 SDK 安装和构建，不需要另写驱动：

```bash
BRIDGE_CVTE_SDK_DIR=/opt/cvte/arm_control_sdk bash scripts/build_robot.sh carm
./build-cvte-only/cvte_carm_check --model d3 --ip 10.42.0.101
```

替换实际 D3 IP；期望 `joint_count=14 valid=1 degraded=0`、`connected=1`。
关节数不同会使当前接口检查失败，应先核对型号与配置，不强行截断或补零。

分别读取左右臂：

```bash
source .venv/bin/activate
python examples/cvte_carm/read_state.py --model d3 --ip 10.42.0.101
```

输出 `left` 和 `right`，各含状态、关节位置、末端位姿。此脚本不使能或移动。

## 第二步：调用差异

共同流程仍是连接、状态、ready、模式与伺服、运动、反馈、结束。
**动作目标必须按侧区分**，不能把 A3 的六维向量传给 D3。

```python
# 在已创建并连接的 CArmDualBot 对象 robot 上，仅查询：
left_q = robot.get_left_joint_pos()
right_q = robot.get_right_joint_pos()
left_status = robot.get_left_status()
right_status = robot.get_right_status()
```

| 操作范围 | SDK 方法 |
|---|---|
| 左侧跟随 | `track_left_joint(left_target)` |
| 右侧跟随 | `track_right_joint(right_target)` |
| 两侧跟随 | `track_joint(left_target, right_target)` |
| 左／右点到点 | `move_left_joint` / `move_right_joint` |
| 左／右末端 | `set_left_eeff` / `set_right_eeff` |
| 整机停止 | `emergency_stop` |

参数顺序、工具号、时间及末端单位按配套 `carm_dual.h` / Python SDK 文档填写。
本页面不提供未经验证的双臂自动摆动脚本；先单侧、单关节低速验证，再验证双侧动作，检查互碰范围。

## 通用 adapter 配置

- `cvte_carm_d3`：两侧，通用数组顺序是 `[left_0…left_6, right_0…right_6]`。
- `cvte_carm_d3_left` / `cvte_carm_d3_right`：一侧视图；配置仅保留该侧关节。
- `CvteCarmAdapterConfig.model = Model::kD3Dual`；两侧模式设 `arm_selection = ArmSelection::kBoth`、`arm_joint_count = 7`。
- 单侧模式只切换视图，整机 `set_ready` / `emergency_stop` 仍可能影响两侧，不代表控制器资源完全隔离。

参考：[双臂配置](../../config/cvte_carm_d3.sample.yaml)。位置跟随已封装；夹爪、笛卡尔和运动学仍走厂商接口。

## 现场验收顺序

1. 记录固件、SDK 版本、左右型号、关节数及末端类型。
2. 检查两侧状态、关节限位、急停和控制器连接。
3. 验证左臂单关节、右臂单关节及停止；分别记录目标与反馈。
4. 验证两侧末端执行器，最后再做双臂协同与互碰范围检查。
5. 把实测结果补充到 [验证记录](../validation.md)，再更新首页状态。
