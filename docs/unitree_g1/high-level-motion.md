# G1 高层运控操作手册

本文对应本项目内置 SDK2 的 G1 `LocoClient`

## 1. 入口与边界

- 终端程序：[`examples/unitree_g1/unitree_g1_motion_demo.cpp`](../../examples/unitree_g1/unitree_g1_motion_demo.cpp)。
- 策略层：[`examples/unitree_g1/unitree_g1_motion_control.hpp`](../../examples/unitree_g1/unitree_g1_motion_control.hpp)。
- 自动化测试：[`tests/demo/unitree_g1_motion_control_test.cpp`](../../tests/demo/unitree_g1_motion_control_test.cpp)。
- 终端集成测试：[`tests/demo/unitree_g1_motion_terminal_test.py`](../../tests/demo/unitree_g1_motion_terminal_test.py)，只启动模拟后端。
- 默认 `--check`：只查询运动模式、FSM，校验 LowState。
- `--dry-run`：模拟后端，不创建 DDS participant，不连接机器人。
- `--enable-motion`：人工键盘控制；启动本身不发送运动命令。

高层使用 `sport` 服务，由官方控制器完成关节控制。程序只订阅 `rt/lowstate`，不发布低层关节命令，
不调用 `ReleaseMode` 或自动选择运动模式，不启动/停止系统服务。
不要同时运行低层 `adapter_bridge`、官方低层例程或其他运控客户端。
进程锁只防止同一主机重复运行本 demo，不能检测其他电脑或其他程序的控制指令。

## 2. 编译

需要 **Linux ARM64 或 Linux x86_64**。macOS 即使是 ARM 芯片也不能直接链接仓库中的 Linux SDK。
SDK 头文件与库必须配套使用，不混用机器人 `/usr/local` 中较旧的 SDK。

在完整项目目录下执行，构建过程不会运行机器人：

```bash
cmake -S . -B build-g1-motion \
  -DBRIDGE_ROBOT=unitree_g1 \
  -DBUILD_UNITREE_G1_MOTION_DEMO=ON \
  -DBUILD_GAMEPAD_SDL=OFF \
  -DBUILD_TESTING=ON
cmake --build build-g1-motion \
  --target unitree_g1_motion_demo unitree_g1_motion_control_test -j2
ctest --test-dir build-g1-motion -R '^unitree_g1_motion_(control|terminal)_test$' --output-on-failure
```

仅生成 G1 所需目标。保留项目内的 SDK 动态库路径，建议在机器人/上位机本地重新编译，
不要直接运行从新版本 Linux 容器复制到旧 Ubuntu 的二进制，避免 glibc 不兼容。
检测到 Python 3 时，CMake 同时注册终端集成测试；没有 Python 时仅运行 C++ 策略测试。

## 3. 先演练，再只读检查

离线 Linux 终端：

```bash
./build-g1-motion/unitree_g1_motion_demo --dry-run
```

macOS 的容器构建和运行方式见 [Docker 构建](../build.md#macos-与-docker)。

每次只按一个键，不需要回车。模拟初始 FSM 为 0；任何输出中的 `DRY RUN` 都是模拟，
不能用于证明机器人执行过动作。动作键统一使用小写字母，短按一次直接请求动作，不需要 Shift、数字键或 `!`。

按 [SSH 登录说明](README.md#1-ssh-登录机器人) 连接机器人：网线直连使用 `ssh unitree@192.168.123.164`（密码 `123`），连接 `MEETINGROOM_WIFI` 时使用 `ssh unitree@172.16.201.79`。

登录后，在机器人上的项目目录执行：

```bash
./build-g1-motion/unitree_g1_motion_demo --check --interface eth0
```

机器人端是 `eth0`；Linux 上位机运行时换成连接机器人的实际网卡。
程序使用 DDS domain 0，最多等待约 2 秒获取健康状态。检查输出应有：

- `CheckMode code=0` 且非空模式名称；
- `GetFsmId code=0`；
- `LowState fresh/CRC/health checks: PASS`。

仅在 FSM=500 时查询平衡模式；其他 FSM 下输出 `GetBalanceMode code=-1` 表示没有发出该查询。
`7301` 的准确含义是 **LocoState not available**，不能直接判断为“固件不支持”。
`LowState.mode_machine` 也不能当作高层 FSM 使用。

其它诊断码：`7302` 为无效 FSM ID，`7303` 为无效 task ID。
本 demo 的 `-10003` 表示已收到退出信号、拒绝新动作，`-10004` 表示查询期间有新终端输入、拒绝新动作；
这些负值由 demo 本地生成，不是机器人错误码。不要把非零回复自动重试成运动命令。

## 4. 人工运控步骤

首次调试按厂商要求使用防跌倒吊架/保护装置，清空运动范围，现场备好原厂遥控器。
零力矩和阻尼都不保证站立支撑，不能依靠人员站在机器人旁边徒手接住倒落的机器人。
本 demo 没有障碍感知，MID360 是否安装驱动不影响命令发送，也不提供避障。

```bash
./build-g1-motion/unitree_g1_motion_demo --enable-motion --interface eth0
```

程序启动后不自动改变姿态。下面是本 demo 采用的保守操作流程，不声称所有固件都接受相同转换：

1. 按 `i` 查看状态。若当前 FSM=0，在可靠支撑下按 `p` 请求阻尼。
2. 再按 `i` 确认 FSM=1。按 `u` 请求起立。
3. **等待并目视确认起立完成**，按 `i` 查看 FSM=4。按 `t` 请求 FSM=500。
4. 等待站稳，按 `i` 确认 FSM=500。按 `b` 请求平衡模式 0。
5. 等待站稳，按 `i` 确认 FSM=500、balance=0。按 `r` 表示操作者确认站稳，解锁一次移动。
6. 短按 `w`，发送 0.5 秒、0.10 m/s 前进请求，结束后自动发送 `StopMove` 并重新锁定。
7. 确认停止，再按 `r` 解锁下一次 `s/a/d/j/l`。长按不会把本次脉冲续成长时间运动。

如果进入时已经处于 FSM=500 且平衡模式为 0，可以从步骤 5 开始，不要为了测试先切到零力矩。
若姿态不适合上述转换、查询结果不符或 API 非零返回，停止这一步，使用原厂操作流程恢复；
不要通过反复发送命令或篡改状态编号绕过限制。

每个姿态请求后有 3 秒操作间隔；这只是防止连续误操作，**并非动作完成判据**。
除了 `p/z`，姿态请求还检查实际 FSM、LowState 和间隔；切换过程中不能同时行走。

## 5. 按键和官方接口对照

所有姿态/手臂键均为单个小写字母，短按一次直接请求动作；状态与操作间隔检查通过后才发送。旧数字键、大写动作键和 `!` 不再绑定动作。
`i` 和 `?` 会取消尚未使用的行走解锁；移动脉冲期间先按空格停车，再选择姿态动作。

| 键 | 官方 G1 API | 本 demo 的前置条件/含义 |
|---|---|---|
| `z` | `ZeroTorque()` → `SetFsmId(0)` | 必须可靠支撑；不要求状态查询成功；不是站立停车 |
| `p` | `Damp()` → `SetFsmId(1)` | 阻尼模式，不保持站立；状态异常时仍可明确请求 |
| `u` | `StandUp()` → `SetFsmId(4)` | FSM 为 1/2/3/4；人工确认姿态适合起立 |
| `t` | `Start()` → `SetFsmId(500)` | FSM 为 4/500；这是运动状态切换，不是启动服务 |
| `b` | `BalanceStand()` → `SetBalanceMode(0)` | FSM=500 |
| `c/v` | `Squat()/Sit()` → FSM 2/3 | FSM 为 4/500；确认地面与空间适合 |
| `o/n` | `HighStand()/LowStand()` | FSM=500、balance=0；发送 SDK 的最大/最小站高语义，不是自定米数 |
| `h` | `WaveHand(false)` → task 0 | FSM=500、balance=0；不带转身；完成需现场观察 |
| `k` | `ShakeHand(0)` → task 2 | FSM=500、balance=0；伸手，保持握手流程未结束 |
| `m` | `ShakeHand(1)` → task 3 | FSM=500；收手后仍需目视确认再解锁行走 |
| `r` | 只查询，不发送动作 | FSM=500、balance=0、健康状态、无未结束握手；解锁一次脉冲 |
| `w/s` | `SetVelocity(±0.10,0,0,0.5)` | 前进/后退，m/s，需先 `r` |
| `a/d` | `SetVelocity(0,±0.08,0,0.5)` | 机器人自身坐标左/右移，m/s，需先 `r` |
| `j/l` | `SetVelocity(0,0,±0.15,0.5)` | 左/右转，rad/s，需先 `r` |
| 空格 / `x` | `StopMove()` | 无条件请求零速度并取消解锁；本程序伸出的握手尝试收回 |
| `q` / Ctrl+C | 尝试停车后退出 | 若本会话未发任何写请求，直接退出；否则尝试 StopMove、握手清理 |
| `i` / `?` | 状态 / 帮助 | 不发送运动指令 |

脉冲是速度请求的持续时间，不能等同于精确位移。按 `w` 的理论速度积分为 5 cm，
实际位移受步态、固件限幅、站姿和响应延迟影响，不保证达到该距离。
SDK 的 `Move(...,true)` 会给出很长的持续时间；本 demo 使用显式有限时长 `SetVelocity`。
握手不使用默认 toggle，而是明确区分 stage 0 和 stage 1。

## 6. 中断的准确含义

空格、`x`、退出和输入断开都不会自动切换零力矩或阻尼，以免站立机器人失去支撑。
StopMove 只设置速度为零；**它不是挥手、起立、坐下等动作的通用取消接口**。
握手可用 stage 1 请求收回，但这同样是一个动作请求。
需要阻尼时在支撑条件满足后显式使用 `p`；紧急情况使用原厂遥控器/急停流程。

SIGINT、SIGTERM、SIGHUP、SIGQUIT 和 Ctrl+Z 会结束键盘循环并尝试清理；
EOF、poll 错误、SDK 异常同样触发清理。Ctrl+Z 在本 demo 中不是暂停后保留运动会话。
SDK 是同步 RPC，单次设置超时 0.3 秒，操作系统调度、多个查询、输出阻塞、网络丢包仍会延迟处理。
SIGKILL、进程崩溃、断电或完全断网无法保证清理请求到达机器人。
机器人端执行 0.5 秒命令超时的行为也需要首次真机验收，不能仅凭代码保证物理安全。

## 7. 自动保护与验收边界

- 单次移动后取消解锁；失败或不确定回复也尝试 StopMove，不自动重放运动。
- 每次行走前重新查询 FSM/平衡模式；状态过期、非有限数、CRC 失败或保守健康检查不通过时拒绝。
- 运行中监控 LowState 的更新，超过 500 ms 无有效新 tick 时尝试停车。
- 本 demo 保守阈值：roll/pitch 绝对值不超过 0.75 rad；前 29 槽位位置/速度/力矩必须有限，
  `motorstate=0`，两路温度分别不超过 80/100。它们是 demo 的准入条件，不是厂商完整故障诊断标准。
- 键盘粘贴/积压指令被丢弃，停车和退出优先；同一输入批次中的多个动作键不会依次执行。
- SDK 初始化不代表硬件可以运动；FSM 值、reply code、实际姿态必须一起判断。

离线已验证：未解锁不发送、方向与速度映射、有限脉冲到期、连发抑制、过期状态、
返回错误、单字母直接执行、旧按键失效、姿态请求间隔、保守状态转换、握手结束、异常后的停车尝试。
2026-10-17 在禁网的 Linux ARM64 容器中编译成功，两个 CTest 目标均通过；
终端测试另外覆盖完整模拟流程、粘贴/积压输入、SIGINT/TERM/HUP/QUIT/TSTP、Ctrl+D 和终端属性恢复。
尚待真机：起立/Start 转换是否被本固件接受、实际正负方向、速度/距离、固件超时停车、动作完成、
遥控器接管和物理中断效果。完成这些前不得把本 demo 当作已验收的安全控制器。

## 8. 官方依据与维护

- [G1 官方开发文档入口](https://support.unitree.com/home/zh/G1_developer)：产品技术文档入口。
- [宇树官方 G1 LocoClient](https://github.com/unitreerobotics/unitree_sdk2/blob/master/include/unitree/robot/g1/loco/g1_loco_client.hpp)：模式、速度与握手接口的官方实现；使用时以本项目固定 SDK 版本为准。
- [本项目 SDK API 编号](https://github.com/Zhijie-He/robot_shared/blob/a801170c41b1b0b527468752b32502a438da3667/library/unitree_sdk2/include/unitree/robot/g1/loco/g1_loco_api.hpp)。
- [本项目 SDK 错误码](https://github.com/Zhijie-He/robot_shared/blob/a801170c41b1b0b527468752b32502a438da3667/library/unitree_sdk2/include/unitree/robot/g1/loco/g1_loco_error.hpp)。
