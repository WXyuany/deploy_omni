# Kepler K2

[首页](../../README.md) · [架构](../architecture.md)

## 当前可以做什么

根工程的 `KeplerK2Adapter` 提供确定性模拟会话，用于验证配置、关节映射、runner、DDS 输入输出。
**没有编译真实 `motor_api` 后端。** 配置请求 `motor_api` 时会明确报错，而不会伪装成已连接。
历史独立 K2 工程不属于本仓库的可部署目标，不能把它的运行记录当作本项目真机验证。

## 编译与测试

```bash
bash scripts/build_robot.sh k2
cmake --build build-k2-only -j2
ctest --test-dir build-k2-only --output-on-failure
```

依赖共享子模块的 Unitree SDK2/CycloneDDS，仅用于公共 DDS 协议，不代表 K2 使用宇树电机驱动。

## 隔离环境验证主程序

样例 `config/kepler_k2.sample.yaml` 使用 12 个映射项，是模拟链路配置，不是 K2 全身关节表。
它会使用 `rt/lowcmd` / `rt/lowstate`，不要接入正在运行其他机器人的 DDS 网络：

```bash
# 仅在隔离测试网络运行，Ctrl+C 结束进程
./build-k2-only/adapter_bridge config/kepler_k2.sample.yaml
```

该程序产生的是模拟关节状态，不能用于判断真实关节、IMU 或电机驱动是否正常。
通用主程序目前没有与 G1 键盘 demo 相同的信号停车处理；不得把退出进程当作物理急停。

## 接入真实硬件还需要什么

1. 准备厂商 `motor_api`、配套依赖与实际关节顺序。
2. 在 `src/bridge/adapters/kepler_k2/` 实现连接、读状态、写命令、健康查询和关闭。
3. 替换模拟会话，保留 `RobotSdkAdapter` 契约；核心与 DDS 层不用添加 K2 特例。
4. 建立实际全身映射配置，验证索引、符号、零位、单位和命令时效。
5. 完成只读状态、低速单关节、停止和断连测试后，再标为真机支持。

源码：[适配器](../../src/bridge/adapters/kepler_k2/kepler_k2_adapter.cpp)、[公开配置](../../include/bridge/adapters/kepler_k2/kepler_k2_adapter.hpp)、[离线测试](../../tests/adapters/kepler_k2_adapter_test.cpp)。
