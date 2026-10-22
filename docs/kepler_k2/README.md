# Kepler K2 使用手册

[首页](../../README.md) · [通用开发指南](../development.md)

等待接入官方api。当前提供模拟后端，可验证关节映射与 DDS 转发。

## 编译与运行

```bash
bash scripts/build_robot.sh k2
cmake --build build-k2-only -j2
ctest --test-dir build-k2-only --output-on-failure

# 在隔离测试网络中运行，Ctrl+C 退出
./build-k2-only/adapter_bridge config/kepler_k2.sample.yaml
```

[配置样例](../../config/kepler_k2.sample.yaml) 使用 12 个模拟关节与 `rt/lowcmd / rt/lowstate`，不代表真机全身关节表。
输出为模拟状态，请勿接入正在运行其他机器人的 DDS 网络。

## 接入官方 API

1. 准备厂商 `motor_api`、依赖和实际关节表。
2. 在适配器中实现连接、读写、健康查询和关闭，保留 `RobotSdkAdapter` 契约。
3. 配置实际关节索引、方向、零位、单位与超时。
4. 验证状态、单关节控制、停止与断连恢复。

当前配置选择 `motor_api` 会报未编译，不能仅通过改配置启用真机。

源码：[适配器](../../src/bridge/adapters/kepler_k2/kepler_k2_adapter.cpp)、[公开配置](../../include/bridge/adapters/kepler_k2/kepler_k2_adapter.hpp)、[测试](../../tests/adapters/kepler_k2_adapter_test.cpp)。
