# 添加新机器人适配器指南

## 概述

添加一个新机器人需要创建一个新的 adapter，实现 `RobotSdkAdapter` 接口，并在工厂中注册。

## 需要创建的文件

```
include/bridge/adapters/<robot_name>/
  └── <robot_name>_adapter.hpp          ← 新适配器头文件

src/bridge/adapters/<robot_name>/
  └── <robot_name>_adapter.cpp          ← 新适配器实现

tests/adapters/
  └── <robot_name>_adapter_test.cpp     ← 测试（可选但建议）
```

## 需要修改的文件

### 1. 工厂注册（必须修改）

文件：`src/bridge/app/robot_assembly_factory.cpp`

```cpp
// 添加 include
#include "../../../include/bridge/adapters/<robot_name>/<robot_name>_adapter.hpp"

// 添加创建函数
[[nodiscard]] RobotAssemblyCreateResult create_<robot_name>_assembly(
    const config::BridgeStartupConfig& startup_config) {
  // 1. 构建 JointMap
  // 2. 创建 adapter
  // 3. 创建 DDS assembly
  // 4. 组装返回
}

// 在 factory_entries 中注册
static const std::vector<RobotFactoryEntry> entries{{
    .robot_type = "kepler_k2",
    .create = &create_kepler_k2_assembly,
}, {
    .robot_type = "<robot_name>",       // ← 新增
    .create = &create_<robot_name>_assembly,
}};
```

---

## 需要实现的接口

`RobotSdkAdapter`（定义在 `include/bridge/core/robot_sdk_adapter.hpp`）有 7 个纯虚函数，**全部必须实现**：

| 方法                   | 返回类型              | 作用         | 真机实现要点             |
|------------------------|-----------------------|--------------|--------------------------|
| `name()`               | `string_view`         | 返回适配器名称 | 直接返回固定字符串       |
| `initialize()`         | `AdapterCallResult`   | 连接机器人     | 调用厂商 SDK 建立连接    |
| `read_state()`         | `AdapterReadResult`   | 读取关节状态   | 调用厂商 SDK 读取传感器数据 |
| `write_command()`      | `AdapterCallResult`   | 发送运动指令   | 调用厂商 SDK 写入指令    |
| `write_safe_command()` | `AdapterCallResult`   | 发送安全指令   | 发送零力矩/安全姿态      |
| `query_health()`       | `AdapterHealthResult` | 查询健康状态   | 检查连接状态、硬件故障   |
| `shutdown()`           | `AdapterCallResult`   | 断开连接       | 调用厂商 SDK 释放资源    |

---

## 必须定义的配置结构

参考 `KeplerK2AdapterConfig`，为新机器人定义自己的配置：

```cpp
struct <RobotName>AdapterConfig {
  SessionConfig session;                     // 连接参数
  SafetyConfig safety;                       // 安全策略
  std::vector<std::string> joint_names;      // 关节名称列表
  // ... 新机器人特有的配置
};
```

---

## 关键数据结构（不需要重写，直接使用）

这些类型定义在 `core/types.hpp`，所有 adapter 共享：

| 结构体               | 用途                                    |
|----------------------|-----------------------------------------|
| `CommandFrame`       | 从 DDS 收到的指令（位置/速度/力矩/刚度/阻尼） |
| `StateFrame`         | 向 DDS 发送的状态（位置/速度/力矩/加速度/IMU） |
| `AdapterCallResult`  | 操作结果（状态码 + 详情 + 可选故障）     |
| `AdapterHealth`      | 健康状态信息                            |
| `BridgeFault`        | 故障信息（分类 + 严重程度 + 消息）       |

---

## 建议的实现结构（PIMPL 模式）

参考 `kepler_k2_adapter.cpp` 的写法：

```cpp
// .hpp 文件
class NewRobotAdapter final : public core::RobotSdkAdapter {
 public:
  explicit NewRobotAdapter(NewRobotAdapterConfig config);
  ~NewRobotAdapter() override;

  // 禁用拷贝
  NewRobotAdapter(const NewRobotAdapter&) = delete;
  NewRobotAdapter& operator=(const NewRobotAdapter&) = delete;
  // 允许移动
  NewRobotAdapter(NewRobotAdapter&&) noexcept;
  NewRobotAdapter& operator=(NewRobotAdapter&&) noexcept;

  // 实现接口
  std::string_view name() const noexcept override;
  AdapterCallResult initialize() override;
  AdapterReadResult read_state() override;
  AdapterCallResult write_command(const CommandFrame& command) override;
  AdapterCallResult write_safe_command() override;
  AdapterHealthResult query_health() const override;
  AdapterCallResult shutdown() override;

 private:
  class Impl;                        // 前向声明
  std::unique_ptr<Impl> impl_;      // PIMPL 指针
};

// .cpp 文件
class NewRobotAdapter::Impl {
 public:
  // 真正的实现逻辑
  // 这里调用厂商 SDK

 private:
  // 厂商 SDK 的 session/handle
  // 状态机
  // 线程安全（mutex）
};

// 外壳只做转发
AdapterCallResult NewRobotAdapter::initialize() {
  return impl_->initialize();
}
```

---

## 真机对接时的核心替换点

以 `KeplerK2` 为例，当前使用 `DeterministicVendorSession` 模拟硬件。对接真机时只需替换为一个调用真实 SDK 的类：

| 模拟方法                    | 真机替换                       |
|-----------------------------|--------------------------------|
| `position()` → 返回 0.0     | → `uni_api_.ReadJointPositions()`  |
| `velocity()` → 返回 0.0     | → `uni_api_.ReadJointVelocities()` |
| `effort()` → 返回 0.0       | → `uni_api_.ReadJointEfforts()`    |
| `apply_command()` → 直接赋值 | → `uni_api_.SendCommand(...)`      |
| `apply_safe_command()` → 填零 | → `uni_api_.SendZeroTorque()`      |
| `reset()` → 清空向量         | → `uni_api_.Disconnect()`          |

**其他代码（状态机、安全检查、PIMPL 外壳）不需要修改。**

---

## 安全检查建议

参考 `kepler_k2_adapter.cpp` 的 `write_command()`，建议对新机器人也做以下检查：

1. **指令有效性**：`command.valid == true`
2. **指令过期**：`command.stale == false`
3. **控制模式**：`command.mode != kUnknown`
4. **向量长度**：关节向量长度匹配 `joint_count`
5. **初始状态**：运动前必须先读取过状态
6. **IMU 可用性**：如果需要 IMU，检查 IMU 是否就绪

---

## 总结

| 项目                                                    | 是否必须         |
|---------------------------------------------------------|------------------|
| 创建 `<robot_name>_adapter.hpp`                         | ✅ 必须          |
| 创建 `<robot_name>_adapter.cpp`                         | ✅ 必须          |
| 实现 `RobotSdkAdapter` 的 7 个方法                      | ✅ 必须          |
| 定义 `AdapterConfig`                                    | ✅ 必须          |
| 在 `robot_assembly_factory.cpp` 注册                    | ✅ 必须          |
| 替换 `DeterministicVendorSession` 为真机 SDK 调用       | ✅ 对接真机时必须 |
| 修改 `core/types.hpp`                                   | ❌ 不需要        |
| 修改 DDS 层                                             | ❌ 不需要        |
| 修改 `bridge_runner`                                    | ❌ 不需要        |
