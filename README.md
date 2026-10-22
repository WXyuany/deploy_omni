# deploy_omni

机器人 SDK 适配与 DDS 转发框架。按机器人独立编译，统一连接、关节状态、关节命令和故障处理，厂商专有功能通过对应 SDK 调用。

## 快速开始

在 Linux x86_64 / ARM64 环境下载项目并安装构建工具：

```bash
git clone --recurse-submodules https://github.com/WXyuany/deploy_omni.git
cd deploy_omni
sudo apt-get update
sudo apt-get install -y build-essential cmake git python3 python3-venv
```

然后选择下方设备手册，按“连接 → 编译 → 验证功能 → 调用接口”操作。每类设备只有一份手册，包含一行验证命令、预期结果、配置和接口说明。

## 设备手册

| 设备 | 使用文档 |
|---|---|
| Unitree G1 | [连接、运控、相机、雷达与 SDK](docs/unitree_g1/README.md) |
| Kepler K2 | [编译、模拟运行与 API 接入](docs/kepler_k2/README.md) |
| CVTE CArm A3 / D3 | [单臂、双臂、夹爪与 SDK](docs/cvte_carm/README.md) |

## 框架结构

```text
上层应用 → 命令入口 → 统一适配器 → 厂商 SDK → 机器人
上层应用 ← 状态出口 ← 统一适配器 ← 厂商 SDK ← 机器人
```

通用层处理关节数据和运行流程；图像、点云、音频等能力由机型专用接口提供。

```text
config/             机器人配置样例
include/bridge/     公开接口
src/bridge/         核心、适配器、DDS 与配置实现
src/apps/           桥接程序与状态工具
examples/           按机型组织的示例
cmake/              构建选项与依赖
scripts/            构建和仓库检查脚本
tests/              测试
docs/               通用开发指南与设备手册
third_party/        固定版本的共享依赖
```

## 二次开发

[开发指南](docs/development.md) 集中说明构建选项、Docker、统一接口、配置、测试和新增设备的方法。

依赖来源与许可见 [第三方说明](THIRD_PARTY.md)。
