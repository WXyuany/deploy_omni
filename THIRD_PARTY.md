# 来源与第三方依赖

本项目基于 [Apolostephen/adapter_any_robot](https://github.com/Apolostephen/adapter_any_robot) 的工作区整理，保留原有核心、适配器与协议实现，并新增机型文档、独立示例和构建组织。
本仓库未新增覆盖全部代码的统一开源许可证；第三方内容沿用各自许可与来源说明。

| 内容 | 来源／获取方式 | 管理方式 |
|---|---|---|
| 共享协议、Unitree SDK2、CycloneDDS | [robot_shared](https://github.com/Zhijie-He/robot_shared) | Git 子模块固定提交 `a801170c41b1b0b527468752b32502a438da3667`；许可见依赖目录 |
| Unitree SDK2 上游 | [unitreerobotics/unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | 使用上述子模块内的配套头文件与库 |
| CVTE CArm SDK | [CVTE 文档中心](https://cvte-bot.feishu.cn/wiki/GyGbwKeWMiqEfDk80QXc9zeCnjf) | 用户自行下载，与控制器及架构匹配；不上传厂商 SDK 包 |
| PM01 ROS 消息 | `third_party/interface_protocol` | 从原工程保留，按现场接口核对 |
| SDL 手柄映射（可选） | [SDL_GameControllerDB](https://github.com/gabomdq/SDL_GameControllerDB) | 不附带数据库快照；按需添加条目 |

`library` 是指向 `third_party/robot_shared/library` 的兼容软链接。
共享依赖保留为子模块，不将完整第三方仓库复制进本项目历史。升级时同时验证协议生成代码、SDK 头文件、二进制 ABI 和测试结果。
