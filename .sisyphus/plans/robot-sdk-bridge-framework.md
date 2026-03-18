# Reusable DDS-SDK Bridge Framework for Humanoid Robots

## TL;DR
> **Summary**: Extract the current monolithic `kepler_k2_bridge.cpp` into a reusable standalone bridge framework built around a generic bridge core and a thin per-robot SDK adapter. The MVP supports the closed loop only: DDS command ingest → validated intermediate command model → robot SDK write → robot SDK state read → intermediate state model → DDS publish.
> **Deliverables**:
> - Reusable `bridge_core` architecture
> - Declarative joint mapping/config layer
> - `RobotSdkAdapter` contract and fake adapter seam
> - First migrated `KeplerK2Adapter`
> - Standalone `kepler_k2_bridge` executable rebuilt on the new core
> **Effort**: Large
> **Parallel**: YES - 2 waves
> **Critical Path**: 1 → 2 → 3 → 5 → 6 → 8

## Context
### Original Request
将收到的 DDS 消息转化成具体电机数据，通过 SDK 发送到机器人；再通过 SDK 读取状态并通过 DDS 返回推理模型。当前已实现 `kepler_k2`，希望封装一套通用代码框架，只需替换不同机器人的 SDK 具体函数即可复用。

### Interview Summary
- 目标不是直接改 `policy_deploy` 主工程后端，而是先做**独立 bridge 可执行程序**。
- 抽象边界采用**适配器分层**：通用层负责 DDS 流程、状态/命令中间模型、关节映射、生命周期、线程与故障处理；每个机器人只保留 SDK 适配器。
- MVP 采用**核心闭环优先**：先覆盖 `DDS cmd -> SDK` 与 `SDK state -> DDS` 双向闭环。
- `kepler_k2_bridge.cpp` 里的增强能力（关节锁、力矩平滑、CSV 日志、彩色打印、额外模式）不进入核心层，只保留为后续扩展钩子。
- 当前仓库未发现正式单元测试/CI 体系；若进入实现阶段，测试策略采用 **tests-after**。

### Metis Review (gaps addressed)
- 已补齐：MVP 范围冻结、故障策略、线程模型、单进程假设、配置权威来源、从单体 K2 bridge 到模块化结构的迁移顺序。
- 已补齐：中间模型必须独立于 Kepler，不允许把 `motor_api` / `SharedMemory` / K2 特例泄漏到通用接口。
- 已补齐：必须为适配器层预留 fake seam，避免桥核心与 SDK 强耦合后无法验证映射/生命周期/故障策略。

### Oracle Review (gaps addressed)
- 已锁定：架构采用 `bridge_core + bridge_dds + robot_sdk_kepler_k2 + thin app` 的 library-first 拆分。
- 已锁定：线程模型为“1 条周期性 read/publish loop + DDS callback 上的 latest-command ingest”，禁止在 DDS 回调线程直写 SDK。
- 已锁定：配置在启动时一次性加载和校验；MVP 默认单进程单机器人实例。
- 已锁定：首次命令到达前采用 **zero-torque / safe command idle policy**，而不是隐式保持位置。

## Work Objectives
### Core Objective
把 `src/sim2/real/robot_backend/kepler_k2/kepler_k2_receive/src/kepler_k2_bridge.cpp` 重构为“可复用于多机器人 SDK”的独立桥接框架，使新机器人接入时主要只新增一个 SDK 适配器和映射配置，而不再复制整份 bridge 主流程。

### Deliverables
- 一个可复用的 `bridge_core` 模块边界设计
- 一个与具体 SDK 解耦的 `CommandFrame` / `StateFrame` 中间模型
- 一个声明式 `JointMap` / bridge config 设计
- 一个明确的 `RobotSdkAdapter` 生命周期与错误语义契约
- 一个按新架构迁移 `KeplerK2Adapter` 的执行方案
- 一个保持独立构建的 CMake 目标拆分方案

### Definition of Done (verifiable conditions with commands)
- 新框架目录与目标文件存在：`test -f` 检查核心头源文件、K2 适配器文件、bridge main、配置样例。
- `kepler_k2_bridge` 新目标可独立编译成功：`cmake --build <build-dir> --target kepler_k2_bridge -j`
- fake adapter / mapping / lifecycle 相关测试可运行通过：`ctest --output-on-failure`
- `kepler_k2_bridge.cpp` 中的核心闭环职责已迁移到复用模块，原主文件只保留 thin main 或被等价替换。
- K2 接入后，可通过桥核心完成：接收 DDS 低层命令、转换为中间命令、调用 SDK、读回状态、发布 DDS 低层状态。

### Must Have
- 单进程单机器人实例模型
- Core 仅负责 DDS、映射、生命周期、故障策略、线程、watchdog
- Adapter 仅负责 SDK 初始化、读状态、写命令、安全停机、健康检查
- Startup-time 静态配置加载并冻结，不做热更新
- 默认控制模式：torque closed-loop passthrough with PD/feedforward
- 默认故障策略：fail-stop 进入 `Faulted`，停止正常下发，发送一次安全命令（若 adapter 支持），继续发布降级状态
- 默认线程模型：一个周期性读状态/发状态 loop；DDS 回调仅更新 latest-command buffer
- 默认首次命令策略：在收到第一条有效 DDS 命令前，下发 zero-torque / adapter safe command
- 一个明确生命周期：`Created -> Initializing -> WaitingForState -> Active -> Faulted -> Stopping -> Stopped`

### Must NOT Have (guardrails, AI slop patterns, scope boundaries)
- 不接入 `BaseRobotBackend`
- 不把 CSV 日志、彩色打印、关节锁、力矩平滑做进 MVP 核心路径
- 不做动态插件系统
- 不做多机器人多进程协调
- 不把 DDS 细节泄漏进 adapter 接口
- 不把 K2 的 `motor_api`、SharedMemory、关节分区假设写进通用 core 类型

## Verification Strategy
> ZERO HUMAN INTERVENTION — all verification is agent-executed.
- Test decision: tests-after + CTest/CMake test binaries
- QA policy: Every task has agent-executed scenarios
- Evidence: `.sisyphus/evidence/task-{N}-{slug}.{ext}`

## Execution Strategy
### Parallel Execution Waves
> Target: 5-8 tasks per wave. <3 per wave (except final) = under-splitting.
> Extract shared dependencies as Wave-1 tasks for max parallelism.

Wave 1: architecture skeleton, types/contracts, mapping/config, fake seams, CMake split
Wave 2: K2 migration, executable wiring, extension hook isolation

### Dependency Matrix (full, all tasks)
| Task | Depends On |
|---|---|
| 1 | - |
| 2 | 1 |
| 3 | 1 |
| 4 | 2,3 |
| 5 | 2,3,4 |
| 6 | 5 |
| 7 | 5 |
| 8 | 6,7 |

### Agent Dispatch Summary (wave → task count → categories)
- Wave 1 → 5 tasks → `general`, `deep`
- Wave 2 → 3 tasks → `deep`, `general`, `unspecified-high`

## TODOs
> Implementation + Test = ONE task. Never separate.
> EVERY task MUST have: Agent Profile + Parallelization + QA Scenarios.

- [x] 1. Extract monolith responsibility map and target module boundaries
- [x] 2. Define bridge core types, adapter contract, and fault semantics
- [x] 3. Implement declarative mapping and frozen startup config layer
- [x] 4. Build bridge runner lifecycle, watchdog, and latest-command flow
- [x] 5. Split DDS transport/codec from core and add fake transport seam
- [x] 6. Implement `KeplerK2Adapter` on top of the new adapter contract
- [x] 7. Rebuild standalone `kepler_k2_bridge` as thin app + shared CMake targets
- [x] 8. Isolate legacy extras behind explicit extension hooks

## Final Verification Wave (MANDATORY — after ALL implementation tasks)
- [x] F1. Plan Compliance Audit — oracle
- [x] F2. Code Quality Review — unspecified-high
- [x] F3. Real Manual QA — unspecified-high (+ playwright if UI)
- [x] F4. Scope Fidelity Check — deep

## Commit Strategy
- Commit 1: scaffold `bridge_core` structure, shared types, config/mapping contracts
- Commit 2: add lifecycle/watchdog/fake adapter tests and reusable DDS bridge loop
- Commit 3: migrate `kepler_k2_bridge` onto `KeplerK2Adapter` and keep standalone build green
- Commit 4: isolate optional extension hooks without enabling legacy extras by default

## Success Criteria
- New robot bring-up requires creating one adapter + one config/mapping set, not copying bridge control flow.
- Core bridge modules contain no K2-specific headers or hard-coded joint partition rules.
- Fault behavior, startup/shutdown order, and command validity rules are explicit and test-covered.
- `kepler_k2_bridge` remains buildable as a standalone executable under the new structure.
