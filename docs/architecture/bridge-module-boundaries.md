# Bridge Module Boundaries

This document is the source-of-truth ownership map for the standalone bridge framework described in `.sisyphus/plans/robot-sdk-bridge-framework.md`.

## Boundary rules

- The framework is a standalone bridge app, not an extension of `BaseRobotBackend`.
- Core closed-loop scope is fixed to `DDS command ingest -> validated intermediate command frame -> SDK write -> SDK state read -> intermediate state frame -> DDS publish`.
- Core owns lifecycle, loop/threading, watchdog, fault handling, command/state flow, mapping, and startup policy.
- Adapter code owns robot SDK init/read/write/shutdown and nothing DDS-specific.
- Startup config is loaded once, validated once, then frozen.
- Before the first valid command, the system must stay in zero-torque or adapter-safe idle.
- Fault policy is fail-stop: stop normal command writes, emit one safe command if supported, continue publishing degraded state.
- Legacy extras such as joint lock, torque smoothing, CSV logging, colored printing, and extra modes stay outside the MVP core and only enter through explicit optional hooks.

## Target modules and ownership

| Target module | Tag | Owns | Must not own |
| --- | --- | --- | --- |
| `bridge_app_main` | `app` | Process entrypoint, CLI/env handoff, config path selection, assembly of runner + transports + adapter, exit code selection | Control loop logic, DDS codec details, SDK I/O, mapping math |
| `bridge_runner` | `core` | Bridge lifecycle state machine (`Created -> Initializing -> WaitingForState -> Active -> Faulted -> Stopping -> Stopped`), startup/shutdown order, periodic read/publish loop, latest-command buffer consumption, watchdog, first-command safe-idle policy, fault transitions | DDS runtime setup, generated DDS message types, SDK-specific APIs, file parsing, optional legacy behavior |
| `dds_ingress` | `transport` | DDS subscription setup, reception of generated DDS command messages, command translation into `CommandFrame`, sim->hardware joint projection, handoff into runner-owned latest-command buffer | SDK writes, periodic loop ownership, lifecycle state machine |
| `dds_egress` | `transport` | DDS publisher setup, hardware->sim joint projection, generated DDS state/status message filling, DDS publication | SDK reads, runner lifecycle decisions |
| `intermediate_frames` | `core` | Robot-agnostic `CommandFrame`, `StateFrame`, status/fault enums, timestamps/sequence fields required by the closed loop | DDS wire types, SDK handles, robot-specific joint naming assumptions |
| `joint_mapping` | `core` | Declarative mapping between transport-level joints and intermediate frames, ordering/index normalization, limits needed for frame validation, command/state field projection rules | DDS transport construction, SDK calls, lifecycle threading |
| `config_loader` | `config` | Startup-time file loading, schema validation, config normalization, frozen config object creation for runner/transport/adapter wiring | Runtime hot reload, loop execution, SDK I/O, fault recovery policy changes at runtime |
| `robot_sdk_adapter` | `adapter` | SDK init, hardware/session setup, safe command capability, command write, state read, health check, shutdown | DDS concepts, transport message types, runner threading policy, config file parsing beyond adapter-specific section intake |
| `imu_source` | `IMU source` | Optional robot-agnostic interface for IMU snapshots when state publication needs them, timestamped IMU acquisition, adapter-owned or external-source binding | Command ingest, runner lifecycle control, DDS subscription ownership |
| `optional_hooks` | `optional extension` | Explicit post-MVP extension seam for logging, telemetry mirrors, joint lock, torque smoothing, extra modes, debug probes | Required bridge safety behavior, baseline command/state path, hidden side effects inside core |

## Ownership by concern

### Startup ownership

1. `bridge_app_main` selects the config source and constructs all modules.
2. `config_loader` parses and validates the full bridge config, then returns an immutable configuration object.
3. `bridge_app_main` builds `dds_ingress`, `dds_egress`, `joint_mapping`, `robot_sdk_adapter`, optional `imu_source`, and `optional_hooks`, then injects them into `bridge_runner`.
4. `bridge_runner` owns runtime startup sequencing:
   - transition to `Initializing`
   - initialize adapter
   - establish transport endpoints
   - obtain first readable robot state
   - publish initial degraded/waiting state as needed
   - enter `WaitingForState` then `Active` only after state and transport are ready
5. Before the first valid command is consumed, `bridge_runner` keeps the adapter in zero-torque or adapter-safe idle.

### Shutdown ownership

- `bridge_runner` owns shutdown sequencing once stop is requested or a fatal fault occurs.
- `bridge_runner` stops normal command consumption, requests one adapter safe command if supported, stops the periodic loop, and transitions through `Stopping` to `Stopped`.
- `robot_sdk_adapter` owns final SDK teardown.
- `bridge_app_main` owns only process-level return codes and final destruction order after the runner has stopped.

### Loop and thread ownership

- Exactly one periodic runtime loop is owned by `bridge_runner`.
- The periodic loop performs: adapter state read -> optional IMU read -> state frame assembly -> DDS egress publish -> watchdog/fault evaluation -> command application for the next cycle according to runner policy.
- `dds_ingress` may run DDS callback threads, but those callbacks only decode and update the runner-owned latest-command buffer.
- DDS callback threads must never call the robot SDK directly.
- `optional_hooks` may observe events but must not become the owner of any required loop.

## Command and state flow boundaries

### Command flow

1. `dds_ingress` receives the generated DDS command message and rejects malformed transport input.
2. `dds_ingress` uses `joint_mapping` and `intermediate_frames` rules to build a validated hardware-ordered `CommandFrame` suitable for the bridge core.
3. `bridge_runner` stores the latest valid command frame and decides when it becomes active in the periodic cycle.
4. `robot_sdk_adapter` translates the active `CommandFrame` into SDK-native command calls.
5. If no valid command has arrived yet, `bridge_runner` sends safe idle behavior instead of inferred hold-position behavior.

### State flow

1. `bridge_runner` triggers a periodic read.
2. `robot_sdk_adapter` reads robot state from the SDK.
3. Optional `imu_source` contributes IMU data if configured.
4. `dds_egress` projects SDK-derived state into a robot-agnostic/sim-ordered shape and fills generated DDS state/status messages.
5. `dds_egress` publishes the resulting typed DDS output messages.

## Config ownership boundaries

- `config_loader` is the only module allowed to parse bridge config files.
- `joint_mapping` receives normalized mapping config, not raw file text.
- `robot_sdk_adapter` receives only its adapter-specific config subset after validation.
- `bridge_runner` receives frozen runtime config values such as loop period, watchdog thresholds, startup idle policy, and fault behavior flags.
- No module may mutate live config after startup.

## Fault boundaries

| Fault class | Primary owner | Boundary behavior |
| --- | --- | --- |
| DDS decode / subscription fault | `dds_ingress` + `bridge_runner` | Transport reports failure; runner decides whether to stay degraded or enter `Faulted` |
| Invalid command content after mapping/validation | `bridge_runner` | Reject command, keep last safe/accepted behavior, preserve SDK isolation |
| SDK init/read/write/health fault | `robot_sdk_adapter` surfaces, `bridge_runner` owns policy | Adapter reports failure cause; runner transitions to `Faulted`, stops normal writes, requests safe command if supported |
| Config/schema fault | `config_loader` | Fail startup before runtime begins |
| Optional feature failure | `optional_hooks` | Must not break core loop; hook errors are isolated unless explicitly configured as fatal |
| IMU acquisition fault | `imu_source` + `bridge_runner` | Publish degraded state or omit IMU fields by policy; do not leak IMU details into command path |

The hard boundary is that adapters surface capabilities and failures, but only `bridge_runner` decides lifecycle transitions and fail-stop behavior.

## Extension boundaries

- `optional_hooks` is the only allowed seam for non-MVP behaviors.
- Hooks may observe or transform data only at explicit extension points chosen by the runner/app wiring.
- Hooks must not silently alter required safety behavior, fault policy, startup sequencing, or transport ownership.
- Legacy extras map here unless and until they are promoted into first-class core requirements.

## Legacy responsibility migration map

| Legacy conceptual responsibility | Target module | Tag | Notes |
| --- | --- | --- | --- |
| `main()` bootstrap and ROS/DDS process bring-up | `bridge_app_main` | `app` | Keep thin; assemble modules only |
| Monolithic lifecycle flags and bridge state transitions | `bridge_runner` | `core` | Central source for runtime state machine |
| DDS subscriber callback ingest | `dds_ingress` | `transport` | Callback writes latest command buffer only |
| DDS publisher wiring and outbound status publishing | `dds_egress` | `transport` | Outbound codec/publish split from core logic |
| Shared command struct intertwined with K2 data layout | `intermediate_frames` | `core` | Replace robot-specific structs with robot-agnostic frames |
| Joint index remap / name remap logic | `joint_mapping` | `core` | Declarative mapping, not adapter hard-code |
| YAML/JSON parameter parsing and defaults | `config_loader` | `config` | Parse once, validate once, freeze |
| `motor_api` session init, command send, state read, shutdown | `robot_sdk_adapter` | `adapter` | Adapter is the only SDK-facing layer |
| IMU polling or SDK-specific inertial snapshot handling | `imu_source` | `IMU source` | Separate source so IMU can be absent or replaced |
| Joint lock / torque smoothing / CSV log / debug prints / extra modes | `optional_hooks` | `optional extension` | Out of MVP core path; explicit opt-in only |

## Non-negotiable seams for downstream tasks

- Downstream type work must treat `intermediate_frames` as the only shared command/state contract across transport and adapters.
- Downstream runner work must keep all runtime control decisions inside `bridge_runner`.
- Downstream adapter work must keep DDS-free interfaces.
- Downstream config work must preserve one-shot load/validate/freeze behavior.
- Downstream extension work must integrate through `optional_hooks`, never by reopening core ownership.
