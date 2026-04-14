# Standalone build check

`kepler_k2_bridge` is currently wired as a thin process entrypoint that loads startup config once, freezes runtime values, selects a registered robot factory, and then assembles these existing pieces:

- `bridge::config::load_bridge_config()`
- `bridge::app::create_robot_assembly()`
- `bridge::core::BridgeRunner`
- `idl/bridge/BridgeMessages.idl` and generated Fast DDS artifacts in the same folder
- `bridge::dds::create_dds_assembly()`
- `bridge::dds::make_dds_runtime()`
- `bridge::adapters::kepler_k2::KeplerK2Adapter`
- `bridge::config::JointMap`

The app now reads `config/kepler_k2.sample.yaml` by default, validates the current sample schema locally inside `bridge_config`, freezes runner, adapter, and joint-map subsets, and lets the app factory map `adapter.kind` to the correct runtime assembly before wiring begins.

The current repository state now provides a hand-written DDS facade/runtime layer. `RobotAssemblyFactory` obtains a DDS assembly from `bridge::dds::create_dds_assembly()`, and that factory owns:

- generated Fast DDS type support registration
- participant / publisher / subscriber ownership via `DdsRuntime`
- a command ingress that translates generated `LowCmd` messages into runner-facing `CommandFrame`
- a state egress that projects runner state into generated `LowState` / `BridgeStatus` publications

This intentionally keeps process-level wiring thin: the app loads config, selects the robot assembly factory, and chooses an exit code, while `BridgeRunner` owns initialization, the periodic read/publish cycle, startup safe-idle behavior, stop request handling, and shutdown. DDS generated types stay in `bridge/dds`, not in `bridge/core`.

Build verification command:

`cmake -S . -B build && cmake --build build --target kepler_k2_bridge -j && ctest --test-dir build --output-on-failure`
