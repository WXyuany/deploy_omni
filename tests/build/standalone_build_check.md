# Standalone build check

`kepler_k2_bridge` is currently wired as a thin process entrypoint that loads startup config once, freezes runtime values, and then assembles these existing pieces:

- `bridge::config::load_bridge_config()`
- `bridge::core::BridgeRunner`
- `bridge::dds::make_lowcmd_codec()`
- `bridge::dds::make_lowstate_codec()`
- `bridge::adapters::kepler_k2::KeplerK2Adapter`
- `bridge::config::JointMap`

The app now reads `config/kepler_k2.sample.yaml` by default, validates the current sample schema locally inside `bridge_config`, and freezes runner, adapter, and joint-map subsets before runtime wiring begins.

The current repository state still does not provide concrete implementation files for `bridge::dds::DdsCommandIngress` or `bridge::dds::DdsStateEgress`. To keep the app linkable without moving lifecycle or loop ownership out of `BridgeRunner`, `src/apps/kepler_k2_bridge_main.cpp` still uses the smallest local seam possible:

- a local ingress that is prepared to decode DDS command samples and project them through `JointMap::project_command_sim_to_hardware()` before submitting hardware-ordered commands to the runner
- a local egress that projects hardware-ordered adapter state through `JointMap::project_state_hardware_to_sim()` before reusing the existing low-state encoder and retaining publications in-process

This intentionally preserves the ownership boundary from `docs/architecture/bridge-module-boundaries.md`: the app only assembles modules and chooses an exit code, while `BridgeRunner` still owns initialization, the periodic read/publish cycle, startup safe-idle behavior, stop request handling, and shutdown.

Build verification command:

`cmake -S . -B build && cmake --build build --target kepler_k2_bridge -j && ctest --test-dir build --output-on-failure`
