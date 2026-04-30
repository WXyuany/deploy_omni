# interface_protocol example copy

This directory is kept as a documentation/example copy of the PM01 ROS2 interface package.

## Important

The main project build does **not** consume this directory anymore.

The active package used by the repository build now lives at:

```text
third_party/interface_protocol/
```

When updating message definitions or package wiring for the real build, make changes in `third_party/interface_protocol/`.

This copy exists only to preserve the original PM01 example layout under `docs/pm01_example/`.

## Maintenance note

If message definitions change, either:

1. update `third_party/interface_protocol/` first and mirror the change here if the example should stay in sync, or
2. remove this example copy later if it becomes misleading.
