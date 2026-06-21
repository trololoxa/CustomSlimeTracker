# Module inventory

This inventory is the current ownership map. It is intentionally compact; update it when adding, moving, or deleting modules.

| Path | Owner layer | Purpose | Notes |
|---|---|---|---|
| `src/main.cpp` | entrypoint | Arduino `setup()`/`loop()` forwarding only | Must stay tiny. |
| `src/defines.h` | global defaults | Compile-time board/default constants | Canonical defaults header. |
| `src/app/` | composition | Firmware setup/loop orchestration and app-level singleton wiring | No domain algorithms here. |
| `src/app/hooks/` | composition glue | Connect app singletons to runtime/serial callbacks | Include-only by design; wiring only. |
| `src/config/` | persisted config | Schema, runtime apply/capture/sanitize, NVS store, print helpers | Config changes need docs/tests. |
| `src/connection/` | hardware/protocol | LSM6DSV, FIFO, sensor-hub, and low-level magnetometer transport | No app/serial dependencies. |
| `src/core/` | pure core | Math primitives and small shared utilities | Host-test friendly. |
| `src/runtime/` | runtime controllers | FIFO runtime, sample pipeline, logs, output snapshots, static tests, bias, mag runtime, state | Should expose status/results, not own CLI parsing. |
| `src/sensor/` | sensor math/models | AHRS, calibration, IMU quality, mag heading/yaw correction | Prefer pure/host-testable logic. |
| `src/serial/` | developer CLI | Fixed-buffer parser, command context, domain command handlers, serial stream helpers | Domain commands live in `.cpp`; headers expose API only. |
| `src/network/` | transport | Wi-Fi station management and UDP transport primitives | No AHRS/FIFO logic or SlimeVR packet formatting here. |
| `src/output/` | protocol output | Host-safe SlimeVR packet writer/protocol helpers | No Wi-Fi state or sensor fusion ownership. |
| `tools/logs/` | host tools | Existing E0 log summarizer | Human/debug summaries. |
| `tools/replay/` | host tools | Replay/metrics tooling for machine logs | Added before major tracking changes. |
| `tests/native/` | host tests | Host-safe C++ regression tests | No real Arduino/SPI/NVS/Wi-Fi hardware behavior. |
| `docs/` | docs | Architecture, testing, CLI, config, replay, tracking pipeline | Docs should describe current code, not stale roadmaps. |

## Experimental/future code policy

If a module is exploratory and not production-ready, keep it out of production command paths. Future research code should go under an explicit experimental area or behind compile-time flags, and commands must return `NOT_IMPLEMENTED` rather than pretending to be active.

## Motion light sleep

- `runtime/motion_light_sleep_controller.*` — host-testable continuous server-absence timeout policy used by the optional LSM6DSV motion light-sleep lifecycle.
- `connection/lsm6dsv_driver.*` — LSM6DSV wake-up INT1 configuration and wake-source clear helper.
