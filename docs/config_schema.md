# Config schema policy

Persistent config lives in `src/config/`. Runtime/session state must not be persisted unless a command deliberately captures it as calibration or user settings.

## Files

| File | Purpose |
|---|---|
| `tracker_config_schema.hpp` | Persistent structs and version constants. |
| `tracker_config_detail.hpp` | CRC/constants/detail helpers. |
| `tracker_config_runtime.hpp/.cpp` | Defaults, sanitize/validate, apply/capture runtime values. |
| `tracker_config_store.hpp/.cpp` | NVS/Preferences persistence and store inspection. |
| `tracker_network_config.hpp/.cpp` | Future network config storage, separate from core IMU config. |
| `tracker_config_print.hpp/.cpp` | Human-readable config printing. |

## Ownership rules

Persisted:

- board/runtime defaults selected by the user;
- IMU/FIFO configuration;
- AHRS tunables;
- gyro bias and gyro temperature compensation;
- accel calibration;
- mag axis/calibration/yaw-correction policy;
- output policy;
- future network configuration.

Runtime only:

- quaternion;
- current tracking state;
- FIFO/quality/performance counters;
- recovery counters;
- static test in-progress state;
- mag heading runtime reference;
- runtime gyro-bias trim;
- prepared output snapshot.

## Change policy

When changing persisted structs:

1. Bump the schema version if layout or meaning changes.
2. Keep default construction deterministic.
3. Sanitize loaded values before applying them to runtime.
4. Invalidate only the calibration block that is actually bad when possible.
5. Add or update native tests for defaults/sanitize/CRC/layout expectations.
6. Update this document and `docs/cli_reference.md` if commands or side effects change.

Current config hardening is intentionally low-cost: compile-time/native-test layout guards and host tests are preferred over runtime-heavy migration logic until Wi-Fi/SlimeVR fields are finalized.
