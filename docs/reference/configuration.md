# Config schema policy

Current slot versions, compatibility and rollback rules are owned by
[calibration storage](../architecture/calibration_storage.md).

Persistent config lives in `src/config/`. Runtime/session state must not be persisted unless a command deliberately captures it as calibration or user settings.

## Files

| File | Purpose |
|---|---|
| `tracker_config_schema.hpp` | Persistent structs and version constants. |
| `tracker_config_detail.hpp` | CRC/constants/detail helpers. |
| `tracker_config_runtime.hpp/.cpp` | Defaults, sanitize/validate, apply/capture runtime values. |
| `tracker_config_storage.hpp/.cpp` | Dual-slot records, selector, sensor signature, quality and candidate schemas. |
| `tracker_config_store.hpp/.cpp` | Dual-slot NVS persistence, legacy migration, candidate wear control and atomic promotion. |
| `tracker_network_config.hpp/.cpp` | Wi-Fi/SlimeVR network config storage, separate from core IMU config. |
| `tracker_config_print.hpp/.cpp` | Human-readable config printing. |


## Active storage transaction model

Active slots, commit markers, candidate freshness and promotion are defined in
[the storage contract](../architecture/calibration_storage.md). Configuration
commands must use that transaction rather than write independent keys.

## Ownership rules

Persisted:

- board/runtime defaults selected by the user;
- IMU/FIFO configuration;
- AHRS tunables;
- gyro bias and gyro temperature compensation;
- accel calibration;
- mag axis/calibration/yaw-correction policy;
- output policy;
- the active `sensorToDevice` proper rotation plus neutral compatibility-reserved bytes;
- network/SlimeVR configuration in its own NVS namespace, including optional physical-tap UserAction mapping.

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
3. Sanitize only defaults or a CRC-proved legacy load, then require non-mutating
   semantic validation before runtime apply.
4. Invalidate only the calibration block that is actually bad when possible.
5. Add or update native tests for defaults/sanitize/CRC/layout expectations.
6. Never sanitize a new CLI/setup/background candidate immediately before save;
   reject it unchanged and retain the old-good generation.
7. Update this document and `docs/reference/cli.md` if commands or side effects change.

Current config hardening is intentionally low-cost: compile-time/native-test layout guards and host tests are preferred over runtime-heavy migration logic. Keep network credentials in the separate `TrackerNetworkConfig` namespace so IMU/calibration resets do not expose or erase Wi-Fi secrets by accident.

The default LSM SPI clock for new/default configs is 8 MHz and the non-Slim
watermark default is 18 words. Existing valid NVS values are deliberately
preserved so performance updates cannot erase or silently reinterpret calibrated
trackers. Users may opt in with `config spi 8000000 save` and
`fifo watermark 18 save`; LSM startup retries at 4 MHz if the higher clock fails.
These live changes are transactional: failed runtime apply or NVS save restores the
previous config. Watermark reconfiguration clears pre-change software queues and
requests controlled orientation recovery. No schema, persisted layout or total
blob-size change is required.

## Current frame and gyro-temperature semantics

`frame.sensorToDevice` is persisted and applied only as a physical
sensor-to-device proper rotation. Finite legacy matrices remain loadable so one
stale frame cannot discard the whole NVS blob; runtime ignores non-rotations and
sanitize disables scale, shear, reflection or non-finite values without clearing
unrelated calibration. The existing frame
schema layout is unchanged.

The gyro block stores a static bias, reference temperature, optional three-axis
temperature slope and quality/range metadata. Static-bias replacement explicitly
invalidates the old temperature model. Temperature clear preserves the static
bias while clearing slope/range metadata. Runtime capture persists
`tempCompValid` only when the in-memory model is explicitly valid.

The guided setup defines device axes as `+X` right, `+Y` forward and `+Z` top/outward. `sensorToDeviceValid` becomes true only after the two-position frame solver produces a finite right-handed proper rotation. No schema bump is required because the frame fields already existed; older configs remain loadable and simply report the frame stage as missing until setup is resumed.


## Network-config tap action semantics

Patch 0020 assigns one previously reserved byte in the existing network-config v1
blob to `tapUserAction`. Zero remains the backward-compatible default (`off`), so
the layout size and schema version do not change. Allowed persisted values are the
SlimeVR UserAction wire values `2` full reset, `3` yaw reset, `4` mounting reset
and `5` pause; every other value sanitizes to `off`. Network-config saves remain
transactional, and runtime tap mapping changes only after a successful persisted
write when the command includes `save`.


The selector remains authoritative while valid. A non-selected marker-based v2/v3 slot
without a matching marker is prepared/uncommitted and cannot become fallback.
With a lost selector, only commit-authoritative slots participate in recovery.
Selector verification failures are reconciled by reading NVS again;
`CommitUncertain` is surfaced when authority cannot be proven. A storage/read
failure at boot latches all persistent active/candidate mutations until a later
`config load` is both read successfully and applied to hardware/runtime. Read-only
`config verify` deliberately cannot clear that latch. Full `config erase` remains
the explicit destructive recovery operation.

Calibration epoch changes and model/evidence ownership are specified in
[calibration storage](../architecture/calibration_storage.md).

## runtime-only magnetic reliability state

Field-reliability thresholds, large-error reacquisition tuning and
continuous-axis collector scheduling are bounded firmware defaults. The only
persistable result is an ordinary calibration candidate using the existing
model/signature/evidence transaction. Active calibration changes only after an
explicit candidate promotion.
