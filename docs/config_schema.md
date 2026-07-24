# Config schema policy

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

Core config no longer depends on one replace-in-place `cfg` blob. Active state is
stored in `cfg_a`/`cfg_b`, with `cfg_s` selecting one verified generation and
`cfg_ac`/`cfg_bc` proving storage-v2 per-slot commit completion. The target marker
is removed before the inactive record is written; selector and matching marker
are committed only after read-back validation. This distinguishes a complete but
never-committed first slot from an authoritative generation. Compatible v1 slots
remain readable and upgrade on the next real save. A valid legacy `cfg` blob
migrates only after the new slot, selector and marker verify. See
[calibration_storage.md](calibration_storage.md).

Calibration candidates use a separate `cfg_c` record and never become active by
being written. New candidates bind freshness to a hash/revision of calibration-owned
fields rather than the whole config generation; unrelated product-policy saves do
not discard useful collection. Promotion requires signature/quality comparison,
live hardware apply and final selector/commit-marker completion. Candidate writes
are throttled independently from explicit user config saves, and a promoted
candidate is terminal until a new stage starts.

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
3. Sanitize loaded values before applying them to runtime.
4. Invalidate only the calibration block that is actually bad when possible.
5. Add or update native tests for defaults/sanitize/CRC/layout expectations.
6. Update this document and `docs/cli_reference.md` if commands or side effects change.

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


The selector remains authoritative while valid. A non-selected storage-v2 slot
without a matching marker is prepared/uncommitted and cannot become fallback.
With a lost selector, only commit-authoritative slots participate in recovery.
Selector verification failures are reconciled by reading NVS again;
`CommitUncertain` is surfaced when authority cannot be proven. A storage/read
failure at boot latches all persistent active/candidate mutations until a later
`config load` is both read successfully and applied to hardware/runtime. Read-only
`config verify` deliberately cannot clear that latch. Full `config erase` remains
the explicit destructive recovery operation.

- 0021d separates configuration persistence, calibration snapshots and calibration events; keeps no-op saves/provenance stable; scopes component saves; uses v3 model-only candidate freshness with v1/v2 compatibility; guards unsaved runtime calibration; resets dependent adaptive state on load/promotion/clear; and refreshes SensorInfo when advertised calibration or sensor capability changes.


## 0021e calibration epoch and field-ownership hardening

0021e finishes the patch-21 integration audit without changing the persisted blob
layout. Calibration models, their evidence, runtime policy and unfinished capture
workspaces now have explicit owners and epoch boundaries:

- invalid gyro, temperature, accel, magnetic and frame models sanitize to canonical
  fail-honest values and clear only their own evidence; independent enable policy is
  preserved;
- a disabled `sensorToDevice` frame is logically identity and dormant matrix bytes no
  longer create false sensor-signature mismatches;
- candidate promotion is rejected when the live unsaved ODR/full-scale/FIFO/frame
  contract differs from the active signature, because calibration-only promotion does
  not reconfigure hardware;
- full config apply, promotion, setup commit/rollback and full calibration erase clear
  incompatible accel, gyro-temperature and magnetic collection workspaces;
- real gyro/temperature correction changes start a new AHRS/runtime-bias epoch, while
  no-op toggles do not disturb tracking;
- guided accel+frame calibration keeps acceleration unavailable until both new models
  are valid, and guided production policy is applied to live AHRS/quality state before
  readiness and persistence;
- manual hard/soft-iron replacement invalidates the previous mag-to-IMU alignment and
  magnetic-yaw apply state; candidate promotion remains atomic because it transfers the
  field and axis models together;
- `cal clear_all` and `ERASE CALIBRATION` also clear the device frame, and persistent
  erase discards any staged candidate that could otherwise resurrect the old model.

Compact and detailed status now report `sensor_to_device_valid` and
`motion_frame_config_ready` separately from `accel_cal_valid`.
