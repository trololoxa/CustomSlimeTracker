# Current implementation baseline

This is the canonical short description of what the `Upgrades` branch actually
implements. Detailed design documents remain useful, but this file and the
source code take precedence over historical roadmaps.

Do not assign a hand-written firmware identity to an archive. Use Git to record
the exact source revision being built:

```bash
git branch --show-current
git rev-parse --short HEAD
git status --short
```

A test report or runtime log is reproducible only when it records the branch,
commit and build environment.

## Build matrix

Committed PlatformIO environments:

```text
BOARD_LOLIN_C3_MINI_DEBUG
BOARD_LOLIN_C3_MINI_PRODUCTION
BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG
BOARD_LOLIN_C3_MINI_SLIM
```

The committed default is:

```text
BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG
```

Production Diagnostic uses the Production feature profile plus the live
`perf`/`motion` profiler. It is the normal wearable diagnostic build, not a
separate tracking algorithm. The complete local quality gate builds all four
environments when PlatformIO is available.

## Runtime tracking pipeline

The current orientation backend is the project-owned `Ahrs6Dof`; the firmware
does not use VQF, SH-2, InvenSense DMP or LSM6DSV SFLP as its production
orientation source.

```text
LSM6DSV FIFO gyro/accel/timestamp/temperature tags
  -> FIFO tag and timestamp reconstruction
  -> raw-to-SI scaling
  -> persisted gyro/temperature and accel calibration
  -> IMU quality decisions
  -> Ahrs6Dof gyro prediction + adaptive gravity correction
  -> optional QMC6309 yaw-only correction
  -> prepared quaternion snapshot
  -> SlimeVR RotationData packet 17
```

The FIFO path uses LSM6DSV hardware timestamps and tracks timestamp faults,
tag-counter discontinuities, FIFO overrun/full states, saturation and recovery.
The SlimeVR runtime consumes prepared snapshots and does not read the AHRS or
FIFO directly.

## Calibration actually applied

Runtime calibration currently includes:

- persisted gyro zero-rate bias;
- direct three-axis linear gyro bias-versus-temperature model;
- bounded RAM-only runtime residual gyro trim when its stationary gates pass;
- six-position accelerometer affine calibration with a full 3x3 matrix;
- QMC6309 hard-iron vector and full soft-iron matrix;
- a separate magnetometer-to-IMU axis-alignment matrix;
- conservative yaw-only magnetic correction with trust and cooldown gates.

`setup calibration` is the user-facing transactional procedure. It keeps the
previous active calibration until the complete candidate passes and is saved.
Low-level `cal ...` and `mag ...` commands remain service/developer interfaces.

## Build/profile behavior

- Debug keeps the full CLI, tests, machine log, streams and live diagnostics.
- Production keeps user setup/calibration, Wi-Fi remote console, SlimeVR and
  product telemetry while excluding heavy developer reporters/tests.
- Production Diagnostic is Production plus `perf` and `motion` live diagnostics.
- Slim assumes valid NVS provisioning/calibration and removes interactive CLI,
  remote console, setup/calibration code, LED and tap runtime while preserving
  the tracking and SlimeVR path.

Profile/source-filter policy is enforced by:

```bash
python tools/validate_source_filters.py
python tools/validate_profile_matrix.py
python tools/validate_documentation.py
```

## SlimeVR protocol currently implemented

Outgoing runtime packets include:

- handshake/discovery;
- SensorInfo;
- RotationData;
- heartbeat and PingPong response;
- BatteryLevel when enabled and valid;
- Tap when enabled;
- Error when tracker health requires it;
- SignalStrength using signed RSSI dBm in the one-byte payload;
- Temperature;
- AcknowledgeConfigChange.

Incoming handling includes:

- discovery response;
- heartbeat;
- PingPong;
- server FeatureFlags storage for diagnostics;
- SetConfigFlag for the runtime magnetometer/yaw toggle;
- ProtocolChange accounting without a protocol switch.

## Known limitations and planned corrections

These are current-code limitations, not statements that the corresponding
feature is already implemented.

### Frame pipeline

Persistent config contains `sensorToDevice`, `mountingOffset` and output-frame
fields. `sensorToDevice` is now runtime-gated as a proper right-handed rotation
and applied after gyro/accel calibration and after magnetic `magToImu` alignment.
Finite legacy non-rotations remain loadable but are ignored and sanitized on save.
The default remains disabled/identity, so existing trackers retain their previous
orientation until a valid transform is configured.
The current AHRS therefore operates in the device frame whenever a valid
transform is configured, and in the legacy sensor frame when the transform is
disabled. Body mounting and recenter remain server-owned.

### Calibration lifecycle and temperature capture

The dedicated setup temperature capture now commits data only after short
contiguous stationary windows pass gyro mean/variance, accel norm/variance,
accel-confidence, temperature-span and FIFO/timestamp quality gates. A brief
touch or quality fault discards only the current sub-second candidate window;
previously accepted temperature-bin progress remains available and collection
continues automatically when the tracker is still again. Gyro static-bias
replacement invalidates the dependent temperature model, while temperature
clear preserves the static bias and removes slope/range metadata. Both paths
reset the volatile runtime trim.

### Recovery

FIFO/timestamp recovery restores normal sample processing and rebases AHRS
timestamps. It does not yet provide a dedicated large-error roll/pitch
reacquisition mode after motion was missed during an unreconstructable gap.

### Motion output and new server features

Acceleration packet 4 is not emitted. The prepared output snapshot currently
contains orientation but no timestamp-coherent linear acceleration, so step
mounting is not ready. Position packet 27 is also not implemented and should not
be synthesized by double-integrating IMU acceleration.

### Remaining protocol work

- SensorInfo acknowledgement packet 15 is not tracked as an acknowledged-state
  machine; SensorInfo is refreshed periodically or when requested locally.
- Firmware FeatureFlags packet 22 is not sent, so bundle capability is not
  negotiated.
- Server FeatureFlags are stored but do not enable optional behavior.
- ProtocolChange is recorded but not applied.
- Control-packet source endpoint validation still needs hardening.

## Test policy

Native tests and project-contract validators are the default proof for pure
math, state machines, packet semantics, profile filters and documentation.
PlatformIO compilation proves all committed build compositions. Hardware tests
are reserved for behavior that cannot be represented on the host: real FIFO
interrupt/timing, sensor-hub traffic, NVS/Preferences, Wi-Fi/UDP integration,
sleep/wake and physical calibration capture.

A documentation/profile/tooling patch requires no tracker runtime test. A patch
that changes one hardware boundary should normally request one short focused
serial/telnet smoke test after all host checks pass, not a collection of long
ideal-condition captures.
