# Current implementation baseline

This is the canonical short description of what the `Upgrades` branch actually
implements. Detailed design documents remain useful, but this file and the
source code take precedence over historical roadmaps.

Firmware identity is generated automatically by the PlatformIO pre-build hook
`tools/generate_build_identity.py`. It never modifies the Git index and writes
its generated header only under the ignored PlatformIO build directory.

A clean build reports the committed revision:

```text
build=BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG git=89cd10ef
```

An uncommitted patch iteration reports both the base commit and a deterministic
fingerprint of tracked plus non-ignored untracked worktree content:

```text
build=BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG git=12ab34cd+7f93a2c1-dirty
```

This matches the project workflow where a patch is committed only after its
corrective revisions and hardware tests are complete. Ignored files, `.git`,
`.pio`, build outputs and the generated identity header do not affect the
fingerprint. There is deliberately no build timestamp, so rebuilding identical
content produces identical firmware metadata.

## Build matrix

Committed PlatformIO environments:

```text
BOARD_LOLIN_C3_MINI_DEBUG
BOARD_LOLIN_C3_MINI_PRODUCTION
BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG
BOARD_LOLIN_C3_MINI_SLIM
```

The additional `BOARD_LOLIN_C3_MINI_DEBUG_LINKCHECK` environment is internal to
`check_all`: it uses the complete Debug source set with a larger no-OTA app
partition and is not intended for normal upload.

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
- firmware/server FeatureFlags exchange and packet-100 bundle negotiation;
- SetConfigFlag for the runtime magnetometer/yaw toggle;
- ProtocolChange accounting without a protocol switch.

## Known limitations and planned corrections

These are current-code limitations, not statements that the corresponding
feature is already implemented.

### Frame pipeline

Only `sensorToDevice` is an active firmware frame setting. It is runtime-gated
as a proper right-handed rotation and configured by the guided setup. Device axes
are `+X` right, `+Y` forward and `+Z` top/outward. In a full calibration, the
first two normal accel six-position captures are reserved for top/+Z and
forward/+Y, so alignment adds no extra face captures. The solver separates any
proper rotation absorbed by the full accel matrix from sensor scale and
non-orthogonality before saving the frame. The transform is applied after
gyro/accel calibration and after magnetic `magToImu` alignment.

Changing calibration or frame state performs a complete deliberate orientation
reset: the AHRS quaternion and prepared output are invalidated, magnetic dependent
state is cleared, and the next plausible accel sample reacquires roll/pitch in
the new frame. Body mounting and recenter remain server-owned. Compatibility
bytes from abandoned firmware mounting/output/duplicate-identity concepts are
forced neutral and are not exposed as capabilities.

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

Recovery is reason-aware. A bounded FIFO full/overrun resets the broken hardware
stream, rebases AHRS time and clears dependent magnetic heading state, but it does
not suppress orientation until the tracker becomes still. Output resumes on the
next genuinely integrated sample while normal adaptive accel correction repairs
tilt and magnetic yaw reacquires naturally. A 32-sample clean-stream window keeps
the state visible as `DEGRADED_TIMING` without blocking movement. Timestamp
corruption, manual resets, runtime reconfiguration and blocking operations still
enter strict recovery once a valid orientation exists: output remains invalid while
post-gap gyro prediction continues with accel correction disabled, then 256
stationary near-1 g samples rebuild roll/pitch while preserving horizontal heading.
A startup-time reconfiguration before the first quaternion bypasses recovery and
continues through normal gravity-based startup convergence, because there is no
prior orientation to recover. This prevents a single
rare FIFO loss from causing a minutes-long blackout without weakening handling of
truly unreconstructable intervals.

### Cooperative FIFO/runtime cadence

Runtime FIFO work is split across app-loop passes. A hardware drain keeps the
configured `maxWordsPerDrain` safety ceiling, while decoded IMU/magnetometer
callbacks are executed in bounded slices. Hardware SPI read time is excluded
from the callback budget: each pass processes at least 12 and at most 64 raw
callbacks, yielding after about 3.5 ms once minimum forward progress is met.
Production keeps a 512-sample raw RAM ring and a larger urgent catch-up budget;
Slim retains the smaller 256-sample ring.
The remaining local batch is retained for the next pass, allowing Wi-Fi/UDP,
battery, tap and LED services to run between slices without starving FIFO
processing. Long `status`, `health`, `slime status` and `slime debug` reports
also service the non-CLI runtime between output sections, so diagnostics cannot
fill the hardware FIFO merely by printing. Configured drain round limits remain
intact. External/manual FIFO resets explicitly discard any local pre-reset batch.

### Motion output and new server features

Valid motion snapshots use negotiated packet-100 output when the server
advertises bundle support. The bundle contains float32 packet 17 followed by
float32 packet 4 from one prepared snapshot. Hard-invalid acceleration falls
back to rotation-only packet 17; rotation is never suppressed only because
motion acceleration is unavailable. Servers without bundle support receive
packet 17 at pose rate and coherent packet 4 at a 50 Hz fallback rate. Packet 23
remains an explicitly disabled experimental compile-time option. Internal
acceleration remains in `g` and is converted to SI `m/s^2` at the packet
boundary. The handshake uses protocol 22, so the server accepts corrected
device-frame acceleration without the historical extra -90 degree local-Z
correction. Rotation and acceleration therefore share the same local basis and
sample boundary, which is the firmware-side requirement for acceleration-based
step mounting. Position packet 27 is not implemented and must not be synthesized
by double-integrating IMU acceleration.

### Session-completeness boundary

- SensorInfo acknowledgement packet 15 is parsed in its special six-byte form
  and drives explicit dirty/waiting/acknowledged state. Local changes make the
  state dirty again and reconnect clears acknowledgement state.
- Firmware/server FeatureFlags are negotiated explicitly; empty or malformed
  responses do not establish capability state. Packet 100 is selected only
  after server support is confirmed.
- SetConfigFlag type 1 applies and persists the magnetometer/yaw state before
  packet 24 acknowledgement. Idempotent retries do not repeat the NVS write.
- UserAction packet 21 is available through `slime action ...`; optional
  physical-tap mapping defaults to off and lives in network config.
- Packet 23 has no compatible legacy negotiation path and remains disabled by
  default; only an explicitly verified server build should enable it.
- ProtocolChange is validated and recorded but intentionally not applied.

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
