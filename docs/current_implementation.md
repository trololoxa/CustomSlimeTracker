# Current implementation baseline

This is the canonical short description of what the `Upgrades` branch actually
implements. Detailed design documents remain useful, but this file and the
source code take precedence over historical roadmaps.

Firmware identity is generated automatically by the PlatformIO pre-build hook
`tools/generate_build_identity.py`. It never modifies the Git index and writes
its generated header only under the ignored PlatformIO build directory.
Successful aggregate target builds also create ignored per-environment JSON
manifests with the full commit, dirty state, environment, UTC time, PlatformIO
version and SHA-256/size for emitted firmware artifacts. Release mode refuses
unknown or dirty source identity, unavailable tool identity, empty artifacts
and target output that survives the mandatory pre-build clean.

A clean build reports the committed revision:

```text
build=BOARD_LOLIN_C3_MINI_PRODUCTION git=89cd10ef89cd10ef89cd10ef89cd10ef89cd10ef
```

An uncommitted patch iteration reports both the base commit and a deterministic
fingerprint of tracked plus non-ignored untracked worktree content:

```text
build=BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG git=12ab34cd12ab34cd12ab34cd12ab34cd12ab34cd+7f93a2c1-dirty
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

The committed default is the locked-down product image:

```text
BOARD_LOLIN_C3_MINI_PRODUCTION
```

Production Diagnostic has a distinct profile identity and Production-family
scheduling, but links the complete bounded diagnostic/capture surface. It is an
explicit service image, not the product default or a separate tracking
algorithm. The complete target gate builds those four upload
profiles plus DebugLinkcheck when PlatformIO is available. The
`check_all.py --host-only` command runs the complete host gate without making a
target-build claim; `check_all.py --release` additionally requires clean Git
identity, explicit
ASan/UBSan and LSan matrices, a strict LOGVER3 static golden gate, all five
target builds and release manifests. The release interface is intentionally
blocked until a real clean cable-free LOGVER3 fixture and independent golden
JSON are added. The strict parser/gate itself is already installed.

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

`setup calibration` is the user-facing transactional procedure. Core config now
uses CRC-protected dual active slots, a selector and storage-v2 per-slot commit
markers, so the previous generation remains available after torn writes. A
separate signed/quality-tagged candidate is bound to the active calibration
revision, can survive unrelated config saves, and can be atomically promoted
without changing tracking before selector commit.
Low-level `cal ...` and `mag ...` commands remain service/developer interfaces.


## Calibration storage safety

The old single `tracker/cfg` blob is migration-only. Active config is stored in
two complete slots plus a CRC-protected selector and per-slot commit markers.
Every save invalidates the target marker, writes and verifies the inactive slot,
commits the selector, then writes the new marker. Boot falls back only to a slot
with commit authority. Present-but-corrupt storage is distinguished from empty
NVS, and a boot read error latches persistent writes off until an authoritative
config is successfully applied to hardware/runtime. Candidate calibration is
stored separately, bound to IMU/mag/ODR/full-scale/frame/schema signature and the
active calibration revision, and remains inactive until explicit two-phase
promotion succeeds. No-op saves skip flash and generation changes.

## Build/profile behavior

- Debug keeps the full CLI, tests, machine log, streams and live diagnostics;
  dormant timing code does not call timers on every sample/loop, and slow
  network/config telemetry cadence matches ProductionDiag.
- Production keeps user setup/calibration, SlimeVR and product telemetry while
  excluding the TCP listener and heavy developer reporters/tests.
- Production Diagnostic is the complete cable-free capture image: full
  diagnostics plus TCP/USB command parity, distinct profile identity and
  Production-family scheduling.
- Slim assumes valid NVS provisioning/calibration and removes interactive CLI,
  remote console, setup/calibration code, LED and tap runtime while preserving
  the tracking and SlimeVR path.

Profile/source-filter policy is enforced by:

```bash
python tools/validate_source_filters.py
python tools/validate_profile_matrix.py
python tools/validate_documentation.py
```

## Cable-free LOGVER3 foundation

Machine-log and test output bind to the initiating USB/TCP stream and remote
session. Disconnect runs an explicit close hook before detach; owned producers
abort and never fall back to USB. Production compiles the TCP listener out,
while Debug/ProductionDiag feed USB and TCP into the same profile-specific
command dispatcher with the same command, rate and duration limits.

IMU/mag/runtime-bias callbacks enqueue fixed immutable records. Background
service serializes at most one complete CSV line per admitted loop after
tracking/network work. `test runtime` samples section timing at 1/16 cadence;
`test static` aggregates expensive moments in bounded 64-sample blocks. Both
retain immutable results and emit only a compact completion marker from the
measured path; full reports are explicitly requested later over USB.

`tools/capture_telnet_log.py` performs ProductionDiag/known-identity and
magnetometer preflight, accepts clean or fingerprinted dirty builds, runs a full 20 Hz static session for 1..21600 seconds, drains the pipeline,
executes the strict LOGVER3 schema/chronology/drop gate and writes a SHA-256
manifest. Release is still blocked because no real positive fixture or reviewed
golden thresholds can be produced without the hardware run.

0025a upgrades this to LOGVER3 E1. A deferred one-hertz `NET` frame makes
Wi-Fi/SlimeVR send, deadline and recovery state visible, while an explicitly
requested post-window `TESTSUM` carries exact test deltas. Static capture
requires semantically ready MAG/BIAS/UDP state and fails closed; runtime capture
preserves a structurally valid failing session with `health_passed=false`.
Accepted TCP sessions protect preflight from motion sleep and use a five-second
host keepalive plus 30-second firmware lease to bound half-open cleanup. The
logger reuses the pipeline's existing temp/bias evaluation, and log/manifest
candidates are prepared as one hash-bound generation before promotion. The tool
does not enable or persist MAG/bias/yaw settings, and TCP remains subject to the
same radio/lwIP failures as SlimeVR UDP.

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

Motion output has a persisted three-mode policy with one canonical shared policy
type used by config and runtime. The `slime motion-mode` setter persists only this
field from the authoritative NVS generation, so unrelated RAM-only edits cannot
hitchhike into storage. The default and migration target is packet-17-only
quaternion output. `bundle` selects the previous
packet-100 path: a negotiated bundle contains float32 packet 17 followed by
float32 packet 4 from one prepared snapshot, while servers without bundle
support receive packet 17 at pose rate and coherent packet 4 at 50 Hz.
`packet23` explicitly selects the existing Q15/Q7 RotationAndAcceleration
encoder. Hard-invalid acceleration falls back to packet 17 in acceleration
modes; rotation is never suppressed only because acceleration is unavailable.
Internal acceleration remains in `g` and is converted at the selected packet
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


## Patch 0022 magnetic heading reliability

The current magnetic runtime inserts a temporal field-reliability monitor between
calibrated heading and yaw correction. Norm, world dip, stationary heading steps,
reference consistency and dwell/hysteresis classify the field as acquiring,
trusted, suspect, disturbed or recovering. Only trusted field is usable for yaw.
Large finite yaw errors use a separate slow reacquisition mode instead of renewing
cooldown indefinitely. A bounded runtime collector can solve a right-handed
mag-to-IMU axis candidate from coherent gyro/magnetometer motion, but it can only
stage an inactive calibration candidate; it never writes or promotes active
calibration in the tracking loop. See [magnetic_heading_reliability.md](magnetic_heading_reliability.md).

### 0022a hardening

The current implementation uses a two-stage mag-axis solve: right-handed signed
permutations establish the coarse frame and bounded `SO(3)` refinement estimates
continuous residual mounting error. Solver evidence is compared with the active
matrix on the same intervals and is persisted as measured candidate quality, so a
real improvement can use ordinary candidate comparison/promotion without `force`.
Reacquisition consumes filtered heading rate, changed environments remain
fail-closed, and solver/NVS work is deferred outside the magnetic sample callback
until FIFO has no pending or urgent work. Legacy persisted finite axis matrices
remain readable; newly generated/setup/manual mappings must be proper rotations.

### 0022b magnetic proof and field-jump behavior

The active magnetic runtime treats abrupt stationary direction changes as a
latched environment discontinuity, not as a transient suspect sample. Candidate
axis solving uses independent training/validation temporal windows and
rate-normalized confidence. Deferred candidate work requires actual hardware FIFO
slack and rotation-output deadline slack. Active calibration remains unchanged
until explicit candidate promotion.

### 0025c magnetic field recovery hardening

Stationary field-jump detection now removes AHRS yaw from its direction signal.
An AHRS yaw correction/reset therefore cannot create a false magnetic disturbance.
When a real disturbance was latched and the tracker remained physically still,
the original field may recover against the stored pre-jump stationary signal even
after 6DoF yaw drift moved the world-heading value outside the old 2.5-degree
return window. Any detected motion invalidates that relative shortcut. Norm/dip
gates, the 1.2-second return dwell, the 5-second recovery dwell and explicit new-
environment acquisition remain fail-closed and unchanged.

### 0026b hot-path ownership

The production magnetic reliability and yaw-correction callbacks consume lightweight
views of the already-owned processed magnetic/heading snapshots rather than copying
those snapshots into aggregate callback inputs. The runtime-owned field result is the
single published copy used by auto-reference/yaw; the monitor does not retain another
full output snapshot. High-dip threshold decisions compare positive squared scale
factors, preserving the 0026a thresholds while avoiding a per-sample square root;
status reporting derives the human-readable scale lazily.
Quaternion-only motion output exits after packet 17 before acceleration unit
conversion. These are execution/stack optimizations only: field/yaw decisions,
packet bytes, fallback policy, persistent settings and calibration state are unchanged.
