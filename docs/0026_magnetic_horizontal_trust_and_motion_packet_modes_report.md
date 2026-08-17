# 0026 magnetic horizontal trust and motion packet modes

## 1. Scope and predecessor

Patch ID: `0026_magnetic_horizontal_trust_and_motion_packet_modes`.

Exact predecessor/base commit:

```text
e0034b5ade299a4ae13057bf9020926040d4e8ba
```

The predecessor contains `0025c_magnetic_stationary_field_recovery_hardening`.
The user explicitly requested one patch containing both the magnetic
observability fix and the persisted SlimeVR motion-packet selector. No unrelated
calibration, AHRS equation, FIFO, Wi-Fi recovery or remote-console change is in
scope.

## 2. Confirmed defects and goals

### 2.1 Healthy high-dip field was permanently rejected

Hardware evidence showed a calibrated and axis-aligned field with approximately:

```text
body norm       443.9
reference norm  441.0
dip              -70.5 deg
horizontal norm  148.5
```

The vector is internally consistent (`horizontal ~= norm * cos(dip)`), but the
fixed `horizontalNormBad/Good=200/260` gate assigned zero horizontal trust.
Consequences were:

```text
reject_horizontal_bad on every yaw update
mag_auto_ref_last_reject_flags included horizontal rejection
mag_ref_valid=no
yaw gate never opened
stationary heading noise could repeatedly latch field disturbance
```

Waiting could not repair an absolute threshold that was above the normal local
horizontal field.

### 2.2 Motion wire policy was not user-selectable or persistent

The runtime already implemented packet 17, packet 4, negotiated packet-100
bundles and the packet-23 encoder, but the selected motion format was controlled
by compile-time policy/negotiation. The requested product contract is:

```text
quaternion  -> packet 17 only (default)
bundle      -> negotiated packet 100 containing 17 + 4,
               with existing separate 17/4 fallback
packet23    -> RotationAndAcceleration packet 23
```

The selected mode must survive reboot.

## 3. Root cause

Magnetic field reliability, heading auto-reference and yaw correction each used
an absolute horizontal-norm gate. That gate ignored local magnetic inclination:
a healthy field can have a strong total norm but a much smaller horizontal
component at high dip. The stationary jump detector also used fixed angular and
rate thresholds even though heading noise grows as horizontal observability
falls.

`TrackerOutputConfig::packetFormat` was an unused/deprecated byte that
sanitization forced to zero. The active runtime packet-mode selection did not
read a persisted policy.

## 4. Implementation

### 4.1 Shared adaptive horizontal trust

`src/sensor/mag_horizontal_trust.hpp` owns one pure policy used by the magnetic
pipeline. The field monitor acquires and slowly adapts the actual horizontal
component alongside total norm and dip, so no per-update trigonometric
reconstruction is required.

Configured `horizontalNormBad/Good` remain conservative upper caps. Once a field
reference exists, effective thresholds may be lowered to:

```text
bad  = max(20, 0.35 * reference horizontal norm)
good = max(40, 0.65 * reference horizontal norm)
```

For the reported hardware field this changes the effective range from 200/260
to about 52/97, so a normal horizontal norm near 149 receives full trust.

The same result is passed to:

- field reliability;
- heading auto-reference;
- yaw correction.

A field with insufficient horizontal observability closes magnetic yaw using
`MAG_FIELD_FLAG_HORIZONTAL_UNOBSERVABLE` without falsely declaring norm/dip
environment corruption.

Stationary heading-jump angle/rate thresholds scale with:

```text
sqrt(reference total norm / reference horizontal norm)
```

clamped to `1..2`, and jump detection is disabled below horizontal trust 0.25.
This preserves fail-closed rejection of real larger discontinuities while
avoiding repeated latches from ordinary high-dip heading noise.

### 4.2 Persisted motion packet policy

The unchanged one-byte `TrackerOutputConfig::packetFormat` storage is reused with
an explicit high marker bit. This keeps `TrackerConfigBlob` at 756 bytes and does
not alter calibration storage.

```text
marker clear                 -> historical value, migrate to quaternion-only
marker + value 0             -> quaternion-only
marker + value 1             -> bundle 17+4 policy
marker + value 2             -> packet 23 policy
unknown/reserved bits/value  -> quaternion-only
```

The command is:

```text
slime motion-mode quaternion
slime motion-mode bundle
slime motion-mode packet23
```

Every successful command transactionally saves the candidate, updates active
RAM config, applies the runtime policy without tearing down the discovered
server session, and confirms the authoritative config application.

The default runtime/config policy and every historical unmarked value are
quaternion-only.

### 4.3 Effective wire behavior

- `quaternion`: packet 17 only; no acceleration packet or motion bundle.
- `bundle`: packet 100 with float32 packet 17 then packet 4 after FeatureFlags
  negotiation. Without bundle support, packet 17 remains at pose rate and packet
  4 uses the existing coherent fallback cadence.
- `packet23`: existing Q15 quaternion + Q7 acceleration encoder.
- Invalid acceleration in either acceleration policy falls back to packet 17.

Selected policy and effective mode are separately visible in `slime status` and
`slime debug`.

## 5. Ownership

```text
sensor/mag_horizontal_trust.hpp  pure observability policy
sensor/mag_field_reliability.*   reference, disturbance and jump/recovery state
sensor/mag_yaw_correction.*      consumes shared trust and applies yaw gate
runtime/mag_runtime_controller.* orchestration and shared snapshot wiring
config/*                         persisted mode encoding/migration
runtime/slimevr_output_runtime.* effective packet scheduling/encoding choice
serial/tracker_slimevr_commands  CLI parsing and transactional persistence
app/hooks/*                      maps persisted policy into runtime config
```

No packet encoding was moved into CLI/config code and no magnetic algorithm was
moved into runtime/serial code.

## 6. Compatibility and preserved invariants

### Persistent/config

- `sizeof(TrackerConfigBlob)` remains 756 bytes.
- `CONFIG_VERSION` and all calibration schema/layout bytes are unchanged.
- Old unmarked `packetFormat` values migrate to quaternion-only on sanitize.
- Unknown marker/reserved combinations fail closed to quaternion-only.
- Gyro, accel, mag calibration, `sensorToDevice`, candidates, provenance and
  revisions are untouched.

### Wire/session

- Handshake protocol version remains 22.
- Packet 17, packet 4, packet 23 and packet 100 byte encoders are unchanged.
- Packet numbering, SensorInfo, FeatureFlags and session ownership are unchanged.
- Changing motion policy only resets motion deadlines; discovery/session is
  preserved.

### Tracking/realtime

- Gyro integration, accel correction, magnetic heading equations, yaw correction
  limits, FIFO order/timestamps and prepared snapshot content are unchanged.
- No heap allocation, blocking wait, NVS write or formatting enters the magnetic
  sample callback.
- The horizontal reference is accumulated/adapted with existing field-reference
  work; yaw/auto-reference reuse the resulting scalar snapshot.

### CLI

The new command always persists. `slime rate`, local `output start/stop/mode` and
serial stream commands no longer overwrite the reused packet-policy byte.

## 7. Regression and fault coverage

Native behavioral tests cover:

- the reported norm/dip/horizontal high-dip field becoming trusted;
- effective thresholds below the healthy local horizontal field;
- a four-degree high-dip wobble not latching disturbance;
- a larger ten-degree discontinuity remaining fail-closed;
- legacy standalone yaw-controller behavior;
- config default, valid-mode round trips and legacy/reserved-bit migration;
- exact packet-17-only behavior;
- explicit packet-23 behavior;
- transactional CLI saves and reloads of all three modes;
- invalid CLI input leaving active/persisted mode unchanged;
- existing bundle, fallback, negotiation and TX-recovery regression suites.

## 8. Performance, RAM and stack impact

The config blob is unchanged. Host ABI size comparison against the exact
predecessor measured:

```text
TrackerConfigBlob                 756 -> 756
MagFieldReliabilityOutput          84 -> 112
MagFieldReliabilityMonitor        280 -> 320
MagYawCorrectionInput             272 -> 296
MagYawCorrectionOutput            112 -> 128
SlimeVROutputRuntimeConfig        112 -> 112
SlimeVROutputRuntimeStatus        784 -> 792
```

New state is fixed-size scalar data only: one cached reference-horizontal scalar,
one acquisition accumulator, diagnostic snapshot scalars and one packet-policy
enum.

Optimized host `-fstack-usage` comparison against the predecessor measured:

```text
MagFieldReliabilityMonitor::update       80 -> 176 bytes
MagYawCorrectionController::update       48 -> 48 bytes
MagRuntimeController::updateYaw...      416 -> 448 bytes
SlimeVROutputRuntime::sendRotation      192 -> 192 bytes
trackerSerialDispatchSlimeVRCommand     848 -> 880 bytes
```

The hot magnetic path performs bounded arithmetic and one bounded square root per
magnetic update; repeated yaw/auto-reference geometry recalculation was removed.
There is no loop, allocation or new call into storage/network/CLI from the sensor
path. All measured functions remain below the project's 1024-byte focused stack
ceiling on the host ABI. ESP32-C3 target stack and firmware-size deltas remain a
toolchain acceptance gate because PlatformIO is unavailable here.

## 9. Verification record

Completed:

```text
python3 tools/check_all.py --clean --skip-pio \
  --native-suite-timeout-s 900 \
  --native-command-timeout-s 300 \
  --tool-timeout-s 180
PASS: complete host gate, 48/48 native tests, validators, policies and replay
Status: host-verified, target build not verified
```

Focused `-Werror` runs passed for:

```text
test_config_schema_detail
test_config_hardening
test_mag_heading_reliability
test_mag_yaw_runtime_bias
test_slimevr_output_runtime
test_slimevr_motion_mode_command
```

A final full standalone sanitizer gate passed all 48 native executables with
`-Werror` and UBSan (`--sanitizer=undefined`).

Clean-application verification on a separate worktree at the exact predecessor
passed:

```text
git apply --check                         PASS
git apply                                 PASS
git diff --check                          PASS
git apply --reverse --check               PASS
all native and source/profile/policy gates PASS
all replay gates                           PASS
```

The predecessor archive contains `tools/logs/parse_e0_log.py`, but the repository
`.gitignore` excludes the generic `logs` directory, so that pre-existing replay
helper is not represented by the Git commit. It was retained from the supplied
source archive when executing the clean-apply replay gates; this patch does not
modify or depend on a changed parser implementation.

Not verified here:

- PlatformIO Debug/Production/ProductionDiag/Slim builds;
- ESP32-C3 firmware size and target stack usage;
- real QMC high-dip recovery;
- real SlimeVR server packet-23 compatibility;
- reboot persistence on physical NVS;
- long target performance/TPS comparison.

## 10. Hardware acceptance

### Magnetic field

1. Flash ProductionDiag and keep the tracker in the previously failing location.
2. Run:

```text
mag heading clear
```

3. Keep it still for at least 15 seconds, then run:

```text
mag heading
mag yaw status
mag processed
```

Require:

```text
field_reference_valid=yes
field_horizontal_norm near the measured local value
field_horizontal_effective_bad_good below that value
field_horizontal_trust > 0.7
field_state=trusted
field_trusted_for_yaw=yes
mag_auto_ref_done=yes
mag_ref_valid=yes
yaw_gate_open=yes when other gates are valid
```

4. Introduce and remove a directional disturbance. Require fail-closed entry and
bounded return, with no repeated stationary-jump latch in the clean high-dip
field.

### Motion packet policy and persistence

For each mode:

```text
slime motion-mode quaternion
slime motion-mode bundle
slime motion-mode packet23
slime status
config print
```

Reboot after each selection and require the same `motion_packet_policy`.

- Quaternion mode: rotation counter grows; acceleration/bundle/packet23 counters
  do not grow.
- Bundle mode on a bundle-capable server: `motion_packet_mode` becomes
  `bundle_100_rotation_17_accel_4` and bundled counter grows. On an older server,
  effective mode becomes the documented 17+4 fallback.
- Packet23 mode: `motion_packet_mode=rotation_acceleration_23` and compact-motion
  counter grows; tracking remains accepted by the target server version.

For all modes require normal server TPS, zero unexplained UDP failures, zero FIFO
overrun/full and no increase in prepared/rotation age.

### Long target gate

Run `perf on` and `test runtime 600` in quaternion and bundle modes, then at least
one 30-minute packet23 run. Compare FIFO, queue age, output deadline, UDP pressure,
free heap and tracker TPS.

## 11. Rollback

Runtime/product rollback without reflashing:

```text
slime motion-mode quaternion
```

Code rollback on the exact predecessor:

```bash
git apply --reverse --check 0026_magnetic_horizontal_trust_and_motion_packet_modes.patch
git apply --reverse 0026_magnetic_horizontal_trust_and_motion_packet_modes.patch
```

Reversing the patch does not require calibration reset. A config written by this
patch has the same blob size/version; the predecessor sanitizes its non-zero
legacy `packetFormat` byte back to zero.

## 12. Known limitations

- Packet23 remains an explicit user choice; there is no server capability bit
  proving compatibility.
- Near-vertical fields below the absolute horizontal floors remain intentionally
  unusable for yaw.
- Adaptive thresholds are reference-relative, so a bad reference must still be
  cleared/reacquired in a clean environment.
- Hardware/HIL evidence is required before release-ready status.
