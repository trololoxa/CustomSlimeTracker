# 0026a motion policy and magnetic reliability hardening

## 1. Scope and predecessor

Patch ID: `0026a_motion_policy_and_magnetic_reliability_hardening`.

Exact predecessor artifact:

```text
0026_magnetic_horizontal_trust_and_motion_packet_modes.patch
SHA-256 042203c9a566f7cf96ccb27ea9fbb5ab836e1071c00233eda80d133b0ab04695
```

The predecessor report identifies its base commit as
`e0034b5ade299a4ae13057bf9020926040d4e8ba` and requires 0025c before 0026.
This patch is a narrow hardening continuation of 0026: it changes no user-visible
motion modes, packet bytes, config-blob size, calibration data or default mode.

Explicitly out of scope by product decision: downgrade to firmware older than
0026. No old-firmware round-trip compatibility is claimed or tested.

## 2. Confirmed review defects

Review of 0026 found the following in-scope debt:

1. `slime motion-mode` called the full config save path on the current RAM config,
   so unrelated unsaved changes such as `slime rate` could hitchhike into NVS.
2. Persisted and runtime motion policy were represented by duplicate enums and
   duplicate mapping/name switches.
3. The serial handler owned persistence/runtime application directly and could
   acknowledge success without a mandatory domain-level apply path.
4. High-dip horizontal trust policy constants lived inside the pure helper rather
   than an explicit typed tuning object.
5. The new stationary-jump detector scaled for high dip, but the older
   `headingStepSoftDeg/headingStepHardDeg` gates remained fixed and could still
   enter suspect/disturbed state from direction noise after heading had become
   poorly observable.
6. Regression coverage was too narrow for long high-dip operation and
   near-vertical fail-closed behavior.

## 3. Implementation

### 3.1 Canonical motion policy

`src/core/slimevr_motion_policy.hpp` now owns the single canonical
`SlimeVRMotionPacketPolicy` enum plus validation/name helpers. Config schema,
config runtime, app wiring, CLI and SlimeVR runtime use this type directly.

`SlimeVRMotionPacketMode` remains intentionally separate. It is not persisted;
it describes the effective negotiated/fallback wire behavior, for example a
persisted `bundle` policy operating as separate packet 17 + packet 4 when a server
does not negotiate packet-100 bundles.

The persisted byte encoding introduced by 0026 is unchanged:

```text
QuaternionOnly                   -> marker + value 0
BundleRotation17Acceleration4    -> marker + value 1
RotationAcceleration23           -> marker + value 2
legacy/unmarked/invalid           -> QuaternionOnly
```

### 3.2 Field-isolated transactional persistence

`TrackerConfigStore::saveSlimeVRMotionPacketPolicy()` is a dedicated service-path
operation. It:

1. validates the requested policy;
2. reads the authoritative persisted tracker config;
3. if no authoritative generation exists, starts from clean defaults rather than
   arbitrary current RAM state;
4. changes only the motion-policy field;
5. commits through the existing transactional config save/read-back path;
6. only after success updates the corresponding field of active RAM config.

Therefore a sequence such as:

```text
slime rate 50          # RAM-only
slime motion-mode packet23
```

persists `packet23` without implicitly persisting the unsaved 50 Hz rate.

The CLI no longer performs storage or runtime wiring itself. It invokes the
`setSlimeVRMotionPacketPolicy` domain hook. App ownership performs the isolated
store operation and then calls the field-local runtime setter. Missing wiring or
storage failure returns an error and does not change runtime policy.

`SlimeVROutputRuntime::setMotionPacketPolicy()` changes only the selected policy
and motion deadlines. It does not restart discovery, rebuild network config or
mutate other runtime settings.

### 3.3 Typed high-dip observability policy

`MagHorizontalTrustConfig` now owns the adaptive horizontal-observability tuning:

```text
absoluteBadFloor        20
absoluteGoodFloor       40
referenceBadFraction    0.35
referenceGoodFraction   0.65
headingNoiseScaleMin    1.0
headingNoiseScaleMax    2.0
```

The pure evaluator consumes the typed config rather than hidden local constants.
Only field reliability owns the adaptive policy in product runtime; yaw correction
consumes the already-computed shared horizontal-trust snapshot. Its standalone
fallback remains the historical absolute `horizontalNormBad/Good` ramp and does
not introduce a second adaptive tuning state.

### 3.4 Complete high-dip directional-gate scaling

The geometry-derived heading-noise scale now applies to both:

- stationary heading-jump minimum angle/rate;
- legacy per-sample heading-step soft/hard thresholds.

Directional heading-step gates are evaluated only when horizontal heading is
observable. The disturbed-to-recovering absolute heading gate uses the same
scaled soft threshold and also requires observability.

This closes the remaining path where a healthy high-dip field could avoid the
new stationary-jump latch yet still be classified disturbed by the older fixed
8/20-degree directional step gates.

Diagnostics add:

```text
field_heading_step_threshold_soft_hard_deg=<soft>,<hard>
```

## 4. Preserved invariants and compatibility

- `sizeof(TrackerConfigBlob)` remains 756 bytes.
- Config/schema versions and calibration storage are unchanged.
- Default and legacy-migration policy remains quaternion-only.
- Packet 17/4/23/100 encoders and wire bytes are unchanged.
- Bundle negotiation and packet-23 invalid-acceleration fallback are unchanged.
- AHRS equations, gyro integration, FIFO/timestamps, magnetic calibration and
  sensor/device coordinate transforms are unchanged.
- No storage, formatting or allocation was added to the magnetic sample path.
- No heap-backed runtime abstraction was introduced; the storage helper uses the
  existing bounded service-path scratch mechanism.
- Same-version reboot persistence remains part of the 0026 contract.
- Old-firmware downgrade compatibility is intentionally not supported/tested.

## 5. Regression coverage

`tests/native/test_slimevr_motion_mode_command.cpp` now proves:

- a first motion-mode save with no authoritative NVS generation starts from clean
  defaults rather than unrelated RAM-only edits;
- once a stored generation exists, unsaved output-rate/local-output edits do not
  hitchhike into a motion-mode save;
- successful save updates persisted, active and runtime policy;
- storage failure leaves active/runtime policy unchanged;
- invalid input leaves policy unchanged;
- missing domain hook fails rather than silently persisting/applying partially.

`tests/native/test_mag_heading_reliability.cpp` additionally proves:

- a high-dip field scales the legacy heading-step soft/hard thresholds;
- a step above the historical fixed 8-degree soft limit is accepted when it is
  below the geometry-adjusted threshold;
- below-floor/near-vertical heading remains fail-closed for yaw without false
  directional disturbance/latch classification;
- a deterministic 30-minute high-dip run with bounded norm/dip/heading noise and
  slow AHRS yaw drift does not enter disturbed state or latch a false stationary
  jump;
- a real directional jump after that long run still latches fail-closed and return
  to the original stationary field recovers through the bounded dwell path.

A source-policy gate prevents reintroduction of duplicate motion policy enums,
whole-config motion-mode saves, hidden horizontal tuning constants, unscaled
legacy heading-step gates or missing long high-dip regressions.

## 6. Performance, RAM and stack impact

Host ABI comparison against 0026:

```text
TrackerConfigBlob                 756 -> 756
MagFieldReliabilityConfig         100 -> 124
MagFieldReliabilityOutput         112 -> 120
MagFieldReliabilityMonitor        320 -> 328
MagYawCorrectionConfig             96 -> 96
SlimeVROutputRuntimeConfig        112 -> 112
SlimeVROutputRuntimeStatus        792 -> 792
```

The extra reliability config/output state is fixed-size scalar tuning/diagnostic
data. The runtime motion policy enum replaces a duplicate enum; it does not add
runtime heap state.

Optimized host `-fstack-usage` comparison:

```text
MagFieldReliabilityMonitor::update             176 -> 176 bytes
MagYawCorrectionController::update(out)         48 -> 48 bytes
MagRuntimeController::fieldReliabilityConfig     8 -> 8 bytes
trackerSerialDispatchSlimeVRCommand             880 -> 880 bytes
new saveSlimeVRMotionPacketPolicy                - -> 96 bytes, dynamic/bounded
```

The magnetic hot path reuses the already-computed geometry scale and adds only
bounded comparisons/multiplications. No additional square root was added over
0026. Target ESP32 timing/stack/firmware-size measurements remain hardware/toolchain
acceptance because PlatformIO is unavailable in this environment.

## 7. Verification record

Completed after the final code changes:

```text
Focused -Werror native:
  test_config_hardening                  PASS
  test_config_schema_detail              PASS
  test_mag_heading_reliability           PASS
  test_slimevr_motion_mode_command       PASS
  test_slimevr_output_runtime            PASS

Focused UBSan + -Werror:
  test_mag_heading_reliability           PASS
  test_slimevr_motion_mode_command       PASS

Production-only host compile with -Werror:
  src/serial/tracker_slimevr_commands.cpp PASS
  src/app/tracker_command_wiring.cpp      PASS
  src/app/tracker_app.cpp (-DARDUINO)     PASS
  src/app/tracker_bootstrap.cpp (-DARDUINO) PASS

Policy/validators:
  0026a source policy                    PASS
  magnetic-heading policy                PASS
  documentation validator                PASS
  profile-matrix validator               PASS
  source-filter validator                PASS
  git diff --check                       PASS
```

Clean-apply verification on an untouched local worktree containing the exact 0026
predecessor content:

```text
git apply --check                        PASS
git apply                                PASS
git diff --check                         PASS
0026a source policy                      PASS
documentation validator                  PASS
test_mag_heading_reliability (-Werror)   PASS
test_slimevr_motion_mode_command (-Werror) PASS
git apply --reverse --check              PASS
```

A full `run_standalone_tests.py` invocation was attempted twice, but the bounded
runner timed out in unrelated long compile-only translation units before reaching
the executable matrix. A `check_all.py --skip-native --skip-pio` aggregation run
also progressed through validators and many policy gates with no observed failure
before the outer environment timeout. These incomplete runs are not reported as
PASS.

PlatformIO is not installed in this environment. Therefore this patch is
**focused-host-verified**, not release-ready.

## 8. Hardware/target acceptance

After applying on top of 0026, build all committed PlatformIO profiles and run the
existing 0026 HIL checks. Additional acceptance for this hardening:

### Persistence isolation

1. Start from a known saved config and note `slime status` / `config print`.
2. Issue a RAM-only change, for example `slime rate 50`, without a separate
   config-save operation.
3. Issue `slime motion-mode packet23`.
4. Reboot.
5. Require `motion_packet_policy=rotation_acceleration_23` while the unrelated
   unsaved rate returns to the previously persisted value.
6. Repeat for `bundle` and `quaternion`.

### High-dip field

In the previously failing location:

1. `mag heading clear`, then keep still until reference acquisition.
2. Require non-zero/full horizontal trust, `field_state=trusted` and
   `field_trusted_for_yaw=yes` when the other gates permit it.
3. Leave the tracker stationary for at least 30 minutes. Require no unexplained
   growth of stationary-jump latches/disturbed entries.
4. Introduce a real directional disturbance and remove it. Require immediate
   fail-closed rejection followed by bounded return/recovery.
5. Throughout the run require zero FIFO overrun/full and no tracking recovery
   caused by the magnetic changes.

## 9. Rollback

Runtime motion fallback remains:

```text
slime motion-mode quaternion
```

Code rollback is only to the immediate 0026 predecessor:

```bash
git apply --reverse --check 0026a_motion_policy_and_magnetic_reliability_hardening.patch
git apply --reverse 0026a_motion_policy_and_magnetic_reliability_hardening.patch
```

No calibration reset is required. Downgrade to older firmware is outside the
supported product lifecycle by explicit product decision.

## 10. Known limitations / unverified gates

- PlatformIO target builds and ESP32-C3 firmware-size/target-stack deltas are not
  verified here.
- Real QMC 30-minute high-dip/HIL recovery is not verified here.
- Packet-23 compatibility/TPS with the target SlimeVR Server remains the 0026 HIL
  gate; this hardening does not change packet-23 encoding.
- Full native/check_all aggregate gates were attempted but did not complete within
  this environment's bounded execution window; only the completed focused and
  policy results above are claimed.
