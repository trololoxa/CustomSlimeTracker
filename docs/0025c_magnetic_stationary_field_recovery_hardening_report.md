# 0025c magnetic stationary-field recovery hardening

## Scope and predecessor

Predecessor: `0025b_remote_cli_transport_parity.patch` applied to the supplied
`Tracker_Firmware.zip`. The archive does not contain authoritative Git metadata,
so no upstream base commit is claimed.

Scope: magnetic field-reliability detection and recovery only. Out of scope:
calibration models, `magToImu`, AHRS equations, yaw-correction gains/limits, FIFO,
networking, CLI permissions, persistent formats and LOGVER3 column layout.

## Confirmed defect and root cause

A hardware status snapshot remained `field_state=disturbed`,
`field_trusted_for_yaw=no`, `field_flags=0xA00` for roughly 30 minutes while the
current sample remained processor-trusted and inside norm limits. `0xA00` is
`STATIONARY_HEADING_JUMP | RECOVERY_PENDING`.

The predecessor measured stationary jumps using raw
`magneticNorthWorldYawRad`. That value contains mutable AHRS world yaw. An AHRS
yaw correction/reset could therefore move magnetic north and falsely latch a field
jump even though the physical body-relative field did not change. After any latch,
recovery required raw world heading to return within 2.5 degrees of its old
reference. While magnetic correction was disabled, ordinary 6DoF yaw drift could
move that value permanently outside the return window. The 30-second
`newEnvironmentStableMs` path only raised `ENVIRONMENT_CHANGED`; it never restored
trust, so waiting longer could not solve the lockout.

## Implementation

- Stationary step, window and filtered-rate calculations use
  `yawInnovationRad = magneticNorthWorldYawRad - currentAhrsYawRad`.
- A simultaneous AHRS/world-heading step therefore cancels instead of appearing as
  magnetic motion.
- A real latched jump stores the pre-jump stationary field signal.
- If gyro/accel continue to prove the tracker never moved, return of that signal
  may clear the latch after the existing 1.2-second dwell even when raw world yaw
  drifted.
- Any detected motion permanently invalidates that relative return for the current
  latch; recovery then requires the existing absolute world reference or explicit
  reacquisition.
- The unchanged 5-second `Recovering` dwell still precedes yaw trust.
- A stable shifted field is not automatically adopted as a new environment.

## Compatibility and invariants

- Persistent/config/candidate schemas: unchanged.
- Calibration bytes, provenance and revision: unchanged.
- Quaternion/output frame and AHRS math: unchanged.
- Mag yaw correction rate, step, innovation and cooldown limits: unchanged.
- LOGVER3 fields and ordering: unchanged.
- Runtime memory: a small fixed set of scalar/bool state and diagnostic counters;
  no heap use, queue or unbounded work.

## Regression coverage

The old code was separately compiled and reproduced both failures:

1. a 12-degree simultaneous magnetic-north/AHRS-yaw step falsely latched;
2. after a real 9-degree disturbance and 5 degrees of AHRS yaw drift, removing the
   disturbance never restored trust.

New native tests cover those cases plus physical-motion invalidation. Existing
moderate disturbance, slow drift, maximum normal correction, real rotation,
reacquisition and axis-solver tests remain in the same suite.

## Verification

PASS:

```text
python3 tools/test_mag_heading_reliability_policy.py
focused native test_mag_heading_reliability with -Werror and UBSan
g++ -Werror compile of src/runtime/mag_status_reporter.cpp
git diff --check
```

A full strict native runner compiled all shared project objects and the changed
magnetic units, but the long compile-only matrix exceeded the execution wrapper
time before completion. PlatformIO and hardware/HIL were not available, so the
patch is host-verified, not release-ready.

## Hardware acceptance

1. Flash ProductionDiag and wait for `field_state=trusted`.
2. Keep the tracker physically still, introduce then remove a directional magnetic
   disturbance. If world yaw drifted, require
   `field_stationary_heading_returns_via_stationary_field` to increment.
3. Require `disturbed -> recovering` after about 1.2 seconds and `trusted` after an
   additional 5 seconds.
4. Repeat while physically rotating the tracker during the disturbance. Require
   `field_stationary_latch_motion_seen=yes`; relative-return count must not change.
5. Inject/reproduce an AHRS-only yaw step if available; it must not increment
   `field_stationary_heading_jumps_latched`.
6. Require zero FIFO overrun/full, zero unexplained sample loss and no tracking
   recovery entry.

Useful command:

```text
mag heading
```

## Rollback

Reverse-apply the patch. No storage migration or calibration rollback is needed.
