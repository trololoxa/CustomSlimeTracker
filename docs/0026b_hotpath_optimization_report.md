# 0026b hot-path optimization report

## 1. Scope and exact predecessor/base

Patch: `0026b_hotpath_optimization.patch`.

Immediate predecessor: `0026a_motion_policy_and_magnetic_reliability_hardening`.
The local verification repository was frozen at synthetic commit
`b2fd095b3bb35266448fd29124683303d85b221a`, containing the complete 0026a state.
This commit is used only as the exact clean-apply baseline for the generated diff;
the patch is intended to be applied immediately after 0026a.

Scope is limited to execution/stack cleanup in the magnetic field-reliability,
magnetic yaw-correction and SlimeVR motion-send paths plus regression/contracts and
canonical documentation. No user-visible feature is added.

## 2. Confirmed measurable goal

Review of 0026/0026a found avoidable work in sample/output paths:

- production magnetic and yaw callbacks copied 248-byte and 296-byte aggregate
  inputs containing snapshots already owned by `MagRuntimeController`;
- high-dip geometry evaluated a `sqrt` for every magnetic update even though the
  result was used only to scale positive thresholds;
- quaternion-only output converted acceleration from g to m/s^2 even though no
  acceleration bytes are emitted in that mode;
- `MagFieldReliabilityMonitor` retained and copied a complete second 120-byte
  last-output snapshot although the runtime already owns the published field result;
- `processRawSample()` reserved a field-output fallback despite all production and
  native controller wiring already supplying `lastFieldReliability`;
- auto-reference rebuilt the complete yaw config even though it needs only the same
  enable/apply/recovery decision already required by yaw correction.

Immediate-predecessor host `-fstack-usage` measurements also showed 416-byte and
448-byte runtime callback frames around the field and yaw leaf functions.

## 3. Evidence and root cause

The overhead was structural rather than required by the algorithms. Runtime already
owns coherent `MagProcessedSample` and `MagHeadingSample` snapshots, so copying them
into another aggregate does not improve temporal coherence. High-dip scale is
strictly positive; comparisons against `threshold * sqrt(scaleSquared)` can be made
against the corresponding squared magnitudes without changing the inequality.
Quaternion-only returns packet 17 and therefore has no consumer for converted
acceleration. The runtime-owned reliability result is already the coherent output for
that sample, so a second monitor-owned copy provides no semantic value. Yaw enable
state is only three booleans and can be snapshotted once before auto-reference/yaw
without extending the lifetime of the complete 96-byte yaw config.

A deterministic before/after decision replay of 120,000 magnetic updates covers
high dip, bounded norm/dip/heading noise, periodic real 10-degree field jumps and
removals, and physical-motion windows. The complete per-step decision stream
(`state`, `flags`, `trustedForYaw`, stationary-jump latch) is identical. Final
counters are identical: 32 state transitions, 10 disturbed entries, 10 recovering
entries, 10 latched stationary jumps, 10 stationary returns, 113309 trusted samples
and 6691 rejected samples.

A supporting host microbenchmark of the old aggregate callback style versus the new
view/single-output style (3,000,000 updates, seven runs) produced median 58.935
ns/update versus 40.535 ns/update, about 31.2% lower host time. This is not ESP32-C3
target evidence.

## 4. Out of scope and preserved invariants

Unchanged:

- magnetic norm/dip/high-dip thresholds and all comparison semantics;
- field state machine, dwell/recovery/latch behavior and counters;
- yaw correction equations, gains, cooldown/reacquisition and apply gates;
- calibration, sensor/output coordinate frames, ODR and sample ordering;
- TrackerConfig/NVS schema, CRC, defaults and motion-policy persistence;
- SlimeVR packet bytes, packet 17/23/100 encoders, feature negotiation, fallback,
  output cadence and latest-state semantics;
- CLI command names and output keys.

No sample is skipped/decimated and no diagnostic gate is disabled. Downgrade behavior
remains the explicit non-goal recorded for 0026a.

## 5. Implementation and ownership

`MagFieldReliabilityInputView` and `MagYawCorrectionInputView` are lightweight
pointer/scalar views used by the production runtime. Existing aggregate overloads
remain wrappers for standalone/native callers. `MagFieldReliabilityMonitor` no longer
retains a duplicate complete last-output object; the runtime-owned result is passed
directly to auto-reference and yaw. Leaf algorithms continue to own all magnetic/yaw
decisions.

`MagHorizontalTrustResult` carries `headingNoiseScaleSquared`. Field reliability
uses squared positive comparisons preserving the original `>=`, `>` and `<=`
boundaries. `magHeadingNoiseScale()` performs `sqrt` only when diagnostic reporting
needs human-readable effective thresholds.

Immutable field-reliability tuning is returned as one shared static const config.
Persisted `horizontalNormBad/Good` values remain explicit in the runtime input view,
so authoritative runtime changes are observed without a per-controller config cache.

`SlimeVROutputRuntime::sendRotation()` returns for quaternion-only or invalid
acceleration before conversion. Valid acceleration modes then perform one shared
conversion and continue through the existing packet23/bundle/fallback branches.

`processRawSample()` requires the already-mandatory runtime `lastFieldReliability`
output pointer instead of reserving a local fallback. A compact `YawEnableState`
snapshots enabled/apply/recovery state once; auto-reference consumes those booleans,
while yaw correction constructs the full config only once from the same state.

## 6. Compatibility

Persistent/config: no blob/schema/version/default/migration/storage change.

Wire: no packet format, byte order, quantization, scheduler rate, negotiation or
fallback change. Quaternion-only still sends packet 17; packet23/bundle behavior is
unchanged.

CLI: commands and status keys are unchanged. Effective high-dip thresholds printed
by `mag heading` are derived lazily but represent the same values.

Calibration/AHRS/FIFO/profiles: no calibration state/layout, AHRS equation, FIFO
policy, source filter or profile feature change.

## 7. Regression and fault coverage

- Existing 0026a native magnetic suite remains the behavioral authority, including
  the deterministic 30-minute high-dip/noise run, near-vertical fail-closed case,
  real disturbance latch and bounded recovery.
- Existing SlimeVR output tests retain exact packet/fallback behavior.
- `tools/test_0026b_hotpath_optimization_policy.py` guards view ownership, absence
  of hot-path `sqrt`, lazy diagnostic conversion, quaternion early return and
  cross-optimization stack ceilings.
- The 120,000-step old/new deterministic decision replay provides direct semantic
  equivalence evidence across repeated disturbance/recovery and movement windows.

No new recovery state or persistent transaction is introduced, so additional
power-cut/retry state-machine behavior is not created by this patch.

## 8. Performance, RAM and stack impact

Host `-fstack-usage` against immediate predecessor 0026a:

| Function/chain | 0026a `-O2` | 0026b `-O2` | 0026a `-Os` | 0026b `-Os` |
| --- | ---: | ---: | ---: | ---: |
| `processRawSample` | 224 B | 112 B | 224 B | 112 B |
| `updateFieldReliabilitySnapshot` | 416 B | 144 B | 416 B | 144 B |
| field leaf `update(view)` | 176 B | 224 B | 144 B | 240 B |
| approximate deepest field chain | 816 B | 480 B | 784 B | 496 B |
| `updateYawCorrectionSnapshot` | 448 B | 208 B | 448 B | 224 B |
| yaw leaf `update(view)` | 48 B | 48 B | 48 B | 48 B |
| approximate deepest yaw chain | 720 B | 368 B | 720 B | 384 B |
| `sendRotation` | 192 B | 192 B | 208 B | 208 B |

The field leaf grows because arithmetic previously performed around/copying into the
aggregate is now visible in the leaf frame; the complete nested production callback
peak is nevertheless substantially smaller. The runtime now treats
`lastFieldReliability` as a required sample-path output dependency, matching all
production/native wiring and removing an otherwise always-reserved 112-byte fallback
object. Struct sizes change `MagFieldReliabilityOutput` 120 -> 112 B and
`MagFieldReliabilityMonitor` 328 -> 208 B after removing its duplicate last-output
copy. `MagRuntimeController` remains 1656 B. No heap allocation or new
persistent/fixed RAM buffer is introduced.

CPU: one `sqrt` is removed from every magnetic update; quaternion-only avoids three
acceleration unit-conversion multiplies per output tick. Supporting host timing is
listed in section 3. Target P50/P95/P99/max timing and real stack/high-water evidence
are still required; this report does not claim an ESP32-C3 performance improvement.

## 9. Verification record

Completed on the candidate worktree:

- `python3 tools/test_calibration_0023gf_policy.py` -> PASS.
- `python3 tools/test_0026a_motion_policy_and_magnetic_reliability_hardening.py`
  -> PASS.
- `python3 tools/test_0026b_hotpath_optimization_policy.py` -> PASS, including
  `-O2` and `-Os` stack ceilings.
- `python3 tools/validate_source_filters.py` -> PASS.
- `python3 tools/validate_profile_matrix.py` -> PASS.
- `python3 tools/validate_documentation.py` -> PASS.
- Focused `-Werror -fsanitize=undefined -fno-sanitize-recover=undefined` build/run
  of `test_mag_heading_reliability` -> PASS.
- Focused equivalent UBSan build/run of `test_slimevr_output_runtime` -> PASS.
- Production-only `-Werror`/UBSan compile of `mag_runtime_controller.cpp`,
  `mag_status_reporter.cpp` and Arduino `tracker_app.cpp` -> PASS.
- Deterministic 120,000-update 0026a-vs-0026b decision-stream comparison -> PASS,
  byte-identical stream and identical final counters.
- Seven-run host microbenchmark -> 0026a median 58.935 ns/update, 0026b median
  40.535 ns/update (supporting host evidence only).
- `git diff --check b2fd095` -> PASS.
- Existing-file line-ending audit against `b2fd095` -> PASS; CRLF ownership is
  preserved and the new report/policy files are ordinary LF files.
- Separate detached worktree at exact `b2fd095`: `git apply --check`, real
  `git apply`, `git diff --check`, 0026a policy, 0026b policy, documentation
  validator and `git apply --reverse --check` -> PASS.
- On the same clean base, `patch --dry-run --binary -p1` -> PASS for the generated
  unified diff fallback.

Two complete-run attempts did not finish within the execution environment's outer
command limit and are **not** reported as PASS:

- `python3 tools/run_standalone_tests.py --clean --extra-cxxflag=-Werror
  --sanitizer=undefined --timeout-s 300` timed out during compile-only production
  units after all shared project objects had compiled; no failing test was observed
  before timeout.
- `python3 tools/run_standalone_tests.py --clean --extra-cxxflag=-Werror
  --timeout-s 300` likewise timed out during the long compile-only matrix before
  test execution completed.
- `python3 tools/check_all.py --clean --skip-native --skip-pio --tool-timeout-s 180`
  progressed through source/profile/documentation validators, build identity,
  quality-gate/tool/capture policies, 0025a/0025b/0026a/0026b and calibration
  policies through 0023f, then hit the outer environment timeout while invoking
  the long 0023g policy. It is therefore not a complete aggregate PASS.

## 10. Not verified

PlatformIO is not installed in the execution environment, so Debug, Production,
ProductionDiag and Slim target compile/link plus firmware size/map are **NOT RUN**.
Physical ESP32-C3/QMC/SlimeVR HIL and ProductionDiag target timing/high-water are
**NOT RUN**. Because the complete standalone/check_all matrices were prevented from
finishing by the environment limit, this candidate is focused-host-verified, not a
full host-gate PASS and not release-ready.

## 11. Hardware acceptance

Build the same ProductionDiag profile/config immediately before and after 0026b.
For each firmware:

```text
perf tracking reset
# run the same 30-minute high-dip motion/stationary workload
perf tracking
mag heading
slime status
```

Require unchanged configured IMU/output rates, zero unexpected FIFO overrun/full,
quality-drop and tracking-recovery deltas, and no regression in rotation delivery.
Compare identical profiler/callback P50/P95/P99/max metrics and stack/high-water
telemetry. Then run the existing 0026 magnetic disturbance/reacquire acceptance and
all three motion packet modes; functional results and packet counters must remain
identical. The project performance release gate remains the 5-12 hour target matrix.

## 12. Rollback and known limitations

Rollback is code-only: reverse 0026b. No persistent/wire/calibration migration is
needed because no stored state changes. The public aggregate magnetic/yaw update
overloads remain intentionally for native/standalone compatibility even though the
production runtime no longer uses them.

Known limitation: host stack/timing evidence depends on the host ABI/compiler and
cannot prove ESP32-C3 latency or target stack high-water. Release acceptance therefore
remains blocked on target build/performance/HIL evidence.
