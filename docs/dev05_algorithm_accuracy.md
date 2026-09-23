# DEV-05: independent math and algorithm scenarios

This is host-only test infrastructure, additive after DEV-04b. It does not change
firmware, packet mode, sensor cadence, thresholds, persistence or target packages.
It measures production modules against an independent synthetic truth. It is not
an end-to-end hardware/FIFO/app replay and does not prove real-device accuracy.

DEV-05a fixes observation contracts; see [review and acceptance](dev05a_report.md).
The broader [math verification plan](dev05_math_verification_plan_ru.md) is a staged
proposal, not coverage already implemented by DEV-05a.

## What is executed

`test_dev05_math_reference` checks quaternion rotation, exponential maps, small
angles, near-pi angles, sign equivalence and invalid normalization. Its double
oracle has no production math includes. Quaternion exponential results are
cross-checked against Rodrigues matrices and hand-known axis rotations. Existing
`test_core_math_ahrs` now measures angular error with double atan2 of the relative
quaternion instead of float acos(dot), which could round small errors to zero.

`test_dev05_algorithm_scenarios` invokes the production Ahrs6Dof, MagHeadingEstimator
and MagYawCorrectionController. It runs 24 named synthetic scenarios. Virtual
sensor time means a 60-second trajectory does not require a 60-second wait.

| Group | Coverage / scope |
| --- | --- |
| Static / gyro | Static pose, arbitrary-axis rotation, noncommuting rotations, 60 s integration |
| Accel | Clean multi-axis, missing segment, bounded dynamic and 20 g out-of-range segment, continuous dynamic input, 25/90 degree tilt relock |
| Timing / noise | Exact fractional 960 Hz clock, timestamp jitter, fixed-seed noise and gyro bias, analytic 1 Hz yaw sine |
| Timestamp assertions | Duplicate/backward samples do not advance integrated timestamp; rejected gap resumes; explicit rebase; crossing 32-bit boundary in 64-bit gyro time |
| Mag | Tilted device, clean correction, untrusted field, stale sample, untrusted tilt, continuous untrusted field, 90/180 degree yaw residual, uint32 millisecond wrap |
| Interactions | Missing accel must not train zero-g statistics; yaw callback at each of 16 gyro phases must preserve accel evidence |
| Invalid input | Invalid gyro preserves orientation; invalid accel probe measures whether valid gyro propagation continues |

The mag scenarios supply upstream `fieldReliable` / trust inputs deliberately.
They test heading/controller contracts, not magnetic disturbance classification,
QMC streaming, the field monitor or real environmental observability. The yaw
application wrapper uses the current world-yaw multiplication convention. The
phase probe uses current `setQuaternion`; adapt the wrapper explicitly when 0029
introduces its dedicated yaw API. That changes the harness hash, requiring a
baseline with the same new harness before comparing implementations.

Synthetic truth is continuous orientation sampled at exact timestamps. Gyro is
piecewise constant per interval (interval average for the yaw sine); gravity is
computed independently in body coordinates. Noise uses xorshift32 with fixed
seed `0x5eed1234`, bounded uniform gyro +/-0.001 rad/s and accel +/-0.003 g, plus
0.002 rad/s Z bias. These are controlled test inputs, not a measured noise model.
The 20 g case tests AHRS admission, not the hardware saturation flag pipeline.

## Verdicts: execution is different from algorithm acceptance

A successful native executable proves that reference checks and current mandatory
assertions completed. It does NOT mean all roadmap behavior is implemented.
Five explicitly labelled diagnostic scenarios record `requirement_met=false` on
the current source. They are visible as `OPEN` during export. Their outcomes are
never relabelled as successful recovery:

- `tilt_90_relock`: ordinary Ahrs6Dof updates remain at 90 degrees after 30 s of clean gravity.
- `invalid_accel_gyro_continuity`: NaN accel rejects the valid gyro update at the module API.
  Success requires the expected quaternion step AND exact integrated timestamp, as
  well as update success. A true return value alone cannot close this contract.
- `missing_accel_statistics`: missing accel trains norm mean from 1 g toward zero.
- `yaw_accel_evidence_phases`: 12 total accel updates lost across the 16 phase trials.
- `mag_180_relock`: current controller rejects the 180 degree residual throughout 30 s.

These are module-level reproductions for 0029/0030, not claims that every app
recovery path fails. Existing production thresholds were not changed. To require
all these contracts, comparison has `--require-roadmap`, which currently FAILS.
That strict result is required when claiming all listed recovery contracts are
fixed. Default comparison only reports regression relative to a baseline and
always includes the still-open contracts; it is not whole-tracker acceptance.

## Measurements and comparison

Each scenario reports orientation RMS, p95, maximum, final error; tilt RMS; yaw
RMS; max quaternion norm error; maximum output step; reject count; recovery
latency where defined; and scenario-specific evidence loss. Quaternions q/-q
are equivalent. Full orientation error uses normalized double relative quaternion
and atan2, preserving small-angle resolution. Tilt compares estimated gravity.
Yaw is projected heading, not a claim of observable absolute yaw in 6D.

- `lag_abs_ms`: phase-delay magnitude at 1 Hz, only for `yaw_sine`. Zero elsewhere
  means not measured there. This is algorithm delay, not network/board latency.
- AHRS `recovery_ms`: the actual timestamp of the first sample in the first complete
  window of 240 consecutive samples below 0.5 degree tilt, relative to scenario start.
  This is a sample-count criterion (about 250 ms at 960 Hz), not an exact elapsed-time
  dwell. Later loss/recovery cannot overwrite it. Endpoint tilt remains a separate
  condition for requirement acceptance.
- Mag `recovery_ms`: first allowed correction after the 6 s clean-window boundary;
  it measures resumption, not complete yaw convergence. Continuous interference
  intentionally has no return window; its null recovery is expected.
- `evidence_loss`: norm-mean delta in g for `missing_accel_statistics`, lost update
  count for `yaw_accel_evidence_phases`, zero/not applicable elsewhere.
- Maximum output step and reject count are diagnostic: a larger legitimate gyro
  rotation or fewer rejections is not automatically worse/better.

Default comparison flags each metric change beyond max(absolute floor, 5% of
baseline magnitude). Floors: 0.0001 degree, 1e-6 quaternion norm/evidence loss,
0.02 ms phase lag. Recovery changes use max(20 ms, 5%). These are comparison
resolution budgets, not universal sensor accuracy limits or firmware gates.
They may be tightened with a reviewed future tool change; `--relative-budget`
sets the relative part and is recorded. No averaging improvements and regressions
into a single score. Any above-budget regression makes comparison exit 1.
Insufficient/malformed/incompatible evidence exits 2 instead of passing.

Compatibility requires identical scenario coverage, actual input FNV-1a checksum,
actual named configuration (including accel divisor and heading settings), harness
SHA-256, compiler version, host platform/pointer width, flags and sanitizer mode.
Windows and WSL evidence are verified separately, not directly A/B-compared.
DEV-05a changes the measurement harness: re-run both firmware versions with this
same corrected harness. Do not compare a DEV-05 report directly with DEV-05a.
A config/threshold sweep is not silently compared as an implementation-only change.

Native runner fingerprints test harness files and src C++/headers before/after
execution for this selected test only. Changes invalidate the run. Line endings
are normalized. Firmware content SHA may differ between A/B runs; harness SHA
must match. Startup HEAD/dirty metadata is retained: dirty working-tree results
are labelled evidence, not a reproducible release commit. External compiler,
SDK/standard headers and ignored dependencies are not content-certified. FNV is
an accidental-input mismatch check, not a cryptographic authenticity proof.
The exporter copies data from completed successful native reports/raw logs; it
never executes report argv. Output cannot overwrite native evidence. JSON writes
use existing bounded Windows replacement handling. No persistent result cache.

## Commands: Windows PowerShell

Use the tool variables from [dev_session.md](dev_session.md). Run the new tooling
checks once for patch acceptance:

```powershell
& $TrackerPython tools/check_all.py --check test_dev05_algorithm_accuracy --check test_dev04_workflow --check test_run_standalone_tests_policy --check validate_documentation
& $TrackerPython tools/run_standalone_tests.py --cxx $TrackerCxx --test test_dev05_observation_contracts --test test_dev05_math_reference --test test_dev05_algorithm_scenarios --test test_core_math_ahrs --build-timeout-s 240 --test-timeout-s 60
```

Copy the printed native report path into this variable (direct assignment, not
Read-Host). `native-EXAMPLE` is a placeholder for the actual newly printed folder:

```powershell
$TrackerAccuracyRun = 'H:\Programming\VRC\CustomSlime\SlimeTracker\build\gate_runs\native-EXAMPLE\summary.json'
& $TrackerPython tools/replay/algorithm_accuracy.py export --report $TrackerAccuracyRun --label before-ahrs-change --output build/accuracy/before.json
```

After the AHRS change run the same selected native test with unchanged flags,
export the new report to `build/accuracy/after.json`, then:

```powershell
& $TrackerPython tools/replay/algorithm_accuracy.py compare build/accuracy/before.json build/accuracy/after.json --output build/accuracy/comparison.json
```

When claiming all listed roadmap recovery contracts are implemented, append
`--require-roadmap`. It is intentionally a failing acceptance on the current
firmware. Export exit 0 means evidence was exported, not all contracts passed.

## WSL acceptance

Sync source as in the session guide. This selected run exercises the new host
math/scenarios with existing executable sanitizer capability probes:

```bash
.venv-dev-linux/bin/python tools/run_standalone_tests.py --cxx /usr/bin/g++ --test test_dev05_observation_contracts --test test_dev05_math_reference --test test_dev05_algorithm_scenarios --test test_core_math_ahrs --sanitizer address-undefined --build-timeout-s 600 --test-timeout-s 180
```

No new Python/C++ packages, plugins or firmware flash are needed. Reuse already
completed matching reports; do not repeat full check_all just to export metrics.
The Linux container evidence supplied with the patch does not replace Windows
integration acceptance. ASan/UBSan is distinct from standalone LSan or target tests.

## Existing logs and existing tests

`replay_machine_log.py` remains metric replay. The committed LOGVER2 baseline
contains decimated diagnostic CAL/Q frames; these cannot reconstruct the 960 Hz
stream or its full scheduling. DEV-05 does not invent samples or call that log
full AHRS replay. Real LOGVER3 release fixtures remain missing and are not replaced
by these synthetic scenarios. No new on-device logger was added.

Existing calibration/frame/FIFO/storage tests remain the owners of their tests;
DEV-05 does not duplicate them or certify all their behavior through this harness.
See the [test map](dev_test_map.md). A future recorded-input adapter needs enough
samples, frame/units/configuration metadata and honest treatment of gaps before
it can make accuracy claims.

## Additional features: benefit and decision

| Feature | Benefit | Decision |
| --- | --- | --- |
| Independent double oracle and robust angular distance | Detect numerical errors hidden by float metrics | Included |
| Fixed inputs, named config and source/harness identity | Make A/B attribution meaningful | Included |
| Per-metric comparison; refuse missing/mismatched data | Prevent false improvements and false green results | Included |
| Timing, stale data, phase interactions, persistent disturbance | Detect correctness failures outside simple stationary tests | Included |
| Production sign/order/freshness mutation experiments | Demonstrate tests actually catch characteristic defects | Performed on temporary copies; not mandatory every run |
| Many-seed noise ensembles and confidence intervals | Useful for stochastic filter tuning | Defer until tuning claims; one seed is not statistical significance |
| Frequency/amplitude sweep and gain/lag response curves | Useful for smoothing/latency tradeoffs | Next optional extension when filter response changes; current 1 Hz probe is limited |
| Dense recorded-input adapter plus real reference orientation | Real-device accuracy comparison | Requires appropriate existing data or justified capture; no fake golden |
| Coupled app/FIFO/calibration replay | Finds integration errors beyond isolated modules | Separate extension with explicit owners/events; do not create a second app implementation |
| Target WCET/current/stack measurements | Validate hot-path cost | Hardware evidence in DEV-06; desktop runtime is not ESP32 timing |
| Full second estimator / automatic threshold tuning | Adds complexity and model ambiguity | Excluded |
| New plugins for this test suite | No demonstrated need | Excluded; existing runners suffice |
