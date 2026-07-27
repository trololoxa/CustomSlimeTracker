# 0023gd magnetometer math and alignment hardening

## Scope

`0023gd_magnetometer_math_and_alignment_hardening` is a suffix hotfix over
`0023gc_mag_fit_stack_and_profile_build_hardening`. It addresses the hardware
failure in which guided setup retained complete three-axis magnetic coverage but
`MagCalibrationCollector::compute()` stopped at `ellipsoid_fit_failed` with no
quality metrics, and it audits the complete hard/soft-iron and `magToImu`
alignment chain for independent non-convergence defects.

The patch does not change persistent config schema 2, candidate format 3,
SlimeVR protocol 22, IMU ODR/full-scale, AHRS product policy, network policy, or
NVS migration behavior.

## Hardware evidence

The failing run had:

```text
dynamic_axis_intervals=160/160
dynamic_axis_candidates_seen=2061
dynamic_axis_excited_axes=3
dynamic_axis_partition_confirmed_axes=3
dynamic_axis_bucket_counts=27,27,27,27,26,26
stored_fit_samples=768
capture_span_xyz=1014,1047,1021
fit_span_xyz=996,1018,1007
capture_norm_min_mean_max=5.385,524.125,1011.750
mag_cal_failure_reason=ellipsoid_fit_failed
```

The reservoir and motion coverage were therefore healthy. The raw ellipsoid
passed close to the ADC origin while its mean radius and hard-iron translation
were both about 500 counts. The old uncentered fixed-constant quadratic system
became singular before any physical quality gate was evaluated.

## Hard/soft-iron mathematical audit

The calibrated magnetic vector remains:

```text
m_cal = softIron * (m_raw - hardIron)
```

The fit is now solved in an affine-normalized coordinate system:

```text
y = D^-1 * (x - mu)
y^T Q y + l^T y = 1
c = -0.5 * Q^-1 * l
k = 1 + c^T Q c
S_raw = D^-T * (Q / k) * D^-1
hardIron = mu + D * c
```

`S_raw` must be symmetric positive definite. Its principal square root produces
an SPD soft-iron correction, leaving only a rigid rotational ambiguity for the
separate `magToImu` stage.

### Defects and fixes

| Defect | Failure mode | Fix | Regression |
|---|---|---|---|
| Uncentered raw-origin fit | A valid translated ellipsoid crossing raw zero made the 9-term fixed-constant solve singular | Center and independently scale X/Y/Z before solving; map center and shape back in double precision | Near-origin hardware reproduction and deterministic translated/anisotropic convergence sweep |
| Translation-dependent algebraic metric | Identical physical ellipsoids received different residuals at different hard-iron offsets | Retain the `0023gb` completed-square normalization by `abs(k)` | Centered and translated fixture equality |
| Raw-norm outlier filter | `|raw|` changed with hard-iron translation and could remove valid samples near the ADC origin | Filter by distance from the sample centroid; derive final normalization from retained samples | Raw-zero and translated ellipsoid fixtures |
| Nonfinite denominator robustness | Centroid/radius statistics divided by total input count even when nonfinite observations were skipped | Track the finite sample count explicitly | Source policy and sanitizer coverage |
| Raw box gate contradicted allowed anisotropy | A valid 4:1 ellipsoid could fail a fixed 0.35 span ratio even though `maxAxisRatio` allowed it | Derive a conservative floor from `maxAxisRatio`; keep directional and geometric gates authoritative | Fully covered 4:1 fixture |
| Coverage checked before outlier filtering | One finite one-axis spike expanded raw reservoir extrema and rejected an otherwise valid ellipsoid before robust fitting | Evaluate minimum radius and box coverage on the centered retained solver set; preserve full-reservoir extrema only as diagnostics | One-axis finite-spike regression |
| Poorly conditioned normal equations | Quadratic/cross/linear columns had unequal scale and failure provided no conditioning evidence | Unit-diagonal equilibration, pivot ratio, explicit solver stage and sample count | Convergence sweep and policy stack/conditioning checks |
| Generic `ellipsoid_fit_failed` | Early math failures erased their exact stage | Separate normalization, linear solve, center, SPD shape, eigen and candidate stages | Failure-stage diagnostics |
| Rejected fit lost metrics | Physical quality rejection printed zeros | Preserve the best finite candidate and all residual/coverage metrics before returning | Strong non-ellipsoidal rejection keeps diagnostics |
| Potential reflection/shear in hard/soft result | A non-SPD correction would leak orientation into calibration | Require positive eigenvalues, positive determinant and invertibility; construct the principal SPD square root | Invalid transform tests |

The physical quality gates were not weakened: inlier ratio, normalized geometric
residual, directional coverage, axis-ratio and centered algebraic residual still
reject unsuitable data.

## `magToImu` alignment audit

Alignment uses calibrated magnetic vectors and native-sensor-frame gyro:

```text
m0 = R * softIron * (raw0 - hardIron)
m1 ~= Exp(-omega_sensor * dt) * m0
```

The solver first evaluates the 24 right-handed signed permutations, then performs
a bounded continuous `SO(3)` refinement. Training and validation remain separate.
The final matrix must be finite, orthonormal and have determinant `+1`.

### Defects and fixes

| Defect | Failure mode | Fix | Regression |
|---|---|---|---|
| Runtime first-N collector freeze | Continuous learning ignored every interval after the first bounded buffer filled | Use the same bounded axis/parity-stratified replacement reservoir as guided setup | Late X/Y/Z runtime reservoir test |
| Processing-time cadence/window accounting | A FIFO backlog drained in one loop gave many sensor-time-separated frames the same `receivedMs`, collapsing independent windows and blocking runtime convergence | Derive cadence and 750 ms window partitions from the hardware/FIFO `t_us` domain | Collapsed-processing-time backlog regression |
| Shuffled-window overcount | Counting transitions in reservoir order treated repeated windows as independent evidence | Count exact unique window IDs in each subset | Deliberately shuffled two-window rejection |
| External fake coverage arguments | Callers could claim more axes/windows than the actual solver dataset contained | Remove external coverage parameters; derive axes/windows/partitions from retained intervals | API/source policy and storage integration update |
| Raw-direction admission | Near raw origin, hard-iron translation made raw direction meaningless and rejected useful motion | Gate observed direction after hard/soft correction | Near-origin hard/soft alignment fixture |
| Raw zero treated as invalid | A valid calibrated point at raw `(0,0,0)` was discarded | Reject zero only after calibration/body mapping | Runtime raw-origin trust test |
| Selected-only gyro mean | Averaging only samples above 2 deg/s biased angular velocity upward | Use both timestamp-coherent endpoints and trapezoidal averaging | Shared interval-builder test |
| Free-running gyro accumulation | Guided setup could include IMU samples outside the interval between two magnetic frames | Store the gyro endpoint captured at each exact magnetic callback | Endpoint timestamp test and runtime diagnostics |
| Raw-first FIFO scheduling | Up to 64 IMU callbacks were dispatched before queued magnetic callbacks, so the apparent endpoint could be tens of milliseconds newer than the magnetic sample | Chronologically interleave magnetic callbacks as raw time reaches each magnetic timestamp, with a bounded eight-callback slice budget | FIFO nearest-endpoint regression |
| Stale endpoint accepted | Gyro/mag timestamps could silently disagree | Shared 5 ms maximum skew, stored skew diagnostic and fail-closed interval reset | Stale-pair rejection test |
| Inconsistent motion limits | Guided/runtime collectors used separate timing/rate/step rules | Shared interval builder and constants | Invalid timing/rate tests |
| Static/dynamic cross-check compared refined matrix | A valid small mechanical refinement could look different from the discrete face solution | Compare the dynamic coarse signed permutation | Source policy and proper-mounting sweep |
| Weak static cross-check vetoed validated dynamic solve | Inclination-only face evidence can be ambiguous yet could discard a dynamic solution that had already passed independent kinematic training/validation | Make the static result warning-only after a validated dynamic solve; retain fail-closed behavior inside the dynamic solver's own evidence gates | Source policy forbids the veto and requires application after the cross-check |
| Endpoint counter mislabeled as IMU samples | One retained gyro endpoint per magnetic frame was printed as the number of IMU samples | Rename the state and output to `gyro_endpoints_seen` | Source policy rejects the old field name |
| Reflections considered a calibration solution | A reflected driver frame cannot be represented by a physical sensor-to-sensor rotation | Keep only determinant `+1` mappings and make driver handedness an explicit contract | All 24 proper mappings plus reflection ambiguity regression |

Dynamic magnetic-vector kinematics cannot distinguish a reflected coordinate
frame from a polarity-inverted proper mapping using motion alone. Therefore the
QMC6309 driver frame is authoritative: output registers are consumed as package
X/Y/Z and future remaps must remain right-handed. A reflection is never persisted
as `magToImu`.

## QMC6309 and FIFO input hardening

The QMC6309 output rails are close to signed 16-bit full scale. Exact `INT16_MIN`
or `INT16_MAX` was too narrow a saturation check. Direct reads and sensor-hub
FIFO parsing now mark any axis with absolute count at or above 31900 as saturated.
The threshold is runtime-only configuration; it does not alter persistent schema.

The FIFO processor now dispatches queued sensor-hub magnetic samples in timestamp
order relative to raw IMU callbacks. If the bounded magnetic callback budget is
exhausted, it stops advancing the raw timeline and resumes on the next app pass.
This preserves bounded work and prevents a falsely coherent endpoint.

New diagnostics include:

```text
runtime_mag_chronological_deferrals_delta
fifo_runtime_mag_chronological_deferrals
gyro_endpoint_valid
gyro_endpoint_t_us
gyro_endpoint_skew_us
dynamic_axis_gyro_skew_rejected
dynamic_axis_timing_rejected
gyro_endpoints_seen
dynamic_axis_motion_rejected
```

## Resource and realtime review

No heap allocation or unbounded queue was added. Hard/soft solving remains a
deferred/setup operation. The magnetic and raw callback budgets remain bounded.
The fit continues to reuse one 9x9-double accumulator, preserving the `0023gc`
cross-ABI stack fix.

Linux GCC `-O2 -fstack-usage` after the patch:

```text
solveLinear9                                  792 bytes
fitFromAccumulator                           768 bytes
MagCalibrationCollector::compute            1520 bytes
buildRefinedCandidates                       816 bytes
solveMagAxisAlignmentDataset                1296 bytes
SetupMagAxisDynamicCollector::update         272 bytes
setupAutoSolveMagAxisDynamic                 304 bytes
setupRunAxisAlignment                        144 bytes
FifoRuntimeProcessor::process                112 bytes
MagRuntimeController::processRawSample       960 bytes
```

`MagRuntimeController::processRawSample()` writes directly into the owned
runtime snapshot instead of keeping a second endpoint-expanded local copy. Its
Linux stack decreases from the 0023gc predecessor's 1024 bytes to 960 bytes.

A host `-Os` direction check over the six directly affected core translation
units (`mag_calibration`, `mag_axis_alignment`, FIFO processing, magnetic
runtime/controller and LSM FIFO) reports `text +7288 bytes`, `data +0` and
`bss +0` versus 0023gc. This is not the final ESP32 linked image size; the
mandatory PlatformIO profile builds remain the authoritative flash/RAM gate.

## Verification

Targeted native regressions:

```text
PASS test_mag_calibration
PASS test_mag_heading_reliability
PASS test_fifo_runtime_processor
PASS test_fifo_pair_coherency
```

The same four tests pass under GCC AddressSanitizer and UndefinedBehaviorSanitizer.
The `0023gd` policy also compiles the affected setup/runtime translation units,
checks stack ceilings, source/profile/documentation contracts and all mathematical
invariants listed above.

A full `check_all.py --clean` run in the authoring environment made forward
progress through the standalone object matrix, including every modified core
module, but the external execution limit stopped the aggregate run before its
final result. It is therefore not claimed as a complete `check_all` pass.

PlatformIO is not available in the authoring environment. The user's mandatory
final gate remains:

```bash
python3 tools/check_all.py --clean --require-pio
```

## Hardware acceptance

After a clean build and flash:

```text
perf on
perf reset
setup calibration full
setup status
mag cal status
mag status
mag heading
perf status
```

Hard/soft acceptance should show a nonzero solver stage, finite normalization
center/scale, finite pivot ratio, nonzero fit metrics and `calibration_valid=yes`.
Alignment acceptance should show three excited axes, at least two independently
confirmed train/validation axes, bounded endpoint skew, a finite validation score,
matching training/validation winners and `axis_alignment_valid=yes`. FIFO overrun,
full, recovery and UDP failure deltas must remain zero or at the accepted baseline.
