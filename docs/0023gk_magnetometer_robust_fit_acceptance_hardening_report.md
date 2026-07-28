# 0023gk magnetometer robust-fit acceptance hardening

## Scope

`0023gk_magnetometer_robust_fit_acceptance_hardening` is a suffix hotfix over
`0023gj_slimevr_connect_trackers_session_restart_hardening`. It addresses two
independent hard/soft-iron acceptance defects exposed by complete guided setup
runs on two similar LSM6DSV/QMC6309 trackers.

The patch does not change persistent config schema 2, candidate format 3,
SlimeVR protocol 22, IMU ODR/full-scale, FIFO/timestamp policy, AHRS policy,
magToImu alignment, networking, or calibration ownership.

## Hardware evidence

Both captures had complete guided-axis evidence:

```text
dynamic_axis_intervals=160/160
dynamic_axis_excited_axes=3
dynamic_axis_partition_confirmed_axes=3
dynamic_axis_bucket_counts=27,27,27,27,26,26
stored_fit_samples=768
```

The first tracker produced:

```text
last_fit_quality=0.310533,0.164865,0.791888,0.979167,1.436176,0.718164
last_fit_quality_limits=0.120000,0.100000,0.650000,0.820000,6.000000,0.133333
mag_cal_failure_reason=geometric_residual_too_high
```

The second tracker produced:

```text
last_fit_quality=0.146809,0.074308,0.799472,0.986979,1.489504,0.910394
last_fit_quality_limits=0.120000,0.100000,0.650000,0.820000,6.000000,0.133333
mag_cal_failure_reason=algebraic_residual_too_high
```

Coverage, conditioning, directional evidence, axis ratio, and the reported
self-scaled inlier ratios were healthy. The two failures nevertheless had
different causes.

## Defect 1: algebraic gate contradicted the geometric gate

For normalized corrected radius `n = 1 + e`, the centered ellipsoid equation
residual is:

```text
n^2 - 1 = 2e + e^2
```

The algebraic RMS is therefore approximately twice normalized radial/geometric
RMS for small errors. With the old limits:

```text
max geometric RMS = 0.10
max algebraic RMS = 0.12
```

the algebraic gate became the real physical limit at roughly six percent radial
error. The nominal ten-percent geometric gate was unreachable for ordinary
finite fits. The second hardware capture demonstrated the contradiction:
`0.074308` passed the authoritative physical geometric gate while its expected
approximately doubled algebraic value `0.146809` failed.

### Fix

The configured algebraic limit remains available as a minimum numerical sanity
ceiling, but the effective limit is now:

```text
max(
    configured algebraic limit,
    max(2.2, 2 + robust threshold cap factor) * geometric limit
)
```

The default factor 2.2 covers `2e + e^2` through the default 15% inlier
threshold and a small numerical margin. If a caller deliberately raises the
robust residual floor above that range, the algebraic conversion factor expands
to `2 + effective threshold factor` instead of silently becoming stricter.
The algebraic metric remains a translation-invariant numerical backstop; it can
no longer be stricter than the physical geometric gate.
`last_fit_quality_limits` now prints the effective limit actually used, so
diagnostics and behavior cannot disagree.

## Defect 2: contaminated RMS disabled robust rejection

The old inlier threshold was:

```text
max(3 * current RMS, 0.08 * expected field norm)
```

When a moderate population of samples was magnetically disturbed, those samples
increased the RMS and therefore increased their own rejection threshold. A
hardware-shaped native fixture with 10-18% samples radially displaced by 35%
kept every sample as an inlier, biased the candidate, and failed the final
geometric gate. This is the classic failure mode of sigma clipping when sigma is
estimated directly from an already contaminated population.

The previous tests covered either smooth distortion affecting the entire sphere
or a very small number of enormous spikes. They did not cover a bounded but
meaningful population of moderate disturbances, so the self-inflating threshold
was not detected.

### Fix

The threshold remains bounded below by the configured absolute floor and remains
sigma-driven for clean data, but is capped at:

```text
max(outlier floor, 1.5 * geometric acceptance limit)
```

With default parameters the inlier threshold is bounded to 15% normalized radial
error. The solver then performs at most three no-heap refit passes. Each pass:

1. computes the current bounded threshold;
2. rebuilds the same reused 9x9 accumulator from accepted samples;
3. requires the existing minimum sample count and 82% inlier ratio;
4. refits through a no-inline stack-isolated helper;
5. stops when the complete bounded 768-slot inlier bitmap converges.

No unbounded loop, allocation, queue, or realtime callback work is added.

## Safety behavior

Native regressions prove three distinct cases:

1. 15% moderate contamination is removed, the common ellipsoid is recovered,
   and final normalized geometric RMS falls below 1%;
2. 20% contamination exceeds the configured 18% outlier budget and remains
   fail-closed as `inlier_ratio_too_low`;
3. strongly non-ellipsoidal data with robust rejection deliberately disabled
   still fails `geometric_residual_too_high` and retains full diagnostics.

The physical quality limits are not relaxed. The patch makes the existing
inlier budget operative and prevents a redundant algebraic representation from
silently overriding the physical radial-error policy.

## Diagnostics

Finite fit output now additionally reports:

```text
last_fit_robust_refit_passes
last_fit_robust_threshold_factor
```

The threshold field is the actual final normalized radial threshold after the
configured floor, contaminated-sigma estimate, and quality cap are combined.
With default parameters it remains in the bounded range `0.08..0.15`.

`last_fit_quality_limits` continues to use the established tuple order:

```text
effective_max_algebraic,max_geometric,min_directional,min_inlier,max_axis_ratio,min_box_coverage
```

For the default configuration the first two values are now `0.220000,0.100000`.

## Resource direction

The large `FitAccumulator` remains a single reused workspace. Candidate
construction during robust refit is isolated behind a no-inline helper so the
caller does not retain another large fit object in its cross-ABI frame.

Linux GCC `-O2 -fstack-usage` measured:

```text
MagCalibrationCollector::compute       1424 bytes
replaceFitFromAccumulator               160 bytes
robustThreshold                          32 bytes
```

The predecessor cross-ABI ceiling for `compute()` remains 1792 bytes. The new
work runs only when an explicit/background hard/soft fit is evaluated, never in
the 960 Hz IMU callback, FIFO parser, AHRS update, or network deadline.

## Hardware acceptance

After clean build/flash:

```text
perf on
perf reset
setup calibration full
mag cal status
perf status
```

Expected on a recoverable capture:

```text
last_fit_robust_refit_passes >= 1       # only when contamination was removed
last_fit_robust_threshold_factor=0.080000..0.150000  # default floor through cap
calibration_valid=yes
axis_alignment_valid=yes
```

A clean capture may legitimately report zero refit passes. If calibration still
fails, use the new failure reason and actual final inlier ratio:

- `inlier_ratio_too_low`: too much of the capture does not belong to one stable
  ellipsoid; move away from metal/current-carrying objects and repeat;
- `geometric_residual_too_high`: the retained majority is still non-ellipsoidal;
- another numerical stage: inspect normalization and pivot diagnostics.

FIFO overrun/full, recovery, output-delivery, and UDP failure deltas must remain
at the accepted baseline.
