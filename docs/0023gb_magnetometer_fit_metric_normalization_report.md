# 0023gb magnetometer fit metric normalization

## Triggering hardware failure

After `0023g`/`0023ga`, guided setup retained full three-axis coverage and both train/validation partitions:

```text
dynamic_axis_intervals=160/160
dynamic_axis_candidates_seen=1039
dynamic_axis_excited_axes=3
dynamic_axis_partition_confirmed_axes=3
dynamic_axis_bucket_counts=27,27,27,27,26,26
stored_fit_samples=768
fit_span_xyz=1003,1035,998
mag_cal_failure_reason=algebraic_residual_too_high
```

All preceding hard/soft gates had therefore passed: sample count, fit-set box coverage, ellipsoid solve, robust inlier ratio, normalized geometric residual and directional coverage. Only the final algebraic residual gate rejected the model.

## Exact defect

The solver fits the raw-coordinate equation:

```text
x^T A x + b^T x = 1
```

With center `c = -0.5 A^-1 b`, the centered equation is:

```text
(x-c)^T (A/k) (x-c) = 1
k = 1 + c^T A c
```

The previous implementation used the RMS of the raw equation residual directly. That residual is multiplied by `k`, so it changes when an otherwise identical ellipsoid is translated by a different hard-iron offset. The quality gate was therefore not translation-invariant and could reject a physically good fit solely because the raw magnetic center was far from zero.

A native regression reproduced the defect with the same quantized ellipsoid translated from the origin to `(400,-300,180)`: the old algebraic RMS increased by more than an order of magnitude while normalized geometric error remained unchanged.

## Fix

`fitFromAccumulator()` now divides raw algebraic RMS by `abs(k)`. The retained metric describes the centered dimensionless ellipsoid equation and is approximately twice normalized geometric norm error for small residuals. The configured `0.12` ceiling is unchanged; its implementation now matches its intended physical meaning instead of being relaxed.

Rejected finite fits now retain their candidate hard-iron, radii, axis ratio, coverage, normalized algebraic/geometric residuals and inlier metrics in `lastResult()`. Compact status, detailed status and apply-failure output expose two compact tuples: `last_fit_quality=algebraic,geometric,directional,inlier,axis_ratio,box_coverage` and `last_fit_quality_limits=max_algebraic,max_geometric,min_directional,min_inlier,max_axis_ratio,min_box_coverage`. Future hardware failures are diagnosable without adding temporary logging or duplicating large print paths.

## Quality-preservation regressions

Native tests cover:

1. translation invariance of the normalized algebraic residual;
2. acceptance of mild physically bounded non-ellipsoidal variation that previously failed only because of hard-iron translation;
3. rejection of strongly non-ellipsoidal data by the physical geometric gate;
4. persistence of failed-fit metrics for diagnostics.

The patch does not change sample reservoirs, ODR, FIFO, SPI, AHRS, networking, NVS schema, candidate format or calibration ownership.

## Required hardware acceptance

Run:

```text
perf on
perf reset
setup calibration full
setup status
mag cal status
perf status
```

A successful run should no longer fail solely from an origin-dependent `algebraic_residual_too_high`. If a quality gate still fails, capture the new fields:

```text
last_fit_quality
last_fit_quality_limits
```

## Source-filter and code-size audit

An intermediate implementation placed the shared diagnostic formatter in `runtime/mag_status_reporter.cpp`. Production, Production-Diag and Slim explicitly exclude that source, so `mag_runtime_controller.cpp` would have acquired an unresolved symbol in those profiles. The final implementation is a GCC/Clang `noinline inline` helper in `mag_status_reporter.hpp`; each using translation unit emits a weak COMDAT definition and the linker keeps one copy. The policy forbids moving it back into the excluded source.

A comparable host relocatable link of the four affected objects changed:

```text
0023ga: text=110667 data=0 bss=4
0023gb: text=112970 data=0 bss=4
Delta:  text=+2303 data=0 bss=0
```

This is a direction-only host measurement, not an ESP32 PlatformIO size claim. The actual firmware size gate remains `check_all.py --clean --require-pio`.

Stack usage under Linux GCC `-O2 -fstack-usage`:

```text
MagCalibrationCollector::compute            2016 bytes (unchanged)
MagRuntimeController::applyCalibration      1136 bytes (unchanged)
magStatusPrintCalibrationFitQuality           96 bytes
magStatusPrintCalibration                    272 bytes (unchanged)
```

## Validation completed

```text
test_mag_calibration: GCC PASS
test_mag_calibration: Clang PASS
test_mag_calibration: ASan+UBSan PASS
test_mag_heading_reliability: PASS
0023g policy: PASS
0023ga policy: PASS
0023gb policy: PASS
calibration integration policy: PASS
magnetic heading reliability policy: PASS
calibration storage stack policy: PASS
source-filter validation: PASS
profile-matrix validation: PASS
documentation validation: PASS
```

The complete standalone matrix compiled every project and compile-only source through the modified app/runtime files before the environment time limit, but did not finish linking/running the entire unchanged test inventory. PlatformIO is not installed in the assistant environment, so ESP32 builds are not claimed.
