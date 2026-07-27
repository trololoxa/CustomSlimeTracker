# 0023gc mag-fit stack and profile-build hardening

## Confirmed defects

### 1. Cross-ABI mag-fit stack overflow

Windows/MSYS2 GCC reported:

```text
MagCalibrationCollector::compute(MagCalibrationResult&) uses 2240 bytes
policy limit: 2048 bytes
```

Linux GCC measured 2016 bytes, leaving only 32 bytes below the old ceiling. The function kept two `FitAccumulator` instances alive simultaneously. Each accumulator owns a 9x9 `double` normal matrix, a 9-element `double` RHS and a count. The first held the raw filtered fit set while the second held the robust inlier refit set. MSYS2 ABI/spill differences pushed the frame over budget.

The fix retains the raw sample count, then reuses the same accumulator for the inlier pass. Initial fit, robust threshold, inlier ratio, conditional refit and final quality checks are unchanged. Linux `-O2 -fstack-usage` measures 1296 bytes after the change. A new 1792-byte ceiling leaves cross-ABI margin instead of merely fitting the original 2048-byte boundary. No heap, global workspace or persistent RAM was added.

### 2. Production/Slim compile failure

Mandatory PlatformIO builds reported:

```text
tracker_app_mag_hooks.hpp: error:
magStatusPrintCalibrationFitQuality was not declared in this scope
```

The compact `printMagCalibrationStatus()` path is compiled when `TRACKER_ENABLE_DETAILED_MAG_STATUS=0`, but `tracker_app_hooks.hpp` included `mag_status_reporter.hpp` only when detailed status was enabled. `0023gb` had placed the shared helper inside that conditional header, so Production, Production-Diag and Slim could parse the call without seeing its declaration.

The helper now lives in `runtime/mag_calibration_fit_quality_reporter.hpp`, which includes only Arduino `Stream` and the calibration model API. The app composition includes this minimal header unconditionally, while the detailed reporter includes the same header for Debug. `mag_status_reporter.cpp` remains excluded from reduced profiles and no out-of-line link dependency is introduced.

## Regression review

- Hard/soft fit samples, reservoir contents and quality thresholds are unchanged.
- `0023gb` centered algebraic residual normalization is unchanged.
- The same accumulator is reset by the existing accumulation helpers before the inlier pass, so stale raw normal-equation state cannot leak into refit.
- Only one large local workspace is live in `compute()`; no heap allocation or shared mutable global was introduced.
- The reporter move changes compile-time visibility only. Output field names and values are unchanged.
- Config schema remains 2 and candidate format remains 3.
- FIFO, ODR, AHRS, calibration promotion, NVS and networking semantics are unchanged.

## Verification gates

The patch is intended to pass:

```text
test_calibration_0023g_policy.py
test_calibration_0023ga_policy.py
test_calibration_0023gb_policy.py
test_calibration_0023gc_policy.py
test_mag_calibration
test_mag_heading_reliability
source-filter validation
profile-matrix validation
documentation validation
Production / Production-Diag / Slim PlatformIO builds
```

The new policy compiles the helper under Production and Slim profile contracts and applies the stricter stack ceiling. Real PlatformIO builds remain authoritative for the complete ESP32 composition and link.
