# 0023ge magnetometer post-audit realtime hardening

## Scope

This suffix is the acceptance-audit follow-up to
`0023gd_magnetometer_math_and_alignment_hardening`. It does not change the
hard/soft-iron model, `magToImu` objective, calibration thresholds, persistent
schema, ODR, FIFO hardware configuration, SlimeVR protocol or NVS semantics.

The review independently re-ran the magnetic fit/alignment regressions, inspected
all changed hot paths, compared stack and object-size direction against 0023gd,
and stress-tested magnetic FIFO backlog scheduling.

## Confirmed 0023gd defects found by the audit

### Due magnetic backlog could still advance the raw timeline

`FifoRuntimeProcessor::process()` correctly detected that the initial due-mag
backlog was not completely dispatched, but only incremented
`magChronologicalDeferrals`; it then entered the raw loop anyway. A sensor-hub
burst larger than the eight-callback allowance could therefore reintroduce the
very gyro/mag endpoint skew that 0023gd intended to remove.

0023ge carries an explicit `chronologicalReady` gate. If already-due mag frames
remain, raw callbacks do not advance until a later app pass drains them.

### Magnetic callbacks bypassed the cooperative time budget

The raw callback loop checked `FIFO_RUNTIME_SLICE_BUDGET_US`, while
`dispatchDueMagCallbacks()` was limited only by count. Up to eight newly-heavier
mag callbacks could run consecutively and exceed the 100 Hz output-service
budget. The callback dispatcher now checks the same post-drain cooperative
budget before every due mag frame and stops fail-closed at the current raw
endpoint.

Separate count and time-budget deferral counters are exposed through `status`
and `perf tracking`.

### Duplicate sensor-to-device SO(3) validation

One mag callback validated the same matrix in `MagRuntimeProcessor::process()`
and again in `MagRuntimeController::processRawSample()`. That repeated three
vector norms, three dot products and a determinant at 60 Hz. The processor now
retains whether the matrix was accepted for the current sample; the controller
reuses that decision and performs only the inverse transpose multiply for the
coherent gyro endpoint.

This decision adds no bytes to `MagProcessedSample` on the audited ABI because
the flag occupies existing padding. The now-incomplete private `last_` snapshot
and its unused overload are removed instead of adding synchronization; one
`MagProcessedSample` of persistent state is eliminated.
`FifoRuntimeQueueStats` grows by eight bytes for the two new diagnostics.

### Cross-ABI fit stack margin was too narrow

`MagCalibrationCollector::compute()` measured 1520 bytes on Linux against a
1792-byte ceiling. Earlier MSYS2 reports in this patch family showed roughly
224 bytes of ABI/compiler growth, leaving only about 48 bytes of practical
margin. The optional refit candidate now lives in a separate no-inline phase and
the geometric metrics workspace is reused. Linux GCC drops the orchestrator to
1216 bytes without changing the initial fit, inlier selection, refit or final
quality decisions. The new suffix ceiling is 1536 bytes. The unused by-value
`compute()` overload is removed so a future caller cannot reintroduce a hidden
result-return temporary.

### Early physical fit rejection lost completed normalization diagnostics

After successful centered normalization, `AxisRadiusTooSmall` and
`BoxCoverageTooLow` returned before copying the center, scale, sample count and
`Normalized` stage into the result. Guided setup could still print an opaque
all-zero solver state. Those diagnostics are now committed before the physical
pre-solve gates.

## Verification

The suffix adds regressions for:

- a delayed ten-frame mag burst with expensive callbacks;
- cooperative budget deferral without raw-timeline advancement;
- exact chronological recovery on the next pass;
- retained sensor-to-device validation and inverse mapping;
- invalid-frame identity fallback;
- non-zero normalization diagnostics on an early coverage rejection.

`0023ge` also reruns the complete `0023gd` policy, including the hard/soft fit,
all 24 proper mounting mappings, FIFO chronology and QMC saturation suites.

## Resource direction versus 0023gd

Host `-Os` object comparison against 0023gd:

- `MagRuntimeController::processRawSample`: code size decreases by 158 bytes
  at the function level and 908 bytes in the controller object;
- `FifoRuntimeProcessor::process`: +10 bytes;
- `dispatchDueMagCallbacks`: +50 bytes for the time-budget gate and split
  diagnostics;
- `MagRuntimeProcessor`: -200 object-text bytes after removing stale snapshot
  support;
- across the six audited firmware translation units (controller, FIFO, mag
  runtime, fit, status and perf), total host object text decreases by 849 bytes;
- controller and FIFO process stack frames remain unchanged on Linux GCC;
- `MagCalibrationCollector::compute()` drops from 1504/1520 to 1216 bytes
  (`-Os`/`-O2` predecessor measurements differed slightly);
- the processor object drops one 104-byte stale sample snapshot while queue
  diagnostics add eight bytes, for a net host-ABI persistent-state reduction of
  96 bytes;
- no heap allocation, queue growth or unbounded work is introduced.

These are host-direction measurements, not ESP32 linker totals. Mandatory
PlatformIO profile builds and Windows/MSYS2 stack policy remain hardware-side
acceptance gates.

## Residual risks not hidden by this suffix

- The independent train/validation rotation-agreement threshold remains
  deliberately conservative. Noisy but physically correct synthetic datasets
  can be rejected fail-closed and require another motion session; no incorrect
  mapping was accepted in the audit fuzzing.
- Robust hard/soft fitting can reject a dataset with sufficiently many or
  sufficiently structured outliers. It does not silently accept a poor model.
- The full aggregate `check_all --clean --require-pio` and physical guided setup
  remain necessary before declaring hardware acceptance.

## Independent mathematical audit results

A deterministic randomized hard/soft sweep exercised 300 translated, rotated
and anisotropic ellipsoids (up to the configured shape limit, with half of the
cases containing a small moderate-outlier population): 287 datasets were
accepted, every clean dataset was accepted, no accepted result exceeded the
error guard, maximum hard-iron error was about 4.42 raw counts and maximum
normalized residual was about 0.0381. The remaining disturbed cases rejected
fail-closed.

A separate `magToImu` sweep exercised 200 random proper mountings with bounded
mechanical residual and small synthetic gyro/mag noise. 156 passed; 44 rejected
only because the independent train/validation rotations disagreed by more than
the deliberately strict 1.25 degree ceiling. No incorrect mapping was accepted
and maximum accepted rotation error was about 1.31 degrees. The same sweep with
noise removed passed 200/200. This is retained as a conservative hardware-tuning
risk, not silently addressed by weakening the proof threshold.
