# pre-0024ab hotpath transform-cache hardening report

## Scope and evidence

The `pre-0024aa` hardware log contained separate idle and active-motion windows. Both showed clean Wi-Fi and UDP delivery, zero FIFO overrun/full counters and complete hardware timestamps, yet the software raw queue reached its 512-entry high-water mark. Live queue age was roughly 150 ms while idle and 360-370 ms during motion, with recorded maxima near 550 ms. Sampled raw callback cost was approximately 0.69-0.72 ms at an IMU period near 1.054 ms. The log therefore points to per-sample CPU cost and backlog, not packet loss, as the next safe optimization target.

The same log exposed a diagnostic defect: software-age p50/p95/p99 fields printed `4294967295` whenever the percentile entered the histogram's open-ended bucket above 100 ms. That value was a bucket sentinel, not a measured latency.

## Confirmed hot-path defect

`imuPipelineMakeCalibratedSample()` reconstructed and validated the same sensor-to-device rotation for every IMU sample. Validation performs finite checks, three vector norms, orthogonality dot products and a determinant. The matrix is immutable between authoritative tracker-config revisions. Magnetic preprocessing repeated the same validation at its own sample rate, and `MagRuntimeController::runtimeConfig()` rebuilt and copied the complete immutable runtime configuration for every magnetic sample.

## Implementation

A fixed-memory `SensorToDeviceFrameCache` now resolves the validated frame by the authoritative tracker-config CRC. Unchanged revisions take the constant-time cached path. A changed revision always calls the existing `makeSensorToDeviceFrame()` validator, so invalid scale, shear, reflection, non-finite values or an invalid flag remain fail-closed. The IMU pipeline and magnetic controller share the cache through explicit dependencies. Standalone/direct callers without that dependency retain the original validation fallback.

`MagRuntimeController` caches the complete immutable `MagRuntimeConfig` by the same revision and returns it by const reference, eliminating repeated construction and value copying. Configuration publication semantics are unchanged: authoritative mutations must sanitize and call `updateCrc()` before the new revision becomes live.

The fixed-memory latency histogram now has finite bounds at 150, 250, 500, 750, 1000 and 2000 ms. If a percentile reaches the final open-ended bucket, it reports the actual observed maximum rather than `UINT32_MAX`.

## Tracking-quality contract

**No tracking equation, cadence or packet payload changes.**

The patch does not change IMU ODR, hardware timestamp use, gyro/accel pairing, sample order, full-rate gyro prediction, accel correction divisor, magnetic heading/yaw calculations, prepared-output cadence, SlimeVR target rate, packet layout, calibration models or persistent schemas. It does not drop, decimate, batch-integrate or reorder sensor samples.

## Tests

Focused tests cover constant-revision cache hits, changed-revision refresh, invalid changed matrices, sanitization recovery, shared magnetic prevalidated input, direct magnetic fallback validation and overload histogram percentiles. The policy compiles changed IMU and magnetic boundaries with stack-usage reporting and locks the unchanged AHRS/output cadence constants.

## Hardware acceptance

After flashing, collect idle and active-motion windows with `perf on`, `perf reset`, and the same commands used for the predecessor log. Expected improvements are lower sampled raw-callback time and reduced queue-age growth. The extended age percentiles must show finite measured values rather than `4294967295`. This patch does not yet implement stale-history dropping or controlled catch-up; if queue age still grows, the next wave must address scheduling/freshness rather than weaken AHRS quality.

## Validation performed

- 64 production, compile-only and Arduino translation units compiled with the standalone runner flags.
- 43/43 native test executables linked and passed.
- The compact frame-cache boundary passed normal optimization and ASan/UBSan.
- `0023ge`, `0023gf`, `0023gg`, `0023gk`, calibration integration, magnetic heading reliability, predecessor pre-0024 and the new pre-0024ab policy passed.
- Source filters, profile matrix and documentation validation passed.
- The 60-second replay, MAGR replay and 600-second baseline replay passed without metric changes.
- Linux `-O2` stack usage was 256 bytes for `imuSamplePipelineProcessRaw`, 176 bytes for `MagRuntimeController::processRawSample`, and 240 bytes for the rare `runtimeConfig()` refresh/cache boundary.

PlatformIO is not installed in the audit environment, so ESP32-C3 compilation remains an explicit hardware-owner gate:

```bash
python3 tools/check_all.py --clean --require-pio
```
