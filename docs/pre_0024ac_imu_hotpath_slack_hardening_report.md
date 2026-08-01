# pre-0024ac IMU hotpath and slack-admission hardening report

## Hardware evidence

The `pre-0024ab` hardware log contains a long mostly-idle window and a second active-motion window with walking, rotations and taps. The predecessor transform cache removed the persistent stale-backlog failure: the software queue returned to zero, high-water was 49-52 samples, processed software-age p99 was 5-7.5 ms, rotation software-age p99 was 7.5-10 ms, and there were no FIFO overrun/full, timestamp fallback or tracking-recovery events.

The remaining measured pressure was CPU/scheduling rather than packet loss:

- sampled raw callback average was 657.8 us while idle and 656.4 us in motion at a 1054 us IMU period;
- the mandatory nested 100 Hz bundled rotation send averaged about 0.83 ms and consumed about 7.2% of observed loop time;
- optional runtime hooks were still entered every ordinary loop even when no calibration work existed;
- short queue bursts still reached roughly 47-54 ms, although they recovered without stale-state persistence;
- the existing aggregate FIFO callback metric did not reveal how much time belonged to calibration, quality, AHRS/recovery, output preparation or diagnostic/output hooks.

The evidence does **not** justify sensor-history dropping or reduced correction cadence. Freshness recovered automatically and tracking delivery remained 99.99%, so this wave removes repeated hotpath work and prevents optional phase collisions without weakening fusion quality.

## Confirmed hotpath defects

### Repeated temperature-compensation evaluation

The same temperature model and current gyro bias were evaluated multiple times for one IMU sample: during calibrated-sample construction, temperature quality marking, runtime-bias gating and runtime-bias residual construction. Those paths used the same temperature and immutable model revision, so recomputation added CPU cost without adding independent evidence.

### Disabled serial stream still read the MCU clock

The per-sample output path called `micros()` before discovering that the ordinary serial stream was off. Production tracking therefore paid a 960 Hz clock-read tax even with no sample stream enabled.

### Optional services had no shared admission contract

Battery, LED, magnetic deferred work, autonomous calibration and remote console were independently polled from the ordinary loop. They could start while software FIFO work was pending or close to the next 100 Hz rotation deadline. Magnetic deferred work and calibration autonomy could also both consume one precomputed slack snapshot in the same loop, recreating a background phase collision.

### Missing per-stage sample telemetry

The sampled raw-callback total showed the size of the problem but could not identify the next safe optimization. Without bounded stage timings, changing AHRS or correction cadence would be guesswork.

## Implementation

### One lightweight temperature evaluation per IMU sample

`GyroTempCompensator::evaluateRuntime()` returns only the fields required by the 960 Hz path: current bias, validity/range state and extrapolation confidence. The full human-readable snapshot now derives from the same runtime evaluation, preserving one source of truth.

`imuSamplePipelineProcessRaw()` evaluates this state once and reuses it for:

- persistent plus runtime gyro-bias subtraction;
- temperature quality flags;
- runtime-bias temperature admission;
- runtime-bias residual construction.

Compatibility overloads remain for standalone callers. Focused tests compare the lightweight state with the full snapshot and compare old/new runtime-bias overload results exactly, including disabled, invalid, in-range, extrapolated and non-finite inputs.

### Serial clock-read gate

The per-sample stream path checks `Off` and `Heartbeat` modes before calling `micros()`. Machine logging, explicit raw/scaled/calibrated streaming and all packet outputs retain their previous behavior.

### Shared optional-service admission

A fixed, header-only admission rule rejects optional work while:

- the software FIFO still has pending work;
- FIFO urgency is active; or
- known rotation-deadline slack is below the service-class threshold.

Thresholds are deliberately conservative:

```text
short service       1500 us
remote console      2500 us
background work     3500 us
```

The ordinary loop takes only three shared admission snapshots rather than querying the clock/deadline separately for every service. Critical FIFO/AHRS work, the outer SlimeVR network lifecycle and tap polling are not gated.

At most one background worker may complete work in one loop. If magnetic deferred service actually performs work, autonomous calibration waits for the next admitted loop. If magnetic service merely polls and has nothing pending, autonomy may use the same background slot. Manual blocking calibration/service paths bypass this ordinary-loop admission and retain transaction ownership.

### Sampled IMU-stage profiler

When `perf on` is active, one of every 64 IMU samples records fixed-memory stage timing for:

```text
scale_calibration
quality
ahrs_recovery
prepared_output
per_sample_outputs
```

The sampling divisor prevents the profiler from becoming another full-rate tax. Optional-service admission skips are counted separately for battery, LED, magnetic deferred work, calibration autonomy and remote console.

## Tracking-quality contract

**No IMU sample is dropped, skipped, reordered or batch-substituted.**

The patch does not change:

- IMU ODR or high-performance sensor mode;
- FIFO pairing, hardware timestamps or chronological magnetic dispatch;
- gyro integration frequency;
- accel observation/correction cadence;
- magnetic heading/yaw equations;
- AHRS equations or quaternion publication;
- prepared-output or SlimeVR target rates;
- packet payloads, calibration models or persistent schemas;
- tap polling cadence.

Deferring optional services can delay background evidence/diagnostic progress under pressure, but cannot change the active calibration or current orientation. Admission-skip counters expose pressure; subsystem-specific pending/drop counters remain authoritative for actual background-work starvation.

## Hardware acceptance

After flashing, collect the same idle and active-motion windows:

```text
perf on
perf reset
motion off
# 10-15 minutes
perf status
perf tracking

perf reset
motion on
# movement, rotations, walking and taps for 10-15 minutes
perf status
perf tracking
motion status
```

Key new fields:

```text
perf_optional_service_admission_skips_battery
perf_optional_service_admission_skips_led
perf_optional_service_admission_skips_mag_deferred
perf_optional_service_admission_skips_calibration_autonomy
perf_optional_service_admission_skips_remote_console
perf_imu_stage_sample_divisor
perf_imu_stage=scale_calibration ...
perf_imu_stage=quality ...
perf_imu_stage=ahrs_recovery ...
perf_imu_stage=prepared_output ...
perf_imu_stage=per_sample_outputs ...
```

Acceptance requires zero FIFO overrun/full and recovery deltas, no monotonic queue-age growth, zero autonomy/magnetic completed-window drops, stable 100 Hz delivery and no tracking-quality regression. Optional admission skips may be nonzero during pressure; they should stop increasing rapidly after the queue clears.

## Validation performed

- 64 production, compile-only and Arduino translation units compiled with the standalone-runner flags.
- 44/44 native test executables linked and passed after the final profiler/admission naming audit.
- Focused runtime-bias test passed with ASan/UBSan.
- All predecessor policies through `0023gl`, `pre-0024`, `pre-0024a` and `pre-0024ab` passed; the new `pre-0024ac` policy passed.
- Calibration integration and magnetic heading reliability policies passed.
- Source-filter, profile-matrix and documentation validation passed.
- 60-second replay, MAGR replay, replay metric equality and the 600-second baseline replay passed without metric changes.
- Linux `-O2` stack usage was 352 bytes for `imuSamplePipelineProcessRaw`, 160 bytes for `TrackerApp::loop`, and 128 bytes for the shared-evaluation `runtimeBiasUpdateEstimator` overload.

PlatformIO is not installed in the audit environment, so ESP32-C3 compilation remains an explicit hardware-owner gate:

```bash
python3 tools/check_all.py --clean --require-pio
```
