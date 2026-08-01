# pre-0024 hotpath headroom foundation

## Scope

`pre-0024_hotpath_headroom_foundation` is a standalone performance and observability wave between `0023gl` and the AHRS quality wave `0024`. It addresses demonstrated scheduling pressure without changing tracking equations, IMU ODR, FIFO sample order, hardware timestamp reconstruction, AHRS correction decisions, SlimeVR packet layouts, output target rate, calibration models, persistent schema, or network protocol.

The input hardware telemetry used for planning was collected from one stationary tracker on a separate router with `perf on` and `motion on`: average loop time was 11.035 ms, the historical `fifo` section occupied 92.57% of measured loop time, no idle yield occurred, rotation delivery remained 99.99%, 79.8% of due rotations were late, and the raw queue reached 83 samples. Because the old `fifo` bucket included SPI drain, raw callbacks and nested full network service, this patch first separates the relevant costs and reduces known periodic/idle taxes.

## Quality-preservation contract

The patch deliberately does **not** implement stale-history dropping, gyro decimation, accel-correction decimation, changed magnetic cadence, or controlled catch-up. Every raw sample remains FIFO ordered and every due magnetic sample is completed before the raw timeline advances. The existing tap poll interval remains 5 ms.

Runtime gyro-bias and calibration-autonomy window mathematics move out of the 960 Hz callback, but the evidence is copied exactly. A completed runtime-bias window asks the FIFO processor to yield only after magnetic samples with the same timestamp are dispatched. The deferred decision is finalized before the next raw IMU sample, preserving the estimator update boundary. The autonomy collector similarly copies the complete accumulator and finalizes it later in the same app lifecycle; if service is unexpectedly stalled, background evidence is dropped fail-closed rather than blocking tracking.

## Changes

### Absolute FIFO slice budget

Hardware SPI drain and raw/magnetic callbacks now share one absolute slice timer. Near an armed pose deadline, a new drain round is reduced to 16 or 32 words. A 750 us emergency floor preserves bounded hardware progress without dropping or reordering samples. The previous application-level normal/urgent budgets remain intact.

### Narrow nested network service

FIFO catch-up no longer calls the full Wi-Fi/SlimeVR runtime. It may only send the newest due rotation for an already established healthy session. Discovery, incoming packets, feature/config work, telemetry, Wi-Fi information polling, timeout/recovery and remote console remain in the ordinary outer update.

### Bounded battery acquisition

The old sparse battery update performed up to 128 synchronous ADC conversions and an insertion sort in one loop. The same discard and trimmed-mean estimator is now built over at most two conversions per service. Values are inserted into the sorted bounded array incrementally, so no final sort burst remains. Startup battery telemetry becomes available after the bounded batch completes rather than immediately during setup.

### Lower idle and periodic taxes

The remote-console listening socket is polled at 20 Hz instead of every app loop; the busy response uses a best-effort nonblocking send and no `flush()`. Status LED policy is derived only at its configured update interval. Slow Wi-Fi diagnostics (RSSI, power-save mode, TX power and MAC) are cached for one second while link/IP state remains fresh. Empty autonomy service is bypassed unless an exact pending/deadline condition exists.

### Lower diagnostic cost

Exact motion-quality, recovery, saturation and dropped-sample counters still observe every IMU sample. Expensive gyro norm, temperature, confidence and aggregate metrics are sampled at 1/16 cadence and report both total samples and metric samples. Runtime profiler bookkeeping overhead is measured separately.

### Freshness and headroom telemetry

The optional DIAG profiler adds fixed-memory histograms for section p50/p95/p99, 10 ms frame busy/headroom, profiler overhead and software-pipeline age. Exact per-entry software queue timestamps exist only when the runtime profiler is compiled, avoiding the additional 2 KiB queue-age array in ordinary Production.

`software_age` starts when a sample leaves the hardware FIFO and enters the application raw queue. Hardware-FIFO residence is intentionally not represented as known; FIFO unread/backlog counters and raw timestamp span remain separate evidence. This avoids presenting a lower-bound software age as a true end-to-end sensor age.

Additional status includes:

- raw queue wait/oldest/span current and maximum;
- hardware drain and callback average/maximum;
- slice-budget stops and near-deadline reduced drains;
- outer versus nested network sections;
- processed, prepared and sent rotation software age histograms;
- battery batch state and maximum reads per service;
- runtime-bias/autonomy deferred-window and drop counters.

## Expected hardware effect

This wave should remove several known periodic spikes and prevent hardware drain plus callbacks from monopolizing one output frame. It also makes the remaining cost attributable. It does not yet guarantee a maximum end-to-end sensor age because age-based urgency and controlled catch-up are intentionally reserved for the next performance wave.

## Host acceptance

Required gates include:

- deterministic battery estimator equivalence and two-read service bound;
- exact per-sample diagnostic counters with sampled expensive metrics;
- profiler histogram, frame and micros-rollover tests;
- runtime-bias numerical-equivalence and before-next-sample ordering;
- autonomy lifecycle tests;
- FIFO raw/magnetic chronology and typed yield tests;
- critical SlimeVR update isolation;
- ASan/UBSan focused tests;
- cross-ABI stack ceilings;
- full native source/test matrix, profile/source/document policies and replays;
- strict patch apply and byte/mode tree equality.

## Hardware acceptance

Run the required build gate first:

```bash
python3 tools/check_all.py --clean --require-pio
```

Then collect four ten-minute baselines with `perf reset` between phases:

1. Production, diagnostics off, stationary and normal movement.
2. Production DIAG, `perf on`, `motion off`.
3. Production DIAG, `perf on`, `motion on` (sampled metrics).
4. Short forensic run with serial/remote output actively consumed.

Repeat the DIAG baseline with 9 trackers and simultaneous boot. Capture:

```text
perf status
perf tracking
motion status
fifo stats
slime status
```

Acceptance is no tracking-quality regression, no FIFO overrun/full/recovery, no sustained queue growth, no background-window drops, bounded battery batches (`max_reads_per_service<=2`), and materially lower remote/battery/diagnostic pressure. The new telemetry should be used to define the following age-based scheduler/catch-up wave before `0024`.

## Verification completed in the audited tree

The audited tree passed:

- 64 production, compile-only and Arduino-composition translation units;
- 43/43 standalone native test executables;
- the complete source/profile/documentation policy chain through `0023gl` plus the new pre-0024 policy;
- focused ASan/UBSan coverage for the deferred runtime-bias path;
- deterministic battery, profiler, motion-diagnostics, FIFO chronology, autonomy lifecycle and critical-network regressions;
- the 60-second replay, MAGR replay, replay-metric equality and 600-second baseline replay.

Measured Linux `-O2 -fstack-usage` maxima for new/changed bounded paths were:

```text
FifoRuntimeProcessor::process                 144 bytes
runtimeBiasFinalizePendingWindow              192 bytes
CalibrationAutonomyController::service        144 bytes
BatteryAdcBatchSampler::service                64 bytes
RuntimeProfiler::recordLoopInterval             8 bytes
```

PlatformIO was not present in the audit environment. No ESP32-C3 firmware-build claim is made; `python3 tools/check_all.py --clean --require-pio` remains mandatory before hardware acceptance.

## Deliberately deferred work

This foundation does not yet add the central absolute-deadline scheduler, age-based urgency, stale-history fast-forward, controlled catch-up, full critical/deferred magnetic split or NVS/solver maintenance slots. Those changes alter freshness/recovery semantics and require the telemetry and headroom evidence produced by this wave. Keeping them out avoids hiding a tracking-quality change inside an observability/low-risk scheduling patch.
