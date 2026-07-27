# 0023gg magnetic timestamp and setup acceptance hardening

## Scope

This suffix follows `0023gf_magnetometer_callback_cross_abi_hardening` and addresses the first complete hardware guided-calibration run after the mathematical repairs. The run proved that hard/soft-iron fitting and three-axis motion collection now work, but exposed three independent acceptance defects:

1. `dynamic_axis_gyro_skew_rejected=4255` despite healthy FIFO counters;
2. automatic axis alignment returned `training_validation_disagreement` even though both partition scores were low; the old diagnostics did not distinguish coarse axis/sign disagreement from disagreement only in continuous refinement;
3. final output verification rejected healthy output because a fixed four-second dwell produced only 207 coherent input observations while the unchanged evidence requirement was 256.

No config schema, candidate format, SlimeVR protocol, ODR, FIFO register policy, AHRS policy, or NVS ownership semantics are changed.

## Root cause 1: free-running magnetic timestamps

A sensor-hub FIFO word has no dedicated timestamp tag. The previous parser anchored the first QMC6309 frame to the IMU/FIFO timeline and then advanced every later frame by the configured nominal `16666.7 us` period. Any real sensor-hub ODR error, phase shift, batching difference, or oscillator mismatch accumulated permanently. Once accumulated error exceeded the shared 5000 us endpoint-skew gate, guided and background `magToImu` collectors rejected nearly every frame. The hardware symptom was 4255 gyro-skew rejections out of 4521 magnetic samples.

`0023gg` anchors every sensor-hub frame to the most recent timestamp available in the current FIFO/IMU domain. Nominal-period extrapolation is retained only when no IMU/FIFO anchor exists. A non-advancing anchor receives a one-microsecond monotonic marker which is deliberately rejected by interval timing rather than fabricating a full nominal period and creating phase drift.

New diagnostics distinguish:

- IMU-anchored magnetic frames;
- nominal fallback frames;
- monotonic corrections;
- latest and maximum correction from the free-running nominal prediction.

## Root cause 2: continuous-refinement disagreement discarded proven axis/sign mapping

The alignment solver independently fits training and validation partitions. Its safety-critical result is the coarse proper signed-permutation mapping. A small continuous SO(3) refinement represents mounting error around that discrete mapping.

The old gate combined two conditions: the coarse winner had to match and the continuous refinements had to agree within 1.25 degrees. The hardware log exposed only the combined failure. Code audit showed that, even when both partitions recovered the same proper axis/sign mapping, slightly different small mechanical refinements caused the entire result to be rejected and setup asked the user to type a mapping manually.

`0023gg` remains fail-closed when the coarse signed-permutation winners disagree. When the coarse winner agrees but continuous refinements do not, the solver discards both disputed refinements, evaluates the shared coarse proper rotation on training, validation, and all retained intervals, and accepts it only if the existing score, direction, separation, observability, generalization, and SO(3) gates pass. Thresholds are not relaxed.

Diagnostics now separate coarse agreement, continuous-refinement agreement, and use of the conservative coarse fallback.

## Root cause 3: fixed verification dwell contradicted its own sample requirement

Final setup verification required 256 coherent input samples but always captured for exactly four seconds. The hardware run produced 207 coherent observations with healthy quaternion, acceleration, gyro, and FIFO metrics. The only failed predicate was therefore sample count, but the old report exposed only the combined `stationary_input_passed=no` result.

`0023gg` keeps the 256-sample requirement. Capture runs for at least four seconds, ends once both snapshot and input evidence requirements are satisfied, and remains bounded by eight seconds. The report prints capture duration and each stationary-input sub-gate independently.

Rollback wording now states that only the current stage transaction is restored and earlier committed resume-mode checkpoints remain authoritative.

## Regression coverage

Native tests cover:

- nominal sensor-hub ODR disagreement without accumulated magnetic timestamp drift;
- explicit hardware timestamp anchoring;
- same coarse axis/sign winner with disagreeing continuous refinements;
- true coarse disagreement remaining fail-closed through existing corrupted-validation tests;
- the exact 207-sample healthy-stationarity case and its isolated sample-count failure.

`tools/test_calibration_0023gg_policy.py` recompiles these tests, checks the timestamp and solver source contracts, and enforces stack ceilings for the FIFO parser, alignment solver, output verifier, and guided verifier orchestration. `tools/check_all.py` runs every predecessor suffix policy explicitly; the new policy deliberately does not recurse through the complete chain again.

## Realtime and resource direction

The new timestamp work is constant-time per 60 Hz magnetic FIFO word: two timestamp reads, one maximum, bounded integer diagnostics, and no heap allocation. It replaces the old nominal addition rather than adding another learner or queue. Setup verification runs only in an explicit guided command. Coarse fallback executes only in deferred/setup solve code, not in the IMU or network hot path.

Hardware acceptance must verify that `mag_timestamp_nominal_fallbacks` and `mag_timestamp_monotonic_adjustments` remain near zero during normal tracking, gyro-skew rejections collapse from the previous thousands, and FIFO/output/network deadline counters remain clean.

## Measured host resource direction

Relative to `0023gf`, the eight directly affected firmware translation units compiled with host GCC `-Os -ffunction-sections -fdata-sections` changed by:

- text: `+1941 bytes` before final linking;
- data: unchanged;
- object-local BSS: `+8 bytes`.

The runtime type-size changes on the host ABI are:

- `Lsm6dsvFifoReader::DrainStats`: `288 -> 312 bytes` (`+24` persistent bytes in the FIFO reader);
- `MagAxisAlignmentResult`: `196 -> 200 bytes`; two persistent autonomy results therefore add about `8` bytes;
- `SetupOutputVerificationResult`: `88 -> 96 bytes`, temporary setup-only;
- `SetupOutputVerificationAccumulator`: unchanged at `144 bytes`.

The effective persistent runtime growth is approximately `32 bytes`, with no heap allocation and no queue-capacity change. The only new normal-runtime arithmetic is bounded integer timestamp anchoring at the magnetic sensor-hub rate, approximately 60 Hz; no new work is added to the 960 Hz IMU callback, AHRS update, or network send path.

Linux GCC `-O2 -fstack-usage` measured:

- `parseSensorHubSlave0Word`: `48 bytes`;
- `solveMagAxisAlignmentDataset`: `1296 bytes`, unchanged from the predecessor;
- `SetupOutputVerificationAccumulator::finish`: `192 bytes`;
- `setupVerifyOutputRuntime`: `64 bytes`.

## Remaining physical limitation

The FIFO provides no timestamp physically attached to the QMC6309 word. The new timestamp is the nearest current IMU/FIFO-domain anchor, not a claim of sub-sample phase knowledge. Back-to-back magnetic words without an intervening IMU timeline advance receive a fail-honest one-microsecond monotonic marker and are rejected by interval timing. This may discard an ambiguous backlog frame, but it cannot create accumulated phase drift or silently pair it with a fabricated gyro interval.
