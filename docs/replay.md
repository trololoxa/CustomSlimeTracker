# Replay and metrics

Replay is the host-side way to compare tracking behavior between firmware versions using the same captured log.

## What replay uses

The compatibility replay gate uses machine-readable E0 logs. New release
evidence must use strict LOGVER3 captured by the ProductionDiag TCP session:

```text
python3 tools/capture_telnet_log.py --host <tracker-ip> --seconds 600 \
  --capture static --rate 20 --mode full --output logver3_static_clean_001.log
```

The important lines are CSV-like frames announced by `LOGFMT`:

```text
Q,FIFO,CAL,BIAS,BIASUPD,MAG,MAGR,YAW,STATE,NET,TESTSUM,LOGSUM,LOGSTAT,TEMPBIN
```

The compatibility replay ignores human CLI/status text. The strict capture gate
also consumes the compact completion marker and final console counters, and
rejects any `# ERR` line; numeric replay metrics still depend only on stable
machine-readable frames.
The `LOGVER` row carries `build_profile`, `pio_env` and `git` fields. Strict
capture requires ProductionDiag and a clean full 40-hex commit; dirty and
abbreviated identities fail closed.

## Strict LOGVER3 integrity gate

Run the structural gate directly with:

```bash
python3 tools/replay/strict_logver3_gate.py \
  --log logver3_static_clean_001.log \
  --capture static \
  --output logver3_static_clean_001.validation.json
```

It validates the exact `LOGFMT` schema and field counts, finite numeric values,
unquoted CSV, known frame/stat types, contiguous global sequence, ordered
Q/FIFO/BIAS/CAL and MAG/MAGR/YAW bundles, monotonic timestamps, quaternion norm,
duration and observed Q rate. It also requires hardware timestamps, zero
FIFO/drop/recovery/backpressure/console-abort counters, a completely drained
deferred pipeline and bounded maximum record age.

LOGVER3 E1 adds one-hertz cumulative `NET` chronology and one exact immutable
post-window `TESTSUM`. Static validation fails closed on UDP deadlines/sends,
MAG trust/heading/field semantics, missing calibrated BIAS state, test-window
fault deltas or a remote lease expiration. Runtime diagnosis uses
`--capture runtime`: schema, chronology, loss and complete drain remain strict,
but observed sensor/network health faults are retained in `health_failures`
with `health_passed=false` so the problem log is not discarded.

LOGVER2 remains a compatibility input for `replay_machine_log.py`; it is never
accepted by the strict LOGVER3 gate.

## Current replay level

The first replay tool is metric replay, not firmware-in-the-loop simulation:

```bash
python tools/replay/replay_machine_log.py tracker.log --pretty
```

It parses the log and computes deterministic metrics such as:

- duration;
- quaternion confidence and diagnostic yaw drift projection;
- FIFO fallback/overrun/full/unknown rows;
- accel trust/norm stats;
- gyro norm stats;
- mag trusted/rejected rows and reject flags;
- full-mode `MAGR` raw/calibrated/body magnetometer vectors for magnetometer replay fixtures;
- yaw correction gate/apply stats;
- tracking state events;
- runtime bias update stats.

You can turn metrics into a gate:

```bash
python tools/replay/replay_machine_log.py tracker.log \
  --min-duration-s 100 \
  --max-fifo-fallback-rows 0 \
  --max-fifo-fault-rows 0 \
  --max-recovering-rows 0
```


## Magnetometer sweep fixture

For magnetometer calibration and axis-mapping work, capture a dedicated full log
with raw magnetometer vectors. `log full` now emits `MAGR` frames:

```text
MAGR,t_us,seq,mag_seq,raw_x,raw_y,raw_z,cal_x,cal_y,cal_z,body_x,body_y,body_z,raw_norm,cal_norm,body_norm,raw_flags,reject_flags,trusted
```

Recommended capture sequence:

```text
stream off
output stop
mag enable save
mag heading auto off
mag yaw disable save
log reset
log full
log rate 20
log header
mag cal reset
mag cal start
# slowly rotate the tracker through many orientations for 60-120 seconds
mag cal stop
mag cal status
log summary
log off
```

Score it with MAGR-specific gates, adjusting the axis-span threshold after the
first real sweep establishes typical raw units:

```bash
python tools/replay/replay_machine_log.py logs/mag_sweep.log \
  --require-magr \
  --min-magr-rows 500 \
  --pretty
```

`MAGR` is intended for host-side magnetometer fitting and regression checks.
Human `mag status`/`mag cal status` output remains useful for inspection, but
should not become replay input.

## Replay levels

1. **Metric replay**: parse logs and compare counters/stats. This exists now.
2. **Strict capture integrity**: exact LOGVER3 schema/chronology/drop/provenance gate. This exists now.
3. **Golden comparison**: bind one real fixture SHA-256 to independently reviewed metric thresholds. The code exists; the first real fixture is still pending hardware capture.
4. **Algorithm replay**: feed recorded calibrated samples into host-safe AHRS/mag/bias modules and compare output. This requires logging enough calibrated/raw data and keeping the algorithm modules host-safe.
5. **Scenario library**: static desk, hand motion, magnetic disturbance, FIFO stress, warm-up/temperature sweep.

## Why this matters

Without replay, tracking changes are judged by feel. With replay, changes can be checked against the same data for drift, recovery count, accel trust, mag rejection, yaw correction behavior, and bias stability.

## Authoritative baseline log

The canonical replay fixture should be captured from machine-readable logging,
not from human `status` output. Use `log full` so the replay parser receives
`Q`, `FIFO`, `CAL`, `BIAS`, `MAG`, `MAGR`, `YAW`, `STATE`, `LOGSUM`, and `LOGSTAT`
frames when those systems are active. `MAGR` is emitted only in `log full` mode.

Recommended baseline command (battery powered, USB physically disconnected):

```bash
python3 tools/capture_telnet_log.py --host <tracker-ip> --seconds 600 \
  --capture static --rate 20 --mode full --output logver3_static_clean_001.log
```

The resulting serial capture is the input for:

```bash
python tools/replay/replay_machine_log.py logs/baseline.log --pretty
```

For regression checks, score the previous and new logs, then compare JSON:

```bash
python tools/replay/replay_machine_log.py logs/baseline_old.log --output before.json
python tools/replay/replay_machine_log.py logs/baseline_new.log --output after.json
python tools/replay/compare_replay_metrics.py before.json after.json --pretty
```

## Committed replay baseline

`tests/fixtures/replay/baseline_replay_001.log` is the first real firmware
replay fixture. It is captured as `baseline_no_yaw_apply`: magnetometer and YAW
frames are present, but yaw reference/apply are intentionally not enabled.

`tools/check_all.py` runs it through `replay_machine_log.py` with these baseline
gates:

```text
min duration:            600 s
max FIFO fallback rows:  0
max FIFO fault rows:     0
max RECOVERING rows:     0
max yaw drift diag:      2.0 deg/min
```

Warnings about missing yaw application are expected for this fixture. Add a
separate fixture later when testing mag reference + yaw correction apply.


## LOGVER3 magnetic fields

LOGVER3 E1 keeps the E0 magnetic additions from patch 0022 and enforces them as
an exact schema. `MAG` contains world dip, field state/trust/flags and norm/dip/heading
reference errors. `YAW` adds normal/reacquisition mode, pending/active state,
stable-field duration and magnetic heading rate. In 0022a the logged
`field_heading_rate_deg_s` is the filtered signed-rate magnitude used by
reacquisition rather than the instantaneous two-sample derivative; CLI status
prints both values. `tools/logs/parse_e0_log.py` remains compatible with LOGVER2;
`strict_logver3_gate.py` is intentionally not.
