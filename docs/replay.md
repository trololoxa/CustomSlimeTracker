# Replay and metrics

Replay is the host-side way to compare tracking behavior between firmware versions using the same captured log.

## What replay uses

The current replay gate uses **machine-readable E0 serial logs** emitted by the firmware:

```text
log full
log header
test static 120
log summary
log off
```

The important lines are CSV-like frames announced by `LOGFMT`:

```text
Q,FIFO,CAL,BIAS,BIASUPD,MAG,MAGR,YAW,STATE,LOGSUM,LOGSTAT,TEMPBIN
```

Human CLI/status text is intentionally ignored. Replay should depend on stable machine-readable frames only.
The `LOGVER` row also carries `build_profile`, `pio_env` and `git` fields. The
parser exposes them under the JSON `logver` object so replay artifacts remain
traceable to both committed and intermediate dirty builds.

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

## Future replay levels

1. **Metric replay**: parse logs and compare counters/stats. This exists now.
2. **Golden comparison**: compare a new summary JSON against a saved baseline with tolerances.
3. **Algorithm replay**: feed recorded calibrated samples into host-safe AHRS/mag/bias modules and compare output. This requires logging enough calibrated/raw data and keeping the algorithm modules host-safe.
4. **Scenario library**: static desk, hand motion, magnetic disturbance, FIFO stress, warm-up/temperature sweep.

## Why this matters

Without replay, tracking changes are judged by feel. With replay, changes can be checked against the same data for drift, recovery count, accel trust, mag rejection, yaw correction behavior, and bias stability.

## Authoritative baseline log

The canonical replay fixture should be captured from machine-readable logging,
not from human `status` output. Use `log full` so the replay parser receives
`Q`, `FIFO`, `CAL`, `BIAS`, `MAG`, `MAGR`, `YAW`, `STATE`, `LOGSUM`, and `LOGSTAT`
frames when those systems are active. `MAGR` is emitted only in `log full` mode.

Recommended baseline sequence:

```text
setup status
log reset
log full
log rate 20
log header
test static 600
log summary
log off
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


## LOGVER 3 magnetic fields

Patch 0022 keeps existing machine-log columns stable and appends magnetic
reliability data. `MAG` adds world dip, field state/trust/flags and norm/dip/heading
reference errors. `YAW` adds normal/reacquisition mode, pending/active state,
stable-field duration and magnetic heading rate. In 0022a the logged
`field_heading_rate_deg_s` is the filtered signed-rate magnitude used by
reacquisition rather than the instantaneous two-sample derivative; CLI status
prints both values. `tools/logs/parse_e0_log.py` reads these fields when present
and remains compatible with LOGVER 2 captures.
