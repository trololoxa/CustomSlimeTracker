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
Q,FIFO,CAL,BIAS,BIASUPD,MAG,YAW,STATE,LOGSUM,LOGSTAT,TEMPBIN
```

Human CLI/status text is intentionally ignored. Replay should depend on stable machine-readable frames only.

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
`Q`, `FIFO`, `CAL`, `BIAS`, `MAG`, `YAW`, `STATE`, `LOGSUM`, and `LOGSTAT`
frames when those systems are active.

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

