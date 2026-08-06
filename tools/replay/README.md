# Replay tools

Replay tools consume firmware machine logs (`LOGVER`, `LOGFMT`, `Q`, `FIFO`,
`MAG`, `MAGR`, `YAW`, `STATE`, `LOGSUM`, `LOGSTAT`, etc.). Human `status` / `health`
text is ignored on purpose. `MAGR` is emitted only in `log full` and carries
raw/calibrated/body magnetometer vectors for magnetometer replay fixtures.

## Score one log

```bash
python tools/replay/replay_machine_log.py tracker.log --pretty
```

Useful gate options:

```bash
python tools/replay/replay_machine_log.py tracker.log \
  --min-duration-s 600 \
  --max-fifo-fallback-rows 0 \
  --max-fifo-fault-rows 0 \
  --max-recovering-rows 0
```

For magnetometer sweep fixtures:

```bash
python tools/replay/replay_machine_log.py logs/mag_sweep_001.log \
  --require-magr \
  --min-magr-rows 500 \
  --pretty
```

## Compare two runs

```bash
python tools/replay/replay_machine_log.py before.log --output before.json
python tools/replay/replay_machine_log.py after.log --output after.json
python tools/replay/compare_replay_metrics.py before.json after.json --pretty
```

The comparator tracks only stable, high-signal metrics: sample counts, FIFO
fault rows, confidence/trust minima, diagnostic yaw drift, mag/yaw ratios,
MAGR vector coverage when present, recovery events, and runtime bias updates.

## Strict LOGVER3 capture integrity

`replay_machine_log.py` intentionally remains compatible with historical
LOGVER2. New fixture candidates must additionally pass:

```bash
python3 tools/replay/strict_logver3_gate.py \
  --log logver3_static_clean_001.log \
  --output logver3_static_clean_001.validation.json
```

Release fixtures use `--fixture ... --golden ...`; omitting the independent
golden JSON fails closed. The golden binds the exact fixture SHA-256 and may
apply reviewed `eq`/`min`/`max` rules to validation-report metrics. Do not use a
synthetic positive log as release evidence.

Create the real candidate with `tools/capture_telnet_log.py`, using battery
power and no USB cable. The capture tool verifies clean ProductionDiag identity,
magnetometer readiness, session ownership, full pipeline drain and console/drop
counters before writing its manifest.

## Magnetic reliability logs

LOGVER 3 appends field-reliability columns to `MAG` and reacquisition columns to
`YAW`. Older captures remain valid because existing column positions are unchanged
and the parser treats appended fields as optional.
