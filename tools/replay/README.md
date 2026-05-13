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
