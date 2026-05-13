# Replay tools

Replay tools consume firmware machine logs (`LOGVER`, `LOGFMT`, `Q`, `FIFO`,
`MAG`, `YAW`, `STATE`, `LOGSUM`, `LOGSTAT`, etc.). Human `status` / `health`
text is ignored on purpose.

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

## Compare two runs

```bash
python tools/replay/replay_machine_log.py before.log --output before.json
python tools/replay/replay_machine_log.py after.log --output after.json
python tools/replay/compare_replay_metrics.py before.json after.json --pretty
```

The comparator tracks only stable, high-signal metrics: sample counts, FIFO
fault rows, confidence/trust minima, diagnostic yaw drift, mag/yaw ratios,
recovery events, and runtime bias updates.
