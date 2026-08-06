# Replay fixtures

This directory stores machine-readable E0 logs used by the compatibility replay
gate. Real captures are required; synthetic text belongs only in parser unit
tests.

## baseline_replay_001.log

Profile: `baseline_no_yaw_apply`

Captured from the command sequence:

```text
log reset
log full
log rate 20
log header
test static 600
log summary
log off
```

Important characteristics:

- about 10 minutes of E0 frames;
- hardware FIFO timestamps only, no fallback timestamp rows;
- no FIFO overrun/full/unknown rows;
- Q/FIFO/CAL/BIAS/MAG/YAW coverage is present;
- magnetometer is enabled and trusted, but yaw apply/reference is intentionally
  not enabled;
- some motion is present, so accel trust may temporarily drop during the log;
- expected diagnostic yaw drift is below 2 deg/min for this fixture.

This LOGVER2 fixture catches compatibility parser/tooling regressions. It is not
a strict LOGVER3 golden and cannot satisfy release preflight. The future
`logver3_static_golden.log` must come from the unattended ProductionDiag TCP
capture with clean identity, battery power and USB physically disconnected;
its JSON thresholds must be reviewed independently.


## Future magnetometer fixtures

Magnetometer calibration/axis fixtures should be captured with `log full` after
firmware support for `MAGR` rows. Use `replay_machine_log.py --require-magr`
for those logs so raw/calibrated/body magnetometer vector coverage is checked.
