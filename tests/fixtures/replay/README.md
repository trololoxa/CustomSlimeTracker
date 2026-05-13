# Replay fixtures

This directory stores machine-readable E0 serial logs used by the host replay
quality gate. These files are intentionally captured from real firmware instead
of being synthetic unit-test data.

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

This fixture is meant to catch parser/tooling regressions and major firmware log
format regressions. It is not a golden promise that yaw correction is active.
A future fixture with mag reference + yaw apply enabled should be added
separately.
