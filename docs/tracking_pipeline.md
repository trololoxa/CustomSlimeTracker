# Tracking pipeline

This document describes the runtime data flow. It is not a tuning guide; it is the ownership map for tracking-related code.

## Boot/setup flow

```text
main.cpp
  -> trackerAppContextSetup()
  -> TrackerApp::setup()
  -> config load/defaults/sanitize/apply
  -> LSM/FIFO init
  -> calibration IO setup
  -> mag runtime setup
  -> CLI setup
```

Boot-time `sleep(2)` is currently intentional developer convenience.

## Runtime loop

```text
CLI poll
FIFO/runtime process
  -> drain bounded FIFO events
  -> reconstruct timestamps
  -> convert raw sample to calibrated sample
  -> update quality monitor
  -> update AHRS 6DoF
  -> update runtime gyro-bias estimator
  -> update tracking state/recovery
  -> process mag samples/yaw correction when trusted
  -> emit stream/log/output if due
CLI poll
heartbeat/output maintenance
```

## Trust boundaries

- Gyro is the dynamic source.
- Accel is a gravity reference only while trustworthy.
- Mag is yaw-only correction only while trustworthy.
- Bad mag must degrade/fallback to 6DoF, not partially corrupt roll/pitch.
- Server/body/mounting/recenter logic is not firmware-owned.

## State and diagnostics

`TrackingStateController` is the single place that should summarize user-visible tracking state. CLI status and machine logs should not invent separate state models.

Machine-readable logs are the source for replay/metrics. Human CLI output is for inspection and should not become a replay input format.
