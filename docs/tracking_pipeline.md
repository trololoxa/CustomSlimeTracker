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

The old unconditional boot-time `sleep(2)` has been replaced by the Debug-only `TRACKER_ENABLE_BOOT_DELAY` / `TRACKER_BOOT_SERIAL_SETTLE_DELAY_MS` path. Production and Slim should boot without the serial settle delay.

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

### Current frame and output boundary

The sample path applies gyro/temperature and accel calibration in native sensor
frame, then applies the validated `sensorToDevice` proper rotation before
quality/AHRS processing. Magnetometer data receives the same rotation after
`magToImu`. With the default disabled transform, behavior remains legacy
identity-compatible.

The prepared output snapshot currently carries orientation only. SlimeVR
`RotationData` packet 17 is emitted, but acceleration packet 4 and
timestamp-coherent linear acceleration are not implemented in this baseline.

## Trust boundaries

- Gyro is the dynamic source.
- Accel is a gravity reference only while trustworthy.
- Mag is yaw-only correction only while trustworthy.
- Bad mag must degrade/fallback to 6DoF, not partially corrupt roll/pitch.
- Server/body/mounting/recenter logic is not firmware-owned.

## State and diagnostics

`TrackingStateController` is the single place that should summarize user-visible tracking state. CLI status and machine logs should not invent separate state models.

Machine-readable logs are the source for replay/metrics. Human CLI output is for inspection and should not become a replay input format. `MAG`/`YAW` frames describe magnetometer trust and yaw correction behavior; full-mode `MAGR` frames carry raw/calibrated/body magnetometer vectors for host-side magnetometer fitting and axis-mapping regression.

### AHRS recovery after multi-second timestamp gaps

Blocking diagnostics such as Wi-Fi scans can pause sensor processing long enough
for FIFO timestamps to jump by multiple seconds. With the production runtime
configuration, AHRS rejects that gap instead of integrating it as real rotation.
After the rejection it rebases `lastIntegratedTimestampUs` to the current sample
so the next normal FIFO sample resumes gyro prediction. This prevents a stale
pre-recovery timestamp baseline from freezing quaternion updates while FIFO and
SlimeVR packet counters continue to advance. `ahrs status` exposes this through
`large_dt_rebase_count`, `last_rebase_t_us`, and `post_fifo_recovery_samples` so
post-scan recovery can be verified without enabling high-rate logs.

`net scan`, `GET WIFISCAN`, and other blocking diagnostics are tracking
interruptions: motion made while the CPU is inside the blocking operation is not
recoverable because the gyro history was not processed in real time. Recovery is
expected to restore clean FIFO/AHRS operation for motion that happens after the
command returns; it is not expected to reconstruct motion that happened during
the scan. FIFO timestamp reconstruction also resets the sensor-hub/magnetometer
timestamp baseline so the first post-recovery mag sample is anchored to the new
IMU stream instead of inheriting a stale pre-recovery 60 Hz mag cadence.
