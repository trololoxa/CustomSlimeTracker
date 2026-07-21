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

### Optimized fusion cadence

Gyro prediction remains at the full accepted IMU sample rate. The small-angle
quaternion exponential uses a bounded polynomial for ordinary ~1 ms samples and
falls back to the exact trigonometric path for larger rotation vectors. Quaternion
normalization remains periodic. Gravity correction aggregates four accepted
samples and applies the same represented valid time at roughly 230-240 Hz; it does
not decimate gyro integration. Prepared motion snapshots are capped at 250 Hz,
well above the 100 Hz network output, and invalid samples still fail closed
immediately.

The host regression suite compares the fast and previous exact gyro paths over an
extreme two-hour 1000 dps trajectory, bounds single-step and acos approximation
error, and verifies static tilt convergence. These tests protect long-duration
orientation quality while removing redundant per-sample trigonometry.

The prepared output snapshot is produced from one accepted IMU sample and carries
the quaternion plus gravity-removed device-frame acceleration under the same
timestamp. A snapshot is invalid unless the AHRS integrated or initialized at that
exact sample timestamp; rejected gyro/timestamp samples therefore cannot relabel a
stale quaternion as fresh. Motion additionally requires completed accel and
sensor-to-device calibration; hard accel saturation invalidates acceleration only.
SlimeVR packet 17 is sent first, then packet 4 is sent from that exact snapshot only
when rotation transport succeeded. Packet 4 is never emitted alone or recomputed.

## Trust boundaries

- Gyro is the dynamic source.
- Accel is a gravity reference only while trustworthy.
- Mag is yaw-only correction only while trustworthy.
- Bad mag must degrade/fallback to 6DoF, not partially corrupt roll/pitch.
- Server/body/mounting/recenter logic is not firmware-owned.

## State and diagnostics

`TrackingStateController` is the single place that should summarize user-visible tracking state. CLI status and machine logs should not invent separate state models.

Machine-readable logs are the source for replay/metrics. Human CLI output is for inspection and should not become a replay input format. `MAG`/`YAW` frames describe magnetometer trust and yaw correction behavior; full-mode `MAGR` frames carry raw/calibrated/body magnetometer vectors for host-side magnetometer fitting and axis-mapping regression.

### AHRS recovery after unreconstructable timestamp gaps

The quality monitor labels a gap once it exceeds the normal sample cadence. That
label is diagnostic: losing one or several samples does not by itself invalidate
orientation. When the measured `dt` is still within the AHRS `maxDtS`, gyro is
integrated across the real interval and prepared output remains live. Recovery is
reserved for a gap larger than AHRS can safely integrate, an explicit blocking
operation, or a real FIFO reset/fault.

Blocking diagnostics such as Wi-Fi scans can pause sensor processing long enough
for FIFO timestamps to jump by multiple seconds. With the production runtime
configuration, AHRS rejects that gap instead of integrating it as real rotation.
After the rejection it rebases `lastIntegratedTimestampUs` to the current sample
so the next normal FIFO sample resumes gyro prediction. While recovery is active,
prepared network orientation is invalidated and accel correction is disabled.
Post-gap gyro is still integrated, so heading changes made after the stream
returns are retained. Recovery exits only after 256 contiguous samples pass
timestamp/FIFO checks, gyro remains below 3 dps and accel remains near 1 g. The
mean accel vector then rebuilds roll/pitch while preserving horizontal heading.
Movement restarts only this short window. `status` exposes recovery entry and
successful tilt-reacquisition counters; `quality stats` keeps routine gap counts.

`net scan`, `GET WIFISCAN`, and other blocking diagnostics are tracking
interruptions: motion made while the CPU is inside the blocking operation is not
reconstructable because the missing gyro history does not exist. The recovery
step does not invent that motion; it restores gravity-consistent tilt after the
tracker is briefly still and preserves the heading available before and after
the gap. FIFO timestamp reconstruction also resets the sensor-hub/magnetometer
timestamp baseline so the first post-recovery mag sample is anchored to the new
IMU stream instead of inheriting a stale pre-recovery 60 Hz mag cadence.
