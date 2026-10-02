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
A persisted output policy chooses how that snapshot crosses the protocol
boundary. The default `quaternion` policy sends packet 17 only. `bundle` sends
float32 packet 17 plus float32 packet 4 in one packet-100 datagram after bundle
negotiation, or packet 17 plus a coherent 50 Hz packet-4 fallback on older
servers. `packet23` sends the existing Q15/Q7 RotationAndAcceleration packet 23
from the same snapshot. If acceleration is hard-invalid, acceleration modes
still send packet 17 alone and count the skip reason. The protocol boundary preserves the device local
axes (`+X right, +Y forward, +Z top/outward`) for both values and advertises
protocol 22, disabling the server's legacy acceleration-only axis correction.

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
so the next normal FIFO sample resumes gyro prediction. A bounded FIFO full/overrun uses non-blocking soft recovery: the FIFO and AHRS
timebase are reset, one faulted snapshot is discarded, then gyro+adaptive accel
updates and network output resume on the next integrated sample. The controller
stays `DEGRADED_TIMING` for 32 clean samples for diagnostics, but movement does
not prolong it. Timestamp corruption, explicit resets and blocking operations use
strict recovery: prepared orientation is invalidated, accel correction is disabled
and post-gap gyro is retained until 256 stationary samples rebuild roll/pitch.
`status` exposes strict and soft entry/completion counters separately.

`net scan`, `GET WIFISCAN`, and other blocking diagnostics are tracking
interruptions: motion made while the CPU is inside the blocking operation is not
reconstructable because the missing gyro history does not exist. The recovery
step does not invent that motion; it restores gravity-consistent tilt after the
tracker is briefly still and preserves the heading available before and after
the gap. FIFO timestamp reconstruction also resets the sensor-hub/magnetometer
timestamp baseline so the first post-recovery mag sample is anchored to the new
IMU stream instead of inheriting a stale pre-recovery 60 Hz mag cadence.

## Magnetic reliability and continuous alignment candidate

After calibrated/body-frame magnetic processing, heading now includes world dip
and passes through `MagFieldReliabilityMonitor`. The monitor applies temporal
norm/dip/heading checks and hysteretic state transitions. `MagYawCorrectionController`
receives only the resulting trusted-field state and uses a separate low-rate
reacquisition mode for innovations above the ordinary gate. In parallel,
`MagAxisAlignmentCollector` observes timestamp-coherent gyro/mag motion, performs
bounded coverage checks and may stage a calibration candidate. This branch is
calibration-only: it does not touch AHRS/FIFO configuration, active calibration,
NVS or the output snapshot unless the user later performs the normal candidate
promotion transaction.
