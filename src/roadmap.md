Delayed:
1. FIFO on IMU
2. Accel calibration
3. SPI speed
4. Accel/gyro/mag Mhz speed
5. Accel trust чуть построже

Phase 1 — IMU bring-up: Closed
  math
  LSM6DSV driver
  raw/scaled logs
  timestamped samples
  gyro startup calibration

Phase 2 — 6DoF: Closed
  gyro-only quaternion
  accel gravity correction
  accel gating
  stable 6DoF validation

Phase 3 — Robust IMU pipeline: Closed
  FIFO
  dropped sample detection
  saturation flags
  config save/load
  command protocol

Phase 4 — Calibration: Active?
  accel 6-position calibration
  gyro bias persistence
  gyro temp compensation
  mounting offset
  body/recenter offset

Phase 5 — Magnetometer: Currently active
  QMC6309 driver +
  raw mag logs +
  hard/soft iron calibration +
  mag -> imu axis alignment +-
  magTrust/gating
  slow yaw correction

Phase 6 — Output/tools:
  quaternion packet
  binary logs
  PC viewer/replay
  gain tuning
  long-run tests

Phase 7 — Advanced:
  MEKF/error-state Kalman
  latency prediction
  optional earth rotation compensation
  SlimeVR-compatible output

For earth compensation check:
    Add gyro_after_mean_dps: x=... y=... z=... to static report
    Don't do temp calibration