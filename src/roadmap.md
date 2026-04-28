Delayed:
1. FIFO on IMU
2. Accel calibration
3. SPI speed
4. Accel/gyro/mag Mhz speed
5. Accel trust чуть построже

Phase 1 — IMU bring-up:
  math
  LSM6DSV driver
  raw/scaled logs
  timestamped samples
  gyro startup calibration

Phase 2 — 6DoF:
  gyro-only quaternion
  accel gravity correction
  accel gating
  stable 6DoF validation

Phase 3 — Robust IMU pipeline:
  FIFO
  dropped sample detection
  saturation flags
  config save/load
  command protocol

Phase 4 — Calibration:
  accel 6-position calibration
  gyro bias persistence
  gyro temp compensation
  mounting offset
  body/recenter offset

Phase 5 — Magnetometer:
  QMC6309 driver
  raw mag logs
  hard/soft iron calibration
  mag -> imu axis alignment
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

g_imuCal.accelBiasG = Vec3(0.00214949f, 0.00605807f, 0.00125885f);
g_imuCal.accelScale = Mat3::diagonal(1.00130630f, 1.00026011f, 1.00308013f);
g_imuCal.accelCalValid = true;