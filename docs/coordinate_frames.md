# Coordinate frames and quaternion conventions

This document records the frame conventions used by the firmware. Keep it in
sync with `core/math.hpp`, AHRS code, mag heading/yaw code, and future SlimeVR
output code.

## Quaternion convention

The project uses Hamilton quaternions with layout:

```text
[w, x, y, z]
```

Quaternion multiplication is Hamilton product.

The active vector rotation helper is:

```cpp
q.rotate(v) = q * [0, v] * q.conjugate()
```

For AHRS, the main orientation is:

```text
q_world_from_sensor
```

Meaning:

```text
world_vector = q_world_from_sensor.rotate(sensor_vector)
```

Gyro angular velocity is measured in sensor/body frame, in radians per second.
For propagation with body-frame gyro:

```text
q_next = q_world_from_sensor * dq_body
```

## World frame

The current 6DoF AHRS default world-up direction is:

```text
+Z = up
```

This is ENU-like for the gravity direction. Yaw is unobservable in 6DoF mode and
will drift unless magnetometer yaw correction is enabled and trusted.

Do not interpret firmware yaw as a stable global heading unless mag yaw
correction is active, gated open, and has a valid heading reference.

## Sensor frame

The LSM6DSV driver outputs calibrated accel/gyro in the IMU sensor frame.
Calibration is applied before AHRS update:

```text
raw LSM6DSV sample
  -> scale to physical units
  -> gyro bias / temp compensation
  -> accel bias/scale matrix
  -> AHRS update
```

The AHRS should receive:

```text
gyro_rad_s in sensor/body frame
accel_g in sensor/body frame
```

## Magnetometer frame

The QMC6309 has its own raw magnetometer frame. Runtime processing applies:

```text
raw mag frame
  -> hard iron subtraction
  -> soft iron matrix
  -> magToImu matrix
  -> IMU/body frame
```

`magToImu` is a sensor-to-sensor alignment matrix. It is not a body mounting
calibration and should not encode SlimeVR body offsets.

## Device/body/mounting frame

Persistent config contains placeholders for:

```text
sensorToDevice
mountingOffset
outputConvention
```

Current firmware policy:

- The firmware estimates where the sensor/device points.
- SlimeVR/server-side logic owns body assignment, mounting calibration, recenter,
  body proportions, AutoBone and Stay Aligned.
- Do not bake body/recenter/mounting offsets into local AHRS output unless a
  future output backend explicitly requires a documented firmware-side output
  convention.

## Mag heading and yaw correction

Mag heading code estimates a magnetic field direction in the current AHRS world
XY plane:

```text
mag body vector
  -> q_world_from_body.rotate(...)
  -> world horizontal projection
  -> magnetic field yaw / north-like yaw
```

Yaw correction compares:

```text
magneticNorthWorldYawRad - referenceWorldYawRad
```

Then applies a slow correction around world-up when gates allow it.

Important gates:

```text
mag trusted
heading valid
mag not stale
horizontal norm good
innovation not too large
gyro not moving too much
accel trusted when required
cooldown inactive
```

The yaw controller is intentionally slow and conservative. It must not fight fast
motion or disturbed magnetic readings.

## SlimeVR output boundary

SlimeVR Server expects tracker orientation as a quaternion. It does not replace
local IMU calibration, gyro bias calibration, accel calibration, mag calibration,
or sensor fusion.

Firmware should eventually output a documented device orientation quaternion.
The server should remain responsible for:

```text
tracker assignment to body part
mounting calibration
body dimensions
recenter/full reset
Stay Aligned/body yaw corrections
```

Do not implement SlimeVR body semantics inside AHRS or mag-yaw code.

## Rules for future changes

When adding or changing frame-related code:

1. Document whether a quaternion maps `A_from_B` or `B_from_A`.
2. State whether a vector is in raw sensor, IMU/body, device, or world frame.
3. Keep gyro in rad/s before AHRS integration.
4. Keep accel in g before AHRS gravity correction.
5. Keep body/server offsets out of sensor calibration code.
6. Add or update native tests for quaternion/frame invariants when possible.
