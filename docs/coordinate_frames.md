# Coordinate frames and quaternion conventions

This document records the frame conventions used by the firmware. Keep it in
sync with `core/math.hpp`, AHRS code, mag heading/yaw code, prepared output
snapshots, and SlimeVR output code.

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

### Active sensor-to-device boundary

`TrackerFrameConfigPersisted` stores a `sensorToDevice` matrix. Runtime enables
it only as a finite proper rotation: orthonormal axes and determinant near +1.
Finite legacy non-rotations remain loadable but are ignored until sanitize/save
replaces them with disabled identity, preserving unrelated NVS calibration.
Gyro bias and accel matrix calibration remain in the native LSM6DSV frame; the
validated rotation is applied afterwards. QMC6309 data receives the same
rotation after `magToImu`, so gyro, accel and magnetometer agree in device frame
before AHRS/heading processing.

The firmware device convention is:

```text
+X = right
+Y = forward
+Z = top/outward
```

`setup calibration` determines this physical case frame from two stationary
gravity observations. During a full accel six-position calibration, the first
capture is top/+Z up and the second is the chosen forward/+Y edge up; both still
count toward the same six accel faces, so no extra positions are added. Use the
same physical +Y edge on every tracker; the USB-connector edge is the recommended
default when the case has no printed arrow. The full 3x3 accel fit can absorb a
small board rotation, so setup performs a polar separation: the proper rotation
is moved into `sensorToDevice`, while scale/non-orthogonality stays in the native
sensor-frame accel matrix. This keeps gyro, accel and mag in one device frame.
If accel calibration already exists, resume mode captures only those two short
observations. `setup frame calibrate` repeats just this stage.

When `sensorToDeviceValid=false`, the transform is identity and behavior remains
compatible with the previous baseline, but setup readiness reports the frame as
missing. Future linear acceleration must be created after this same boundary.
SlimeVR body-part mounting, recenter and skeleton offsets remain server-owned.

## Magnetometer frame

The QMC6309 has its own raw magnetometer frame. Runtime processing applies:

```text
raw mag frame
  -> hard iron subtraction
  -> full 3x3 soft-iron ellipsoid correction matrix
  -> magToImu matrix
  -> native IMU frame
  -> sensorToDevice rotation
  -> device frame
```

The soft-iron matrix is produced by the mag ellipsoid calibration stage. It can
contain off-diagonal cross-axis terms; it is not limited to diagonal scaling.

`magToImu` is a sensor-to-sensor alignment matrix. It is not a body mounting
calibration and should not encode SlimeVR body offsets.

## Device/body/mounting frame

Only `sensorToDevice` is an active firmware frame field. Historical bytes that
were intended for firmware mounting offsets, output-convention selection and a
duplicate device identity remain in the binary blob solely for NVS layout
compatibility; sanitize always resets them to neutral values.

Current firmware policy:

- The firmware estimates where the physical tracker device points.
- SlimeVR/server-side logic owns body assignment, mounting calibration, recenter,
  body proportions, AutoBone and Stay Aligned.
- Firmware output uses one fixed native convention until a complete, tested
  protocol adapter is introduced.

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

Firmware currently publishes the AHRS device/sensor orientation through prepared
output snapshots. Local serial output and SlimeVR UDP consume that same snapshot
boundary instead of reading AHRS/FIFO objects directly.
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
