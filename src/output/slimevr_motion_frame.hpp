#pragma once

#include <cstdint>

#include "core/math.hpp"

namespace tracker {

// SlimeVR motion output contract used by packet 17 + packet 4.
//
// Local/device axes are right-handed and intentionally match the corrected
// tracker acceleration convention accepted by SlimeVR protocol 22:
//   +X = right
//   +Y = forward
//   +Z = top/outward
//
// The prepared quaternion is q_world_from_device. Packet 4 acceleration is
// gravity-removed linear acceleration expressed in that same device frame.
// SlimeVR Server still applies its normal IMU-to-server world-axis offset to
// rotation, but protocol 22 tells it not to apply the legacy extra -90 degree
// local Z correction to acceleration.
namespace slimevr_motion_frame {

constexpr uint8_t FIXED_ACCEL_PROTOCOL_MIN_VERSION = 22;
constexpr uint8_t PROTOCOL_VERSION = FIXED_ACCEL_PROTOCOL_MIN_VERSION;
constexpr float STANDARD_GRAVITY_MPS2 = 9.80665f;

constexpr const char* CONTRACT_NAME = "device_x_right_y_forward_z_up";
constexpr const char* ROTATION_CONVENTION_NAME = "world_from_device";
constexpr const char* ACCELERATION_FRAME_NAME = "device";
constexpr const char* ACCELERATION_UNITS_NAME = "mps2";

constexpr bool protocolUsesCorrectedAcceleration(uint8_t protocolVersion) {
    return protocolVersion >= FIXED_ACCEL_PROTOCOL_MIN_VERSION;
}

// Packet 17 stores x/y/z/w but represents the same Hamilton
// q_world_from_device used by the AHRS. Normalization belongs at this protocol
// boundary and does not alter the local basis.
inline Quat rotationWireFromWorldDevice(const Quat& qWorldFromDevice) {
    return qWorldFromDevice.normalized().withPositiveW();
}

// Packet 4 must preserve the local device axes exactly. The only conversion is
// from internal g to SI m/s^2.
inline Vec3 accelerationWireMps2FromDeviceG(const Vec3& linearAccelerationDeviceG) {
    return linearAccelerationDeviceG * STANDARD_GRAVITY_MPS2;
}

} // namespace slimevr_motion_frame

// Existing protocol users include this public constant from the packet writer.
constexpr uint8_t SLIMEVR_PROTOCOL_VERSION = slimevr_motion_frame::PROTOCOL_VERSION;
static_assert(SLIMEVR_PROTOCOL_VERSION >= slimevr_motion_frame::FIXED_ACCEL_PROTOCOL_MIN_VERSION,
              "device-frame packet 4 requires SlimeVR fixed-acceleration protocol");

} // namespace tracker
