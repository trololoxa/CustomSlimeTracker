#pragma once

#include <cstdint>

namespace tracker {

// Canonical persisted/runtime policy for SlimeVR motion transport. Keep this
// pure and transport-agnostic so config, runtime, app wiring and CLI share one
// source of truth without depending on each other's implementation layers.
enum class SlimeVRMotionPacketPolicy : uint8_t {
    QuaternionOnly = 0,
    BundleRotation17Acceleration4 = 1,
    RotationAcceleration23 = 2,
};

inline bool slimevrMotionPacketPolicyValid(SlimeVRMotionPacketPolicy policy) {
    return static_cast<uint8_t>(policy) <=
           static_cast<uint8_t>(SlimeVRMotionPacketPolicy::RotationAcceleration23);
}

inline const char* slimevrMotionPacketPolicyName(SlimeVRMotionPacketPolicy policy) {
    switch (policy) {
        case SlimeVRMotionPacketPolicy::QuaternionOnly: return "quaternion_only";
        case SlimeVRMotionPacketPolicy::BundleRotation17Acceleration4:
            return "bundle_100_rotation_17_accel_4";
        case SlimeVRMotionPacketPolicy::RotationAcceleration23:
            return "rotation_acceleration_23";
    }
    return "quaternion_only";
}

} // namespace tracker
