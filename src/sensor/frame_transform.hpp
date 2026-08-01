#pragma once

#include <cmath>
#include <cstdint>

#include "core/math.hpp"

namespace tracker {

// Physical sensor axes and tracker/device axes are both right-handed.  Their
// relationship must therefore be a proper rotation (orthonormal, det ~= +1),
// not an arbitrary scale/shear matrix.  Calibration matrices remain in the
// native sensor frame and this transform is applied afterwards.
struct SensorToDeviceFrame {
    bool enabled = false;
    Mat3 rotation = Mat3::identity();

    Vec3 apply(const Vec3& sensorVector) const {
        return enabled ? rotation * sensorVector : sensorVector;
    }

    Vec3 inverseApply(const Vec3& deviceVector) const {
        return enabled ? rotation.transposed() * deviceVector : deviceVector;
    }
};

inline bool isProperRotationMatrix(const Mat3& m,
                                   float axisNormTolerance = 0.02f,
                                   float orthogonalityTolerance = 0.02f,
                                   float determinantTolerance = 0.05f) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (!std::isfinite(m.m[r][c])) return false;
        }
    }

    const Vec3 r0 = m.row(0);
    const Vec3 r1 = m.row(1);
    const Vec3 r2 = m.row(2);
    if (std::fabs(r0.norm() - 1.0f) > axisNormTolerance ||
        std::fabs(r1.norm() - 1.0f) > axisNormTolerance ||
        std::fabs(r2.norm() - 1.0f) > axisNormTolerance) {
        return false;
    }
    if (std::fabs(dot(r0, r1)) > orthogonalityTolerance ||
        std::fabs(dot(r0, r2)) > orthogonalityTolerance ||
        std::fabs(dot(r1, r2)) > orthogonalityTolerance) {
        return false;
    }
    return std::fabs(m.determinant() - 1.0f) <= determinantTolerance;
}

inline SensorToDeviceFrame makeSensorToDeviceFrame(bool valid, const Mat3& rotation) {
    SensorToDeviceFrame out;
    out.enabled = valid && isProperRotationMatrix(rotation);
    out.rotation = out.enabled ? rotation : Mat3::identity();
    return out;
}

// Runtime cache for the validated sensor-to-device frame. TrackerConfig::sanitize()
// already guarantees that a persisted valid frame is a proper rotation, but
// hot-path consumers also need to remain fail-closed if a runtime caller presents
// an unsanitized candidate. The config CRC is the explicit runtime revision:
// every authoritative config mutation must sanitize/updateCrc before apply.
class SensorToDeviceFrameCache {
public:
    const SensorToDeviceFrame& resolve(uint32_t configRevision,
                                       bool valid,
                                       const Mat3& rotation) {
        if (initialized_ && configRevision_ == configRevision) {
            return frame_;
        }
        frame_ = makeSensorToDeviceFrame(valid, rotation);
        configRevision_ = configRevision;
        initialized_ = true;
        ++refreshes_;
        return frame_;
    }

    void invalidate() {
        initialized_ = false;
        configRevision_ = 0u;
        frame_ = SensorToDeviceFrame{};
    }

    bool initialized() const { return initialized_; }
    uint32_t configRevision() const { return configRevision_; }
    uint32_t refreshes() const { return refreshes_; }
    const SensorToDeviceFrame& frame() const { return frame_; }

private:
    SensorToDeviceFrame frame_;
    uint32_t configRevision_ = 0u;
    uint32_t refreshes_ = 0u;
    bool initialized_ = false;
};

} // namespace tracker
