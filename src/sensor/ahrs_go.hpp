#pragma once

#include <cstdint>
#include <cmath>

#include "core/math.hpp"

namespace tracker {

// ============================================================
// Gyro-only quaternion integrator
// ============================================================
// Purpose:
//   - First AHRS stage.
//   - Integrates calibrated gyro into orientation quaternion.
//   - No accel correction yet.
//   - No magnetometer correction yet.
//
// Convention:
//   q_world_from_sensor rotates vectors from sensor frame to world frame.
//   Gyro input is angular velocity in sensor/body frame, rad/s.
//   Propagation:
//      q_next = q * dq_body
//
// Important limitation:
//   Gyro-only orientation will drift over time.
//   Roll/pitch/yaw are all integrated from gyro only.
//   Next step will add accel gravity correction for roll/pitch.
// ============================================================

struct GyroOnlyAhrsConfig {
    // Ignore impossible or startup dt values.
    float minDtS = 0.0001f;   // 100 us
    float maxDtS = 0.0200f;   // 20 ms

    // If dt is larger than maxDtS, we can either skip update or clamp dt.
    // For diagnostics, clamp=true is useful because quaternion keeps moving.
    // For final FIFO-based pipeline, large dt should almost never happen.
    bool clampLargeDt = true;

    // Normalize every N predict calls.
    // Quaternion integration already returns normalized dq, but accumulated
    // floating-point error still exists.
    uint32_t normalizeEvery = 16;

    // Optional deadband for very tiny gyro noise.
    // Keep 0.0f while evaluating sensor/calibration quality.
    float gyroDeadbandRadS = 0.0f;
};

struct GyroOnlyAhrsStats {
    uint32_t predictCount = 0;
    uint32_t skippedBadDt = 0;
    uint32_t clampedLargeDt = 0;
    uint32_t normalizedCount = 0;

    uint64_t lastTimestampUs = 0;
    float lastDtS = 0.0f;
    float lastUsedDtS = 0.0f;

    Vec3 lastGyroRadS = Vec3::zero();
    Vec3 lastRotationVectorRad = Vec3::zero();
    float lastRotationAngleRad = 0.0f;
};

class GyroOnlyAhrs {
public:
    explicit GyroOnlyAhrs(const GyroOnlyAhrsConfig& config = GyroOnlyAhrsConfig{})
        : cfg_(config) {}

    void reset(const Quat& initialQ = Quat::identity(), uint64_t timestampUs = 0) {
        q_ = initialQ.normalized().withPositiveW();
        stats_ = GyroOnlyAhrsStats{};
        stats_.lastTimestampUs = timestampUs;
        initialized_ = timestampUs != 0;
    }

    bool initialized() const {
        return initialized_;
    }

    const Quat& quaternion() const {
        return q_;
    }

    Quat quaternionPositiveW() const {
        return q_.withPositiveW();
    }

    Vec3 eulerRad() const {
        return q_.toEulerXYZ();
    }

    Vec3 eulerDeg() const {
        return eulerRad() * MATH_RAD_TO_DEG;
    }

    const GyroOnlyAhrsStats& stats() const {
        return stats_;
    }

    const GyroOnlyAhrsConfig& config() const {
        return cfg_;
    }

    void setQuaternion(const Quat& q) {
        q_ = q.normalized().withPositiveW();
    }

    // Main update function.
    // gyroRadS must already be calibrated: raw gyro - startup bias.
    // Returns true if quaternion was updated.
    bool predict(const Vec3& gyroRadS, uint64_t timestampUs) {
        if (!gyroRadS.isFinite() || timestampUs == 0) {
            stats_.skippedBadDt++;
            return false;
        }

        if (!initialized_) {
            stats_.lastTimestampUs = timestampUs;
            initialized_ = true;
            stats_.lastGyroRadS = gyroRadS;
            return false;
        }

        const uint64_t lastUs = stats_.lastTimestampUs;
        stats_.lastTimestampUs = timestampUs;

        if (timestampUs <= lastUs) {
            stats_.skippedBadDt++;
            return false;
        }

        float dtS = static_cast<float>(timestampUs - lastUs) * 1.0e-6f;
        stats_.lastDtS = dtS;

        if (!std::isfinite(dtS) || dtS < cfg_.minDtS) {
            stats_.skippedBadDt++;
            return false;
        }

        if (dtS > cfg_.maxDtS) {
            if (!cfg_.clampLargeDt) {
                stats_.skippedBadDt++;
                return false;
            }
            dtS = cfg_.maxDtS;
            stats_.clampedLargeDt++;
        }

        Vec3 gyroUsed = gyroRadS;
        if (cfg_.gyroDeadbandRadS > 0.0f) {
            if (std::fabs(gyroUsed.x) < cfg_.gyroDeadbandRadS) gyroUsed.x = 0.0f;
            if (std::fabs(gyroUsed.y) < cfg_.gyroDeadbandRadS) gyroUsed.y = 0.0f;
            if (std::fabs(gyroUsed.z) < cfg_.gyroDeadbandRadS) gyroUsed.z = 0.0f;
        }

        const Vec3 rotationVectorRad = gyroUsed * dtS;
        const float angleRad = rotationVectorRad.norm();

        q_ = integrateBodyRate(q_, gyroUsed, dtS);

        stats_.predictCount++;
        stats_.lastUsedDtS = dtS;
        stats_.lastGyroRadS = gyroUsed;
        stats_.lastRotationVectorRad = rotationVectorRad;
        stats_.lastRotationAngleRad = angleRad;

        if (cfg_.normalizeEvery > 0 && (stats_.predictCount % cfg_.normalizeEvery) == 0) {
            q_.normalizeInPlace();
            q_ = q_.withPositiveW();
            stats_.normalizedCount++;
        }

        return true;
    }

private:
    GyroOnlyAhrsConfig cfg_;
    GyroOnlyAhrsStats stats_;
    Quat q_ = Quat::identity();
    bool initialized_ = false;
};

// ============================================================
// Debug/visual helper
// ============================================================

struct GyroOnlyAhrsDebugSnapshot {
    Quat q = Quat::identity();
    Vec3 eulerDeg = Vec3::zero();
    float dtMs = 0.0f;
    float gyroNormDps = 0.0f;
    float lastAngleDeg = 0.0f;
    uint32_t predictCount = 0;
    uint32_t skippedBadDt = 0;
    uint32_t clampedLargeDt = 0;
};

inline GyroOnlyAhrsDebugSnapshot makeGyroOnlyDebugSnapshot(const GyroOnlyAhrs& ahrs) {
    const GyroOnlyAhrsStats& st = ahrs.stats();

    GyroOnlyAhrsDebugSnapshot s;
    s.q = ahrs.quaternionPositiveW();
    s.eulerDeg = ahrs.eulerDeg();
    s.dtMs = st.lastUsedDtS * 1000.0f;
    s.gyroNormDps = st.lastGyroRadS.norm() * MATH_RAD_TO_DEG;
    s.lastAngleDeg = st.lastRotationAngleRad * MATH_RAD_TO_DEG;
    s.predictCount = st.predictCount;
    s.skippedBadDt = st.skippedBadDt;
    s.clampedLargeDt = st.clampedLargeDt;
    return s;
}

} // namespace tracker
