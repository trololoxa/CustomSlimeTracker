#pragma once

#include <cstdint>
#include <cmath>

#include "core/math.hpp"

namespace tracker {

// ============================================================
// Adaptive 6DoF AHRS: gyro + accel gravity correction
// ============================================================
// Purpose:
//   - Integrate calibrated gyro at high rate.
//   - Use accelerometer only as a gravity/up direction correction.
//   - Reject/downweight accel when |a| is far from 1g or innovation is too large.
//
// Convention:
//   q_world_from_sensor rotates vectors from sensor frame to world frame.
//   Gyro input is angular velocity in sensor/body frame, rad/s.
//   Accel input is measured specific force in sensor frame, g.
//
// In static conditions, normalized accelerometer is treated as the measured
// world-up direction expressed in sensor frame. Therefore the accel correction
// tries to make:
//      q_world_from_sensor.rotate(accel_sensor_unit) == worldUp
//
// Yaw is not observable from gyro+accel and will drift. That is expected.
// ============================================================

struct Ahrs6DofConfig {
    // World vertical direction. ENU-style default: +Z is up.
    Vec3 worldUp = Vec3(0.0f, 0.0f, 1.0f);

    // Timestamp guards.
    float minDtS = 0.0001f;   // 100 us
    float maxDtS = 0.0200f;   // 20 ms
    bool clampLargeDt = true;

    // Gyro noise deadband. Keep 0 while validating sensor data.
    float gyroDeadbandRadS = 0.0f;

    // Accel correction proportional gain, in 1/s.
    // Effective correction angle per update is roughly:
    //      correction_rad = accelKp * accelTrust * error_rad * dt
    // Good starting range: 1.0..8.0
    float accelKp = 3.0f;

    // Per-sample correction limiter. Prevents violent roll/pitch snaps.
    float maxAccelCorrectionRadPerUpdate = 2.0f * MATH_DEG_TO_RAD;

    // Accel norm gate. At rest |a| should be close to 1g.
    // <= good error: full trust
    // >= bad error: zero trust
    float accelNormGoodErrorG = 0.06f;
    float accelNormBadErrorG = 0.35f;

    // Accel innovation gate. This is the current angle between the gravity/up
    // direction estimated from accel and the configured worldUp direction.
    float accelInnovationGoodRad = 8.0f * MATH_DEG_TO_RAD;
    float accelInnovationBadRad = 45.0f * MATH_DEG_TO_RAD;

    // Normalize quaternion every N updates.
    uint32_t normalizeEvery = 16;
};

struct Ahrs6DofAccelGate {
    bool accepted = false;
    float trust = 0.0f;
    float normG = 0.0f;
    float normErrorG = 0.0f;
    float innovationRad = 0.0f;
    float normTrust = 0.0f;
    float innovationTrust = 0.0f;
};

struct Ahrs6DofStats {
    uint32_t updateCount = 0;
    uint32_t gyroPredictCount = 0;
    uint32_t accelUpdateCount = 0;
    uint32_t accelRejectedCount = 0;
    uint32_t skippedBadDt = 0;
    uint32_t clampedLargeDt = 0;
    uint32_t normalizedCount = 0;

    // Timestamp semantics:
    //   lastSeenTimestampUs       = latest timestamp observed by update(), even if rejected.
    //   lastIntegratedTimestampUs = latest timestamp that actually affected gyro integration.
    //
    // Keep lastTimestampUs as a backwards-compatible mirror of
    // lastIntegratedTimestampUs for existing diagnostics/users of stats().
    uint64_t lastSeenTimestampUs = 0;
    uint64_t lastIntegratedTimestampUs = 0;
    uint64_t lastTimestampUs = 0;
    float lastDtS = 0.0f;
    float lastUsedDtS = 0.0f;

    Vec3 lastGyroRadS = Vec3::zero();
    Vec3 lastAccelG = Vec3::zero();
    Vec3 lastAccelUnitSensor = Vec3::zero();
    Vec3 lastAccelUnitWorld = Vec3::zero();
    Vec3 lastAccelErrorWorld = Vec3::zero();
    Vec3 lastAccelCorrectionWorldRad = Vec3::zero();

    float lastGyroAngleRad = 0.0f;
    float lastAccelCorrectionAngleRad = 0.0f;

    Ahrs6DofAccelGate lastAccelGate;
};

class Ahrs6Dof {
public:
    explicit Ahrs6Dof(const Ahrs6DofConfig& config = Ahrs6DofConfig{})
        : cfg_(config) {
        cfg_.worldUp = cfg_.worldUp.normalized();
        if (cfg_.worldUp.normSq() < MATH_EPSILON) {
            cfg_.worldUp = Vec3(0.0f, 0.0f, 1.0f);
        }
    }

    void reset(const Quat& initialQ = Quat::identity(), uint64_t timestampUs = 0) {
        q_ = initialQ.normalized().withPositiveW();
        stats_ = Ahrs6DofStats{};
        stats_.lastSeenTimestampUs = timestampUs;
        stats_.lastIntegratedTimestampUs = timestampUs;
        stats_.lastTimestampUs = timestampUs;
        initialized_ = timestampUs != 0;
    }

    bool resetFromAccel(const Vec3& accelG, uint64_t timestampUs = 0) {
        const Vec3 a = accelG.normalized();
        if (!a.isFinite() || a.normSq() < MATH_EPSILON) {
            reset(Quat::identity(), timestampUs);
            return false;
        }

        // q rotates measured up direction from sensor frame into worldUp.
        q_ = Quat::fromTwoUnitVectors(a, cfg_.worldUp).withPositiveW();
        stats_ = Ahrs6DofStats{};
        stats_.lastSeenTimestampUs = timestampUs;
        stats_.lastIntegratedTimestampUs = timestampUs;
        stats_.lastTimestampUs = timestampUs;
        initialized_ = timestampUs != 0;
        return true;
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

    const Ahrs6DofStats& stats() const {
        return stats_;
    }

    const Ahrs6DofConfig& config() const {
        return cfg_;
    }

    void setQuaternion(const Quat& q) {
        q_ = q.normalized().withPositiveW();
    }

    // Explicitly rebase AHRS integration time after a known recovery event
    // (FIFO reset, timestamp reconstruction reset, sensor reset). This does
    // not rotate the quaternion and must not be used for ordinary rejected
    // samples; it is for explicit RECOVERING paths where the missing interval
    // cannot be reconstructed safely.
    void rebaseTimestamp(uint64_t timestampUs) {
        stats_.lastSeenTimestampUs = timestampUs;
        if (!initialized_) {
            return;
        }

        stats_.lastIntegratedTimestampUs = timestampUs;
        stats_.lastTimestampUs = timestampUs;
        stats_.lastDtS = 0.0f;
        stats_.lastUsedDtS = 0.0f;
    }

    Ahrs6DofAccelGate evaluateAccelGate(const Vec3& accelG) const {
        Ahrs6DofAccelGate gate;

        if (!accelG.isFinite()) {
            return gate;
        }

        gate.normG = accelG.norm();
        gate.normErrorG = std::fabs(gate.normG - 1.0f);

        if (gate.normG < MATH_EPSILON) {
            return gate;
        }

        gate.normTrust = rampDown(gate.normErrorG,
                                  cfg_.accelNormGoodErrorG,
                                  cfg_.accelNormBadErrorG);

        const Vec3 accelUnitSensor = accelG / gate.normG;
        const Vec3 accelUnitWorld = q_.rotate(accelUnitSensor).normalized();

        if (!accelUnitWorld.isFinite() || accelUnitWorld.normSq() < MATH_EPSILON) {
            return gate;
        }

        gate.innovationRad = angleBetweenUnitVectors(accelUnitWorld, cfg_.worldUp);
        gate.innovationTrust = rampDown(gate.innovationRad,
                                        cfg_.accelInnovationGoodRad,
                                        cfg_.accelInnovationBadRad);

        gate.trust = gate.normTrust * gate.innovationTrust;
        gate.accepted = gate.trust > 0.0f;
        return gate;
    }

    // Main update function.
    // gyroRadS must already be calibrated: raw gyro - startup bias.
    // accelG may be raw scaled accel or calibrated accel if accel calibration exists.
    // Returns true if gyro prediction was performed.
    bool update(const Vec3& gyroRadS, const Vec3& accelG, uint64_t timestampUs) {
        if (!gyroRadS.isFinite() || !accelG.isFinite() || timestampUs == 0) {
            stats_.skippedBadDt++;
            return false;
        }

        if (!initialized_) {
            resetFromAccel(accelG, timestampUs);
            stats_.lastGyroRadS = gyroRadS;
            stats_.lastAccelG = accelG;
            return false;
        }

        stats_.lastSeenTimestampUs = timestampUs;
        const uint64_t lastUs = stats_.lastIntegratedTimestampUs;

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

        // 1. High-rate gyro prediction.
        const Vec3 gyroRotationVector = gyroUsed * dtS;
        q_ = integrateBodyRate(q_, gyroUsed, dtS);

        stats_.updateCount++;
        stats_.gyroPredictCount++;
        stats_.lastIntegratedTimestampUs = timestampUs;
        stats_.lastTimestampUs = timestampUs;
        stats_.lastUsedDtS = dtS;
        stats_.lastGyroRadS = gyroUsed;
        stats_.lastAccelG = accelG;
        stats_.lastGyroAngleRad = gyroRotationVector.norm();

        // 2. Accel gravity correction.
        applyAccelCorrection(accelG, dtS);

        if (cfg_.normalizeEvery > 0 && (stats_.updateCount % cfg_.normalizeEvery) == 0) {
            q_.normalizeInPlace();
            q_ = q_.withPositiveW();
            stats_.normalizedCount++;
        }

        return true;
    }

private:
    static float rampDown(float x, float good, float bad) {
        if (x <= good) return 1.0f;
        if (x >= bad) return 0.0f;
        if (bad <= good) return 0.0f;
        return 1.0f - ((x - good) / (bad - good));
    }

    void applyAccelCorrection(const Vec3& accelG, float dtS) {
        Ahrs6DofAccelGate gate = evaluateAccelGate(accelG);
        stats_.lastAccelGate = gate;
        stats_.lastAccelCorrectionWorldRad = Vec3::zero();
        stats_.lastAccelCorrectionAngleRad = 0.0f;

        if (!gate.accepted) {
            stats_.accelRejectedCount++;
            return;
        }

        const Vec3 accelUnitSensor = accelG / gate.normG;
        const Vec3 accelUnitWorld = q_.rotate(accelUnitSensor).normalized();

        // errorWorld rotates accelUnitWorld toward worldUp.
        // For small angles, correction ~= cross(current, target).
        const Vec3 errorWorld = cross(accelUnitWorld, cfg_.worldUp);

        Vec3 correctionWorldRad = errorWorld * (cfg_.accelKp * gate.trust * dtS);

        const float corrNorm = correctionWorldRad.norm();
        if (corrNorm > cfg_.maxAccelCorrectionRadPerUpdate && corrNorm > MATH_EPSILON) {
            correctionWorldRad *= cfg_.maxAccelCorrectionRadPerUpdate / corrNorm;
        }

        q_ = applyWorldCorrection(q_, correctionWorldRad).withPositiveW();

        stats_.accelUpdateCount++;
        stats_.lastAccelUnitSensor = accelUnitSensor;
        stats_.lastAccelUnitWorld = accelUnitWorld;
        stats_.lastAccelErrorWorld = errorWorld;
        stats_.lastAccelCorrectionWorldRad = correctionWorldRad;
        stats_.lastAccelCorrectionAngleRad = correctionWorldRad.norm();
    }

    Ahrs6DofConfig cfg_;
    Ahrs6DofStats stats_;
    Quat q_ = Quat::identity();
    bool initialized_ = false;
};

// ============================================================
// Debug/visual helper
// ============================================================

struct Ahrs6DofDebugSnapshot {
    Quat q = Quat::identity();
    Vec3 eulerDeg = Vec3::zero();

    float dtMs = 0.0f;
    float gyroNormDps = 0.0f;
    float gyroAngleDeg = 0.0f;

    float accelNormG = 0.0f;
    float accelTrust = 0.0f;
    float accelInnovationDeg = 0.0f;
    float accelCorrectionDeg = 0.0f;

    uint32_t updateCount = 0;
    uint32_t accelUpdateCount = 0;
    uint32_t accelRejectedCount = 0;
    uint32_t skippedBadDt = 0;
    uint32_t clampedLargeDt = 0;
};

inline Ahrs6DofDebugSnapshot makeAhrs6DofDebugSnapshot(const Ahrs6Dof& ahrs) {
    const Ahrs6DofStats& st = ahrs.stats();

    Ahrs6DofDebugSnapshot s;
    s.q = ahrs.quaternionPositiveW();
    s.eulerDeg = ahrs.eulerDeg();

    s.dtMs = st.lastUsedDtS * 1000.0f;
    s.gyroNormDps = st.lastGyroRadS.norm() * MATH_RAD_TO_DEG;
    s.gyroAngleDeg = st.lastGyroAngleRad * MATH_RAD_TO_DEG;

    s.accelNormG = st.lastAccelGate.normG;
    s.accelTrust = st.lastAccelGate.trust;
    s.accelInnovationDeg = st.lastAccelGate.innovationRad * MATH_RAD_TO_DEG;
    s.accelCorrectionDeg = st.lastAccelCorrectionAngleRad * MATH_RAD_TO_DEG;

    s.updateCount = st.updateCount;
    s.accelUpdateCount = st.accelUpdateCount;
    s.accelRejectedCount = st.accelRejectedCount;
    s.skippedBadDt = st.skippedBadDt;
    s.clampedLargeDt = st.clampedLargeDt;

    return s;
}

} // namespace tracker
