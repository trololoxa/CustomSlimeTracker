#pragma once

#include <cstdint>

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

    // Master switch for accel gravity correction. Gyro integration still runs.
    bool accelCorrectionEnabled = true;

    // Adaptive accel trust adds norm-variance and gyro-motion gates on top of
    // norm/innovation checks. This is intentionally conservative: it only
    // weakens accel correction during vibration, impacts or very fast motion.
    bool adaptiveAccelCorrection = true;

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

    // Adaptive accel trust: exponentially tracked variance of accel norm.
    // stddev <= 0.01g => full trust, stddev >= 0.08g => zero adaptive trust.
    float accelNormVarianceGoodG2 = square(0.010f);
    float accelNormVarianceBadG2 = square(0.080f);
    float accelNormVarianceAlpha = 0.02f;

    // Very fast gyro motion often correlates with linear acceleration/vibration.
    // Keep the gate permissive so normal body movement is not penalized.
    float gyroNormAccelTrustGoodRadS = 250.0f * MATH_DEG_TO_RAD;
    float gyroNormAccelTrustBadRadS = 720.0f * MATH_DEG_TO_RAD;

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
    float varianceTrust = 1.0f;
    float gyroMotionTrust = 1.0f;
};

struct Ahrs6DofStats {
    uint32_t updateCount = 0;
    uint32_t gyroPredictCount = 0;
    uint32_t accelUpdateCount = 0;
    uint32_t accelRejectedCount = 0;
    uint32_t skippedBadDt = 0;
    uint32_t startupAccelRejectedCount = 0;
    uint32_t clampedLargeDt = 0;
    uint32_t normalizedCount = 0;

    // Recovery/rebase diagnostics. largeDtRebaseCount counts automatic
    // rebases after a rejected over-large dt when clampLargeDt=false.
    // fifoRecoveryRebaseCount counts explicit recovery-timebase rebases,
    // normally from FIFO recovery / tracking recovery. postFifoRecoverySamples
    // is reset on each explicit recovery rebase and then counts accepted gyro
    // prediction updates after it.
    uint32_t largeDtRebaseCount = 0;
    uint32_t fifoRecoveryRebaseCount = 0;
    uint64_t lastRebaseTimestampUs = 0;
    uint32_t postFifoRecoverySamples = 0;

    // Timestamp semantics:
    //   lastSeenTimestampUs       = latest timestamp observed by update(), even if rejected.
    //   lastIntegratedTimestampUs = latest timestamp represented by the current quaternion.
    //   lastTimestampUs           = timebase baseline used for the next dt calculation.
    //
    // A rejected oversized gap advances lastTimestampUs so normal prediction can
    // resume, but it must not advance lastIntegratedTimestampUs or make stale
    // orientation look coherent with the rejected sample.
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

    // Adaptive accel-correction diagnostics.
    bool accelNormStatsInitialized = false;
    float accelNormMeanG = 0.0f;
    float accelNormVarianceG2 = 0.0f;
    float lastAccelNormVarianceTrust = 1.0f;
    float lastGyroMotionTrust = 1.0f;
    float lastAdaptiveAccelTrust = 1.0f;

    Ahrs6DofAccelGate lastAccelGate;
};

class Ahrs6Dof {
public:
    explicit Ahrs6Dof(const Ahrs6DofConfig& config = Ahrs6DofConfig{});

    void reset(const Quat& initialQ = Quat::identity(), uint64_t timestampUs = 0);
    bool resetFromAccel(const Vec3& accelG, uint64_t timestampUs = 0);

    // Rebuild roll/pitch from gravity after an unreconstructable sample gap
    // while preserving the pre-gap horizontal heading. This is intentionally
    // separate from resetFromAccel(), whose startup solution has no yaw
    // reference. Returns false until accel is a plausible gravity vector.
    bool reacquireTiltFromAccelPreserveHeading(const Vec3& accelG, uint64_t timestampUs);

    bool initialized() const;
    const Quat& quaternion() const;
    Quat quaternionPositiveW() const;
    Vec3 eulerRad() const;
    Vec3 eulerDeg() const;
    const Ahrs6DofStats& stats() const;
    const Ahrs6DofConfig& config() const;

    void setConfig(const Ahrs6DofConfig& config);
    void setQuaternion(const Quat& q);

    // Explicitly rebase AHRS integration time after a known recovery event
    // (FIFO reset, timestamp reconstruction reset, sensor reset). This does
    // not rotate the quaternion and must not be used for ordinary rejected
    // samples; it is for explicit RECOVERING paths where the missing interval
    // cannot be reconstructed safely.
    void rebaseTimestamp(uint64_t timestampUs);

    Ahrs6DofAccelGate evaluateAccelGate(const Vec3& accelG) const;

    // Main update function.
    // gyroRadS must already be calibrated: raw gyro - startup bias.
    // accelG may be raw scaled accel or calibrated accel if accel calibration exists.
    // Returns true if gyro prediction was performed.
    bool update(const Vec3& gyroRadS, const Vec3& accelG, uint64_t timestampUs);
    bool update(const Vec3& gyroRadS, const Vec3& accelG, float accelNormG, uint64_t timestampUs);

private:
    struct AccelEvaluation {
        Ahrs6DofAccelGate gate;
        Vec3 accelUnitSensor = Vec3::zero();
        Vec3 accelUnitWorld = Vec3::zero();
        bool vectorsValid = false;
    };

    static Ahrs6DofConfig sanitizeConfig(Ahrs6DofConfig cfg);
    static float rampDown(float x, float good, float bad);

    AccelEvaluation evaluateAccel(const Vec3& accelG, float accelNormG) const;
    void updateAdaptiveAccelTrust(float accelNormG, float gyroNorm, uint8_t representedSamples);
    void applyAccelCorrection(const Vec3& accelG, float accelNormG, float dtS);
    void resetAccelCorrectionAccumulator();
    void accumulateAccelCorrection(const Vec3& accelG,
                                   float accelNormG,
                                   float gyroNormSq,
                                   float dtS);

    Ahrs6DofConfig cfg_;
    Ahrs6DofStats stats_;
    Quat q_ = Quat::identity();
    Vec3 accelCorrectionWeightedSum_ = Vec3::zero();
    float accelCorrectionValidDtS_ = 0.0f;
    float accelCorrectionElapsedDtS_ = 0.0f;
    float accelCorrectionMaxGyroNormSq_ = 0.0f;
    uint8_t accelCorrectionSamples_ = 0;
    uint8_t accelCorrectionValidSamples_ = 0;
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
    uint32_t startupAccelRejectedCount = 0;
    uint32_t clampedLargeDt = 0;
    uint32_t largeDtRebaseCount = 0;
    uint32_t fifoRecoveryRebaseCount = 0;
    uint64_t lastRebaseTimestampUs = 0;
    uint32_t postFifoRecoverySamples = 0;

    float accelNormVarianceG2 = 0.0f;
    float accelVarianceTrust = 1.0f;
    float gyroMotionTrust = 1.0f;
};

Ahrs6DofDebugSnapshot makeAhrs6DofDebugSnapshot(const Ahrs6Dof& ahrs);

} // namespace tracker
