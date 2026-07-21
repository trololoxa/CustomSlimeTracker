#include "sensor/ahrs_6dof.hpp"

#include <cmath>

#include "build_config/tracking_tuning.hpp"

namespace tracker {

Ahrs6Dof::Ahrs6Dof(const Ahrs6DofConfig& config)
    : cfg_(sanitizeConfig(config)) {}

void Ahrs6Dof::reset(const Quat& initialQ, uint64_t timestampUs) {
    q_ = initialQ.normalized().withPositiveW();
    stats_ = Ahrs6DofStats{};
    stats_.lastSeenTimestampUs = timestampUs;
    stats_.lastIntegratedTimestampUs = timestampUs;
    stats_.lastTimestampUs = timestampUs;
    resetAccelCorrectionAccumulator();
    initialized_ = timestampUs != 0;
}

bool Ahrs6Dof::resetFromAccel(const Vec3& accelG, uint64_t timestampUs) {
    if (timestampUs != 0) {
        stats_.lastSeenTimestampUs = timestampUs;
    }

    const float n = accelG.norm();
    if (!accelG.isFinite() || !std::isfinite(n) || n < MATH_EPSILON) {
        return false;
    }

    // Do not silently initialize AHRS to identity from a rejected/zero accel
    // vector. Startup should wait for a plausible gravity vector; otherwise
    // roll/pitch may start wrong and look like a valid quaternion.
    if (std::fabs(n - 1.0f) > cfg_.accelNormBadErrorG) {
        return false;
    }

    const Vec3 a = accelG / n;
    if (!a.isFinite() || a.normSq() < MATH_EPSILON) {
        return false;
    }

    // q rotates measured up direction from sensor frame into worldUp.
    const uint32_t startupRejects = stats_.startupAccelRejectedCount;
    q_ = Quat::fromTwoUnitVectors(a, cfg_.worldUp).withPositiveW();
    stats_ = Ahrs6DofStats{};
    stats_.startupAccelRejectedCount = startupRejects;
    stats_.lastSeenTimestampUs = timestampUs;
    stats_.lastIntegratedTimestampUs = timestampUs;
    stats_.lastTimestampUs = timestampUs;
    resetAccelCorrectionAccumulator();
    initialized_ = timestampUs != 0;
    return true;
}

bool Ahrs6Dof::reacquireTiltFromAccelPreserveHeading(const Vec3& accelG, uint64_t timestampUs) {
    if (timestampUs == 0) return false;

    const float n = accelG.norm();
    if (!accelG.isFinite() || !std::isfinite(n) || n < MATH_EPSILON ||
        std::fabs(n - 1.0f) > cfg_.accelNormGoodErrorG) {
        return false;
    }
    if (!initialized_) return resetFromAccel(accelG, timestampUs);

    const Vec3 measuredUp = accelG / n;
    const Vec3 predictedUp = q_.rotate(measuredUp).normalized();
    if (!predictedUp.isFinite()) return false;

    // Left-multiply the smallest world-frame rotation that restores gravity.
    // Its axis is horizontal, so the correction contains no rotation around
    // world-up and therefore does not invent a yaw observation. The antipodal
    // case is ambiguous; rotate around the best observable device horizontal
    // axis to keep that axis' heading continuous.
    Quat correction;
    if (dot(predictedUp, cfg_.worldUp) < -0.9999f) {
        Vec3 axis = q_.rotate(Vec3::unitY());
        axis -= cfg_.worldUp * dot(axis, cfg_.worldUp);
        if (!axis.normalizeInPlace()) {
            axis = q_.rotate(Vec3::unitX());
            axis -= cfg_.worldUp * dot(axis, cfg_.worldUp);
            if (!axis.normalizeInPlace()) return false;
        }
        correction = Quat::fromAxisAngle(axis, MATH_PI);
    } else {
        correction = Quat::fromTwoUnitVectors(predictedUp, cfg_.worldUp);
    }

    const Quat candidate = (correction * q_).normalized().withPositiveW();
    const Vec3 mappedUp = candidate.rotate(measuredUp).normalized();
    if (!candidate.isFinite() || !mappedUp.isFinite() ||
        dot(mappedUp, cfg_.worldUp) < 0.999f) {
        return false;
    }

    q_ = candidate;
    initialized_ = true;
    stats_.lastSeenTimestampUs = timestampUs;
    stats_.lastIntegratedTimestampUs = timestampUs;
    stats_.lastTimestampUs = timestampUs;
    stats_.lastDtS = 0.0f;
    stats_.lastUsedDtS = 0.0f;
    stats_.lastGyroRadS = Vec3::zero();
    stats_.lastGyroAngleRad = 0.0f;
    stats_.lastAccelG = accelG;
    stats_.lastAccelUnitSensor = measuredUp;
    stats_.lastAccelUnitWorld = cfg_.worldUp;
    stats_.lastAccelErrorWorld = Vec3::zero();
    stats_.accelNormStatsInitialized = false;
    stats_.accelNormMeanG = 0.0f;
    stats_.accelNormVarianceG2 = 0.0f;
    stats_.lastAccelNormVarianceTrust = 1.0f;
    stats_.lastGyroMotionTrust = 1.0f;
    stats_.lastAdaptiveAccelTrust = 1.0f;
    stats_.lastAccelGate = Ahrs6DofAccelGate{};
    stats_.lastAccelCorrectionWorldRad = Vec3::zero();
    stats_.lastAccelCorrectionAngleRad = 0.0f;
    resetAccelCorrectionAccumulator();
    return true;
}

bool Ahrs6Dof::initialized() const {
    return initialized_;
}

const Quat& Ahrs6Dof::quaternion() const {
    return q_;
}

Quat Ahrs6Dof::quaternionPositiveW() const {
    return q_.withPositiveW();
}

Vec3 Ahrs6Dof::eulerRad() const {
    return q_.toEulerXYZ();
}

Vec3 Ahrs6Dof::eulerDeg() const {
    return eulerRad() * MATH_RAD_TO_DEG;
}

const Ahrs6DofStats& Ahrs6Dof::stats() const {
    return stats_;
}

const Ahrs6DofConfig& Ahrs6Dof::config() const {
    return cfg_;
}

void Ahrs6Dof::setConfig(const Ahrs6DofConfig& config) {
    cfg_ = sanitizeConfig(config);
    resetAccelCorrectionAccumulator();
}

void Ahrs6Dof::setQuaternion(const Quat& q) {
    q_ = q.normalized().withPositiveW();
    resetAccelCorrectionAccumulator();
}

void Ahrs6Dof::rebaseTimestamp(uint64_t timestampUs) {
    stats_.lastSeenTimestampUs = timestampUs;
    if (!initialized_) {
        return;
    }

    stats_.lastTimestampUs = timestampUs;
    stats_.lastDtS = 0.0f;
    stats_.lastUsedDtS = 0.0f;
    stats_.fifoRecoveryRebaseCount++;
    stats_.lastRebaseTimestampUs = timestampUs;
    stats_.postFifoRecoverySamples = 0;
    resetAccelCorrectionAccumulator();
}

Ahrs6DofAccelGate Ahrs6Dof::evaluateAccelGate(const Vec3& accelG) const {
    return evaluateAccel(accelG, accelG.norm()).gate;
}

bool Ahrs6Dof::update(const Vec3& gyroRadS, const Vec3& accelG, uint64_t timestampUs) {
    return update(gyroRadS, accelG, accelG.norm(), timestampUs);
}

bool Ahrs6Dof::update(const Vec3& gyroRadS, const Vec3& accelG, float accelNormG, uint64_t timestampUs) {
    if (!gyroRadS.isFinite() || !accelG.isFinite() || timestampUs == 0) {
        stats_.skippedBadDt++;
        return false;
    }

    if (!initialized_) {
        stats_.lastSeenTimestampUs = timestampUs;
        stats_.lastGyroRadS = gyroRadS;
        stats_.lastAccelG = accelG;
        if (!resetFromAccel(accelG, timestampUs)) {
            stats_.startupAccelRejectedCount++;
        }
        return false;
    }

    stats_.lastSeenTimestampUs = timestampUs;
    const uint64_t lastUs = stats_.lastTimestampUs;

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
            // A blocking operation such as Wi-Fi scan or a FIFO overrun can
            // leave a multi-second timestamp gap. Reject that sample, but
            // rebase the integration timestamp to the current stream position.
            // Otherwise every following normal sample is still compared against
            // the old pre-gap timestamp, so AHRS gyro prediction remains
            // permanently frozen while diagnostics still show fresh samples.
            stats_.skippedBadDt++;
            stats_.largeDtRebaseCount++;
            stats_.lastRebaseTimestampUs = timestampUs;
            stats_.lastTimestampUs = timestampUs;
            stats_.lastUsedDtS = 0.0f;
            stats_.lastGyroRadS = gyroRadS;
            stats_.lastAccelG = accelG;
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

    const float gyroNormSq = gyroUsed.normSq();

    // 1. Full-rate gyro prediction. The incremental exponential map uses a
    // sixth-order small-angle path and leaves normalization to normalizeEvery.
    q_ = integrateBodyRateFast(q_, gyroUsed, dtS);

    stats_.updateCount++;
    stats_.gyroPredictCount++;
    stats_.lastIntegratedTimestampUs = timestampUs;
    stats_.lastTimestampUs = timestampUs;
    stats_.lastUsedDtS = dtS;
    stats_.lastGyroRadS = gyroUsed;
    stats_.lastAccelG = accelG;
    if (stats_.fifoRecoveryRebaseCount > 0) {
        stats_.postFifoRecoverySamples++;
    }

    // 2. Gravity correction is accumulated over a four-sample window. Gyro
    // prediction remains full-rate; only the low-bandwidth absolute tilt
    // correction is decimated, with the full represented dt applied at once.
    accumulateAccelCorrection(accelG, accelNormG, gyroNormSq, dtS);

    if (cfg_.normalizeEvery > 0 && (stats_.updateCount % cfg_.normalizeEvery) == 0) {
        q_.normalizeInPlace();
        q_ = q_.withPositiveW();
        stats_.normalizedCount++;
    }

    return true;
}

Ahrs6DofConfig Ahrs6Dof::sanitizeConfig(Ahrs6DofConfig cfg) {
    cfg.worldUp = cfg.worldUp.normalized();
    if (!cfg.worldUp.isFinite() || cfg.worldUp.normSq() < MATH_EPSILON) {
        cfg.worldUp = Vec3(0.0f, 0.0f, 1.0f);
    }

    if (!std::isfinite(cfg.minDtS) || cfg.minDtS <= 0.0f) cfg.minDtS = 0.0001f;
    if (!std::isfinite(cfg.maxDtS) || cfg.maxDtS <= cfg.minDtS) cfg.maxDtS = 0.0200f;
    if (!std::isfinite(cfg.gyroDeadbandRadS) || cfg.gyroDeadbandRadS < 0.0f) cfg.gyroDeadbandRadS = 0.0f;
    if (!std::isfinite(cfg.accelKp) || cfg.accelKp <= 0.0f) cfg.accelKp = 3.0f;
    if (!std::isfinite(cfg.maxAccelCorrectionRadPerUpdate) || cfg.maxAccelCorrectionRadPerUpdate <= 0.0f) {
        cfg.maxAccelCorrectionRadPerUpdate = 2.0f * MATH_DEG_TO_RAD;
    }
    if (!std::isfinite(cfg.accelNormGoodErrorG) || cfg.accelNormGoodErrorG < 0.0f) cfg.accelNormGoodErrorG = 0.06f;
    if (!std::isfinite(cfg.accelNormBadErrorG) || cfg.accelNormBadErrorG <= cfg.accelNormGoodErrorG) cfg.accelNormBadErrorG = 0.35f;
    if (!std::isfinite(cfg.accelInnovationGoodRad) || cfg.accelInnovationGoodRad < 0.0f) cfg.accelInnovationGoodRad = 8.0f * MATH_DEG_TO_RAD;
    if (!std::isfinite(cfg.accelInnovationBadRad) || cfg.accelInnovationBadRad <= cfg.accelInnovationGoodRad) cfg.accelInnovationBadRad = 45.0f * MATH_DEG_TO_RAD;
    if (!std::isfinite(cfg.accelNormVarianceGoodG2) || cfg.accelNormVarianceGoodG2 < 0.0f) cfg.accelNormVarianceGoodG2 = square(0.010f);
    if (!std::isfinite(cfg.accelNormVarianceBadG2) || cfg.accelNormVarianceBadG2 <= cfg.accelNormVarianceGoodG2) cfg.accelNormVarianceBadG2 = square(0.080f);
    if (!std::isfinite(cfg.accelNormVarianceAlpha) || cfg.accelNormVarianceAlpha <= 0.0f || cfg.accelNormVarianceAlpha > 1.0f) cfg.accelNormVarianceAlpha = 0.02f;
    if (!std::isfinite(cfg.gyroNormAccelTrustGoodRadS) || cfg.gyroNormAccelTrustGoodRadS < 0.0f) cfg.gyroNormAccelTrustGoodRadS = 250.0f * MATH_DEG_TO_RAD;
    if (!std::isfinite(cfg.gyroNormAccelTrustBadRadS) || cfg.gyroNormAccelTrustBadRadS <= cfg.gyroNormAccelTrustGoodRadS) cfg.gyroNormAccelTrustBadRadS = 720.0f * MATH_DEG_TO_RAD;
    if (cfg.normalizeEvery == 0) cfg.normalizeEvery = 16;
    return cfg;
}

float Ahrs6Dof::rampDown(float x, float good, float bad) {
    if (x <= good) return 1.0f;
    if (x >= bad) return 0.0f;
    if (bad <= good) return 0.0f;
    return 1.0f - ((x - good) / (bad - good));
}

Ahrs6Dof::AccelEvaluation Ahrs6Dof::evaluateAccel(const Vec3& accelG, float accelNormG) const {
    AccelEvaluation eval;
    Ahrs6DofAccelGate& gate = eval.gate;

    if (!cfg_.accelCorrectionEnabled) {
        return eval;
    }

    if (!accelG.isFinite() || !std::isfinite(accelNormG)) {
        return eval;
    }

    gate.normG = accelNormG;
    gate.normErrorG = std::fabs(gate.normG - 1.0f);

    if (gate.normG < MATH_EPSILON) {
        return eval;
    }

    gate.normTrust = rampDown(gate.normErrorG,
                              cfg_.accelNormGoodErrorG,
                              cfg_.accelNormBadErrorG);

    eval.accelUnitSensor = accelG / gate.normG;
    eval.accelUnitWorld = q_.rotate(eval.accelUnitSensor);

    if (!eval.accelUnitWorld.isFinite() || eval.accelUnitWorld.normSq() < MATH_EPSILON) {
        return eval;
    }

    eval.vectorsValid = true;
    gate.innovationRad = acosFastUnitDot(dot(eval.accelUnitWorld, cfg_.worldUp));
    gate.innovationTrust = rampDown(gate.innovationRad,
                                    cfg_.accelInnovationGoodRad,
                                    cfg_.accelInnovationBadRad);

    gate.varianceTrust = cfg_.adaptiveAccelCorrection ? stats_.lastAccelNormVarianceTrust : 1.0f;
    gate.gyroMotionTrust = cfg_.adaptiveAccelCorrection ? stats_.lastGyroMotionTrust : 1.0f;
    gate.trust = gate.normTrust * gate.innovationTrust * gate.varianceTrust * gate.gyroMotionTrust;
    gate.accepted = gate.trust > 0.0f;
    return eval;
}

void Ahrs6Dof::updateAdaptiveAccelTrust(float accelNormG, float gyroNorm, uint8_t representedSamples) {
    stats_.lastAccelNormVarianceTrust = 1.0f;
    stats_.lastGyroMotionTrust = 1.0f;
    stats_.lastAdaptiveAccelTrust = 1.0f;

    if (!cfg_.adaptiveAccelCorrection || !std::isfinite(accelNormG) || !std::isfinite(gyroNorm)) {
        return;
    }

    if (!stats_.accelNormStatsInitialized) {
        stats_.accelNormStatsInitialized = true;
        stats_.accelNormMeanG = accelNormG;
        stats_.accelNormVarianceG2 = 0.0f;
    } else {
        float retention = 1.0f;
        const float perSampleRetention = 1.0f - cfg_.accelNormVarianceAlpha;
        const uint8_t count = representedSamples > 0u ? representedSamples : 1u;
        for (uint8_t i = 0; i < count; ++i) retention *= perSampleRetention;
        const float alpha = 1.0f - retention;
        const float delta = accelNormG - stats_.accelNormMeanG;
        stats_.accelNormMeanG += alpha * delta;
        const float delta2 = accelNormG - stats_.accelNormMeanG;
        const float sampleVar = delta * delta2;
        stats_.accelNormVarianceG2 = (1.0f - alpha) * stats_.accelNormVarianceG2 + alpha * std::fabs(sampleVar);
    }

    stats_.lastAccelNormVarianceTrust = rampDown(stats_.accelNormVarianceG2,
                                                cfg_.accelNormVarianceGoodG2,
                                                cfg_.accelNormVarianceBadG2);

    stats_.lastGyroMotionTrust = rampDown(gyroNorm,
                                          cfg_.gyroNormAccelTrustGoodRadS,
                                          cfg_.gyroNormAccelTrustBadRadS);
    stats_.lastAdaptiveAccelTrust = stats_.lastAccelNormVarianceTrust * stats_.lastGyroMotionTrust;
}

void Ahrs6Dof::applyAccelCorrection(const Vec3& accelG, float accelNormG, float dtS) {
    const AccelEvaluation eval = evaluateAccel(accelG, accelNormG);
    const Ahrs6DofAccelGate& gate = eval.gate;
    stats_.lastAccelGate = gate;
    stats_.lastAccelCorrectionWorldRad = Vec3::zero();
    stats_.lastAccelCorrectionAngleRad = 0.0f;

    if (!gate.accepted || !eval.vectorsValid) {
        stats_.accelRejectedCount++;
        return;
    }

    // errorWorld rotates accelUnitWorld toward worldUp.
    // For small angles, correction ~= cross(current, target).
    const Vec3 errorWorld = cross(eval.accelUnitWorld, cfg_.worldUp);

    Vec3 correctionWorldRad = errorWorld * (cfg_.accelKp * gate.trust * dtS);

    const float corrNormSq = correctionWorldRad.normSq();
    float corrNorm = std::sqrt(corrNormSq);
    if (corrNorm > cfg_.maxAccelCorrectionRadPerUpdate && corrNorm > MATH_EPSILON) {
        correctionWorldRad *= cfg_.maxAccelCorrectionRadPerUpdate / corrNorm;
        corrNorm = cfg_.maxAccelCorrectionRadPerUpdate;
    }

    q_ = applyWorldCorrectionFast(q_, correctionWorldRad);

    stats_.accelUpdateCount++;
    stats_.lastAccelUnitSensor = eval.accelUnitSensor;
    stats_.lastAccelUnitWorld = eval.accelUnitWorld;
    stats_.lastAccelErrorWorld = errorWorld;
    stats_.lastAccelCorrectionWorldRad = correctionWorldRad;
    stats_.lastAccelCorrectionAngleRad = corrNorm;
}

void Ahrs6Dof::resetAccelCorrectionAccumulator() {
    accelCorrectionWeightedSum_ = Vec3::zero();
    accelCorrectionValidDtS_ = 0.0f;
    accelCorrectionElapsedDtS_ = 0.0f;
    accelCorrectionMaxGyroNormSq_ = 0.0f;
    accelCorrectionSamples_ = 0;
    accelCorrectionValidSamples_ = 0;
}

void Ahrs6Dof::accumulateAccelCorrection(const Vec3& accelG,
                                         float accelNormG,
                                         float gyroNormSq,
                                         float dtS) {
    accelCorrectionElapsedDtS_ += dtS;
    if (std::isfinite(gyroNormSq) && gyroNormSq > accelCorrectionMaxGyroNormSq_) {
        accelCorrectionMaxGyroNormSq_ = gyroNormSq;
    }
    if (accelG.isFinite() && std::isfinite(accelNormG) && accelNormG > MATH_EPSILON) {
        accelCorrectionWeightedSum_ += accelG * dtS;
        accelCorrectionValidDtS_ += dtS;
        ++accelCorrectionValidSamples_;
    }

    ++accelCorrectionSamples_;
    if (accelCorrectionSamples_ < cfg::AHRS_ACCEL_CORRECTION_DIVISOR) return;

    const uint8_t representedSamples = accelCorrectionSamples_;
    const float gyroNorm = std::sqrt(accelCorrectionMaxGyroNormSq_);
    stats_.lastGyroAngleRad = gyroNorm * accelCorrectionElapsedDtS_;

    if (accelCorrectionValidDtS_ > MATH_EPSILON) {
        const Vec3 averagedAccel = accelCorrectionWeightedSum_ / accelCorrectionValidDtS_;
        const float averagedNorm = averagedAccel.norm();
        updateAdaptiveAccelTrust(averagedNorm, gyroNorm, accelCorrectionValidSamples_);
        applyAccelCorrection(averagedAccel, averagedNorm, accelCorrectionValidDtS_);
    } else {
        updateAdaptiveAccelTrust(0.0f, gyroNorm, representedSamples);
        applyAccelCorrection(Vec3::zero(), 0.0f, accelCorrectionElapsedDtS_);
    }

    resetAccelCorrectionAccumulator();
}

Ahrs6DofDebugSnapshot makeAhrs6DofDebugSnapshot(const Ahrs6Dof& ahrs) {
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
    s.accelNormVarianceG2 = st.accelNormVarianceG2;
    s.accelVarianceTrust = st.lastAccelNormVarianceTrust;
    s.gyroMotionTrust = st.lastGyroMotionTrust;

    s.updateCount = st.updateCount;
    s.accelUpdateCount = st.accelUpdateCount;
    s.accelRejectedCount = st.accelRejectedCount;
    s.skippedBadDt = st.skippedBadDt;
    s.clampedLargeDt = st.clampedLargeDt;
    s.largeDtRebaseCount = st.largeDtRebaseCount;
    s.fifoRecoveryRebaseCount = st.fifoRecoveryRebaseCount;
    s.lastRebaseTimestampUs = st.lastRebaseTimestampUs;
    s.postFifoRecoverySamples = st.postFifoRecoverySamples;
    s.startupAccelRejectedCount = st.startupAccelRejectedCount;

    return s;
}

} // namespace tracker
