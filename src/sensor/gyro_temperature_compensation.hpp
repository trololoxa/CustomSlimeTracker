#pragma once

#include <cstdint>
#include <cmath>

#include "core/math.hpp"

namespace tracker {

// ============================================================
// Gyro temperature compensation
// ============================================================
// Model:
//   gyroBias(T) = referenceBias + slopeRadSPerC * (T - referenceTempC)
//
// Important:
//   - At first slope is zero, so behavior is exactly the same as normal
//     startup gyro bias calibration.
//   - Slope must be learned from long stationary logs or manually configured.
//   - Online learning is gated; do not learn during motion.
// ============================================================

struct GyroTempCompConfig {
    bool enabled = true;
    bool learningEnabled = false;

    float minDeltaTempForLearningC = 2.0f;
    float learnAlpha = 0.05f;

    // Reject physically suspicious slopes during online learning.
    // 0.10 dps/C is intentionally permissive for early experiments.
    float maxAbsSlopeDpsPerC = 0.10f;

    // Stationary rolling mean must be reasonably small after current model.
    // Otherwise the window probably contains motion and should not train temp comp.
    float maxResidualMeanDpsForLearning = 0.30f;
};

struct GyroTempCompSnapshot {
    bool valid = false;
    bool enabled = false;
    bool learningEnabled = false;

    float referenceTempC = 25.0f;
    float currentTempC = 25.0f;
    float deltaTempC = 0.0f;

    Vec3 referenceBiasRadS = Vec3::zero();
    Vec3 referenceBiasDps = Vec3::zero();

    Vec3 slopeRadSPerC = Vec3::zero();
    Vec3 slopeDpsPerC = Vec3::zero();

    Vec3 currentBiasRadS = Vec3::zero();
    Vec3 currentBiasDps = Vec3::zero();

    uint32_t learnAccepted = 0;
    uint32_t learnRejected = 0;
};

class GyroTempCompensator {
public:
    explicit GyroTempCompensator(const GyroTempCompConfig& config = GyroTempCompConfig{})
        : cfg_(config) {}

    void reset(const Vec3& referenceBiasRadS, float referenceTempC) {
        referenceBiasRadS_ = referenceBiasRadS;
        referenceTempC_ = referenceTempC;
        slopeRadSPerC_ = Vec3::zero();
        valid_ = true;
        learnAccepted_ = 0;
        learnRejected_ = 0;
    }

    void setConfig(const GyroTempCompConfig& config) {
        cfg_ = config;
    }

    const GyroTempCompConfig& config() const {
        return cfg_;
    }

    void setEnabled(bool enabled) {
        cfg_.enabled = enabled;
    }

    void setLearningEnabled(bool enabled) {
        cfg_.learningEnabled = enabled;
    }

    void setSlopeDpsPerC(const Vec3& slopeDpsPerC) {
        slopeRadSPerC_ = slopeDpsPerC * MATH_DEG_TO_RAD;
    }

    void setSlopeRadSPerC(const Vec3& slopeRadSPerC) {
        slopeRadSPerC_ = slopeRadSPerC;
    }

    bool valid() const {
        return valid_;
    }

    Vec3 referenceBiasRadS() const {
        return referenceBiasRadS_;
    }

    Vec3 referenceBiasDps() const {
        return referenceBiasRadS_ * MATH_RAD_TO_DEG;
    }

    float referenceTempC() const {
        return referenceTempC_;
    }

    Vec3 slopeRadSPerC() const {
        return slopeRadSPerC_;
    }

    Vec3 slopeDpsPerC() const {
        return slopeRadSPerC_ * MATH_RAD_TO_DEG;
    }

    Vec3 biasAt(float tempC) const {
        if (!valid_) {
            return Vec3::zero();
        }

        if (!cfg_.enabled) {
            return referenceBiasRadS_;
        }

        const float dT = tempC - referenceTempC_;
        return referenceBiasRadS_ + slopeRadSPerC_ * dT;
    }

    Vec3 correctedGyro(const Vec3& rawGyroRadS, float tempC) const {
        return rawGyroRadS - biasAt(tempC);
    }

    // Learn slope from a stationary rolling mean of raw gyro.
    // rawGyroMeanRadS should be BEFORE compensation, averaged over 1-2 seconds.
    bool learnFromStationaryMean(const Vec3& rawGyroMeanRadS,
                                 float tempC,
                                 bool stationaryGate) {
        if (!valid_ || !cfg_.learningEnabled || !stationaryGate || !rawGyroMeanRadS.isFinite()) {
            learnRejected_++;
            return false;
        }

        const float dT = tempC - referenceTempC_;
        if (std::fabs(dT) < cfg_.minDeltaTempForLearningC) {
            learnRejected_++;
            return false;
        }

        const Vec3 currentResidualRadS = rawGyroMeanRadS - biasAt(tempC);
        const float currentResidualDps = currentResidualRadS.norm() * MATH_RAD_TO_DEG;
        if (currentResidualDps > cfg_.maxResidualMeanDpsForLearning) {
            learnRejected_++;
            return false;
        }

        const Vec3 candidateSlopeRadSPerC = (rawGyroMeanRadS - referenceBiasRadS_) / dT;
        const Vec3 candidateSlopeDpsPerC = candidateSlopeRadSPerC * MATH_RAD_TO_DEG;

        if (std::fabs(candidateSlopeDpsPerC.x) > cfg_.maxAbsSlopeDpsPerC ||
            std::fabs(candidateSlopeDpsPerC.y) > cfg_.maxAbsSlopeDpsPerC ||
            std::fabs(candidateSlopeDpsPerC.z) > cfg_.maxAbsSlopeDpsPerC) {
            learnRejected_++;
            return false;
        }

        slopeRadSPerC_ = lerp(slopeRadSPerC_, candidateSlopeRadSPerC, cfg_.learnAlpha);
        learnAccepted_++;
        return true;
    }

    GyroTempCompSnapshot snapshot(float currentTempC) const {
        GyroTempCompSnapshot s;
        s.valid = valid_;
        s.enabled = cfg_.enabled;
        s.learningEnabled = cfg_.learningEnabled;
        s.referenceTempC = referenceTempC_;
        s.currentTempC = currentTempC;
        s.deltaTempC = currentTempC - referenceTempC_;
        s.referenceBiasRadS = referenceBiasRadS_;
        s.referenceBiasDps = referenceBiasDps();
        s.slopeRadSPerC = slopeRadSPerC_;
        s.slopeDpsPerC = slopeDpsPerC();
        s.currentBiasRadS = biasAt(currentTempC);
        s.currentBiasDps = s.currentBiasRadS * MATH_RAD_TO_DEG;
        s.learnAccepted = learnAccepted_;
        s.learnRejected = learnRejected_;
        return s;
    }

private:
    GyroTempCompConfig cfg_;
    bool valid_ = false;

    Vec3 referenceBiasRadS_ = Vec3::zero();
    float referenceTempC_ = 25.0f;
    Vec3 slopeRadSPerC_ = Vec3::zero();

    uint32_t learnAccepted_ = 0;
    uint32_t learnRejected_ = 0;
};

} // namespace tracker
