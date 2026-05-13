#pragma once

#include <cstdint>

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

    // Production quality metadata. The compensator can still be valid without
    // a fitted range, but runtime/status should then treat it as startup-bias
    // compensation rather than a validated temperature model.
    float calibratedTempMinC = 0.0f;
    float calibratedTempMaxC = 0.0f;
    float fitQuality = 0.0f;
    float fitResidualBeforeDps = 0.0f;
    float fitResidualAfterDps = 0.0f;
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

    float calibratedTempMinC = 0.0f;
    float calibratedTempMaxC = 0.0f;
    float fitQuality = 0.0f;
    float fitResidualBeforeDps = 0.0f;
    float fitResidualAfterDps = 0.0f;
    bool hasCalibratedRange = false;
    bool tempOutOfRange = false;
};

class GyroTempCompensator {
public:
    explicit GyroTempCompensator(const GyroTempCompConfig& config = GyroTempCompConfig{});

    void reset(const Vec3& referenceBiasRadS, float referenceTempC);
    void setModel(const Vec3& referenceBiasRadS,
                  float referenceTempC,
                  const Vec3& slopeRadSPerC);
    void adjustReferenceBias(const Vec3& deltaRadS);

    void setQualityMetadata(float tempMinC,
                            float tempMaxC,
                            float fitQuality,
                            float residualBeforeDps,
                            float residualAfterDps);

    void setConfig(const GyroTempCompConfig& config);
    const GyroTempCompConfig& config() const;

    void setEnabled(bool enabled);
    void setLearningEnabled(bool enabled);

    void setSlopeDpsPerC(const Vec3& slopeDpsPerC);
    void setSlopeRadSPerC(const Vec3& slopeRadSPerC);

    bool valid() const;

    Vec3 referenceBiasRadS() const;
    Vec3 referenceBiasDps() const;
    float referenceTempC() const;
    Vec3 slopeRadSPerC() const;
    Vec3 slopeDpsPerC() const;

    Vec3 biasAt(float tempC) const;
    Vec3 correctedGyro(const Vec3& rawGyroRadS, float tempC) const;

    // Learn slope from a stationary rolling mean of raw gyro.
    // rawGyroMeanRadS should be BEFORE compensation, averaged over 1-2 seconds.
    bool learnFromStationaryMean(const Vec3& rawGyroMeanRadS,
                                 float tempC,
                                 bool stationaryGate);

    GyroTempCompSnapshot snapshot(float currentTempC) const;

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
