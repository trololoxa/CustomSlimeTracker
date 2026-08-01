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
//   - Slope is produced by the validated setup/static-fit pipeline.
//   - Runtime online slope learning is intentionally not exposed until a full
//     producer/persistence/rollback contract exists.
// ============================================================

struct GyroTempCompConfig {
    bool enabled = true;

    // Reject physically implausible slopes produced by the validated
    // setup/static-fit pipeline. This is an offline fit acceptance limit,
    // not a runtime online-learning parameter.
    float maxAcceptedSlopeDpsPerC = 0.10f;

    // Production quality metadata. The compensator can still be valid without
    // a fitted range, but runtime/status should then treat it as startup-bias
    // compensation rather than a validated temperature model.
    float calibratedTempMinC = 0.0f;
    float calibratedTempMaxC = 0.0f;
    float fitQuality = 0.0f;
    float fitResidualBeforeDps = 0.0f;
    float fitResidualAfterDps = 0.0f;

    // Temperature calibration range is treated as a confidence ramp, not as
    // a hard runtime boundary.  Small extrapolation beyond the fitted range is
    // common during normal warm-up and must not disable tracking or runtime
    // bias learning by itself.
    float softExtrapolationMarginC = 5.0f;
    float hardExtrapolationMarginC = 12.0f;
};

// Lightweight per-sample evaluation used by the 960 Hz tracking hot path.
// It contains only values required for gyro correction and runtime quality
// gates. Human-readable DPS conversions and fit-report metadata remain in the
// full snapshot API and are not recomputed for every IMU sample.
struct GyroTempCompRuntimeEval {
    bool valid = false;
    bool temperatureModelValid = false;
    bool enabled = false;
    bool hasCalibratedRange = false;
    bool tempOutOfRange = false;
    bool tempSoftExtrapolated = false;
    bool tempHardExtrapolated = false;

    float currentTempC = 25.0f;
    Vec3 currentBiasRadS = Vec3::zero();
    float tempDistanceToRangeC = 0.0f;
    float softExtrapolationMarginC = 0.0f;
    float hardExtrapolationMarginC = 0.0f;
    float extrapolationConfidence = 1.0f;
};

struct GyroTempCompSnapshot {
    bool valid = false;
    bool temperatureModelValid = false;
    bool enabled = false;

    float referenceTempC = 25.0f;
    float currentTempC = 25.0f;
    float deltaTempC = 0.0f;

    Vec3 referenceBiasRadS = Vec3::zero();
    Vec3 referenceBiasDps = Vec3::zero();

    Vec3 slopeRadSPerC = Vec3::zero();
    Vec3 slopeDpsPerC = Vec3::zero();

    Vec3 currentBiasRadS = Vec3::zero();
    Vec3 currentBiasDps = Vec3::zero();

    float calibratedTempMinC = 0.0f;
    float calibratedTempMaxC = 0.0f;
    float fitQuality = 0.0f;
    float fitResidualBeforeDps = 0.0f;
    float fitResidualAfterDps = 0.0f;
    bool hasCalibratedRange = false;
    bool tempOutOfRange = false;
    float tempDistanceToRangeC = 0.0f;
    float softExtrapolationMarginC = 0.0f;
    float hardExtrapolationMarginC = 0.0f;
    bool tempSoftExtrapolated = false;
    bool tempHardExtrapolated = false;
    float extrapolationConfidence = 1.0f;
};

class GyroTempCompensator {
public:
    explicit GyroTempCompensator(const GyroTempCompConfig& config = GyroTempCompConfig{});

    // Legacy name retained for source compatibility.  reset() now means
    // "replace the static bias and invalidate any previous temperature model".
    void reset(const Vec3& referenceBiasRadS, float referenceTempC);
    void setStaticBias(const Vec3& referenceBiasRadS, float referenceTempC);
    void clearAll();
    void invalidateTemperatureModel();
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

    void setSlopeDpsPerC(const Vec3& slopeDpsPerC);
    void setSlopeRadSPerC(const Vec3& slopeRadSPerC);

    bool valid() const;
    bool temperatureModelValid() const;

    Vec3 referenceBiasRadS() const;
    Vec3 referenceBiasDps() const;
    float referenceTempC() const;
    Vec3 slopeRadSPerC() const;
    Vec3 slopeDpsPerC() const;
    bool acceptsSlopeDpsPerC(const Vec3& slopeDpsPerC) const;
    bool acceptsSlopeRadSPerC(const Vec3& slopeRadSPerC) const;

    Vec3 biasAt(float tempC) const;
    Vec3 correctedGyro(const Vec3& rawGyroRadS, float tempC) const;

    GyroTempCompRuntimeEval evaluateRuntime(float currentTempC) const;
    GyroTempCompSnapshot snapshot(float currentTempC) const;

private:
    GyroTempCompConfig cfg_;
    bool valid_ = false;
    bool temperatureModelValid_ = false;

    Vec3 referenceBiasRadS_ = Vec3::zero();
    float referenceTempC_ = 25.0f;
    Vec3 slopeRadSPerC_ = Vec3::zero();

    void clearQualityMetadata();
};

} // namespace tracker
