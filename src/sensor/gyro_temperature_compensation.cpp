#include "sensor/gyro_temperature_compensation.hpp"

#include <cmath>

namespace tracker {

GyroTempCompensator::GyroTempCompensator(const GyroTempCompConfig& config)
    : cfg_(config) {}

void GyroTempCompensator::clearQualityMetadata() {
    cfg_.calibratedTempMinC = 0.0f;
    cfg_.calibratedTempMaxC = 0.0f;
    cfg_.fitQuality = 0.0f;
    cfg_.fitResidualBeforeDps = 0.0f;
    cfg_.fitResidualAfterDps = 0.0f;
}

void GyroTempCompensator::reset(const Vec3& referenceBiasRadS, float referenceTempC) {
    setStaticBias(referenceBiasRadS, referenceTempC);
}

void GyroTempCompensator::setStaticBias(const Vec3& referenceBiasRadS, float referenceTempC) {
    referenceBiasRadS_ = referenceBiasRadS;
    referenceTempC_ = referenceTempC;
    slopeRadSPerC_ = Vec3::zero();
    valid_ = referenceBiasRadS.isFinite() && std::isfinite(referenceTempC);
    temperatureModelValid_ = false;
    clearQualityMetadata();
    learnAccepted_ = 0;
    learnRejected_ = 0;
}

void GyroTempCompensator::clearAll() {
    valid_ = false;
    temperatureModelValid_ = false;
    referenceBiasRadS_ = Vec3::zero();
    referenceTempC_ = 25.0f;
    slopeRadSPerC_ = Vec3::zero();
    clearQualityMetadata();
    learnAccepted_ = 0;
    learnRejected_ = 0;
}

void GyroTempCompensator::invalidateTemperatureModel() {
    temperatureModelValid_ = false;
    slopeRadSPerC_ = Vec3::zero();
    clearQualityMetadata();
    learnAccepted_ = 0;
    learnRejected_ = 0;
}

void GyroTempCompensator::setModel(const Vec3& referenceBiasRadS,
                                   float referenceTempC,
                                   const Vec3& slopeRadSPerC) {
    referenceBiasRadS_ = referenceBiasRadS;
    referenceTempC_ = referenceTempC;
    slopeRadSPerC_ = slopeRadSPerC;
    valid_ = referenceBiasRadS.isFinite() && std::isfinite(referenceTempC);
    temperatureModelValid_ = valid_ && slopeRadSPerC.isFinite();
    if (!temperatureModelValid_) {
        slopeRadSPerC_ = Vec3::zero();
        clearQualityMetadata();
    }
}

void GyroTempCompensator::adjustReferenceBias(const Vec3& deltaRadS) {
    if (!deltaRadS.isFinite()) return;
    referenceBiasRadS_ += deltaRadS;
    valid_ = true;
}

void GyroTempCompensator::setQualityMetadata(float tempMinC,
                                             float tempMaxC,
                                             float fitQuality,
                                             float residualBeforeDps,
                                             float residualAfterDps) {
    cfg_.calibratedTempMinC = tempMinC;
    cfg_.calibratedTempMaxC = tempMaxC;
    cfg_.fitQuality = fitQuality;
    cfg_.fitResidualBeforeDps = residualBeforeDps;
    cfg_.fitResidualAfterDps = residualAfterDps;
}

void GyroTempCompensator::setConfig(const GyroTempCompConfig& config) {
    cfg_ = config;
}

const GyroTempCompConfig& GyroTempCompensator::config() const {
    return cfg_;
}

void GyroTempCompensator::setEnabled(bool enabled) {
    cfg_.enabled = enabled;
}

void GyroTempCompensator::setLearningEnabled(bool enabled) {
    cfg_.learningEnabled = enabled;
}

void GyroTempCompensator::setSlopeDpsPerC(const Vec3& slopeDpsPerC) {
    setSlopeRadSPerC(slopeDpsPerC * MATH_DEG_TO_RAD);
}

void GyroTempCompensator::setSlopeRadSPerC(const Vec3& slopeRadSPerC) {
    if (!valid_ || !slopeRadSPerC.isFinite()) {
        invalidateTemperatureModel();
        return;
    }
    slopeRadSPerC_ = slopeRadSPerC;
    temperatureModelValid_ = true;
}

bool GyroTempCompensator::valid() const {
    return valid_;
}

bool GyroTempCompensator::temperatureModelValid() const {
    return valid_ && temperatureModelValid_;
}

Vec3 GyroTempCompensator::referenceBiasRadS() const {
    return referenceBiasRadS_;
}

Vec3 GyroTempCompensator::referenceBiasDps() const {
    return referenceBiasRadS_ * MATH_RAD_TO_DEG;
}

float GyroTempCompensator::referenceTempC() const {
    return referenceTempC_;
}

Vec3 GyroTempCompensator::slopeRadSPerC() const {
    return slopeRadSPerC_;
}

Vec3 GyroTempCompensator::slopeDpsPerC() const {
    return slopeRadSPerC_ * MATH_RAD_TO_DEG;
}

Vec3 GyroTempCompensator::biasAt(float tempC) const {
    if (!valid_) {
        return Vec3::zero();
    }

    if (!cfg_.enabled || !temperatureModelValid_) {
        return referenceBiasRadS_;
    }

    const float dT = tempC - referenceTempC_;
    return referenceBiasRadS_ + slopeRadSPerC_ * dT;
}

Vec3 GyroTempCompensator::correctedGyro(const Vec3& rawGyroRadS, float tempC) const {
    return rawGyroRadS - biasAt(tempC);
}

bool GyroTempCompensator::learnFromStationaryMean(const Vec3& rawGyroMeanRadS,
                                                  float tempC,
                                                  bool stationaryGate) {
    if (!valid_ || !temperatureModelValid_ || !cfg_.learningEnabled || !stationaryGate || !rawGyroMeanRadS.isFinite()) {
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

GyroTempCompSnapshot GyroTempCompensator::snapshot(float currentTempC) const {
    GyroTempCompSnapshot s;
    s.valid = valid_;
    s.temperatureModelValid = temperatureModelValid();
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
    s.calibratedTempMinC = cfg_.calibratedTempMinC;
    s.calibratedTempMaxC = cfg_.calibratedTempMaxC;
    s.fitQuality = cfg_.fitQuality;
    s.fitResidualBeforeDps = cfg_.fitResidualBeforeDps;
    s.fitResidualAfterDps = cfg_.fitResidualAfterDps;
    s.softExtrapolationMarginC = cfg_.softExtrapolationMarginC;
    s.hardExtrapolationMarginC = cfg_.hardExtrapolationMarginC;
    s.hasCalibratedRange = s.temperatureModelValid && cfg_.calibratedTempMaxC > cfg_.calibratedTempMinC;
    if (s.hasCalibratedRange && std::isfinite(currentTempC)) {
        if (currentTempC < cfg_.calibratedTempMinC) {
            s.tempDistanceToRangeC = cfg_.calibratedTempMinC - currentTempC;
        } else if (currentTempC > cfg_.calibratedTempMaxC) {
            s.tempDistanceToRangeC = currentTempC - cfg_.calibratedTempMaxC;
        }
    }
    s.tempOutOfRange = s.hasCalibratedRange && s.tempDistanceToRangeC > 0.0f;

    const float softMarginC = std::isfinite(cfg_.softExtrapolationMarginC) && cfg_.softExtrapolationMarginC > 0.0f
        ? cfg_.softExtrapolationMarginC
        : 0.0f;
    float hardMarginC = std::isfinite(cfg_.hardExtrapolationMarginC) && cfg_.hardExtrapolationMarginC > softMarginC
        ? cfg_.hardExtrapolationMarginC
        : softMarginC;

    if (!s.tempOutOfRange) {
        s.extrapolationConfidence = 1.0f;
    } else if (softMarginC > 0.0f && s.tempDistanceToRangeC <= softMarginC) {
        s.tempSoftExtrapolated = true;
        const float u = clampf(s.tempDistanceToRangeC / softMarginC, 0.0f, 1.0f);
        s.extrapolationConfidence = 1.0f - 0.20f * u;
    } else if (hardMarginC > softMarginC && s.tempDistanceToRangeC <= hardMarginC) {
        s.tempSoftExtrapolated = true;
        const float u = clampf((s.tempDistanceToRangeC - softMarginC) / (hardMarginC - softMarginC), 0.0f, 1.0f);
        s.extrapolationConfidence = 0.80f - 0.45f * u;
    } else {
        s.tempHardExtrapolated = true;
        s.extrapolationConfidence = 0.20f;
    }
    return s;
}

} // namespace tracker
