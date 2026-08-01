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
}

void GyroTempCompensator::clearAll() {
    valid_ = false;
    temperatureModelValid_ = false;
    referenceBiasRadS_ = Vec3::zero();
    referenceTempC_ = 25.0f;
    slopeRadSPerC_ = Vec3::zero();
    clearQualityMetadata();
}

void GyroTempCompensator::invalidateTemperatureModel() {
    temperatureModelValid_ = false;
    slopeRadSPerC_ = Vec3::zero();
    clearQualityMetadata();
}

void GyroTempCompensator::setModel(const Vec3& referenceBiasRadS,
                                   float referenceTempC,
                                   const Vec3& slopeRadSPerC) {
    referenceBiasRadS_ = referenceBiasRadS;
    referenceTempC_ = referenceTempC;
    slopeRadSPerC_ = slopeRadSPerC;
    valid_ = referenceBiasRadS.isFinite() && std::isfinite(referenceTempC);
    temperatureModelValid_ = valid_ && acceptsSlopeRadSPerC(slopeRadSPerC);
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

void GyroTempCompensator::setSlopeDpsPerC(const Vec3& slopeDpsPerC) {
    setSlopeRadSPerC(slopeDpsPerC * MATH_DEG_TO_RAD);
}

void GyroTempCompensator::setSlopeRadSPerC(const Vec3& slopeRadSPerC) {
    if (!valid_ || !acceptsSlopeRadSPerC(slopeRadSPerC)) {
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

bool GyroTempCompensator::acceptsSlopeDpsPerC(const Vec3& slopeDpsPerC) const {
    const float limit = cfg_.maxAcceptedSlopeDpsPerC;
    if (!slopeDpsPerC.isFinite() || !std::isfinite(limit) || limit <= 0.0f) {
        return false;
    }
    return std::fabs(slopeDpsPerC.x) <= limit &&
        std::fabs(slopeDpsPerC.y) <= limit &&
        std::fabs(slopeDpsPerC.z) <= limit;
}

bool GyroTempCompensator::acceptsSlopeRadSPerC(const Vec3& slopeRadSPerC) const {
    return slopeRadSPerC.isFinite() &&
        acceptsSlopeDpsPerC(slopeRadSPerC * MATH_RAD_TO_DEG);
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

GyroTempCompRuntimeEval GyroTempCompensator::evaluateRuntime(float currentTempC) const {
    GyroTempCompRuntimeEval eval;
    eval.valid = valid_;
    eval.temperatureModelValid = temperatureModelValid();
    eval.enabled = cfg_.enabled;
    eval.currentTempC = currentTempC;
    eval.currentBiasRadS = biasAt(currentTempC);
    eval.softExtrapolationMarginC = cfg_.softExtrapolationMarginC;
    eval.hardExtrapolationMarginC = cfg_.hardExtrapolationMarginC;
    eval.hasCalibratedRange = eval.temperatureModelValid &&
        cfg_.calibratedTempMaxC > cfg_.calibratedTempMinC;

    if (eval.hasCalibratedRange && std::isfinite(currentTempC)) {
        if (currentTempC < cfg_.calibratedTempMinC) {
            eval.tempDistanceToRangeC = cfg_.calibratedTempMinC - currentTempC;
        } else if (currentTempC > cfg_.calibratedTempMaxC) {
            eval.tempDistanceToRangeC = currentTempC - cfg_.calibratedTempMaxC;
        }
    }
    eval.tempOutOfRange = eval.hasCalibratedRange &&
        eval.tempDistanceToRangeC > 0.0f;

    const float softMarginC =
        std::isfinite(cfg_.softExtrapolationMarginC) &&
        cfg_.softExtrapolationMarginC > 0.0f
            ? cfg_.softExtrapolationMarginC
            : 0.0f;
    const float hardMarginC =
        std::isfinite(cfg_.hardExtrapolationMarginC) &&
        cfg_.hardExtrapolationMarginC > softMarginC
            ? cfg_.hardExtrapolationMarginC
            : softMarginC;

    if (!eval.tempOutOfRange) {
        eval.extrapolationConfidence = 1.0f;
    } else if (softMarginC > 0.0f &&
               eval.tempDistanceToRangeC <= softMarginC) {
        eval.tempSoftExtrapolated = true;
        const float u = clampf(eval.tempDistanceToRangeC / softMarginC, 0.0f, 1.0f);
        eval.extrapolationConfidence = 1.0f - 0.20f * u;
    } else if (hardMarginC > softMarginC &&
               eval.tempDistanceToRangeC <= hardMarginC) {
        eval.tempSoftExtrapolated = true;
        const float u = clampf(
            (eval.tempDistanceToRangeC - softMarginC) / (hardMarginC - softMarginC),
            0.0f,
            1.0f);
        eval.extrapolationConfidence = 0.80f - 0.45f * u;
    } else {
        eval.tempHardExtrapolated = true;
        eval.extrapolationConfidence = 0.20f;
    }
    return eval;
}

GyroTempCompSnapshot GyroTempCompensator::snapshot(float currentTempC) const {
    const GyroTempCompRuntimeEval eval = evaluateRuntime(currentTempC);
    GyroTempCompSnapshot s;
    s.valid = eval.valid;
    s.temperatureModelValid = eval.temperatureModelValid;
    s.enabled = eval.enabled;
    s.referenceTempC = referenceTempC_;
    s.currentTempC = currentTempC;
    s.deltaTempC = currentTempC - referenceTempC_;
    s.referenceBiasRadS = referenceBiasRadS_;
    s.referenceBiasDps = referenceBiasDps();
    s.slopeRadSPerC = slopeRadSPerC_;
    s.slopeDpsPerC = slopeDpsPerC();
    s.currentBiasRadS = eval.currentBiasRadS;
    s.currentBiasDps = s.currentBiasRadS * MATH_RAD_TO_DEG;
    s.calibratedTempMinC = cfg_.calibratedTempMinC;
    s.calibratedTempMaxC = cfg_.calibratedTempMaxC;
    s.fitQuality = cfg_.fitQuality;
    s.fitResidualBeforeDps = cfg_.fitResidualBeforeDps;
    s.fitResidualAfterDps = cfg_.fitResidualAfterDps;
    s.softExtrapolationMarginC = eval.softExtrapolationMarginC;
    s.hardExtrapolationMarginC = eval.hardExtrapolationMarginC;
    s.hasCalibratedRange = eval.hasCalibratedRange;
    s.tempOutOfRange = eval.tempOutOfRange;
    s.tempDistanceToRangeC = eval.tempDistanceToRangeC;
    s.tempSoftExtrapolated = eval.tempSoftExtrapolated;
    s.tempHardExtrapolated = eval.tempHardExtrapolated;
    s.extrapolationConfidence = eval.extrapolationConfidence;
    return s;
}

} // namespace tracker
