#include "runtime/gyro_temp_static_fit.hpp"

#include <cmath>

#include "core/math.hpp"

namespace tracker {


namespace gyro_temp_static_fit_detail {

void printVec3Line(Stream& out, const char* key, const Vec3& v, int decimals) {
    out.print(key);
    out.print('=');
    out.print(v.x, decimals);
    out.print(',');
    out.print(v.y, decimals);
    out.print(',');
    out.println(v.z, decimals);
}

struct WeightedFit1D {
    double sw = 0.0;
    double sx = 0.0;
    double sy = 0.0;
    double sxx = 0.0;
    double sxy = 0.0;

    void push(float x, float y, float w) {
        const double xd = static_cast<double>(x);
        const double yd = static_cast<double>(y);
        const double wd = static_cast<double>(w);

        sw += wd;
        sx += wd * xd;
        sy += wd * yd;
        sxx += wd * xd * xd;
        sxy += wd * xd * yd;
    }

    bool solve(float refX, float& interceptAtRef, float& slope) const {
        if (sw <= 0.0) return false;
        const double denom = sw * sxx - sx * sx;
        if (std::fabs(denom) < 1.0e-9) return false;
        const double m = (sw * sxy - sx * sy) / denom;
        const double b = (sy - m * sx) / sw;
        slope = static_cast<float>(m);
        interceptAtRef = static_cast<float>(b + m * static_cast<double>(refX));
        return std::isfinite(interceptAtRef) && std::isfinite(slope);
    }
};

} // namespace gyro_temp_static_fit_detail

bool fitGyroTempFromLastStatic(GyroTempStaticFitDeps& deps, bool persist, Stream& out) {
    if (!deps.gyroTempComp || !deps.imuCal || !deps.runtimeBias || !deps.config || !deps.configStore) {
        out.println("# ERR gyro temp fit dependencies are not configured");
        return false;
    }

    GyroTempCompensator& gyroTempComp = *deps.gyroTempComp;
    ImuCalibration& imuCal = *deps.imuCal;
    RuntimeGyroBiasEstimator& runtimeBias = *deps.runtimeBias;
    TrackerConfig& config = *deps.config;
    TrackerConfigStore& configStore = *deps.configStore;

    if (!gyroTempComp.valid()) {
        out.println("# ERR gyro temp comp is not valid; run cal gyro first");
        return false;
    }

    if (!deps.lastCompletedStaticTestValid ||
        !deps.lastCompletedStaticTest ||
        deps.lastCompletedStaticTest->samples < 1000 ||
        deps.lastCompletedStaticTest->gyroAfterRadS.count < 1000) {
        out.println("# ERR no usable completed static test data; run test static first and let it finish");
        return false;
    }

    const StaticRuntimeTest& test = *deps.lastCompletedStaticTest;

    using gyro_temp_static_fit_detail::WeightedFit1D;
    WeightedFit1D fitX, fitY, fitZ;

    uint32_t usableBins = 0;
    uint32_t usableSamples = 0;
    uint32_t badBinSamples = 0;
    float tempMinC = 0.0f;
    float tempMaxC = 0.0f;
    bool haveTemp = false;

    for (uint8_t i = 0; i < STATIC_TEMP_BIN_COUNT; ++i) {
        const StaticTempBinStats& b = test.tempBins[i];
        if (b.gyroAfterRadS.count < 512 || b.tempC.count < 512) continue;

        const float tempMean = b.tempC.mean();
        const Vec3 gyroMeanDps = b.gyroAfterRadS.mean() * MATH_RAD_TO_DEG;
        const float w = static_cast<float>(b.gyroAfterRadS.count);

        if (!std::isfinite(tempMean) || !gyroMeanDps.isFinite()) continue;

        fitX.push(tempMean, gyroMeanDps.x, w);
        fitY.push(tempMean, gyroMeanDps.y, w);
        fitZ.push(tempMean, gyroMeanDps.z, w);

        usableBins++;
        usableSamples += b.gyroAfterRadS.count;
        badBinSamples += b.badQualitySamples;

        if (!haveTemp) {
            tempMinC = tempMaxC = tempMean;
            haveTemp = true;
        } else {
            if (tempMean < tempMinC) tempMinC = tempMean;
            if (tempMean > tempMaxC) tempMaxC = tempMean;
        }
    }

    const float tempRangeC = haveTemp ? (tempMaxC - tempMinC) : 0.0f;
    const float fitRefTempC = test.tempC.mean();

    out.println("# GYRO TEMP FIT FROM STATIC BINS");
    out.print("usable_bins="); out.println(usableBins);
    out.print("usable_samples="); out.println(usableSamples);
    out.print("bad_bin_samples="); out.println(badBinSamples);
    out.print("temp_min_c="); out.println(tempMinC, 3);
    out.print("temp_max_c="); out.println(tempMaxC, 3);
    out.print("temp_range_c="); out.println(tempRangeC, 3);
    out.print("fit_reference_temp_c="); out.println(fitRefTempC, 3);

    if (usableBins < 4 || usableSamples < 10000 || tempRangeC < 3.0f || !std::isfinite(fitRefTempC)) {
        out.println("# ERR insufficient temperature coverage for production temp fit");
        return false;
    }

    float ix = 0.0f, iy = 0.0f, iz = 0.0f;
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    if (!fitX.solve(fitRefTempC, ix, sx) ||
        !fitY.solve(fitRefTempC, iy, sy) ||
        !fitZ.solve(fitRefTempC, iz, sz)) {
        out.println("# ERR linear temperature fit failed");
        return false;
    }

    const Vec3 residualAtRefDps(ix, iy, iz);
    const Vec3 residualSlopeDpsPerC(sx, sy, sz);

    const GyroTempCompConfig& cfg = gyroTempComp.config();
    if (std::fabs(residualSlopeDpsPerC.x) > cfg.maxAbsSlopeDpsPerC ||
        std::fabs(residualSlopeDpsPerC.y) > cfg.maxAbsSlopeDpsPerC ||
        std::fabs(residualSlopeDpsPerC.z) > cfg.maxAbsSlopeDpsPerC) {
        out.println("# ERR fitted residual slope exceeds maxAbsSlopeDpsPerC");
        out.print("max_abs_slope_dps_per_c="); out.println(cfg.maxAbsSlopeDpsPerC, 6);
        gyro_temp_static_fit_detail::printVec3Line(out, "residual_slope_dps_per_c", residualSlopeDpsPerC, 8);
        return false;
    }

    double beforeSq = 0.0;
    double afterSq = 0.0;
    double weightSum = 0.0;

    for (uint8_t i = 0; i < STATIC_TEMP_BIN_COUNT; ++i) {
        const StaticTempBinStats& b = test.tempBins[i];
        if (b.gyroAfterRadS.count < 512 || b.tempC.count < 512) continue;
        const float tempMean = b.tempC.mean();
        const Vec3 y = b.gyroAfterRadS.mean() * MATH_RAD_TO_DEG;
        const Vec3 pred = residualAtRefDps + residualSlopeDpsPerC * (tempMean - fitRefTempC);
        const Vec3 after = y - pred;
        const double w = static_cast<double>(b.gyroAfterRadS.count);
        beforeSq += w * static_cast<double>(y.normSq());
        afterSq += w * static_cast<double>(after.normSq());
        weightSum += w;
    }

    const float residualBeforeDps = weightSum > 0.0 ? static_cast<float>(std::sqrt(beforeSq / weightSum)) : 0.0f;
    const float residualAfterDps = weightSum > 0.0 ? static_cast<float>(std::sqrt(afterSq / weightSum)) : 0.0f;
    const float improvement = residualBeforeDps > 1.0e-6f
        ? clampf((residualBeforeDps - residualAfterDps) / residualBeforeDps, 0.0f, 1.0f)
        : 0.0f;
    const float coverageScore = clampf(tempRangeC / 8.0f, 0.0f, 1.0f);
    const float binScore = clampf(static_cast<float>(usableBins) / 8.0f, 0.0f, 1.0f);
    const float qualityPenalty = usableSamples > 0 ? clampf(static_cast<float>(badBinSamples) / static_cast<float>(usableSamples), 0.0f, 1.0f) : 1.0f;
    const float fitQuality = clampf((0.45f * improvement + 0.35f * coverageScore + 0.20f * binScore) * (1.0f - qualityPenalty), 0.0f, 1.0f);

    const Vec3 oldSlopeRadSPerC = gyroTempComp.slopeRadSPerC();
    const Vec3 oldSlopeDpsPerC = oldSlopeRadSPerC * MATH_RAD_TO_DEG;
    const Vec3 newSlopeDpsPerC = oldSlopeDpsPerC + residualSlopeDpsPerC;
    const Vec3 newSlopeRadSPerC = newSlopeDpsPerC * MATH_DEG_TO_RAD;
    const Vec3 oldBiasAtFitRefRadS = gyroTempComp.biasAt(fitRefTempC);
    const Vec3 newReferenceBiasRadS = oldBiasAtFitRefRadS + residualAtRefDps * MATH_DEG_TO_RAD;

    gyro_temp_static_fit_detail::printVec3Line(out, "residual_at_ref_dps", residualAtRefDps, 8);
    gyro_temp_static_fit_detail::printVec3Line(out, "residual_slope_dps_per_c", residualSlopeDpsPerC, 8);
    gyro_temp_static_fit_detail::printVec3Line(out, "old_slope_dps_per_c", oldSlopeDpsPerC, 8);
    gyro_temp_static_fit_detail::printVec3Line(out, "new_slope_dps_per_c", newSlopeDpsPerC, 8);
    gyro_temp_static_fit_detail::printVec3Line(out, "new_reference_bias_dps", newReferenceBiasRadS * MATH_RAD_TO_DEG, 8);
    out.print("residual_before_rms_dps="); out.println(residualBeforeDps, 8);
    out.print("residual_after_rms_dps="); out.println(residualAfterDps, 8);
    out.print("fit_improvement_ratio="); out.println(improvement, 6);
    out.print("fit_quality="); out.println(fitQuality, 6);

    const bool goodEnoughToApply = fitQuality >= 0.45f && residualAfterDps < residualBeforeDps;
    out.print("recommended_save="); out.println(goodEnoughToApply ? "yes" : "no");

    if (!goodEnoughToApply) {
        out.println("# ERR temp fit quality is too low; not applying model");
        return false;
    }

    if (!persist) {
        out.println("# OK gyro temperature compensation fit preview only; model was NOT applied");
        out.println("# TIP run: cal temp fit_static save   to apply and save this model");
        return true;
    }

    gyroTempComp.setModel(newReferenceBiasRadS, fitRefTempC, newSlopeRadSPerC);
    gyroTempComp.setEnabled(true);
    gyroTempComp.setQualityMetadata(tempMinC, tempMaxC, fitQuality, residualBeforeDps, residualAfterDps);
    imuCal.gyroBiasValid = true;
    imuCal.gyroBiasRadS = gyroTempComp.referenceBiasRadS();

    // A new base temperature model invalidates any runtime trim learned against the
    // previous model.  Runtime bias is intentionally RAM-only and must restart clean.
    const bool runtimeBiasWasEnabled = runtimeBias.enabled;
    runtimeBias.runtimeTrimRadS = Vec3::zero();
    runtimeBias.resetCounters();
    runtimeBias.enabled = runtimeBiasWasEnabled;

    config.captureFromGyroTempComp(gyroTempComp);
    config.sanitize();
    config.updateCrc();

    if (!configStore.save(config)) {
        out.print("# ERR gyro temp fit save failed: ");
        out.println(configStore.lastErrorName());
        return false;
    }

    out.println("# OK gyro temperature compensation fitted and saved");
    return true;
}

} // namespace tracker
