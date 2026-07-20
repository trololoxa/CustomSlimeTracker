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

struct TempFitPoint {
    bool active = true;
    float tempC = 0.0f;
    Vec3 residualDps = Vec3::zero();
    float weight = 0.0f;
    uint32_t goodSamples = 0;
    uint32_t badSamples = 0;
    float residualAfterNormDps = 0.0f;
};

struct TempFitResult {
    bool valid = false;
    Vec3 residualAtRefDps = Vec3::zero();
    Vec3 residualSlopeDpsPerC = Vec3::zero();
    float residualBeforeDps = 0.0f;
    float residualAfterDps = 0.0f;
    float maxResidualAfterDps = 0.0f;
    float tempMinC = 0.0f;
    float tempMaxC = 0.0f;
    float tempRangeC = 0.0f;
    uint32_t bins = 0;
    uint32_t samples = 0;
};

float weightedMeanTemp(const TempFitPoint* points, uint8_t count) {
    double sw = 0.0;
    double st = 0.0;
    for (uint8_t i = 0; i < count; ++i) {
        const TempFitPoint& p = points[i];
        if (!p.active || p.weight <= 0.0f || !std::isfinite(p.tempC)) continue;
        sw += static_cast<double>(p.weight);
        st += static_cast<double>(p.weight) * static_cast<double>(p.tempC);
    }
    return sw > 0.0 ? static_cast<float>(st / sw) : 0.0f;
}

bool solveTempFit(TempFitPoint* points, uint8_t count, float refTempC, TempFitResult& out) {
    WeightedFit1D fitX, fitY, fitZ;
    out = TempFitResult{};
    bool haveTemp = false;

    for (uint8_t i = 0; i < count; ++i) {
        const TempFitPoint& p = points[i];
        if (!p.active || p.weight <= 0.0f || !std::isfinite(p.tempC) || !p.residualDps.isFinite()) continue;

        fitX.push(p.tempC, p.residualDps.x, p.weight);
        fitY.push(p.tempC, p.residualDps.y, p.weight);
        fitZ.push(p.tempC, p.residualDps.z, p.weight);

        out.bins++;
        out.samples += p.goodSamples;
        if (!haveTemp) {
            out.tempMinC = out.tempMaxC = p.tempC;
            haveTemp = true;
        } else {
            if (p.tempC < out.tempMinC) out.tempMinC = p.tempC;
            if (p.tempC > out.tempMaxC) out.tempMaxC = p.tempC;
        }
    }

    if (out.bins < 2 || !haveTemp) return false;
    out.tempRangeC = out.tempMaxC - out.tempMinC;

    float ix = 0.0f, iy = 0.0f, iz = 0.0f;
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    if (!fitX.solve(refTempC, ix, sx) ||
        !fitY.solve(refTempC, iy, sy) ||
        !fitZ.solve(refTempC, iz, sz)) {
        return false;
    }

    out.residualAtRefDps = Vec3(ix, iy, iz);
    out.residualSlopeDpsPerC = Vec3(sx, sy, sz);

    double beforeSq = 0.0;
    double afterSq = 0.0;
    double weightSum = 0.0;
    float maxAfter = 0.0f;

    for (uint8_t i = 0; i < count; ++i) {
        TempFitPoint& p = points[i];
        if (!p.active || p.weight <= 0.0f) continue;
        const Vec3 pred = out.residualAtRefDps + out.residualSlopeDpsPerC * (p.tempC - refTempC);
        const Vec3 after = p.residualDps - pred;
        const float afterNorm = after.norm();
        p.residualAfterNormDps = afterNorm;
        if (afterNorm > maxAfter) maxAfter = afterNorm;

        const double w = static_cast<double>(p.weight);
        beforeSq += w * static_cast<double>(p.residualDps.normSq());
        afterSq += w * static_cast<double>(after.normSq());
        weightSum += w;
    }

    if (weightSum <= 0.0) return false;
    out.residualBeforeDps = static_cast<float>(std::sqrt(beforeSq / weightSum));
    out.residualAfterDps = static_cast<float>(std::sqrt(afterSq / weightSum));
    out.maxResidualAfterDps = maxAfter;
    out.valid = out.residualAtRefDps.isFinite() && out.residualSlopeDpsPerC.isFinite() &&
        std::isfinite(out.residualBeforeDps) && std::isfinite(out.residualAfterDps);
    return out.valid;
}

uint8_t collectTempFitPoints(const StaticRuntimeTest& test,
                             TempFitPoint* points,
                             uint8_t capacity,
                             uint32_t minSamplesPerBin,
                             uint32_t& goodSamples,
                             uint32_t& badSamples) {
    uint8_t count = 0;
    goodSamples = 0;
    badSamples = 0;
    for (uint8_t i = 0; i < STATIC_TEMP_BIN_COUNT && count < capacity; ++i) {
        const StaticTempBinStats& b = test.tempBins[i];
        badSamples += b.badQualitySamples;
        if (b.gyroAfterRadS.count < minSamplesPerBin || b.tempC.count < minSamplesPerBin) continue;

        const float tempMean = b.tempC.mean();
        const Vec3 gyroMeanDps = b.gyroAfterRadS.mean() * MATH_RAD_TO_DEG;
        if (!std::isfinite(tempMean) || !gyroMeanDps.isFinite()) continue;

        TempFitPoint& p = points[count++];
        p.active = true;
        p.tempC = tempMean;
        p.residualDps = gyroMeanDps;
        p.goodSamples = b.gyroAfterRadS.count;
        p.badSamples = b.badQualitySamples;
        // Weight by sqrt(N), not N, so one very dense temperature bin cannot
        // dominate the whole slope.  This is important during warm-up where the
        // tracker may sit for a long time near the final plateau.
        p.weight = std::sqrt(static_cast<float>(p.goodSamples));
        p.residualAfterNormDps = 0.0f;
        goodSamples += p.goodSamples;
    }
    return count;
}

} // namespace gyro_temp_static_fit_detail

bool fitGyroTempFromCompletedStaticTestEx(GyroTempStaticFitDeps& deps,
                                           const StaticRuntimeTest& test,
                                           GyroTempStaticFitMode mode,
                                           Stream& out) {
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

    if (test.samples < 1000 || test.gyroAfterRadS.count < 1000) {
        out.println("# ERR no usable gyro temperature capture data");
        return false;
    }

    using namespace gyro_temp_static_fit_detail;
    TempFitPoint points[STATIC_TEMP_BIN_COUNT];
    uint32_t usableSamples = 0;
    uint32_t badBinSamples = 0;
    constexpr uint32_t kMinSamplesPerBin = 512;
    uint8_t usableBins = collectTempFitPoints(test, points, STATIC_TEMP_BIN_COUNT, kMinSamplesPerBin, usableSamples, badBinSamples);

    const float fitRefTempC = weightedMeanTemp(points, usableBins);
    TempFitResult initial;
    const bool initialOk = solveTempFit(points, usableBins, fitRefTempC, initial);

    out.println("# GYRO TEMP FIT FROM TEMPERATURE BINS");
    out.print("usable_bins="); out.println(usableBins);
    out.print("usable_samples="); out.println(usableSamples);
    out.print("bad_bin_samples="); out.println(badBinSamples);
    out.print("fit_reference_temp_c="); out.println(fitRefTempC, 3);

    if (!initialOk || usableBins < 4 || usableSamples < 10000 || initial.tempRangeC < 3.0f || !std::isfinite(fitRefTempC)) {
        out.print("temp_min_c="); out.println(initial.tempMinC, 3);
        out.print("temp_max_c="); out.println(initial.tempMaxC, 3);
        out.print("temp_range_c="); out.println(initial.tempRangeC, 3);
        out.println("# ERR insufficient temperature coverage for production temp fit");
        return false;
    }

    const float robustThresholdDps = std::fmax(0.018f, initial.residualAfterDps * 2.75f);
    uint8_t inlierBins = 0;
    uint32_t inlierSamples = 0;
    for (uint8_t i = 0; i < usableBins; ++i) {
        TempFitPoint& p = points[i];
        if (!p.active) continue;
        const bool inlier = p.residualAfterNormDps <= robustThresholdDps;
        p.active = inlier;
        if (inlier) {
            inlierBins++;
            inlierSamples += p.goodSamples;
        }
    }

    TempFitResult robust;
    bool robustOk = false;
    if (inlierBins >= 4 && inlierSamples >= 8000) {
        const float robustRefTempC = weightedMeanTemp(points, usableBins);
        robustOk = solveTempFit(points, usableBins, robustRefTempC, robust);
    }

    // If the rejection pass would make the fit underdetermined, keep the
    // initial all-bin fit rather than manufacturing a fragile model from too
    // little data.
    const TempFitResult& fit = robustOk ? robust : initial;
    const float refTempC = robustOk ? weightedMeanTemp(points, usableBins) : fitRefTempC;
    if (!robustOk) {
        inlierBins = usableBins;
        inlierSamples = usableSamples;
        for (uint8_t i = 0; i < usableBins; ++i) points[i].active = true;
    }

    const float tempRangeC = fit.tempRangeC;
    const float inlierRatio = usableSamples > 0
        ? static_cast<float>(inlierSamples) / static_cast<float>(usableSamples)
        : 0.0f;
    const float badSampleRatio = (usableSamples + badBinSamples) > 0
        ? static_cast<float>(badBinSamples) / static_cast<float>(usableSamples + badBinSamples)
        : 1.0f;

    out.print("temp_min_c="); out.println(fit.tempMinC, 3);
    out.print("temp_max_c="); out.println(fit.tempMaxC, 3);
    out.print("temp_range_c="); out.println(tempRangeC, 3);
    out.print("robust_fit="); out.println(robustOk ? "yes" : "no");
    out.print("robust_threshold_dps="); out.println(robustThresholdDps, 8);
    out.print("inlier_bins="); out.println(inlierBins);
    out.print("inlier_samples="); out.println(inlierSamples);
    out.print("inlier_ratio="); out.println(inlierRatio, 6);
    out.print("bad_sample_ratio="); out.println(badSampleRatio, 6);

    const GyroTempCompConfig& cfg = gyroTempComp.config();
    if (std::fabs(fit.residualSlopeDpsPerC.x) > cfg.maxAcceptedSlopeDpsPerC ||
        std::fabs(fit.residualSlopeDpsPerC.y) > cfg.maxAcceptedSlopeDpsPerC ||
        std::fabs(fit.residualSlopeDpsPerC.z) > cfg.maxAcceptedSlopeDpsPerC) {
        out.println("# ERR fitted residual slope exceeds maxAcceptedSlopeDpsPerC");
        out.print("max_accepted_slope_dps_per_c="); out.println(cfg.maxAcceptedSlopeDpsPerC, 6);
        gyro_temp_static_fit_detail::printVec3Line(out, "residual_slope_dps_per_c", fit.residualSlopeDpsPerC, 8);
        return false;
    }

    const float residualBeforeDps = fit.residualBeforeDps;
    const float residualAfterDps = fit.residualAfterDps;
    const float improvement = residualBeforeDps > 1.0e-6f
        ? clampf((residualBeforeDps - residualAfterDps) / residualBeforeDps, 0.0f, 1.0f)
        : 0.0f;
    const float coverageScore = clampf(tempRangeC / 10.0f, 0.0f, 1.0f);
    const float binScore = clampf(static_cast<float>(inlierBins) / 10.0f, 0.0f, 1.0f);
    const float residualScore = 1.0f - clampf(residualAfterDps / 0.060f, 0.0f, 1.0f);
    const float inlierScore = clampf((inlierRatio - 0.70f) / 0.30f, 0.0f, 1.0f);
    const float fitQuality = clampf((0.25f * improvement +
                                     0.25f * residualScore +
                                     0.25f * coverageScore +
                                     0.15f * binScore +
                                     0.10f * inlierScore) * (1.0f - badSampleRatio),
                                    0.0f, 1.0f);

    const Vec3 oldSlopeRadSPerC = gyroTempComp.slopeRadSPerC();
    const Vec3 oldSlopeDpsPerC = oldSlopeRadSPerC * MATH_RAD_TO_DEG;
    const Vec3 newSlopeDpsPerC = oldSlopeDpsPerC + fit.residualSlopeDpsPerC;
    const Vec3 newSlopeRadSPerC = newSlopeDpsPerC * MATH_DEG_TO_RAD;
    const Vec3 oldBiasAtFitRefRadS = gyroTempComp.biasAt(refTempC);
    const Vec3 newReferenceBiasRadS = oldBiasAtFitRefRadS + fit.residualAtRefDps * MATH_DEG_TO_RAD;

    gyro_temp_static_fit_detail::printVec3Line(out, "residual_at_ref_dps", fit.residualAtRefDps, 8);
    gyro_temp_static_fit_detail::printVec3Line(out, "residual_slope_dps_per_c", fit.residualSlopeDpsPerC, 8);
    gyro_temp_static_fit_detail::printVec3Line(out, "old_slope_dps_per_c", oldSlopeDpsPerC, 8);
    gyro_temp_static_fit_detail::printVec3Line(out, "new_slope_dps_per_c", newSlopeDpsPerC, 8);
    gyro_temp_static_fit_detail::printVec3Line(out, "new_reference_bias_dps", newReferenceBiasRadS * MATH_RAD_TO_DEG, 8);
    out.print("residual_before_rms_dps="); out.println(residualBeforeDps, 8);
    out.print("residual_after_rms_dps="); out.println(residualAfterDps, 8);
    out.print("residual_after_max_dps="); out.println(fit.maxResidualAfterDps, 8);
    out.print("fit_improvement_ratio="); out.println(improvement, 6);
    out.print("fit_quality="); out.println(fitQuality, 6);

    const bool residualLowEnough = residualAfterDps <= 0.035f;
    const bool improvesEnough = residualAfterDps < residualBeforeDps * 0.98f || residualLowEnough || residualBeforeDps <= 0.020f;
    const bool goodEnoughToApply = fitQuality >= 0.45f &&
        inlierRatio >= 0.75f &&
        residualAfterDps <= 0.060f &&
        improvesEnough;
    out.print("recommended_save="); out.println(goodEnoughToApply ? "yes" : "no");

    if (!goodEnoughToApply) {
        out.println("# ERR temp fit quality is too low; not applying model");
        return false;
    }

    if (mode == GyroTempStaticFitMode::PreviewOnly) {
        out.println("# OK gyro temperature compensation fit preview only; model was NOT applied");
        out.println("# TIP run: cal temp fit_static save   to apply and save this model");
        return true;
    }

    gyroTempComp.setModel(newReferenceBiasRadS, refTempC, newSlopeRadSPerC);
    gyroTempComp.setEnabled(true);
    gyroTempComp.setQualityMetadata(fit.tempMinC, fit.tempMaxC, fitQuality, residualBeforeDps, residualAfterDps);
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

    if (mode == GyroTempStaticFitMode::ApplyRam) {
        out.println("# OK gyro temperature compensation fitted and applied to RAM");
        return true;
    }

    if (!configStore.save(config)) {
        out.print("# ERR gyro temp fit save failed: ");
        out.println(configStore.lastErrorName());
        return false;
    }

    out.println("# OK gyro temperature compensation fitted and saved");
    return true;
}

bool fitGyroTempFromCompletedStaticTest(GyroTempStaticFitDeps& deps,
                                           const StaticRuntimeTest& test,
                                           bool persist,
                                           Stream& out) {
    return fitGyroTempFromCompletedStaticTestEx(
        deps,
        test,
        persist ? GyroTempStaticFitMode::ApplyAndSave : GyroTempStaticFitMode::PreviewOnly,
        out
    );
}

bool fitGyroTempFromLastStatic(GyroTempStaticFitDeps& deps, bool persist, Stream& out) {
    if (!deps.lastCompletedStaticTestValid || !deps.lastCompletedStaticTest) {
        out.println("# ERR no usable completed static test data; run test static first and let it finish");
        return false;
    }
    return fitGyroTempFromCompletedStaticTest(deps, *deps.lastCompletedStaticTest, persist, out);
}

} // namespace tracker
