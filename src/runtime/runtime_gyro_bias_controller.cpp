#include "runtime/runtime_gyro_bias_controller.hpp"

#include <cmath>

namespace tracker {
namespace runtime_bias_detail {

void printU64Dec(Stream& out, uint64_t v) {
    char buf[21];
    size_t i = sizeof(buf);
    buf[--i] = '\0';
    if (v == 0) {
        buf[--i] = '0';
    } else {
        while (v > 0 && i > 0) {
            buf[--i] = static_cast<char>('0' + (v % 10));
            v /= 10;
        }
    }
    out.print(&buf[i]);
}

void printVec3Line(Stream& out, const char* label, const Vec3& v, uint8_t decimals = 6) {
    out.print(label);
    out.print('=');
    out.print(v.x, decimals);
    out.print(',');
    out.print(v.y, decimals);
    out.print(',');
    out.println(v.z, decimals);
}

} // namespace runtime_bias_detail

bool runtimeBiasHasBaseGyroBiasModel(const ImuCalibration& imuCal,
                                            const GyroTempCompensator& gyroTempComp) {
    return gyroTempComp.valid() || imuCal.gyroBiasValid;
}

Vec3 runtimeBiasBaseGyroBiasRadS(const ImuCalibration& imuCal,
                                        const GyroTempCompensator& gyroTempComp,
                                        float tempC) {
    if (gyroTempComp.valid()) return gyroTempComp.biasAt(tempC);
    if (imuCal.gyroBiasValid) return imuCal.gyroBiasRadS;
    return Vec3::zero();
}

Vec3 runtimeBiasCurrentGyroBiasRadS(const RuntimeGyroBiasEstimator& bias,
                                           const ImuCalibration& imuCal,
                                           const GyroTempCompensator& gyroTempComp,
                                           float tempC) {
    return runtimeBiasCurrentGyroBiasRadS(
        bias, imuCal, gyroTempComp.evaluateRuntime(tempC));
}

Vec3 runtimeBiasCurrentGyroBiasRadS(const RuntimeGyroBiasEstimator& bias,
                                           const ImuCalibration& imuCal,
                                           const GyroTempCompRuntimeEval& tempEval) {
    Vec3 out = tempEval.valid
        ? tempEval.currentBiasRadS
        : (imuCal.gyroBiasValid ? imuCal.gyroBiasRadS : Vec3::zero());
    if (bias.runtimeTrimRadS.isFinite()) {
        out += bias.runtimeTrimRadS;
    }
    return out;
}

uint32_t runtimeBiasGyroBiasRuntimeFlags(const RuntimeGyroBiasEstimator& bias,
                                                const GyroTempCompensator& gyroTempComp,
                                                float tempC) {
    const GyroTempCompSnapshot s = gyroTempComp.snapshot(tempC);
    uint32_t flags = 0;
    if (s.valid) flags |= 1u << 0;
    if (s.enabled) flags |= 1u << 1;
    if (s.hasCalibratedRange) flags |= 1u << 2;
    if (s.tempOutOfRange) flags |= 1u << 3;
    if (bias.enabled) flags |= 1u << 4;
    if (bias.runtimeTrimRadS.norm() > (0.00001f * MATH_DEG_TO_RAD)) flags |= 1u << 5;
    if (bias.dryRun) flags |= 1u << 6;
    return flags;
}

void runtimeBiasApplyGyroTempQualityFlags(const GyroTempCompensator& gyroTempComp,
                                                 ImuQualityResult& quality,
                                                 float tempC) {
    runtimeBiasApplyGyroTempQualityFlags(
        gyroTempComp.evaluateRuntime(tempC), quality);
}

void runtimeBiasApplyGyroTempQualityFlags(const GyroTempCompRuntimeEval& tempEval,
                                                 ImuQualityResult& quality) {
    if (tempEval.valid && tempEval.enabled && tempEval.hasCalibratedRange &&
        tempEval.tempOutOfRange) {
        // The calibrated temperature interval is a confidence hint, not a
        // hard runtime cliff. Just outside the fitted range, keep tracking
        // confidence almost unchanged; degrade more only for large
        // extrapolation.
        const float multiplier = clampf(
            tempEval.extrapolationConfidence, 0.35f, 0.98f);
        quality.markTempCompOutOfRange(multiplier);
    }
}


RuntimeBiasTempGate runtimeBiasTempGateFor(const RuntimeGyroBiasEstimator& bias,
                                                  const GyroTempCompensator& gyroTempComp,
                                                  float tempC) {
    return runtimeBiasTempGateFor(bias, gyroTempComp.evaluateRuntime(tempC));
}

RuntimeBiasTempGate runtimeBiasTempGateFor(const RuntimeGyroBiasEstimator& bias,
                                                  const GyroTempCompRuntimeEval& tempEval) {
    RuntimeBiasTempGate gate;

    // Temperature range is meaningful only for an active validated temp model.
    // If temp compensation is disabled, runtime bias trim can still operate on
    // top of the plain gyro bias model using the normal stationary gates.
    gate.rangeRelevant = tempEval.valid && tempEval.enabled &&
        tempEval.hasCalibratedRange;
    if (!gate.rangeRelevant || !std::isfinite(tempEval.currentTempC)) {
        return gate;
    }

    gate.outOfRange = tempEval.tempOutOfRange;
    gate.distanceToRangeC = tempEval.tempDistanceToRangeC;
    if (!gate.outOfRange) {
        return gate;
    }

    const float configuredMarginC =
        std::isfinite(bias.tempExtrapolationMarginC) &&
        bias.tempExtrapolationMarginC >= 0.0f
            ? bias.tempExtrapolationMarginC
            : 0.0f;
    const float softMarginC =
        std::isfinite(tempEval.softExtrapolationMarginC) &&
        tempEval.softExtrapolationMarginC > 0.0f
            ? tempEval.softExtrapolationMarginC
            : 0.0f;
    const float cautiousMarginC = configuredMarginC > softMarginC
        ? configuredMarginC
        : softMarginC;
    const float hardMarginC =
        std::isfinite(tempEval.hardExtrapolationMarginC) &&
        tempEval.hardExtrapolationMarginC > cautiousMarginC
            ? tempEval.hardExtrapolationMarginC
            : cautiousMarginC;

    const float baseScale = clampf(bias.outOfRangeGainScale, 0.01f, 1.0f);
    gate.gainScale = clampf(
        baseScale * clampf(tempEval.extrapolationConfidence, 0.20f, 1.0f),
        0.01f,
        1.0f);

    if (bias.allowOutOfRangeEstimator &&
        gate.distanceToRangeC <= cautiousMarginC) {
        gate.nearOutOfRange = true;
    } else if (bias.allowOutOfRangeEstimator &&
               gate.distanceToRangeC <= hardMarginC) {
        gate.nearOutOfRange = true;
        gate.farOutOfRange = true;
        gate.gainScale = clampf(gate.gainScale * 0.5f, 0.01f, 1.0f);
    } else {
        gate.farOutOfRange = true;
        gate.gainScale = clampf(gate.gainScale * 0.25f, 0.01f, 1.0f);
        gate.reject = bias.requireTempCompRange;
    }

    return gate;
}

uint32_t runtimeBiasDecisionFlags(bool accepted,
                                         bool badTiming,
                                         bool saturated,
                                         bool calibrationBad,
                                         bool tempBad,
                                         bool motionBad,
                                         bool accelBad,
                                         bool finiteBad,
                                         bool priming,
                                         bool dryRun,
                                         bool tempCautious) {
    uint32_t flags = 0;
    if (accepted) flags |= 1u << 0;
    if (badTiming) flags |= 1u << 1;
    if (saturated) flags |= 1u << 2;
    if (calibrationBad) flags |= 1u << 3;
    if (tempBad) flags |= 1u << 4;
    if (motionBad) flags |= 1u << 5;
    if (accelBad) flags |= 1u << 6;
    if (finiteBad) flags |= 1u << 7;
    if (priming) flags |= 1u << 8;
    if (dryRun) flags |= 1u << 9;
    if (tempCautious) flags |= 1u << 10;
    return flags;
}


void emitRuntimeBiasUpdateLog(const RuntimeGyroBiasUpdateDeps& deps,
                                     uint64_t tUs,
                                     float tempC,
                                     const Vec3& residualDps,
                                     const Vec3& stdDps,
                                     const Vec3& deltaDps,
                                     const Vec3& trimDps,
                                     uint32_t flags) {
    if (!deps.logEnabled || deps.logSequence == nullptr || deps.logStream == nullptr) return;
    Stream& out = *deps.logStream;
    constexpr int RUNTIME_BIAS_LOG_RESERVE_BYTES = 640;
    if (out.availableForWrite() < RUNTIME_BIAS_LOG_RESERVE_BYTES) {
        if (deps.logCounters != nullptr) ++deps.logCounters->backpressureDrop;
        return;
    }
    const uint32_t seq = (*deps.logSequence)++;
    out.print("BIASUPD,"); runtime_bias_detail::printU64Dec(out, tUs);
    out.print(','); out.print(seq);
    out.print(','); out.print(tempC, 3);
    out.print(','); out.print(residualDps.x, 8);
    out.print(','); out.print(residualDps.y, 8);
    out.print(','); out.print(residualDps.z, 8);
    out.print(','); out.print(stdDps.x, 8);
    out.print(','); out.print(stdDps.y, 8);
    out.print(','); out.print(stdDps.z, 8);
    out.print(','); out.print(deltaDps.x, 8);
    out.print(','); out.print(deltaDps.y, 8);
    out.print(','); out.print(deltaDps.z, 8);
    out.print(','); out.print(trimDps.x, 8);
    out.print(','); out.print(trimDps.y, 8);
    out.print(','); out.print(trimDps.z, 8);
    out.print(",0x"); out.println(flags, HEX);
    if (deps.logCounters != nullptr) {
        deps.logCounters->biasUpdate++;
    }
}

Vec3 clampRuntimeTrimDps(const RuntimeGyroBiasEstimator& bias, const Vec3& trimDps) {
    const float limit = bias.maxRuntimeTrimDps;
    return Vec3(
        clampf(trimDps.x, -limit, limit),
        clampf(trimDps.y, -limit, limit),
        clampf(trimDps.z, -limit, limit)
    );
}

bool runtimeBiasUpdateEstimator(const RuntimeGyroBiasUpdateDeps& deps,
                                const Lsm6dsv::Sample& scaled,
                                const Lsm6dsv::Sample& calibrated,
                                const ImuQualityResult& quality,
                                uint64_t timestampUs) {
    const GyroTempCompRuntimeEval tempEval =
        deps.gyroTempComp.evaluateRuntime(calibrated.temp_c);
    const Vec3 currentGyroBiasRadS = runtimeBiasCurrentGyroBiasRadS(
        deps.bias, deps.imuCal, tempEval);
    return runtimeBiasUpdateEstimator(
        deps, scaled, calibrated, quality, timestampUs,
        tempEval, currentGyroBiasRadS);
}

bool runtimeBiasUpdateEstimator(const RuntimeGyroBiasUpdateDeps& deps,
                                const Lsm6dsv::Sample& scaled,
                                const Lsm6dsv::Sample& calibrated,
                                const ImuQualityResult& quality,
                                uint64_t timestampUs,
                                const GyroTempCompRuntimeEval& tempEval,
                                const Vec3& currentGyroBiasRadS) {
    RuntimeGyroBiasEstimator& bias = deps.bias;
    if (!bias.enabled) return false;

    const bool calibrationBad = !runtimeBiasHasBaseGyroBiasModel(deps.imuCal, deps.gyroTempComp) ||
        (bias.requireAccelCalibration && !deps.imuCal.accelCalValid);

    const RuntimeBiasTempGate sampleTempGate = runtimeBiasTempGateFor(
        bias, tempEval);
    const bool tempBad = sampleTempGate.reject;

    const bool badTiming = deps.trackingRecovering ||
        !quality.shouldUpdateAhrs ||
        quality.shouldRequestFifoRecovery ||
        quality.has(imu_quality_flags::TIMESTAMP_ZERO) ||
        quality.has(imu_quality_flags::TIMESTAMP_NON_MONOTONIC) ||
        quality.has(imu_quality_flags::TIMESTAMP_BACKWARDS) ||
        quality.has(imu_quality_flags::TIMESTAMP_LARGE_GAP) ||
        quality.has(imu_quality_flags::FIFO_OVERRUN) ||
        quality.has(imu_quality_flags::FIFO_FULL) ||
        quality.has(imu_quality_flags::FIFO_UNKNOWN_TAG);

    const bool saturated = quality.has(imu_quality_flags::GYRO_SATURATED) ||
                           quality.has(imu_quality_flags::ACCEL_SATURATED) ||
                           quality.has(imu_quality_flags::GYRO_NEAR_SATURATION) ||
                           quality.has(imu_quality_flags::ACCEL_NEAR_SATURATION);

    const bool finiteBad = !scaled.gyro_rad_s.isFinite() ||
                           !calibrated.gyro_rad_s.isFinite() ||
                           !calibrated.accel_g.isFinite() ||
                           !std::isfinite(calibrated.temp_c);

    if (calibrationBad || tempBad || badTiming || saturated || finiteBad) {
        if (calibrationBad) bias.calibrationRejects++;
        if (tempBad) {
            bias.tempRejects++;
            if (sampleTempGate.farOutOfRange) bias.tempFarRejects++;
        }
        if (badTiming) bias.badTimingRejects++;
        if (saturated) bias.saturationRejects++;
        if (finiteBad) bias.finiteRejects++;
        bias.rejected++;
        bias.consecutiveStationaryWindows = 0;
        bias.lastTempDistanceToRangeC = sampleTempGate.distanceToRangeC;
        bias.lastUpdateGainScale = sampleTempGate.gainScale;
        bias.lastDecisionFlags = runtimeBiasDecisionFlags(
            false, badTiming, saturated, calibrationBad, tempBad,
            false, false, finiteBad, false, false,
            sampleTempGate.nearOutOfRange);
        bias.resetWindow();
        return false;
    }

    const Ahrs6DofStats& ast = deps.ahrs.stats();
    if (sampleTempGate.nearOutOfRange) bias.tempCautiousSamples++;

    // runtimeTrimRadS is persisted only in RAM but shares the same native
    // sensor frame as the static/temperature bias. Derive the residual from
    // the unrotated scaled gyro so a non-identity frame cannot mix axes.
    const Vec3 sensorResidualRadS = scaled.gyro_rad_s - currentGyroBiasRadS;
    bias.calibratedGyroRadS.push(sensorResidualRadS);
    bias.accelNormG.push(quality.accelNormValid
        ? quality.accelNormG
        : calibrated.accel_g.norm());
    bias.accelTrust.push(ast.lastAccelGate.trust);
    bias.tempC.push(calibrated.temp_c);

    if (bias.calibratedGyroRadS.count < bias.windowSamplesRequired) return false;

    if (bias.completedWindowPending) {
        // This should be impossible because the FIFO processor yields at every
        // completed window. Preserve the older evidence and fail closed rather
        // than blocking tracking or overwriting a pending estimator decision.
        ++bias.completedWindowDrops;
        bias.resetWindow();
        return false;
    }

    bias.completedCalibratedGyroRadS = bias.calibratedGyroRadS;
    bias.completedAccelNormG = bias.accelNormG;
    bias.completedAccelTrust = bias.accelTrust;
    bias.completedTempC = bias.tempC;
    bias.completedTimestampUs = timestampUs;
    bias.completedWindowPending = true;
    ++bias.windowsDeferred;
    bias.resetWindow();
    return true;
}

bool runtimeBiasFinalizePendingWindow(const RuntimeGyroBiasUpdateDeps& deps) {
    RuntimeGyroBiasEstimator& bias = deps.bias;
    if (!bias.completedWindowPending) return false;

    const Vec3Stats& gyroStats = bias.completedCalibratedGyroRadS;
    const ScalarStats& accelNormStats = bias.completedAccelNormG;
    const ScalarStats& accelTrustStats = bias.completedAccelTrust;
    const ScalarStats& tempStats = bias.completedTempC;
    const uint64_t timestampUs = bias.completedTimestampUs;

    bias.windows++;

    const Vec3 meanGyroDps = gyroStats.mean() * MATH_RAD_TO_DEG;
    const Vec3 stdGyroDps = gyroStats.stddev() * MATH_RAD_TO_DEG;
    const float gyroMeanNormDps = meanGyroDps.norm();
    const float gyroStdNormDps = stdGyroDps.norm();
    float gyroStdAxisMax = stdGyroDps.x;
    if (stdGyroDps.y > gyroStdAxisMax) gyroStdAxisMax = stdGyroDps.y;
    if (stdGyroDps.z > gyroStdAxisMax) gyroStdAxisMax = stdGyroDps.z;
    const float accelMeanG = accelNormStats.mean();
    const float accelMeanErrG = std::fabs(accelMeanG - 1.0f);
    const float accelStdG = accelNormStats.stddev();
    const float accelTrustMean = accelTrustStats.mean();
    const float tempMin = tempStats.minValue;
    const float tempMax = tempStats.maxValue;
    const float tempSpanC = tempMax - tempMin;

    bias.lastResidualDps = meanGyroDps;
    bias.lastGyroStdDps = stdGyroDps;
    bias.lastAccelNormMeanG = accelMeanG;
    bias.lastAccelNormStdG = accelStdG;
    bias.lastAccelTrustMean = accelTrustMean;
    bias.lastTempC = tempStats.mean();
    bias.lastTempSpanC = tempSpanC;
    const RuntimeBiasTempGate windowTempGate = runtimeBiasTempGateFor(
        bias, deps.gyroTempComp, bias.lastTempC);
    const bool tempCautious = windowTempGate.nearOutOfRange;
    bias.lastTempDistanceToRangeC = windowTempGate.distanceToRangeC;
    bias.lastUpdateGainScale = windowTempGate.gainScale;

    const bool motionBad = gyroMeanNormDps > bias.gyroMeanMaxDps ||
        gyroStdNormDps > bias.gyroStdNormMaxDps ||
        gyroStdAxisMax > bias.gyroStdAxisMaxDps;

    const bool accelBad = accelMeanErrG > bias.accelNormMeanMaxErrG ||
        accelStdG > bias.accelNormStdMaxG ||
        accelTrustMean < bias.accelTrustMin;

    const bool windowTempBad = tempSpanC > bias.maxWindowTempDeltaC;

    if (motionBad || accelBad || windowTempBad) {
        if (motionBad) bias.motionRejects++;
        if (accelBad) bias.accelRejects++;
        if (windowTempBad) bias.tempRejects++;
        bias.rejected++;
        bias.consecutiveStationaryWindows = 0;
        bias.lastDecisionFlags = runtimeBiasDecisionFlags(
            false, false, false, false, windowTempBad, motionBad,
            accelBad, false, false, false, tempCautious);
        bias.resetCompletedWindow();
        return true;
    }

    if (tempCautious) bias.tempCautiousWindows++;

    bias.stationaryWindows++;
    if (bias.consecutiveStationaryWindows < 255) {
        bias.consecutiveStationaryWindows++;
    }

    const bool priming =
        bias.consecutiveStationaryWindows < bias.stationaryWindowsBeforeUpdate;
    if (priming) {
        bias.primingWindows++;
        bias.lastAppliedDeltaDps = Vec3::zero();
        bias.lastDecisionFlags = runtimeBiasDecisionFlags(
            false, false, false, false, false, false, false,
            false, true, bias.dryRun, tempCautious);
        emitRuntimeBiasUpdateLog(
            deps, timestampUs, bias.lastTempC, meanGyroDps, stdGyroDps,
            Vec3::zero(), bias.runtimeTrimRadS * MATH_RAD_TO_DEG,
            bias.lastDecisionFlags);
        bias.resetCompletedWindow();
        return true;
    }

    const float effectiveGainScale = clampf(bias.lastUpdateGainScale, 0.01f, 1.0f);
    Vec3 deltaDps = meanGyroDps * (bias.updateAlpha * effectiveGainScale);
    const float maxStep = bias.maxUpdateStepDps * effectiveGainScale;
    deltaDps.x = clampf(deltaDps.x, -maxStep, maxStep);
    deltaDps.y = clampf(deltaDps.y, -maxStep, maxStep);
    deltaDps.z = clampf(deltaDps.z, -maxStep, maxStep);

    const Vec3 oldTrimDps = bias.runtimeTrimRadS * MATH_RAD_TO_DEG;
    const Vec3 newTrimDps = clampRuntimeTrimDps(bias, oldTrimDps + deltaDps);
    const Vec3 appliedDeltaDps = newTrimDps - oldTrimDps;

    if (!bias.dryRun) {
        bias.runtimeTrimRadS = newTrimDps * MATH_DEG_TO_RAD;
        bias.accepted++;
        bias.updates++;
    } else {
        bias.dryRunUpdates++;
    }

    bias.lastAppliedDeltaDps = appliedDeltaDps;
    bias.lastUpdateMs = millis();
    bias.lastDecisionFlags = runtimeBiasDecisionFlags(
        !bias.dryRun, false, false, false, false, false, false,
        false, false, bias.dryRun, tempCautious);
    emitRuntimeBiasUpdateLog(
        deps, timestampUs, bias.lastTempC, meanGyroDps, stdGyroDps,
        appliedDeltaDps, newTrimDps, bias.lastDecisionFlags);
    bias.resetCompletedWindow();
    return true;
}

void runtimeBiasPrintStatus(Stream& out,
                                   const RuntimeGyroBiasEstimator& bias,
                                   const ImuCalibration& imuCal,
                                   const GyroTempCompensator& gyroTempComp,
                                   float latestTempC) {
    out.println("# RUNTIME GYRO BIAS");
    out.print("enabled="); out.println(bias.enabled ? "yes" : "no");
    out.print("dry_run="); out.println(bias.dryRun ? "yes" : "no");
    out.print("require_accel_calibration="); out.println(bias.requireAccelCalibration ? "yes" : "no");
    out.print("require_temp_comp_range="); out.println(bias.requireTempCompRange ? "yes" : "no");
    out.print("allow_out_of_range_estimator="); out.println(bias.allowOutOfRangeEstimator ? "yes" : "no");
    out.print("window_samples_required="); out.println(bias.windowSamplesRequired);
    out.print("current_window_samples="); out.println(bias.calibratedGyroRadS.count);
    out.print("stationary_windows_before_update="); out.println(bias.stationaryWindowsBeforeUpdate);
    out.print("gyro_mean_max_dps="); out.println(bias.gyroMeanMaxDps, 6);
    out.print("gyro_std_norm_max_dps="); out.println(bias.gyroStdNormMaxDps, 6);
    out.print("gyro_std_axis_max_dps="); out.println(bias.gyroStdAxisMaxDps, 6);
    out.print("accel_norm_mean_max_err_g="); out.println(bias.accelNormMeanMaxErrG, 6);
    out.print("accel_norm_std_max_g="); out.println(bias.accelNormStdMaxG, 6);
    out.print("accel_trust_min="); out.println(bias.accelTrustMin, 6);
    out.print("max_window_temp_delta_c="); out.println(bias.maxWindowTempDeltaC, 6);
    out.print("temp_extrapolation_margin_c="); out.println(bias.tempExtrapolationMarginC, 6);
    out.print("out_of_range_gain_scale="); out.println(bias.outOfRangeGainScale, 6);
    out.print("update_alpha="); out.println(bias.updateAlpha, 6);
    out.print("max_update_step_dps="); out.println(bias.maxUpdateStepDps, 6);
    out.print("max_runtime_trim_dps="); out.println(bias.maxRuntimeTrimDps, 6);
    out.print("windows="); out.println(bias.windows);
    out.print("windows_deferred="); out.println(bias.windowsDeferred);
    out.print("completed_window_pending="); out.println(bias.completedWindowPending ? "yes" : "no");
    out.print("completed_window_drops="); out.println(bias.completedWindowDrops);
    out.print("stationary_windows="); out.println(bias.stationaryWindows);
    out.print("priming_windows="); out.println(bias.primingWindows);
    out.print("consecutive_stationary_windows="); out.println(bias.consecutiveStationaryWindows);
    out.print("accepted="); out.println(bias.accepted);
    out.print("rejected="); out.println(bias.rejected);
    out.print("bad_timing_rejects="); out.println(bias.badTimingRejects);
    out.print("motion_rejects="); out.println(bias.motionRejects);
    out.print("accel_rejects="); out.println(bias.accelRejects);
    out.print("saturation_rejects="); out.println(bias.saturationRejects);
    out.print("temp_rejects="); out.println(bias.tempRejects);
    out.print("temp_cautious_samples="); out.println(bias.tempCautiousSamples);
    out.print("temp_cautious_windows="); out.println(bias.tempCautiousWindows);
    out.print("temp_far_rejects="); out.println(bias.tempFarRejects);
    out.print("calibration_rejects="); out.println(bias.calibrationRejects);
    out.print("finite_rejects="); out.println(bias.finiteRejects);
    out.print("updates="); out.println(bias.updates);
    out.print("dry_run_updates="); out.println(bias.dryRunUpdates);
    out.print("last_decision_flags=0x"); out.println(bias.lastDecisionFlags, HEX);
    runtime_bias_detail::printVec3Line(out, "last_residual_dps", bias.lastResidualDps, 8);
    runtime_bias_detail::printVec3Line(out, "last_gyro_std_dps", bias.lastGyroStdDps, 8);
    runtime_bias_detail::printVec3Line(out, "last_applied_delta_dps", bias.lastAppliedDeltaDps, 8);
    out.print("last_accel_norm_mean_g="); out.println(bias.lastAccelNormMeanG, 6);
    out.print("last_accel_norm_std_g="); out.println(bias.lastAccelNormStdG, 6);
    out.print("last_accel_trust_mean="); out.println(bias.lastAccelTrustMean, 6);
    out.print("last_temp_c="); out.println(bias.lastTempC, 3);
    out.print("last_temp_span_c="); out.println(bias.lastTempSpanC, 6);
    out.print("last_temp_distance_to_range_c="); out.println(bias.lastTempDistanceToRangeC, 6);
    out.print("last_update_gain_scale="); out.println(bias.lastUpdateGainScale, 6);
    out.print("last_update_age_s="); out.println(bias.lastUpdateMs == 0 ? 0UL : (millis() - bias.lastUpdateMs) / 1000UL);
    runtime_bias_detail::printVec3Line(out, "base_bias_dps", runtimeBiasBaseGyroBiasRadS(imuCal, gyroTempComp, latestTempC) * MATH_RAD_TO_DEG, 8);
    runtime_bias_detail::printVec3Line(out, "runtime_trim_dps", bias.runtimeTrimRadS * MATH_RAD_TO_DEG, 8);
    runtime_bias_detail::printVec3Line(out, "current_bias_dps", runtimeBiasCurrentGyroBiasRadS(bias, imuCal, gyroTempComp, latestTempC) * MATH_RAD_TO_DEG, 8);
}

void runtimeBiasSetEnabled(RuntimeGyroBiasEstimator& bias, bool enabled) {
    bias.enabled = enabled;
    bias.consecutiveStationaryWindows = 0;
    bias.resetWindow();
}

void runtimeBiasReset(RuntimeGyroBiasEstimator& bias) {
    const bool wasEnabled = bias.enabled;
    const bool wasDryRun = bias.dryRun;
    bias.runtimeTrimRadS = Vec3::zero();
    bias.resetCounters();
    bias.enabled = wasEnabled;
    bias.dryRun = wasDryRun;
}

} // namespace tracker
