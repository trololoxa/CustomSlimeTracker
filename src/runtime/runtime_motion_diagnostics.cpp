#include "runtime/runtime_motion_diagnostics.hpp"

#include "core/math.hpp"

namespace tracker {

namespace {
const char* yesNo(bool v) { return v ? "yes" : "no"; }
}

void RuntimeMotionDiagnostics::begin(uint32_t nowMs) {
    stats_.configured = true;
    stats_.enabled = false;
    reset(nowMs);
}

void RuntimeMotionDiagnostics::setEnabled(bool enabled, uint32_t nowMs) {
    stats_.enabled = enabled;
    reset(nowMs);
}

void RuntimeMotionDiagnostics::reset(uint32_t nowMs) {
    const bool keepEnabled = stats_.enabled;
    const bool keepConfigured = stats_.configured;
    stats_ = WindowStats{};
    stats_.enabled = keepEnabled;
    stats_.configured = keepConfigured;
    stats_.resetMs = nowMs;
}

uint32_t RuntimeMotionDiagnostics::windowMs(uint32_t nowMs) const {
    return nowMs - stats_.resetMs;
}

void RuntimeMotionDiagnostics::updateMinMaxU32(uint32_t value, uint32_t& minValue, uint32_t& maxValue, bool first) {
    if (first || value < minValue) minValue = value;
    if (first || value > maxValue) maxValue = value;
}

void RuntimeMotionDiagnostics::updateMinMaxF(float value, float& minValue, float& maxValue, bool first) {
    if (first || value < minValue) minValue = value;
    if (first || value > maxValue) maxValue = value;
}

void RuntimeMotionDiagnostics::recordSample(const Lsm6dsv::RawSample& raw,
                                            const Lsm6dsv::Sample& calibrated,
                                            const ImuQualityResult& quality) {
    if (!stats_.enabled) return;

    ++stats_.samples;
    stats_.lastTimestampUs = raw.t_us;

    if (quality.shouldUpdateAhrs) ++stats_.ahrsUsableSamples;
    if (!quality.shouldUpdateAhrs || quality.has(imu_quality_flags::SAMPLE_NOT_AHRS_USABLE)) ++stats_.ahrsSkippedSamples;
    if (!quality.shouldUseAccelCorrection || quality.has(imu_quality_flags::ACCEL_NOT_AHRS_USABLE)) ++stats_.accelCorrectionDisabledSamples;

    stats_.estimatedDroppedSamples += quality.estimatedDroppedBefore;

    if (quality.has(imu_quality_flags::TIMESTAMP_HARDWARE)) ++stats_.hwTimestampSamples;
    if (quality.has(imu_quality_flags::TIMESTAMP_FALLBACK)) ++stats_.fallbackTimestampSamples;
    if (quality.has(imu_quality_flags::TIMESTAMP_LARGE_GAP)) ++stats_.largeGapSamples;
    if (quality.has(imu_quality_flags::TIMESTAMP_NON_MONOTONIC) || quality.has(imu_quality_flags::TIMESTAMP_BACKWARDS)) {
        ++stats_.nonMonotonicTimestampSamples;
    }
    if (quality.has(imu_quality_flags::FIFO_RECOVERY_REQUESTED)) ++stats_.fifoRecoveryRequests;

    if (quality.has(imu_quality_flags::GYRO_SATURATED)) ++stats_.gyroSaturatedSamples;
    if (quality.has(imu_quality_flags::GYRO_NEAR_SATURATION)) ++stats_.gyroNearSaturatedSamples;
    if (quality.has(imu_quality_flags::ACCEL_SATURATED)) ++stats_.accelSaturatedSamples;
    if (quality.has(imu_quality_flags::ACCEL_NEAR_SATURATION)) ++stats_.accelNearSaturatedSamples;
    if (quality.has(imu_quality_flags::ACCEL_NORM_OUTLIER)) ++stats_.accelNormOutliers;
    if (quality.has(imu_quality_flags::ACCEL_COMPONENT_MISSING)) ++stats_.accelComponentMissingSamples;
    if (quality.has(imu_quality_flags::FIFO_PAIR_DEGRADED)) ++stats_.pairCoherencyDegradedSamples;

    const uint32_t divisor = stats_.sampleDivisor == 0u ? 1u : stats_.sampleDivisor;
    if (((stats_.samples - 1u) % divisor) != 0u) {
        stats_.qualityFlagsOr |= quality.flags;
        stats_.lastQualityFlags = quality.flags;
        return;
    }

    const bool firstMetric = stats_.metricSamples == 0u;
    ++stats_.metricSamples;
    stats_.lastRecordMs = millis();

    if (quality.dtUs > 0u) {
        stats_.dtSumUs += static_cast<double>(quality.dtUs);
        updateMinMaxU32(quality.dtUs, stats_.dtMinUs, stats_.dtMaxUs, firstMetric || stats_.dtMinUs == 0u);
    }

    const float gyroNormDps = calibrated.gyro_rad_s.norm() * MATH_RAD_TO_DEG;
    stats_.gyroNormLastDps = gyroNormDps;
    stats_.gyroNormSumDps += static_cast<double>(gyroNormDps);
    if (gyroNormDps > stats_.gyroNormMaxDps || firstMetric) stats_.gyroNormMaxDps = gyroNormDps;

    if (quality.accelNormValid) {
        const float accelNormG = quality.accelNormG;
        const bool firstAccel = stats_.accelObservationSamples == 0u;
        ++stats_.accelObservationSamples;
        stats_.accelNormLastG = accelNormG;
        stats_.accelNormSumG += static_cast<double>(accelNormG);
        updateMinMaxF(accelNormG, stats_.accelNormMinG, stats_.accelNormMaxG, firstAccel);
    }

    stats_.confidenceLast = quality.overallConfidence;
    stats_.confidenceSum += static_cast<double>(quality.overallConfidence);

    if (firstMetric) {
        stats_.tempStartC = calibrated.temp_c;
        stats_.tempValid = true;
    }
    stats_.tempLastC = calibrated.temp_c;

    stats_.qualityFlagsOr |= quality.flags;
    stats_.lastQualityFlags = quality.flags;
}

float RuntimeMotionDiagnostics::sampleRateHz(const WindowStats& s, uint32_t windowMs) {
    if (windowMs == 0u) return 0.0f;
    return 1000.0f * static_cast<float>(s.samples) / static_cast<float>(windowMs);
}

float RuntimeMotionDiagnostics::avgDtUs(const WindowStats& s) {
    return s.metricSamples == 0u ? 0.0f : static_cast<float>(s.dtSumUs / static_cast<double>(s.metricSamples));
}

float RuntimeMotionDiagnostics::avgGyroNormDps(const WindowStats& s) {
    return s.metricSamples == 0u ? 0.0f : static_cast<float>(s.gyroNormSumDps / static_cast<double>(s.metricSamples));
}

float RuntimeMotionDiagnostics::avgAccelNormG(const WindowStats& s) {
    return s.accelObservationSamples == 0u
        ? 0.0f
        : static_cast<float>(s.accelNormSumG / static_cast<double>(s.accelObservationSamples));
}

float RuntimeMotionDiagnostics::avgConfidence(const WindowStats& s) {
    return s.metricSamples == 0u ? 0.0f : static_cast<float>(s.confidenceSum / static_cast<double>(s.metricSamples));
}

void RuntimeMotionDiagnostics::printStatus(Stream& out, uint32_t nowMs) const {
    const uint32_t winMs = windowMs(nowMs);
    out.println("# MOTION DIAGNOSTICS");
    out.print("motion_configured="); out.println(yesNo(stats_.configured));
    out.print("motion_enabled="); out.println(yesNo(stats_.enabled));
    out.print("motion_window_ms="); out.println(winMs);
    out.print("motion_last_record_age_ms="); out.println(stats_.lastRecordMs == 0u ? 0u : nowMs - stats_.lastRecordMs);
    out.print("motion_samples="); out.println(stats_.samples);
    out.print("motion_sample_rate_hz="); out.println(sampleRateHz(stats_, winMs), 3);
    out.print("motion_metric_samples="); out.println(stats_.metricSamples);
    out.print("motion_sample_divisor="); out.println(stats_.sampleDivisor);

    out.print("motion_dt_avg_us="); out.println(avgDtUs(stats_), 3);
    out.print("motion_dt_min_us="); out.println(stats_.dtMinUs);
    out.print("motion_dt_max_us="); out.println(stats_.dtMaxUs);
    out.print("motion_large_gap_samples="); out.println(stats_.largeGapSamples);
    out.print("motion_nonmonotonic_timestamp_samples="); out.println(stats_.nonMonotonicTimestampSamples);
    out.print("motion_estimated_dropped_samples="); out.println(stats_.estimatedDroppedSamples);
    out.print("motion_hw_timestamp_samples="); out.println(stats_.hwTimestampSamples);
    out.print("motion_fallback_timestamp_samples="); out.println(stats_.fallbackTimestampSamples);

    out.print("motion_gyro_norm_avg_dps="); out.println(avgGyroNormDps(stats_), 3);
    out.print("motion_gyro_norm_max_dps="); out.println(stats_.gyroNormMaxDps, 3);
    out.print("motion_gyro_norm_last_dps="); out.println(stats_.gyroNormLastDps, 3);
    out.print("motion_gyro_saturated_samples="); out.println(stats_.gyroSaturatedSamples);
    out.print("motion_gyro_near_saturated_samples="); out.println(stats_.gyroNearSaturatedSamples);

    out.print("motion_accel_norm_avg_g="); out.println(avgAccelNormG(stats_), 5);
    out.print("motion_accel_norm_min_g="); out.println(stats_.accelNormMinG, 5);
    out.print("motion_accel_norm_max_g="); out.println(stats_.accelNormMaxG, 5);
    out.print("motion_accel_norm_last_g="); out.println(stats_.accelNormLastG, 5);
    out.print("motion_accel_saturated_samples="); out.println(stats_.accelSaturatedSamples);
    out.print("motion_accel_near_saturated_samples="); out.println(stats_.accelNearSaturatedSamples);
    out.print("motion_accel_norm_outliers="); out.println(stats_.accelNormOutliers);
    out.print("motion_accel_component_missing_samples="); out.println(stats_.accelComponentMissingSamples);
    out.print("motion_pair_coherency_degraded_samples="); out.println(stats_.pairCoherencyDegradedSamples);
    out.print("motion_accel_observation_samples="); out.println(stats_.accelObservationSamples);
    out.print("motion_accel_correction_disabled_samples="); out.println(stats_.accelCorrectionDisabledSamples);

    out.print("motion_ahrs_usable_samples="); out.println(stats_.ahrsUsableSamples);
    out.print("motion_ahrs_skipped_samples="); out.println(stats_.ahrsSkippedSamples);
    out.print("motion_fifo_recovery_requests="); out.println(stats_.fifoRecoveryRequests);
    out.print("motion_confidence_avg="); out.println(avgConfidence(stats_), 4);
    out.print("motion_confidence_last="); out.println(stats_.confidenceLast, 4);

    out.print("motion_temp_start_c="); out.println(stats_.tempStartC, 2);
    out.print("motion_temp_last_c="); out.println(stats_.tempLastC, 2);
    out.print("motion_temp_delta_c="); out.println(stats_.tempValid ? stats_.tempLastC - stats_.tempStartC : 0.0f, 2);
    const float minutes = static_cast<float>(winMs) / 60000.0f;
    out.print("motion_temp_slope_c_per_min="); out.println((stats_.tempValid && minutes > 0.0f) ? (stats_.tempLastC - stats_.tempStartC) / minutes : 0.0f, 4);

    out.print("motion_quality_flags_or=0x"); out.println(stats_.qualityFlagsOr, HEX);
    out.print("motion_last_quality_flags=0x"); out.println(stats_.lastQualityFlags, HEX);
}

} // namespace tracker
