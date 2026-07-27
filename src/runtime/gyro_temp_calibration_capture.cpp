#include "runtime/gyro_temp_calibration_capture.hpp"

#include <cmath>

namespace tracker {

namespace {

bool allAxesWithin(const Vec3& v, float limit) {
    return std::fabs(v.x) <= limit &&
        std::fabs(v.y) <= limit &&
        std::fabs(v.z) <= limit;
}

float maxAbsComponent(const Vec3& v) {
    return std::fmax(std::fabs(v.x), std::fmax(std::fabs(v.y), std::fabs(v.z)));
}

Vec3 standardError(const Vec3& stddev, uint32_t samples) {
    if (samples == 0u) return Vec3::zero();
    const float invSqrtN = 1.0f / std::sqrt(static_cast<float>(samples));
    return stddev * invSqrtN;
}

} // namespace

GyroTempCalibrationCapture::GyroTempCalibrationCapture(
    const GyroTempCalibrationCaptureConfig& cfg
) {
    setConfig(cfg);
}

void GyroTempCalibrationCapture::setConfig(const GyroTempCalibrationCaptureConfig& cfg) {
    cfg_ = cfg;
    if (cfg_.targetWindowSamples == 0) cfg_.targetWindowSamples = 1;
    if (cfg_.minWindowSamples == 0) cfg_.minWindowSamples = 1;
    if (cfg_.minWindowSamples > cfg_.targetWindowSamples) {
        cfg_.minWindowSamples = cfg_.targetWindowSamples;
    }
}

const GyroTempCalibrationCaptureConfig& GyroTempCalibrationCapture::config() const {
    return cfg_;
}

void GyroTempCalibrationCapture::PendingWindow::reset() {
    binIndex = -1;
    tempC.reset();
    gyroRadS.reset();
    accelNormG.reset();
    accelConfidence.reset();
}

uint32_t GyroTempCalibrationCapture::PendingWindow::samples() const {
    return gyroRadS.count;
}

void GyroTempCalibrationCapture::start(uint32_t nowMs, uint32_t maxDurationMs) {
    capture_.reset();
    capture_.active = true;
    capture_.stopRequested = false;
    capture_.durationMs = maxDurationMs;
    capture_.startMs = nowMs;
    capture_.lastProgressMs = nowMs;
    diagnostics_ = GyroTempCalibrationCaptureDiagnostics{};
    resetPendingWindow();
    haveAcceptedGyroReference_ = false;
    acceptedReferenceTempC_ = 0.0f;
    acceptedReferenceGyroMeanDps_ = Vec3::zero();
    lastAcceptedTempC_ = 0.0f;
    lastAcceptedGyroMeanDps_ = Vec3::zero();
    completed_ = false;
}

void GyroTempCalibrationCapture::stop(uint32_t nowMs) {
    if (!capture_.active) return;
    finalizePendingWindow(true);
    capture_.active = false;
    capture_.stopRequested = false;
    capture_.durationMs = nowMs - capture_.startMs;
    completed_ = true;
}

void GyroTempCalibrationCapture::reset() {
    capture_.reset();
    diagnostics_ = GyroTempCalibrationCaptureDiagnostics{};
    resetPendingWindow();
    haveAcceptedGyroReference_ = false;
    acceptedReferenceTempC_ = 0.0f;
    acceptedReferenceGyroMeanDps_ = Vec3::zero();
    lastAcceptedTempC_ = 0.0f;
    lastAcceptedGyroMeanDps_ = Vec3::zero();
    completed_ = false;
}

bool GyroTempCalibrationCapture::active() const {
    return capture_.active;
}

bool GyroTempCalibrationCapture::completed() const {
    return completed_ && !capture_.active;
}

uint32_t GyroTempCalibrationCapture::elapsedMs(uint32_t nowMs) const {
    return nowMs - capture_.startMs;
}

bool GyroTempCalibrationCapture::baseQualityGood(
    const Lsm6dsv::Sample& calibrated,
    const ImuQualityResult& quality
) const {
    if (!calibrated.gyro_rad_s.isFinite() ||
        !calibrated.accel_g.isFinite() ||
        !std::isfinite(calibrated.temp_c)) {
        return false;
    }

    return quality.shouldUpdateAhrs &&
        quality.shouldUseAccelCorrection &&
        !quality.shouldRequestFifoRecovery &&
        !quality.has(imu_quality_flags::TIMESTAMP_ZERO) &&
        !quality.has(imu_quality_flags::TIMESTAMP_NON_MONOTONIC) &&
        !quality.has(imu_quality_flags::TIMESTAMP_BACKWARDS) &&
        !quality.has(imu_quality_flags::TIMESTAMP_LARGE_GAP) &&
        !quality.has(imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW) &&
        !quality.has(imu_quality_flags::FIFO_OVERRUN) &&
        !quality.has(imu_quality_flags::FIFO_FULL) &&
        !quality.has(imu_quality_flags::FIFO_RECOVERY_REQUESTED) &&
        !quality.has(imu_quality_flags::GYRO_SATURATED) &&
        !quality.has(imu_quality_flags::ACCEL_SATURATED);
}

bool GyroTempCalibrationCapture::immediateStationaryGood(
    const Lsm6dsv::Sample& calibrated,
    const ImuQualityResult& quality
) const {
    const float accelNormG = quality.accelNormValid
        ? quality.accelNormG
        : calibrated.accel_g.norm();
    const float gyroNormDps = calibrated.gyro_rad_s.norm() * MATH_RAD_TO_DEG;

    return std::isfinite(accelNormG) &&
        std::isfinite(gyroNormDps) &&
        gyroNormDps <= cfg_.maxInstantGyroNormDps &&
        std::fabs(accelNormG - 1.0f) <= cfg_.maxInstantAccelNormErrorG &&
        quality.accelConfidence >= cfg_.minInstantAccelConfidence;
}

void GyroTempCalibrationCapture::recordRejectedSample(int bin, bool motionRejected) {
    diagnostics_.rejectedSamples++;
    if (motionRejected) diagnostics_.motionRejectedSamples++;
    else diagnostics_.qualityRejectedSamples++;

    if (bin >= 0) {
        capture_.tempBins[bin].badQualitySamples++;
    } else {
        capture_.tempBinOutOfRangeSamples++;
    }
}

void GyroTempCalibrationCapture::rejectPendingWindow(bool motionReset) {
    const uint32_t count = pending_.samples();
    if (count == 0) return;

    diagnostics_.rejectedWindows++;
    diagnostics_.rejectedSamples += count;
    if (motionReset) {
        diagnostics_.motionWindowResets++;
        diagnostics_.motionRejectedSamples += count;
    }

    if (pending_.binIndex >= 0) {
        capture_.tempBins[pending_.binIndex].badQualitySamples += count;
    } else {
        capture_.tempBinOutOfRangeSamples += count;
    }
    resetPendingWindow();
}

void GyroTempCalibrationCapture::finalizePendingWindow(bool allowShortWindow) {
    const uint32_t count = pending_.samples();
    if (count == 0) return;
    if (!allowShortWindow && count < cfg_.targetWindowSamples) return;

    if (count < cfg_.minWindowSamples || pending_.binIndex < 0) {
        diagnostics_.discardedPartialSamples += count;
        resetPendingWindow();
        return;
    }

    const float tempMeanC = pending_.tempC.mean();
    const Vec3 gyroMeanDps = pending_.gyroRadS.mean() * MATH_RAD_TO_DEG;
    const Vec3 gyroStdDps = pending_.gyroRadS.stddev() * MATH_RAD_TO_DEG;
    const Vec3 gyroMeanStdErrorDps = standardError(gyroStdDps, count);
    const float accelMeanG = pending_.accelNormG.mean();
    const float accelStdG = pending_.accelNormG.stddev();
    const float accelConfidenceMean = pending_.accelConfidence.mean();
    const float tempSpanC = pending_.tempC.maxValue - pending_.tempC.minValue;

    Vec3 anchorResidualDps = Vec3::zero();
    Vec3 adjacentResidualDps = Vec3::zero();
    bool initialMeanGood = true;
    bool thermalConsistencyGood = true;
    if (!haveAcceptedGyroReference_) {
        initialMeanGood = gyroMeanDps.norm() <= cfg_.maxInitialGyroMeanNormDps;
    } else {
        anchorResidualDps = gyroMeanDps - acceptedReferenceGyroMeanDps_;
        adjacentResidualDps = gyroMeanDps - lastAcceptedGyroMeanDps_;
        const float anchorAllowance = cfg_.gyroAnchorSlackDps +
            cfg_.maxThermalSlopeDpsPerC * std::fabs(tempMeanC - acceptedReferenceTempC_);
        const float adjacentAllowance = cfg_.gyroAdjacentSlackDps +
            cfg_.maxThermalSlopeDpsPerC * std::fabs(tempMeanC - lastAcceptedTempC_);
        thermalConsistencyGood =
            maxAbsComponent(anchorResidualDps) <= anchorAllowance &&
            maxAbsComponent(adjacentResidualDps) <= adjacentAllowance;
    }

    diagnostics_.lastGyroMeanDps = gyroMeanDps;
    diagnostics_.lastGyroStdDps = gyroStdDps;
    diagnostics_.lastGyroMeanStdErrorDps = gyroMeanStdErrorDps;
    diagnostics_.lastGyroAnchorResidualDps = anchorResidualDps;
    diagnostics_.lastGyroAdjacentResidualDps = adjacentResidualDps;
    diagnostics_.lastAccelNormMeanG = accelMeanG;
    diagnostics_.lastAccelNormStdG = accelStdG;
    diagnostics_.lastAccelConfidenceMean = accelConfidenceMean;
    diagnostics_.lastTempSpanC = tempSpanC;

    const bool hardNoiseGood = gyroStdDps.isFinite() &&
        gyroStdDps.norm() <= cfg_.maxGyroStdNormDps &&
        allAxesWithin(gyroStdDps, cfg_.maxGyroStdAxisDps);
    const bool meanPrecisionGood = gyroMeanStdErrorDps.isFinite() &&
        gyroMeanStdErrorDps.norm() <= cfg_.maxGyroMeanStdErrorNormDps &&
        allAxesWithin(gyroMeanStdErrorDps, cfg_.maxGyroMeanStdErrorAxisDps);
    const bool gyroGood = gyroMeanDps.isFinite() &&
        std::isfinite(tempMeanC) &&
        hardNoiseGood && meanPrecisionGood && initialMeanGood && thermalConsistencyGood;
    const bool accelGood = std::isfinite(accelMeanG) &&
        std::isfinite(accelStdG) &&
        std::isfinite(accelConfidenceMean) &&
        std::fabs(accelMeanG - 1.0f) <= cfg_.maxAccelNormMeanErrorG &&
        accelStdG <= cfg_.maxAccelNormStdG &&
        accelConfidenceMean >= cfg_.minAccelConfidenceMean;
    const bool temperatureGood = std::isfinite(tempSpanC) &&
        tempSpanC <= cfg_.maxWindowTempSpanC;

    if (!gyroGood || !accelGood || !temperatureGood) {
        diagnostics_.rejectedWindows++;
        diagnostics_.rejectedSamples += count;
        if (!gyroGood) {
            diagnostics_.gyroRejectedWindows++;
            if (!hardNoiseGood) diagnostics_.gyroHardNoiseRejectedWindows++;
            if (!meanPrecisionGood) diagnostics_.gyroMeanPrecisionRejectedWindows++;
            if (!initialMeanGood) diagnostics_.gyroInitialMeanRejectedWindows++;
            if (!thermalConsistencyGood) diagnostics_.gyroThermalConsistencyRejectedWindows++;
        }
        if (!accelGood) diagnostics_.accelRejectedWindows++;
        if (!temperatureGood) diagnostics_.temperatureRejectedWindows++;
        capture_.tempBins[pending_.binIndex].badQualitySamples += count;
        resetPendingWindow();
        return;
    }

    StaticTempBinStats& bin = capture_.tempBins[pending_.binIndex];
    bin.tempC.merge(pending_.tempC);
    bin.accelNormG.merge(pending_.accelNormG);
    bin.gyroAfterRadS.merge(pending_.gyroRadS);

    if (!haveAcceptedGyroReference_) {
        haveAcceptedGyroReference_ = true;
        acceptedReferenceTempC_ = tempMeanC;
        acceptedReferenceGyroMeanDps_ = gyroMeanDps;
    }
    lastAcceptedTempC_ = tempMeanC;
    lastAcceptedGyroMeanDps_ = gyroMeanDps;

    diagnostics_.acceptedWindows++;
    diagnostics_.acceptedSamples += count;
    resetPendingWindow();
}

void GyroTempCalibrationCapture::resetPendingWindow() {
    pending_.reset();
    diagnostics_.currentWindowSamples = 0;
}

void GyroTempCalibrationCapture::updateSample(const Lsm6dsv::Sample& calibrated,
                                              const ImuQualityResult& quality,
                                              uint32_t nowMs) {
    if (!capture_.active) return;

    StaticRuntimeTest& t = capture_;
    if (!t.tempCaptured) {
        t.tempCaptured = true;
        t.tempStartC = calibrated.temp_c;
    }
    t.tempEndC = calibrated.temp_c;
    t.tempC.push(calibrated.temp_c);
    t.gyroAfterRadS.push(calibrated.gyro_rad_s);

    const float accelNormG = quality.accelNormValid
        ? quality.accelNormG
        : calibrated.accel_g.norm();
    t.accelNormG.push(accelNormG);
    t.accelTrust.push(quality.accelConfidence);
    t.samples++;

    const int bin = staticTempBinIndex(calibrated.temp_c);
    const bool qualityGood = baseQualityGood(calibrated, quality);
    const bool stationaryGood = qualityGood && immediateStationaryGood(calibrated, quality);
    if (!qualityGood || !stationaryGood || bin < 0) {
        const bool motionRejected = qualityGood && !stationaryGood;
        rejectPendingWindow(motionRejected);
        recordRejectedSample(bin, motionRejected);
    } else {
        if (pending_.samples() > 0 && pending_.binIndex != bin) {
            // Crossing a 1 C bin boundary is not a motion failure. Commit a
            // sufficiently long partial window or quietly discard a very short
            // boundary fragment.
            finalizePendingWindow(true);
        }
        if (pending_.samples() == 0) pending_.binIndex = static_cast<int8_t>(bin);

        pending_.tempC.push(calibrated.temp_c);
        pending_.gyroRadS.push(calibrated.gyro_rad_s);
        pending_.accelNormG.push(accelNormG);
        pending_.accelConfidence.push(quality.accelConfidence);
        diagnostics_.currentWindowSamples = pending_.samples();

        if (pending_.samples() >= cfg_.targetWindowSamples) {
            finalizePendingWindow(false);
        }
    }

    if (t.durationMs > 0 && nowMs - t.startMs >= t.durationMs) {
        stop(nowMs);
    }
}

const StaticRuntimeTest& GyroTempCalibrationCapture::capture() const {
    return capture_;
}

StaticRuntimeTest& GyroTempCalibrationCapture::capture() {
    return capture_;
}

const GyroTempCalibrationCaptureDiagnostics& GyroTempCalibrationCapture::diagnostics() const {
    return diagnostics_;
}

uint32_t GyroTempCalibrationCapture::usableTempBins(uint32_t minSamplesPerBin) const {
    uint32_t bins = 0;
    for (uint8_t i = 0; i < STATIC_TEMP_BIN_COUNT; ++i) {
        if (capture_.tempBins[i].gyroAfterRadS.count >= minSamplesPerBin &&
            capture_.tempBins[i].tempC.count >= minSamplesPerBin) {
            bins++;
        }
    }
    return bins;
}

float GyroTempCalibrationCapture::tempRangeC() const {
    if (!capture_.tempCaptured) return 0.0f;
    return capture_.tempEndC >= capture_.tempStartC
        ? (capture_.tempEndC - capture_.tempStartC)
        : (capture_.tempStartC - capture_.tempEndC);
}

} // namespace tracker
