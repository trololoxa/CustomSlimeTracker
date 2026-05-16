#include "runtime/gyro_temp_calibration_capture.hpp"

namespace tracker {

void GyroTempCalibrationCapture::start(uint32_t nowMs, uint32_t maxDurationMs) {
    capture_.reset();
    capture_.active = true;
    capture_.stopRequested = false;
    capture_.durationMs = maxDurationMs;
    capture_.startMs = nowMs;
    capture_.lastProgressMs = nowMs;
    completed_ = false;
}

void GyroTempCalibrationCapture::stop(uint32_t nowMs) {
    if (!capture_.active) return;
    capture_.active = false;
    capture_.stopRequested = false;
    capture_.durationMs = nowMs - capture_.startMs;
    completed_ = true;
}

void GyroTempCalibrationCapture::reset() {
    capture_.reset();
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

    const float accelNormG = quality.accelNormValid ? quality.accelNormG : calibrated.accel_g.norm();
    t.accelNormG.push(accelNormG);
    t.accelTrust.push(quality.accelConfidence);

    const bool goodForTempFit = quality.shouldUpdateAhrs &&
        !quality.shouldRequestFifoRecovery &&
        !quality.has(imu_quality_flags::TIMESTAMP_ZERO) &&
        !quality.has(imu_quality_flags::TIMESTAMP_NON_MONOTONIC) &&
        !quality.has(imu_quality_flags::TIMESTAMP_BACKWARDS) &&
        !quality.has(imu_quality_flags::TIMESTAMP_LARGE_GAP) &&
        !quality.has(imu_quality_flags::FIFO_OVERRUN) &&
        !quality.has(imu_quality_flags::FIFO_FULL) &&
        !quality.has(imu_quality_flags::GYRO_SATURATED) &&
        !quality.has(imu_quality_flags::ACCEL_SATURATED);

    const int bin = staticTempBinIndex(calibrated.temp_c);
    if (bin >= 0) {
        t.tempBins[bin].push(calibrated.temp_c,
                             calibrated.gyro_rad_s,
                             accelNormG,
                             goodForTempFit);
    } else {
        t.tempBinOutOfRangeSamples++;
    }

    t.samples++;

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
