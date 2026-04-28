#pragma once

#include <cstdint>
#include <cmath>

#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"

namespace tracker {

// ============================================================
// IMU calibration layer
// ============================================================
// This file intentionally does not contain AHRS logic.
// It only estimates and applies sensor calibration values.
//
// Current implemented calibration:
//   - startup gyro bias calibration while tracker is stationary
//   - basic stationary detection
//   - accel identity calibration placeholder
//
// Later planned calibration:
//   - 6-position accel bias/scale calibration
//   - temperature model for gyro bias
//   - persistent NVS config with version + CRC
// ============================================================

struct ImuCalibration {
    // Gyro bias in rad/s, sensor frame.
    Vec3 gyroBiasRadS = Vec3::zero();

    // Accel calibration: accel_calibrated = accelScale * (accel_raw_g - accelBiasG)
    // For now this defaults to identity.
    Vec3 accelBiasG = Vec3::zero();
    Mat3 accelScale = Mat3::identity();

    bool gyroBiasValid = false;
    bool accelCalValid = false;

    Vec3 applyGyro(const Vec3& gyroRadS) const {
        return gyroRadS - gyroBiasRadS;
    }

    Vec3 applyAccel(const Vec3& accelG) const {
        return accelScale * (accelG - accelBiasG);
    }

    Lsm6dsv::Sample apply(const Lsm6dsv::Sample& s) const {
        Lsm6dsv::Sample out = s;
        if (gyroBiasValid) {
            out.gyro_rad_s = applyGyro(s.gyro_rad_s);
        }
        if (accelCalValid) {
            out.accel_g = applyAccel(s.accel_g);
        }
        return out;
    }
};

struct StationaryDetectorParams {
    // Conservative thresholds for startup calibration.
    // If calibration often fails even when the board is still, relax slightly.
    float maxGyroNormRadS = 2.0f * MATH_DEG_TO_RAD; // 2 dps
    float maxAccelNormErrorG = 0.08f;                // | |a| - 1g | < 0.08g

    // Additional stability check against slow handling/touching.
    float maxGyroVarianceRadS2 = square(0.35f * MATH_DEG_TO_RAD);
    float maxAccelVarianceG2 = square(0.015f);

    // First samples after power-up / ODR change can be less stable.
    uint32_t warmupSamples = 64;
};

struct StationaryStats {
    uint32_t count = 0;

    Vec3 gyroMeanRadS = Vec3::zero();
    Vec3 accelMeanG = Vec3::zero();

    Vec3 gyroM2 = Vec3::zero();
    Vec3 accelM2 = Vec3::zero();

    float gyroNormMeanRadS = 0.0f;
    float accelNormMeanG = 0.0f;

    void reset() {
        count = 0;
        gyroMeanRadS = Vec3::zero();
        accelMeanG = Vec3::zero();
        gyroM2 = Vec3::zero();
        accelM2 = Vec3::zero();
        gyroNormMeanRadS = 0.0f;
        accelNormMeanG = 0.0f;
    }

    void push(const Lsm6dsv::Sample& s) {
        count++;

        const float n = static_cast<float>(count);

        const Vec3 gyroDelta = s.gyro_rad_s - gyroMeanRadS;
        gyroMeanRadS += gyroDelta / n;
        const Vec3 gyroDelta2 = s.gyro_rad_s - gyroMeanRadS;
        gyroM2 += hadamard(gyroDelta, gyroDelta2);

        const Vec3 accelDelta = s.accel_g - accelMeanG;
        accelMeanG += accelDelta / n;
        const Vec3 accelDelta2 = s.accel_g - accelMeanG;
        accelM2 += hadamard(accelDelta, accelDelta2);

        gyroNormMeanRadS += (s.gyro_rad_s.norm() - gyroNormMeanRadS) / n;
        accelNormMeanG += (s.accel_g.norm() - accelNormMeanG) / n;
    }

    Vec3 gyroVarianceRadS2() const {
        if (count < 2) return Vec3::zero();
        return gyroM2 / static_cast<float>(count - 1);
    }

    Vec3 accelVarianceG2() const {
        if (count < 2) return Vec3::zero();
        return accelM2 / static_cast<float>(count - 1);
    }

    float gyroVarianceNormRadS2() const {
        return gyroVarianceRadS2().norm();
    }

    float accelVarianceNormG2() const {
        return accelVarianceG2().norm();
    }
};

class StationaryDetector {
public:
    explicit StationaryDetector(const StationaryDetectorParams& params = StationaryDetectorParams{})
        : params_(params) {}

    void reset() {
        samplesSeen_ = 0;
        stats_.reset();
    }

    bool pushAndCheckInstant(const Lsm6dsv::Sample& s) {
        samplesSeen_++;

        if (samplesSeen_ <= params_.warmupSamples) {
            return false;
        }

        if (!s.gyro_rad_s.isFinite() || !s.accel_g.isFinite()) {
            stats_.reset();
            return false;
        }

        const float gyroNorm = s.gyro_rad_s.norm();
        const float accelNorm = s.accel_g.norm();
        const float accelNormError = std::fabs(accelNorm - 1.0f);

        const bool instantStill =
            gyroNorm <= params_.maxGyroNormRadS &&
            accelNormError <= params_.maxAccelNormErrorG;

        if (!instantStill) {
            stats_.reset();
            return false;
        }

        stats_.push(s);
        return true;
    }

    bool windowLooksStationary(uint32_t minSamples) const {
        if (stats_.count < minSamples) {
            return false;
        }

        const float accelNormError = std::fabs(stats_.accelNormMeanG - 1.0f);

        return stats_.gyroMeanRadS.norm() <= params_.maxGyroNormRadS &&
               accelNormError <= params_.maxAccelNormErrorG &&
               stats_.gyroVarianceNormRadS2() <= params_.maxGyroVarianceRadS2 &&
               stats_.accelVarianceNormG2() <= params_.maxAccelVarianceG2;
    }

    const StationaryStats& stats() const {
        return stats_;
    }

    const StationaryDetectorParams& params() const {
        return params_;
    }

private:
    StationaryDetectorParams params_;
    uint32_t samplesSeen_ = 0;
    StationaryStats stats_;
};

struct GyroStartupCalibrationParams {
    // At 960 Hz: 1536 samples ~= 1.6 seconds.
    // At 480 Hz: 1536 samples ~= 3.2 seconds.
    uint32_t requiredStationarySamples = 1536;

    // Safety cap. If board is being moved, calibration should fail instead of
    // silently producing a bad bias.
    uint32_t maxTotalSamples = 9600;

    StationaryDetectorParams stationary;
};

struct GyroStartupCalibrationResult {
    bool success = false;
    Vec3 gyroBiasRadS = Vec3::zero();
    Vec3 gyroBiasDps = Vec3::zero();
    Vec3 accelMeanG = Vec3::zero();
    float accelNormMeanG = 0.0f;
    float gyroNoiseNormRadS2 = 0.0f;
    float accelNoiseNormG2 = 0.0f;
    uint32_t stationarySamples = 0;
    uint32_t totalSamples = 0;
};

class GyroStartupCalibrator {
public:
    explicit GyroStartupCalibrator(const GyroStartupCalibrationParams& params = GyroStartupCalibrationParams{})
        : params_(params), detector_(params.stationary) {}

    void reset() {
        result_ = GyroStartupCalibrationResult{};
        detector_.reset();
    }

    // Push one already-scaled LSM6DSV sample.
    // Returns true when calibration finished successfully.
    bool push(const Lsm6dsv::Sample& s) {
        result_.totalSamples++;

        const bool instantStill = detector_.pushAndCheckInstant(s);
        if (!instantStill) {
            return false;
        }

        const StationaryStats& st = detector_.stats();
        result_.stationarySamples = st.count;

        if (!detector_.windowLooksStationary(params_.requiredStationarySamples)) {
            return false;
        }

        result_.success = true;
        result_.gyroBiasRadS = st.gyroMeanRadS;
        result_.gyroBiasDps = st.gyroMeanRadS * MATH_RAD_TO_DEG;
        result_.accelMeanG = st.accelMeanG;
        result_.accelNormMeanG = st.accelNormMeanG;
        result_.gyroNoiseNormRadS2 = st.gyroVarianceNormRadS2();
        result_.accelNoiseNormG2 = st.accelVarianceNormG2();
        return true;
    }

    bool failedByTimeout() const {
        return !result_.success && result_.totalSamples >= params_.maxTotalSamples;
    }

    bool done() const {
        return result_.success;
    }

    const GyroStartupCalibrationResult& result() const {
        return result_;
    }

    const GyroStartupCalibrationParams& params() const {
        return params_;
    }

private:
    GyroStartupCalibrationParams params_;
    StationaryDetector detector_;
    GyroStartupCalibrationResult result_;
};

// A tiny online helper for later. Do not use this aggressively yet.
// It can slowly update gyro bias only when an external gate says the tracker is stationary.
class OnlineGyroBiasEstimator {
public:
    explicit OnlineGyroBiasEstimator(float alpha = 0.002f) : alpha_(alpha) {}

    void reset(const Vec3& initialBias = Vec3::zero()) {
        biasRadS_ = initialBias;
        initialized_ = true;
    }

    void updateIfStationary(const Vec3& gyroRadS, bool stationary) {
        if (!stationary || !gyroRadS.isFinite()) {
            return;
        }

        if (!initialized_) {
            biasRadS_ = gyroRadS;
            initialized_ = true;
            return;
        }

        biasRadS_ = lerp(biasRadS_, gyroRadS, alpha_);
    }

    Vec3 biasRadS() const {
        return biasRadS_;
    }

    Vec3 biasDps() const {
        return biasRadS_ * MATH_RAD_TO_DEG;
    }

    bool initialized() const {
        return initialized_;
    }

private:
    float alpha_ = 0.002f;
    Vec3 biasRadS_ = Vec3::zero();
    bool initialized_ = false;
};

} // namespace tracker
