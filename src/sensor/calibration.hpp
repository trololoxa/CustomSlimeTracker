#pragma once

#include <cstdint>

#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"

namespace tracker {

// ============================================================
// IMU calibration layer
// ============================================================
// This file intentionally does not contain AHRS logic.
// It only estimates and applies sensor calibration values.
// ============================================================

struct ImuCalibration {
    // Gyro bias in rad/s, sensor frame.
    Vec3 gyroBiasRadS = Vec3::zero();

    // Accel calibration: accel_calibrated = accelScale * (accel_raw_g - accelBiasG)
    Vec3 accelBiasG = Vec3::zero();
    Mat3 accelScale = Mat3::identity();

    bool gyroBiasValid = false;
    bool accelCalValid = false;

    Vec3 applyGyro(const Vec3& gyroRadS) const;
    Vec3 applyAccel(const Vec3& accelG) const;
    Lsm6dsv::Sample apply(const Lsm6dsv::Sample& s) const;
};

struct StationaryDetectorParams {
    float maxGyroNormRadS = 2.0f * MATH_DEG_TO_RAD; // 2 dps
    float maxAccelNormErrorG = 0.08f;                // | |a| - 1g | < 0.08g
    float maxGyroVarianceRadS2 = square(0.35f * MATH_DEG_TO_RAD);
    float maxAccelVarianceG2 = square(0.015f);
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

    void reset();
    void push(const Lsm6dsv::Sample& s);

    Vec3 gyroVarianceRadS2() const;
    Vec3 accelVarianceG2() const;
    float gyroVarianceNormRadS2() const;
    float accelVarianceNormG2() const;
};

class StationaryDetector {
public:
    explicit StationaryDetector(const StationaryDetectorParams& params = StationaryDetectorParams{});

    void reset();
    bool pushAndCheckInstant(const Lsm6dsv::Sample& s);
    bool windowLooksStationary(uint32_t minSamples) const;

    const StationaryStats& stats() const;
    const StationaryDetectorParams& params() const;

private:
    StationaryDetectorParams params_;
    uint32_t samplesSeen_ = 0;
    StationaryStats stats_;
};

struct GyroStartupCalibrationParams {
    uint32_t requiredStationarySamples = 1536;
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
    explicit GyroStartupCalibrator(const GyroStartupCalibrationParams& params = GyroStartupCalibrationParams{});

    void reset();
    bool push(const Lsm6dsv::Sample& s);

    bool failedByTimeout() const;
    bool done() const;
    const GyroStartupCalibrationResult& result() const;
    const GyroStartupCalibrationParams& params() const;

private:
    GyroStartupCalibrationParams params_;
    StationaryDetector detector_;
    GyroStartupCalibrationResult result_;
};

// A tiny online helper for later. Do not use this aggressively yet.
// It can slowly update gyro bias only when an external gate says the tracker is stationary.
class OnlineGyroBiasEstimator {
public:
    explicit OnlineGyroBiasEstimator(float alpha = 0.002f);

    void reset(const Vec3& initialBias = Vec3::zero());
    void updateIfStationary(const Vec3& gyroRadS, bool stationary);

    Vec3 biasRadS() const;
    Vec3 biasDps() const;
    bool initialized() const;

private:
    float alpha_ = 0.002f;
    Vec3 biasRadS_ = Vec3::zero();
    bool initialized_ = false;
};

} // namespace tracker
