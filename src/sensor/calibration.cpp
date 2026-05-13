#include "sensor/calibration.hpp"

#include <cmath>

namespace tracker {

Vec3 ImuCalibration::applyGyro(const Vec3& gyroRadS) const {
    return gyroRadS - gyroBiasRadS;
}

Vec3 ImuCalibration::applyAccel(const Vec3& accelG) const {
    return accelScale * (accelG - accelBiasG);
}

Lsm6dsv::Sample ImuCalibration::apply(const Lsm6dsv::Sample& s) const {
    Lsm6dsv::Sample out = s;
    if (gyroBiasValid) {
        out.gyro_rad_s = applyGyro(s.gyro_rad_s);
    }
    if (accelCalValid) {
        out.accel_g = applyAccel(s.accel_g);
    }
    return out;
}

void StationaryStats::reset() {
    count = 0;
    gyroMeanRadS = Vec3::zero();
    accelMeanG = Vec3::zero();
    gyroM2 = Vec3::zero();
    accelM2 = Vec3::zero();
    gyroNormMeanRadS = 0.0f;
    accelNormMeanG = 0.0f;
}

void StationaryStats::push(const Lsm6dsv::Sample& s) {
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

Vec3 StationaryStats::gyroVarianceRadS2() const {
    if (count < 2) return Vec3::zero();
    return gyroM2 / static_cast<float>(count - 1);
}

Vec3 StationaryStats::accelVarianceG2() const {
    if (count < 2) return Vec3::zero();
    return accelM2 / static_cast<float>(count - 1);
}

float StationaryStats::gyroVarianceNormRadS2() const {
    return gyroVarianceRadS2().norm();
}

float StationaryStats::accelVarianceNormG2() const {
    return accelVarianceG2().norm();
}

StationaryDetector::StationaryDetector(const StationaryDetectorParams& params)
    : params_(params) {}

void StationaryDetector::reset() {
    samplesSeen_ = 0;
    stats_.reset();
}

bool StationaryDetector::pushAndCheckInstant(const Lsm6dsv::Sample& s) {
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

bool StationaryDetector::windowLooksStationary(uint32_t minSamples) const {
    if (stats_.count < minSamples) {
        return false;
    }

    const float accelNormError = std::fabs(stats_.accelNormMeanG - 1.0f);

    return stats_.gyroMeanRadS.norm() <= params_.maxGyroNormRadS &&
           accelNormError <= params_.maxAccelNormErrorG &&
           stats_.gyroVarianceNormRadS2() <= params_.maxGyroVarianceRadS2 &&
           stats_.accelVarianceNormG2() <= params_.maxAccelVarianceG2;
}

const StationaryStats& StationaryDetector::stats() const {
    return stats_;
}

const StationaryDetectorParams& StationaryDetector::params() const {
    return params_;
}

GyroStartupCalibrator::GyroStartupCalibrator(const GyroStartupCalibrationParams& params)
    : params_(params), detector_(params.stationary) {}

void GyroStartupCalibrator::reset() {
    result_ = GyroStartupCalibrationResult{};
    detector_.reset();
}

bool GyroStartupCalibrator::push(const Lsm6dsv::Sample& s) {
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

bool GyroStartupCalibrator::failedByTimeout() const {
    return !result_.success && result_.totalSamples >= params_.maxTotalSamples;
}

bool GyroStartupCalibrator::done() const {
    return result_.success;
}

const GyroStartupCalibrationResult& GyroStartupCalibrator::result() const {
    return result_;
}

const GyroStartupCalibrationParams& GyroStartupCalibrator::params() const {
    return params_;
}

OnlineGyroBiasEstimator::OnlineGyroBiasEstimator(float alpha) : alpha_(alpha) {}

void OnlineGyroBiasEstimator::reset(const Vec3& initialBias) {
    biasRadS_ = initialBias;
    initialized_ = true;
}

void OnlineGyroBiasEstimator::updateIfStationary(const Vec3& gyroRadS, bool stationary) {
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

Vec3 OnlineGyroBiasEstimator::biasRadS() const {
    return biasRadS_;
}

Vec3 OnlineGyroBiasEstimator::biasDps() const {
    return biasRadS_ * MATH_RAD_TO_DEG;
}

bool OnlineGyroBiasEstimator::initialized() const {
    return initialized_;
}

} // namespace tracker
