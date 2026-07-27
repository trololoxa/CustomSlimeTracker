#include "runtime/setup_output_verifier.hpp"

#include <algorithm>
#include <cmath>

namespace tracker {
namespace {

float quaternionAngularDifferenceDeg(const Quat& a, const Quat& b) {
    const Quat an = a.normalized();
    const Quat bn = b.normalized();
    const float d = std::fabs(an.w * bn.w + an.x * bn.x + an.y * bn.y + an.z * bn.z);
    const float clamped = clampf(d, 0.0f, 1.0f);
    return 2.0f * std::acos(clamped) * MATH_RAD_TO_DEG;
}

} // namespace

void SetupOutputVerificationAccumulator::reset() {
    *this = SetupOutputVerificationAccumulator{};
}

void SetupOutputVerificationAccumulator::push(const TrackerPreparedOutputSnapshot& snapshot) {
    snapshots_++;
    if (!snapshot.valid || !snapshot.q.isFinite()) {
        invalidSnapshots_++;
        quaternionFinite_ = false;
        return;
    }

    if (haveSequence_ && snapshot.sequence == lastSequence_) {
        duplicateSnapshots_++;
        return;
    }
    haveSequence_ = true;
    lastSequence_ = snapshot.sequence;
    uniqueSnapshots_++;

    const float qNorm = snapshot.q.norm();
    if (!isFinite(qNorm)) {
        invalidSnapshots_++;
        quaternionFinite_ = false;
        return;
    }
    maximumQuaternionNormError_ = std::max(
        maximumQuaternionNormError_, std::fabs(qNorm - 1.0f));

    if (haveQuaternion_) {
        maximumQuaternionStepDeg_ = std::max(
            maximumQuaternionStepDeg_,
            quaternionAngularDifferenceDeg(previousQuaternion_, snapshot.q));
    }
    previousQuaternion_ = snapshot.q;
    haveQuaternion_ = true;

    if (!snapshot.linearAccelerationValid ||
        !snapshot.linearAccelerationDeviceG.isFinite()) {
        return;
    }

    const float normG = snapshot.linearAccelerationDeviceG.norm();
    if (!isFinite(normG)) return;
    linearAccelerationValidSnapshots_++;
    linearAccelerationNormSum_ += static_cast<double>(normG);
    linearAccelerationNormSqSum_ += static_cast<double>(normG) * static_cast<double>(normG);
    maximumLinearAccelerationNorm_ = std::max(maximumLinearAccelerationNorm_, normG);
}

void SetupOutputVerificationAccumulator::pushInputSample(
    const Vec3& calibratedGyroRadS,
    const Vec3& calibratedAccelG) {
    if (!calibratedGyroRadS.isFinite() || !calibratedAccelG.isFinite()) return;
    const float accelNormG = calibratedAccelG.norm();
    if (!isFinite(accelNormG)) return;

    const Vec3 gyroDps = calibratedGyroRadS * MATH_RAD_TO_DEG;
    const float values[3] = {gyroDps.x, gyroDps.y, gyroDps.z};
    for (uint8_t i = 0; i < 3; ++i) {
        const double v = static_cast<double>(values[i]);
        gyroSum_[i] += v;
        gyroSqSum_[i] += v * v;
    }
    const double a = static_cast<double>(accelNormG);
    accelNormSum_ += a;
    accelNormSqSum_ += a * a;
    inputSamples_++;
}

SetupOutputVerificationResult SetupOutputVerificationAccumulator::finish(
    const SetupOutputVerificationConfig& config,
    bool streamHealthPassed) const {
    SetupOutputVerificationResult result;
    result.snapshots = snapshots_;
    result.uniqueSnapshots = uniqueSnapshots_;
    result.linearAccelerationValidSnapshots = linearAccelerationValidSnapshots_;
    result.invalidSnapshots = invalidSnapshots_;
    result.duplicateSnapshots = duplicateSnapshots_;
    result.inputSamples = inputSamples_;
    result.quaternionFinite = quaternionFinite_;
    result.maximumQuaternionNormError = maximumQuaternionNormError_;
    result.maximumQuaternionStepDeg = maximumQuaternionStepDeg_;
    result.linearAccelerationPeakG = maximumLinearAccelerationNorm_;
    result.streamHealthPassed = streamHealthPassed;
    result.outputAvailable = uniqueSnapshots_ >= config.minimumSnapshots;

    if (uniqueSnapshots_ > 0u) {
        result.linearAccelerationValidRatio =
            static_cast<float>(linearAccelerationValidSnapshots_) /
            static_cast<float>(uniqueSnapshots_);
    }
    if (linearAccelerationValidSnapshots_ > 0u) {
        const double count = static_cast<double>(linearAccelerationValidSnapshots_);
        result.linearAccelerationMeanG =
            static_cast<float>(linearAccelerationNormSum_ / count);
        result.linearAccelerationRmsG =
            static_cast<float>(std::sqrt(linearAccelerationNormSqSum_ / count));
    }

    if (inputSamples_ > 0u) {
        const double n = static_cast<double>(inputSamples_);
        float gyroMean[3] = {};
        float gyroMeanSe[3] = {};
        for (uint8_t i = 0; i < 3; ++i) {
            const double mean = gyroSum_[i] / n;
            const double variance = std::max(0.0, gyroSqSum_[i] / n - mean * mean);
            gyroMean[i] = static_cast<float>(mean);
            gyroMeanSe[i] = static_cast<float>(std::sqrt(variance / n));
        }
        result.inputGyroMeanDps = Vec3(gyroMean[0], gyroMean[1], gyroMean[2]);
        result.inputGyroMeanStdErrorDps = Vec3(gyroMeanSe[0], gyroMeanSe[1], gyroMeanSe[2]);
        const double accelMean = accelNormSum_ / n;
        const double accelVariance = std::max(0.0, accelNormSqSum_ / n - accelMean * accelMean);
        result.inputAccelNormMeanG = static_cast<float>(accelMean);
        result.inputAccelNormStdG = static_cast<float>(std::sqrt(accelVariance));
    }

    result.quaternionNormPassed =
        quaternionFinite_ &&
        maximumQuaternionNormError_ <= config.maximumQuaternionNormError;
    result.quaternionContinuityPassed =
        quaternionFinite_ &&
        maximumQuaternionStepDeg_ <= config.maximumQuaternionStepDeg;
    result.linearAccelerationPassed =
        result.linearAccelerationValidRatio >= config.minimumLinearAccelerationValidRatio &&
        result.linearAccelerationMeanG <= config.maximumLinearAccelerationMeanG &&
        result.linearAccelerationRmsG <= config.maximumLinearAccelerationRmsG &&
        result.linearAccelerationPeakG <= config.maximumLinearAccelerationPeakG;
    result.stationaryInputSampleCountPassed =
        inputSamples_ >= config.minimumInputSamples;
    result.stationaryGyroMeanPassed =
        result.inputGyroMeanDps.norm() <= config.maximumInputGyroMeanNormDps;
    result.stationaryGyroPrecisionPassed =
        std::fabs(result.inputGyroMeanStdErrorDps.x) <= config.maximumInputGyroMeanStdErrorAxisDps &&
        std::fabs(result.inputGyroMeanStdErrorDps.y) <= config.maximumInputGyroMeanStdErrorAxisDps &&
        std::fabs(result.inputGyroMeanStdErrorDps.z) <= config.maximumInputGyroMeanStdErrorAxisDps;
    result.stationaryAccelMeanPassed =
        std::fabs(result.inputAccelNormMeanG - 1.0f) <= config.maximumInputAccelNormMeanErrorG;
    result.stationaryAccelStdPassed =
        result.inputAccelNormStdG <= config.maximumInputAccelNormStdG;
    result.stationaryInputPassed =
        result.stationaryInputSampleCountPassed &&
        result.stationaryGyroMeanPassed &&
        result.stationaryGyroPrecisionPassed &&
        result.stationaryAccelMeanPassed &&
        result.stationaryAccelStdPassed;

    result.valid =
        result.outputAvailable &&
        result.quaternionNormPassed &&
        result.quaternionContinuityPassed &&
        result.linearAccelerationPassed &&
        result.stationaryInputPassed &&
        result.streamHealthPassed &&
        invalidSnapshots_ == 0u;
    return result;
}

} // namespace tracker
