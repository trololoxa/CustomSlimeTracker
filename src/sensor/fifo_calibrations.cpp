#include "sensor/fifo_calibrations.hpp"

#include <Arduino.h>

#include <cmath>

namespace tracker {

namespace {

Vec3 standardError(const Vec3& stddev, uint32_t samples) {
    if (samples == 0u) return Vec3(999.0f, 999.0f, 999.0f);
    const float scale = 1.0f / std::sqrt(static_cast<float>(samples));
    return stddev * scale;
}

} // namespace

namespace {

struct ScalarStats {
    uint32_t count = 0;
    double sum = 0.0;
    double sumSq = 0.0;
    float minValue = 0.0f;
    float maxValue = 0.0f;

    void push(float x) {
        if (!std::isfinite(x)) return;
        if (count == 0) {
            minValue = x;
            maxValue = x;
        } else {
            if (x < minValue) minValue = x;
            if (x > maxValue) maxValue = x;
        }
        count++;
        sum += static_cast<double>(x);
        sumSq += static_cast<double>(x) * static_cast<double>(x);
    }

    float mean() const {
        return count == 0 ? 0.0f : static_cast<float>(sum / static_cast<double>(count));
    }
};

struct Vec3Stats {
    uint32_t count = 0;
    Vec3 sum = Vec3::zero();
    Vec3 sumSq = Vec3::zero();

    void push(const Vec3& v) {
        if (!v.isFinite()) return;
        count++;
        sum += v;
        sumSq += hadamard(v, v);
    }

    Vec3 mean() const {
        return count == 0 ? Vec3::zero() : sum / static_cast<float>(count);
    }

    Vec3 variance() const {
        if (count < 2) return Vec3::zero();
        const Vec3 m = mean();
        Vec3 v = sumSq / static_cast<float>(count);
        v -= hadamard(m, m);
        if (v.x < 0.0f) v.x = 0.0f;
        if (v.y < 0.0f) v.y = 0.0f;
        if (v.z < 0.0f) v.z = 0.0f;
        return v;
    }
};

} // namespace

bool fifoCalibrationIoValid(const FifoCalibrationIo& io) {
    return io.lsm != nullptr &&
           io.fifo != nullptr &&
           io.rawBuffer != nullptr &&
           io.rawBufferCapacity > 0 &&
           io.waitForFifoEvent != nullptr;
}

void fifoCalibrationUpdateTemperature(FifoCalibrationIo& io) {
    if (!io.fifo) return;

    const auto& fs = io.fifo->stats();
    if (fs.latestTempValid) {
        io.latestTempC = fs.latestTempC;
    }
}

bool fifoCalibrationWait(FifoCalibrationIo& io, uint32_t timeoutMs) {
    if (!fifoCalibrationIoValid(io)) return false;
    return io.waitForFifoEvent(timeoutMs, io.waitUser);
}

FifoDrainResult fifoCalibrationDrainOnce(FifoCalibrationIo& io) {
    FifoDrainResult r;
    if (!fifoCalibrationIoValid(io)) return r;

    const uint64_t drainTimestampUs = micros();
    r.ok = io.fifo->drainRawSamples(
        io.rawBuffer,
        io.rawBufferCapacity,
        r.count,
        drainTimestampUs,
        io.maxWordsPerDrain
    );

    fifoCalibrationUpdateTemperature(io);
    return r;
}

FifoGyroStartupCalibrator::FifoGyroStartupCalibrator(
    const FifoGyroStartupCalibrationParams& params
) : params_(params) {}

const FifoGyroStartupCalibrationParams& FifoGyroStartupCalibrator::params() const {
    return params_;
}

bool FifoGyroStartupCalibrator::run(FifoCalibrationIo& io,
                                    GyroStartupCalibrationResult& result,
                                    FifoGyroCalibrationProgressCallback progressCb,
                                    void* progressUser) {
    result = GyroStartupCalibrationResult{};
    if (!fifoCalibrationIoValid(io)) return false;

    Vec3Stats trainGyro;
    Vec3Stats trainAccel;
    ScalarStats trainAccelNorm;
    ScalarStats trainTemp;
    Vec3Stats validationGyro;
    Vec3Stats validationAccel;
    ScalarStats validationTemp;

    uint32_t warmupSeen = 0;
    uint32_t accepted = 0;
    uint32_t rejected = 0;
    uint32_t consecutiveRejected = 0;
    uint32_t total = 0;
    const uint32_t targetAccepted =
        params_.requiredStationarySamples + params_.validationSamples;

    auto resetContinuousWindow = [&]() {
        trainGyro = Vec3Stats{};
        trainAccel = Vec3Stats{};
        trainAccelNorm = ScalarStats{};
        trainTemp = ScalarStats{};
        validationGyro = Vec3Stats{};
        validationAccel = Vec3Stats{};
        validationTemp = ScalarStats{};
        accepted = 0;
        warmupSeen = 0;
        consecutiveRejected = 0;
    };

    while (accepted < targetAccepted && total < params_.maxTotalSamples) {
        if (!fifoCalibrationWait(io, params_.fifoWaitTimeoutMs)) {
            publishProgress(progressCb, progressUser, io, accepted, total, rejected, warmupSeen);
            continue;
        }

        bool drainedAny = false;
        const bool drainOk = fifoCalibrationDrainBounded(
            io,
            [&](const Lsm6dsv::RawSample&, const Lsm6dsv::Sample& scaled) -> bool {
                if (accepted >= targetAccepted || total >= params_.maxTotalSamples) return false;
                total++;

                if (warmupSeen < params_.warmupSamples) {
                    warmupSeen++;
                    return true;
                }

                const float gyroNorm = scaled.gyro_rad_s.norm();
                const float accelNorm = scaled.accel_g.norm();
                const bool stationary = scaled.gyro_rad_s.isFinite() &&
                    scaled.accel_g.isFinite() && std::isfinite(scaled.temp_c) &&
                    gyroNorm <= params_.maxGyroNormRadS &&
                    std::fabs(accelNorm - 1.0f) <= params_.maxAccelNormErrorG;

                if (!stationary) {
                    rejected++;
                    consecutiveRejected++;
                    if (accepted != 0u &&
                        params_.resetAfterConsecutiveRejected != 0u &&
                        consecutiveRejected >= params_.resetAfterConsecutiveRejected) {
                        resetContinuousWindow();
                    }
                    return true;
                }

                consecutiveRejected = 0;
                if (accepted < params_.requiredStationarySamples) {
                    trainGyro.push(scaled.gyro_rad_s);
                    trainAccel.push(scaled.accel_g);
                    trainAccelNorm.push(accelNorm);
                    trainTemp.push(scaled.temp_c);
                } else {
                    validationGyro.push(scaled.gyro_rad_s);
                    validationAccel.push(scaled.accel_g);
                    validationTemp.push(scaled.temp_c);
                }
                accepted++;
                return true;
            },
            &drainedAny);

        if (!drainOk) return false;
        if (drainedAny) {
            publishProgress(progressCb, progressUser, io, accepted, total, rejected, warmupSeen);
        }
    }

    result.stationarySamples = accepted;
    result.validationSamples = validationGyro.count;
    result.totalSamples = total;
    if (trainGyro.count == 0u) return false;

    result.gyroBiasRadS = trainGyro.mean();
    result.gyroBiasDps = result.gyroBiasRadS * MATH_RAD_TO_DEG;
    result.accelMeanG = trainAccel.mean();
    result.accelNormMeanG = trainAccelNorm.mean();

    const Vec3 gyroVar = trainGyro.variance();
    const Vec3 accelVar = trainAccel.variance();
    result.gyroNoiseNormRadS2 = gyroVar.x + gyroVar.y + gyroVar.z;
    result.accelNoiseNormG2 = accelVar.x + accelVar.y + accelVar.z;
    result.gyroStdDps = Vec3(
        std::sqrt(gyroVar.x), std::sqrt(gyroVar.y), std::sqrt(gyroVar.z)) * MATH_RAD_TO_DEG;
    result.gyroMeanStdErrorDps = standardError(result.gyroStdDps, trainGyro.count);
    result.accelStdG = Vec3(
        std::sqrt(accelVar.x), std::sqrt(accelVar.y), std::sqrt(accelVar.z));
    float tempMin = trainTemp.count == 0u ? 0.0f : trainTemp.minValue;
    float tempMax = trainTemp.count == 0u ? 0.0f : trainTemp.maxValue;
    if (validationTemp.count != 0u) {
        if (trainTemp.count == 0u) {
            tempMin = validationTemp.minValue;
            tempMax = validationTemp.maxValue;
        } else {
            tempMin = std::min(tempMin, validationTemp.minValue);
            tempMax = std::max(tempMax, validationTemp.maxValue);
        }
    }
    result.temperatureSpanC = tempMax - tempMin;
    result.validationResidualDps =
        (validationGyro.mean() - result.gyroBiasRadS) * MATH_RAD_TO_DEG;
    result.validationAccelMeanDeltaG =
        (validationAccel.mean() - result.accelMeanG).norm();

    const Vec3 validationGyroVar = validationGyro.variance();
    const Vec3 validationAccelVar = validationAccel.variance();
    result.validationGyroStdDps = Vec3(
        std::sqrt(validationGyroVar.x) * MATH_RAD_TO_DEG,
        std::sqrt(validationGyroVar.y) * MATH_RAD_TO_DEG,
        std::sqrt(validationGyroVar.z) * MATH_RAD_TO_DEG);
    result.validationGyroMeanStdErrorDps =
        standardError(result.validationGyroStdDps, validationGyro.count);
    result.validationAccelStdG = Vec3(
        std::sqrt(validationAccelVar.x),
        std::sqrt(validationAccelVar.y),
        std::sqrt(validationAccelVar.z));
    result.validationAccelNormMeanG = validationAccel.mean().norm();

    // The shared evaluator applies params_.maxTemperatureSpanC together with
    // the fit/held-out precision and vibration gates.
    (void)fifoGyroStartupCalibrationEvaluateQuality(result, params_);
    return result.success;
}

void FifoGyroStartupCalibrator::applyResultToCalibration(
    const GyroStartupCalibrationResult& result,
    ImuCalibration& imuCal,
    GyroTempCompensator* tempComp,
    float referenceTempC
) {
    if (!result.success) return;

    imuCal.gyroBiasRadS = result.gyroBiasRadS;
    imuCal.gyroBiasValid = true;

    if (tempComp) {
        tempComp->setStaticBias(result.gyroBiasRadS, referenceTempC);
    }
}

void FifoGyroStartupCalibrator::publishProgress(FifoGyroCalibrationProgressCallback cb,
                                                void* user,
                                                FifoCalibrationIo& io,
                                                uint32_t accepted,
                                                uint32_t total,
                                                uint32_t rejected,
                                                uint32_t warmupSeen) const {
    if (!cb) return;

    FifoGyroStartupCalibrationProgress p;
    p.stationarySamples = accepted;
    p.totalSamples = total;
    p.rejectedSamples = rejected;
    p.warmupSamples = warmupSeen;
    p.requiredStationarySamples = params_.requiredStationarySamples + params_.validationSamples;
    p.latestTempC = io.latestTempC;
    cb(p, user);
}

FifoAccel6PosCalibrationRunner::FifoAccel6PosCalibrationRunner(
    const FifoAccel6PosCaptureParams& params
) {
    setParams(params);
}

void FifoAccel6PosCalibrationRunner::setParams(const FifoAccel6PosCaptureParams& params) {
    params_ = params;

    Accel6PosCapture::Params p;
    p.requiredSamples = params.requiredSamples;
    p.maxGyroNormDps = params.maxGyroNormDps;
    p.minAccelNormG = params.minAccelNormG;
    p.maxAccelNormG = params.maxAccelNormG;
    p.resetAfterConsecutiveRejected = params.resetAfterConsecutiveRejected;
    capture_ = Accel6PosCapture(p);
}

const FifoAccel6PosCaptureParams& FifoAccel6PosCalibrationRunner::params() const {
    return params_;
}

Accel6PosCalibration& FifoAccel6PosCalibrationRunner::calibration() {
    return cal_;
}

const Accel6PosCalibration& FifoAccel6PosCalibrationRunner::calibration() const {
    return cal_;
}

void FifoAccel6PosCalibrationRunner::reset() {
    cal_.reset();
    for (auto& face : validationFaces_) face = Accel6PosCalibration::FaceData{};
    setParams(params_);
}

bool FifoAccel6PosCalibrationRunner::captureValidationFace(
    FifoCalibrationIo& io,
    Accel6PosCalibration::Face face,
    Accel6PosCalibration::FaceData& out,
    FifoAccelCaptureProgressCallback progressCb,
    void* progressUser) {
    out = Accel6PosCalibration::FaceData{};
    if (params_.validationSamples == 0u) return false;

    Accel6PosCapture::Params validationParams;
    validationParams.requiredSamples = params_.validationSamples;
    validationParams.maxGyroNormDps = params_.maxGyroNormDps;
    validationParams.minAccelNormG = params_.minAccelNormG;
    validationParams.maxAccelNormG = params_.maxAccelNormG;
    validationParams.resetAfterConsecutiveRejected = params_.resetAfterConsecutiveRejected;
    Accel6PosCapture validationCapture(validationParams);
    validationCapture.begin(face);

    while (!validationCapture.done()) {
        if (!fifoCalibrationWait(io, params_.fifoWaitTimeoutMs)) continue;
        const bool drainOk = fifoCalibrationDrainBounded(
            io,
            [&](const Lsm6dsv::RawSample&, const Lsm6dsv::Sample& scaled) -> bool {
                validationCapture.push(scaled);
                return !validationCapture.done();
            });
        if (!drainOk) {
            validationCapture.cancel();
            return false;
        }
        // Re-use the normal progress callback so the user sees that the tracker
        // is still collecting; the setup flow does not require another prompt.
        (void)progressCb;
        (void)progressUser;
    }

    const auto snap = validationCapture.snapshot();
    validationCapture.cancel();
    out.valid = snap.acceptedSamples >= params_.validationSamples;
    out.samples = snap.acceptedSamples;
    out.meanG = snap.meanG;
    out.varianceG2 = snap.varianceG2;
    out.meanNormG = snap.meanNormG;
    return out.valid;
}

bool FifoAccel6PosCalibrationRunner::captureFace(FifoCalibrationIo& io,
                                                 Accel6PosCalibration::Face face,
                                                 FifoAccelCaptureProgressCallback progressCb,
                                                 void* progressUser) {
    if (!fifoCalibrationIoValid(io)) return false;
    if (face == Accel6PosCalibration::Face::Invalid) return false;

    capture_.begin(face);
    publishProgress(progressCb, progressUser, io);
    while (!capture_.done()) {
        if (!fifoCalibrationWait(io, params_.fifoWaitTimeoutMs)) {
            publishProgress(progressCb, progressUser, io);
            continue;
        }
        bool drainedAny = false;
        const bool drainOk = fifoCalibrationDrainBounded(
            io,
            [&](const Lsm6dsv::RawSample&, const Lsm6dsv::Sample& scaled) -> bool {
                capture_.push(scaled);
                return !capture_.done();
            },
            &drainedAny);
        if (!drainOk) {
            capture_.cancel();
            return false;
        }
        if (drainedAny) publishProgress(progressCb, progressUser, io);
    }

    const auto train = capture_.snapshot();
    capture_.cancel();
    Accel6PosCalibration::FaceData validation;
    if (!captureValidationFace(io, face, validation, progressCb, progressUser)) return false;
    const uint8_t idx = static_cast<uint8_t>(face);
    validationFaces_[idx] = validation;
    return cal_.setFace(face, train.meanG, train.acceptedSamples, train.varianceG2);
}

bool FifoAccel6PosCalibrationRunner::captureAutoFace(FifoCalibrationIo& io,
                                                     FifoAccelAutoFaceCaptureResult& result,
                                                     FifoAccelCaptureProgressCallback progressCb,
                                                     void* progressUser) {
    result = FifoAccelAutoFaceCaptureResult{};
    if (!fifoCalibrationIoValid(io)) return false;

    capture_.begin(Accel6PosCalibration::Face::Invalid);
    publishProgress(progressCb, progressUser, io);
    while (!capture_.done()) {
        if (!fifoCalibrationWait(io, params_.fifoWaitTimeoutMs)) {
            publishProgress(progressCb, progressUser, io);
            continue;
        }
        bool drainedAny = false;
        const bool drainOk = fifoCalibrationDrainBounded(
            io,
            [&](const Lsm6dsv::RawSample&, const Lsm6dsv::Sample& scaled) -> bool {
                capture_.push(scaled);
                return !capture_.done();
            },
            &drainedAny);
        if (!drainOk) {
            capture_.cancel();
            return false;
        }
        if (drainedAny) publishProgress(progressCb, progressUser, io);
    }

    const auto train = capture_.snapshot();
    capture_.cancel();
    result.acceptedSamples = train.acceptedSamples;
    result.rejectedSamples = train.rejectedSamples;
    result.meanG = train.meanG;
    result.varianceG2 = train.varianceG2;
    result.meanNormG = train.meanNormG;
    result.detection = Accel6PosCalibration::detectFace(train.meanG, params_.autoFaceDetection);
    result.detectedFace = result.detection.face;

    if (!result.detection.valid) {
        result.ambiguous = true;
        return false;
    }
    if (cal_.hasFace(result.detectedFace)) {
        result.duplicate = true;
        return false;
    }

    Accel6PosCalibration::FaceData validation;
    if (!captureValidationFace(io, result.detectedFace, validation, progressCb, progressUser)) {
        return false;
    }
    const auto validationDetection =
        Accel6PosCalibration::detectFace(validation.meanG, params_.autoFaceDetection);
    if (!validationDetection.valid || validationDetection.face != result.detectedFace) {
        result.ambiguous = true;
        return false;
    }

    validationFaces_[static_cast<uint8_t>(result.detectedFace)] = validation;
    result.success = cal_.setFace(
        result.detectedFace, train.meanG, train.acceptedSamples, train.varianceG2);
    return result.success;
}

bool FifoAccel6PosCalibrationRunner::compute() {
    if (!cal_.compute()) return false;
    const auto& fit = cal_.result();
    for (uint8_t i = 0; i < 6u; ++i) {
        const auto& heldOut = validationFaces_[i];
        if (!heldOut.valid || heldOut.samples < params_.validationSamples) {
            cal_.addQualityFlag(accel_cal_quality_flags::INDEPENDENT_VALIDATION_FAILED);
            return false;
        }
        const auto face = static_cast<Accel6PosCalibration::Face>(i);
        const Vec3 calibrated = Accel6PosCalibration::apply(heldOut.meanG, fit);
        const float normError = std::fabs(calibrated.norm() - 1.0f);
        const float axisResidual =
            (calibrated - Accel6PosCalibration::expectedVector(face)).norm();
        if (normError > params_.maxValidationNormErrorG ||
            axisResidual > params_.maxValidationAxisResidualG) {
            cal_.addQualityFlag(accel_cal_quality_flags::INDEPENDENT_VALIDATION_FAILED);
            return false;
        }
    }
    return true;
}

bool FifoAccel6PosCalibrationRunner::applyToImuCalibration(ImuCalibration& imuCal) const {
    const auto& r = cal_.result();
    if (!r.valid) return false;

    imuCal.accelBiasG = r.biasG;
    imuCal.accelScale = r.scaleMatrix;
    imuCal.accelCalValid = true;
    return true;
}

void FifoAccel6PosCalibrationRunner::publishProgress(FifoAccelCaptureProgressCallback cb,
                                                     void* user,
                                                     FifoCalibrationIo& io) const {
    if (!cb) return;

    const auto s = capture_.snapshot();
    FifoAccel6PosCaptureProgress p;
    p.active = s.active;
    p.face = s.face;
    p.acceptedSamples = s.acceptedSamples;
    p.rejectedSamples = s.rejectedSamples;
    p.requiredSamples = s.requiredSamples;
    p.meanG = s.meanG;
    p.varianceG2 = s.varianceG2;
    p.meanNormG = s.meanNormG;
    p.latestTempC = io.latestTempC;
    cb(p, user);
}

} // namespace tracker
