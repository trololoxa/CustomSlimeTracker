#include "sensor/fifo_calibrations.hpp"

#include <Arduino.h>

#include <cmath>

namespace tracker {

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
    if (!fifoCalibrationIoValid(io)) {
        return false;
    }

    Vec3Stats gyroStats;
    Vec3Stats accelStats;
    ScalarStats accelNormStats;

    uint32_t warmupSeen = 0;
    uint32_t accepted = 0;
    uint32_t rejected = 0;
    uint32_t total = 0;

    while (accepted < params_.requiredStationarySamples &&
           total < params_.maxTotalSamples) {
        if (!fifoCalibrationWait(io, params_.fifoWaitTimeoutMs)) {
            publishProgress(progressCb, progressUser, io, accepted, total, rejected, warmupSeen);
            continue;
        }

        bool drainedAny = false;
        const bool drainOk = fifoCalibrationDrainBounded(
            io,
            [&](const Lsm6dsv::RawSample&, const Lsm6dsv::Sample& scaled) -> bool {
                if (accepted >= params_.requiredStationarySamples ||
                    total >= params_.maxTotalSamples) {
                    return false;
                }

                total++;

                if (warmupSeen < params_.warmupSamples) {
                    warmupSeen++;
                    return true;
                }

                const float gyroNorm = scaled.gyro_rad_s.norm();
                const float accelNorm = scaled.accel_g.norm();

                const bool stationary = scaled.gyro_rad_s.isFinite() &&
                                        scaled.accel_g.isFinite() &&
                                        gyroNorm <= params_.maxGyroNormRadS &&
                                        std::fabs(accelNorm - 1.0f) <= params_.maxAccelNormErrorG;

                if (!stationary) {
                    rejected++;
                    return true;
                }

                gyroStats.push(scaled.gyro_rad_s);
                accelStats.push(scaled.accel_g);
                accelNormStats.push(accelNorm);
                accepted++;
                return true;
            },
            &drainedAny
        );

        if (!drainOk) {
            return false;
        }

        if (drainedAny) {
            publishProgress(progressCb, progressUser, io, accepted, total, rejected, warmupSeen);
        }
    }

    result.stationarySamples = accepted;
    result.totalSamples = total;
    result.success = accepted >= params_.requiredStationarySamples;

    if (accepted > 0) {
        result.gyroBiasRadS = gyroStats.mean();
        result.gyroBiasDps = result.gyroBiasRadS * MATH_RAD_TO_DEG;
        result.accelMeanG = accelStats.mean();
        result.accelNormMeanG = accelNormStats.mean();

        const Vec3 gyroVar = gyroStats.variance();
        const Vec3 accelVar = accelStats.variance();
        result.gyroNoiseNormRadS2 = gyroVar.x + gyroVar.y + gyroVar.z;
        result.accelNoiseNormG2 = accelVar.x + accelVar.y + accelVar.z;
    }

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
    p.requiredStationarySamples = params_.requiredStationarySamples;
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
    setParams(params_);
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
            &drainedAny
        );

        if (!drainOk) {
            capture_.cancel();
            return false;
        }

        if (drainedAny) {
            publishProgress(progressCb, progressUser, io);
        }
    }

    const bool ok = capture_.finish(cal_);
    publishProgress(progressCb, progressUser, io);
    return ok;
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
            &drainedAny
        );

        if (!drainOk) {
            capture_.cancel();
            return false;
        }

        if (drainedAny) {
            publishProgress(progressCb, progressUser, io);
        }
    }

    const auto snap = capture_.snapshot();
    capture_.cancel();

    result.acceptedSamples = snap.acceptedSamples;
    result.rejectedSamples = snap.rejectedSamples;
    result.meanG = snap.meanG;
    result.varianceG2 = snap.varianceG2;
    result.meanNormG = snap.meanNormG;
    result.detection = Accel6PosCalibration::detectFace(snap.meanG, params_.autoFaceDetection);
    result.detectedFace = result.detection.face;

    if (!result.detection.valid) {
        result.ambiguous = true;
        publishProgress(progressCb, progressUser, io);
        return false;
    }

    if (cal_.hasFace(result.detectedFace)) {
        result.duplicate = true;
        publishProgress(progressCb, progressUser, io);
        return false;
    }

    result.success = cal_.setFace(result.detectedFace, snap.meanG, snap.acceptedSamples, snap.varianceG2);
    publishProgress(progressCb, progressUser, io);
    return result.success;
}

bool FifoAccel6PosCalibrationRunner::compute() {
    return cal_.compute();
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
