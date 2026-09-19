#include "sensor/fifo_calibrations.hpp"

#include <Arduino.h>

#include <cmath>

namespace tracker {

namespace {

bool captureCancelled(const FifoCalibrationIo& io) {
    return io.cancelRequested && io.cancelRequested(io.cancelUser);
}

bool validCaptureParams(const FifoGyroStartupCalibrationParams& p) {
    const uint64_t needed = static_cast<uint64_t>(p.requiredStationarySamples) + p.validationSamples + p.warmupSamples;
    if (p.requiredStationarySamples == 0u || p.validationSamples == 0u ||
        needed > p.maxTotalSamples || p.maxTotalSamples > 1000000u) return false;
    const float limits[] = {p.maxGyroNormRadS, p.maxAccelNormErrorG, p.maxGyroStdDps,
        p.maxGyroMeanStdErrorDps, p.maxValidationGyroMeanStdErrorDps, p.maxAccelStdG,
        p.maxTemperatureSpanC, p.maxValidationBiasErrorDps, p.maxValidationAccelMeanDeltaG};
    for (float value : limits) if (!std::isfinite(value) || value <= 0.0f) return false;
    return true;
}

bool validCaptureParams(const FifoAccel6PosCaptureParams& p) {
    return p.requiredSamples > 0u && p.requiredSamples <= 65535u &&
        p.validationSamples > 0u && p.validationSamples <= 65535u &&
        std::isfinite(p.maxGyroNormDps) && p.maxGyroNormDps > 0.0f &&
        std::isfinite(p.minAccelNormG) && p.minAccelNormG > 0.0f &&
        std::isfinite(p.maxAccelNormG) && p.maxAccelNormG > p.minAccelNormG &&
        std::isfinite(p.maxValidationNormErrorG) && p.maxValidationNormErrorG > 0.0f &&
        std::isfinite(p.maxValidationAxisResidualG) && p.maxValidationAxisResidualG > 0.0f;
}


Vec3 standardError(const Vec3& stddev, uint32_t samples) {
    if (samples == 0u) return Vec3(999.0f, 999.0f, 999.0f);
    const float scale = 1.0f / std::sqrt(static_cast<float>(samples));
    return stddev * scale;
}

} // namespace

const char* fifoCalibrationCaptureStatusName(FifoCalibrationCaptureStatus status) {
    switch (status) {
        case FifoCalibrationCaptureStatus::Idle: return "idle";
        case FifoCalibrationCaptureStatus::Completed: return "completed";
        case FifoCalibrationCaptureStatus::InvalidIo: return "invalid_io";
        case FifoCalibrationCaptureStatus::InvalidRequest: return "invalid_request";
        case FifoCalibrationCaptureStatus::Cancelled: return "cancelled";
        case FifoCalibrationCaptureStatus::DeadlineExceeded: return "deadline_exceeded";
        case FifoCalibrationCaptureStatus::SensorUnavailable: return "sensor_unavailable";
        case FifoCalibrationCaptureStatus::DrainFailed: return "drain_failed";
        case FifoCalibrationCaptureStatus::SampleBudgetExhausted: return "sample_budget_exhausted";
        case FifoCalibrationCaptureStatus::QualityRejected: return "quality_rejected";
    }
    return "unknown";
}

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
    lastStatus_ = FifoCalibrationCaptureStatus::Idle;
    if (!fifoCalibrationIoValid(io)) {
        lastStatus_ = FifoCalibrationCaptureStatus::InvalidIo;
        return false;
    }

    if (!validCaptureParams(params_)) {
        lastStatus_ = FifoCalibrationCaptureStatus::InvalidRequest;
        return false;
    }
    FifoCalibrationCaptureSession session(io, lastStatus_, params_.maximumCaptureMs,
        params_.fifoWaitTimeoutMs, params_.maximumConsecutiveWaitTimeouts);
    if (!session.check()) return false;

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
        if (!session.wait()) {
            resetContinuousWindow();
            if (session.failed()) { return false; }
            continue;
        }

        bool drainedAny = false;
        const bool drainOk = fifoCalibrationDrainBounded(
            io,
            [&](const Lsm6dsv::RawSample& raw, const Lsm6dsv::Sample& scaled) -> bool {
                if (!session.acceptFresh(raw)) { resetContinuousWindow(); return true; }
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

        if (!drainOk) {
            resetContinuousWindow();
            if (!session.recoverDrain()) { return false; }
            continue;
        }
        if (!session.service()) { return false; }
        if (drainedAny) {
            publishProgress(progressCb, progressUser, io, accepted, total, rejected, warmupSeen);
        }
    }

    if (!session.check()) { return false; }
    result.stationarySamples = accepted;
    result.validationSamples = validationGyro.count;
    result.totalSamples = total;
    if (trainGyro.count == 0u) {
        lastStatus_ = FifoCalibrationCaptureStatus::SampleBudgetExhausted;
        return false;
    }

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
    if (!session.check()) { result.success = false; return false; }
    lastStatus_ = result.success
        ? FifoCalibrationCaptureStatus::Completed
        : (total >= params_.maxTotalSamples
            ? FifoCalibrationCaptureStatus::SampleBudgetExhausted
            : FifoCalibrationCaptureStatus::QualityRejected);
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
    lastStatus_ = FifoCalibrationCaptureStatus::Idle;
}

uint32_t FifoCalibrationCaptureSession::now() const {
    return io_.clockMs ? io_.clockMs(io_.clockUser) : millis();
}

FifoCalibrationCaptureSession::FifoCalibrationCaptureSession(
    FifoCalibrationIo& io, FifoCalibrationCaptureStatus& status,
    uint32_t maximumMs, uint32_t waitMs, uint8_t timeoutLimit)
    : io_(io), status_(status), startedMs_(now()), maximumMs_(maximumMs),
      waitMs_(waitMs), timeoutLimit_(timeoutLimit) {
    status_ = FifoCalibrationCaptureStatus::Idle;
    if (!fifoCalibrationIoValid(io_)) {
        status_ = FifoCalibrationCaptureStatus::InvalidIo;
    } else if (maximumMs == 0u || maximumMs > 120000u || waitMs == 0u ||
               waitMs > 1000u || timeoutLimit == 0u || io.maxDrainRoundsPerEvent == 0u) {
        status_ = FifoCalibrationCaptureStatus::InvalidRequest;
    } else if (check()) {
        owned_ = !io_.serviceCapture || io_.serviceCapture(FifoCalibrationService::Begin, io_.serviceUser);
        if (!owned_) status_ = FifoCalibrationCaptureStatus::SensorUnavailable;
    }
}

FifoCalibrationCaptureSession::~FifoCalibrationCaptureSession() {
    if (owned_ && io_.serviceCapture) {
        (void)io_.serviceCapture(FifoCalibrationService::End, io_.serviceUser);
    }
}

bool FifoCalibrationCaptureSession::check() {
    if (failed()) return false;
    if (captureCancelled(io_)) status_ = FifoCalibrationCaptureStatus::Cancelled;
    else if (now() - startedMs_ >= maximumMs_) status_ = FifoCalibrationCaptureStatus::DeadlineExceeded;
    return !failed();
}

bool FifoCalibrationCaptureSession::service() {
    io_.fifo->discardMagDuringCalibrationCapture();
    if (io_.serviceCapture && !io_.serviceCapture(FifoCalibrationService::Progress, io_.serviceUser)) {
        status_ = FifoCalibrationCaptureStatus::SensorUnavailable;
    }
    return check();
}

bool FifoCalibrationCaptureSession::wait() {
    if (!check()) return false;
    // Bound both wall time and attempts, even if an adapter returns early.
    uint32_t remaining = waitMs_;
    while (remaining != 0u) {
        const uint32_t deadlineRemaining = maximumMs_ - (now() - startedMs_);
        const uint32_t slice = std::min<uint32_t>(20u, std::min(remaining, deadlineRemaining));
        const bool ready = fifoCalibrationWait(io_, slice);
        if (!service()) return false;
        if (ready) { timeouts_ = 0u; return true; }
        remaining -= slice;
    }
    if (++timeouts_ >= timeoutLimit_) status_ = FifoCalibrationCaptureStatus::SensorUnavailable;
    return false;
}

bool FifoCalibrationCaptureSession::acceptFresh(const Lsm6dsv::RawSample& raw) {
    const float period = io_.fifo->samplePeriodUs();
    const bool fresh = raw.t_us != 0u && raw.components == Lsm6dsv::SAMPLE_COMPONENT_COMPLETE &&
        raw.coherency == Lsm6dsv::SampleCoherency::Coherent && fifoCalibrationSampleFlagsAcceptable(raw.flags) &&
        (lastSampleUs_ == 0u || (raw.t_us > lastSampleUs_ &&
            static_cast<double>(raw.t_us - lastSampleUs_) <= static_cast<double>(period) * 4.0));
    if (raw.t_us > lastSampleUs_) lastSampleUs_ = raw.t_us;
    return fresh;
}

bool FifoCalibrationCaptureSession::recoverDrain() {
    if (!check()) return false;
    const uint64_t anchor = io_.fifo->stats().lastAssignedTimestampUs;
    if (++drainRecoveries_ > 2u || !io_.fifo->resetFifo()) {
        status_ = FifoCalibrationCaptureStatus::DrainFailed;
        return false;
    }
    io_.fifo->resetTimestampReconstruction(anchor);
    lastSampleUs_ = 0u;
    return service();
}

bool FifoAccel6PosCalibrationRunner::captureValidationFace(
    FifoCalibrationIo& io,
    FifoCalibrationCaptureSession& session,
    Accel6PosCalibration::Face face,
    Accel6PosCalibration::FaceData& out,
    FifoAccelCaptureProgressCallback progressCb,
    void* progressUser) {
    out = Accel6PosCalibration::FaceData{};
    if (params_.validationSamples == 0u) {
        lastStatus_ = FifoCalibrationCaptureStatus::InvalidRequest;
        return false;
    }

    Accel6PosCapture::Params validationParams;
    validationParams.requiredSamples = params_.validationSamples;
    validationParams.maxGyroNormDps = params_.maxGyroNormDps;
    validationParams.minAccelNormG = params_.minAccelNormG;
    validationParams.maxAccelNormG = params_.maxAccelNormG;
    validationParams.resetAfterConsecutiveRejected = params_.resetAfterConsecutiveRejected;
    Accel6PosCapture validationCapture(validationParams);
    validationCapture.begin(face);


    while (!validationCapture.done()) {
        if (!session.wait()) {
            validationCapture.begin(face);
            if (session.failed()) { validationCapture.cancel(); return false; }
            continue;
        }
        const bool drainOk = fifoCalibrationDrainBounded(
            io,
            [&](const Lsm6dsv::RawSample& raw, const Lsm6dsv::Sample& scaled) -> bool {
                if (!session.acceptFresh(raw)) { validationCapture.begin(face); return true; }
                validationCapture.push(scaled);
                return !validationCapture.done();
            });
        if (!drainOk) {
            validationCapture.begin(face);
            if (!session.recoverDrain()) { validationCapture.cancel(); return false; }
            continue;
        }
        if (!session.service()) { validationCapture.cancel(); return false; }
        if (progressCb) {
            const auto evidence = validationCapture.snapshot();
            FifoAccel6PosCaptureProgress progress;
            progress.active = true;
            progress.face = face;
            progress.acceptedSamples = evidence.acceptedSamples;
            progress.rejectedSamples = evidence.rejectedSamples;
            progress.requiredSamples = params_.validationSamples;
            progress.meanG = evidence.meanG;
            progress.varianceG2 = evidence.varianceG2;
            progress.meanNormG = evidence.meanG.norm();
            progress.latestTempC = io.latestTempC;
            progressCb(progress, progressUser);
        }
    }

    if (!session.check()) { validationCapture.cancel(); return false; }
    const auto snap = validationCapture.snapshot();
    validationCapture.cancel();
    out.valid = snap.acceptedSamples >= params_.validationSamples;
    out.samples = snap.acceptedSamples;
    out.meanG = snap.meanG;
    out.varianceG2 = snap.varianceG2;
    out.meanNormG = snap.meanNormG;
    lastStatus_ = out.valid
        ? FifoCalibrationCaptureStatus::Idle
        : FifoCalibrationCaptureStatus::QualityRejected;
    return out.valid;
}

bool FifoAccel6PosCalibrationRunner::captureFace(FifoCalibrationIo& io,
                                                 Accel6PosCalibration::Face face,
                                                 FifoAccelCaptureProgressCallback progressCb,
                                                 void* progressUser) {
    lastStatus_ = FifoCalibrationCaptureStatus::Idle;
    if (!fifoCalibrationIoValid(io)) {
        lastStatus_ = FifoCalibrationCaptureStatus::InvalidIo;
        return false;
    }
    if (static_cast<uint8_t>(face) >= 6u) {
        lastStatus_ = FifoCalibrationCaptureStatus::InvalidRequest;
        return false;
    }

    if (!validCaptureParams(params_)) {
        lastStatus_ = FifoCalibrationCaptureStatus::InvalidRequest;
        return false;
    }
    FifoCalibrationCaptureSession session(io, lastStatus_, params_.maximumCaptureMs,
        params_.fifoWaitTimeoutMs, params_.maximumConsecutiveWaitTimeouts);
    if (!session.check()) return false;

    capture_.begin(face);
    publishProgress(progressCb, progressUser, io);
    while (!capture_.done()) {
        if (!session.wait()) {
            capture_.begin(face);
            if (session.failed()) { capture_.cancel(); return false; }
            continue;
        }
        bool drainedAny = false;
        const bool drainOk = fifoCalibrationDrainBounded(
            io,
            [&](const Lsm6dsv::RawSample& raw, const Lsm6dsv::Sample& scaled) -> bool {
                if (!session.acceptFresh(raw)) { capture_.begin(face); return true; }
                capture_.push(scaled);
                return !capture_.done();
            },
            &drainedAny);
        if (!drainOk) {
            capture_.begin(face);
            if (!session.recoverDrain()) { capture_.cancel(); return false; }
            continue;
        }
        if (!session.service()) { capture_.cancel(); return false; }
        if (drainedAny) publishProgress(progressCb, progressUser, io);
    }

    const auto train = capture_.snapshot();
    capture_.cancel();
    Accel6PosCalibration::FaceData validation;
    if (!captureValidationFace(io, session, face, validation, progressCb, progressUser)) return false;
    if (!session.check()) { capture_.cancel(); return false; }
    const uint8_t idx = static_cast<uint8_t>(face);
    validationFaces_[idx] = validation;
    const bool stored = cal_.setFace(face, train.meanG, train.acceptedSamples, train.varianceG2);
    lastStatus_ = stored
        ? FifoCalibrationCaptureStatus::Completed
        : FifoCalibrationCaptureStatus::QualityRejected;
    return stored;
}

bool FifoAccel6PosCalibrationRunner::captureAutoFace(FifoCalibrationIo& io,
                                                     FifoAccelAutoFaceCaptureResult& result,
                                                     FifoAccelCaptureProgressCallback progressCb,
                                                     void* progressUser) {
    result = FifoAccelAutoFaceCaptureResult{};
    lastStatus_ = FifoCalibrationCaptureStatus::Idle;
    if (!fifoCalibrationIoValid(io)) {
        lastStatus_ = FifoCalibrationCaptureStatus::InvalidIo;
        return false;
    }

    if (!validCaptureParams(params_)) {
        lastStatus_ = FifoCalibrationCaptureStatus::InvalidRequest;
        return false;
    }
    FifoCalibrationCaptureSession session(io, lastStatus_, params_.maximumCaptureMs,
        params_.fifoWaitTimeoutMs, params_.maximumConsecutiveWaitTimeouts);
    if (!session.check()) return false;

    capture_.begin(Accel6PosCalibration::Face::Invalid);
    publishProgress(progressCb, progressUser, io);
    while (!capture_.done()) {
        if (!session.wait()) {
            capture_.begin(Accel6PosCalibration::Face::Invalid);
            if (session.failed()) { capture_.cancel(); return false; }
            continue;
        }
        bool drainedAny = false;
        const bool drainOk = fifoCalibrationDrainBounded(
            io,
            [&](const Lsm6dsv::RawSample& raw, const Lsm6dsv::Sample& scaled) -> bool {
                if (!session.acceptFresh(raw)) { capture_.begin(Accel6PosCalibration::Face::Invalid); return true; }
                capture_.push(scaled);
                return !capture_.done();
            },
            &drainedAny);
        if (!drainOk) {
            capture_.begin(Accel6PosCalibration::Face::Invalid);
            if (!session.recoverDrain()) { capture_.cancel(); return false; }
            continue;
        }
        if (!session.service()) { capture_.cancel(); return false; }
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
        lastStatus_ = FifoCalibrationCaptureStatus::QualityRejected;
        return false;
    }
    if (cal_.hasFace(result.detectedFace)) {
        result.duplicate = true;
        lastStatus_ = FifoCalibrationCaptureStatus::QualityRejected;
        return false;
    }

    Accel6PosCalibration::FaceData validation;
    if (!captureValidationFace(io, session, result.detectedFace, validation, progressCb, progressUser)) {
        return false;
    }
    const auto validationDetection =
        Accel6PosCalibration::detectFace(validation.meanG, params_.autoFaceDetection);
    if (!validationDetection.valid || validationDetection.face != result.detectedFace) {
        result.ambiguous = true;
        lastStatus_ = FifoCalibrationCaptureStatus::QualityRejected;
        return false;
    }

    if (!session.check()) return false;
    validationFaces_[static_cast<uint8_t>(result.detectedFace)] = validation;
    result.success = cal_.setFace(
        result.detectedFace, train.meanG, train.acceptedSamples, train.varianceG2);
    lastStatus_ = result.success
        ? FifoCalibrationCaptureStatus::Completed
        : FifoCalibrationCaptureStatus::QualityRejected;
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
