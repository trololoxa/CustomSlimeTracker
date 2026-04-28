#pragma once

#include <cstddef>
#include <cstdint>
#include <cmath>

#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/calibration.hpp"
#include "sensor/accel_6pos_calibration.hpp"
#include "sensor/gyro_temperature_compensation.hpp"

namespace tracker {

// ============================================================
// FIFO-based calibration helpers
// ============================================================
// This file moves calibration logic out of main.cpp.
// It is designed for the new FIFO v2 path:
//   - FIFO INT1 watermark / overrun / full
//   - hardware FIFO timestamp tags
//   - FIFO temperature tags
//   - bounded FIFO drain loops
//
// Main.cpp still owns:
//   - GPIO interrupt ISR
//   - wait-for-FIFO-event callback
//   - printing callbacks, if desired
// ============================================================

struct FifoCalibrationIo {
    Lsm6dsv* lsm = nullptr;
    Lsm6dsvFifoReader* fifo = nullptr;

    Lsm6dsv::RawSample* rawBuffer = nullptr;
    size_t rawBufferCapacity = 0;
    uint16_t maxWordsPerDrain = 384;
    uint8_t maxDrainRoundsPerEvent = 6;

    // Return true when FIFO should be drained. Usually this waits for INT1.
    // It may also fallback to FIFO_STATUS if an interrupt was missed.
    bool (*waitForFifoEvent)(uint32_t timeoutMs, void* user) = nullptr;
    void* waitUser = nullptr;

    float latestTempC = 25.0f;
};

struct FifoDrainResult {
    bool ok = false;
    size_t count = 0;
    uint8_t rounds = 0;
};

inline bool fifoCalibrationIoValid(const FifoCalibrationIo& io) {
    return io.lsm != nullptr &&
           io.fifo != nullptr &&
           io.rawBuffer != nullptr &&
           io.rawBufferCapacity > 0 &&
           io.waitForFifoEvent != nullptr;
}

inline void fifoCalibrationUpdateTemperature(FifoCalibrationIo& io) {
    if (!io.fifo) return;

    const auto& fs = io.fifo->stats();
    if (fs.latestTempValid) {
        io.latestTempC = fs.latestTempC;
    }
}

inline bool fifoCalibrationWait(FifoCalibrationIo& io, uint32_t timeoutMs) {
    if (!fifoCalibrationIoValid(io)) return false;
    return io.waitForFifoEvent(timeoutMs, io.waitUser);
}

inline FifoDrainResult fifoCalibrationDrainOnce(FifoCalibrationIo& io) {
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

template <typename SampleCallback>
bool fifoCalibrationDrainBounded(FifoCalibrationIo& io,
                                 SampleCallback onSample,
                                 bool* drainedAny = nullptr) {
    if (drainedAny) *drainedAny = false;
    if (!fifoCalibrationIoValid(io)) return false;

    for (uint8_t round = 0; round < io.maxDrainRoundsPerEvent; ++round) {
        FifoDrainResult dr = fifoCalibrationDrainOnce(io);
        if (!dr.ok) return false;
        if (dr.count == 0) break;

        if (drainedAny) *drainedAny = true;

        for (size_t i = 0; i < dr.count; ++i) {
            Lsm6dsv::Sample scaled = io.lsm->scale(io.rawBuffer[i]);
            scaled.temp_c = io.latestTempC;
            if (!onSample(io.rawBuffer[i], scaled)) {
                return true;
            }
        }
    }

    return true;
}

// ============================================================
// FIFO gyro startup calibration
// ============================================================

struct FifoGyroStartupCalibrationParams {
    uint32_t requiredStationarySamples = 1536;
    uint32_t maxTotalSamples = 9600;
    uint32_t warmupSamples = 64;

    float maxGyroNormRadS = 3.0f * MATH_DEG_TO_RAD;
    float maxAccelNormErrorG = 0.08f;

    uint32_t fifoWaitTimeoutMs = 1000;
};

struct FifoGyroStartupCalibrationProgress {
    uint32_t stationarySamples = 0;
    uint32_t totalSamples = 0;
    uint32_t rejectedSamples = 0;
    uint32_t warmupSamples = 0;
    uint32_t requiredStationarySamples = 0;
    float latestTempC = 25.0f;

    uint32_t fifoIntCount = 0;
    uint32_t fifoIntMissed = 0;
};

using FifoGyroCalibrationProgressCallback = void (*)(
    const FifoGyroStartupCalibrationProgress& progress,
    void* user
);

class FifoGyroStartupCalibrator {
public:
    explicit FifoGyroStartupCalibrator(
        const FifoGyroStartupCalibrationParams& params = FifoGyroStartupCalibrationParams{}
    ) : params_(params) {}

    const FifoGyroStartupCalibrationParams& params() const {
        return params_;
    }

    bool run(FifoCalibrationIo& io,
             GyroStartupCalibrationResult& result,
             FifoGyroCalibrationProgressCallback progressCb = nullptr,
             void* progressUser = nullptr) {
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

    static void applyResultToCalibration(const GyroStartupCalibrationResult& result,
                                         ImuCalibration& imuCal,
                                         GyroTempCompensator* tempComp = nullptr,
                                         float referenceTempC = 25.0f) {
        if (!result.success) return;

        imuCal.gyroBiasRadS = result.gyroBiasRadS;
        imuCal.gyroBiasValid = true;

        if (tempComp) {
            tempComp->reset(result.gyroBiasRadS, referenceTempC);
        }
    }

private:
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

    void publishProgress(FifoGyroCalibrationProgressCallback cb,
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

    FifoGyroStartupCalibrationParams params_;
};

// ============================================================
// FIFO accel 6-position calibration runner
// ============================================================

struct FifoAccel6PosCaptureParams {
    uint32_t requiredSamples = 1024;
    float maxGyroNormDps = 2.0f;
    float minAccelNormG = 0.75f;
    float maxAccelNormG = 1.25f;
    uint32_t fifoWaitTimeoutMs = 1000;
};

struct FifoAccel6PosCaptureProgress {
    bool active = false;
    Accel6PosCalibration::Face face = Accel6PosCalibration::Face::Invalid;
    uint32_t acceptedSamples = 0;
    uint32_t rejectedSamples = 0;
    uint32_t requiredSamples = 0;
    Vec3 meanG = Vec3::zero();
    Vec3 varianceG2 = Vec3::zero();
    float meanNormG = 0.0f;
    float latestTempC = 25.0f;
};

using FifoAccelCaptureProgressCallback = void (*)(
    const FifoAccel6PosCaptureProgress& progress,
    void* user
);

class FifoAccel6PosCalibrationRunner {
public:
    explicit FifoAccel6PosCalibrationRunner(
        const FifoAccel6PosCaptureParams& params = FifoAccel6PosCaptureParams{}
    ) {
        setParams(params);
    }

    void setParams(const FifoAccel6PosCaptureParams& params) {
        params_ = params;

        Accel6PosCapture::Params p;
        p.requiredSamples = params.requiredSamples;
        p.maxGyroNormDps = params.maxGyroNormDps;
        p.minAccelNormG = params.minAccelNormG;
        p.maxAccelNormG = params.maxAccelNormG;
        capture_ = Accel6PosCapture(p);
    }

    const FifoAccel6PosCaptureParams& params() const {
        return params_;
    }

    Accel6PosCalibration& calibration() {
        return cal_;
    }

    const Accel6PosCalibration& calibration() const {
        return cal_;
    }

    void reset() {
        cal_.reset();
        setParams(params_);
    }

    bool captureFace(FifoCalibrationIo& io,
                     Accel6PosCalibration::Face face,
                     FifoAccelCaptureProgressCallback progressCb = nullptr,
                     void* progressUser = nullptr) {
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

    bool compute() {
        return cal_.compute();
    }

    bool applyToImuCalibration(ImuCalibration& imuCal) const {
        const auto& r = cal_.result();
        if (!r.valid) return false;

        imuCal.accelBiasG = r.biasG;
        imuCal.accelScale = r.scaleMatrix;
        imuCal.accelCalValid = true;
        return true;
    }

private:
    void publishProgress(FifoAccelCaptureProgressCallback cb,
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

    FifoAccel6PosCaptureParams params_;
    Accel6PosCalibration cal_;
    Accel6PosCapture capture_;
};

} // namespace tracker
