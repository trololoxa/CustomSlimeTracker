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

struct FifoCalibrationIo {
    Lsm6dsv* lsm = nullptr;
    Lsm6dsvFifoReader* fifo = nullptr;

    Lsm6dsv::RawSample* rawBuffer = nullptr;
    size_t rawBufferCapacity = 0;
    uint16_t maxWordsPerDrain = 384;
    uint8_t maxDrainRoundsPerEvent = 6;

    bool (*waitForFifoEvent)(uint32_t timeoutMs, void* user) = nullptr;
    void* waitUser = nullptr;

    float latestTempC = 25.0f;
};

struct FifoDrainResult {
    bool ok = false;
    size_t count = 0;
    uint8_t rounds = 0;
};

bool fifoCalibrationIoValid(const FifoCalibrationIo& io);
void fifoCalibrationUpdateTemperature(FifoCalibrationIo& io);
bool fifoCalibrationWait(FifoCalibrationIo& io, uint32_t timeoutMs);
FifoDrainResult fifoCalibrationDrainOnce(FifoCalibrationIo& io);

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
    uint32_t validationSamples = 384;
    uint32_t maxTotalSamples = 9600;
    uint32_t warmupSamples = 64;
    uint32_t resetAfterConsecutiveRejected = 16;

    float maxGyroNormRadS = 3.0f * MATH_DEG_TO_RAD;
    float maxAccelNormErrorG = 0.08f;
    // Raw 960 Hz sample noise is allowed to be higher than the final bias
    // uncertainty. Bias quality is primarily proven by standard error of the
    // long mean and a separate held-out mean, while this remains a hard guard
    // against obvious motion/vibration.
    float maxGyroStdDps = 0.80f;
    float maxGyroMeanStdErrorDps = 0.035f;
    float maxValidationGyroMeanStdErrorDps = 0.050f;
    float maxAccelStdG = 0.025f;
    float maxTemperatureSpanC = 0.50f;
    float maxValidationBiasErrorDps = 0.08f;
    float maxValidationAccelMeanDeltaG = 0.05f;

    uint32_t fifoWaitTimeoutMs = 1000;
};

inline bool fifoGyroVecAbsAtMost(const Vec3& value, float limit) {
    return std::fabs(value.x) <= limit &&
           std::fabs(value.y) <= limit &&
           std::fabs(value.z) <= limit;
}

inline bool fifoGyroStartupCalibrationEvaluateQuality(
    GyroStartupCalibrationResult& result,
    const FifoGyroStartupCalibrationParams& params) {
    result.trainNoiseGatePassed =
        fifoGyroVecAbsAtMost(result.gyroStdDps, params.maxGyroStdDps) &&
        fifoGyroVecAbsAtMost(result.accelStdG, params.maxAccelStdG);
    result.trainMeanPrecisionGatePassed =
        fifoGyroVecAbsAtMost(result.gyroMeanStdErrorDps,
                             params.maxGyroMeanStdErrorDps);
    result.validationGatePassed = result.validationSamples >= params.validationSamples &&
        result.validationResidualDps.norm() <= params.maxValidationBiasErrorDps &&
        fifoGyroVecAbsAtMost(result.validationGyroStdDps, params.maxGyroStdDps) &&
        fifoGyroVecAbsAtMost(result.validationGyroMeanStdErrorDps,
                             params.maxValidationGyroMeanStdErrorDps) &&
        fifoGyroVecAbsAtMost(result.validationAccelStdG, params.maxAccelStdG) &&
        std::fabs(result.validationAccelNormMeanG - 1.0f) <=
            params.maxAccelNormErrorG &&
        result.validationAccelMeanDeltaG <= params.maxValidationAccelMeanDeltaG;
    result.temperatureGatePassed =
        result.temperatureSpanC <= params.maxTemperatureSpanC;

    result.success = result.stationarySamples >=
            params.requiredStationarySamples + params.validationSamples &&
        result.validationSamples >= params.validationSamples &&
        result.trainNoiseGatePassed &&
        result.trainMeanPrecisionGatePassed &&
        result.validationGatePassed &&
        result.temperatureGatePassed;
    return result.success;
}

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
    );

    const FifoGyroStartupCalibrationParams& params() const;

    bool run(FifoCalibrationIo& io,
             GyroStartupCalibrationResult& result,
             FifoGyroCalibrationProgressCallback progressCb = nullptr,
             void* progressUser = nullptr);

    static void applyResultToCalibration(const GyroStartupCalibrationResult& result,
                                         ImuCalibration& imuCal,
                                         GyroTempCompensator* tempComp = nullptr,
                                         float referenceTempC = 25.0f);

private:
    void publishProgress(FifoGyroCalibrationProgressCallback cb,
                         void* user,
                         FifoCalibrationIo& io,
                         uint32_t accepted,
                         uint32_t total,
                         uint32_t rejected,
                         uint32_t warmupSeen) const;

    FifoGyroStartupCalibrationParams params_;
};

// ============================================================
// FIFO accel 6-position calibration runner
// ============================================================

struct FifoAccel6PosCaptureParams {
    uint32_t requiredSamples = 1024;
    uint32_t validationSamples = 256;
    float maxValidationNormErrorG = 0.080f;
    float maxValidationAxisResidualG = 0.220f;
    float maxGyroNormDps = 2.0f;
    float minAccelNormG = 0.75f;
    float maxAccelNormG = 1.25f;
    uint32_t fifoWaitTimeoutMs = 1000;
    // Reset a partially-collected face after this many consecutive moving/invalid samples.
    // This gives the user time to physically move the tracker between sides without
    // blending two stable positions into one face mean.
    uint32_t resetAfterConsecutiveRejected = 8;
    Accel6PosCalibration::FaceDetectionParams autoFaceDetection;
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

struct FifoAccelAutoFaceCaptureResult {
    bool success = false;
    bool duplicate = false;
    bool ambiguous = false;
    Accel6PosCalibration::Face detectedFace = Accel6PosCalibration::Face::Invalid;
    Accel6PosCalibration::FaceDetectionResult detection;
    uint32_t acceptedSamples = 0;
    uint32_t rejectedSamples = 0;
    Vec3 meanG = Vec3::zero();
    Vec3 varianceG2 = Vec3::zero();
    float meanNormG = 0.0f;
};

using FifoAccelCaptureProgressCallback = void (*)(
    const FifoAccel6PosCaptureProgress& progress,
    void* user
);

class FifoAccel6PosCalibrationRunner {
public:
    explicit FifoAccel6PosCalibrationRunner(
        const FifoAccel6PosCaptureParams& params = FifoAccel6PosCaptureParams{}
    );

    void setParams(const FifoAccel6PosCaptureParams& params);
    const FifoAccel6PosCaptureParams& params() const;

    Accel6PosCalibration& calibration();
    const Accel6PosCalibration& calibration() const;

    void reset();

    bool captureFace(FifoCalibrationIo& io,
                     Accel6PosCalibration::Face face,
                     FifoAccelCaptureProgressCallback progressCb = nullptr,
                     void* progressUser = nullptr);

    bool captureAutoFace(FifoCalibrationIo& io,
                         FifoAccelAutoFaceCaptureResult& result,
                         FifoAccelCaptureProgressCallback progressCb = nullptr,
                         void* progressUser = nullptr);

    bool compute();
    bool applyToImuCalibration(ImuCalibration& imuCal) const;

private:
    void publishProgress(FifoAccelCaptureProgressCallback cb,
                         void* user,
                         FifoCalibrationIo& io) const;

    FifoAccel6PosCaptureParams params_;
    Accel6PosCalibration cal_;
    Accel6PosCapture capture_;
    Accel6PosCalibration::FaceData validationFaces_[6] = {};

    bool captureValidationFace(FifoCalibrationIo& io,
                               Accel6PosCalibration::Face face,
                               Accel6PosCalibration::FaceData& out,
                               FifoAccelCaptureProgressCallback progressCb,
                               void* progressUser);
};

} // namespace tracker
