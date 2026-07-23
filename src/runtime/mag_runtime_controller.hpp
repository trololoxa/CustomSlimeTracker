#pragma once

#include <Arduino.h>
#include <cstdint>

#include "connection/lsm6dsv_fifo.hpp"
#include "core/math.hpp"
#include "runtime/mag_runtime_state.hpp"
#include "sensor/mag_heading.hpp"
#include "sensor/mag_runtime.hpp"
#include "sensor/mag_yaw_correction.hpp"

namespace tracker {

class Ahrs6Dof;
class ImuQualityMonitor;
class Lsm6dsvSensorHub;
class Qmc6309;
class TrackerConfig;
class TrackerConfigStore;
class MagCalibrationCollector;

struct MagRuntimeControllerCallbacks {
    void (*resetFifoRuntime)(void* user) = nullptr;
    void* resetFifoRuntimeUser = nullptr;
    void (*requestTrackingRecovery)(uint32_t reasonFlags,
                                    const char* reason,
                                    uint64_t timestampUs,
                                    void* user) = nullptr;
    void* requestTrackingRecoveryUser = nullptr;

    void (*emitStateEvent)(const char* state,
                           const char* reason,
                           uint64_t timestampUs,
                           uint32_t flags,
                           float confidence,
                           void* user) = nullptr;
    void* emitStateEventUser = nullptr;

    void (*emitMagFrame)(const MagProcessedSample& mag,
                         const MagHeadingSample& heading,
                         const MagYawCorrectionOutput& yaw,
                         uint32_t rejectFlagsForUse,
                         bool trustedForUse,
                         void* user) = nullptr;
    void* emitMagFrameUser = nullptr;

    void (*recordStaticMagYawSample)(float magHeadingErrorDeg,
                                     const MagHeadingSample& heading,
                                     const MagYawCorrectionOutput& yaw,
                                     void* user) = nullptr;
    void* recordStaticMagYawSampleUser = nullptr;
};

struct MagRuntimeControllerDeps {
    Stream* out = nullptr;

    TrackerConfig* config = nullptr;
    TrackerConfigStore* configStore = nullptr;

    Lsm6dsvSensorHub* hub = nullptr;
    Qmc6309* qmc = nullptr;
    Lsm6dsvFifoReader* fifo = nullptr;
    ImuQualityMonitor* quality = nullptr;
    Ahrs6Dof* ahrs = nullptr;

    MagRuntimeState* state = nullptr;
    MagRuntimeProcessor* processor = nullptr;
    MagCalibrationCollector* calibrationCollector = nullptr;
    MagHeadingEstimator* headingEstimator = nullptr;
    MagHeadingReferenceState* headingRef = nullptr;
    MagHeadingAutoReferenceState* headingAutoRef = nullptr;
    MagYawCorrectionController* yawCorrection = nullptr;

    MagProcessedSample* lastProcessed = nullptr;
    MagHeadingSample* lastHeading = nullptr;
    MagYawCorrectionOutput* lastYawCorrection = nullptr;

    const float* lastOutputConfidence = nullptr;
    const bool* accelCalibrationReady = nullptr;
    const uint64_t* fallbackTimestampUs = nullptr;
    bool (*recoveryActive)(void* user) = nullptr;
    void* recoveryActiveUser = nullptr;

    float magHubPeriodUs = 0.0f;

    MagRuntimeControllerCallbacks callbacks;
};

class MagRuntimeController {
public:
    void begin(const MagRuntimeControllerDeps& deps);

    MagRuntimeConfig runtimeConfig() const;
    MagHeadingConfig headingConfig() const;
    MagYawCorrectionConfig yawConfig() const;

    float headingErrorToReferenceRad(const MagHeadingSample& heading) const;
    float headingErrorToReferenceDeg(const MagHeadingSample& heading) const;

    void resetYawCorrectionRuntime();
    void resetOrientationState(const char* reason, uint64_t timestampUs, bool rebaseAhrsTimebase);

    bool setEnabled(bool enabled, bool persist);
    // Boot-only path: FIFO parser was already configured from persisted config.
    // Starts QMC/sensor-hub streaming without a redundant FIFO reconfigure or
    // tracking recovery.
    bool startFromPreconfiguredFifo();
    bool setHeadingReference(const char* reason, bool verbose);
    void clearHeadingReference();
    bool setAutoReferenceEnabled(bool enabled);
    bool setYawCorrectionApplyEnabled(bool enabled, bool persist);

    bool startCalibration();
    void stopCalibration();
    void resetCalibration();
    bool applyCalibration(bool persist);

    void processRawSample(const Lsm6dsvFifoReader::MagRawSample& mag);

private:
    MagRuntimeControllerDeps deps_;

    Stream& stream() const;
    bool accelReady() const;
    bool isRecoveryActive() const;
    float outputConfidence() const;

    bool initSensorHub();
    void resetRuntimeCounters();
    bool applyHardwareEnabledState(bool enabled, uint64_t keepTimestampUs);
    bool reconfigureFifoForMagEnabled(bool enabled, uint64_t keepTimestampUs);

    static float magRawNorm(const Lsm6dsvFifoReader::MagRawSample& m);
    static float rampUp(float x, float bad, float good);

    void updateAutoReference(uint32_t nowMs,
                             float gyroNormDps,
                             float accelTrust,
                             bool magTrustedForUse);
    bool applyYawCorrectionToAhrs(const MagYawCorrectionOutput& yaw);
};

} // namespace tracker
