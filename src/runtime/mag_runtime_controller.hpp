#pragma once

#include <Arduino.h>
#include <cstdint>

#include "connection/lsm6dsv_fifo.hpp"
#include "core/math.hpp"
#include "runtime/mag_runtime_state.hpp"
#include "sensor/mag_axis_alignment.hpp"
#include "sensor/mag_field_reliability.hpp"
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

enum MagDeferredServiceRejectFlags : uint32_t {
    MAG_DEFERRED_REJECT_NONE = 0,
    MAG_DEFERRED_REJECT_SOFTWARE_FIFO_PENDING = 1u << 0,
    MAG_DEFERRED_REJECT_HARDWARE_FIFO_STATUS = 1u << 1,
    MAG_DEFERRED_REJECT_HARDWARE_FIFO_BUSY = 1u << 2,
    MAG_DEFERRED_REJECT_OUTPUT_DEADLINE = 1u << 3,
};

struct MagDeferredServiceGate {
    bool allowed = false;
    bool fifoStatusValid = false;
    uint16_t fifoUnreadWords = 0;
    uint32_t rotationDeadlineSlackMs = 0xFFFFFFFFUL;
    uint32_t rejectFlags = MAG_DEFERRED_REJECT_NONE;
};

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
                         const MagFieldReliabilityOutput& reliability,
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

    bool (*evaluateDeferredServiceGate)(MagDeferredServiceGate& gate,
                                        void* user) = nullptr;
    void* evaluateDeferredServiceGateUser = nullptr;
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
    MagFieldReliabilityMonitor* fieldReliability = nullptr;
    MagAxisAlignmentCollector* axisAlignmentCollector = nullptr;
    MagAxisAlignmentRuntimeState* axisAlignmentState = nullptr;
    TrackerConfig* axisAlignmentCandidateWorkspace = nullptr;
    MagHeadingReferenceState* headingRef = nullptr;
    MagHeadingAutoReferenceState* headingAutoRef = nullptr;
    MagYawCorrectionController* yawCorrection = nullptr;

    MagProcessedSample* lastProcessed = nullptr;
    MagHeadingSample* lastHeading = nullptr;
    MagFieldReliabilityOutput* lastFieldReliability = nullptr;
    MagYawCorrectionOutput* lastYawCorrection = nullptr;

    const float* lastOutputConfidence = nullptr;
    const Lsm6dsv::Sample* lastCalibratedSample = nullptr;
    const uint64_t* lastImuTimestampUs = nullptr;
    const uint32_t* lastImuSampleSequence = nullptr;
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
    MagFieldReliabilityConfig fieldReliabilityConfig() const;

    float headingErrorToReferenceRad(const MagHeadingSample& heading) const;
    float headingErrorToReferenceDeg(const MagHeadingSample& heading) const;

    void resetYawCorrectionRuntime();
    void resetAxisAlignmentCandidate();
    void setAxisAlignmentLearningEnabled(bool enabled);
    bool axisAlignmentLearningEnabled() const { return axisAlignmentLearningEnabled_; }
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
    // Runs bounded solver/storage work outside the FIFO/mag sample callback.
    // Returns true when one deferred action was serviced.
    bool serviceDeferred();

private:
    MagRuntimeControllerDeps deps_;
    bool axisAlignmentLearningEnabled_ = true;

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

    void captureGyroEndpoint(MagProcessedSample& processed) const;
    void updateHeadingSnapshot(uint32_t nowMs);
    void updateFieldReliabilitySnapshot(uint32_t nowMs,
                                        float gyroNormDps,
                                        float accelTrust,
                                        bool processorTrustedForUse,
                                        MagFieldReliabilityOutput& reliability);
    void updateYawCorrectionSnapshot(uint32_t nowMs,
                                     float gyroNormDps,
                                     float accelTrust,
                                     bool magTrustedForUse,
                                     uint32_t magRejectFlagsForUse,
                                     const MagFieldReliabilityOutput& reliability);

    void updateAutoReference(uint32_t nowMs,
                             float gyroNormDps,
                             float accelTrust,
                             bool magTrustedForUse);
    void updateAxisAlignmentCandidate(uint32_t nowMs);
    bool stageAxisAlignmentCandidate(const MagAxisAlignmentResult& result, uint32_t nowMs);
    bool deferredServiceAllowed(MagDeferredServiceGate& gate) const;
    bool applyYawCorrectionToAhrs(const MagYawCorrectionOutput& yaw);
};

} // namespace tracker
