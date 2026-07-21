#pragma once

#include <Arduino.h>
#include <cstdint>

#include "sensor/imu_quality.hpp"

namespace tracker {

enum class TrackingRecoveryReasonId : uint8_t {
    None,
    FifoQuality,
    UnreconstructableTimestampGap,
    ManualFifoReset,
    BlockingOperation,
    RuntimeReconfigure,
    Other,
};

const char* trackingRecoveryReasonName(TrackingRecoveryReasonId reason);

enum class TrackingStateId : uint8_t {
    CalibrationRequired,
    StartupConvergence,
    Tracking6Dof,
    Tracking6DofMagYaw,
    DegradedTiming,
    DegradedAccel,
    DegradedMag,
    Recovering,
    SensorFault,
};

const char* trackingStateIdName(TrackingStateId state);

// A quality "large gap" starts above the dropped-sample diagnostic threshold,
// but orientation recovery is only required when the actual dt is too large
// for AHRS gyro integration. Routine one/few-sample gaps stay observable in
// quality counters without suppressing tracking until the device is still.
bool trackingTimestampGapRequiresRecovery(const ImuQualityResult& quality,
                                          float maxAhrsDtS);

// Plain FIFO loss can continue without a stationary blackout only when the
// timestamp discontinuity is bounded. Larger losses and corrupted timestamp
// metadata retain strict recovery semantics.
bool trackingFifoLossCanUseSoftRecovery(const ImuQualityResult& quality);

// Strict recovery normally predicts with gyro only until a stable gravity
// window can restore tilt. If no quaternion exists yet, gyro-only prediction
// cannot initialize AHRS, so startup accel must remain available.
bool trackingRecoveryNeedsAhrsBootstrap(bool recoveryActive,
                                        bool ahrsInitialized);

struct TrackingStateInputs {
    bool accelCalValid = false;
    bool gyroBiasValid = false;
    bool ahrsInitialized = false;

    bool qualityRecoveryRequested = false;
    uint32_t qualityFlags = 0;

    bool sensorFault = false;

    bool magRuntimeEnabled = false;
    bool magSampleSeen = false;
    bool magTrusted = false;
    uint32_t magRejectFlags = 0;

    bool magHeadingReferenceValid = false;
    bool magYawControllerEnabled = false;
    bool magYawApplied = false;
    uint32_t magYawRejectFlags = 0;
};

struct TrackingStateEventSink {
    Stream* out = nullptr;
    float confidence = 0.0f;

    // False before the first valid AHRS orientation exists. A recovery request
    // in that state becomes ordinary startup convergence: there is no prior
    // quaternion to preserve or recover.
    bool hasRecoverableOrientation = true;

    void (*prepareRecovery)(const char* reason,
                             uint64_t timestampUs,
                             bool rebaseAhrsTimebase,
                             void* user) = nullptr;
    void* prepareRecoveryUser = nullptr;

    bool (*reacquireTilt)(const Vec3& accelG,
                          uint64_t timestampUs,
                          void* user) = nullptr;
    void* reacquireTiltUser = nullptr;

    void (*emitStateEvent)(const char* state,
                           const char* reason,
                           uint64_t timestampUs,
                           uint32_t flags,
                           float confidence,
                           void* user) = nullptr;
    void* emitStateEventUser = nullptr;
};

class TrackingStateController {
public:
    struct Snapshot {
        bool recoveryActive = false;
        bool softRecoveryActive = false;
        uint32_t recoveryStableSamples = 0;
        uint16_t softRecoveryGoodSamples = 0;
        uint8_t recoveryRejectStreak = 0;
        uint32_t recoveryEnterCount = 0;
        uint32_t recoveryTiltReacquireCount = 0;
        uint32_t recoveryBootstrapBypassCount = 0;
        uint32_t softRecoveryEnterCount = 0;
        uint32_t softRecoveryCompleteCount = 0;
        uint32_t recoveryLastFlags = 0;
        uint64_t recoveryLastTimestampUs = 0;
        TrackingRecoveryReasonId recoveryLastReason = TrackingRecoveryReasonId::None;
        uint32_t recoveryFifoQualityRequests = 0;
        uint32_t recoveryTimestampGapRequests = 0;
        uint32_t recoveryManualResetRequests = 0;
        uint32_t recoveryBlockingOperationRequests = 0;
        uint32_t recoveryRuntimeReconfigureRequests = 0;
        uint32_t recoveryOtherRequests = 0;
    };

    void setStableSamplesRequired(uint32_t samples);
    uint32_t stableSamplesRequired() const;

    bool recoveryActive() const;
    bool softRecoveryActive() const;
    uint32_t recoveryStableSamples() const;
    uint16_t softRecoveryGoodSamples() const;
    uint8_t recoveryRejectStreak() const;
    uint32_t recoveryEnterCount() const;
    uint32_t recoveryTiltReacquireCount() const;
    uint32_t recoveryBootstrapBypassCount() const;
    uint32_t softRecoveryEnterCount() const;
    uint32_t softRecoveryCompleteCount() const;
    uint32_t recoveryLastFlags() const;
    uint64_t recoveryLastTimestampUs() const;
    TrackingRecoveryReasonId recoveryLastReason() const;

    Snapshot snapshot() const;
    void reset();

    void enterRecovery(uint32_t reasonFlags,
                       const char* reason,
                       uint64_t timestampUs,
                       const TrackingStateEventSink& sink);

    void updateRecovery(const ImuQualityResult& quality,
                        const Vec3& gyroRadS,
                        const Vec3& accelG,
                        uint64_t lastSampleTimestampUs,
                        bool ahrsIntegrated,
                        const TrackingStateEventSink& sink);

    TrackingStateId evaluateState(const TrackingStateInputs& in) const;
    const char* stateName(const TrackingStateInputs& in) const;

    // Compatibility wrapper for older call sites/tests that only have the
    // original high-level booleans available.
    const char* stateName(bool accelCalValid,
                          bool gyroBiasValid,
                          bool qualityRecoveryRequested,
                          bool ahrsInitialized,
                          bool magHeadingReferenceValid,
                          bool magYawApplied) const;

    void printRecoveryStatus(Stream& out) const;

private:
    static TrackingRecoveryReasonId classifyRecoveryReason(const char* reason);
    static bool useSoftFifoRecovery(const char* reason,
                                    TrackingRecoveryReasonId reasonId,
                                    uint32_t flags);
    void countRecoveryRequest(TrackingRecoveryReasonId reason);
    void updateSoftRecovery(const ImuQualityResult& quality,
                            uint64_t timestampUs,
                            bool ahrsIntegrated,
                            const TrackingStateEventSink& sink);
    static bool hasTimingFault(uint32_t flags);
    static bool hasAccelFault(uint32_t flags);
    static bool hasSensorFault(uint32_t flags);
    static bool hasMagDegradation(const TrackingStateInputs& in);

    static constexpr uint8_t MAX_RECOVERY_REJECT_STREAK = 8;
    static constexpr uint16_t SOFT_RECOVERY_GOOD_SAMPLES_REQUIRED = 32;

    uint32_t stableSamplesRequired_ = 256;
    bool recoveryActive_ = false;
    bool softRecoveryActive_ = false;
    uint32_t recoveryStableSamples_ = 0;
    uint16_t softRecoveryGoodSamples_ = 0;
    uint8_t recoveryRejectStreak_ = 0;
    Vec3 recoveryAccelSum_ = Vec3::zero();
    uint32_t recoveryEnterCount_ = 0;
    uint32_t recoveryTiltReacquireCount_ = 0;
    uint32_t recoveryBootstrapBypassCount_ = 0;
    uint32_t softRecoveryEnterCount_ = 0;
    uint32_t softRecoveryCompleteCount_ = 0;
    uint32_t recoveryLastFlags_ = 0;
    uint64_t recoveryLastTimestampUs_ = 0;
    TrackingRecoveryReasonId recoveryLastReason_ = TrackingRecoveryReasonId::None;
    uint32_t recoveryFifoQualityRequests_ = 0;
    uint32_t recoveryTimestampGapRequests_ = 0;
    uint32_t recoveryManualResetRequests_ = 0;
    uint32_t recoveryBlockingOperationRequests_ = 0;
    uint32_t recoveryRuntimeReconfigureRequests_ = 0;
    uint32_t recoveryOtherRequests_ = 0;
};

} // namespace tracker
