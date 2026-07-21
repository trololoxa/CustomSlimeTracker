#pragma once

#include <Arduino.h>
#include <cstdint>

#include "sensor/imu_quality.hpp"

namespace tracker {

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
        uint32_t recoveryStableSamples = 0;
        uint32_t recoveryEnterCount = 0;
        uint32_t recoveryTiltReacquireCount = 0;
        uint32_t recoveryLastFlags = 0;
        uint64_t recoveryLastTimestampUs = 0;
    };

    void setStableSamplesRequired(uint32_t samples);
    uint32_t stableSamplesRequired() const;

    bool recoveryActive() const;
    uint32_t recoveryStableSamples() const;
    uint32_t recoveryEnterCount() const;
    uint32_t recoveryTiltReacquireCount() const;
    uint32_t recoveryLastFlags() const;
    uint64_t recoveryLastTimestampUs() const;

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
    static bool hasTimingFault(uint32_t flags);
    static bool hasAccelFault(uint32_t flags);
    static bool hasSensorFault(uint32_t flags);
    static bool hasMagDegradation(const TrackingStateInputs& in);

    uint32_t stableSamplesRequired_ = 256;
    bool recoveryActive_ = false;
    uint32_t recoveryStableSamples_ = 0;
    Vec3 recoveryAccelSum_ = Vec3::zero();
    uint32_t recoveryEnterCount_ = 0;
    uint32_t recoveryTiltReacquireCount_ = 0;
    uint32_t recoveryLastFlags_ = 0;
    uint64_t recoveryLastTimestampUs_ = 0;
};

} // namespace tracker
