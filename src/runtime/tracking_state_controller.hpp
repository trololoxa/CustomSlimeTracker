#pragma once

#include <Arduino.h>
#include <cstdint>

#include "sensor/imu_quality.hpp"

namespace tracker {

struct TrackingStateEventSink {
    Stream* out = nullptr;
    float confidence = 0.0f;

    void (*resetOrientation)(const char* reason,
                             uint64_t timestampUs,
                             bool rebaseAhrsTimebase,
                             void* user) = nullptr;
    void* resetOrientationUser = nullptr;

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
        uint32_t recoveryLastFlags = 0;
        uint64_t recoveryLastTimestampUs = 0;
    };

    void setStableSamplesRequired(uint32_t samples);
    uint32_t stableSamplesRequired() const;

    bool recoveryActive() const;
    uint32_t recoveryStableSamples() const;
    uint32_t recoveryEnterCount() const;
    uint32_t recoveryLastFlags() const;
    uint64_t recoveryLastTimestampUs() const;

    Snapshot snapshot() const;
    void reset();

    void enterRecovery(uint32_t reasonFlags,
                       const char* reason,
                       uint64_t timestampUs,
                       const TrackingStateEventSink& sink);

    void updateRecovery(const ImuQualityResult& quality,
                        uint64_t lastSampleTimestampUs,
                        const TrackingStateEventSink& sink);

    const char* stateName(bool accelCalValid,
                          bool gyroBiasValid,
                          bool qualityRecoveryRequested,
                          bool ahrsInitialized,
                          bool magHeadingReferenceValid,
                          bool magYawApplied) const;

    void printRecoveryStatus(Stream& out) const;

private:
    uint32_t stableSamplesRequired_ = 128;
    bool recoveryActive_ = false;
    uint32_t recoveryStableSamples_ = 0;
    uint32_t recoveryEnterCount_ = 0;
    uint32_t recoveryLastFlags_ = 0;
    uint64_t recoveryLastTimestampUs_ = 0;
};

} // namespace tracker
