#pragma once

#include <Arduino.h>
#include <stdint.h>

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

    void setStableSamplesRequired(uint32_t samples) {
        stableSamplesRequired_ = samples == 0 ? 1 : samples;
    }

    uint32_t stableSamplesRequired() const { return stableSamplesRequired_; }

    bool recoveryActive() const { return recoveryActive_; }
    uint32_t recoveryStableSamples() const { return recoveryStableSamples_; }
    uint32_t recoveryEnterCount() const { return recoveryEnterCount_; }
    uint32_t recoveryLastFlags() const { return recoveryLastFlags_; }
    uint64_t recoveryLastTimestampUs() const { return recoveryLastTimestampUs_; }

    Snapshot snapshot() const {
        Snapshot s;
        s.recoveryActive = recoveryActive_;
        s.recoveryStableSamples = recoveryStableSamples_;
        s.recoveryEnterCount = recoveryEnterCount_;
        s.recoveryLastFlags = recoveryLastFlags_;
        s.recoveryLastTimestampUs = recoveryLastTimestampUs_;
        return s;
    }

    void reset() {
        recoveryActive_ = false;
        recoveryStableSamples_ = 0;
        recoveryEnterCount_ = 0;
        recoveryLastFlags_ = 0;
        recoveryLastTimestampUs_ = 0;
    }

    void enterRecovery(uint32_t reasonFlags,
                       const char* reason,
                       uint64_t timestampUs,
                       const TrackingStateEventSink& sink) {
        const bool wasRecovering = recoveryActive_;

        recoveryActive_ = true;
        recoveryStableSamples_ = 0;
        recoveryLastFlags_ = reasonFlags;
        recoveryLastTimestampUs_ = timestampUs;

        // Entering recovery is a state transition. Repeated FIFO-recovery samples
        // while already recovering must not repeatedly reset mag/AHRS-dependent
        // state or spam Serial, otherwise diagnostics themselves can starve FIFO.
        if (!wasRecovering) {
            recoveryEnterCount_++;

            if (sink.resetOrientation) {
                sink.resetOrientation(reason, timestampUs, true, sink.resetOrientationUser);
            }

            if (sink.out) {
                sink.out->print("# TRACKING state=RECOVERING");
                if (reason && reason[0] != '\0') {
                    sink.out->print(" reason=");
                    sink.out->print(reason);
                }
                sink.out->print(" flags=0x");
                sink.out->println(reasonFlags, HEX);
            }

            if (sink.emitStateEvent) {
                sink.emitStateEvent("RECOVERING", reason, timestampUs, reasonFlags, sink.confidence, sink.emitStateEventUser);
            }
        }
    }

    void updateRecovery(const ImuQualityResult& quality,
                        uint64_t lastSampleTimestampUs,
                        const TrackingStateEventSink& sink) {
        if (!recoveryActive_) return;

        const bool stable = quality.shouldUpdateAhrs &&
                            !quality.shouldRequestFifoRecovery &&
                            !quality.has(imu_quality_flags::TIMESTAMP_ZERO) &&
                            !quality.has(imu_quality_flags::TIMESTAMP_NON_MONOTONIC) &&
                            !quality.has(imu_quality_flags::TIMESTAMP_BACKWARDS) &&
                            !quality.has(imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW) &&
                            !quality.has(imu_quality_flags::TIMESTAMP_LARGE_GAP) &&
                            !quality.has(imu_quality_flags::FIFO_OVERRUN) &&
                            !quality.has(imu_quality_flags::FIFO_FULL) &&
                            !quality.has(imu_quality_flags::FIFO_UNKNOWN_TAG);

        if (!stable) {
            recoveryStableSamples_ = 0;
            return;
        }

        recoveryStableSamples_++;
        if (recoveryStableSamples_ >= stableSamplesRequired_) {
            recoveryActive_ = false;
            recoveryStableSamples_ = 0;
            const uint64_t eventTs = quality.dtUs != 0 ? lastSampleTimestampUs : recoveryLastTimestampUs_;

            if (sink.out) {
                sink.out->println("# TRACKING state=TRACKING_6DOF reason=recovery_stable");
            }

            if (sink.emitStateEvent) {
                sink.emitStateEvent("TRACKING_6DOF", "recovery_stable", eventTs, quality.flags, quality.overallConfidence, sink.emitStateEventUser);
            }
        }
    }

    const char* stateName(bool accelCalValid,
                          bool gyroBiasValid,
                          bool qualityRecoveryRequested,
                          bool ahrsInitialized,
                          bool magHeadingReferenceValid,
                          bool magYawApplied) const {
        if (!accelCalValid || !gyroBiasValid) return "CALIBRATION_REQUIRED";
        if (recoveryActive_) return "RECOVERING";
        if (qualityRecoveryRequested) return "DEGRADED_TIMING";
        if (!ahrsInitialized) return "STARTUP_CONVERGENCE";
        if (magHeadingReferenceValid && magYawApplied) return "TRACKING_6DOF_MAG_YAW";
        return "TRACKING_6DOF";
    }

    void printRecoveryStatus(Stream& out) const {
        out.print("tracking_recovery_active="); out.println(recoveryActive_ ? "yes" : "no");
        out.print("tracking_recovery_enter_count="); out.println(recoveryEnterCount_);
        out.print("tracking_recovery_last_flags=0x"); out.println(recoveryLastFlags_, HEX);
    }

private:
    uint32_t stableSamplesRequired_ = 128;
    bool recoveryActive_ = false;
    uint32_t recoveryStableSamples_ = 0;
    uint32_t recoveryEnterCount_ = 0;
    uint32_t recoveryLastFlags_ = 0;
    uint64_t recoveryLastTimestampUs_ = 0;
};

} // namespace tracker
