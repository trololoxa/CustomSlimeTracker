#include "runtime/tracking_state_controller.hpp"

#include <cmath>

#include "sensor/mag_runtime.hpp"
#include "sensor/mag_yaw_correction.hpp"
#include "runtime/tracker_console_suppress.hpp"

namespace tracker {

const char* trackingStateIdName(TrackingStateId state) {
    switch (state) {
        case TrackingStateId::CalibrationRequired: return "CALIBRATION_REQUIRED";
        case TrackingStateId::StartupConvergence: return "STARTUP_CONVERGENCE";
        case TrackingStateId::Tracking6Dof: return "TRACKING_6DOF";
        case TrackingStateId::Tracking6DofMagYaw: return "TRACKING_6DOF_MAG_YAW";
        case TrackingStateId::DegradedTiming: return "DEGRADED_TIMING";
        case TrackingStateId::DegradedAccel: return "DEGRADED_ACCEL";
        case TrackingStateId::DegradedMag: return "DEGRADED_MAG";
        case TrackingStateId::Recovering: return "RECOVERING";
        case TrackingStateId::SensorFault: return "SENSOR_FAULT";
    }
    return "UNKNOWN";
}

bool trackingTimestampGapRequiresRecovery(const ImuQualityResult& quality,
                                          float maxAhrsDtS) {
    if (!quality.has(imu_quality_flags::TIMESTAMP_LARGE_GAP)) return false;
    if (!std::isfinite(maxAhrsDtS) || maxAhrsDtS <= 0.0f) return true;
    return static_cast<float>(quality.dtUs) * 1.0e-6f > maxAhrsDtS;
}

void TrackingStateController::setStableSamplesRequired(uint32_t samples) {
    stableSamplesRequired_ = samples == 0 ? 1 : samples;
}

uint32_t TrackingStateController::stableSamplesRequired() const { return stableSamplesRequired_; }

bool TrackingStateController::recoveryActive() const { return recoveryActive_; }
uint32_t TrackingStateController::recoveryStableSamples() const { return recoveryStableSamples_; }
uint8_t TrackingStateController::recoveryRejectStreak() const { return recoveryRejectStreak_; }
uint32_t TrackingStateController::recoveryEnterCount() const { return recoveryEnterCount_; }
uint32_t TrackingStateController::recoveryTiltReacquireCount() const { return recoveryTiltReacquireCount_; }
uint32_t TrackingStateController::recoveryLastFlags() const { return recoveryLastFlags_; }
uint64_t TrackingStateController::recoveryLastTimestampUs() const { return recoveryLastTimestampUs_; }

TrackingStateController::Snapshot TrackingStateController::snapshot() const {
    Snapshot s;
    s.recoveryActive = recoveryActive_;
    s.recoveryStableSamples = recoveryStableSamples_;
    s.recoveryRejectStreak = recoveryRejectStreak_;
    s.recoveryEnterCount = recoveryEnterCount_;
    s.recoveryTiltReacquireCount = recoveryTiltReacquireCount_;
    s.recoveryLastFlags = recoveryLastFlags_;
    s.recoveryLastTimestampUs = recoveryLastTimestampUs_;
    return s;
}

void TrackingStateController::reset() {
    recoveryActive_ = false;
    recoveryStableSamples_ = 0;
    recoveryRejectStreak_ = 0;
    recoveryAccelSum_ = Vec3::zero();
    recoveryEnterCount_ = 0;
    recoveryTiltReacquireCount_ = 0;
    recoveryLastFlags_ = 0;
    recoveryLastTimestampUs_ = 0;
}

void TrackingStateController::enterRecovery(uint32_t reasonFlags,
                                            const char* reason,
                                            uint64_t timestampUs,
                                            const TrackingStateEventSink& sink) {
    const bool wasRecovering = recoveryActive_;

    recoveryActive_ = true;
    recoveryStableSamples_ = 0;
    recoveryRejectStreak_ = 0;
    recoveryAccelSum_ = Vec3::zero();
    recoveryLastFlags_ = reasonFlags;
    recoveryLastTimestampUs_ = timestampUs;

    // Entering recovery is a state transition. Repeated FIFO-recovery samples
    // while already recovering must not repeatedly reset mag/AHRS-dependent
    // state or spam Serial, otherwise diagnostics themselves can starve FIFO.
    if (!wasRecovering) {
        recoveryEnterCount_++;

        if (sink.prepareRecovery) {
            sink.prepareRecovery(reason, timestampUs, true, sink.prepareRecoveryUser);
        }

        if (sink.out && !trackerConsoleTrackingMessagesSuppressed(millis())) {
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

void TrackingStateController::updateRecovery(const ImuQualityResult& quality,
                                             const Vec3& gyroRadS,
                                             const Vec3& accelG,
                                             uint64_t lastSampleTimestampUs,
                                             const TrackingStateEventSink& sink) {
    if (!recoveryActive_) return;

    constexpr float kMaxRecoveryGyroRadS = 3.0f * MATH_DEG_TO_RAD;
    constexpr float kMaxRecoveryAccelErrorG = 0.08f;

    const bool vectorsValid = gyroRadS.isFinite() && accelG.isFinite();
    const float gyroNorm = vectorsValid ? gyroRadS.norm() : 0.0f;
    const float accelNorm = quality.accelNormValid ? quality.accelNormG : accelG.norm();
    const bool motionStable = vectorsValid &&
                              std::isfinite(gyroNorm) &&
                              gyroNorm <= kMaxRecoveryGyroRadS &&
                              std::isfinite(accelNorm) &&
                              std::fabs(accelNorm - 1.0f) <= kMaxRecoveryAccelErrorG;
    const bool hardStreamFault = quality.shouldRequestFifoRecovery ||
                                 quality.has(imu_quality_flags::TIMESTAMP_BACKWARDS) ||
                                 quality.has(imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW) ||
                                 quality.has(imu_quality_flags::FIFO_OVERRUN) ||
                                 quality.has(imu_quality_flags::FIFO_FULL) ||
                                 quality.has(imu_quality_flags::FIFO_UNKNOWN_TAG);
    const bool stable = motionStable &&
                        !hardStreamFault &&
                        quality.shouldUpdateAhrs &&
                        quality.shouldUseAccelCorrection &&
                        quality.accelConfidence >= 0.75f;

    if (!stable) {
        // Real motion or a broken FIFO invalidates the accumulated gravity
        // direction immediately.  Only isolated unusable timestamp/quality
        // samples are tolerated, so a busy Wi-Fi loop cannot make recovery
        // impossible without blending two physical orientations together.
        if (!motionStable || hardStreamFault) {
            recoveryStableSamples_ = 0;
            recoveryRejectStreak_ = 0;
            recoveryAccelSum_ = Vec3::zero();
            return;
        }
        if (recoveryRejectStreak_ < 0xffu) ++recoveryRejectStreak_;
        if (recoveryRejectStreak_ > MAX_RECOVERY_REJECT_STREAK) {
            recoveryStableSamples_ = 0;
            recoveryAccelSum_ = Vec3::zero();
        }
        return;
    }

    recoveryRejectStreak_ = 0;
    recoveryAccelSum_ += accelG;
    recoveryStableSamples_++;
    if (recoveryStableSamples_ < stableSamplesRequired_) return;

    const Vec3 meanAccel = recoveryAccelSum_ / static_cast<float>(recoveryStableSamples_);
    const float meanAccelNorm = meanAccel.norm();
    if (!std::isfinite(meanAccelNorm) ||
        std::fabs(meanAccelNorm - 1.0f) > kMaxRecoveryAccelErrorG ||
        !sink.reacquireTilt ||
        !sink.reacquireTilt(meanAccel, lastSampleTimestampUs, sink.reacquireTiltUser)) {
        recoveryStableSamples_ = 0;
        recoveryRejectStreak_ = 0;
        recoveryAccelSum_ = Vec3::zero();
        return;
    }

    recoveryActive_ = false;
    recoveryStableSamples_ = 0;
    recoveryRejectStreak_ = 0;
    recoveryAccelSum_ = Vec3::zero();
    recoveryTiltReacquireCount_++;
    const uint64_t eventTs = quality.dtUs != 0 ? lastSampleTimestampUs : recoveryLastTimestampUs_;

    if (sink.out && !trackerConsoleTrackingMessagesSuppressed(millis())) {
        sink.out->println("# TRACKING state=TRACKING_6DOF reason=recovery_stable");
    }

    if (sink.emitStateEvent) {
        sink.emitStateEvent("TRACKING_6DOF", "recovery_stable", eventTs, quality.flags, quality.overallConfidence, sink.emitStateEventUser);
    }
}

bool TrackingStateController::hasTimingFault(uint32_t flags) {
    constexpr uint32_t mask = imu_quality_flags::TIMESTAMP_ZERO |
                              imu_quality_flags::TIMESTAMP_NON_MONOTONIC |
                              imu_quality_flags::TIMESTAMP_LARGE_GAP |
                              imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW |
                              imu_quality_flags::TIMESTAMP_META_MISMATCH |
                              imu_quality_flags::TIMESTAMP_BACKWARDS |
                              imu_quality_flags::FIFO_OVERRUN |
                              imu_quality_flags::FIFO_FULL |
                              imu_quality_flags::FIFO_UNKNOWN_TAG |
                              imu_quality_flags::FIFO_ORPHAN_WORDS |
                              imu_quality_flags::FIFO_TAG_COUNTER_JUMP |
                              imu_quality_flags::FIFO_GYRO_TAG_COUNTER_JUMP |
                              imu_quality_flags::FIFO_ACCEL_TAG_COUNTER_JUMP |
                              imu_quality_flags::FIFO_RECOVERY_REQUESTED |
                              imu_quality_flags::SAMPLE_DROPPED_BEFORE |
                              imu_quality_flags::SAMPLE_NOT_AHRS_USABLE;
    return (flags & mask) != 0;
}

bool TrackingStateController::hasAccelFault(uint32_t flags) {
    constexpr uint32_t mask = imu_quality_flags::ACCEL_SATURATED |
                              imu_quality_flags::ACCEL_NEAR_SATURATION |
                              imu_quality_flags::ACCEL_NORM_OUTLIER |
                              imu_quality_flags::ACCEL_NOT_AHRS_USABLE;
    return (flags & mask) != 0;
}

bool TrackingStateController::hasSensorFault(uint32_t flags) {
    constexpr uint32_t mask = imu_quality_flags::GYRO_SATURATED |
                              imu_quality_flags::GYRO_NEAR_SATURATION;
    return (flags & mask) != 0;
}

bool TrackingStateController::hasMagDegradation(const TrackingStateInputs& in) {
    if (!in.magRuntimeEnabled) return false;
    if (in.magSampleSeen && !in.magTrusted) return true;

    constexpr uint32_t magHardRejects = MAG_REJECT_RAW_SATURATED |
                                        MAG_REJECT_RAW_NONFINITE |
                                        MAG_REJECT_NOT_CALIBRATED |
                                        MAG_REJECT_AXIS_NOT_ALIGNED |
                                        MAG_REJECT_NORM_TOO_LOW |
                                        MAG_REJECT_NORM_TOO_HIGH |
                                        MAG_REJECT_STALE |
                                        MAG_REJECT_ZERO_NORM;
    if ((in.magRejectFlags & magHardRejects) != 0) return true;

    constexpr uint32_t yawMagRejects = MAG_YAW_REJECT_HEADING_INVALID |
                                       MAG_YAW_REJECT_MAG_NOT_TRUSTED |
                                       MAG_YAW_REJECT_MAG_STALE |
                                       MAG_YAW_REJECT_HORIZONTAL_BAD |
                                       MAG_YAW_REJECT_INNOVATION_TOO_LARGE |
                                       MAG_YAW_REJECT_NONFINITE;
    if (in.magYawControllerEnabled && (in.magYawRejectFlags & yawMagRejects) != 0) return true;

    return false;
}

TrackingStateId TrackingStateController::evaluateState(const TrackingStateInputs& in) const {
    if (!in.accelCalValid || !in.gyroBiasValid) return TrackingStateId::CalibrationRequired;
    if (recoveryActive_) return TrackingStateId::Recovering;
    if (in.sensorFault || hasSensorFault(in.qualityFlags)) return TrackingStateId::SensorFault;
    if (in.qualityRecoveryRequested || hasTimingFault(in.qualityFlags)) return TrackingStateId::DegradedTiming;
    if (hasAccelFault(in.qualityFlags)) return TrackingStateId::DegradedAccel;
    if (!in.ahrsInitialized) return TrackingStateId::StartupConvergence;
    if (hasMagDegradation(in)) return TrackingStateId::DegradedMag;
    if (in.magHeadingReferenceValid && in.magYawApplied) return TrackingStateId::Tracking6DofMagYaw;
    return TrackingStateId::Tracking6Dof;
}

const char* TrackingStateController::stateName(const TrackingStateInputs& in) const {
    return trackingStateIdName(evaluateState(in));
}

const char* TrackingStateController::stateName(bool accelCalValid,
                                               bool gyroBiasValid,
                                               bool qualityRecoveryRequested,
                                               bool ahrsInitialized,
                                               bool magHeadingReferenceValid,
                                               bool magYawApplied) const {
    TrackingStateInputs in;
    in.accelCalValid = accelCalValid;
    in.gyroBiasValid = gyroBiasValid;
    in.qualityRecoveryRequested = qualityRecoveryRequested;
    in.ahrsInitialized = ahrsInitialized;
    in.magHeadingReferenceValid = magHeadingReferenceValid;
    in.magYawApplied = magYawApplied;
    return stateName(in);
}

void TrackingStateController::printRecoveryStatus(Stream& out) const {
    out.print("tracking_recovery_active="); out.println(recoveryActive_ ? "yes" : "no");
    out.print("tracking_recovery_stable_samples="); out.println(recoveryStableSamples_);
    out.print("tracking_recovery_reject_streak="); out.println(recoveryRejectStreak_);
    out.print("tracking_recovery_enter_count="); out.println(recoveryEnterCount_);
    out.print("tracking_recovery_tilt_reacquire_count="); out.println(recoveryTiltReacquireCount_);
    out.print("tracking_recovery_last_flags=0x"); out.println(recoveryLastFlags_, HEX);
}

} // namespace tracker
