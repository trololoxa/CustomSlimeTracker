#include "runtime/tracking_state_controller.hpp"

#include <cmath>
#include <cstring>

#include "sensor/mag_runtime.hpp"
#include "sensor/mag_yaw_correction.hpp"
#include "runtime/tracker_console_suppress.hpp"

namespace tracker {

const char* trackingRecoveryReasonName(TrackingRecoveryReasonId reason) {
    switch (reason) {
        case TrackingRecoveryReasonId::None: return "none";
        case TrackingRecoveryReasonId::FifoQuality: return "fifo_quality";
        case TrackingRecoveryReasonId::UnreconstructableTimestampGap: return "unreconstructable_timestamp_gap";
        case TrackingRecoveryReasonId::ManualFifoReset: return "manual_fifo_reset";
        case TrackingRecoveryReasonId::BlockingOperation: return "blocking_operation";
        case TrackingRecoveryReasonId::RuntimeReconfigure: return "runtime_reconfigure";
        case TrackingRecoveryReasonId::Other: return "other";
    }
    return "other";
}

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

bool trackingRecoveryNeedsAhrsBootstrap(bool recoveryActive,
                                        bool ahrsInitialized) {
    return recoveryActive && !ahrsInitialized;
}

bool trackingFifoLossCanUseSoftRecovery(const ImuQualityResult& quality) {
    constexpr uint32_t fifoLossMask = imu_quality_flags::FIFO_OVERRUN |
                                      imu_quality_flags::FIFO_FULL;
    constexpr uint32_t strictFaultMask = imu_quality_flags::TIMESTAMP_BACKWARDS |
                                         imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW |
                                         imu_quality_flags::FIFO_UNKNOWN_TAG;
    constexpr uint32_t maxSoftGapUs = 250000u;
    constexpr uint32_t maxSoftDroppedSamples = 256u;

    if ((quality.flags & fifoLossMask) == 0u ||
        (quality.flags & strictFaultMask) != 0u) {
        return false;
    }
    if (quality.dtUs > maxSoftGapUs ||
        quality.estimatedDroppedBefore > maxSoftDroppedSamples) {
        return false;
    }
    return true;
}

void TrackingStateController::setStableSamplesRequired(uint32_t samples) {
    stableSamplesRequired_ = samples == 0 ? 1 : samples;
}

uint32_t TrackingStateController::stableSamplesRequired() const { return stableSamplesRequired_; }

bool TrackingStateController::recoveryActive() const { return recoveryActive_; }
bool TrackingStateController::softRecoveryActive() const { return softRecoveryActive_; }
uint32_t TrackingStateController::recoveryStableSamples() const { return recoveryStableSamples_; }
uint16_t TrackingStateController::softRecoveryGoodSamples() const { return softRecoveryGoodSamples_; }
uint8_t TrackingStateController::recoveryRejectStreak() const { return recoveryRejectStreak_; }
uint32_t TrackingStateController::recoveryEnterCount() const { return recoveryEnterCount_; }
uint32_t TrackingStateController::recoveryTiltReacquireCount() const { return recoveryTiltReacquireCount_; }
uint32_t TrackingStateController::recoveryBootstrapBypassCount() const { return recoveryBootstrapBypassCount_; }
uint32_t TrackingStateController::softRecoveryEnterCount() const { return softRecoveryEnterCount_; }
uint32_t TrackingStateController::softRecoveryCompleteCount() const { return softRecoveryCompleteCount_; }
uint32_t TrackingStateController::recoveryLastFlags() const { return recoveryLastFlags_; }
uint64_t TrackingStateController::recoveryLastTimestampUs() const { return recoveryLastTimestampUs_; }
TrackingRecoveryReasonId TrackingStateController::recoveryLastReason() const { return recoveryLastReason_; }

TrackingStateController::Snapshot TrackingStateController::snapshot() const {
    Snapshot s;
    s.recoveryActive = recoveryActive_;
    s.softRecoveryActive = softRecoveryActive_;
    s.recoveryStableSamples = recoveryStableSamples_;
    s.softRecoveryGoodSamples = softRecoveryGoodSamples_;
    s.recoveryRejectStreak = recoveryRejectStreak_;
    s.recoveryEnterCount = recoveryEnterCount_;
    s.recoveryTiltReacquireCount = recoveryTiltReacquireCount_;
    s.recoveryBootstrapBypassCount = recoveryBootstrapBypassCount_;
    s.softRecoveryEnterCount = softRecoveryEnterCount_;
    s.softRecoveryCompleteCount = softRecoveryCompleteCount_;
    s.recoveryLastFlags = recoveryLastFlags_;
    s.recoveryLastTimestampUs = recoveryLastTimestampUs_;
    s.recoveryLastReason = recoveryLastReason_;
    s.recoveryFifoQualityRequests = recoveryFifoQualityRequests_;
    s.recoveryTimestampGapRequests = recoveryTimestampGapRequests_;
    s.recoveryManualResetRequests = recoveryManualResetRequests_;
    s.recoveryBlockingOperationRequests = recoveryBlockingOperationRequests_;
    s.recoveryRuntimeReconfigureRequests = recoveryRuntimeReconfigureRequests_;
    s.recoveryOtherRequests = recoveryOtherRequests_;
    return s;
}

void TrackingStateController::reset() {
    recoveryActive_ = false;
    softRecoveryActive_ = false;
    recoveryStableSamples_ = 0;
    softRecoveryGoodSamples_ = 0;
    recoveryRejectStreak_ = 0;
    recoveryAccelSum_ = Vec3::zero();
    recoveryEnterCount_ = 0;
    recoveryTiltReacquireCount_ = 0;
    recoveryBootstrapBypassCount_ = 0;
    softRecoveryEnterCount_ = 0;
    softRecoveryCompleteCount_ = 0;
    recoveryLastFlags_ = 0;
    recoveryLastTimestampUs_ = 0;
    recoveryLastReason_ = TrackingRecoveryReasonId::None;
    recoveryFifoQualityRequests_ = 0;
    recoveryTimestampGapRequests_ = 0;
    recoveryManualResetRequests_ = 0;
    recoveryBlockingOperationRequests_ = 0;
    recoveryRuntimeReconfigureRequests_ = 0;
    recoveryOtherRequests_ = 0;
}

TrackingRecoveryReasonId TrackingStateController::classifyRecoveryReason(const char* reason) {
    if (reason == nullptr || reason[0] == '\0') return TrackingRecoveryReasonId::Other;
    if (std::strcmp(reason, "fifo_recovery") == 0 ||
        std::strcmp(reason, "fifo_recovery_soft") == 0) {
        return TrackingRecoveryReasonId::FifoQuality;
    }
    if (std::strcmp(reason, "unreconstructable_dt_gap") == 0) return TrackingRecoveryReasonId::UnreconstructableTimestampGap;
    if (std::strcmp(reason, "manual_fifo_reset") == 0) return TrackingRecoveryReasonId::ManualFifoReset;
    if (std::strcmp(reason, "blocking_wifi_scan") == 0) return TrackingRecoveryReasonId::BlockingOperation;
    if (std::strcmp(reason, "imu_fifo_reconfigure") == 0 ||
        std::strcmp(reason, "mag_fifo_reconfigure") == 0) {
        return TrackingRecoveryReasonId::RuntimeReconfigure;
    }
    return TrackingRecoveryReasonId::Other;
}

bool TrackingStateController::useSoftFifoRecovery(const char* reason,
                                                  TrackingRecoveryReasonId reasonId,
                                                  uint32_t flags) {
    if (reasonId != TrackingRecoveryReasonId::FifoQuality || reason == nullptr ||
        std::strcmp(reason, "fifo_recovery_soft") != 0) {
        return false;
    }

    constexpr uint32_t fifoLossMask = imu_quality_flags::FIFO_OVERRUN |
                                      imu_quality_flags::FIFO_FULL;
    constexpr uint32_t strictFaultMask = imu_quality_flags::TIMESTAMP_BACKWARDS |
                                         imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW |
                                         imu_quality_flags::FIFO_UNKNOWN_TAG;
    return (flags & fifoLossMask) != 0u && (flags & strictFaultMask) == 0u;
}

void TrackingStateController::countRecoveryRequest(TrackingRecoveryReasonId reason) {
    switch (reason) {
        case TrackingRecoveryReasonId::FifoQuality: recoveryFifoQualityRequests_++; break;
        case TrackingRecoveryReasonId::UnreconstructableTimestampGap: recoveryTimestampGapRequests_++; break;
        case TrackingRecoveryReasonId::ManualFifoReset: recoveryManualResetRequests_++; break;
        case TrackingRecoveryReasonId::BlockingOperation: recoveryBlockingOperationRequests_++; break;
        case TrackingRecoveryReasonId::RuntimeReconfigure: recoveryRuntimeReconfigureRequests_++; break;
        case TrackingRecoveryReasonId::None: break;
        case TrackingRecoveryReasonId::Other: recoveryOtherRequests_++; break;
    }
}

void TrackingStateController::enterRecovery(uint32_t reasonFlags,
                                            const char* reason,
                                            uint64_t timestampUs,
                                            const TrackingStateEventSink& sink) {
    const TrackingRecoveryReasonId reasonId = classifyRecoveryReason(reason);
    countRecoveryRequest(reasonId);

    recoveryLastFlags_ = reasonFlags;
    recoveryLastTimestampUs_ = timestampUs;
    recoveryLastReason_ = reasonId;

    // A startup-time FIFO/magnetometer reconfiguration may request recovery
    // before AHRS has produced its first quaternion. There is no orientation
    // to preserve in that state, and strict gyro-only recovery would deadlock:
    // an uninitialized AHRS requires gravity to initialize. Keep diagnostics,
    // clear dependent state, and let normal STARTUP_CONVERGENCE own bootstrap.
    if (!sink.hasRecoverableOrientation) {
        recoveryActive_ = false;
        softRecoveryActive_ = false;
        recoveryStableSamples_ = 0;
        softRecoveryGoodSamples_ = 0;
        recoveryRejectStreak_ = 0;
        recoveryAccelSum_ = Vec3::zero();
        recoveryBootstrapBypassCount_++;

        if (sink.prepareRecovery) {
            sink.prepareRecovery(reason, timestampUs, false, sink.prepareRecoveryUser);
        }
        if (sink.emitStateEvent) {
            sink.emitStateEvent("STARTUP_CONVERGENCE",
                                "recovery_before_first_orientation",
                                timestampUs,
                                reasonFlags,
                                sink.confidence,
                                sink.emitStateEventUser);
        }
        return;
    }

    // A plain FIFO full/overrun means samples were lost, but stopping all
    // output until the user becomes motionless does not reconstruct the lost
    // yaw and can create a minutes-long blackout. Preserve the current
    // quaternion, rebase time, reset dependent magnetic state, and let normal
    // adaptive accel correction repair tilt while output resumes on the next
    // genuinely integrated sample. Timestamp corruption and explicit blocking
    // operations remain strict recovery events.
    if (useSoftFifoRecovery(reason, reasonId, reasonFlags) && !recoveryActive_) {
        const bool wasSoftRecovering = softRecoveryActive_;
        softRecoveryActive_ = true;
        softRecoveryGoodSamples_ = 0;

        // Every FIFO reset establishes a new timestamp stream, including a
        // repeated loss during the same soft-recovery episode.
        if (sink.prepareRecovery) {
            sink.prepareRecovery(reason, timestampUs, true, sink.prepareRecoveryUser);
        }

        if (!wasSoftRecovering) {
            softRecoveryEnterCount_++;
            if (sink.out && !trackerConsoleTrackingMessagesSuppressed(millis())) {
                sink.out->print("# TRACKING state=DEGRADED_TIMING reason=fifo_soft_recovery flags=0x");
                sink.out->println(reasonFlags, HEX);
            }
            if (sink.emitStateEvent) {
                sink.emitStateEvent("DEGRADED_TIMING",
                                    "fifo_soft_recovery",
                                    timestampUs,
                                    reasonFlags,
                                    sink.confidence,
                                    sink.emitStateEventUser);
            }
        }
        return;
    }

    const bool wasRecovering = recoveryActive_;
    softRecoveryActive_ = false;
    softRecoveryGoodSamples_ = 0;
    recoveryActive_ = true;
    recoveryStableSamples_ = 0;
    recoveryRejectStreak_ = 0;
    recoveryAccelSum_ = Vec3::zero();

    // Entering strict recovery is a state transition. Repeated recovery
    // samples must not spam the console, but a new strict transition rebases
    // every orientation-dependent consumer exactly once.
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

void TrackingStateController::updateSoftRecovery(const ImuQualityResult& quality,
                                                 uint64_t timestampUs,
                                                 bool ahrsIntegrated,
                                                 const TrackingStateEventSink& sink) {
    if (!softRecoveryActive_) return;

    const bool hardStreamFault = quality.shouldRequestFifoRecovery ||
                                 quality.has(imu_quality_flags::TIMESTAMP_BACKWARDS) ||
                                 quality.has(imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW) ||
                                 quality.has(imu_quality_flags::FIFO_UNKNOWN_TAG);
    if (hardStreamFault || !ahrsIntegrated) {
        softRecoveryGoodSamples_ = 0;
        return;
    }
    if (!quality.shouldUseAccelCorrection || !quality.accelNormValid) {
        // Gyro-only continuity keeps heading alive but is not enough evidence
        // to declare the post-reset stream fully coherent. Preserve the good
        // streak across a short accel dropout instead of resetting it.
        return;
    }

    if (softRecoveryGoodSamples_ < SOFT_RECOVERY_GOOD_SAMPLES_REQUIRED) {
        ++softRecoveryGoodSamples_;
    }
    if (softRecoveryGoodSamples_ < SOFT_RECOVERY_GOOD_SAMPLES_REQUIRED) return;

    softRecoveryActive_ = false;
    softRecoveryGoodSamples_ = 0;
    softRecoveryCompleteCount_++;

    if (sink.out && !trackerConsoleTrackingMessagesSuppressed(millis())) {
        sink.out->println("# TRACKING state=TRACKING_6DOF reason=fifo_soft_recovery_complete");
    }
    if (sink.emitStateEvent) {
        sink.emitStateEvent("TRACKING_6DOF",
                            "fifo_soft_recovery_complete",
                            timestampUs,
                            quality.flags,
                            quality.overallConfidence,
                            sink.emitStateEventUser);
    }
}

void TrackingStateController::updateRecovery(const ImuQualityResult& quality,
                                             const Vec3& gyroRadS,
                                             const Vec3& accelG,
                                             uint64_t lastSampleTimestampUs,
                                             bool ahrsIntegrated,
                                             const TrackingStateEventSink& sink) {
    updateSoftRecovery(quality, lastSampleTimestampUs, ahrsIntegrated, sink);
    if (!recoveryActive_) return;

    constexpr float kMaxRecoveryGyroRadS = 3.0f * MATH_DEG_TO_RAD;
    constexpr float kMaxRecoveryAccelErrorG = 0.08f;

    const bool gyroValid = gyroRadS.isFinite();
    const float gyroNorm = gyroValid ? gyroRadS.norm() : 0.0f;
    const bool gyroStable = gyroValid && std::isfinite(gyroNorm) &&
                            gyroNorm <= kMaxRecoveryGyroRadS;
    const bool accelObservationAvailable = quality.shouldUseAccelCorrection &&
                                            quality.accelNormValid && accelG.isFinite();
    const float accelNorm = accelObservationAvailable ? quality.accelNormG : 0.0f;
    const bool accelStable = accelObservationAvailable && std::isfinite(accelNorm) &&
                             std::fabs(accelNorm - 1.0f) <= kMaxRecoveryAccelErrorG;
    const bool motionStable = gyroStable && accelStable;
    const bool hardStreamFault = quality.shouldRequestFifoRecovery ||
                                 quality.has(imu_quality_flags::TIMESTAMP_BACKWARDS) ||
                                 quality.has(imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW) ||
                                 quality.has(imu_quality_flags::FIFO_OVERRUN) ||
                                 quality.has(imu_quality_flags::FIFO_FULL) ||
                                 quality.has(imu_quality_flags::FIFO_UNKNOWN_TAG);
    const bool stable = motionStable &&
                        !hardStreamFault &&
                        ahrsIntegrated &&
                        quality.shouldUseAccelCorrection &&
                        quality.accelConfidence >= 0.75f;

    if (!stable) {
        // Real motion or a broken FIFO invalidates the accumulated gravity
        // direction immediately. Only isolated unusable timestamp/quality
        // samples are tolerated, so a busy Wi-Fi loop cannot make recovery
        // impossible without blending two physical orientations together.
        if (!gyroStable || hardStreamFault || (accelObservationAvailable && !accelStable)) {
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
                              imu_quality_flags::ACCEL_NOT_AHRS_USABLE |
                              imu_quality_flags::ACCEL_COMPONENT_MISSING |
                              imu_quality_flags::FIFO_PAIR_DEGRADED;
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
    if (softRecoveryActive_ || in.qualityRecoveryRequested || hasTimingFault(in.qualityFlags)) {
        return TrackingStateId::DegradedTiming;
    }
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
    out.print("tracking_soft_recovery_active="); out.println(softRecoveryActive_ ? "yes" : "no");
    out.print("tracking_recovery_stable_samples="); out.println(recoveryStableSamples_);
    out.print("tracking_soft_recovery_good_samples="); out.println(softRecoveryGoodSamples_);
    out.print("tracking_recovery_reject_streak="); out.println(recoveryRejectStreak_);
    out.print("tracking_recovery_enter_count="); out.println(recoveryEnterCount_);
    out.print("tracking_recovery_tilt_reacquire_count="); out.println(recoveryTiltReacquireCount_);
    out.print("tracking_recovery_bootstrap_bypass_count="); out.println(recoveryBootstrapBypassCount_);
    out.print("tracking_soft_recovery_enter_count="); out.println(softRecoveryEnterCount_);
    out.print("tracking_soft_recovery_complete_count="); out.println(softRecoveryCompleteCount_);
    out.print("tracking_recovery_last_flags=0x"); out.println(recoveryLastFlags_, HEX);
    out.print("tracking_recovery_last_reason="); out.println(trackingRecoveryReasonName(recoveryLastReason_));
    out.print("tracking_recovery_fifo_quality_requests="); out.println(recoveryFifoQualityRequests_);
    out.print("tracking_recovery_timestamp_gap_requests="); out.println(recoveryTimestampGapRequests_);
    out.print("tracking_recovery_manual_reset_requests="); out.println(recoveryManualResetRequests_);
    out.print("tracking_recovery_blocking_operation_requests="); out.println(recoveryBlockingOperationRequests_);
    out.print("tracking_recovery_runtime_reconfigure_requests="); out.println(recoveryRuntimeReconfigureRequests_);
    out.print("tracking_recovery_other_requests="); out.println(recoveryOtherRequests_);
}

} // namespace tracker
