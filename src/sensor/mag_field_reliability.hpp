#pragma once

#include <cstdint>

#include "core/math.hpp"
#include "sensor/mag_heading.hpp"
#include "sensor/mag_horizontal_trust.hpp"
#include "sensor/mag_runtime.hpp"

namespace tracker {

enum class MagFieldReliabilityState : uint8_t {
    Unavailable = 0,
    Acquiring = 1,
    Trusted = 2,
    Suspect = 3,
    Disturbed = 4,
    Recovering = 5,
};

enum MagFieldReliabilityFlags : uint32_t {
    MAG_FIELD_FLAG_NONE = 0,
    MAG_FIELD_FLAG_INPUT_INVALID = 1u << 0,
    MAG_FIELD_FLAG_PROCESSOR_UNTRUSTED = 1u << 1,
    MAG_FIELD_FLAG_REFERENCE_MISSING = 1u << 2,
    MAG_FIELD_FLAG_NORM_SOFT = 1u << 3,
    MAG_FIELD_FLAG_NORM_HARD = 1u << 4,
    MAG_FIELD_FLAG_DIP_SOFT = 1u << 5,
    MAG_FIELD_FLAG_DIP_HARD = 1u << 6,
    MAG_FIELD_FLAG_HEADING_STEP_SOFT = 1u << 7,
    MAG_FIELD_FLAG_HEADING_STEP_HARD = 1u << 8,
    MAG_FIELD_FLAG_RECOVERY_PENDING = 1u << 9,
    MAG_FIELD_FLAG_ENVIRONMENT_CHANGED = 1u << 10,
    // A short-window magnetic-direction discontinuity while gyro/accel say
    // the tracker is stationary. AHRS yaw is removed from the detection signal
    // so an internal yaw correction/reset cannot create a false field jump.
    // The latch clears only after a proven return or explicit reacquisition.
    MAG_FIELD_FLAG_STATIONARY_HEADING_JUMP = 1u << 11,
    // Environmental norm/dip may be stable while the horizontal component is
    // too weak for a reliable yaw observation. This closes yaw use without
    // declaring the whole magnetic environment disturbed.
    MAG_FIELD_FLAG_HORIZONTAL_UNOBSERVABLE = 1u << 12,
};

struct MagFieldReliabilityConfig {
    uint32_t acquireStableMs = 3000;
    uint32_t suspectClearMs = 1200;
    uint32_t recoverStableMs = 5000;
    uint32_t newEnvironmentStableMs = 30000;
    uint8_t badSamplesToDisturbed = 3;

    float normSoftFraction = 0.12f;
    float normHardFraction = 0.25f;
    float dipSoftDeg = 8.0f;
    float dipHardDeg = 18.0f;

    float stationaryGyroMaxDps = 3.0f;
    float stationaryAccelTrustMin = 0.80f;
    float headingStepSoftDeg = 8.0f;
    float headingStepHardDeg = 20.0f;

    // User-configured absolute yaw thresholds are upper caps. The acquired
    // field reference can lower them for high-dip environments.
    float horizontalNormBad = 200.0f;
    float horizontalNormGood = 260.0f;
    MagHorizontalTrustConfig horizontalTrust;
    float minimumHeadingTrustForJumpDetection = 0.25f;

    // Distinguish an abrupt local-field change from slow gyro yaw drift.
    // The normal yaw controller is rate-limited to 2 deg/s, so the default
    // discontinuity rate is deliberately above any correction-generated
    // heading motion while still catching a 5-19 degree one-shot shift.
    uint32_t stationaryHeadingWindowMs = 1500;
    float stationaryHeadingJumpMinDeg = 3.0f;
    float stationaryHeadingJumpRateDegS = 5.0f;
    float stationaryHeadingJumpRateMarginOverGyroDegS = 3.0f;
    float referenceReturnDeg = 2.5f;
    uint32_t referenceReturnStableMs = 1200;

    float headingRateFilterTimeConstantS = 0.75f;
    uint32_t headingRateResetGapMs = 500;
    float referenceAdaptTimeConstantS = 120.0f;
};

struct MagFieldReliabilityInput {
    MagProcessedSample mag;
    MagHeadingSample heading;
    bool processorTrustedForUse = false;
    float gyroNormDps = 0.0f;
    float accelTrust = 0.0f;
    uint32_t nowMs = 0;
};

// Lightweight callback view over already-published coherent snapshots. The
// owning aggregate above remains convenient for native tests and standalone
// callers, while the runtime avoids copying both snapshots onto its stack.
struct MagFieldReliabilityInputView {
    const MagProcessedSample* mag = nullptr;
    const MagHeadingSample* heading = nullptr;
    bool processorTrustedForUse = false;
    float gyroNormDps = 0.0f;
    float accelTrust = 0.0f;
    float horizontalNormBad = 200.0f;
    float horizontalNormGood = 260.0f;
    uint32_t nowMs = 0;
};

struct MagFieldReliabilityOutput {
    bool valid = false;
    bool trustedForYaw = false;
    bool referenceValid = false;
    MagFieldReliabilityState state = MagFieldReliabilityState::Unavailable;
    uint32_t flags = MAG_FIELD_FLAG_NONE;
    uint32_t nowMs = 0;
    uint32_t stableMs = 0;

    float fieldNorm = 0.0f;
    float dipDeg = 0.0f;
    float referenceNorm = 0.0f;
    float referenceDipDeg = 0.0f;
    float referenceHeadingYawDeg = 0.0f;
    float referenceHeadingErrorDeg = 0.0f;
    float normRelativeError = 0.0f;
    float dipErrorDeg = 0.0f;
    // Field-direction motion with AHRS yaw removed. This is the signal used
    // for stationary discontinuity detection so an AHRS yaw correction/reset
    // cannot masquerade as a local magnetic-field jump.
    float stationaryFieldYawDeg = 0.0f;
    float horizontalNorm = 0.0f;
    float referenceHorizontalNorm = 0.0f;
    float horizontalTrust = 0.0f;
    float horizontalEffectiveBad = 0.0f;
    float horizontalEffectiveGood = 0.0f;
    // Geometry multiplier is kept squared so the sensor hot path can compare
    // squared angles/rates without sqrt. Human-readable thresholds are derived
    // lazily by the CLI reporter.
    float headingNoiseScaleSquared = 1.0f;
    float stationaryHeadingJumpRateGyroFloorDegS = 0.0f;
    float headingStepDeg = 0.0f;
    float headingInstantRateDegS = 0.0f;
    float headingRateDegS = 0.0f;
    float stationaryWindowHeadingDeltaDeg = 0.0f;
    float stationaryWindowHeadingRateDegS = 0.0f;
    float stationaryLatchReferenceFieldYawDeg = 0.0f;
    float stationaryLatchReferenceErrorDeg = 0.0f;
    bool stationaryLatchMotionSeen = false;
    bool stationaryHeadingJumpLatched = false;
};

struct MagFieldReliabilityStats {
    uint32_t updates = 0;
    uint32_t trustedSamples = 0;
    uint32_t rejectedSamples = 0;
    uint32_t stateTransitions = 0;
    uint32_t enteredDisturbed = 0;
    uint32_t enteredRecovering = 0;
    uint32_t environmentChangesDetected = 0;
    uint32_t referencesAcquired = 0;
    uint32_t normSoftRejects = 0;
    uint32_t normHardRejects = 0;
    uint32_t dipSoftRejects = 0;
    uint32_t dipHardRejects = 0;
    uint32_t headingStepSoftRejects = 0;
    uint32_t headingStepHardRejects = 0;
    uint32_t stationaryHeadingJumpRejects = 0;
    uint32_t stationaryHeadingJumpsLatched = 0;
    uint32_t stationaryHeadingReturns = 0;
    uint32_t stationaryHeadingReturnsViaWorld = 0;
    uint32_t stationaryHeadingReturnsViaStationaryField = 0;
};

class MagFieldReliabilityMonitor {
public:
    void reset();
    // Explicit user/config epoch action. Keeps cumulative diagnostics but
    // discards the environmental reference and starts a fresh acquisition.
    void restartAcquisition();

    bool update(const MagFieldReliabilityInput& in,
                const MagFieldReliabilityConfig& cfg,
                MagFieldReliabilityOutput& out) {
        const MagFieldReliabilityInputView view{
            &in.mag,
            &in.heading,
            in.processorTrustedForUse,
            in.gyroNormDps,
            in.accelTrust,
            cfg.horizontalNormBad,
            cfg.horizontalNormGood,
            in.nowMs,
        };
        return update(view, cfg, out);
    }

    // Hot-path overload: pass existing coherent snapshots by reference instead
    // of copying them into a large aggregate input object on the callback stack.
    bool update(const MagFieldReliabilityInputView& in,
                const MagFieldReliabilityConfig& cfg,
                MagFieldReliabilityOutput& out);

    const MagFieldReliabilityStats& stats() const { return stats_; }
    static const char* stateName(MagFieldReliabilityState state);

private:
    void transition(MagFieldReliabilityState next, uint32_t nowMs);
    void resetAcquisitionAccumulator();
    void accumulateReferenceSample(const MagProcessedSample& mag,
                                   const MagHeadingSample& heading);
    void acquireReferenceFromAccumulator(const MagProcessedSample& fallbackMag,
                                         const MagHeadingSample& fallbackHeading,
                                         uint32_t nowMs);
    void adaptReference(const MagProcessedSample& mag,
                        const MagHeadingSample& heading,
                        float gyroNormDps,
                        float accelTrust,
                        const MagFieldReliabilityConfig& cfg,
                        uint32_t dtMs);

    MagFieldReliabilityState state_ = MagFieldReliabilityState::Unavailable;
    bool referenceValid_ = false;
    float referenceNorm_ = 0.0f;
    float referenceDipDeg_ = 0.0f;
    float referenceHorizontalNorm_ = 0.0f;
    float referenceHeadingYawRad_ = 0.0f;
    uint32_t stateSinceMs_ = 0;
    uint32_t goodSinceMs_ = 0;
    uint8_t consecutiveBad_ = 0;
    uint32_t newEnvironmentSinceMs_ = 0;
    bool environmentChangedLatched_ = false;

    bool havePreviousHeading_ = false;
    float previousStationaryFieldYawRad_ = 0.0f;
    uint32_t previousHeadingMs_ = 0;
    bool headingRateInitialized_ = false;
    float filteredHeadingRateDegS_ = 0.0f;

    bool stationaryWindowActive_ = false;
    uint32_t stationaryWindowStartMs_ = 0;
    float stationaryWindowStartFieldYawRad_ = 0.0f;
    bool stationaryHeadingJumpLatched_ = false;
    float stationaryLatchReferenceFieldYawRad_ = 0.0f;
    bool stationaryLatchMotionSeen_ = false;
    uint32_t referenceReturnSinceMs_ = 0;

    double acquireNormSum_ = 0.0;
    double acquireDipSum_ = 0.0;
    double acquireHorizontalNormSum_ = 0.0;
    double acquireHeadingSinSum_ = 0.0;
    double acquireHeadingCosSum_ = 0.0;
    uint32_t acquireSampleCount_ = 0;

    MagFieldReliabilityStats stats_;
};

} // namespace tracker
