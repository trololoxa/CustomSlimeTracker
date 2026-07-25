#pragma once

#include <cstdint>

#include "core/math.hpp"
#include "sensor/mag_runtime.hpp"
#include "sensor/mag_heading.hpp"

namespace tracker {

enum MagYawCorrectionRejectFlags : uint32_t {
    MAG_YAW_REJECT_NONE                  = 0,
    MAG_YAW_REJECT_DISABLED              = 1u << 0,
    MAG_YAW_REJECT_APPLY_DISABLED        = 1u << 1,
    MAG_YAW_REJECT_NO_REFERENCE          = 1u << 2,
    MAG_YAW_REJECT_HEADING_INVALID       = 1u << 3,
    MAG_YAW_REJECT_MAG_NOT_TRUSTED       = 1u << 4,
    MAG_YAW_REJECT_MAG_STALE             = 1u << 5,
    MAG_YAW_REJECT_HORIZONTAL_BAD        = 1u << 6,
    MAG_YAW_REJECT_INNOVATION_TOO_LARGE  = 1u << 7,
    MAG_YAW_REJECT_GYRO_MOVING           = 1u << 8,
    MAG_YAW_REJECT_ACCEL_NOT_TRUSTED     = 1u << 9,
    MAG_YAW_REJECT_DT_INVALID            = 1u << 10,
    MAG_YAW_REJECT_NONFINITE             = 1u << 11,

    // Recovery latch:
    // after strong motion or magnetic disturbance, keep yaw correction closed
    // for a short time even after instant gates become good again.
    MAG_YAW_REJECT_COOLDOWN              = 1u << 12,
    MAG_YAW_REJECT_REACQUIRE_PENDING      = 1u << 13,
};

enum class MagYawCorrectionMode : uint8_t {
    Normal = 0,
    Reacquiring = 1,
};

struct MagYawCorrectionConfig {
    bool enabled = true;
    bool applyEnabled = false;

    float maxInnovationDeg = 25.0f;
    uint32_t maxMagAgeMs = 250;

    float horizontalNormGood = 260.0f;
    float horizontalNormBad = 200.0f;

    float gyroNormGoodDps = 8.0f;
    float gyroNormBadDps = 35.0f;

    float accelTrustGood = 0.70f;
    float accelTrustBad = 0.20f;
    bool requireAccelTrusted = true;

    float timeConstantS = 30.0f;
    float maxCorrectionRateDegS = 2.0f;
    float maxCorrectionStepDeg = 0.25f;
    float fallbackDtS = 1.0f / 60.0f;

    // New recovery cooldowns.
    // Motion cooldown is short; magnetic disturbance cooldown is longer.
    uint32_t gyroMovingCooldownMs = 1000;
    uint32_t accelBadCooldownMs = 750;
    uint32_t magDisturbanceCooldownMs = 3000;

    bool reacquisitionEnabled = true;
    float reacquireInnovationMaxDeg = 170.0f;
    uint32_t reacquireMinFieldStableMs = 8000;
    float reacquireMaxHeadingRateDegS = 2.0f;
    float reacquireTimeConstantS = 90.0f;
    float reacquireMaxCorrectionRateDegS = 0.75f;
    float reacquireMaxCorrectionStepDeg = 0.08f;
};

struct MagYawCorrectionInput {
    MagProcessedSample mag;
    MagHeadingSample heading;

    bool referenceValid = false;
    float referenceWorldYawRad = 0.0f;

    bool magTrustedForUse = false;
    uint32_t magRejectFlagsForUse = MAG_REJECT_NONE;

    float gyroNormDps = 0.0f;
    float accelTrust = 0.0f;
    bool fieldReliable = false;
    uint32_t fieldStableMs = 0;
    float magneticHeadingRateDegS = 0.0f;

    uint32_t nowMs = 0;
};

struct MagYawCorrectionOutput {
    bool valid = false;

    bool gateOpen = false;
    bool applyAllowed = false;
    bool applied = false;
    MagYawCorrectionMode mode = MagYawCorrectionMode::Normal;
    bool reacquirePending = false;
    bool reacquireActive = false;

    uint32_t rejectFlags = MAG_YAW_REJECT_NONE;

    uint32_t nowMs = 0;
    uint32_t dtMs = 0;

    float errorRad = 0.0f;
    float errorDeg = 0.0f;

    float horizontalNorm = 0.0f;
    float horizontalTrust = 0.0f;

    float gyroNormDps = 0.0f;
    float gyroTrust = 0.0f;

    float accelTrust = 0.0f;
    float accelGateTrust = 0.0f;
    uint32_t fieldStableMs = 0;
    float magneticHeadingRateDegS = 0.0f;

    float combinedTrust = 0.0f;

    float correctionRateRadS = 0.0f;
    float correctionRateDegS = 0.0f;

    float correctionStepRad = 0.0f;
    float correctionStepDeg = 0.0f;

    uint32_t magAgeMs = 0;
    uint32_t magSeq = 0;
    uint64_t magTimestampUs = 0;

    bool cooldownActive = false;
    uint32_t cooldownRemainingMs = 0;
    uint32_t cooldownReasonFlags = 0;
};

struct MagYawCorrectionStats {
    uint32_t updates = 0;
    uint32_t gateOpenCount = 0;
    uint32_t gateClosedCount = 0;
    uint32_t applyAllowedCount = 0;
    uint32_t appliedCount = 0;

    uint32_t rejectDisabled = 0;
    uint32_t rejectApplyDisabled = 0;
    uint32_t rejectNoReference = 0;
    uint32_t rejectHeadingInvalid = 0;
    uint32_t rejectMagNotTrusted = 0;
    uint32_t rejectMagStale = 0;
    uint32_t rejectHorizontalBad = 0;
    uint32_t rejectInnovationTooLarge = 0;
    uint32_t rejectGyroMoving = 0;
    uint32_t rejectAccelNotTrusted = 0;
    uint32_t rejectDtInvalid = 0;
    uint32_t rejectNonfinite = 0;
    uint32_t rejectCooldown = 0;
    uint32_t rejectReacquirePending = 0;
    uint32_t reacquireGateOpenCount = 0;
    uint32_t reacquireAppliedCount = 0;
    uint32_t reacquireCompletedCount = 0;

    float lastAbsErrorDeg = 0.0f;
    float maxAbsErrorDeg = 0.0f;
    double sumAbsErrorDeg = 0.0;
    uint32_t errorSamples = 0;

    double sumCorrectionStepDeg = 0.0;
    float lastCorrectionStepDeg = 0.0f;
    float maxAbsCorrectionStepDeg = 0.0f;

    float meanAbsErrorDeg() const {
        return errorSamples > 0
            ? static_cast<float>(sumAbsErrorDeg / static_cast<double>(errorSamples))
            : 0.0f;
    }
};

class MagYawCorrectionController {
public:
    void reset();

    const MagYawCorrectionOutput& last() const;
    const MagYawCorrectionStats& stats() const;

    void markApplied(float correctionStepDeg);

    bool update(const MagYawCorrectionInput& in,
                const MagYawCorrectionConfig& cfg,
                MagYawCorrectionOutput& out);

    bool update(const MagYawCorrectionInput& in,
                const MagYawCorrectionConfig& cfg);

private:
    static bool timeBefore(uint32_t a, uint32_t b);

    void expireCooldownIfNeeded(uint32_t nowMs);
    void requestCooldown(uint32_t nowMs, uint32_t durationMs, uint32_t reasonFlags);
    bool cooldownActive(uint32_t nowMs) const;
    uint32_t cooldownRemainingMs(uint32_t nowMs) const;

    static float rampUp(float x, float bad, float good);
    static float rampDown(float x, float good, float bad);

    void addReject(MagYawCorrectionOutput& out, uint32_t flag);
    void countRejects(uint32_t flags);

    MagYawCorrectionOutput last_;
    MagYawCorrectionStats stats_;
    uint32_t lastUpdateMs_ = 0;

    uint32_t cooldownUntilMs_ = 0;
    uint32_t cooldownReasonFlags_ = MAG_YAW_REJECT_NONE;
    bool reacquiring_ = false;
    uint32_t reacquireCandidateSinceMs_ = 0;
};

} // namespace tracker
