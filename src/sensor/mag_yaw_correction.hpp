#pragma once

#include <cstdint>
#include <cmath>

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

    uint32_t nowMs = 0;
};

struct MagYawCorrectionOutput {
    bool valid = false;

    bool gateOpen = false;
    bool applyAllowed = false;
    bool applied = false;

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
    void reset() {
        last_ = MagYawCorrectionOutput{};
        stats_ = MagYawCorrectionStats{};
        lastUpdateMs_ = 0;
        cooldownUntilMs_ = 0;
        cooldownReasonFlags_ = MAG_YAW_REJECT_NONE;
    }

    const MagYawCorrectionOutput& last() const { return last_; }
    const MagYawCorrectionStats& stats() const { return stats_; }

    void markApplied(float correctionStepDeg) {
        stats_.appliedCount++;
        stats_.lastCorrectionStepDeg = correctionStepDeg;
        last_.applied = true;
    }

    bool update(const MagYawCorrectionInput& in,
                const MagYawCorrectionConfig& cfg,
                MagYawCorrectionOutput& out) {
        out = MagYawCorrectionOutput{};
        out.valid = true;
        out.nowMs = in.nowMs;
        out.horizontalNorm = in.heading.horizontalNorm;
        out.gyroNormDps = in.gyroNormDps;
        out.accelTrust = in.accelTrust;
        out.magSeq = in.mag.seq;
        out.magTimestampUs = in.mag.t_us;

        expireCooldownIfNeeded(in.nowMs);

        if (lastUpdateMs_ != 0 && in.nowMs >= lastUpdateMs_) {
            out.dtMs = in.nowMs - lastUpdateMs_;
        } else {
            out.dtMs = 0;
        }
        lastUpdateMs_ = in.nowMs;

        stats_.updates++;

        if (!cfg.enabled) {
            addReject(out, MAG_YAW_REJECT_DISABLED);
        }

        if (!in.referenceValid) {
            addReject(out, MAG_YAW_REJECT_NO_REFERENCE);
        }

        if (!in.heading.valid) {
            addReject(out, MAG_YAW_REJECT_HEADING_INVALID);
        }

        if (!in.magTrustedForUse) {
            addReject(out, MAG_YAW_REJECT_MAG_NOT_TRUSTED);
        }

        if (in.magRejectFlagsForUse & MAG_REJECT_STALE) {
            addReject(out, MAG_YAW_REJECT_MAG_STALE);
        }

        out.magAgeMs = MagRuntimeProcessor::ageMsForUse(in.mag, in.nowMs);
        if (out.magAgeMs > cfg.maxMagAgeMs) {
            addReject(out, MAG_YAW_REJECT_MAG_STALE);
        }

        out.horizontalTrust = rampUp(in.heading.horizontalNorm,
                                     cfg.horizontalNormBad,
                                     cfg.horizontalNormGood);
        if (!tracker::isFinite(in.heading.horizontalNorm) || out.horizontalTrust <= 0.0f) {
            addReject(out, MAG_YAW_REJECT_HORIZONTAL_BAD);
        }

        out.gyroTrust = rampDown(in.gyroNormDps,
                                 cfg.gyroNormGoodDps,
                                 cfg.gyroNormBadDps);
        if (!tracker::isFinite(in.gyroNormDps) || out.gyroTrust <= 0.0f) {
            addReject(out, MAG_YAW_REJECT_GYRO_MOVING);
        }

        out.accelGateTrust = rampUp(in.accelTrust,
                                    cfg.accelTrustBad,
                                    cfg.accelTrustGood);
        if (cfg.requireAccelTrusted && (!tracker::isFinite(in.accelTrust) || out.accelGateTrust <= 0.0f)) {
            addReject(out, MAG_YAW_REJECT_ACCEL_NOT_TRUSTED);
        }

        if (in.referenceValid && in.heading.valid) {
            out.errorRad = wrapPi(in.heading.magneticNorthWorldYawRad - in.referenceWorldYawRad);
            out.errorDeg = out.errorRad * MATH_RAD_TO_DEG;

            const float absErr = std::fabs(out.errorDeg);
            stats_.lastAbsErrorDeg = absErr;
            if (stats_.errorSamples == 0 || absErr > stats_.maxAbsErrorDeg) {
                stats_.maxAbsErrorDeg = absErr;
            }
            stats_.sumAbsErrorDeg += static_cast<double>(absErr);
            stats_.errorSamples++;

            if (!tracker::isFinite(out.errorRad) || !tracker::isFinite(out.errorDeg)) {
                addReject(out, MAG_YAW_REJECT_NONFINITE);
            }

            if (absErr > cfg.maxInnovationDeg) {
                addReject(out, MAG_YAW_REJECT_INNOVATION_TOO_LARGE);
            }
        }

        const uint32_t instantRejectFlags = out.rejectFlags;

        if (instantRejectFlags & MAG_YAW_REJECT_GYRO_MOVING) {
            requestCooldown(in.nowMs, cfg.gyroMovingCooldownMs, instantRejectFlags);
        }

        if (instantRejectFlags & MAG_YAW_REJECT_ACCEL_NOT_TRUSTED) {
            requestCooldown(in.nowMs, cfg.accelBadCooldownMs, instantRejectFlags);
        }

        if (instantRejectFlags & (
                MAG_YAW_REJECT_HEADING_INVALID |
                MAG_YAW_REJECT_MAG_NOT_TRUSTED |
                MAG_YAW_REJECT_HORIZONTAL_BAD |
                MAG_YAW_REJECT_INNOVATION_TOO_LARGE)) {
            requestCooldown(in.nowMs, cfg.magDisturbanceCooldownMs, instantRejectFlags);
        }

        if (cfg.enabled && in.referenceValid && cooldownActive(in.nowMs)) {
            addReject(out, MAG_YAW_REJECT_COOLDOWN);
            out.cooldownActive = true;
            out.cooldownRemainingMs = cooldownRemainingMs(in.nowMs);
            out.cooldownReasonFlags = cooldownReasonFlags_;
        }

        float dtS = static_cast<float>(out.dtMs) * 0.001f;
        if (dtS <= 0.0f || dtS > 1.0f || !tracker::isFinite(dtS)) {
            dtS = cfg.fallbackDtS;
        }

        if (dtS <= 0.0f || !tracker::isFinite(dtS)) {
            addReject(out, MAG_YAW_REJECT_DT_INVALID);
        }

        out.combinedTrust = out.horizontalTrust * out.gyroTrust;
        if (cfg.requireAccelTrusted) {
            out.combinedTrust *= out.accelGateTrust;
        }
        out.combinedTrust = clampf(out.combinedTrust, 0.0f, 1.0f);

        if (out.rejectFlags == MAG_YAW_REJECT_NONE) {
            out.gateOpen = true;
            stats_.gateOpenCount++;

            const float tc = cfg.timeConstantS > 0.001f ? cfg.timeConstantS : 30.0f;

            float rateRadS = -out.errorRad / tc;
            rateRadS *= out.combinedTrust;

            const float maxRateRadS = cfg.maxCorrectionRateDegS * MATH_DEG_TO_RAD;
            rateRadS = clampf(rateRadS, -maxRateRadS, maxRateRadS);

            float stepRad = rateRadS * dtS;

            const float maxStepRad = cfg.maxCorrectionStepDeg * MATH_DEG_TO_RAD;
            stepRad = clampf(stepRad, -maxStepRad, maxStepRad);

            out.correctionRateRadS = rateRadS;
            out.correctionRateDegS = rateRadS * MATH_RAD_TO_DEG;
            out.correctionStepRad = stepRad;
            out.correctionStepDeg = stepRad * MATH_RAD_TO_DEG;

            if (cfg.applyEnabled) {
                out.applyAllowed = true;
                stats_.applyAllowedCount++;
            } else {
                addReject(out, MAG_YAW_REJECT_APPLY_DISABLED);
            }
        } else {
            out.gateOpen = false;
            stats_.gateClosedCount++;
            countRejects(out.rejectFlags);
        }

        if (out.applyAllowed && out.correctionStepRad != 0.0f) {
            stats_.lastCorrectionStepDeg = out.correctionStepDeg;
            const float absStep = std::fabs(out.correctionStepDeg);
            if (absStep > stats_.maxAbsCorrectionStepDeg) {
                stats_.maxAbsCorrectionStepDeg = absStep;
            }
            stats_.sumCorrectionStepDeg += static_cast<double>(out.correctionStepDeg);
        }

        last_ = out;
        return out.valid;
    }

    bool update(const MagYawCorrectionInput& in,
                const MagYawCorrectionConfig& cfg) {
        MagYawCorrectionOutput out;
        return update(in, cfg, out);
    }

private:
    static bool timeBefore(uint32_t a, uint32_t b) {
        return static_cast<int32_t>(a - b) < 0;
    }

    void expireCooldownIfNeeded(uint32_t nowMs) {
        if (cooldownUntilMs_ == 0) return;
        if (!timeBefore(nowMs, cooldownUntilMs_)) {
            cooldownUntilMs_ = 0;
            cooldownReasonFlags_ = MAG_YAW_REJECT_NONE;
        }
    }

    void requestCooldown(uint32_t nowMs, uint32_t durationMs, uint32_t reasonFlags) {
        if (durationMs == 0) return;

        const uint32_t until = nowMs + durationMs;
        if (cooldownUntilMs_ == 0 || timeBefore(cooldownUntilMs_, until)) {
            cooldownUntilMs_ = until;
            cooldownReasonFlags_ = reasonFlags;
        } else {
            cooldownReasonFlags_ |= reasonFlags;
        }
    }

    bool cooldownActive(uint32_t nowMs) const {
        return cooldownUntilMs_ != 0 && timeBefore(nowMs, cooldownUntilMs_);
    }

    uint32_t cooldownRemainingMs(uint32_t nowMs) const {
        if (!cooldownActive(nowMs)) return 0;
        return cooldownUntilMs_ - nowMs;
    }

    static float rampUp(float x, float bad, float good) {
        if (x >= good) return 1.0f;
        if (x <= bad) return 0.0f;
        if (good <= bad) return 0.0f;
        return (x - bad) / (good - bad);
    }

    static float rampDown(float x, float good, float bad) {
        if (x <= good) return 1.0f;
        if (x >= bad) return 0.0f;
        if (bad <= good) return 0.0f;
        return 1.0f - ((x - good) / (bad - good));
    }

    void addReject(MagYawCorrectionOutput& out, uint32_t flag) {
        out.rejectFlags |= flag;
    }

    void countRejects(uint32_t flags) {
        if (flags & MAG_YAW_REJECT_DISABLED)             stats_.rejectDisabled++;
        if (flags & MAG_YAW_REJECT_APPLY_DISABLED)       stats_.rejectApplyDisabled++;
        if (flags & MAG_YAW_REJECT_NO_REFERENCE)         stats_.rejectNoReference++;
        if (flags & MAG_YAW_REJECT_HEADING_INVALID)      stats_.rejectHeadingInvalid++;
        if (flags & MAG_YAW_REJECT_MAG_NOT_TRUSTED)      stats_.rejectMagNotTrusted++;
        if (flags & MAG_YAW_REJECT_MAG_STALE)            stats_.rejectMagStale++;
        if (flags & MAG_YAW_REJECT_HORIZONTAL_BAD)       stats_.rejectHorizontalBad++;
        if (flags & MAG_YAW_REJECT_INNOVATION_TOO_LARGE) stats_.rejectInnovationTooLarge++;
        if (flags & MAG_YAW_REJECT_GYRO_MOVING)          stats_.rejectGyroMoving++;
        if (flags & MAG_YAW_REJECT_ACCEL_NOT_TRUSTED)    stats_.rejectAccelNotTrusted++;
        if (flags & MAG_YAW_REJECT_DT_INVALID)           stats_.rejectDtInvalid++;
        if (flags & MAG_YAW_REJECT_NONFINITE)            stats_.rejectNonfinite++;
        if (flags & MAG_YAW_REJECT_COOLDOWN)             stats_.rejectCooldown++;
    }

    MagYawCorrectionOutput last_;
    MagYawCorrectionStats stats_;
    uint32_t lastUpdateMs_ = 0;

    uint32_t cooldownUntilMs_ = 0;
    uint32_t cooldownReasonFlags_ = MAG_YAW_REJECT_NONE;
};

} // namespace tracker