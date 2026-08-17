#include "sensor/mag_yaw_correction.hpp"

#include <cmath>

namespace tracker {

void MagYawCorrectionController::reset() {
    last_ = MagYawCorrectionOutput{};
    stats_ = MagYawCorrectionStats{};
    lastUpdateMs_ = 0;
    cooldownUntilMs_ = 0;
    cooldownReasonFlags_ = MAG_YAW_REJECT_NONE;
    reacquiring_ = false;
    reacquireCandidateSinceMs_ = 0;
}

const MagYawCorrectionOutput& MagYawCorrectionController::last() const {
    return last_;
}

const MagYawCorrectionStats& MagYawCorrectionController::stats() const {
    return stats_;
}

void MagYawCorrectionController::markApplied(float correctionStepDeg) {
    stats_.appliedCount++;
    stats_.lastCorrectionStepDeg = correctionStepDeg;
    last_.applied = true;
}

bool MagYawCorrectionController::update(const MagYawCorrectionInputView& in,
                                        const MagYawCorrectionConfig& cfg,
                                        MagYawCorrectionOutput& out) {
    if (in.mag == nullptr || in.heading == nullptr) {
        out = MagYawCorrectionOutput{};
        out.nowMs = in.nowMs;
        addReject(out, MAG_YAW_REJECT_NONFINITE);
        last_ = out;
        return false;
    }
    const MagProcessedSample& mag = *in.mag;
    const MagHeadingSample& heading = *in.heading;
    out = MagYawCorrectionOutput{};
    out.valid = true;
    out.nowMs = in.nowMs;
    out.horizontalNorm = heading.horizontalNorm;
    out.gyroNormDps = in.gyroNormDps;
    out.accelTrust = in.accelTrust;
    out.fieldStableMs = in.fieldStableMs;
    out.magneticHeadingRateDegS = in.magneticHeadingRateDegS;
    out.magSeq = mag.seq;
    out.magTimestampUs = mag.t_us;

    expireCooldownIfNeeded(in.nowMs);
    out.dtMs = stats_.updates > 0 ? in.nowMs - lastUpdateMs_ : 0u;
    lastUpdateMs_ = in.nowMs;
    stats_.updates++;

    if (!cfg.enabled) addReject(out, MAG_YAW_REJECT_DISABLED);
    if (!in.referenceValid) addReject(out, MAG_YAW_REJECT_NO_REFERENCE);
    if (!heading.valid) addReject(out, MAG_YAW_REJECT_HEADING_INVALID);
    if (!in.magTrustedForUse || !in.fieldReliable) addReject(out, MAG_YAW_REJECT_MAG_NOT_TRUSTED);
    if (in.magRejectFlagsForUse & MAG_REJECT_STALE) addReject(out, MAG_YAW_REJECT_MAG_STALE);

    out.magAgeMs = MagRuntimeProcessor::ageMsForUse(mag, in.nowMs);
    if (out.magAgeMs > cfg.maxMagAgeMs) addReject(out, MAG_YAW_REJECT_MAG_STALE);

    if (in.horizontalTrustValid) {
        out.horizontalTrust = in.horizontalTrust;
        out.horizontalReferenceNorm = in.horizontalReferenceNorm;
        out.horizontalEffectiveBad = in.horizontalEffectiveBad;
        out.horizontalEffectiveGood = in.horizontalEffectiveGood;
    } else {
        // Preserve the standalone/controller API for callers that do not own
        // field reliability. Without a field reference there is no adaptive
        // geometry to share, so use only the configured absolute legacy ramp.
        out.horizontalReferenceNorm = 0.0f;
        out.horizontalEffectiveBad = cfg.horizontalNormBad;
        out.horizontalEffectiveGood = cfg.horizontalNormGood;
        out.horizontalTrust = rampUp(
            heading.horizontalNorm, cfg.horizontalNormBad, cfg.horizontalNormGood);
    }
    if (!tracker::isFinite(out.horizontalTrust) || out.horizontalTrust <= 0.0f) {
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

    bool innovationLarge = false;
    bool reacquireEligible = false;
    if (in.referenceValid && heading.valid) {
        out.errorRad = wrapPi(heading.magneticNorthWorldYawRad - in.referenceWorldYawRad);
        out.errorDeg = out.errorRad * MATH_RAD_TO_DEG;
        const float absErr = std::fabs(out.errorDeg);
        stats_.lastAbsErrorDeg = absErr;
        if (stats_.errorSamples == 0 || absErr > stats_.maxAbsErrorDeg) stats_.maxAbsErrorDeg = absErr;
        stats_.sumAbsErrorDeg += static_cast<double>(absErr);
        stats_.errorSamples++;

        if (!tracker::isFinite(out.errorRad) || !tracker::isFinite(out.errorDeg)) {
            addReject(out, MAG_YAW_REJECT_NONFINITE);
        } else if (absErr > cfg.maxInnovationDeg) {
            innovationLarge = true;
            const bool withinReacquireRange = absErr <= cfg.reacquireInnovationMaxDeg;
            const bool reacquireConditions = in.fieldReliable &&
                std::fabs(in.magneticHeadingRateDegS) <= cfg.reacquireMaxHeadingRateDegS &&
                in.gyroNormDps <= cfg.gyroNormGoodDps &&
                in.accelTrust >= cfg.accelTrustGood;
            if (reacquireConditions && withinReacquireRange) {
                if (reacquireCandidateSinceMs_ == 0u) reacquireCandidateSinceMs_ = in.nowMs;
            } else {
                reacquireCandidateSinceMs_ = 0u;
            }
            const uint32_t localStableMs = reacquireCandidateSinceMs_ == 0u
                ? 0u : in.nowMs - reacquireCandidateSinceMs_;
            const bool fieldStable = in.fieldStableMs >= cfg.reacquireMinFieldStableMs &&
                                     localStableMs >= cfg.reacquireMinFieldStableMs;
            reacquireEligible = cfg.reacquisitionEnabled && withinReacquireRange && fieldStable;
            if (reacquireEligible) {
                out.mode = MagYawCorrectionMode::Reacquiring;
                out.reacquireActive = true;
                if (!reacquiring_) stats_.reacquireGateOpenCount++;
                reacquiring_ = true;
            } else if (cfg.reacquisitionEnabled && withinReacquireRange && in.fieldReliable) {
                reacquiring_ = false;
                out.reacquirePending = true;
                addReject(out, MAG_YAW_REJECT_REACQUIRE_PENDING);
            } else {
                reacquiring_ = false;
                addReject(out, MAG_YAW_REJECT_INNOVATION_TOO_LARGE);
            }
        } else {
            reacquireCandidateSinceMs_ = 0u;
            if (reacquiring_) {
                reacquiring_ = false;
                stats_.reacquireCompletedCount++;
            }
        }
    }

    const uint32_t instantRejectFlags = out.rejectFlags;
    if (instantRejectFlags & MAG_YAW_REJECT_GYRO_MOVING) {
        requestCooldown(in.nowMs, cfg.gyroMovingCooldownMs, instantRejectFlags);
    }
    if (instantRejectFlags & MAG_YAW_REJECT_ACCEL_NOT_TRUSTED) {
        requestCooldown(in.nowMs, cfg.accelBadCooldownMs, instantRejectFlags);
    }
    if (instantRejectFlags & (MAG_YAW_REJECT_HEADING_INVALID |
                              MAG_YAW_REJECT_MAG_NOT_TRUSTED |
                              MAG_YAW_REJECT_HORIZONTAL_BAD)) {
        requestCooldown(in.nowMs, cfg.magDisturbanceCooldownMs, instantRejectFlags);
    }

    if (cfg.enabled && in.referenceValid && cooldownActive(in.nowMs)) {
        addReject(out, MAG_YAW_REJECT_COOLDOWN);
        out.cooldownActive = true;
        out.cooldownRemainingMs = cooldownRemainingMs(in.nowMs);
        out.cooldownReasonFlags = cooldownReasonFlags_;
    }

    float dtS = static_cast<float>(out.dtMs) * 0.001f;
    if (dtS <= 0.0f || dtS > 1.0f || !tracker::isFinite(dtS)) dtS = cfg.fallbackDtS;
    if (dtS <= 0.0f || !tracker::isFinite(dtS)) addReject(out, MAG_YAW_REJECT_DT_INVALID);

    out.combinedTrust = out.horizontalTrust * out.gyroTrust;
    if (cfg.requireAccelTrusted) out.combinedTrust *= out.accelGateTrust;
    out.combinedTrust = clampf(out.combinedTrust, 0.0f, 1.0f);

    if (out.rejectFlags == MAG_YAW_REJECT_NONE) {
        out.gateOpen = true;
        stats_.gateOpenCount++;

        const bool reacquireMode = innovationLarge && reacquireEligible;
        const float tc = reacquireMode
            ? (cfg.reacquireTimeConstantS > 0.001f ? cfg.reacquireTimeConstantS : 90.0f)
            : (cfg.timeConstantS > 0.001f ? cfg.timeConstantS : 30.0f);
        const float maxRateDegS = reacquireMode
            ? cfg.reacquireMaxCorrectionRateDegS
            : cfg.maxCorrectionRateDegS;
        const float maxStepDeg = reacquireMode
            ? cfg.reacquireMaxCorrectionStepDeg
            : cfg.maxCorrectionStepDeg;

        float rateRadS = -out.errorRad / tc;
        rateRadS *= out.combinedTrust;
        const float maxRateRadS = maxRateDegS * MATH_DEG_TO_RAD;
        rateRadS = clampf(rateRadS, -maxRateRadS, maxRateRadS);
        float stepRad = rateRadS * dtS;
        const float maxStepRad = maxStepDeg * MATH_DEG_TO_RAD;
        stepRad = clampf(stepRad, -maxStepRad, maxStepRad);

        out.correctionRateRadS = rateRadS;
        out.correctionRateDegS = rateRadS * MATH_RAD_TO_DEG;
        out.correctionStepRad = stepRad;
        out.correctionStepDeg = stepRad * MATH_RAD_TO_DEG;

        if (cfg.applyEnabled) {
            out.applyAllowed = true;
            stats_.applyAllowedCount++;
            if (reacquireMode) stats_.reacquireAppliedCount++;
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
        if (absStep > stats_.maxAbsCorrectionStepDeg) stats_.maxAbsCorrectionStepDeg = absStep;
        stats_.sumCorrectionStepDeg += static_cast<double>(out.correctionStepDeg);
    }

    last_ = out;
    return out.valid;
}

bool MagYawCorrectionController::update(const MagYawCorrectionInput& in,
                                        const MagYawCorrectionConfig& cfg) {
    MagYawCorrectionOutput out;
    return update(in, cfg, out);
}

bool MagYawCorrectionController::timeBefore(uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a - b) < 0;
}

void MagYawCorrectionController::expireCooldownIfNeeded(uint32_t nowMs) {
    if (cooldownUntilMs_ == 0) return;
    if (!timeBefore(nowMs, cooldownUntilMs_)) {
        cooldownUntilMs_ = 0;
        cooldownReasonFlags_ = MAG_YAW_REJECT_NONE;
    }
}

void MagYawCorrectionController::requestCooldown(uint32_t nowMs, uint32_t durationMs, uint32_t reasonFlags) {
    if (durationMs == 0) return;

    const uint32_t until = nowMs + durationMs;
    if (cooldownUntilMs_ == 0 || timeBefore(cooldownUntilMs_, until)) {
        cooldownUntilMs_ = until;
        cooldownReasonFlags_ = reasonFlags;
    } else {
        cooldownReasonFlags_ |= reasonFlags;
    }
}

bool MagYawCorrectionController::cooldownActive(uint32_t nowMs) const {
    return cooldownUntilMs_ != 0 && timeBefore(nowMs, cooldownUntilMs_);
}

uint32_t MagYawCorrectionController::cooldownRemainingMs(uint32_t nowMs) const {
    if (!cooldownActive(nowMs)) return 0;
    return cooldownUntilMs_ - nowMs;
}

float MagYawCorrectionController::rampUp(float x, float bad, float good) {
    if (x >= good) return 1.0f;
    if (x <= bad) return 0.0f;
    if (good <= bad) return 0.0f;
    return (x - bad) / (good - bad);
}

float MagYawCorrectionController::rampDown(float x, float good, float bad) {
    if (x <= good) return 1.0f;
    if (x >= bad) return 0.0f;
    if (bad <= good) return 0.0f;
    return 1.0f - ((x - good) / (bad - good));
}

void MagYawCorrectionController::addReject(MagYawCorrectionOutput& out, uint32_t flag) {
    out.rejectFlags |= flag;
}

void MagYawCorrectionController::countRejects(uint32_t flags) {
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
    if (flags & MAG_YAW_REJECT_REACQUIRE_PENDING)     stats_.rejectReacquirePending++;
}

} // namespace tracker
