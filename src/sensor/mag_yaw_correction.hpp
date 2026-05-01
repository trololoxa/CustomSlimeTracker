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
    MAG_YAW_REJECT_NO_REFERENCE          = 1u << 1,
    MAG_YAW_REJECT_HEADING_INVALID       = 1u << 2,
    MAG_YAW_REJECT_MAG_NOT_TRUSTED       = 1u << 3,
    MAG_YAW_REJECT_MAG_STALE             = 1u << 4,
    MAG_YAW_REJECT_HORIZONTAL_SMALL      = 1u << 5,
    MAG_YAW_REJECT_INNOVATION_TOO_LARGE  = 1u << 6,
    MAG_YAW_REJECT_DT_INVALID            = 1u << 7,
    MAG_YAW_REJECT_NONFINITE             = 1u << 8,
};

struct MagYawCorrectionConfig {
    // Dry-run controller is enabled by default, but it does not apply anything.
    bool enabled = true;
    bool dryRun = true;

    // Safety gates.
    float maxInnovationDeg = 25.0f;
    float minHorizontalNorm = 220.0f;
    uint32_t maxMagAgeMs = 250;

    // Correction dynamics.
    // error / timeConstant = target correction rate.
    float timeConstantS = 30.0f;

    // Hard limits.
    float maxCorrectionRateDegS = 2.0f;
    float maxCorrectionStepDeg = 0.25f;

    // Used only if controller update interval is weird.
    float fallbackDtS = 1.0f / 60.0f;
};

struct MagYawCorrectionInput {
    MagProcessedSample mag;
    MagHeadingSample heading;

    bool referenceValid = false;
    float referenceWorldYawRad = 0.0f;

    // Use-time mag trust, including stale.
    bool magTrustedForUse = false;
    uint32_t magRejectFlagsForUse = MAG_REJECT_NONE;

    uint32_t nowMs = 0;
};

struct MagYawCorrectionOutput {
    bool valid = false;
    bool gateOpen = false;
    bool wouldApply = false;

    uint32_t rejectFlags = MAG_YAW_REJECT_NONE;

    uint32_t nowMs = 0;
    uint32_t dtMs = 0;

    float errorRad = 0.0f;
    float errorDeg = 0.0f;

    float correctionRateRadS = 0.0f;
    float correctionRateDegS = 0.0f;

    float correctionStepRad = 0.0f;
    float correctionStepDeg = 0.0f;

    float horizontalNorm = 0.0f;
    uint32_t magAgeMs = 0;

    uint32_t magSeq = 0;
    uint64_t magTimestampUs = 0;
};

struct MagYawCorrectionStats {
    uint32_t updates = 0;
    uint32_t gateOpenCount = 0;
    uint32_t gateClosedCount = 0;
    uint32_t wouldApplyCount = 0;

    uint32_t rejectDisabled = 0;
    uint32_t rejectNoReference = 0;
    uint32_t rejectHeadingInvalid = 0;
    uint32_t rejectMagNotTrusted = 0;
    uint32_t rejectMagStale = 0;
    uint32_t rejectHorizontalSmall = 0;
    uint32_t rejectInnovationTooLarge = 0;
    uint32_t rejectDtInvalid = 0;
    uint32_t rejectNonfinite = 0;

    float lastAbsErrorDeg = 0.0f;
    float maxAbsErrorDeg = 0.0f;
    double sumAbsErrorDeg = 0.0;
    uint32_t errorSamples = 0;

    float meanAbsErrorDeg() const {
        return errorSamples > 0 ? static_cast<float>(sumAbsErrorDeg / static_cast<double>(errorSamples)) : 0.0f;
    }
};

class MagYawCorrectionController {
public:
    void reset() {
        last_ = MagYawCorrectionOutput{};
        stats_ = MagYawCorrectionStats{};
        lastUpdateMs_ = 0;
    }

    const MagYawCorrectionOutput& last() const { return last_; }
    const MagYawCorrectionStats& stats() const { return stats_; }

    bool update(const MagYawCorrectionInput& in,
                const MagYawCorrectionConfig& cfg,
                MagYawCorrectionOutput& out) {
        out = MagYawCorrectionOutput{};
        out.valid = true;
        out.nowMs = in.nowMs;
        out.horizontalNorm = in.heading.horizontalNorm;
        out.magSeq = in.mag.seq;
        out.magTimestampUs = in.mag.t_us;

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

        if (!tracker::isFinite(in.heading.horizontalNorm) ||
            in.heading.horizontalNorm < cfg.minHorizontalNorm) {
            addReject(out, MAG_YAW_REJECT_HORIZONTAL_SMALL);
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

        float dtS = static_cast<float>(out.dtMs) * 0.001f;
        if (dtS <= 0.0f || dtS > 1.0f || !tracker::isFinite(dtS)) {
            dtS = cfg.fallbackDtS;
        }

        if (dtS <= 0.0f || !tracker::isFinite(dtS)) {
            addReject(out, MAG_YAW_REJECT_DT_INVALID);
        }

        if (out.rejectFlags == MAG_YAW_REJECT_NONE) {
            out.gateOpen = true;
            stats_.gateOpenCount++;

            const float tc = cfg.timeConstantS > 0.001f ? cfg.timeConstantS : 30.0f;

            // Negative feedback:
            // If magnetic north in world frame moved positive relative to ref,
            // future real correction should rotate the AHRS yaw estimate back.
            float rateRadS = -out.errorRad / tc;

            const float maxRateRadS = cfg.maxCorrectionRateDegS * MATH_DEG_TO_RAD;
            rateRadS = clampf(rateRadS, -maxRateRadS, maxRateRadS);

            float stepRad = rateRadS * dtS;

            const float maxStepRad = cfg.maxCorrectionStepDeg * MATH_DEG_TO_RAD;
            stepRad = clampf(stepRad, -maxStepRad, maxStepRad);

            out.correctionRateRadS = rateRadS;
            out.correctionRateDegS = rateRadS * MATH_RAD_TO_DEG;
            out.correctionStepRad = stepRad;
            out.correctionStepDeg = stepRad * MATH_RAD_TO_DEG;

            out.wouldApply = !cfg.dryRun;
            if (out.wouldApply) {
                stats_.wouldApplyCount++;
            }
        } else {
            out.gateOpen = false;
            stats_.gateClosedCount++;
            countRejects(out.rejectFlags);
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
    void addReject(MagYawCorrectionOutput& out, uint32_t flag) {
        out.rejectFlags |= flag;
    }

    void countRejects(uint32_t flags) {
        if (flags & MAG_YAW_REJECT_DISABLED)             stats_.rejectDisabled++;
        if (flags & MAG_YAW_REJECT_NO_REFERENCE)         stats_.rejectNoReference++;
        if (flags & MAG_YAW_REJECT_HEADING_INVALID)      stats_.rejectHeadingInvalid++;
        if (flags & MAG_YAW_REJECT_MAG_NOT_TRUSTED)      stats_.rejectMagNotTrusted++;
        if (flags & MAG_YAW_REJECT_MAG_STALE)            stats_.rejectMagStale++;
        if (flags & MAG_YAW_REJECT_HORIZONTAL_SMALL)     stats_.rejectHorizontalSmall++;
        if (flags & MAG_YAW_REJECT_INNOVATION_TOO_LARGE) stats_.rejectInnovationTooLarge++;
        if (flags & MAG_YAW_REJECT_DT_INVALID)           stats_.rejectDtInvalid++;
        if (flags & MAG_YAW_REJECT_NONFINITE)            stats_.rejectNonfinite++;
    }

    MagYawCorrectionOutput last_;
    MagYawCorrectionStats stats_;
    uint32_t lastUpdateMs_ = 0;
};

} // namespace tracker