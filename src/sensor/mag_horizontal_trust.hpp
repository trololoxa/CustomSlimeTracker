#pragma once

#include <algorithm>
#include <cmath>

#include "core/math.hpp"

namespace tracker {

// Horizontal magnetic heading becomes noisy as the local field approaches the
// vertical axis. Existing user-configured absolute thresholds remain hard
// upper caps, while an acquired environmental reference may lower them to a
// physically appropriate range. This keeps old conservative settings useful
// in strong-horizontal-field environments without rejecting a healthy field
// solely because local inclination makes its horizontal component smaller.
struct MagHorizontalTrustConfig {
    float absoluteBadFloor = 20.0f;
    float absoluteGoodFloor = 40.0f;
    float referenceBadFraction = 0.35f;
    float referenceGoodFraction = 0.65f;
    float headingNoiseScaleMin = 1.0f;
    float headingNoiseScaleMax = 2.0f;
};

struct MagHorizontalTrustResult {
    bool valid = false;
    float trust = 0.0f;
    float effectiveBad = 0.0f;
    float effectiveGood = 0.0f;
    // Square of the geometry multiplier used by heading direction gates.
    // Keeping this squared lets the hot path compare squared angles/rates and
    // avoid a per-sample sqrt; diagnostics may recover the human-readable
    // scale outside the sensor callback.
    float headingNoiseScaleSquared = 1.0f;
};

inline float magHorizontalNormFromField(float fieldNorm, float dipDeg) {
    if (!tracker::isFinite(fieldNorm) || !tracker::isFinite(dipDeg) || fieldNorm <= 0.0f) {
        return 0.0f;
    }
    const float horizontal = std::fabs(fieldNorm * std::cos(dipDeg * MATH_DEG_TO_RAD));
    return tracker::isFinite(horizontal) ? horizontal : 0.0f;
}

inline MagHorizontalTrustResult evaluateMagHorizontalTrustFromReference(
    float horizontalNorm,
    float configuredBad,
    float configuredGood,
    float referenceHorizontalNorm,
    float referenceFieldNorm,
    const MagHorizontalTrustConfig& cfg) {
    MagHorizontalTrustResult out;
    if (!tracker::isFinite(horizontalNorm) || horizontalNorm < 0.0f ||
        !tracker::isFinite(configuredBad) || !tracker::isFinite(configuredGood) ||
        configuredBad < 0.0f || configuredGood <= configuredBad ||
        !tracker::isFinite(referenceHorizontalNorm) || referenceHorizontalNorm < 0.0f ||
        !tracker::isFinite(referenceFieldNorm) || referenceFieldNorm < 0.0f ||
        !tracker::isFinite(cfg.absoluteBadFloor) || cfg.absoluteBadFloor < 0.0f ||
        !tracker::isFinite(cfg.absoluteGoodFloor) ||
        cfg.absoluteGoodFloor <= cfg.absoluteBadFloor ||
        !tracker::isFinite(cfg.referenceBadFraction) || cfg.referenceBadFraction <= 0.0f ||
        !tracker::isFinite(cfg.referenceGoodFraction) ||
        cfg.referenceGoodFraction <= cfg.referenceBadFraction ||
        !tracker::isFinite(cfg.headingNoiseScaleMin) || cfg.headingNoiseScaleMin < 1.0f ||
        !tracker::isFinite(cfg.headingNoiseScaleMax) ||
        cfg.headingNoiseScaleMax < cfg.headingNoiseScaleMin) {
        return out;
    }

    out.effectiveBad = configuredBad;
    out.effectiveGood = configuredGood;

    if (referenceHorizontalNorm > MATH_EPSILON) {
        // Absolute floors prevent a nearly vertical or badly calibrated field
        // from being treated as a usable heading source. Fractions preserve a
        // broad disturbance margin around the acquired local reference.
        const float adaptiveBad = std::max(
            cfg.absoluteBadFloor,
            referenceHorizontalNorm * cfg.referenceBadFraction);
        const float adaptiveGood = std::max(
            cfg.absoluteGoodFloor,
            referenceHorizontalNorm * cfg.referenceGoodFraction);
        out.effectiveBad = std::min(configuredBad, adaptiveBad);
        out.effectiveGood = std::min(configuredGood, adaptiveGood);
        if (out.effectiveGood <= out.effectiveBad + 1.0f) {
            out.effectiveGood = out.effectiveBad + 1.0f;
        }

        const float geometryRatio = referenceFieldNorm /
            std::max(referenceHorizontalNorm, MATH_EPSILON);
        const float scaleMinSquared = cfg.headingNoiseScaleMin * cfg.headingNoiseScaleMin;
        const float scaleMaxSquared = cfg.headingNoiseScaleMax * cfg.headingNoiseScaleMax;
        out.headingNoiseScaleSquared = clampf(
            std::max(geometryRatio, 1.0f),
            scaleMinSquared,
            scaleMaxSquared);
    }

    if (horizontalNorm >= out.effectiveGood) {
        out.trust = 1.0f;
    } else if (horizontalNorm <= out.effectiveBad) {
        out.trust = 0.0f;
    } else {
        out.trust = (horizontalNorm - out.effectiveBad) /
                    (out.effectiveGood - out.effectiveBad);
    }
    out.trust = clampf(out.trust, 0.0f, 1.0f);
    out.valid = true;
    return out;
}

inline float magHeadingNoiseScale(float headingNoiseScaleSquared) {
    if (!tracker::isFinite(headingNoiseScaleSquared) || headingNoiseScaleSquared <= 0.0f) {
        return 1.0f;
    }
    return std::sqrt(headingNoiseScaleSquared);
}

inline MagHorizontalTrustResult evaluateMagHorizontalTrust(
    float horizontalNorm,
    float configuredBad,
    float configuredGood,
    float referenceFieldNorm,
    float referenceDipDeg,
    const MagHorizontalTrustConfig& cfg) {
    return evaluateMagHorizontalTrustFromReference(
        horizontalNorm,
        configuredBad,
        configuredGood,
        magHorizontalNormFromField(referenceFieldNorm, referenceDipDeg),
        referenceFieldNorm,
        cfg);
}

} // namespace tracker
