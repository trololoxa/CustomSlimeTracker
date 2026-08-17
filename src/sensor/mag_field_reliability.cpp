#include "sensor/mag_field_reliability.hpp"

#include <algorithm>
#include <cmath>

namespace tracker {

namespace {

bool finiteInput(const MagProcessedSample& mag,
                 const MagHeadingSample& heading,
                 float gyroNormDps,
                 float accelTrust) {
    return mag.valid && heading.valid &&
           tracker::isFinite(mag.bodyNorm) && mag.bodyNorm > MATH_EPSILON &&
           tracker::isFinite(heading.dipDeg) &&
           tracker::isFinite(heading.magneticNorthWorldYawRad) &&
           tracker::isFinite(heading.currentAhrsYawRad) &&
           tracker::isFinite(heading.yawInnovationRad) &&
           tracker::isFinite(gyroNormDps) && tracker::isFinite(accelTrust);
}

uint32_t elapsedMs(uint32_t nowMs, uint32_t sinceMs) {
    return sinceMs == 0u ? 0u : nowMs - sinceMs;
}

bool magnitudeAtLeastScaledThreshold(float magnitude,
                                     float baseThreshold,
                                     float scaleSquared) {
    return magnitude * magnitude >=
           baseThreshold * baseThreshold * scaleSquared;
}

bool magnitudeExceedsScaledThreshold(float magnitude,
                                     float baseThreshold,
                                     float scaleSquared) {
    return magnitude * magnitude >
           baseThreshold * baseThreshold * scaleSquared;
}

bool magnitudeAtMostScaledThreshold(float magnitude,
                                    float baseThreshold,
                                    float scaleSquared) {
    return magnitude * magnitude <=
           baseThreshold * baseThreshold * scaleSquared;
}

} // namespace

void MagFieldReliabilityMonitor::reset() {
    *this = MagFieldReliabilityMonitor{};
}

void MagFieldReliabilityMonitor::restartAcquisition() {
    state_ = MagFieldReliabilityState::Acquiring;
    referenceValid_ = false;
    referenceNorm_ = 0.0f;
    referenceDipDeg_ = 0.0f;
    referenceHorizontalNorm_ = 0.0f;
    referenceHeadingYawRad_ = 0.0f;
    stateSinceMs_ = 0;
    goodSinceMs_ = 0;
    consecutiveBad_ = 0;
    newEnvironmentSinceMs_ = 0;
    environmentChangedLatched_ = false;
    havePreviousHeading_ = false;
    previousStationaryFieldYawRad_ = 0.0f;
    previousHeadingMs_ = 0;
    headingRateInitialized_ = false;
    filteredHeadingRateDegS_ = 0.0f;
    stationaryWindowActive_ = false;
    stationaryWindowStartMs_ = 0;
    stationaryWindowStartFieldYawRad_ = 0.0f;
    stationaryHeadingJumpLatched_ = false;
    stationaryLatchReferenceFieldYawRad_ = 0.0f;
    stationaryLatchMotionSeen_ = false;
    referenceReturnSinceMs_ = 0;
    resetAcquisitionAccumulator();
}

void MagFieldReliabilityMonitor::transition(MagFieldReliabilityState next, uint32_t nowMs) {
    if (state_ == next) return;
    state_ = next;
    stateSinceMs_ = nowMs;
    stats_.stateTransitions++;
    if (next == MagFieldReliabilityState::Disturbed) {
        newEnvironmentSinceMs_ = 0u;
        environmentChangedLatched_ = false;
        stats_.enteredDisturbed++;
    }
    if (next == MagFieldReliabilityState::Recovering) {
        newEnvironmentSinceMs_ = 0u;
        environmentChangedLatched_ = false;
        stats_.enteredRecovering++;
    }
    if (next == MagFieldReliabilityState::Trusted) {
        newEnvironmentSinceMs_ = 0u;
        environmentChangedLatched_ = false;
    }
}

void MagFieldReliabilityMonitor::resetAcquisitionAccumulator() {
    acquireNormSum_ = 0.0;
    acquireDipSum_ = 0.0;
    acquireHorizontalNormSum_ = 0.0;
    acquireHeadingSinSum_ = 0.0;
    acquireHeadingCosSum_ = 0.0;
    acquireSampleCount_ = 0;
}

void MagFieldReliabilityMonitor::accumulateReferenceSample(const MagProcessedSample& mag,
                                                           const MagHeadingSample& heading) {
    acquireNormSum_ += static_cast<double>(mag.bodyNorm);
    acquireDipSum_ += static_cast<double>(heading.dipDeg);
    acquireHorizontalNormSum_ += static_cast<double>(heading.horizontalNorm);
    acquireHeadingSinSum_ += std::sin(static_cast<double>(heading.magneticNorthWorldYawRad));
    acquireHeadingCosSum_ += std::cos(static_cast<double>(heading.magneticNorthWorldYawRad));
    acquireSampleCount_++;
}

void MagFieldReliabilityMonitor::acquireReferenceFromAccumulator(
    const MagProcessedSample& fallbackMag,
    const MagHeadingSample& fallbackHeading,
    uint32_t nowMs) {
    if (acquireSampleCount_ > 0u) {
        const double inv = 1.0 / static_cast<double>(acquireSampleCount_);
        referenceNorm_ = static_cast<float>(acquireNormSum_ * inv);
        referenceDipDeg_ = static_cast<float>(acquireDipSum_ * inv);
        referenceHorizontalNorm_ = static_cast<float>(acquireHorizontalNormSum_ * inv);
        referenceHeadingYawRad_ = static_cast<float>(
            std::atan2(acquireHeadingSinSum_, acquireHeadingCosSum_));
    } else {
        referenceNorm_ = fallbackMag.bodyNorm;
        referenceDipDeg_ = fallbackHeading.dipDeg;
        referenceHorizontalNorm_ = fallbackHeading.horizontalNorm;
        referenceHeadingYawRad_ = fallbackHeading.magneticNorthWorldYawRad;
    }
    referenceValid_ = true;
    stationaryWindowActive_ = true;
    stationaryWindowStartMs_ = nowMs;
    stationaryWindowStartFieldYawRad_ = wrapPi(fallbackHeading.yawInnovationRad);
    resetAcquisitionAccumulator();
    stats_.referencesAcquired++;
}

void MagFieldReliabilityMonitor::adaptReference(const MagProcessedSample& mag,
                                                const MagHeadingSample& heading,
                                                float gyroNormDps,
                                                float accelTrust,
                                                const MagFieldReliabilityConfig& cfg,
                                                uint32_t dtMs) {
    if (!referenceValid_ || dtMs == 0u || cfg.referenceAdaptTimeConstantS <= 0.0f) return;
    if (gyroNormDps > cfg.stationaryGyroMaxDps ||
        accelTrust < cfg.stationaryAccelTrustMin) return;

    const float dtS = static_cast<float>(dtMs) * 0.001f;
    const float alpha = clampf(dtS / (cfg.referenceAdaptTimeConstantS + dtS), 0.0f, 0.02f);
    referenceNorm_ += alpha * (mag.bodyNorm - referenceNorm_);
    referenceDipDeg_ += alpha * (heading.dipDeg - referenceDipDeg_);
    referenceHorizontalNorm_ += alpha *
        (heading.horizontalNorm - referenceHorizontalNorm_);
}

bool MagFieldReliabilityMonitor::update(const MagFieldReliabilityInputView& in,
                                        const MagFieldReliabilityConfig& cfg,
                                        MagFieldReliabilityOutput& out) {
    if (in.mag == nullptr || in.heading == nullptr) {
        out = MagFieldReliabilityOutput{};
        out.nowMs = in.nowMs;
        out.state = state_;
        out.flags = MAG_FIELD_FLAG_INPUT_INVALID;
        stats_.updates++;
        stats_.rejectedSamples++;
        return false;
    }
    const MagProcessedSample& mag = *in.mag;
    const MagHeadingSample& heading = *in.heading;
    const bool processorTrustedForUse = in.processorTrustedForUse;
    const float gyroNormDps = in.gyroNormDps;
    const float accelTrust = in.accelTrust;
    const uint32_t nowMs = in.nowMs;
    out = MagFieldReliabilityOutput{};
    out.nowMs = nowMs;
    out.state = state_;
    stats_.updates++;

    const uint32_t previousMs = previousHeadingMs_;
    const bool inputFinite = finiteInput(mag, heading, gyroNormDps, accelTrust);
    if (!inputFinite) out.flags |= MAG_FIELD_FLAG_INPUT_INVALID;
    if (!processorTrustedForUse) out.flags |= MAG_FIELD_FLAG_PROCESSOR_UNTRUSTED;

    const float stationaryFieldYawRad = inputFinite
        ? wrapPi(heading.yawInnovationRad)
        : 0.0f;
    if (inputFinite) {
        out.valid = true;
        out.fieldNorm = mag.bodyNorm;
        out.dipDeg = heading.dipDeg;
        out.stationaryFieldYawDeg = stationaryFieldYawRad * MATH_RAD_TO_DEG;

        if (havePreviousHeading_) {
            const uint32_t dtMs = nowMs - previousHeadingMs_;
            // Remove AHRS yaw from the magnetic-world heading before measuring
            // field motion. A bounded yaw correction or an explicit AHRS yaw
            // rebase rotates both terms together and therefore produces no
            // synthetic magnetic-field step.
            const float signedStepDeg = wrapPi(
                stationaryFieldYawRad - previousStationaryFieldYawRad_) * MATH_RAD_TO_DEG;
            out.headingStepDeg = std::fabs(signedStepDeg);
            if (dtMs > 0u && dtMs <= cfg.headingRateResetGapMs) {
                const float instantRate = signedStepDeg * 1000.0f / static_cast<float>(dtMs);
                out.headingInstantRateDegS = std::fabs(instantRate);
                const float dtS = static_cast<float>(dtMs) * 0.001f;
                const float tau = std::max(cfg.headingRateFilterTimeConstantS, 0.05f);
                const float alpha = clampf(dtS / (tau + dtS), 0.0f, 1.0f);
                if (!headingRateInitialized_) {
                    filteredHeadingRateDegS_ = instantRate;
                    headingRateInitialized_ = true;
                } else {
                    filteredHeadingRateDegS_ += alpha * (instantRate - filteredHeadingRateDegS_);
                }
            } else {
                headingRateInitialized_ = false;
                filteredHeadingRateDegS_ = 0.0f;
            }
        }
        out.headingRateDegS = std::fabs(filteredHeadingRateDegS_);
        previousStationaryFieldYawRad_ = stationaryFieldYawRad;
        previousHeadingMs_ = nowMs;
        havePreviousHeading_ = true;
    }

    if (!referenceValid_) {
        out.flags |= MAG_FIELD_FLAG_REFERENCE_MISSING;
    } else if (inputFinite) {
        out.referenceNorm = referenceNorm_;
        out.referenceDipDeg = referenceDipDeg_;
        out.referenceHeadingYawDeg = referenceHeadingYawRad_ * MATH_RAD_TO_DEG;
        out.referenceHeadingErrorDeg = std::fabs(wrapPi(
            heading.magneticNorthWorldYawRad - referenceHeadingYawRad_)) * MATH_RAD_TO_DEG;
        out.normRelativeError = std::fabs(mag.bodyNorm - referenceNorm_) /
                                (std::fabs(referenceNorm_) + MATH_EPSILON);
        out.dipErrorDeg = std::fabs(heading.dipDeg - referenceDipDeg_);
        if (out.normRelativeError > cfg.normSoftFraction) out.flags |= MAG_FIELD_FLAG_NORM_SOFT;
        if (out.normRelativeError > cfg.normHardFraction) out.flags |= MAG_FIELD_FLAG_NORM_HARD;
        const bool dipObservable = gyroNormDps <= 120.0f && accelTrust >= 0.20f;
        if (dipObservable && out.dipErrorDeg > cfg.dipSoftDeg) out.flags |= MAG_FIELD_FLAG_DIP_SOFT;
        if (dipObservable && out.dipErrorDeg > cfg.dipHardDeg) out.flags |= MAG_FIELD_FLAG_DIP_HARD;
    }

    const MagHorizontalTrustResult horizontal = evaluateMagHorizontalTrustFromReference(
        inputFinite ? heading.horizontalNorm : 0.0f,
        in.horizontalNormBad,
        in.horizontalNormGood,
        referenceValid_ ? referenceHorizontalNorm_ : 0.0f,
        referenceValid_ ? referenceNorm_ : 0.0f,
        cfg.horizontalTrust
    );
    out.horizontalNorm = inputFinite ? heading.horizontalNorm : 0.0f;
    out.referenceHorizontalNorm = referenceValid_ ? referenceHorizontalNorm_ : 0.0f;
    out.horizontalTrust = horizontal.trust;
    out.horizontalEffectiveBad = horizontal.effectiveBad;
    out.horizontalEffectiveGood = horizontal.effectiveGood;
    const bool headingObservable = horizontal.valid &&
        horizontal.trust >= cfg.minimumHeadingTrustForJumpDetection;
    if (referenceValid_ && !headingObservable) {
        out.flags |= MAG_FIELD_FLAG_HORIZONTAL_UNOBSERVABLE;
    }

    out.headingNoiseScaleSquared = horizontal.valid
        ? horizontal.headingNoiseScaleSquared
        : 1.0f;
    out.stationaryHeadingJumpRateGyroFloorDegS =
        gyroNormDps + cfg.stationaryHeadingJumpRateMarginOverGyroDegS;

    const bool stationaryForHeadingCheck = inputFinite &&
        gyroNormDps <= cfg.stationaryGyroMaxDps &&
        accelTrust >= cfg.stationaryAccelTrustMin;
    const bool stationaryForHeadingJumpCheck = stationaryForHeadingCheck &&
        headingObservable;

    // A one-sample heading step is not enough by itself because the existing
    // soft gate can clear once the disturbed field becomes temporally stable.
    // Keep a short stationary-window anchor and latch a discontinuity when the
    // cumulative change is too fast to be ordinary calibrated gyro drift or
    // our own bounded yaw correction.  The latch is intentionally independent
    // of the old 8/20 degree per-sample thresholds.
    if (!stationaryForHeadingJumpCheck || !referenceValid_) {
        if (stationaryHeadingJumpLatched_ && !stationaryForHeadingCheck) {
            stationaryLatchMotionSeen_ = true;
        }
        stationaryWindowActive_ = false;
        stationaryWindowStartMs_ = 0u;
        referenceReturnSinceMs_ = 0u;
    } else {
        if (!stationaryWindowActive_) {
            stationaryWindowActive_ = true;
            stationaryWindowStartMs_ = nowMs;
            stationaryWindowStartFieldYawRad_ = stationaryFieldYawRad;
        } else {
            const uint32_t windowMs = nowMs - stationaryWindowStartMs_;
            const float signedWindowDeltaDeg = wrapPi(
                stationaryFieldYawRad - stationaryWindowStartFieldYawRad_) *
                MATH_RAD_TO_DEG;
            out.stationaryWindowHeadingDeltaDeg = std::fabs(signedWindowDeltaDeg);
            if (windowMs > 0u) {
                out.stationaryWindowHeadingRateDegS =
                    out.stationaryWindowHeadingDeltaDeg * 1000.0f /
                    static_cast<float>(windowMs);
            }

            const bool withinDetectionWindow = windowMs <= cfg.stationaryHeadingWindowMs;
            const bool windowRateAboveGyroFloor =
                out.stationaryWindowHeadingRateDegS >=
                out.stationaryHeadingJumpRateGyroFloorDegS;
            const bool cumulativeDiscontinuity = withinDetectionWindow &&
                magnitudeAtLeastScaledThreshold(
                    out.stationaryWindowHeadingDeltaDeg,
                    cfg.stationaryHeadingJumpMinDeg,
                    out.headingNoiseScaleSquared) &&
                windowRateAboveGyroFloor &&
                magnitudeAtLeastScaledThreshold(
                    out.stationaryWindowHeadingRateDegS,
                    cfg.stationaryHeadingJumpRateDegS,
                    out.headingNoiseScaleSquared);
            // The rolling anchor is periodically refreshed.  Also test the
            // adjacent-sample step so an abrupt field jump cannot land exactly
            // on an anchor boundary and become the new undetected baseline.
            const bool instantaneousDiscontinuity = previousMs != 0u &&
                magnitudeAtLeastScaledThreshold(
                    out.headingStepDeg,
                    cfg.stationaryHeadingJumpMinDeg,
                    out.headingNoiseScaleSquared) &&
                out.headingInstantRateDegS >= out.stationaryHeadingJumpRateGyroFloorDegS &&
                magnitudeAtLeastScaledThreshold(
                    out.headingInstantRateDegS,
                    cfg.stationaryHeadingJumpRateDegS,
                    out.headingNoiseScaleSquared);
            const bool discontinuity = cumulativeDiscontinuity || instantaneousDiscontinuity;
            if (discontinuity && !stationaryHeadingJumpLatched_) {
                stationaryHeadingJumpLatched_ = true;
                stationaryLatchReferenceFieldYawRad_ = stationaryWindowStartFieldYawRad_;
                stationaryLatchMotionSeen_ = false;
                referenceReturnSinceMs_ = 0u;
                stats_.stationaryHeadingJumpsLatched++;
            }

            if (windowMs >= cfg.stationaryHeadingWindowMs) {
                stationaryWindowStartMs_ = nowMs;
                stationaryWindowStartFieldYawRad_ = stationaryFieldYawRad;
            }
        }
    }

    out.stationaryHeadingJumpLatched = stationaryHeadingJumpLatched_;
    out.stationaryLatchReferenceFieldYawDeg =
        stationaryLatchReferenceFieldYawRad_ * MATH_RAD_TO_DEG;
    out.stationaryLatchMotionSeen = stationaryLatchMotionSeen_;
    if (inputFinite && stationaryHeadingJumpLatched_) {
        out.stationaryLatchReferenceErrorDeg = std::fabs(wrapPi(
            stationaryFieldYawRad - stationaryLatchReferenceFieldYawRad_)) * MATH_RAD_TO_DEG;
    }
    if (stationaryHeadingJumpLatched_) {
        out.flags |= MAG_FIELD_FLAG_STATIONARY_HEADING_JUMP;
        stats_.stationaryHeadingJumpRejects++;
    }
    if (stationaryForHeadingCheck && headingObservable && previousMs != 0u) {
        if (magnitudeExceedsScaledThreshold(
                out.headingStepDeg,
                cfg.headingStepSoftDeg,
                out.headingNoiseScaleSquared)) {
            out.flags |= MAG_FIELD_FLAG_HEADING_STEP_SOFT;
        }
        if (magnitudeExceedsScaledThreshold(
                out.headingStepDeg,
                cfg.headingStepHardDeg,
                out.headingNoiseScaleSquared)) {
            out.flags |= MAG_FIELD_FLAG_HEADING_STEP_HARD;
        }
    }

    if (out.flags & MAG_FIELD_FLAG_NORM_SOFT) stats_.normSoftRejects++;
    if (out.flags & MAG_FIELD_FLAG_NORM_HARD) stats_.normHardRejects++;
    if (out.flags & MAG_FIELD_FLAG_DIP_SOFT) stats_.dipSoftRejects++;
    if (out.flags & MAG_FIELD_FLAG_DIP_HARD) stats_.dipHardRejects++;
    if (out.flags & MAG_FIELD_FLAG_HEADING_STEP_SOFT) stats_.headingStepSoftRejects++;
    if (out.flags & MAG_FIELD_FLAG_HEADING_STEP_HARD) stats_.headingStepHardRejects++;

    const uint32_t hardFlags = MAG_FIELD_FLAG_INPUT_INVALID |
                               MAG_FIELD_FLAG_PROCESSOR_UNTRUSTED |
                               MAG_FIELD_FLAG_NORM_HARD |
                               MAG_FIELD_FLAG_DIP_HARD |
                               MAG_FIELD_FLAG_HEADING_STEP_HARD;
    const uint32_t softFlags = MAG_FIELD_FLAG_NORM_SOFT |
                               MAG_FIELD_FLAG_DIP_SOFT |
                               MAG_FIELD_FLAG_HEADING_STEP_SOFT;
    const bool hardBad = (out.flags & hardFlags) != 0u;
    const bool softBad = (out.flags & softFlags) != 0u;
    const bool baseGood = inputFinite && processorTrustedForUse && !hardBad && !softBad;

    if (!inputFinite || !processorTrustedForUse) {
        goodSinceMs_ = 0;
        resetAcquisitionAccumulator();
        consecutiveBad_ = static_cast<uint8_t>(std::min<int>(255, consecutiveBad_ + 1));
        transition(MagFieldReliabilityState::Unavailable, nowMs);
    } else if (!referenceValid_) {
        const bool acquisitionGood = baseGood && stationaryForHeadingCheck;
        transition(MagFieldReliabilityState::Acquiring, nowMs);
        if (acquisitionGood) {
            if (goodSinceMs_ == 0u) {
                goodSinceMs_ = nowMs;
                resetAcquisitionAccumulator();
            }
            accumulateReferenceSample(mag, heading);
            if (elapsedMs(nowMs, goodSinceMs_) >= cfg.acquireStableMs) {
                acquireReferenceFromAccumulator(mag, heading, nowMs);
                consecutiveBad_ = 0;
                transition(MagFieldReliabilityState::Trusted, nowMs);
            }
        } else {
            goodSinceMs_ = 0;
            consecutiveBad_ = 0;
            resetAcquisitionAccumulator();
        }
    } else if (stationaryHeadingJumpLatched_) {
        // Do not let the ordinary suspect-clear path re-trust a stable local
        // disturbance. A stable return may be proven by the absolute world
        // reference, or by the pre-jump stationary field signal when no motion
        // occurred since latching. Explicit restartAcquisition() remains the
        // path for intentionally accepting a genuinely new environment.
        const float headingToReferenceDeg = std::fabs(wrapPi(
            heading.magneticNorthWorldYawRad - referenceHeadingYawRad_)) * MATH_RAD_TO_DEG;
        const float stationaryFieldReturnErrorDeg = std::fabs(wrapPi(
            stationaryFieldYawRad - stationaryLatchReferenceFieldYawRad_)) * MATH_RAD_TO_DEG;
        out.stationaryLatchReferenceFieldYawDeg =
            stationaryLatchReferenceFieldYawRad_ * MATH_RAD_TO_DEG;
        out.stationaryLatchReferenceErrorDeg = stationaryFieldReturnErrorDeg;
        out.stationaryLatchMotionSeen = stationaryLatchMotionSeen_;

        const bool returnedViaWorldHeading =
            headingToReferenceDeg <= cfg.referenceReturnDeg;
        // If the tracker has remained physically stationary since the latch,
        // compare the magnetic direction with AHRS yaw removed. This lets the
        // original field recover after arbitrary 6DoF yaw drift while yaw
        // correction is disabled, without accepting a stable shifted field.
        const bool returnedViaStationaryField = !stationaryLatchMotionSeen_ &&
            stationaryFieldReturnErrorDeg <= cfg.referenceReturnDeg;
        if (stationaryForHeadingJumpCheck &&
            (returnedViaWorldHeading || returnedViaStationaryField)) {
            newEnvironmentSinceMs_ = 0u;
            if (referenceReturnSinceMs_ == 0u) referenceReturnSinceMs_ = nowMs;
            if (elapsedMs(nowMs, referenceReturnSinceMs_) >= cfg.referenceReturnStableMs) {
                stationaryHeadingJumpLatched_ = false;
                stationaryLatchReferenceFieldYawRad_ = 0.0f;
                stationaryLatchMotionSeen_ = false;
                referenceReturnSinceMs_ = 0u;
                goodSinceMs_ = nowMs;
                consecutiveBad_ = 0u;
                stats_.stationaryHeadingReturns++;
                if (returnedViaWorldHeading) {
                    stats_.stationaryHeadingReturnsViaWorld++;
                } else {
                    stats_.stationaryHeadingReturnsViaStationaryField++;
                }
                transition(MagFieldReliabilityState::Recovering, nowMs);
            } else {
                goodSinceMs_ = 0u;
                transition(MagFieldReliabilityState::Disturbed, nowMs);
            }
        } else {
            referenceReturnSinceMs_ = 0u;
            if (stationaryForHeadingJumpCheck) {
                if (newEnvironmentSinceMs_ == 0u) newEnvironmentSinceMs_ = nowMs;
                if (!environmentChangedLatched_ &&
                    elapsedMs(nowMs, newEnvironmentSinceMs_) >= cfg.newEnvironmentStableMs) {
                    environmentChangedLatched_ = true;
                    stats_.environmentChangesDetected++;
                }
            } else {
                newEnvironmentSinceMs_ = 0u;
            }
            goodSinceMs_ = 0u;
            consecutiveBad_ = cfg.badSamplesToDisturbed;
            transition(MagFieldReliabilityState::Disturbed, nowMs);
        }
    } else if (hardBad) {
        goodSinceMs_ = 0;
        consecutiveBad_ = cfg.badSamplesToDisturbed;
        transition(MagFieldReliabilityState::Disturbed, nowMs);
    } else if (softBad) {
        goodSinceMs_ = 0;
        consecutiveBad_ = static_cast<uint8_t>(std::min<int>(255, consecutiveBad_ + 1));
        if (consecutiveBad_ >= cfg.badSamplesToDisturbed) {
            transition(MagFieldReliabilityState::Disturbed, nowMs);
        } else {
            transition(MagFieldReliabilityState::Suspect, nowMs);
        }
    } else {
        consecutiveBad_ = 0;
        if (goodSinceMs_ == 0u) goodSinceMs_ = nowMs;
        if (state_ == MagFieldReliabilityState::Disturbed) {
            if (!stationaryForHeadingCheck) {
                newEnvironmentSinceMs_ = 0;
                goodSinceMs_ = 0;
            } else {
                const float headingToReferenceDeg = std::fabs(wrapPi(
                    heading.magneticNorthWorldYawRad - referenceHeadingYawRad_)) * MATH_RAD_TO_DEG;
                if (headingObservable &&
                    magnitudeAtMostScaledThreshold(
                        headingToReferenceDeg,
                        cfg.headingStepSoftDeg,
                        out.headingNoiseScaleSquared)) {
                    transition(MagFieldReliabilityState::Recovering, nowMs);
                    goodSinceMs_ = nowMs;
                } else {
                    if (newEnvironmentSinceMs_ == 0u) newEnvironmentSinceMs_ = nowMs;
                    if (!environmentChangedLatched_ &&
                        elapsedMs(nowMs, newEnvironmentSinceMs_) >= cfg.newEnvironmentStableMs) {
                        environmentChangedLatched_ = true;
                        stats_.environmentChangesDetected++;
                    }
                    goodSinceMs_ = 0;
                }
            }
        } else if (state_ == MagFieldReliabilityState::Recovering) {
            if (elapsedMs(nowMs, goodSinceMs_) >= cfg.recoverStableMs) {
                transition(MagFieldReliabilityState::Trusted, nowMs);
            }
        } else if (state_ == MagFieldReliabilityState::Suspect) {
            if (elapsedMs(nowMs, goodSinceMs_) >= cfg.suspectClearMs) {
                transition(MagFieldReliabilityState::Trusted, nowMs);
            }
        } else if (state_ == MagFieldReliabilityState::Unavailable ||
                   state_ == MagFieldReliabilityState::Acquiring) {
            if (elapsedMs(nowMs, goodSinceMs_) >= cfg.acquireStableMs) {
                transition(MagFieldReliabilityState::Trusted, nowMs);
            }
        }

        if (state_ == MagFieldReliabilityState::Trusted) {
            const uint32_t dtMs = previousMs == 0u ? 0u : nowMs - previousMs;
            adaptReference(mag, heading, gyroNormDps, accelTrust, cfg, dtMs);
        }
    }

    out.state = state_;
    out.referenceValid = referenceValid_;
    out.stationaryHeadingJumpLatched = stationaryHeadingJumpLatched_;
    out.stationaryLatchReferenceFieldYawDeg =
        stationaryLatchReferenceFieldYawRad_ * MATH_RAD_TO_DEG;
    out.stationaryLatchMotionSeen = stationaryLatchMotionSeen_;
    if (inputFinite && stationaryHeadingJumpLatched_) {
        out.stationaryLatchReferenceErrorDeg = std::fabs(wrapPi(
            stationaryFieldYawRad - stationaryLatchReferenceFieldYawRad_)) * MATH_RAD_TO_DEG;
    }
    if (stationaryHeadingJumpLatched_) {
        out.flags |= MAG_FIELD_FLAG_STATIONARY_HEADING_JUMP;
    } else {
        out.flags &= ~MAG_FIELD_FLAG_STATIONARY_HEADING_JUMP;
    }
    if (referenceValid_) out.flags &= ~MAG_FIELD_FLAG_REFERENCE_MISSING;
    if (environmentChangedLatched_) out.flags |= MAG_FIELD_FLAG_ENVIRONMENT_CHANGED;
    out.referenceNorm = referenceNorm_;
    out.referenceDipDeg = referenceDipDeg_;
    out.referenceHeadingYawDeg = referenceHeadingYawRad_ * MATH_RAD_TO_DEG;
    if (inputFinite) {
        out.referenceHeadingErrorDeg = std::fabs(wrapPi(
            heading.magneticNorthWorldYawRad - referenceHeadingYawRad_)) * MATH_RAD_TO_DEG;
    }
    out.stableMs = elapsedMs(nowMs, goodSinceMs_);
    out.trustedForYaw = referenceValid_ &&
                        state_ == MagFieldReliabilityState::Trusted && baseGood &&
                        headingObservable;
    if (state_ == MagFieldReliabilityState::Recovering ||
        state_ == MagFieldReliabilityState::Disturbed) {
        out.flags |= MAG_FIELD_FLAG_RECOVERY_PENDING;
    }

    if (out.trustedForYaw) stats_.trustedSamples++;
    else stats_.rejectedSamples++;

    return out.valid;
}

const char* MagFieldReliabilityMonitor::stateName(MagFieldReliabilityState state) {
    switch (state) {
        case MagFieldReliabilityState::Unavailable: return "unavailable";
        case MagFieldReliabilityState::Acquiring: return "acquiring";
        case MagFieldReliabilityState::Trusted: return "trusted";
        case MagFieldReliabilityState::Suspect: return "suspect";
        case MagFieldReliabilityState::Disturbed: return "disturbed";
        case MagFieldReliabilityState::Recovering: return "recovering";
    }
    return "unknown";
}

} // namespace tracker
