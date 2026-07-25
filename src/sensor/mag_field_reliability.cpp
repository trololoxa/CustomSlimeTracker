#include "sensor/mag_field_reliability.hpp"

#include <algorithm>
#include <cmath>

namespace tracker {

namespace {

bool finiteInput(const MagFieldReliabilityInput& in) {
    return in.mag.valid && in.heading.valid &&
           tracker::isFinite(in.mag.bodyNorm) && in.mag.bodyNorm > MATH_EPSILON &&
           tracker::isFinite(in.heading.dipDeg) &&
           tracker::isFinite(in.heading.magneticNorthWorldYawRad) &&
           tracker::isFinite(in.gyroNormDps) && tracker::isFinite(in.accelTrust);
}

uint32_t elapsedMs(uint32_t nowMs, uint32_t sinceMs) {
    return sinceMs == 0u ? 0u : nowMs - sinceMs;
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
    referenceHeadingYawRad_ = 0.0f;
    stateSinceMs_ = 0;
    goodSinceMs_ = 0;
    consecutiveBad_ = 0;
    newEnvironmentSinceMs_ = 0;
    environmentChangedLatched_ = false;
    havePreviousHeading_ = false;
    previousHeadingYawRad_ = 0.0f;
    previousHeadingMs_ = 0;
    headingRateInitialized_ = false;
    filteredHeadingRateDegS_ = 0.0f;
    stationaryWindowActive_ = false;
    stationaryWindowStartMs_ = 0;
    stationaryWindowStartHeadingYawRad_ = 0.0f;
    stationaryHeadingJumpLatched_ = false;
    referenceReturnSinceMs_ = 0;
    resetAcquisitionAccumulator();
    last_ = MagFieldReliabilityOutput{};
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
    acquireHeadingSinSum_ = 0.0;
    acquireHeadingCosSum_ = 0.0;
    acquireSampleCount_ = 0;
}

void MagFieldReliabilityMonitor::accumulateReferenceSample(const MagFieldReliabilityInput& in) {
    acquireNormSum_ += static_cast<double>(in.mag.bodyNorm);
    acquireDipSum_ += static_cast<double>(in.heading.dipDeg);
    acquireHeadingSinSum_ += std::sin(static_cast<double>(in.heading.magneticNorthWorldYawRad));
    acquireHeadingCosSum_ += std::cos(static_cast<double>(in.heading.magneticNorthWorldYawRad));
    acquireSampleCount_++;
}

void MagFieldReliabilityMonitor::acquireReferenceFromAccumulator(
    const MagFieldReliabilityInput& fallback) {
    if (acquireSampleCount_ > 0u) {
        const double inv = 1.0 / static_cast<double>(acquireSampleCount_);
        referenceNorm_ = static_cast<float>(acquireNormSum_ * inv);
        referenceDipDeg_ = static_cast<float>(acquireDipSum_ * inv);
        referenceHeadingYawRad_ = static_cast<float>(
            std::atan2(acquireHeadingSinSum_, acquireHeadingCosSum_));
    } else {
        referenceNorm_ = fallback.mag.bodyNorm;
        referenceDipDeg_ = fallback.heading.dipDeg;
        referenceHeadingYawRad_ = fallback.heading.magneticNorthWorldYawRad;
    }
    referenceValid_ = true;
    stationaryWindowActive_ = true;
    stationaryWindowStartMs_ = fallback.nowMs;
    stationaryWindowStartHeadingYawRad_ = fallback.heading.magneticNorthWorldYawRad;
    resetAcquisitionAccumulator();
    stats_.referencesAcquired++;
}

void MagFieldReliabilityMonitor::adaptReference(const MagFieldReliabilityInput& in,
                                                const MagFieldReliabilityConfig& cfg,
                                                uint32_t dtMs) {
    if (!referenceValid_ || dtMs == 0u || cfg.referenceAdaptTimeConstantS <= 0.0f) return;
    if (in.gyroNormDps > cfg.stationaryGyroMaxDps ||
        in.accelTrust < cfg.stationaryAccelTrustMin) return;

    const float dtS = static_cast<float>(dtMs) * 0.001f;
    const float alpha = clampf(dtS / (cfg.referenceAdaptTimeConstantS + dtS), 0.0f, 0.02f);
    referenceNorm_ += alpha * (in.mag.bodyNorm - referenceNorm_);
    referenceDipDeg_ += alpha * (in.heading.dipDeg - referenceDipDeg_);
}

bool MagFieldReliabilityMonitor::update(const MagFieldReliabilityInput& in,
                                        const MagFieldReliabilityConfig& cfg,
                                        MagFieldReliabilityOutput& out) {
    out = MagFieldReliabilityOutput{};
    out.nowMs = in.nowMs;
    out.state = state_;
    stats_.updates++;

    const uint32_t previousMs = previousHeadingMs_;
    const bool inputFinite = finiteInput(in);
    if (!inputFinite) out.flags |= MAG_FIELD_FLAG_INPUT_INVALID;
    if (!in.processorTrustedForUse) out.flags |= MAG_FIELD_FLAG_PROCESSOR_UNTRUSTED;

    if (inputFinite) {
        out.valid = true;
        out.fieldNorm = in.mag.bodyNorm;
        out.dipDeg = in.heading.dipDeg;

        if (havePreviousHeading_) {
            const uint32_t dtMs = in.nowMs - previousHeadingMs_;
            const float signedStepDeg = wrapPi(
                in.heading.magneticNorthWorldYawRad - previousHeadingYawRad_) * MATH_RAD_TO_DEG;
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
        previousHeadingYawRad_ = in.heading.magneticNorthWorldYawRad;
        previousHeadingMs_ = in.nowMs;
        havePreviousHeading_ = true;
    }

    if (!referenceValid_) {
        out.flags |= MAG_FIELD_FLAG_REFERENCE_MISSING;
    } else if (inputFinite) {
        out.referenceNorm = referenceNorm_;
        out.referenceDipDeg = referenceDipDeg_;
        out.referenceHeadingYawDeg = referenceHeadingYawRad_ * MATH_RAD_TO_DEG;
        out.referenceHeadingErrorDeg = std::fabs(wrapPi(
            in.heading.magneticNorthWorldYawRad - referenceHeadingYawRad_)) * MATH_RAD_TO_DEG;
        out.normRelativeError = std::fabs(in.mag.bodyNorm - referenceNorm_) /
                                (std::fabs(referenceNorm_) + MATH_EPSILON);
        out.dipErrorDeg = std::fabs(in.heading.dipDeg - referenceDipDeg_);
        if (out.normRelativeError > cfg.normSoftFraction) out.flags |= MAG_FIELD_FLAG_NORM_SOFT;
        if (out.normRelativeError > cfg.normHardFraction) out.flags |= MAG_FIELD_FLAG_NORM_HARD;
        const bool dipObservable = in.gyroNormDps <= 120.0f && in.accelTrust >= 0.20f;
        if (dipObservable && out.dipErrorDeg > cfg.dipSoftDeg) out.flags |= MAG_FIELD_FLAG_DIP_SOFT;
        if (dipObservable && out.dipErrorDeg > cfg.dipHardDeg) out.flags |= MAG_FIELD_FLAG_DIP_HARD;
    }

    const bool stationaryForHeadingCheck = inputFinite &&
        in.gyroNormDps <= cfg.stationaryGyroMaxDps &&
        in.accelTrust >= cfg.stationaryAccelTrustMin;

    // A one-sample heading step is not enough by itself because the existing
    // soft gate can clear once the disturbed field becomes temporally stable.
    // Keep a short stationary-window anchor and latch a discontinuity when the
    // cumulative change is too fast to be ordinary calibrated gyro drift or
    // our own bounded yaw correction.  The latch is intentionally independent
    // of the old 8/20 degree per-sample thresholds.
    if (!stationaryForHeadingCheck || !referenceValid_) {
        stationaryWindowActive_ = false;
        stationaryWindowStartMs_ = 0u;
        referenceReturnSinceMs_ = 0u;
    } else {
        if (!stationaryWindowActive_) {
            stationaryWindowActive_ = true;
            stationaryWindowStartMs_ = in.nowMs;
            stationaryWindowStartHeadingYawRad_ = in.heading.magneticNorthWorldYawRad;
        } else {
            const uint32_t windowMs = in.nowMs - stationaryWindowStartMs_;
            const float signedWindowDeltaDeg = wrapPi(
                in.heading.magneticNorthWorldYawRad - stationaryWindowStartHeadingYawRad_) *
                MATH_RAD_TO_DEG;
            out.stationaryWindowHeadingDeltaDeg = std::fabs(signedWindowDeltaDeg);
            if (windowMs > 0u) {
                out.stationaryWindowHeadingRateDegS =
                    out.stationaryWindowHeadingDeltaDeg * 1000.0f /
                    static_cast<float>(windowMs);
            }

            const bool withinDetectionWindow = windowMs <= cfg.stationaryHeadingWindowMs;
            const float jumpRateThresholdDegS = std::max(
                cfg.stationaryHeadingJumpRateDegS,
                in.gyroNormDps + cfg.stationaryHeadingJumpRateMarginOverGyroDegS);
            const bool cumulativeDiscontinuity = withinDetectionWindow &&
                out.stationaryWindowHeadingDeltaDeg >= cfg.stationaryHeadingJumpMinDeg &&
                out.stationaryWindowHeadingRateDegS >= jumpRateThresholdDegS;
            // The rolling anchor is periodically refreshed.  Also test the
            // adjacent-sample step so an abrupt field jump cannot land exactly
            // on an anchor boundary and become the new undetected baseline.
            const bool instantaneousDiscontinuity = previousMs != 0u &&
                out.headingStepDeg >= cfg.stationaryHeadingJumpMinDeg &&
                out.headingInstantRateDegS >= jumpRateThresholdDegS;
            const bool discontinuity = cumulativeDiscontinuity || instantaneousDiscontinuity;
            if (discontinuity && !stationaryHeadingJumpLatched_) {
                stationaryHeadingJumpLatched_ = true;
                referenceReturnSinceMs_ = 0u;
                stats_.stationaryHeadingJumpsLatched++;
            }

            if (windowMs >= cfg.stationaryHeadingWindowMs) {
                stationaryWindowStartMs_ = in.nowMs;
                stationaryWindowStartHeadingYawRad_ = in.heading.magneticNorthWorldYawRad;
            }
        }
    }

    out.stationaryHeadingJumpLatched = stationaryHeadingJumpLatched_;
    if (stationaryHeadingJumpLatched_) {
        out.flags |= MAG_FIELD_FLAG_STATIONARY_HEADING_JUMP;
        stats_.stationaryHeadingJumpRejects++;
    }
    if (stationaryForHeadingCheck && previousMs != 0u) {
        if (out.headingStepDeg > cfg.headingStepSoftDeg) out.flags |= MAG_FIELD_FLAG_HEADING_STEP_SOFT;
        if (out.headingStepDeg > cfg.headingStepHardDeg) out.flags |= MAG_FIELD_FLAG_HEADING_STEP_HARD;
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
    const bool baseGood = inputFinite && in.processorTrustedForUse && !hardBad && !softBad;

    if (!inputFinite || !in.processorTrustedForUse) {
        goodSinceMs_ = 0;
        resetAcquisitionAccumulator();
        consecutiveBad_ = static_cast<uint8_t>(std::min<int>(255, consecutiveBad_ + 1));
        transition(MagFieldReliabilityState::Unavailable, in.nowMs);
    } else if (!referenceValid_) {
        const bool acquisitionGood = baseGood && stationaryForHeadingCheck;
        transition(MagFieldReliabilityState::Acquiring, in.nowMs);
        if (acquisitionGood) {
            if (goodSinceMs_ == 0u) {
                goodSinceMs_ = in.nowMs;
                resetAcquisitionAccumulator();
            }
            accumulateReferenceSample(in);
            if (elapsedMs(in.nowMs, goodSinceMs_) >= cfg.acquireStableMs) {
                acquireReferenceFromAccumulator(in);
                consecutiveBad_ = 0;
                transition(MagFieldReliabilityState::Trusted, in.nowMs);
            }
        } else {
            goodSinceMs_ = 0;
            consecutiveBad_ = 0;
            resetAcquisitionAccumulator();
        }
    } else if (stationaryHeadingJumpLatched_) {
        // Do not let the ordinary suspect-clear path re-trust a stable local
        // disturbance.  Only a stable return near the established heading
        // reference can clear this latch; explicit restartAcquisition() also
        // clears it for an intentional environment change.
        const float headingToReferenceDeg = std::fabs(wrapPi(
            in.heading.magneticNorthWorldYawRad - referenceHeadingYawRad_)) * MATH_RAD_TO_DEG;
        if (stationaryForHeadingCheck && headingToReferenceDeg <= cfg.referenceReturnDeg) {
            newEnvironmentSinceMs_ = 0u;
            if (referenceReturnSinceMs_ == 0u) referenceReturnSinceMs_ = in.nowMs;
            if (elapsedMs(in.nowMs, referenceReturnSinceMs_) >= cfg.referenceReturnStableMs) {
                stationaryHeadingJumpLatched_ = false;
                referenceReturnSinceMs_ = 0u;
                goodSinceMs_ = in.nowMs;
                consecutiveBad_ = 0u;
                stats_.stationaryHeadingReturns++;
                transition(MagFieldReliabilityState::Recovering, in.nowMs);
            } else {
                goodSinceMs_ = 0u;
                transition(MagFieldReliabilityState::Disturbed, in.nowMs);
            }
        } else {
            referenceReturnSinceMs_ = 0u;
            if (stationaryForHeadingCheck) {
                if (newEnvironmentSinceMs_ == 0u) newEnvironmentSinceMs_ = in.nowMs;
                if (!environmentChangedLatched_ &&
                    elapsedMs(in.nowMs, newEnvironmentSinceMs_) >= cfg.newEnvironmentStableMs) {
                    environmentChangedLatched_ = true;
                    stats_.environmentChangesDetected++;
                }
            } else {
                newEnvironmentSinceMs_ = 0u;
            }
            goodSinceMs_ = 0u;
            consecutiveBad_ = cfg.badSamplesToDisturbed;
            transition(MagFieldReliabilityState::Disturbed, in.nowMs);
        }
    } else if (hardBad) {
        goodSinceMs_ = 0;
        consecutiveBad_ = cfg.badSamplesToDisturbed;
        transition(MagFieldReliabilityState::Disturbed, in.nowMs);
    } else if (softBad) {
        goodSinceMs_ = 0;
        consecutiveBad_ = static_cast<uint8_t>(std::min<int>(255, consecutiveBad_ + 1));
        if (consecutiveBad_ >= cfg.badSamplesToDisturbed) {
            transition(MagFieldReliabilityState::Disturbed, in.nowMs);
        } else {
            transition(MagFieldReliabilityState::Suspect, in.nowMs);
        }
    } else {
        consecutiveBad_ = 0;
        if (goodSinceMs_ == 0u) goodSinceMs_ = in.nowMs;
        if (state_ == MagFieldReliabilityState::Disturbed) {
            if (!stationaryForHeadingCheck) {
                newEnvironmentSinceMs_ = 0;
                goodSinceMs_ = 0;
            } else {
                const float headingToReferenceDeg = std::fabs(wrapPi(
                    in.heading.magneticNorthWorldYawRad - referenceHeadingYawRad_)) * MATH_RAD_TO_DEG;
                if (headingToReferenceDeg <= cfg.headingStepSoftDeg) {
                    transition(MagFieldReliabilityState::Recovering, in.nowMs);
                    goodSinceMs_ = in.nowMs;
                } else {
                    if (newEnvironmentSinceMs_ == 0u) newEnvironmentSinceMs_ = in.nowMs;
                    if (!environmentChangedLatched_ &&
                        elapsedMs(in.nowMs, newEnvironmentSinceMs_) >= cfg.newEnvironmentStableMs) {
                        environmentChangedLatched_ = true;
                        stats_.environmentChangesDetected++;
                    }
                    goodSinceMs_ = 0;
                }
            }
        } else if (state_ == MagFieldReliabilityState::Recovering) {
            if (elapsedMs(in.nowMs, goodSinceMs_) >= cfg.recoverStableMs) {
                transition(MagFieldReliabilityState::Trusted, in.nowMs);
            }
        } else if (state_ == MagFieldReliabilityState::Suspect) {
            if (elapsedMs(in.nowMs, goodSinceMs_) >= cfg.suspectClearMs) {
                transition(MagFieldReliabilityState::Trusted, in.nowMs);
            }
        } else if (state_ == MagFieldReliabilityState::Unavailable ||
                   state_ == MagFieldReliabilityState::Acquiring) {
            if (elapsedMs(in.nowMs, goodSinceMs_) >= cfg.acquireStableMs) {
                transition(MagFieldReliabilityState::Trusted, in.nowMs);
            }
        }

        if (state_ == MagFieldReliabilityState::Trusted) {
            const uint32_t dtMs = previousMs == 0u ? 0u : in.nowMs - previousMs;
            adaptReference(in, cfg, dtMs);
        }
    }

    out.state = state_;
    out.referenceValid = referenceValid_;
    out.stationaryHeadingJumpLatched = stationaryHeadingJumpLatched_;
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
            in.heading.magneticNorthWorldYawRad - referenceHeadingYawRad_)) * MATH_RAD_TO_DEG;
    }
    out.stableMs = elapsedMs(in.nowMs, goodSinceMs_);
    out.trustedForYaw = referenceValid_ &&
                        state_ == MagFieldReliabilityState::Trusted && baseGood;
    if (state_ == MagFieldReliabilityState::Recovering ||
        state_ == MagFieldReliabilityState::Disturbed) {
        out.flags |= MAG_FIELD_FLAG_RECOVERY_PENDING;
    }

    if (out.trustedForYaw) stats_.trustedSamples++;
    else stats_.rejectedSamples++;

    last_ = out;
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
