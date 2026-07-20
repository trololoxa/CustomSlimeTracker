#pragma once

#include <cmath>
#include <cstdint>

#include "core/math.hpp"

namespace tracker {

// Small stats used by command-driven static tests and runtime bias diagnostics.
// Keep these POD-style helpers outside main.cpp so diagnostics can grow without
// making the firmware entry point harder to review.
struct ScalarStats {
    uint32_t count = 0;
    double sum = 0.0;
    double sumSq = 0.0;
    float minValue = 0.0f;
    float maxValue = 0.0f;

    void reset() {
        count = 0;
        sum = 0.0;
        sumSq = 0.0;
        minValue = 0.0f;
        maxValue = 0.0f;
    }

    void push(float x) {
        if (!std::isfinite(x)) return;
        if (count == 0) {
            minValue = x;
            maxValue = x;
        } else {
            if (x < minValue) minValue = x;
            if (x > maxValue) maxValue = x;
        }
        count++;
        const double xd = static_cast<double>(x);
        sum += xd;
        sumSq += xd * xd;
    }

    float mean() const {
        return count == 0 ? 0.0f : static_cast<float>(sum / static_cast<double>(count));
    }

    float stddev() const {
        if (count < 2) return 0.0f;
        const double n = static_cast<double>(count);
        const double m = sum / n;
        const double v = (sumSq / n) - (m * m);
        return static_cast<float>(v > 0.0 ? std::sqrt(v) : 0.0);
    }

    void merge(const ScalarStats& other) {
        if (other.count == 0) return;
        if (count == 0) {
            minValue = other.minValue;
            maxValue = other.maxValue;
        } else {
            if (other.minValue < minValue) minValue = other.minValue;
            if (other.maxValue > maxValue) maxValue = other.maxValue;
        }
        count += other.count;
        sum += other.sum;
        sumSq += other.sumSq;
    }
};

struct Vec3Stats {
    uint32_t count = 0;
    Vec3 sum = Vec3::zero();
    Vec3 sumSq = Vec3::zero();
    Vec3 minValue = Vec3::zero();
    Vec3 maxValue = Vec3::zero();

    void reset() {
        count = 0;
        sum = Vec3::zero();
        sumSq = Vec3::zero();
        minValue = Vec3::zero();
        maxValue = Vec3::zero();
    }

    void push(const Vec3& v) {
        if (!v.isFinite()) return;

        if (count == 0) {
            minValue = v;
            maxValue = v;
        } else {
            if (v.x < minValue.x) minValue.x = v.x;
            if (v.y < minValue.y) minValue.y = v.y;
            if (v.z < minValue.z) minValue.z = v.z;

            if (v.x > maxValue.x) maxValue.x = v.x;
            if (v.y > maxValue.y) maxValue.y = v.y;
            if (v.z > maxValue.z) maxValue.z = v.z;
        }

        count++;
        sum += v;
        sumSq += hadamard(v, v);
    }

    Vec3 mean() const {
        return count == 0 ? Vec3::zero() : sum / static_cast<float>(count);
    }

    Vec3 stddev() const {
        if (count < 2) return Vec3::zero();

        const Vec3 m = mean();
        const Vec3 meanSq = sumSq / static_cast<float>(count);
        Vec3 v = meanSq - hadamard(m, m);

        if (v.x < 0.0f) v.x = 0.0f;
        if (v.y < 0.0f) v.y = 0.0f;
        if (v.z < 0.0f) v.z = 0.0f;

        return Vec3(std::sqrt(v.x), std::sqrt(v.y), std::sqrt(v.z));
    }

    void merge(const Vec3Stats& other) {
        if (other.count == 0) return;
        if (count == 0) {
            minValue = other.minValue;
            maxValue = other.maxValue;
        } else {
            if (other.minValue.x < minValue.x) minValue.x = other.minValue.x;
            if (other.minValue.y < minValue.y) minValue.y = other.minValue.y;
            if (other.minValue.z < minValue.z) minValue.z = other.minValue.z;
            if (other.maxValue.x > maxValue.x) maxValue.x = other.maxValue.x;
            if (other.maxValue.y > maxValue.y) maxValue.y = other.maxValue.y;
            if (other.maxValue.z > maxValue.z) maxValue.z = other.maxValue.z;
        }
        count += other.count;
        sum += other.sum;
        sumSq += other.sumSq;
    }
};

static constexpr uint8_t STATIC_TEMP_BIN_COUNT = 48;
static constexpr float STATIC_TEMP_BIN_MIN_C = 10.0f;
static constexpr float STATIC_TEMP_BIN_WIDTH_C = 1.0f;

struct StaticTempBinStats {
    ScalarStats tempC;
    ScalarStats accelNormG;
    Vec3Stats gyroAfterRadS;
    uint32_t badQualitySamples = 0;

    void reset() {
        tempC.reset();
        accelNormG.reset();
        gyroAfterRadS.reset();
        badQualitySamples = 0;
    }

    void push(float temp, const Vec3& gyroAfter, float accelNorm, bool goodQuality) {
        if (!goodQuality) {
            badQualitySamples++;
            return;
        }
        tempC.push(temp);
        accelNormG.push(accelNorm);
        gyroAfterRadS.push(gyroAfter);
    }
};

inline int staticTempBinIndex(float tempC) {
    if (!std::isfinite(tempC)) return -1;
    const int idx = static_cast<int>(std::floor((tempC - STATIC_TEMP_BIN_MIN_C) / STATIC_TEMP_BIN_WIDTH_C));
    if (idx < 0 || idx >= static_cast<int>(STATIC_TEMP_BIN_COUNT)) return -1;
    return idx;
}

struct StaticRuntimeTest {
    bool active = false;
    bool stopRequested = false;
    uint32_t durationMs = 0;
    uint32_t startMs = 0;
    uint32_t lastProgressMs = 0;

    float tempStartC = 0.0f;
    float tempEndC = 0.0f;
    bool tempCaptured = false;

    uint32_t samples = 0;
    uint32_t hwTs = 0;
    uint32_t fallbackTs = 0;
    uint32_t badTs = 0;
    uint32_t droppedEstimate = 0;
    uint32_t recoveryRequests = 0;
    uint32_t ahrsSkipped = 0;
    uint32_t accelDisabled = 0;

    uint32_t fifoOverrunAtStart = 0;
    uint32_t fifoFullAtStart = 0;
    uint32_t fifoUnknownAtStart = 0;
    uint32_t fifoHwTsAtStart = 0;
    uint32_t fifoFbTsAtStart = 0;

    // Perf counter snapshots. These make `test static` useful as a before/after
    // optimization benchmark without changing AHRS/FIFO behavior.
    uint32_t perfFifoProcessCallsAtStart = 0;
    uint64_t perfFifoProcessSumUsAtStart = 0;
    uint32_t perfSampleProcessCallsAtStart = 0;
    uint64_t perfSampleProcessSumUsAtStart = 0;
    uint32_t perfFifoEmptyPollsAtStart = 0;
    uint32_t perfFifoIrqEventsAtStart = 0;
    uint32_t perfFifoFallbackStatusPollsAtStart = 0;
    uint32_t perfFifoFallbackEventsAtStart = 0;

    // Magnetometer / yaw correction snapshots.
    bool magEnabledAtStart = false;
    bool magYawApplyEnabledAtStart = false;
    bool magRefValidAtStart = false;

    float magErrorStartDeg = 0.0f;
    float magErrorEndDeg = 0.0f;
    float magErrorAbsMaxDeg = 0.0f;
    double magErrorAbsSumDeg = 0.0;
    uint32_t magErrorSamples = 0;

    uint32_t magTrustedAtStart = 0;
    uint32_t magRejectedAtStart = 0;

    uint32_t magHeadingValidAtStart = 0;
    uint32_t magHeadingRejectedAtStart = 0;

    uint32_t magYawUpdatesAtStart = 0;
    uint32_t magYawGateOpenAtStart = 0;
    uint32_t magYawGateClosedAtStart = 0;
    uint32_t magYawApplyAllowedAtStart = 0;
    uint32_t magYawAppliedAtStart = 0;

    uint32_t magYawRejectNoReferenceAtStart = 0;
    uint32_t magYawRejectHeadingInvalidAtStart = 0;
    uint32_t magYawRejectMagNotTrustedAtStart = 0;
    uint32_t magYawRejectMagStaleAtStart = 0;
    uint32_t magYawRejectHorizontalBadAtStart = 0;
    uint32_t magYawRejectInnovationTooLargeAtStart = 0;
    uint32_t magYawRejectGyroMovingAtStart = 0;
    uint32_t magYawRejectAccelNotTrustedAtStart = 0;

    ScalarStats dtUs;
    ScalarStats accelNormG;
    ScalarStats accelTrust;
    ScalarStats tempC;
    Vec3Stats gyroAfterRadS;
    StaticTempBinStats tempBins[STATIC_TEMP_BIN_COUNT];
    uint32_t tempBinOutOfRangeSamples = 0;

    // Static-test instrumentation runs in the hot FIFO path. Keep counters here
    // so we can prove whether the test itself is starving FIFO service.
    uint32_t updateDecimator = 0;
    uint32_t poseSamples = 0;
    uint32_t slowUpdateCount = 0;
    uint32_t maxUpdateUs = 0;
    uint32_t maxSampleProcessUs = 0;
    uint32_t maxFifoProcessUs = 0;

    ScalarStats magHeadingHorizontalNorm;
    ScalarStats magYawCombinedTrust;
    ScalarStats magYawCorrectionRateDegS;
    ScalarStats magYawCorrectionStepDeg;

    bool poseCaptured = false;
    Vec3 eulerStartDeg = Vec3::zero();
    Vec3 eulerEndDeg = Vec3::zero();
    Quat qStart = Quat::identity();
    Quat qEnd = Quat::identity();

    void reset() {
        active = false;
        stopRequested = false;
        durationMs = 0;
        startMs = 0;
        lastProgressMs = 0;
        tempStartC = 0.0f;
        tempEndC = 0.0f;
        tempCaptured = false;
        samples = 0;
        hwTs = 0;
        fallbackTs = 0;
        badTs = 0;
        droppedEstimate = 0;
        recoveryRequests = 0;
        ahrsSkipped = 0;
        accelDisabled = 0;
        fifoOverrunAtStart = 0;
        fifoFullAtStart = 0;
        fifoUnknownAtStart = 0;
        fifoHwTsAtStart = 0;
        fifoFbTsAtStart = 0;

        perfFifoProcessCallsAtStart = 0;
        perfFifoProcessSumUsAtStart = 0;
        perfSampleProcessCallsAtStart = 0;
        perfSampleProcessSumUsAtStart = 0;
        perfFifoEmptyPollsAtStart = 0;
        perfFifoIrqEventsAtStart = 0;
        perfFifoFallbackStatusPollsAtStart = 0;
        perfFifoFallbackEventsAtStart = 0;

        magEnabledAtStart = false;
        magYawApplyEnabledAtStart = false;
        magRefValidAtStart = false;

        magErrorStartDeg = 0.0f;
        magErrorEndDeg = 0.0f;
        magErrorAbsMaxDeg = 0.0f;
        magErrorAbsSumDeg = 0.0;
        magErrorSamples = 0;

        magTrustedAtStart = 0;
        magRejectedAtStart = 0;

        magHeadingValidAtStart = 0;
        magHeadingRejectedAtStart = 0;

        magYawUpdatesAtStart = 0;
        magYawGateOpenAtStart = 0;
        magYawGateClosedAtStart = 0;
        magYawApplyAllowedAtStart = 0;
        magYawAppliedAtStart = 0;

        magYawRejectNoReferenceAtStart = 0;
        magYawRejectHeadingInvalidAtStart = 0;
        magYawRejectMagNotTrustedAtStart = 0;
        magYawRejectMagStaleAtStart = 0;
        magYawRejectHorizontalBadAtStart = 0;
        magYawRejectInnovationTooLargeAtStart = 0;
        magYawRejectGyroMovingAtStart = 0;
        magYawRejectAccelNotTrustedAtStart = 0;

        dtUs.reset();
        accelNormG.reset();
        accelTrust.reset();
        tempC.reset();
        gyroAfterRadS.reset();
        for (uint8_t i = 0; i < STATIC_TEMP_BIN_COUNT; ++i) tempBins[i].reset();
        tempBinOutOfRangeSamples = 0;
        updateDecimator = 0;
        poseSamples = 0;
        slowUpdateCount = 0;
        maxUpdateUs = 0;
        maxSampleProcessUs = 0;
        maxFifoProcessUs = 0;
        magHeadingHorizontalNorm.reset();
        magYawCombinedTrust.reset();
        magYawCorrectionRateDegS.reset();
        magYawCorrectionStepDeg.reset();
        poseCaptured = false;
        eulerStartDeg = Vec3::zero();
        eulerEndDeg = Vec3::zero();
        qStart = Quat::identity();
        qEnd = Quat::identity();
    }
};

} // namespace tracker
