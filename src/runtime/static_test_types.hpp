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

    // Exact network counter snapshots for the measured window. Periodic NET
    // records describe chronology; these deltas remain authoritative even if
    // a boundary record is delayed by serializer backpressure.
    bool networkMetricsValid = false;
    uint32_t wifiDisconnectsAtStart = 0;
    uint32_t wifiConnectTimeoutsAtStart = 0;
    uint32_t udpSendFailuresAtStart = 0;
    uint32_t rotationSendFailuresAtStart = 0;
    uint32_t rotationMissedDeadlinesAtStart = 0;
    uint32_t rotationLateEventsAtStart = 0;
    uint32_t txPressureFailuresAtStart = 0;
    uint32_t txOtherFailuresAtStart = 0;
    uint32_t udpRebindSuccessesAtStart = 0;
    uint32_t udpRebindFailuresAtStart = 0;
    uint32_t udpFullReopensAtStart = 0;

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
    uint32_t updateTimingSamples = 0;
    uint32_t slowUpdateCount = 0;
    uint32_t maxUpdateUs = 0;
    uint32_t maxSampleProcessUs = 0;
    uint32_t maxFifoProcessUs = 0;

    // Immutable end-of-window snapshots used by the explicitly requested
    // detailed report. Capturing these before the test is released prevents a
    // later `test report static` from accidentally including post-test faults.
    uint32_t finishedElapsedMs = 0;
    bool stoppedByCommand = false;
    uint32_t perfSampleCallsDelta = 0;
    uint64_t perfSampleSumUsDelta = 0;
    uint32_t perfFifoCallsDelta = 0;
    uint64_t perfFifoSumUsDelta = 0;
    uint32_t perfEmptyPollsDelta = 0;
    uint32_t perfIrqEventsDelta = 0;
    uint32_t perfFallbackPollsDelta = 0;
    uint32_t perfFallbackEventsDelta = 0;
    uint32_t fifoOverrunDelta = 0;
    uint32_t fifoFullDelta = 0;
    uint32_t fifoUnknownDelta = 0;
    uint32_t fifoHwTsDelta = 0;
    uint32_t fifoFbTsDelta = 0;
    uint32_t wifiDisconnectsDelta = 0;
    uint32_t wifiConnectTimeoutsDelta = 0;
    uint32_t udpSendFailuresDelta = 0;
    uint32_t rotationSendFailuresDelta = 0;
    uint32_t rotationMissedDeadlinesDelta = 0;
    uint32_t rotationLateEventsDelta = 0;
    uint32_t txPressureFailuresDelta = 0;
    uint32_t txOtherFailuresDelta = 0;
    uint32_t udpRebindSuccessesDelta = 0;
    uint32_t udpRebindFailuresDelta = 0;
    uint32_t udpFullReopensDelta = 0;
    bool magRefValidEnd = false;
    uint32_t magTrustedDelta = 0;
    uint32_t magRejectedDelta = 0;
    uint32_t magHeadingValidDelta = 0;
    uint32_t magHeadingRejectedDelta = 0;
    uint32_t magYawUpdatesDelta = 0;
    uint32_t magYawGateOpenDelta = 0;
    uint32_t magYawGateClosedDelta = 0;
    uint32_t magYawApplyAllowedDelta = 0;
    uint32_t magYawAppliedDelta = 0;
    uint32_t magYawRejectNoReferenceDelta = 0;
    uint32_t magYawRejectHeadingInvalidDelta = 0;
    uint32_t magYawRejectMagNotTrustedDelta = 0;
    uint32_t magYawRejectMagStaleDelta = 0;
    uint32_t magYawRejectHorizontalBadDelta = 0;
    uint32_t magYawRejectInnovationTooLargeDelta = 0;
    uint32_t magYawRejectGyroMovingDelta = 0;
    uint32_t magYawRejectAccelNotTrustedDelta = 0;
    float lastMagYawErrorDeg = 0.0f;
    float lastMagYawCorrectionRateDegS = 0.0f;
    float lastMagYawCorrectionStepDeg = 0.0f;

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
        networkMetricsValid = false;
        wifiDisconnectsAtStart = 0;
        wifiConnectTimeoutsAtStart = 0;
        udpSendFailuresAtStart = 0;
        rotationSendFailuresAtStart = 0;
        rotationMissedDeadlinesAtStart = 0;
        rotationLateEventsAtStart = 0;
        txPressureFailuresAtStart = 0;
        txOtherFailuresAtStart = 0;
        udpRebindSuccessesAtStart = 0;
        udpRebindFailuresAtStart = 0;
        udpFullReopensAtStart = 0;

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
        updateTimingSamples = 0;
        slowUpdateCount = 0;
        maxUpdateUs = 0;
        maxSampleProcessUs = 0;
        maxFifoProcessUs = 0;
        finishedElapsedMs = 0;
        stoppedByCommand = false;
        perfSampleCallsDelta = 0;
        perfSampleSumUsDelta = 0;
        perfFifoCallsDelta = 0;
        perfFifoSumUsDelta = 0;
        perfEmptyPollsDelta = 0;
        perfIrqEventsDelta = 0;
        perfFallbackPollsDelta = 0;
        perfFallbackEventsDelta = 0;
        fifoOverrunDelta = 0;
        fifoFullDelta = 0;
        fifoUnknownDelta = 0;
        fifoHwTsDelta = 0;
        fifoFbTsDelta = 0;
        wifiDisconnectsDelta = 0;
        wifiConnectTimeoutsDelta = 0;
        udpSendFailuresDelta = 0;
        rotationSendFailuresDelta = 0;
        rotationMissedDeadlinesDelta = 0;
        rotationLateEventsDelta = 0;
        txPressureFailuresDelta = 0;
        txOtherFailuresDelta = 0;
        udpRebindSuccessesDelta = 0;
        udpRebindFailuresDelta = 0;
        udpFullReopensDelta = 0;
        magRefValidEnd = false;
        magTrustedDelta = 0;
        magRejectedDelta = 0;
        magHeadingValidDelta = 0;
        magHeadingRejectedDelta = 0;
        magYawUpdatesDelta = 0;
        magYawGateOpenDelta = 0;
        magYawGateClosedDelta = 0;
        magYawApplyAllowedDelta = 0;
        magYawAppliedDelta = 0;
        magYawRejectNoReferenceDelta = 0;
        magYawRejectHeadingInvalidDelta = 0;
        magYawRejectMagNotTrustedDelta = 0;
        magYawRejectMagStaleDelta = 0;
        magYawRejectHorizontalBadDelta = 0;
        magYawRejectInnovationTooLargeDelta = 0;
        magYawRejectGyroMovingDelta = 0;
        magYawRejectAccelNotTrustedDelta = 0;
        lastMagYawErrorDeg = 0.0f;
        lastMagYawCorrectionRateDegS = 0.0f;
        lastMagYawCorrectionStepDeg = 0.0f;
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

// Float blocks keep per-sample work on the ESP32-C3 inexpensive. Sums are
// centred on the first value in each block: this avoids the catastrophic
// precision loss of summing values such as dt_us^2 in float, while reserving
// double-precision reconstruction for the deferred block merge.
struct StaticScalarStatsBlock {
    uint32_t count = 0;
    float origin = 0.0f;
    float centeredSum = 0.0f;
    float centeredSumSq = 0.0f;
    float minValue = 0.0f;
    float maxValue = 0.0f;

    void reset() { *this = StaticScalarStatsBlock{}; }

    void push(float value) {
        if (!std::isfinite(value)) return;
        if (count == 0u) {
            origin = value;
            minValue = value;
            maxValue = value;
        } else {
            if (value < minValue) minValue = value;
            if (value > maxValue) maxValue = value;
        }
        ++count;
        const float centered = value - origin;
        centeredSum += centered;
        centeredSumSq += centered * centered;
    }

    void mergeInto(ScalarStats& target) const {
        if (count == 0u) return;
        ScalarStats block;
        block.count = count;
        const double n = static_cast<double>(count);
        const double o = static_cast<double>(origin);
        const double s = static_cast<double>(centeredSum);
        block.sum = n * o + s;
        block.sumSq = n * o * o + 2.0 * o * s + static_cast<double>(centeredSumSq);
        block.minValue = minValue;
        block.maxValue = maxValue;
        target.merge(block);
    }
};

struct StaticVec3StatsBlock {
    uint32_t count = 0;
    Vec3 origin = Vec3::zero();
    Vec3 centeredSum = Vec3::zero();
    Vec3 centeredSumSq = Vec3::zero();
    Vec3 minValue = Vec3::zero();
    Vec3 maxValue = Vec3::zero();

    void reset() { *this = StaticVec3StatsBlock{}; }

    void push(const Vec3& value) {
        if (!value.isFinite()) return;
        if (count == 0u) {
            origin = value;
            minValue = value;
            maxValue = value;
        } else {
            if (value.x < minValue.x) minValue.x = value.x;
            if (value.y < minValue.y) minValue.y = value.y;
            if (value.z < minValue.z) minValue.z = value.z;
            if (value.x > maxValue.x) maxValue.x = value.x;
            if (value.y > maxValue.y) maxValue.y = value.y;
            if (value.z > maxValue.z) maxValue.z = value.z;
        }
        ++count;
        const Vec3 centered = value - origin;
        centeredSum += centered;
        centeredSumSq += hadamard(centered, centered);
    }

    void mergeInto(Vec3Stats& target) const {
        if (count == 0u) return;
        Vec3Stats block;
        block.count = count;
        const float n = static_cast<float>(count);
        block.sum = origin * n + centeredSum;
        block.sumSq = hadamard(origin, origin) * n +
            hadamard(origin, centeredSum) * 2.0f + centeredSumSq;
        block.minValue = minValue;
        block.maxValue = maxValue;
        target.merge(block);
    }
};

struct StaticTempBinStatsBlock {
    StaticScalarStatsBlock tempC;
    StaticScalarStatsBlock accelNormG;
    StaticVec3StatsBlock gyroAfterRadS;
    uint32_t badQualitySamples = 0;

    void reset() { *this = StaticTempBinStatsBlock{}; }

    void push(float temp, const Vec3& gyroAfter, float accelNorm, bool goodQuality) {
        if (!goodQuality) {
            ++badQualitySamples;
            return;
        }
        tempC.push(temp);
        accelNormG.push(accelNorm);
        gyroAfterRadS.push(gyroAfter);
    }

    void mergeInto(StaticTempBinStats& target) const {
        tempC.mergeInto(target.tempC);
        accelNormG.mergeInto(target.accelNormG);
        gyroAfterRadS.mergeInto(target.gyroAfterRadS);
        target.badQualitySamples += badQualitySamples;
    }
};

struct StaticTestStatsBlock {
    static constexpr uint8_t TEMP_BIN_SLOTS = 4u;

    uint32_t bufferedSamples = 0;
    StaticScalarStatsBlock dtUs;
    StaticScalarStatsBlock accelNormG;
    StaticScalarStatsBlock accelTrust;
    StaticScalarStatsBlock tempC;
    StaticVec3StatsBlock gyroAfterRadS;
    int8_t tempBinIndices[TEMP_BIN_SLOTS] = {-1, -1, -1, -1};
    StaticTempBinStatsBlock tempBins[TEMP_BIN_SLOTS];

    void reset() {
        *this = StaticTestStatsBlock{};
        for (uint8_t i = 0u; i < TEMP_BIN_SLOTS; ++i) tempBinIndices[i] = -1;
    }

    bool push(uint32_t sampleDtUs,
              float sampleAccelConfidence,
              float sampleTempC,
              const Vec3& sampleGyroRadS,
              float accelNorm,
              int tempBinIdx,
              bool goodForTempFit) {
        int8_t tempSlot = -1;
        if (tempBinIdx >= 0 && tempBinIdx < static_cast<int>(STATIC_TEMP_BIN_COUNT)) {
            for (uint8_t i = 0u; i < TEMP_BIN_SLOTS; ++i) {
                if (tempBinIndices[i] == tempBinIdx) {
                    tempSlot = static_cast<int8_t>(i);
                    break;
                }
                if (tempSlot < 0 && tempBinIndices[i] < 0) {
                    tempSlot = static_cast<int8_t>(i);
                }
            }
            if (tempSlot < 0) return false;
            tempBinIndices[static_cast<uint8_t>(tempSlot)] = static_cast<int8_t>(tempBinIdx);
        }
        if (sampleDtUs > 0u) dtUs.push(static_cast<float>(sampleDtUs));
        accelNormG.push(accelNorm);
        accelTrust.push(sampleAccelConfidence);
        tempC.push(sampleTempC);
        gyroAfterRadS.push(sampleGyroRadS);
        if (tempSlot >= 0) {
            tempBins[static_cast<uint8_t>(tempSlot)].push(sampleTempC,
                                                          sampleGyroRadS,
                                                          accelNorm,
                                                          goodForTempFit);
        }
        ++bufferedSamples;
        return true;
    }

    void mergeInto(StaticRuntimeTest& target) const {
        dtUs.mergeInto(target.dtUs);
        accelNormG.mergeInto(target.accelNormG);
        accelTrust.mergeInto(target.accelTrust);
        tempC.mergeInto(target.tempC);
        gyroAfterRadS.mergeInto(target.gyroAfterRadS);
        for (uint8_t i = 0u; i < TEMP_BIN_SLOTS; ++i) {
            const int8_t bin = tempBinIndices[i];
            if (bin >= 0) tempBins[i].mergeInto(target.tempBins[static_cast<uint8_t>(bin)]);
        }
    }
};

static_assert(sizeof(StaticTestStatsBlock) <= 768u,
              "Blocked static-test aggregation exceeded its audited RAM budget");

} // namespace tracker
