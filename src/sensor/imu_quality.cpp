#include "sensor/imu_quality.hpp"

#include <cmath>

namespace tracker {

bool ImuQualityResult::has(uint32_t f) const {
    return (flags & f) != 0;
}

void ImuQualityResult::markTempCompOutOfRange(float confidenceMultiplier) {
    flags |= imu_quality_flags::TEMP_COMP_OUT_OF_RANGE;
    if (!std::isfinite(confidenceMultiplier) || confidenceMultiplier < 0.0f) {
        confidenceMultiplier = 0.0f;
    }
    if (confidenceMultiplier > 1.0f) {
        confidenceMultiplier = 1.0f;
    }
    gyroConfidence *= confidenceMultiplier;
    overallConfidence *= confidenceMultiplier;
}

Vec3 ImuQualityResult::accelForAhrs(const Vec3& calibratedAccelG) const {
    return shouldUseAccelCorrection ? calibratedAccelG : Vec3::zero();
}

float ImuQualityCounters::meanDtUs() const {
    const uint32_t validDtCount = samples - zeroTimestampSamples - nonMonotonicTimestampSamples;
    return validDtCount == 0 ? 0.0f : static_cast<float>(sumDtUs / static_cast<double>(validDtCount));
}

ImuQualityMonitor::ImuQualityMonitor(const ImuQualityConfig& cfg)
    : cfg_(cfg) {}

void ImuQualityMonitor::setConfig(const ImuQualityConfig& cfg) {
    cfg_ = cfg;
}

const ImuQualityConfig& ImuQualityMonitor::config() const {
    return cfg_;
}

void ImuQualityMonitor::reset() {
    counters_ = ImuQualityCounters{};
    lastSeenTimestampUs_ = 0;
    lastAcceptedTimestampUs_ = 0;
    lastStatsValid_ = false;
    recoveryRequested_ = false;
    lastRecoveryFlags_ = 0;
    lastStats_ = Lsm6dsvFifoReader::DrainStats{};
}

const ImuQualityCounters& ImuQualityMonitor::counters() const {
    return counters_;
}

bool ImuQualityMonitor::recoveryRequested() const {
    return recoveryRequested_;
}

uint32_t ImuQualityMonitor::lastRecoveryFlags() const {
    return lastRecoveryFlags_;
}

void ImuQualityMonitor::clearRecoveryRequest() {
    recoveryRequested_ = false;
    lastRecoveryFlags_ = 0;
}

void ImuQualityMonitor::resetStreamRecoveryState() {
    lastSeenTimestampUs_ = 0;
    lastAcceptedTimestampUs_ = 0;
    recoveryRequested_ = false;
    lastRecoveryFlags_ = 0;
    lastStatsValid_ = false;
}

void ImuQualityMonitor::syncFifoStats(const Lsm6dsvFifoReader::DrainStats& stats) {
    lastStats_ = stats;
    lastStatsValid_ = true;
}

ImuQualityResult ImuQualityMonitor::evaluate(const Lsm6dsv::RawSample& raw,
                                             const Lsm6dsv::Sample& calibrated,
                                             const Lsm6dsvFifoReader::DrainStats& fifoStats,
                                             bool checkFifoStatsDelta) {
    ImuQualityResult q;
    counters_.samples++;

    const float expectedDt = expectedDtUs(fifoStats);

    evaluateRawFlags(raw, q);
    evaluateTimestamp(raw, expectedDt, q);
    if (checkFifoStatsDelta) {
        evaluateFifoStatsDelta(fifoStats, q);
    }
    evaluateSaturation(raw, q);
    evaluateAccelNorm(calibrated, q);
    finalizeDecision(q);
    updateCounters(q);

    return q;
}

float ImuQualityMonitor::expectedDtUs(const Lsm6dsvFifoReader::DrainStats& fifoStats) const {
    if (cfg_.expectedDtUs > 0.0f) return cfg_.expectedDtUs;
    if (fifoStats.samplePeriodUs > 0.0f) return fifoStats.samplePeriodUs;
    return 0.0f;
}

void ImuQualityMonitor::evaluateRawFlags(const Lsm6dsv::RawSample& raw, ImuQualityResult& q) {
    if ((raw.flags & Lsm6dsvFifoReader::FIFO_FLAG_TS_HARDWARE) != 0) {
        q.flags |= imu_quality_flags::TIMESTAMP_HARDWARE;
    }
    if ((raw.flags & Lsm6dsvFifoReader::FIFO_FLAG_TS_FALLBACK) != 0) {
        q.flags |= imu_quality_flags::TIMESTAMP_FALLBACK;
        q.timestampConfidence *= 0.75f;
    }
    if ((raw.flags & Lsm6dsvFifoReader::FIFO_FLAG_STATUS_OVR) != 0) {
        q.flags |= imu_quality_flags::FIFO_OVERRUN;
    }
    if ((raw.flags & Lsm6dsvFifoReader::FIFO_FLAG_STATUS_FULL) != 0) {
        q.flags |= imu_quality_flags::FIFO_FULL;
    }
    if ((raw.flags & Lsm6dsvFifoReader::FIFO_FLAG_UNKNOWN_TAG) != 0) {
        q.flags |= imu_quality_flags::FIFO_UNKNOWN_TAG;
    }
    if ((raw.flags & Lsm6dsvFifoReader::FIFO_FLAG_ORPHAN_WORDS) != 0) {
        q.flags |= imu_quality_flags::FIFO_ORPHAN_WORDS;
    }
    if ((raw.components & Lsm6dsv::SAMPLE_COMPONENT_GYRO) == 0u) {
        q.flags |= imu_quality_flags::GYRO_COMPONENT_MISSING;
        q.gyroConfidence = 0.0f;
    }
    if ((raw.components & Lsm6dsv::SAMPLE_COMPONENT_ACCEL) == 0u) {
        q.flags |= imu_quality_flags::ACCEL_COMPONENT_MISSING;
        q.accelConfidence = 0.0f;
    }
    if (raw.coherency == Lsm6dsv::SampleCoherency::PairCounterMismatch) {
        q.flags |= imu_quality_flags::FIFO_PAIR_DEGRADED;
        q.accelConfidence = 0.0f;
    }
}

void ImuQualityMonitor::evaluateTimestamp(const Lsm6dsv::RawSample& raw,
                                          float expectedDt,
                                          ImuQualityResult& q) {
    // Keep the latest observed timestamp separate from the timestamp used
    // as the baseline for dt quality. A rejected/non-monotonic timestamp
    // must not poison the next normal sample's dt calculation. This mirrors
    // the AHRS lastSeenTimestampUs vs lastIntegratedTimestampUs semantics.
    if (raw.t_us != 0) {
        lastSeenTimestampUs_ = raw.t_us;
    }

    if (raw.t_us == 0) {
        q.flags |= imu_quality_flags::TIMESTAMP_ZERO;
        q.timestampConfidence = 0.0f;
        return;
    }

    if (lastAcceptedTimestampUs_ != 0) {
        if (raw.t_us <= lastAcceptedTimestampUs_) {
            q.flags |= imu_quality_flags::TIMESTAMP_NON_MONOTONIC;
            q.timestampConfidence = 0.0f;
            return;
        }

        const uint64_t dt64 = raw.t_us - lastAcceptedTimestampUs_;
        q.dtUs = dt64 > 0xFFFFFFFFULL ? 0xFFFFFFFFUL : static_cast<uint32_t>(dt64);

        if (expectedDt > 0.0f) {
            const float gapThreshold = expectedDt * cfg_.largeGapFactor;
            if (static_cast<float>(q.dtUs) > gapThreshold) {
                q.flags |= imu_quality_flags::TIMESTAMP_LARGE_GAP;

                const float ratio = static_cast<float>(q.dtUs) / expectedDt;
                const int32_t missing = static_cast<int32_t>(std::lround(ratio)) - 1;
                q.estimatedDroppedBefore = missing > 0 ? static_cast<uint32_t>(missing) : 1u;
                q.flags |= imu_quality_flags::SAMPLE_DROPPED_BEFORE;
                q.timestampConfidence *= 0.5f;
            }
        }

        if (counters_.samples == 1 || counters_.minDtUs == 0.0f) {
            counters_.minDtUs = static_cast<float>(q.dtUs);
            counters_.maxDtUs = static_cast<float>(q.dtUs);
        } else {
            if (static_cast<float>(q.dtUs) < counters_.minDtUs) counters_.minDtUs = static_cast<float>(q.dtUs);
            if (static_cast<float>(q.dtUs) > counters_.maxDtUs) counters_.maxDtUs = static_cast<float>(q.dtUs);
        }
        counters_.sumDtUs += static_cast<double>(q.dtUs);
    }

    lastAcceptedTimestampUs_ = raw.t_us;
}

void ImuQualityMonitor::evaluateFifoStatsDelta(const Lsm6dsvFifoReader::DrainStats& stats,
                                               ImuQualityResult& q) {
    if (!lastStatsValid_) {
        syncFifoStats(stats);
        return;
    }

    const uint32_t dOverrun = delta(stats.overrunEvents, lastStats_.overrunEvents);
    const uint32_t dFull = delta(stats.fullEvents, lastStats_.fullEvents);
    const uint32_t dUnknown = delta(stats.unknownWords, lastStats_.unknownWords);
    const uint32_t dTag = delta(stats.tagCounterJumps, lastStats_.tagCounterJumps);
    const uint32_t dGyroTag = delta(stats.gyroTagCounterJumps, lastStats_.gyroTagCounterJumps);
    const uint32_t dAccelTag = delta(stats.accelTagCounterJumps, lastStats_.accelTagCounterJumps);
    const uint32_t dTsQueue = delta(stats.timestampQueueOverflow, lastStats_.timestampQueueOverflow);
    const uint32_t dWaitQueue = delta(stats.waitingSampleQueueOverflow, lastStats_.waitingSampleQueueOverflow);
    const uint32_t dCompletedQueue = delta(stats.completedSampleQueueOverflow, lastStats_.completedSampleQueueOverflow);
    const uint32_t dBackwards = delta(stats.timestampBackwards, lastStats_.timestampBackwards);
    const uint32_t dMetaXl = delta(stats.timestampMetaBdrXlMismatch, lastStats_.timestampMetaBdrXlMismatch);
    const uint32_t dMetaGy = delta(stats.timestampMetaBdrGyMismatch, lastStats_.timestampMetaBdrGyMismatch);

    if (dOverrun > 0) q.flags |= imu_quality_flags::FIFO_OVERRUN;
    if (dFull > 0) q.flags |= imu_quality_flags::FIFO_FULL;
    if (dUnknown > 0) q.flags |= imu_quality_flags::FIFO_UNKNOWN_TAG;
    if (dTag > 0) q.flags |= imu_quality_flags::FIFO_TAG_COUNTER_JUMP;
    if (dGyroTag > 0) q.flags |= imu_quality_flags::FIFO_GYRO_TAG_COUNTER_JUMP;
    if (dAccelTag > 0) q.flags |= imu_quality_flags::FIFO_ACCEL_TAG_COUNTER_JUMP;
    if (dTsQueue > 0 || dWaitQueue > 0) q.flags |= imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW;
    if (dCompletedQueue > 0) q.flags |= imu_quality_flags::FIFO_COMPLETED_QUEUE_OVERFLOW;
    if (dBackwards > 0) q.flags |= imu_quality_flags::TIMESTAMP_BACKWARDS;
    if (dMetaXl > 0 || dMetaGy > 0) q.flags |= imu_quality_flags::TIMESTAMP_META_MISMATCH;

    counters_.fifoOverrunEvents += dOverrun;
    counters_.fifoFullEvents += dFull;
    counters_.fifoUnknownTagEvents += dUnknown;
    counters_.fifoTagCounterJumps += dTag;
    counters_.fifoGyroTagCounterJumps += dGyroTag;
    counters_.fifoAccelTagCounterJumps += dAccelTag;
    counters_.timestampQueueOverflows += dTsQueue;
    counters_.waitingSampleQueueOverflows += dWaitQueue;
    counters_.completedSampleQueueOverflows += dCompletedQueue;
    counters_.timestampBackwards += dBackwards;
    counters_.timestampMetaMismatches += dMetaXl + dMetaGy;

    lastStats_ = stats;
}

void ImuQualityMonitor::evaluateSaturation(const Lsm6dsv::RawSample& raw, ImuQualityResult& q) {
    const bool hasGyro = (raw.components & Lsm6dsv::SAMPLE_COMPONENT_GYRO) != 0u;
    const bool hasAccel = (raw.components & Lsm6dsv::SAMPLE_COMPONENT_ACCEL) != 0u;

    if (hasGyro && (raw.flags & Lsm6dsv::FLAG_GYRO_SATURATED) != 0) {
        q.flags |= imu_quality_flags::GYRO_SATURATED;
        q.gyroConfidence = 0.0f;
    }
    if (hasAccel && (raw.flags & Lsm6dsv::FLAG_ACCEL_SATURATED) != 0) {
        q.flags |= imu_quality_flags::ACCEL_SATURATED;
        q.accelConfidence = 0.0f;
    }

    if (hasGyro && (nearAbs(raw.gx, cfg_.gyroNearSaturationAbsRaw) ||
        nearAbs(raw.gy, cfg_.gyroNearSaturationAbsRaw) ||
        nearAbs(raw.gz, cfg_.gyroNearSaturationAbsRaw))) {
        q.flags |= imu_quality_flags::GYRO_NEAR_SATURATION;
        q.gyroConfidence *= 0.5f;
    }

    if (hasAccel && (nearAbs(raw.ax, cfg_.accelNearSaturationAbsRaw) ||
        nearAbs(raw.ay, cfg_.accelNearSaturationAbsRaw) ||
        nearAbs(raw.az, cfg_.accelNearSaturationAbsRaw))) {
        q.flags |= imu_quality_flags::ACCEL_NEAR_SATURATION;
        q.accelConfidence *= 0.5f;
    }
}

void ImuQualityMonitor::evaluateAccelNorm(const Lsm6dsv::Sample& calibrated, ImuQualityResult& q) {
    if (q.has(imu_quality_flags::ACCEL_COMPONENT_MISSING) ||
        q.has(imu_quality_flags::FIFO_PAIR_DEGRADED)) {
        q.accelNormG = 0.0f;
        q.accelNormValid = false;
        return;
    }
    const float n = calibrated.accel_g.norm();
    q.accelNormG = n;
    q.accelNormValid = std::isfinite(n);
    if (!q.accelNormValid || n < cfg_.accelNormOutlierMinG || n > cfg_.accelNormOutlierMaxG) {
        q.flags |= imu_quality_flags::ACCEL_NORM_OUTLIER;
        q.accelConfidence *= 0.0f;
    }
}

void ImuQualityMonitor::finalizeDecision(ImuQualityResult& q) {
    if (q.has(imu_quality_flags::TIMESTAMP_ZERO) ||
        q.has(imu_quality_flags::TIMESTAMP_NON_MONOTONIC) ||
        q.has(imu_quality_flags::TIMESTAMP_BACKWARDS) ||
        q.has(imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW)) {
        q.timestampConfidence = 0.0f;
    }

    if (q.has(imu_quality_flags::FIFO_OVERRUN) && cfg_.requestRecoveryOnFifoOverrun) {
        requestRecovery(q, imu_quality_flags::FIFO_OVERRUN);
    }
    if (q.has(imu_quality_flags::FIFO_FULL) && cfg_.requestRecoveryOnFifoFull) {
        requestRecovery(q, imu_quality_flags::FIFO_FULL);
    }
    if (q.has(imu_quality_flags::FIFO_UNKNOWN_TAG) && cfg_.requestRecoveryOnUnknownTag) {
        requestRecovery(q, imu_quality_flags::FIFO_UNKNOWN_TAG);
    }
    if (q.has(imu_quality_flags::TIMESTAMP_BACKWARDS) && cfg_.requestRecoveryOnTimestampBackwards) {
        requestRecovery(q, imu_quality_flags::TIMESTAMP_BACKWARDS);
    }
    if (q.has(imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW) && cfg_.requestRecoveryOnTimestampQueueOverflow) {
        requestRecovery(q, imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW);
    }
    if (q.has(imu_quality_flags::FIFO_COMPLETED_QUEUE_OVERFLOW) &&
        cfg_.requestRecoveryOnCompletedQueueOverflow) {
        requestRecovery(q, imu_quality_flags::FIFO_COMPLETED_QUEUE_OVERFLOW);
    }

    q.shouldUpdateAhrs = !q.has(imu_quality_flags::GYRO_COMPONENT_MISSING);
    if (cfg_.skipAhrsOnBadTimestamp && q.timestampConfidence <= 0.0f) {
        q.shouldUpdateAhrs = false;
    }
    if (cfg_.skipAhrsOnGyroSaturation && q.has(imu_quality_flags::GYRO_SATURATED)) {
        q.shouldUpdateAhrs = false;
    }

    q.shouldUseAccelOutput = !q.has(imu_quality_flags::ACCEL_COMPONENT_MISSING) &&
                             !q.has(imu_quality_flags::FIFO_PAIR_DEGRADED) &&
                             !q.has(imu_quality_flags::ACCEL_SATURATED);

    q.shouldUseAccelCorrection = q.shouldUseAccelOutput;
    if (cfg_.disableAccelCorrectionOnAccelSaturation && q.has(imu_quality_flags::ACCEL_SATURATED)) {
        q.shouldUseAccelCorrection = false;
    }
    if (cfg_.disableAccelCorrectionOnAccelNormOutlier && q.has(imu_quality_flags::ACCEL_NORM_OUTLIER)) {
        q.shouldUseAccelCorrection = false;
    }

    if (!q.shouldUpdateAhrs) q.flags |= imu_quality_flags::SAMPLE_NOT_AHRS_USABLE;
    if (!q.shouldUseAccelCorrection) q.flags |= imu_quality_flags::ACCEL_NOT_AHRS_USABLE;

    q.overallConfidence = q.timestampConfidence * q.gyroConfidence;
    if (q.accelConfidence < q.overallConfidence) {
        // Overall remains gyro/timestamp-oriented, but severe accel problems
        // should still lower generic confidence for output packets.
        q.overallConfidence = 0.5f * q.overallConfidence + 0.5f * q.accelConfidence;
    }
}

void ImuQualityMonitor::updateCounters(const ImuQualityResult& q) {
    if (q.has(imu_quality_flags::TIMESTAMP_HARDWARE)) counters_.hwTimestampSamples++;
    if (q.has(imu_quality_flags::TIMESTAMP_FALLBACK)) counters_.fallbackTimestampSamples++;
    if (q.has(imu_quality_flags::TIMESTAMP_ZERO)) counters_.zeroTimestampSamples++;
    if (q.has(imu_quality_flags::TIMESTAMP_NON_MONOTONIC)) counters_.nonMonotonicTimestampSamples++;
    if (q.has(imu_quality_flags::TIMESTAMP_LARGE_GAP)) counters_.largeGapSamples++;
    if (q.has(imu_quality_flags::SAMPLE_DROPPED_BEFORE)) {
        counters_.estimatedDroppedSamples += q.estimatedDroppedBefore;
    }

    if (q.has(imu_quality_flags::GYRO_SATURATED)) counters_.gyroSaturatedSamples++;
    if (q.has(imu_quality_flags::ACCEL_SATURATED)) counters_.accelSaturatedSamples++;
    if (q.has(imu_quality_flags::GYRO_NEAR_SATURATION)) counters_.gyroNearSaturatedSamples++;
    if (q.has(imu_quality_flags::ACCEL_NEAR_SATURATION)) counters_.accelNearSaturatedSamples++;
    if (q.has(imu_quality_flags::ACCEL_NORM_OUTLIER)) counters_.accelNormOutliers++;
    if (q.has(imu_quality_flags::ACCEL_COMPONENT_MISSING)) counters_.accelComponentMissingSamples++;
    if (q.has(imu_quality_flags::GYRO_COMPONENT_MISSING)) counters_.gyroComponentMissingSamples++;
    if (q.has(imu_quality_flags::FIFO_PAIR_DEGRADED)) counters_.pairCoherencyDegradedSamples++;

    if (!q.shouldUpdateAhrs) counters_.ahrsSkippedSamples++;
    if (!q.shouldUseAccelCorrection) counters_.accelCorrectionDisabledSamples++;
    if (q.shouldRequestFifoRecovery) counters_.fifoRecoveryRequests++;
}

void ImuQualityMonitor::requestRecovery(ImuQualityResult& q, uint32_t reasonFlag) {
    if (reasonFlag == imu_quality_flags::FIFO_OVERRUN) counters_.fifoRecoveryOverrunRequests++;
    else if (reasonFlag == imu_quality_flags::FIFO_FULL) counters_.fifoRecoveryFullRequests++;
    else if (reasonFlag == imu_quality_flags::FIFO_UNKNOWN_TAG) counters_.fifoRecoveryUnknownTagRequests++;
    else if (reasonFlag == imu_quality_flags::TIMESTAMP_BACKWARDS) counters_.fifoRecoveryTimestampBackwardsRequests++;
    else if (reasonFlag == imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW) counters_.fifoRecoveryTimestampQueueOverflowRequests++;
    else if (reasonFlag == imu_quality_flags::FIFO_COMPLETED_QUEUE_OVERFLOW) counters_.fifoRecoveryCompletedQueueOverflowRequests++;

    q.flags |= imu_quality_flags::FIFO_RECOVERY_REQUESTED;
    q.shouldRequestFifoRecovery = true;
    recoveryRequested_ = true;
    lastRecoveryFlags_ |= reasonFlag;
}

uint32_t ImuQualityMonitor::delta(uint32_t current, uint32_t previous) {
    return current >= previous ? current - previous : 0;
}

bool ImuQualityMonitor::nearAbs(int16_t v, int16_t threshold) {
    const int32_t a = v < 0 ? -static_cast<int32_t>(v) : static_cast<int32_t>(v);
    return a >= static_cast<int32_t>(threshold);
}

} // namespace tracker
