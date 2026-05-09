#pragma once

#include <cstdint>
#include <cmath>

#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"

namespace tracker {

// ============================================================
// IMU sample quality / dropped-sample / saturation monitor
// ============================================================
// Purpose:
//   - Convert low-level FIFO/driver flags into explicit sample-quality flags.
//   - Detect timestamp gaps / non-monotonic timestamps.
//   - Track FIFO overrun/full/unknown/tag-counter/timestamp faults.
//   - Track hard and near saturation for accel/gyro.
//   - Provide AHRS decisions: update/skip and accel-correction allow/deny.
//   - Request FIFO recovery after serious stream faults.
//
// This module does NOT modify RawSample. It creates a parallel quality result
// suitable for logs, output packets, and AHRS gating.
// ============================================================

namespace imu_quality_flags {
static constexpr uint32_t OK                         = 0u;

// Timestamp source / faults.
static constexpr uint32_t TIMESTAMP_HARDWARE         = 1u << 0;
static constexpr uint32_t TIMESTAMP_FALLBACK         = 1u << 1;
static constexpr uint32_t TIMESTAMP_ZERO             = 1u << 2;
static constexpr uint32_t TIMESTAMP_NON_MONOTONIC    = 1u << 3;
static constexpr uint32_t TIMESTAMP_LARGE_GAP        = 1u << 4;
static constexpr uint32_t TIMESTAMP_QUEUE_OVERFLOW   = 1u << 5;
static constexpr uint32_t TIMESTAMP_META_MISMATCH    = 1u << 6;
static constexpr uint32_t TIMESTAMP_BACKWARDS        = 1u << 7;

// FIFO stream faults.
static constexpr uint32_t FIFO_OVERRUN               = 1u << 8;
static constexpr uint32_t FIFO_FULL                  = 1u << 9;
static constexpr uint32_t FIFO_UNKNOWN_TAG           = 1u << 10;
static constexpr uint32_t FIFO_ORPHAN_WORDS          = 1u << 11;
static constexpr uint32_t FIFO_TAG_COUNTER_JUMP      = 1u << 12;
static constexpr uint32_t FIFO_GYRO_TAG_COUNTER_JUMP = 1u << 13;
static constexpr uint32_t FIFO_ACCEL_TAG_COUNTER_JUMP= 1u << 14;
static constexpr uint32_t FIFO_RECOVERY_REQUESTED    = 1u << 15;

// Sensor data quality.
static constexpr uint32_t GYRO_SATURATED             = 1u << 16;
static constexpr uint32_t ACCEL_SATURATED            = 1u << 17;
static constexpr uint32_t GYRO_NEAR_SATURATION       = 1u << 18;
static constexpr uint32_t ACCEL_NEAR_SATURATION      = 1u << 19;
static constexpr uint32_t ACCEL_NORM_OUTLIER         = 1u << 20;
static constexpr uint32_t SAMPLE_DROPPED_BEFORE      = 1u << 21;
static constexpr uint32_t SAMPLE_NOT_AHRS_USABLE     = 1u << 22;
static constexpr uint32_t ACCEL_NOT_AHRS_USABLE      = 1u << 23;
static constexpr uint32_t TEMP_COMP_OUT_OF_RANGE       = 1u << 24;
}

struct ImuQualityConfig {
    // If <= 0, monitor will use fifo.stats().samplePeriodUs.
    float expectedDtUs = 0.0f;

    // dt > expectedDtUs * largeGapFactor means samples were probably skipped.
    float largeGapFactor = 1.75f;

    // dt < expectedDtUs * smallGapFactor means timestamp is suspicious.
    // Currently counted as non-monotonic only when dt <= 0; small gaps are kept as info.
    float smallGapFactor = 0.25f;

    // Raw near-saturation thresholds. Hard saturation is INT16_MIN/MAX.
    // Near saturation warns before hard clipping.
    int16_t gyroNearSaturationAbsRaw = 30000;
    int16_t accelNearSaturationAbsRaw = 30000;

    // Accel norm outlier. Used only for accel correction gating, not for gyro integration.
    float accelNormOutlierMinG = 0.50f;
    float accelNormOutlierMaxG = 1.50f;

    // Recovery policy.
    bool requestRecoveryOnFifoOverrun = true;
    bool requestRecoveryOnFifoFull = true;
    bool requestRecoveryOnUnknownTag = true;
    bool requestRecoveryOnTimestampBackwards = true;
    bool requestRecoveryOnTimestampQueueOverflow = true;

    // AHRS policy.
    bool skipAhrsOnBadTimestamp = true;
    bool skipAhrsOnGyroSaturation = true;
    bool disableAccelCorrectionOnAccelSaturation = true;
    bool disableAccelCorrectionOnAccelNormOutlier = true;
};

struct ImuQualityResult {
    uint32_t flags = imu_quality_flags::OK;
    uint32_t estimatedDroppedBefore = 0;
    uint32_t dtUs = 0;

    float timestampConfidence = 1.0f;
    float gyroConfidence = 1.0f;
    float accelConfidence = 1.0f;
    float overallConfidence = 1.0f;

    // Cached calibrated accel norm. This is calculated once by the quality
    // monitor and can be reused by AHRS/static-test hot paths to avoid
    // duplicate sqrt() calls without changing any decision logic.
    float accelNormG = 0.0f;
    bool accelNormValid = false;

    bool shouldUpdateAhrs = true;
    bool shouldUseAccelCorrection = true;
    bool shouldRequestFifoRecovery = false;

    bool has(uint32_t f) const {
        return (flags & f) != 0;
    }

    void markTempCompOutOfRange(float confidenceMultiplier = 0.85f) {
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

    Vec3 accelForAhrs(const Vec3& calibratedAccelG) const {
        return shouldUseAccelCorrection ? calibratedAccelG : Vec3::zero();
    }
};

struct ImuQualityCounters {
    uint32_t samples = 0;

    uint32_t hwTimestampSamples = 0;
    uint32_t fallbackTimestampSamples = 0;
    uint32_t zeroTimestampSamples = 0;
    uint32_t nonMonotonicTimestampSamples = 0;
    uint32_t largeGapSamples = 0;
    uint32_t estimatedDroppedSamples = 0;

    uint32_t fifoOverrunEvents = 0;
    uint32_t fifoFullEvents = 0;
    uint32_t fifoUnknownTagEvents = 0;
    uint32_t fifoTagCounterJumps = 0;
    uint32_t fifoGyroTagCounterJumps = 0;
    uint32_t fifoAccelTagCounterJumps = 0;
    uint32_t timestampQueueOverflows = 0;
    uint32_t waitingSampleQueueOverflows = 0;
    uint32_t timestampBackwards = 0;
    uint32_t timestampMetaMismatches = 0;

    uint32_t gyroSaturatedSamples = 0;
    uint32_t accelSaturatedSamples = 0;
    uint32_t gyroNearSaturatedSamples = 0;
    uint32_t accelNearSaturatedSamples = 0;
    uint32_t accelNormOutliers = 0;

    uint32_t ahrsSkippedSamples = 0;
    uint32_t accelCorrectionDisabledSamples = 0;
    uint32_t fifoRecoveryRequests = 0;

    float minDtUs = 0.0f;
    float maxDtUs = 0.0f;
    double sumDtUs = 0.0;

    float meanDtUs() const {
        const uint32_t validDtCount = samples - zeroTimestampSamples - nonMonotonicTimestampSamples;
        return validDtCount == 0 ? 0.0f : static_cast<float>(sumDtUs / static_cast<double>(validDtCount));
    }
};

class ImuQualityMonitor {
public:
    explicit ImuQualityMonitor(const ImuQualityConfig& cfg = ImuQualityConfig{})
        : cfg_(cfg) {}

    void setConfig(const ImuQualityConfig& cfg) {
        cfg_ = cfg;
    }

    const ImuQualityConfig& config() const {
        return cfg_;
    }

    void reset() {
        counters_ = ImuQualityCounters{};
        lastSeenTimestampUs_ = 0;
        lastAcceptedTimestampUs_ = 0;
        lastStatsValid_ = false;
        recoveryRequested_ = false;
        lastRecoveryFlags_ = 0;
        lastStats_ = Lsm6dsvFifoReader::DrainStats{};
    }

    const ImuQualityCounters& counters() const {
        return counters_;
    }

    bool recoveryRequested() const {
        return recoveryRequested_;
    }

    uint32_t lastRecoveryFlags() const {
        return lastRecoveryFlags_;
    }

    void clearRecoveryRequest() {
        recoveryRequested_ = false;
        lastRecoveryFlags_ = 0;
    }

    void syncFifoStats(const Lsm6dsvFifoReader::DrainStats& stats) {
        lastStats_ = stats;
        lastStatsValid_ = true;
    }

    ImuQualityResult evaluate(const Lsm6dsv::RawSample& raw,
                              const Lsm6dsv::Sample& calibrated,
                              const Lsm6dsvFifoReader::DrainStats& fifoStats,
                              bool checkFifoStatsDelta = true) {
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

private:
    float expectedDtUs(const Lsm6dsvFifoReader::DrainStats& fifoStats) const {
        if (cfg_.expectedDtUs > 0.0f) return cfg_.expectedDtUs;
        if (fifoStats.samplePeriodUs > 0.0f) return fifoStats.samplePeriodUs;
        return 0.0f;
    }

    void evaluateRawFlags(const Lsm6dsv::RawSample& raw, ImuQualityResult& q) {
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
    }

    void evaluateTimestamp(const Lsm6dsv::RawSample& raw,
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

    void evaluateFifoStatsDelta(const Lsm6dsvFifoReader::DrainStats& stats,
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
        counters_.timestampBackwards += dBackwards;
        counters_.timestampMetaMismatches += dMetaXl + dMetaGy;

        lastStats_ = stats;
    }

    void evaluateSaturation(const Lsm6dsv::RawSample& raw, ImuQualityResult& q) {
        if ((raw.flags & Lsm6dsv::FLAG_GYRO_SATURATED) != 0) {
            q.flags |= imu_quality_flags::GYRO_SATURATED;
            q.gyroConfidence = 0.0f;
        }
        if ((raw.flags & Lsm6dsv::FLAG_ACCEL_SATURATED) != 0) {
            q.flags |= imu_quality_flags::ACCEL_SATURATED;
            q.accelConfidence = 0.0f;
        }

        if (nearAbs(raw.gx, cfg_.gyroNearSaturationAbsRaw) ||
            nearAbs(raw.gy, cfg_.gyroNearSaturationAbsRaw) ||
            nearAbs(raw.gz, cfg_.gyroNearSaturationAbsRaw)) {
            q.flags |= imu_quality_flags::GYRO_NEAR_SATURATION;
            q.gyroConfidence *= 0.5f;
        }

        if (nearAbs(raw.ax, cfg_.accelNearSaturationAbsRaw) ||
            nearAbs(raw.ay, cfg_.accelNearSaturationAbsRaw) ||
            nearAbs(raw.az, cfg_.accelNearSaturationAbsRaw)) {
            q.flags |= imu_quality_flags::ACCEL_NEAR_SATURATION;
            q.accelConfidence *= 0.5f;
        }
    }

    void evaluateAccelNorm(const Lsm6dsv::Sample& calibrated, ImuQualityResult& q) {
        const float n = calibrated.accel_g.norm();
        q.accelNormG = n;
        q.accelNormValid = std::isfinite(n);
        if (!q.accelNormValid || n < cfg_.accelNormOutlierMinG || n > cfg_.accelNormOutlierMaxG) {
            q.flags |= imu_quality_flags::ACCEL_NORM_OUTLIER;
            q.accelConfidence *= 0.0f;
        }
    }

    void finalizeDecision(ImuQualityResult& q) {
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

        q.shouldUpdateAhrs = true;
        if (cfg_.skipAhrsOnBadTimestamp && q.timestampConfidence <= 0.0f) {
            q.shouldUpdateAhrs = false;
        }
        if (cfg_.skipAhrsOnGyroSaturation && q.has(imu_quality_flags::GYRO_SATURATED)) {
            q.shouldUpdateAhrs = false;
        }

        q.shouldUseAccelCorrection = true;
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

    void updateCounters(const ImuQualityResult& q) {
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

        if (!q.shouldUpdateAhrs) counters_.ahrsSkippedSamples++;
        if (!q.shouldUseAccelCorrection) counters_.accelCorrectionDisabledSamples++;
        if (q.shouldRequestFifoRecovery) counters_.fifoRecoveryRequests++;
    }

    void requestRecovery(ImuQualityResult& q, uint32_t reasonFlag) {
        q.flags |= imu_quality_flags::FIFO_RECOVERY_REQUESTED;
        q.shouldRequestFifoRecovery = true;
        recoveryRequested_ = true;
        lastRecoveryFlags_ |= reasonFlag;
    }

    static uint32_t delta(uint32_t current, uint32_t previous) {
        return current >= previous ? current - previous : 0;
    }

    static bool nearAbs(int16_t v, int16_t threshold) {
        const int32_t a = v < 0 ? -static_cast<int32_t>(v) : static_cast<int32_t>(v);
        return a >= static_cast<int32_t>(threshold);
    }

    ImuQualityConfig cfg_;
    ImuQualityCounters counters_;
    uint64_t lastSeenTimestampUs_ = 0;
    uint64_t lastAcceptedTimestampUs_ = 0;

    bool lastStatsValid_ = false;
    Lsm6dsvFifoReader::DrainStats lastStats_;

    bool recoveryRequested_ = false;
    uint32_t lastRecoveryFlags_ = 0;
};

} // namespace tracker
