#pragma once

#include <cstdint>

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
// Unknown FIFO tags are diagnosed but are not recovery-triggering by
// default, because valid-but-disabled batched sources such as sensor-hub
// words can otherwise create an endless reset loop.
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
static constexpr uint32_t TEMP_COMP_OUT_OF_RANGE     = 1u << 24;
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
    bool requestRecoveryOnUnknownTag = false;
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

    bool has(uint32_t f) const;
    void markTempCompOutOfRange(float confidenceMultiplier = 0.85f);
    Vec3 accelForAhrs(const Vec3& calibratedAccelG) const;
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

    float meanDtUs() const;
};

class ImuQualityMonitor {
public:
    explicit ImuQualityMonitor(const ImuQualityConfig& cfg = ImuQualityConfig{});

    void setConfig(const ImuQualityConfig& cfg);
    const ImuQualityConfig& config() const;

    void reset();
    const ImuQualityCounters& counters() const;

    bool recoveryRequested() const;
    uint32_t lastRecoveryFlags() const;
    void clearRecoveryRequest();

    // Stream recovery clears timestamp/recovery baselines after an explicit
    // FIFO reset without losing long-running diagnostic counters. Use this
    // after blocking commands or FIFO overrun recovery, before syncing fresh
    // FIFO stats, so the next real sample becomes a clean timing baseline.
    void resetStreamRecoveryState();

    void syncFifoStats(const Lsm6dsvFifoReader::DrainStats& stats);

    ImuQualityResult evaluate(const Lsm6dsv::RawSample& raw,
                              const Lsm6dsv::Sample& calibrated,
                              const Lsm6dsvFifoReader::DrainStats& fifoStats,
                              bool checkFifoStatsDelta = true);

private:
    float expectedDtUs(const Lsm6dsvFifoReader::DrainStats& fifoStats) const;
    void evaluateRawFlags(const Lsm6dsv::RawSample& raw, ImuQualityResult& q);
    void evaluateTimestamp(const Lsm6dsv::RawSample& raw, float expectedDt, ImuQualityResult& q);
    void evaluateFifoStatsDelta(const Lsm6dsvFifoReader::DrainStats& stats, ImuQualityResult& q);
    void evaluateSaturation(const Lsm6dsv::RawSample& raw, ImuQualityResult& q);
    void evaluateAccelNorm(const Lsm6dsv::Sample& calibrated, ImuQualityResult& q);
    void finalizeDecision(ImuQualityResult& q);
    void updateCounters(const ImuQualityResult& q);
    void requestRecovery(ImuQualityResult& q, uint32_t reasonFlag);

    static uint32_t delta(uint32_t current, uint32_t previous);
    static bool nearAbs(int16_t v, int16_t threshold);

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
