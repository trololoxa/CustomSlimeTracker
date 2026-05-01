#pragma once

#include <cstdint>
#include <cmath>

#include "core/math.hpp"
#include "connection/lsm6dsv_fifo.hpp"

namespace tracker {

enum MagRejectFlags : uint32_t {
    MAG_REJECT_NONE             = 0,
    MAG_REJECT_DISABLED         = 1u << 0,
    MAG_REJECT_RAW_SATURATED    = 1u << 1,
    MAG_REJECT_RAW_NONFINITE    = 1u << 2,
    MAG_REJECT_NOT_CALIBRATED   = 1u << 3,
    MAG_REJECT_AXIS_NOT_ALIGNED = 1u << 4,
    MAG_REJECT_NORM_TOO_LOW     = 1u << 5,
    MAG_REJECT_NORM_TOO_HIGH    = 1u << 6,

    // Important:
    // STALE is not assigned during process(), because process() receives a
    // freshly popped FIFO sample. STALE is evaluated at use-time from
    // receivedMs, not from raw.t_us.
    MAG_REJECT_STALE            = 1u << 7,

    MAG_REJECT_ZERO_NORM        = 1u << 8,
};

struct MagRuntimeConfig {
    bool enabled = false;

    bool calibrationValid = false;
    bool axisAlignmentValid = false;

    Vec3 hardIron = Vec3::zero();
    Mat3 softIron = Mat3::identity();
    Mat3 magToImu = Mat3::identity();

    float expectedFieldNorm = 1.0f;
    float minTrustNorm = 0.25f;
    float maxTrustNorm = 2.50f;

    // Runtime use-time gate. This is evaluated against receivedMs.
    uint32_t maxSampleAgeMs = 250;

    float minUsableNorm = 1.0e-6f;
};

struct MagProcessedSample {
    bool valid = false;
    bool trusted = false;

    // Reject flags valid at process-time, excluding STALE.
    // Use MagRuntimeProcessor::rejectFlagsForUse(...) to include STALE.
    uint32_t rejectFlags = MAG_REJECT_NONE;

    // Sensor/FIFO timestamp. This is for IMU/AHRS alignment, not for MCU stale checks.
    uint64_t t_us = 0;

    // MCU time when this sample was received/processed.
    // This is the timebase used for stale checks.
    uint32_t receivedMs = 0;

    uint32_t seq = 0;

    Vec3 raw = Vec3::zero();

    // hardIron removed, softIron applied; still in QMC/mag frame.
    Vec3 calibratedMagFrame = Vec3::zero();

    // calibrated + magToImu applied; this is what AHRS should eventually use.
    Vec3 body = Vec3::zero();

    float rawNorm = 0.0f;
    float calibratedNorm = 0.0f;
    float bodyNorm = 0.0f;

    uint16_t rawFlags = 0;
};

struct MagRuntimeStats {
    uint32_t rawSamples = 0;
    uint32_t processedSamples = 0;
    uint32_t trustedSamples = 0;
    uint32_t rejectedSamples = 0;

    uint32_t rejectedDisabled = 0;
    uint32_t rejectedRawSaturated = 0;
    uint32_t rejectedRawNonfinite = 0;
    uint32_t rejectedNotCalibrated = 0;
    uint32_t rejectedAxisNotAligned = 0;
    uint32_t rejectedNormTooLow = 0;
    uint32_t rejectedNormTooHigh = 0;
    uint32_t rejectedStale = 0;
    uint32_t rejectedZeroNorm = 0;

    uint32_t lastSampleMs = 0;
    uint32_t lastTrustedMs = 0;

    float rawNormMin = 0.0f;
    float rawNormMax = 0.0f;
    double rawNormSum = 0.0;

    float bodyNormMin = 0.0f;
    float bodyNormMax = 0.0f;
    double bodyNormSum = 0.0;

    float trustedBodyNormMin = 0.0f;
    float trustedBodyNormMax = 0.0f;
    double trustedBodyNormSum = 0.0;

    float rawNormMean() const {
        return rawSamples > 0 ? static_cast<float>(rawNormSum / static_cast<double>(rawSamples)) : 0.0f;
    }

    float bodyNormMean() const {
        return processedSamples > 0 ? static_cast<float>(bodyNormSum / static_cast<double>(processedSamples)) : 0.0f;
    }

    float trustedBodyNormMean() const {
        return trustedSamples > 0 ? static_cast<float>(trustedBodyNormSum / static_cast<double>(trustedSamples)) : 0.0f;
    }
};

class MagRuntimeProcessor {
public:
    void reset() {
        last_ = MagProcessedSample{};
        stats_ = MagRuntimeStats{};
    }

    const MagProcessedSample& last() const { return last_; }
    const MagRuntimeStats& stats() const { return stats_; }

    bool process(const Lsm6dsvFifoReader::MagRawSample& raw,
                 const MagRuntimeConfig& cfg,
                 uint32_t nowMs,
                 MagProcessedSample& out) {
        out = MagProcessedSample{};
        out.valid = true;
        out.t_us = raw.t_us;
        out.receivedMs = nowMs;
        out.seq = raw.seq;
        out.rawFlags = raw.flags;

        out.raw = Vec3(
            static_cast<float>(raw.x),
            static_cast<float>(raw.y),
            static_cast<float>(raw.z)
        );

        stats_.rawSamples++;
        stats_.lastSampleMs = nowMs;

        out.rawNorm = out.raw.norm();
        pushRawNorm(out.rawNorm);

        if (!cfg.enabled) {
            addReject(out, MAG_REJECT_DISABLED);
        }

        if ((raw.flags & Lsm6dsvFifoReader::MAG_FLAG_RAW_SATURATED) != 0) {
            addReject(out, MAG_REJECT_RAW_SATURATED);
        }

        if (!out.raw.isFinite() || !tracker::isFinite(out.rawNorm)) {
            addReject(out, MAG_REJECT_RAW_NONFINITE);
        }

        if (out.rawNorm <= cfg.minUsableNorm) {
            addReject(out, MAG_REJECT_ZERO_NORM);
        }

        if (cfg.calibrationValid) {
            const Vec3 hardCorrected = out.raw - cfg.hardIron;
            out.calibratedMagFrame = cfg.softIron * hardCorrected;
        } else {
            out.calibratedMagFrame = out.raw;
            addReject(out, MAG_REJECT_NOT_CALIBRATED);
        }

        if (cfg.axisAlignmentValid) {
            out.body = cfg.magToImu * out.calibratedMagFrame;
        } else {
            out.body = out.calibratedMagFrame;
            addReject(out, MAG_REJECT_AXIS_NOT_ALIGNED);
        }

        out.calibratedNorm = out.calibratedMagFrame.norm();
        out.bodyNorm = out.body.norm();

        if (!tracker::isFinite(out.calibratedNorm) ||
            !tracker::isFinite(out.bodyNorm) ||
            !out.calibratedMagFrame.isFinite() ||
            !out.body.isFinite()) {
            addReject(out, MAG_REJECT_RAW_NONFINITE);
        }

        if (out.bodyNorm <= cfg.minUsableNorm) {
            addReject(out, MAG_REJECT_ZERO_NORM);
        }

        if (cfg.calibrationValid) {
            if (out.bodyNorm < cfg.minTrustNorm) {
                addReject(out, MAG_REJECT_NORM_TOO_LOW);
            }
            if (out.bodyNorm > cfg.maxTrustNorm) {
                addReject(out, MAG_REJECT_NORM_TOO_HIGH);
            }
        }

        out.trusted = (out.rejectFlags == MAG_REJECT_NONE);

        stats_.processedSamples++;
        pushBodyNorm(out.bodyNorm);

        if (out.trusted) {
            stats_.trustedSamples++;
            stats_.lastTrustedMs = nowMs;
            pushTrustedBodyNorm(out.bodyNorm);
        } else {
            stats_.rejectedSamples++;
            countRejects(out.rejectFlags);
        }

        last_ = out;
        return out.valid;
    }

    bool process(const Lsm6dsvFifoReader::MagRawSample& raw,
                 const MagRuntimeConfig& cfg,
                 uint32_t nowMs) {
        MagProcessedSample out;
        return process(raw, cfg, nowMs, out);
    }

    static uint32_t ageMsForUse(const MagProcessedSample& sample, uint32_t nowMs) {
        if (!sample.valid || sample.receivedMs == 0) {
            return 0xFFFFFFFFUL;
        }
        return nowMs - sample.receivedMs;
    }

    static uint32_t rejectFlagsForUse(const MagProcessedSample& sample,
                                      const MagRuntimeConfig& cfg,
                                      uint32_t nowMs) {
        uint32_t flags = sample.rejectFlags;
        const uint32_t ageMs = ageMsForUse(sample, nowMs);
        if (ageMs > cfg.maxSampleAgeMs) {
            flags |= MAG_REJECT_STALE;
        }
        return flags;
    }

    static bool trustedForUse(const MagProcessedSample& sample,
                              const MagRuntimeConfig& cfg,
                              uint32_t nowMs) {
        if (!sample.valid) return false;
        return rejectFlagsForUse(sample, cfg, nowMs) == MAG_REJECT_NONE;
    }

    static const char* rejectFlagName(uint32_t singleFlag) {
        switch (singleFlag) {
            case MAG_REJECT_NONE: return "none";
            case MAG_REJECT_DISABLED: return "disabled";
            case MAG_REJECT_RAW_SATURATED: return "raw_saturated";
            case MAG_REJECT_RAW_NONFINITE: return "raw_nonfinite";
            case MAG_REJECT_NOT_CALIBRATED: return "not_calibrated";
            case MAG_REJECT_AXIS_NOT_ALIGNED: return "axis_not_aligned";
            case MAG_REJECT_NORM_TOO_LOW: return "norm_too_low";
            case MAG_REJECT_NORM_TOO_HIGH: return "norm_too_high";
            case MAG_REJECT_STALE: return "stale";
            case MAG_REJECT_ZERO_NORM: return "zero_norm";
        }
        return "unknown";
    }

private:
    void addReject(MagProcessedSample& out, uint32_t flag) {
        out.rejectFlags |= flag;
    }

    void countRejects(uint32_t flags) {
        if (flags & MAG_REJECT_DISABLED)         stats_.rejectedDisabled++;
        if (flags & MAG_REJECT_RAW_SATURATED)    stats_.rejectedRawSaturated++;
        if (flags & MAG_REJECT_RAW_NONFINITE)    stats_.rejectedRawNonfinite++;
        if (flags & MAG_REJECT_NOT_CALIBRATED)   stats_.rejectedNotCalibrated++;
        if (flags & MAG_REJECT_AXIS_NOT_ALIGNED) stats_.rejectedAxisNotAligned++;
        if (flags & MAG_REJECT_NORM_TOO_LOW)     stats_.rejectedNormTooLow++;
        if (flags & MAG_REJECT_NORM_TOO_HIGH)    stats_.rejectedNormTooHigh++;
        if (flags & MAG_REJECT_STALE)            stats_.rejectedStale++;
        if (flags & MAG_REJECT_ZERO_NORM)        stats_.rejectedZeroNorm++;
    }

    void pushRawNorm(float n) {
        if (!tracker::isFinite(n)) return;
        if (stats_.rawSamples == 1) {
            stats_.rawNormMin = n;
            stats_.rawNormMax = n;
        } else {
            if (n < stats_.rawNormMin) stats_.rawNormMin = n;
            if (n > stats_.rawNormMax) stats_.rawNormMax = n;
        }
        stats_.rawNormSum += static_cast<double>(n);
    }

    void pushBodyNorm(float n) {
        if (!tracker::isFinite(n)) return;
        if (stats_.processedSamples == 0) {
            stats_.bodyNormMin = n;
            stats_.bodyNormMax = n;
        } else {
            if (n < stats_.bodyNormMin) stats_.bodyNormMin = n;
            if (n > stats_.bodyNormMax) stats_.bodyNormMax = n;
        }
        stats_.bodyNormSum += static_cast<double>(n);
    }

    void pushTrustedBodyNorm(float n) {
        if (!tracker::isFinite(n)) return;
        if (stats_.trustedSamples == 1) {
            stats_.trustedBodyNormMin = n;
            stats_.trustedBodyNormMax = n;
        } else {
            if (n < stats_.trustedBodyNormMin) stats_.trustedBodyNormMin = n;
            if (n > stats_.trustedBodyNormMax) stats_.trustedBodyNormMax = n;
        }
        stats_.trustedBodyNormSum += static_cast<double>(n);
    }

    MagProcessedSample last_;
    MagRuntimeStats stats_;
};

} // namespace tracker