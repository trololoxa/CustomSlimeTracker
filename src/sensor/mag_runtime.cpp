#include "sensor/mag_runtime.hpp"

#include "sensor/frame_transform.hpp"

namespace tracker {

float MagRuntimeStats::rawNormMean() const {
    return rawSamples > 0 ? static_cast<float>(rawNormSum / static_cast<double>(rawSamples)) : 0.0f;
}

float MagRuntimeStats::bodyNormMean() const {
    return processedSamples > 0 ? static_cast<float>(bodyNormSum / static_cast<double>(processedSamples)) : 0.0f;
}

float MagRuntimeStats::trustedBodyNormMean() const {
    return trustedSamples > 0 ? static_cast<float>(trustedBodyNormSum / static_cast<double>(trustedSamples)) : 0.0f;
}

void MagRuntimeProcessor::reset() {
    last_ = MagProcessedSample{};
    stats_ = MagRuntimeStats{};
}

const MagProcessedSample& MagRuntimeProcessor::last() const { return last_; }
const MagRuntimeStats& MagRuntimeProcessor::stats() const { return stats_; }

bool MagRuntimeProcessor::process(const Lsm6dsvFifoReader::MagRawSample& raw,
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

    // magToImu produces the native IMU sensor frame.  Keep magnetometer,
    // gyro and accelerometer in one device frame before heading estimation.
    const SensorToDeviceFrame frame = makeSensorToDeviceFrame(
        cfg.sensorToDeviceValid,
        cfg.sensorToDevice
    );
    out.body = frame.apply(out.body);

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

bool MagRuntimeProcessor::process(const Lsm6dsvFifoReader::MagRawSample& raw,
                                  const MagRuntimeConfig& cfg,
                                  uint32_t nowMs) {
    MagProcessedSample out;
    return process(raw, cfg, nowMs, out);
}

uint32_t MagRuntimeProcessor::ageMsForUse(const MagProcessedSample& sample, uint32_t nowMs) {
    if (!sample.valid) {
        return 0xFFFFFFFFUL;
    }
    // receivedMs is a wrapping uint32_t clock, not an invalid-value sentinel.
    // A valid sample can legitimately be stamped at boot or exactly at wrap.
    return nowMs - sample.receivedMs;
}

uint32_t MagRuntimeProcessor::rejectFlagsForUse(const MagProcessedSample& sample,
                                                const MagRuntimeConfig& cfg,
                                                uint32_t nowMs) {
    uint32_t flags = sample.rejectFlags;
    const uint32_t ageMs = ageMsForUse(sample, nowMs);
    if (ageMs > cfg.maxSampleAgeMs) {
        flags |= MAG_REJECT_STALE;
    }
    return flags;
}

bool MagRuntimeProcessor::trustedForUse(const MagProcessedSample& sample,
                                        const MagRuntimeConfig& cfg,
                                        uint32_t nowMs) {
    if (!sample.valid) return false;
    return rejectFlagsForUse(sample, cfg, nowMs) == MAG_REJECT_NONE;
}

const char* MagRuntimeProcessor::rejectFlagName(uint32_t singleFlag) {
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

void MagRuntimeProcessor::addReject(MagProcessedSample& out, uint32_t flag) {
    out.rejectFlags |= flag;
}

void MagRuntimeProcessor::countRejects(uint32_t flags) {
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

void MagRuntimeProcessor::pushRawNorm(float n) {
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

void MagRuntimeProcessor::pushBodyNorm(float n) {
    if (!tracker::isFinite(n)) return;
    if (stats_.processedSamples == 1) {
        stats_.bodyNormMin = n;
        stats_.bodyNormMax = n;
    } else {
        if (n < stats_.bodyNormMin) stats_.bodyNormMin = n;
        if (n > stats_.bodyNormMax) stats_.bodyNormMax = n;
    }
    stats_.bodyNormSum += static_cast<double>(n);
}

void MagRuntimeProcessor::pushTrustedBodyNorm(float n) {
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

} // namespace tracker
