#pragma once

#include <cstdint>
#include <cmath>

#include "core/math.hpp"
#include "sensor/mag_runtime.hpp"

namespace tracker {

enum MagHeadingRejectFlags : uint32_t {
    MAG_HEADING_REJECT_NONE             = 0,
    MAG_HEADING_REJECT_MAG_INVALID      = 1u << 0,
    MAG_HEADING_REJECT_MAG_NOT_TRUSTED  = 1u << 1,
    MAG_HEADING_REJECT_QUAT_INVALID     = 1u << 2,
    MAG_HEADING_REJECT_WORLD_NONFINITE  = 1u << 3,
    MAG_HEADING_REJECT_HORIZONTAL_SMALL = 1u << 4,
};

struct MagHeadingConfig {
    bool requireTrustedMag = true;
    float minHorizontalNorm = 1.0e-6f;
};

struct MagHeadingSample {
    bool valid = false;
    uint32_t rejectFlags = MAG_HEADING_REJECT_NONE;

    uint64_t magTimestampUs = 0;
    uint32_t magReceivedMs = 0;
    uint32_t magSeq = 0;

    Quat qWorldFromBody = Quat::identity();

    Vec3 magBody = Vec3::zero();
    Vec3 magWorld = Vec3::zero();
    Vec3 magWorldHorizontal = Vec3::zero();

    float magBodyNorm = 0.0f;
    float magWorldNorm = 0.0f;
    float horizontalNorm = 0.0f;

    // Direction of magnetic field in current AHRS world XY plane.
    // This is not yet a correction. It is diagnostic.
    float magneticFieldWorldYawRad = 0.0f;
    float magneticFieldWorldYawDeg = 0.0f;

    // Opposite direction. Depending on sensor convention and local field sign,
    // one of these two will be more convenient as "north-like" heading.
    float magneticNorthWorldYawRad = 0.0f;
    float magneticNorthWorldYawDeg = 0.0f;

    float currentAhrsYawRad = 0.0f;
    float currentAhrsYawDeg = 0.0f;

    // Difference between current AHRS yaw and magnetic-north-like direction.
    // This is only a diagnostic innovation candidate for future yaw correction.
    float yawInnovationRad = 0.0f;
    float yawInnovationDeg = 0.0f;
};

struct MagHeadingStats {
    uint32_t attempts = 0;
    uint32_t valid = 0;
    uint32_t rejected = 0;

    uint32_t rejectMagInvalid = 0;
    uint32_t rejectMagNotTrusted = 0;
    uint32_t rejectQuatInvalid = 0;
    uint32_t rejectWorldNonfinite = 0;
    uint32_t rejectHorizontalSmall = 0;

    uint32_t lastValidMs = 0;
    uint32_t lastAttemptMs = 0;
};

class MagHeadingEstimator {
public:
    void reset() {
        last_ = MagHeadingSample{};
        stats_ = MagHeadingStats{};
    }

    const MagHeadingSample& last() const { return last_; }
    const MagHeadingStats& stats() const { return stats_; }

    bool update(const MagProcessedSample& mag,
                const Quat& qWorldFromBody,
                const MagHeadingConfig& cfg,
                uint32_t nowMs,
                MagHeadingSample& out) {
        out = MagHeadingSample{};
        out.magTimestampUs = mag.t_us;
        out.magReceivedMs = mag.receivedMs;
        out.magSeq = mag.seq;
        out.qWorldFromBody = qWorldFromBody.normalized();
        out.magBody = mag.body;
        out.magBodyNorm = mag.bodyNorm;

        stats_.attempts++;
        stats_.lastAttemptMs = nowMs;

        if (!mag.valid) {
            addReject(out, MAG_HEADING_REJECT_MAG_INVALID);
        }

        if (cfg.requireTrustedMag && !mag.trusted) {
            addReject(out, MAG_HEADING_REJECT_MAG_NOT_TRUSTED);
        }

        if (!out.qWorldFromBody.isFinite()) {
            addReject(out, MAG_HEADING_REJECT_QUAT_INVALID);
        }

        if (!out.magBody.isFinite() || out.magBody.normSq() <= MATH_EPSILON) {
            addReject(out, MAG_HEADING_REJECT_MAG_INVALID);
        }

        if (out.rejectFlags == MAG_HEADING_REJECT_NONE) {
            out.magWorld = out.qWorldFromBody.rotate(out.magBody);
            out.magWorldNorm = out.magWorld.norm();

            if (!out.magWorld.isFinite() || !tracker::isFinite(out.magWorldNorm)) {
                addReject(out, MAG_HEADING_REJECT_WORLD_NONFINITE);
            }
        }

        if (out.rejectFlags == MAG_HEADING_REJECT_NONE) {
            out.magWorldHorizontal = Vec3(out.magWorld.x, out.magWorld.y, 0.0f);
            out.horizontalNorm = out.magWorldHorizontal.norm();

            if (!tracker::isFinite(out.horizontalNorm) || out.horizontalNorm < cfg.minHorizontalNorm) {
                addReject(out, MAG_HEADING_REJECT_HORIZONTAL_SMALL);
            }
        }

        if (out.rejectFlags == MAG_HEADING_REJECT_NONE) {
            out.magneticFieldWorldYawRad = std::atan2(out.magWorldHorizontal.y, out.magWorldHorizontal.x);
            out.magneticFieldWorldYawDeg = out.magneticFieldWorldYawRad * MATH_RAD_TO_DEG;

            out.magneticNorthWorldYawRad = wrapPi(out.magneticFieldWorldYawRad + MATH_PI);
            out.magneticNorthWorldYawDeg = out.magneticNorthWorldYawRad * MATH_RAD_TO_DEG;

            const Vec3 euler = out.qWorldFromBody.toEulerXYZ();
            out.currentAhrsYawRad = euler.z;
            out.currentAhrsYawDeg = euler.z * MATH_RAD_TO_DEG;

            out.yawInnovationRad = wrapPi(out.magneticNorthWorldYawRad - out.currentAhrsYawRad);
            out.yawInnovationDeg = out.yawInnovationRad * MATH_RAD_TO_DEG;

            out.valid = true;
            stats_.valid++;
            stats_.lastValidMs = nowMs;
        } else {
            out.valid = false;
            stats_.rejected++;
            countRejects(out.rejectFlags);
        }

        last_ = out;
        return out.valid;
    }

    bool update(const MagProcessedSample& mag,
                const Quat& qWorldFromBody,
                const MagHeadingConfig& cfg,
                uint32_t nowMs) {
        MagHeadingSample out;
        return update(mag, qWorldFromBody, cfg, nowMs, out);
    }

private:
    void addReject(MagHeadingSample& out, uint32_t flag) {
        out.rejectFlags |= flag;
    }

    void countRejects(uint32_t flags) {
        if (flags & MAG_HEADING_REJECT_MAG_INVALID)      stats_.rejectMagInvalid++;
        if (flags & MAG_HEADING_REJECT_MAG_NOT_TRUSTED)  stats_.rejectMagNotTrusted++;
        if (flags & MAG_HEADING_REJECT_QUAT_INVALID)     stats_.rejectQuatInvalid++;
        if (flags & MAG_HEADING_REJECT_WORLD_NONFINITE)  stats_.rejectWorldNonfinite++;
        if (flags & MAG_HEADING_REJECT_HORIZONTAL_SMALL) stats_.rejectHorizontalSmall++;
    }

    MagHeadingSample last_;
    MagHeadingStats stats_;
};

} // namespace tracker