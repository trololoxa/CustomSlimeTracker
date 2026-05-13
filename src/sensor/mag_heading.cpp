#include "sensor/mag_heading.hpp"

#include <cmath>

namespace tracker {

void MagHeadingEstimator::reset() {
    last_ = MagHeadingSample{};
    stats_ = MagHeadingStats{};
}

const MagHeadingSample& MagHeadingEstimator::last() const { return last_; }
const MagHeadingStats& MagHeadingEstimator::stats() const { return stats_; }

bool MagHeadingEstimator::update(const MagProcessedSample& mag,
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

bool MagHeadingEstimator::update(const MagProcessedSample& mag,
                                 const Quat& qWorldFromBody,
                                 const MagHeadingConfig& cfg,
                                 uint32_t nowMs) {
    MagHeadingSample out;
    return update(mag, qWorldFromBody, cfg, nowMs, out);
}

void MagHeadingEstimator::addReject(MagHeadingSample& out, uint32_t flag) {
    out.rejectFlags |= flag;
}

void MagHeadingEstimator::countRejects(uint32_t flags) {
    if (flags & MAG_HEADING_REJECT_MAG_INVALID)      stats_.rejectMagInvalid++;
    if (flags & MAG_HEADING_REJECT_MAG_NOT_TRUSTED)  stats_.rejectMagNotTrusted++;
    if (flags & MAG_HEADING_REJECT_QUAT_INVALID)     stats_.rejectQuatInvalid++;
    if (flags & MAG_HEADING_REJECT_WORLD_NONFINITE)  stats_.rejectWorldNonfinite++;
    if (flags & MAG_HEADING_REJECT_HORIZONTAL_SMALL) stats_.rejectHorizontalSmall++;
}

} // namespace tracker
