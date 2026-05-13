#pragma once

#include <cstdint>

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

    float magneticFieldWorldYawRad = 0.0f;
    float magneticFieldWorldYawDeg = 0.0f;

    float magneticNorthWorldYawRad = 0.0f;
    float magneticNorthWorldYawDeg = 0.0f;

    float currentAhrsYawRad = 0.0f;
    float currentAhrsYawDeg = 0.0f;

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
    void reset();

    const MagHeadingSample& last() const;
    const MagHeadingStats& stats() const;

    bool update(const MagProcessedSample& mag,
                const Quat& qWorldFromBody,
                const MagHeadingConfig& cfg,
                uint32_t nowMs,
                MagHeadingSample& out);

    bool update(const MagProcessedSample& mag,
                const Quat& qWorldFromBody,
                const MagHeadingConfig& cfg,
                uint32_t nowMs);

private:
    void addReject(MagHeadingSample& out, uint32_t flag);
    void countRejects(uint32_t flags);

    MagHeadingSample last_;
    MagHeadingStats stats_;
};

} // namespace tracker
