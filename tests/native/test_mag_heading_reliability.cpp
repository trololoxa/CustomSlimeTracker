#include "test_common.hpp"

#include <initializer_list>
#include <limits>

#include "sensor/frame_transform.hpp"
#include "sensor/mag_axis_alignment.hpp"
#include "sensor/mag_field_reliability.hpp"
#include "sensor/mag_yaw_correction.hpp"

using namespace tracker;

static MagFieldReliabilityInput makeFieldInput(
    uint32_t ms, float yawDeg, float dipDeg, float norm, float ahrsYawDeg = 0.0f) {
    MagFieldReliabilityInput in;
    in.nowMs = ms;
    in.processorTrustedForUse = true;
    in.gyroNormDps = 0.2f;
    in.accelTrust = 1.0f;
    in.mag.valid = true;
    in.mag.trusted = true;
    in.mag.bodyNorm = norm;
    in.mag.seq = ms + 1u;
    in.mag.receivedMs = ms;
    in.mag.t_us = static_cast<uint64_t>(ms) * 1000ULL;
    in.heading.valid = true;
    in.heading.dipDeg = dipDeg;
    in.heading.horizontalNorm = magHorizontalNormFromField(norm, dipDeg);
    in.heading.magneticNorthWorldYawRad = yawDeg * MATH_DEG_TO_RAD;
    in.heading.currentAhrsYawRad = ahrsYawDeg * MATH_DEG_TO_RAD;
    in.heading.yawInnovationRad = wrapPi(
        in.heading.magneticNorthWorldYawRad - in.heading.currentAhrsYawRad);
    return in;
}

static void testFieldReliabilityFailsClosedAcrossChangedEnvironment(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 1000;
    cfg.recoverStableMs = 1000;
    cfg.suspectClearMs = 300;
    cfg.newEnvironmentStableMs = 400;
    cfg.referenceReturnStableMs = 300;

    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(1101, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK(ctx, out.referenceValid);

    monitor.update(makeFieldInput(1201, 35.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
    CHECK(ctx, !out.trustedForYaw);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_HEADING_STEP_HARD) != 0u);

    // A stable but directionally different field is not silently accepted as a
    // replacement environment while the old absolute heading reference exists.
    monitor.update(makeFieldInput(1301, 35.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(1801, 35.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(2901, 35.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
    CHECK(ctx, !out.trustedForYaw);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_ENVIRONMENT_CHANGED) != 0u);
    CHECK(ctx, monitor.stats().environmentChangesDetected == 1u);

    // Returning to the original environment enters a fresh recovery dwell.
    monitor.update(makeFieldInput(3001, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
    monitor.update(makeFieldInput(3402, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Recovering);
    CHECK(ctx, !out.trustedForYaw);
    monitor.update(makeFieldInput(4503, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);

    // An explicit reference reset is the only path that can accept a genuinely
    // new environment; it averages a new acquisition window rather than one sample.
    monitor.restartAcquisition();
    monitor.update(makeFieldInput(5000, 35.1f, 54.8f, 502.0f), cfg, out);
    monitor.update(makeFieldInput(6101, 34.9f, 55.2f, 498.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK_NEAR(ctx, out.referenceHeadingYawDeg, 35.0f, 0.2f);
    CHECK_NEAR(ctx, out.referenceNorm, 500.0f, 0.5f);
}


static void testHighDipHealthyFieldUsesAdaptiveHorizontalTrust(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500u;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;

    constexpr float kNorm = 443.87f;
    constexpr float kDipDeg = -70.45f;
    monitor.update(makeFieldInput(1u, 0.0f, kDipDeg, kNorm), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, kDipDeg, kNorm), cfg, out);
    // One post-acquisition sample evaluates observability against the newly
    // established reference rather than the conservative bootstrap limits.
    monitor.update(makeFieldInput(618u, 0.0f, kDipDeg, kNorm), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK(ctx, out.horizontalNorm > 145.0f && out.horizontalNorm < 152.0f);
    CHECK(ctx, out.horizontalEffectiveBad < out.horizontalNorm);
    CHECK(ctx, out.horizontalEffectiveGood < out.horizontalNorm);
    CHECK_NEAR(ctx, out.horizontalTrust, 1.0f, 1.0e-5f);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_HORIZONTAL_UNOBSERVABLE) == 0u);
    CHECK(ctx, magHeadingNoiseScale(out.headingNoiseScaleSquared) > 1.0f);

    // A four-degree one-shot wobble is below the inclination-scaled
    // discontinuity threshold and must not repeatedly poison the field state.
    monitor.update(makeFieldInput(635u, 4.0f, kDipDeg, kNorm), cfg, out);
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);

    // A real larger field jump remains fail-closed.
    monitor.update(makeFieldInput(652u, 10.0f, kDipDeg, kNorm), cfg, out);
    CHECK(ctx, out.stationaryHeadingJumpLatched);
    CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
}

static void testHighDipLegacyHeadingStepGateScalesWithObservability(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500u;
    // Isolate the legacy per-sample soft/hard gate from the newer stationary
    // discontinuity latch. The nine-degree step is above the historical fixed
    // 8-degree soft threshold but below the high-dip scaled threshold.
    cfg.stationaryHeadingJumpMinDeg = 20.0f;
    cfg.stationaryHeadingJumpRateDegS = 100.0f;

    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    constexpr float kNorm = 443.87f;
    constexpr float kDipDeg = -70.45f;
    monitor.update(makeFieldInput(1u, 0.0f, kDipDeg, kNorm), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, kDipDeg, kNorm), cfg, out);
    monitor.update(makeFieldInput(618u, 0.0f, kDipDeg, kNorm), cfg, out);
    CHECK(ctx, out.trustedForYaw);
    const float headingScale = magHeadingNoiseScale(out.headingNoiseScaleSquared);
    CHECK(ctx, cfg.headingStepSoftDeg * headingScale > cfg.headingStepSoftDeg);
    CHECK(ctx, cfg.headingStepHardDeg * headingScale > cfg.headingStepHardDeg);

    monitor.update(makeFieldInput(635u, 9.0f, kDipDeg, kNorm), cfg, out);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_HEADING_STEP_SOFT) == 0u);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_HEADING_STEP_HARD) == 0u);
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
}

static void testUnobservableHorizontalHeadingSkipsDirectionalStepGate(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500u;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    constexpr float kNorm = 443.87f;
    constexpr float kDipDeg = -70.45f;
    monitor.update(makeFieldInput(1u, 0.0f, kDipDeg, kNorm), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, kDipDeg, kNorm), cfg, out);
    monitor.update(makeFieldInput(618u, 0.0f, kDipDeg, kNorm), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    auto unobservable = makeFieldInput(635u, 12.0f, kDipDeg, kNorm);
    unobservable.heading.horizontalNorm = 15.0f;
    monitor.update(unobservable, cfg, out);
    CHECK(ctx, out.horizontalTrust == 0.0f);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_HORIZONTAL_UNOBSERVABLE) != 0u);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_HEADING_STEP_SOFT) == 0u);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_HEADING_STEP_HARD) == 0u);
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    CHECK(ctx, !out.trustedForYaw);
}

static void testNearVerticalFieldRemainsFailClosedForYaw(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500u;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    constexpr float kNorm = 443.87f;
    constexpr float kDipDeg = -88.0f; // horizontal component ~15.5, below absolute floor
    monitor.update(makeFieldInput(1u, 0.0f, kDipDeg, kNorm), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, kDipDeg, kNorm), cfg, out);
    monitor.update(makeFieldInput(618u, 0.0f, kDipDeg, kNorm), cfg, out);
    CHECK(ctx, out.referenceValid);
    CHECK(ctx, out.horizontalEffectiveBad >= cfg.horizontalTrust.absoluteBadFloor);
    CHECK(ctx, out.horizontalTrust == 0.0f);
    CHECK(ctx, !out.trustedForYaw);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_HORIZONTAL_UNOBSERVABLE) != 0u);

    // Directional noise from an effectively vertical field is not evidence of
    // an environmental heading jump; yaw simply remains unavailable.
    monitor.update(makeFieldInput(635u, 25.0f, kDipDeg, kNorm), cfg, out);
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_HEADING_STEP_HARD) == 0u);
    CHECK(ctx, !out.trustedForYaw);
}

static void testThirtyMinuteHighDipNoiseAndDisturbanceRecovery(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500u;
    cfg.referenceReturnStableMs = 300u;
    cfg.recoverStableMs = 500u;

    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    constexpr float kNorm = 443.87f;
    constexpr float kDipDeg = -70.45f;
    monitor.update(makeFieldInput(1u, 0.0f, kDipDeg, kNorm), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, kDipDeg, kNorm), cfg, out);
    monitor.update(makeFieldInput(618u, 0.0f, kDipDeg, kNorm), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    static constexpr float kJitterDeg[] = {-1.2f, 0.4f, 1.1f, -0.6f, 0.8f, -0.3f, 0.2f};
    uint32_t nowMs = 635u;
    float ahrsYawDeg = 0.0f;
    const uint32_t samples = (30u * 60u * 1000u) / 17u;
    for (uint32_t i = 0; i < samples; ++i) {
        ahrsYawDeg += 0.00017f; // ~0.01 deg/s 6DoF yaw drift
        const float localYawDeg = kJitterDeg[i % (sizeof(kJitterDeg) / sizeof(kJitterDeg[0]))];
        const float norm = kNorm * (1.0f + (static_cast<int>(i % 9u) - 4) * 0.0008f);
        const float dip = kDipDeg + (static_cast<int>(i % 7u) - 3) * 0.05f;
        monitor.update(makeFieldInput(nowMs, ahrsYawDeg + localYawDeg, dip, norm, ahrsYawDeg), cfg, out);
        nowMs += 17u;
    }
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    CHECK(ctx, monitor.stats().enteredDisturbed == 0u);

    // A real local-field direction jump still latches after the long clean run.
    monitor.update(makeFieldInput(nowMs, ahrsYawDeg + 12.0f, kDipDeg, kNorm, ahrsYawDeg), cfg, out);
    CHECK(ctx, out.stationaryHeadingJumpLatched);
    CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
    nowMs += 17u;

    // The original body-relative field returns while the tracker remains still.
    for (uint32_t i = 0; i < 24u; ++i) {
        monitor.update(makeFieldInput(nowMs, ahrsYawDeg, kDipDeg, kNorm, ahrsYawDeg), cfg, out);
        nowMs += 17u;
    }
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    CHECK(ctx, out.state == MagFieldReliabilityState::Recovering);
    for (uint32_t i = 0; i < 36u; ++i) {
        monitor.update(makeFieldInput(nowMs, ahrsYawDeg, kDipDeg, kNorm, ahrsYawDeg), cfg, out);
        nowMs += 17u;
    }
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
}

static void testYawGateUsesSameAdaptiveHorizontalTrust(TestContext& ctx) {
    MagYawCorrectionController controller;
    MagYawCorrectionConfig cfg;
    cfg.applyEnabled = true;

    MagYawCorrectionInput in;
    in.nowMs = 1000u;
    in.mag.valid = true;
    in.mag.trusted = true;
    in.mag.receivedMs = 1000u;
    in.mag.seq = 1u;
    in.heading.valid = true;
    in.heading.horizontalNorm = 148.5f;
    in.heading.magneticNorthWorldYawRad = 0.0f;
    in.referenceValid = true;
    in.referenceWorldYawRad = 0.0f;
    in.magTrustedForUse = true;
    in.gyroNormDps = 0.2f;
    in.accelTrust = 1.0f;
    in.fieldReliable = true;
    in.fieldStableMs = 10000u;
    const MagHorizontalTrustResult horizontal = evaluateMagHorizontalTrust(
        in.heading.horizontalNorm, cfg.horizontalNormBad, cfg.horizontalNormGood,
        443.87f, -70.45f, MagHorizontalTrustConfig{});
    in.horizontalTrustValid = horizontal.valid;
    in.horizontalTrust = horizontal.trust;
    in.horizontalReferenceNorm = magHorizontalNormFromField(443.87f, -70.45f);
    in.horizontalEffectiveBad = horizontal.effectiveBad;
    in.horizontalEffectiveGood = horizontal.effectiveGood;

    MagYawCorrectionOutput out;
    CHECK(ctx, controller.update(in, cfg, out));
    CHECK(ctx, out.gateOpen);
    CHECK(ctx, out.applyAllowed);
    CHECK(ctx, (out.rejectFlags & MAG_YAW_REJECT_HORIZONTAL_BAD) == 0u);
    CHECK_NEAR(ctx, out.horizontalTrust, 1.0f, 1.0e-5f);
    CHECK(ctx, out.horizontalEffectiveGood < out.horizontalNorm);
}

static void testModerateStationaryHeadingJumpStaysFailClosed(TestContext& ctx) {
    for (float shiftDeg : {5.0f, 9.0f, 12.0f, 19.0f}) {
        MagFieldReliabilityConfig cfg;
        cfg.acquireStableMs = 500;
        cfg.referenceReturnStableMs = 200;
        cfg.recoverStableMs = 300;

        MagFieldReliabilityMonitor monitor;
        MagFieldReliabilityOutput out;
        monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
        monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
        CHECK(ctx, out.trustedForYaw);

        monitor.update(makeFieldInput(618u, shiftDeg, 55.0f, 500.0f), cfg, out);
        CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
        CHECK(ctx, !out.trustedForYaw);
        CHECK(ctx, out.stationaryHeadingJumpLatched);
        CHECK(ctx, (out.flags & MAG_FIELD_FLAG_STATIONARY_HEADING_JUMP) != 0u);

        for (uint32_t i = 0; i < 180u; ++i) {
            monitor.update(makeFieldInput(635u + i * 17u, shiftDeg, 55.0f, 500.0f), cfg, out);
        }
        CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
        CHECK(ctx, !out.trustedForYaw);
        CHECK(ctx, out.stationaryHeadingJumpLatched);
    }
}


static void testAhrsYawFrameJumpDoesNotMasqueradeAsFieldJump(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f, 0.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f, 0.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    // An AHRS yaw correction/reset rotates magnetic north in the chosen world
    // frame by the same amount, while the actual body-relative field is unchanged.
    monitor.update(makeFieldInput(618u, 12.0f, 55.0f, 500.0f, 12.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    CHECK_NEAR(ctx, out.headingStepDeg, 0.0f, 1.0e-4f);
}

static void testStationaryFieldReturnRecoversAfterAhrsYawDrift(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    cfg.referenceReturnStableMs = 300;
    cfg.recoverStableMs = 500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f, 0.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f, 0.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    // A real same-norm local field shift latches.
    monitor.update(makeFieldInput(618u, 9.0f, 55.0f, 500.0f, 0.0f), cfg, out);
    CHECK(ctx, out.stationaryHeadingJumpLatched);
    CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);

    // While correction is fail-closed, ordinary 6DoF yaw drift changes the world
    // heading but leaves the stationary body-relative field signal unchanged.
    monitor.update(makeFieldInput(2000u, 14.0f, 55.0f, 500.0f, 5.0f), cfg, out);
    CHECK(ctx, out.stationaryHeadingJumpLatched);

    // Remove the disturbance. World heading is now five degrees from the old
    // reference, so the old implementation could never recover; the invariant
    // stationary field has returned exactly to its pre-jump baseline.
    monitor.update(makeFieldInput(2100u, 5.0f, 55.0f, 500.0f, 5.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
    CHECK(ctx, out.stationaryLatchReferenceErrorDeg <= cfg.referenceReturnDeg);
    monitor.update(makeFieldInput(2501u, 5.0f, 55.0f, 500.0f, 5.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Recovering);
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    monitor.update(makeFieldInput(3102u, 5.0f, 55.0f, 500.0f, 5.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK(ctx, monitor.stats().stationaryHeadingReturnsViaStationaryField == 1u);
}

static void testStationaryFieldShortcutDisablesAfterPhysicalMotion(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    cfg.referenceReturnStableMs = 200;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(618u, 9.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.stationaryHeadingJumpLatched);

    MagFieldReliabilityInput moving = makeFieldInput(700u, 9.0f, 55.0f, 500.0f);
    moving.gyroNormDps = 20.0f;
    monitor.update(moving, cfg, out);
    CHECK(ctx, out.stationaryLatchMotionSeen);

    // Matching the old body-relative value is no longer sufficient after motion;
    // only the absolute world-field reference may clear the latch.
    monitor.update(makeFieldInput(900u, 5.0f, 55.0f, 500.0f, 5.0f), cfg, out);
    monitor.update(makeFieldInput(1201u, 5.0f, 55.0f, 500.0f, 5.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
    CHECK(ctx, out.stationaryHeadingJumpLatched);
}

static void testStationaryJumpAtWindowBoundaryStillLatches(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    cfg.stationaryHeadingWindowMs = 1500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    // Refresh the rolling anchor, then place the jump just beyond its nominal
    // window boundary. Adjacent-sample detection must cover this boundary.
    monitor.update(makeFieldInput(2202u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(2219u, 9.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.stationaryHeadingJumpLatched);
    CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
    CHECK(ctx, !out.trustedForYaw);
}

static void testSlowStationaryYawDriftRemainsCorrectable(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    uint32_t nowMs = 618u;
    float yawDeg = 0.0f;
    for (uint32_t i = 0; i < 360u; ++i) {
        yawDeg += 1.0f * 0.017f; // 1 deg/s: faster than normal gyro drift, below discontinuity gate.
        monitor.update(makeFieldInput(nowMs, yawDeg, 55.0f, 500.0f), cfg, out);
        nowMs += 17u;
    }
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    CHECK(ctx, out.referenceHeadingErrorDeg > 5.0f);
}

static void testFastButPlausibleYawDriftDoesNotSelfLatch(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    uint32_t nowMs = 618u;
    float yawDeg = 0.0f;
    for (uint32_t i = 0; i < 240u; ++i) {
        yawDeg += 4.0f * 0.017f;
        monitor.update(makeFieldInput(nowMs, yawDeg, 55.0f, 500.0f), cfg, out);
        nowMs += 17u;
    }
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
}

static void testMaximumNormalYawCorrectionDoesNotSelfLatch(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    uint32_t nowMs = 618u;
    float yawDeg = 0.0f;
    for (uint32_t i = 0; i < 360u; ++i) {
        yawDeg += 2.0f * 0.017f; // Current maximum normal yaw-correction rate.
        monitor.update(makeFieldInput(nowMs, yawDeg, 55.0f, 500.0f), cfg, out);
        nowMs += 17u;
    }
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
}

static void testRealRotationDoesNotTriggerStationaryJumpLatch(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    uint32_t nowMs = 618u;
    float yawDeg = 0.0f;
    for (uint32_t i = 0; i < 120u; ++i) {
        yawDeg += 30.0f * 0.017f;
        auto in = makeFieldInput(nowMs, yawDeg, 55.0f, 500.0f);
        in.gyroNormDps = 30.0f;
        monitor.update(in, cfg, out);
        nowMs += 17u;
    }
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_STATIONARY_HEADING_JUMP) == 0u);
}

static void testVerySlowPhysicalRotationDoesNotTriggerJumpLatch(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    uint32_t nowMs = 618u;
    float yawDeg = 0.0f;
    for (uint32_t i = 0; i < 240u; ++i) {
        yawDeg += 3.0f * 0.017f;
        auto in = makeFieldInput(nowMs, yawDeg, 55.0f, 500.0f);
        in.gyroNormDps = 3.0f;
        monitor.update(in, cfg, out);
        nowMs += 17u;
    }
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_STATIONARY_HEADING_JUMP) == 0u);
}

static void testFilteredHeadingRateAllowsNoisySixtyHertzReacquisition(TestContext& ctx) {
    MagFieldReliabilityConfig fieldCfg;
    fieldCfg.acquireStableMs = 1000;
    fieldCfg.headingRateFilterTimeConstantS = 0.75f;

    MagYawCorrectionConfig yawCfg;
    yawCfg.applyEnabled = true;
    yawCfg.maxInnovationDeg = 25.0f;
    yawCfg.reacquireMinFieldStableMs = 8000;
    yawCfg.reacquireMaxHeadingRateDegS = 2.0f;

    MagFieldReliabilityMonitor monitor;
    MagYawCorrectionController yaw;
    MagFieldReliabilityOutput fieldOut;
    MagYawCorrectionOutput yawOut;
    bool reacquired = false;
    float maxFilteredAfterSettling = 0.0f;

    uint32_t nowMs = 1u;
    for (uint32_t i = 0; i < 780u; ++i) {
        const float jitter = (i & 1u) ? 0.05f : -0.05f;
        const float headingDeg = 40.0f + jitter;
        const auto fieldIn = makeFieldInput(nowMs, headingDeg, 55.0f, 500.0f);
        monitor.update(fieldIn, fieldCfg, fieldOut);
        if (nowMs > 3000u) {
            maxFilteredAfterSettling = std::max(maxFilteredAfterSettling, fieldOut.headingRateDegS);
        }

        MagYawCorrectionInput yi;
        yi.referenceValid = true;
        yi.referenceWorldYawRad = 0.0f;
        yi.mag = fieldIn.mag;
        yi.heading = fieldIn.heading;
        yi.heading.horizontalNorm = 300.0f;
        yi.magTrustedForUse = true;
        yi.magRejectFlagsForUse = MAG_REJECT_NONE;
        yi.gyroNormDps = 0.2f;
        yi.accelTrust = 1.0f;
        yi.fieldReliable = fieldOut.trustedForYaw;
        yi.fieldStableMs = fieldOut.stableMs;
        yi.magneticHeadingRateDegS = fieldOut.headingRateDegS;
        yi.nowMs = nowMs;
        yaw.update(yi, yawCfg, yawOut);
        reacquired = reacquired || yawOut.reacquireActive;
        nowMs += 17u;
    }

    CHECK(ctx, fieldOut.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, fieldOut.trustedForYaw);
    CHECK(ctx, maxFilteredAfterSettling < yawCfg.reacquireMaxHeadingRateDegS);
    CHECK(ctx, reacquired);
    CHECK(ctx, yaw.stats().reacquireGateOpenCount >= 1u);
}


static void testIntervalBuilderUsesTimestampCoherentGyroEndpoints(TestContext& ctx) {
    MagAxisAlignmentInterval interval;
    MagAxisIntervalBuildFailure failure = MagAxisIntervalBuildFailure::InvalidInput;
    const Vec3 gyro0(1.0f, 0.0f, 0.0f);
    const Vec3 gyro1(0.0f, 1.0f, 0.0f);
    CHECK(ctx, buildMagAxisAlignmentInterval(
        gyro0, gyro1,
        Vec3(1.0f, 0.0f, 0.0f), Vec3(0.99f, -0.01f, 0.0f),
        0.02f, 7u, interval, &failure));
    CHECK(ctx, failure == MagAxisIntervalBuildFailure::None);
    CHECK_NEAR(ctx, interval.gyroSensorRadS.x, 0.5f, 1.0e-6f);
    CHECK_NEAR(ctx, interval.gyroSensorRadS.y, 0.5f, 1.0e-6f);
    CHECK_NEAR(ctx, interval.gyroSensorRadS.z, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, interval.dtS, 0.02f, 1.0e-7f);
    CHECK(ctx, interval.windowId == 7u);

    CHECK(ctx, !buildMagAxisAlignmentInterval(
        gyro0, gyro1, Vec3(1,0,0), Vec3(0,1,0),
        0.001f, 0u, interval, &failure));
    CHECK(ctx, failure == MagAxisIntervalBuildFailure::InvalidTiming);

    CHECK(ctx, !buildMagAxisAlignmentInterval(
        Vec3::zero(), Vec3::zero(), Vec3(1,0,0), Vec3(0,1,0),
        0.02f, 0u, interval, &failure));
    CHECK(ctx, failure == MagAxisIntervalBuildFailure::MotionOutOfRange);

    CHECK(ctx, !buildMagAxisAlignmentInterval(
        Vec3(std::numeric_limits<float>::quiet_NaN(), 0, 0), gyro1,
        Vec3(1,0,0), Vec3(0,1,0), 0.02f, 0u, interval, &failure));
    CHECK(ctx, failure == MagAxisIntervalBuildFailure::InvalidInput);
}

static void makeAlignmentDatasetForTransform(
    const Mat3& magToImu,
    MagAxisAlignmentInterval (&intervals)[64]) {
    const Mat3 magFromImu = magToImu.transposed();
    Vec3 mImu = Vec3(0.45f, 0.20f, 0.87f).normalized();
    for (uint16_t i = 0; i < 64u; ++i) {
        const Vec3 axis = (i % 6u == 0u) ? Vec3(1.0f, 0.20f, 0.10f).normalized()
                        : (i % 6u == 1u) ? Vec3(0.10f, 1.0f, 0.30f).normalized()
                        : (i % 6u == 2u) ? Vec3(0.25f, -0.10f, 1.0f).normalized()
                        : (i % 6u == 3u) ? Vec3(-0.70f, 0.50f, 0.50f).normalized()
                        : (i % 6u == 4u) ? Vec3(0.55f, 0.70f, -0.35f).normalized()
                                         : Vec3(-0.40f, -0.25f, 0.88f).normalized();
        const Vec3 gyro = axis * ((70.0f + static_cast<float>(i % 5u) * 8.0f) * MATH_DEG_TO_RAD);
        const float dt = 0.025f;
        const Vec3 nextImu = Quat::fromRotationVector(gyro * (-dt)).rotate(mImu).normalized();
        intervals[i] = MagAxisAlignmentInterval{
            gyro,
            magFromImu * mImu,
            magFromImu * nextImu,
            dt,
            static_cast<uint16_t>(i / 8u)};
        mImu = nextImu;
    }
}

static void testAllProperSignedPermutationMountingsConverge(TestContext& ctx) {
    static constexpr uint8_t permutations[6][3] = {
        {0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}
    };
    const Mat3 residual = Quat::fromEulerXYZ(
        1.3f * MATH_DEG_TO_RAD,
       -0.9f * MATH_DEG_TO_RAD,
        1.7f * MATH_DEG_TO_RAD).toRotationMatrix();
    uint32_t solved = 0u;
    for (const auto& axes : permutations) {
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                for (int sz = -1; sz <= 1; sz += 2) {
                    const Mat3 coarse = MagAxisAlignmentCollector::signedPermutation(
                        axes[0], static_cast<float>(sx),
                        axes[1], static_cast<float>(sy),
                        axes[2], static_cast<float>(sz));
                    if (!MagAxisAlignmentCollector::properRotation(coarse)) continue;
                    const Mat3 expected = residual * coarse;
                    MagAxisAlignmentInterval intervals[64];
                    makeAlignmentDatasetForTransform(expected, intervals);
                    MagAxisAlignmentSolvePolicy policy;
                    MagAxisAlignmentResult result;
                    CHECK(ctx, solveMagAxisAlignmentDataset(
                        intervals, 64u, Vec3::zero(), Mat3::identity(),
                        nullptr, policy, result));
                    CHECK(ctx, result.valid);
                    CHECK(ctx, result.validationPassed);
                    CHECK(ctx, result.validationWinnerMatchesTraining);
                    CHECK(ctx, MagAxisAlignmentCollector::properRotation(result.magToImu));
                    CHECK(ctx, magAxisRotationDifferenceDeg(result.magToImu, expected) < 1.0f);
                    solved++;
                }
            }
        }
    }
    CHECK(ctx, solved == 24u);
}

static void testReflectionAmbiguityRequiresRightHandedDriverContract(TestContext& ctx) {
    const Mat3 reflected(-1,0,0, 0,1,0, 0,0,1);
    MagAxisAlignmentInterval intervals[64];
    makeAlignmentDatasetForTransform(reflected, intervals);
    MagAxisAlignmentSolvePolicy policy;
    MagAxisAlignmentResult result;
    CHECK(ctx, solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), Mat3::identity(),
        nullptr, policy, result));
    CHECK(ctx, result.valid);
    CHECK(ctx, !MagAxisAlignmentCollector::properRotation(reflected));

    // Dynamic vector kinematics cannot distinguish F from -F because both m
    // and -m obey the same dm/dt equation. If a driver silently emits a
    // left-handed frame F, the SO(3)-only solver finds -F, a proper rotation
    // with globally inverted magnetic polarity. Therefore reflections must be
    // corrected in the driver; calibration must never persist det=-1.
    const Mat3 polarityEquivalent = reflected * -1.0f;
    CHECK(ctx, MagAxisAlignmentCollector::properRotation(polarityEquivalent));
    CHECK(ctx, magAxisRotationDifferenceDeg(result.magToImu, polarityEquivalent) < 1.0f);
    CHECK(ctx, MagAxisAlignmentCollector::properRotation(result.magToImu));
}

static void testOnlyProperSignedPermutationsAreAccepted(TestContext& ctx) {
    static constexpr uint8_t perms[6][3] = {
        {0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}
    };
    int proper = 0;
    for (const auto& p : perms) {
        for (int sx=-1; sx<=1; sx+=2) for (int sy=-1; sy<=1; sy+=2) for (int sz=-1; sz<=1; sz+=2) {
            const Mat3 m = MagAxisAlignmentCollector::signedPermutation(
                p[0], static_cast<float>(sx), p[1], static_cast<float>(sy), p[2], static_cast<float>(sz));
            if (MagAxisAlignmentCollector::properRotation(m)) proper++;
        }
    }
    CHECK(ctx, proper == 24);
    CHECK(ctx, !MagAxisAlignmentCollector::properRotation(Mat3(-1,0,0, 0,1,0, 0,0,1)));
}


static void testAxisCollectorRejectsStaleGyroPair(TestContext& ctx) {
    MagAxisAlignmentCollector collector;
    MagProcessedSample mag;
    mag.valid = mag.trusted = true;
    mag.raw = Vec3(1.0f, 0.0f, 0.0f);
    mag.calibratedMagFrame = mag.raw;
    mag.rawNorm = 1.0f;
    mag.calibratedNorm = 1.0f;
    mag.seq = 1u;
    mag.t_us = 100000u;
    mag.receivedMs = 100u;
    CHECK(ctx, !collector.observe(Vec3(0.0f, 0.0f, 1.0f), 90000u, mag, true));
    CHECK(ctx, collector.stats().intervalsRejectedGyroSkew == 1u);
    CHECK(ctx, collector.intervalCount() == 0u);
}


static void testSixtyHertzCollectionSpansIndependentWindows(TestContext& ctx) {
    MagAxisAlignmentCollector collector;
    Vec3 m = Vec3(0.45f, 0.2f, 0.87f).normalized();
    uint64_t tUs = 100000u;
    uint32_t ms = 100u;

    MagProcessedSample first;
    first.valid = first.trusted = true;
    first.raw = m;
    first.calibratedMagFrame = m;
    first.rawNorm = 1.0f;
    first.calibratedNorm = 1.0f;
    first.seq = 1u;
    first.t_us = tUs;
    first.receivedMs = ms;
    collector.observe(Vec3::zero(), tUs, first, true);

    for (uint32_t i = 0; i < 480u; ++i) {
        const Vec3 gyro = ((i / 60u) & 1u) == 0u ? Vec3(0.8f, 0.1f, 0.0f)
                                                  : Vec3(0.0f, 0.9f, 0.1f);
        const float dt = 0.017f;
        m = (m - cross(gyro, m) * dt).normalized();
        tUs += 17000u;
        ms += 17u;
        MagProcessedSample mag;
        mag.valid = mag.trusted = true;
        mag.raw = m;
        mag.calibratedMagFrame = m;
        mag.rawNorm = 1.0f;
        mag.calibratedNorm = 1.0f;
        mag.seq = i + 2u;
        mag.t_us = tUs;
        mag.receivedMs = ms;
        collector.observe(gyro, tUs, mag, true);
    }

    CHECK(ctx, collector.intervalCount() >= MagAxisAlignmentCollector::kTargetIntervals);
    CHECK(ctx, collector.independentWindows() >= 4u);
    CHECK(ctx, collector.readyToSolve());
    CHECK(ctx, collector.stats().intervalsSkippedCadence > 0u);
    CHECK(ctx, collector.stats().intervalsRejectedCapacity == 0u);
}

static void testRuntimeCollectorUsesSensorTimeAcrossProcessingBacklog(TestContext& ctx) {
    MagAxisAlignmentCollector collector;
    Vec3 m = Vec3(0.45f, 0.2f, 0.87f).normalized();
    uint64_t tUs = 100000u;
    constexpr uint32_t kCollapsedProcessingMs = 500u;

    MagProcessedSample first;
    first.valid = first.trusted = true;
    first.raw = m;
    first.calibratedMagFrame = m;
    first.rawNorm = 1.0f;
    first.calibratedNorm = 1.0f;
    first.seq = 1u;
    first.t_us = tUs;
    first.receivedMs = kCollapsedProcessingMs;
    collector.observe(Vec3::zero(), tUs, first, true);

    for (uint32_t i = 0; i < 480u; ++i) {
        const Vec3 gyro = ((i / 60u) & 1u) == 0u ? Vec3(0.8f, 0.1f, 0.0f)
                                                  : Vec3(0.0f, 0.9f, 0.1f);
        constexpr float dt = 0.017f;
        m = Quat::fromRotationVector(gyro * (-dt)).rotate(m).normalized();
        tUs += 17000u;
        MagProcessedSample mag;
        mag.valid = mag.trusted = true;
        mag.raw = m;
        mag.calibratedMagFrame = m;
        mag.rawNorm = 1.0f;
        mag.calibratedNorm = 1.0f;
        mag.seq = i + 2u;
        mag.t_us = tUs;
        // Model a queued FIFO burst drained inside one millisecond. Runtime
        // cadence/window evidence must remain tied to sensor time, not this
        // collapsed processing-time stamp.
        mag.receivedMs = kCollapsedProcessingMs;
        collector.observe(gyro, tUs, mag, true);
    }

    CHECK(ctx, collector.intervalCount() >= MagAxisAlignmentCollector::kTargetIntervals);
    CHECK(ctx, collector.independentWindows() >= 4u);
    CHECK(ctx, collector.readyToSolve());
    CHECK(ctx, collector.stats().intervalsSkippedCadence > 0u);
}

static void testDynamicAxisSolverRefinesMechanicalMisalignment(TestContext& ctx) {
    const Mat3 coarse(0,-1,0, 1,0,0, 0,0,1);
    const Mat3 residual = Quat::fromEulerXYZ(
        0.7f * MATH_DEG_TO_RAD,
       -1.1f * MATH_DEG_TO_RAD,
        1.6f * MATH_DEG_TO_RAD).toRotationMatrix();
    const Mat3 expected = residual * coarse;
    const Mat3 inverse = expected.transposed();

    MagAxisAlignmentInterval intervals[64];
    const Vec3 starts[4] = {
        Vec3(0.45f, 0.20f, 0.87f).normalized(),
        Vec3(-0.30f, 0.91f, 0.28f).normalized(),
        Vec3(0.82f, -0.44f, 0.36f).normalized(),
        Vec3(-0.70f, -0.18f, 0.69f).normalized(),
    };
    for (uint16_t i = 0; i < 64u; ++i) {
        const uint16_t local = static_cast<uint16_t>(i % 16u);
        Vec3 m0 = local == 0u ? starts[i / 16u]
                              : expected * intervals[i - 1u].mag1Raw;
        const Vec3 gyro = (i % 5u == 0u) ? Vec3(1.10f, 0.20f, -0.05f)
                         : (i % 5u == 1u) ? Vec3(-0.10f, 0.95f, 0.35f)
                         : (i % 5u == 2u) ? Vec3(0.30f, -0.15f, 1.05f)
                         : (i % 5u == 3u) ? Vec3(0.75f, 0.65f, -0.25f)
                                          : Vec3(-0.55f, 0.40f, 0.85f);
        const float dt = 0.050f;
        const Vec3 m1 = Quat::fromRotationVector(gyro * (-dt)).rotate(m0).normalized();
        intervals[i].gyroSensorRadS = gyro;
        intervals[i].mag0Raw = inverse * m0;
        intervals[i].mag1Raw = inverse * m1;
        intervals[i].dtS = dt;
        intervals[i].windowId = static_cast<uint16_t>(i / 8u);
    }

    MagAxisAlignmentSolvePolicy policy;
    policy.minIntervals = 24u;
    policy.minTrainingIntervals = 12u;
    policy.minValidationIntervals = 12u;
    policy.minIndependentWindows = 4u;
    policy.minExcitedAxes = 2u;
    policy.minTrainingWindows = 2u;
    policy.minValidationWindows = 2u;
    MagAxisAlignmentResult result;
    CHECK(ctx, solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), Mat3::identity(),
        &coarse, policy, result));
    CHECK(ctx, result.valid);
    CHECK(ctx, result.refined);
    CHECK(ctx, result.activeCompared);
    CHECK(ctx, result.improvesActive);
    CHECK(ctx, result.validationPassed);
    CHECK(ctx, result.validationWinnerMatchesTraining);
    CHECK(ctx, MagAxisAlignmentCollector::properRotation(result.magToImu));
    CHECK(ctx, magAxisRotationDifferenceDeg(result.coarseMagToImu, coarse) < 0.1f);
    CHECK(ctx, magAxisRotationDifferenceDeg(result.magToImu, expected) < 0.65f);
    CHECK(ctx, result.refinementAngleDeg > 1.0f);
    CHECK(ctx, result.refinementAngleDeg < 4.0f);
    CHECK(ctx, result.score < result.coarseScore);
    CHECK(ctx, result.score < result.activeScore);
    CHECK(ctx, result.qualityScore > 0.62f);
}

static bool makeRateDataset(float rateDegS,
                            MagAxisAlignmentInterval (&intervals)[64],
                            Mat3& coarse,
                            Mat3& expected) {
    coarse = Mat3(0,-1,0, 1,0,0, 0,0,1);
    const Mat3 residual = Quat::fromEulerXYZ(
        0.7f * MATH_DEG_TO_RAD,
       -1.1f * MATH_DEG_TO_RAD,
        1.6f * MATH_DEG_TO_RAD).toRotationMatrix();
    expected = residual * coarse;
    const Mat3 inverse = expected.transposed();
    Vec3 m = Vec3(0.45f, 0.20f, 0.87f).normalized();
    const float rate = rateDegS * MATH_DEG_TO_RAD;
    for (uint16_t i = 0; i < 64u; ++i) {
        const Vec3 axis = (i % 4u == 0u) ? Vec3(1.0f, 0.2f, 0.1f).normalized()
                        : (i % 4u == 1u) ? Vec3(0.1f, 1.0f, 0.3f).normalized()
                        : (i % 4u == 2u) ? Vec3(0.25f, -0.1f, 1.0f).normalized()
                                         : Vec3(-0.7f, 0.5f, 0.5f).normalized();
        const Vec3 gyro = axis * rate;
        const float dt = 0.017f;
        const Vec3 next = Quat::fromRotationVector(gyro * (-dt)).rotate(m).normalized();
        intervals[i] = MagAxisAlignmentInterval{
            gyro, inverse * m, inverse * next, dt,
            static_cast<uint16_t>(i / 8u)};
        m = next;
    }
    return true;
}

static void testSolverConfidenceIsRateNormalized(TestContext& ctx) {
    for (float rateDegS : {30.0f, 60.0f, 90.0f, 120.0f}) {
        MagAxisAlignmentInterval intervals[64];
        Mat3 coarse;
        Mat3 expected;
        makeRateDataset(rateDegS, intervals, coarse, expected);
        MagAxisAlignmentSolvePolicy policy;
        MagAxisAlignmentResult result;
        CHECK(ctx, solveMagAxisAlignmentDataset(
            intervals, 64u, Vec3::zero(), Mat3::identity(),
            &coarse, policy, result));
        CHECK(ctx, result.valid);
        CHECK(ctx, result.validationPassed);
        CHECK(ctx, result.validationWinnerMatchesTraining);
        CHECK(ctx, result.trainingValidationRotationDifferenceDeg <=
                   policy.maxTrainingValidationRotationDifferenceDeg);
        CHECK(ctx, result.normalizedSeparation >= policy.minNormalizedSeparation);
        CHECK(ctx, result.totalObservableRotationDeg >= policy.minTotalObservableRotationDeg);
        CHECK(ctx, magAxisRotationDifferenceDeg(result.magToImu, expected) < 1.5f);
        if (rateDegS >= 60.0f) CHECK(ctx, result.improvesActive);
    }
}


static void testSameCoarseWinnerFallsBackWhenRefinementsDisagree(TestContext& ctx) {
    const Mat3 coarse(0,-1,0, 1,0,0, 0,0,1);
    const Mat3 inverse = coarse.transposed();
    const Mat3 validationFrameBias = Quat::fromEulerXYZ(
        2.2f * MATH_DEG_TO_RAD, 0.0f, 0.0f).toRotationMatrix();
    const Mat3 validationRawBias = validationFrameBias.transposed();

    MagAxisAlignmentInterval intervals[64];
    Vec3 m = Vec3(0.45f, 0.20f, 0.87f).normalized();
    for (uint16_t i = 0; i < 64u; ++i) {
        const Vec3 axis = (i % 4u == 0u) ? Vec3(1.0f, 0.2f, 0.1f).normalized()
                        : (i % 4u == 1u) ? Vec3(0.1f, 1.0f, 0.3f).normalized()
                        : (i % 4u == 2u) ? Vec3(0.25f, -0.1f, 1.0f).normalized()
                                         : Vec3(-0.7f, 0.5f, 0.5f).normalized();
        const Vec3 gyro = axis * (90.0f * MATH_DEG_TO_RAD);
        const float dt = 0.025f;
        const Vec3 next = Quat::fromRotationVector(gyro * (-dt)).rotate(m).normalized();
        Vec3 raw0 = inverse * m;
        Vec3 raw1 = inverse * next;
        const uint16_t windowId = static_cast<uint16_t>(i / 8u);
        if ((windowId & 1u) != 0u) {
            raw0 = validationRawBias * raw0;
            raw1 = validationRawBias * raw1;
        }
        intervals[i] = MagAxisAlignmentInterval{gyro, raw0, raw1, dt, windowId};
        m = next;
    }

    MagAxisAlignmentSolvePolicy policy;
    MagAxisAlignmentResult result;
    CHECK(ctx, solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), Mat3::identity(),
        nullptr, policy, result));
    CHECK(ctx, result.valid);
    CHECK(ctx, result.coarseWinnerMatchesTraining);
    CHECK(ctx, !result.continuousRefinementAgreement);
    CHECK(ctx, result.coarseConsensusFallbackUsed);
    CHECK(ctx, result.validationWinnerMatchesTraining);
    CHECK(ctx, result.validationPassed);
    CHECK(ctx, !result.refined);
    CHECK(ctx, result.trainingValidationRotationDifferenceDeg >
               policy.maxTrainingValidationRotationDifferenceDeg);
    CHECK(ctx, magAxisRotationDifferenceDeg(result.magToImu, coarse) < 0.1f);
}

static void testHoldoutRejectsTrainingOnlyFit(TestContext& ctx) {
    MagAxisAlignmentInterval intervals[64];
    Mat3 coarse;
    Mat3 expected;
    makeRateDataset(120.0f, intervals, coarse, expected);

    // Corrupt only validation windows with a non-rigid extra endpoint change.
    // Training remains perfectly solvable, but its fitted matrix must not be
    // considered proven when independent windows violate the same dynamics.
    const Mat3 validationBias = Quat::fromEulerXYZ(
        8.0f * MATH_DEG_TO_RAD, 0.0f, 0.0f).toRotationMatrix();
    for (auto& in : intervals) {
        if ((in.windowId & 1u) != 0u) {
            in.mag1Raw = validationBias * in.mag1Raw;
        }
    }

    MagAxisAlignmentSolvePolicy policy;
    MagAxisAlignmentResult result;
    CHECK(ctx, !solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), Mat3::identity(),
        &coarse, policy, result));
    CHECK(ctx, !result.valid);
    CHECK(ctx, !result.validationPassed || !result.validationWinnerMatchesTraining ||
               result.validationScore - result.trainingScore > policy.maxValidationGeneralizationGap);
}

static void testGuidedAxisReservoirPreservesLateAxesAndPartitions(TestContext& ctx) {
    MagAxisIntervalReservoir<60> reservoir;
    uint16_t window = 0u;

    auto feedAxis = [&](uint8_t axis, uint32_t count) {
        for (uint32_t i = 0; i < count; ++i) {
            Vec3 gyro = Vec3::zero();
            if (axis == 0u) gyro.x = 1.0f;
            else if (axis == 1u) gyro.y = 1.0f;
            else gyro.z = 1.0f;
            const float phase = static_cast<float>(i % 360u) * MATH_DEG_TO_RAD;
            const Vec3 m0(std::cos(phase), std::sin(phase), 0.25f);
            const Vec3 m1(std::cos(phase + 0.02f), std::sin(phase + 0.02f), 0.25f);
            reservoir.consider(MagAxisAlignmentInterval{
                gyro, m0, m1, 0.02f, window});
            if ((i % 8u) == 7u) window++;
        }
    };

    // A first-N collector would freeze in the X segment and discard all later
    // Y/Z evidence. The stratified reservoir must retain every observed axis
    // and both train/validation window parities despite a very long final tail.
    feedAxis(0u, 600u);
    feedAxis(1u, 600u);
    feedAxis(2u, 6000u);

    CHECK(ctx, reservoir.size() == 60u);
    CHECK(ctx, reservoir.seen() == 7200u);
    CHECK(ctx, reservoir.replacements() > 0u);
    CHECK(ctx, reservoir.skipped() > 0u);
    CHECK(ctx, reservoir.excitedAxes() == 3u);
    CHECK(ctx, reservoir.partitionConfirmedAxes() == 3u);
    CHECK(ctx, reservoir.independentWindows() >= 6u);
    uint32_t bruteForceWindows = 0u;
    for (uint16_t i = 0; i < reservoir.size(); ++i) {
        bool first = true;
        for (uint16_t j = 0; j < i; ++j) {
            if (reservoir.data()[j].windowId == reservoir.data()[i].windowId) {
                first = false;
                break;
            }
        }
        if (first) bruteForceWindows++;
    }
    CHECK(ctx, reservoir.independentWindows() == bruteForceWindows);
    for (uint8_t bucket = 0; bucket < 6u; ++bucket) {
        CHECK(ctx, reservoir.bucketCount(bucket) == 10u);
        CHECK(ctx, reservoir.bucketSeen(bucket) > reservoir.bucketCount(bucket));
    }

    reservoir.reset();
    CHECK(ctx, reservoir.size() == 0u);
    CHECK(ctx, reservoir.seen() == 0u);
    CHECK(ctx, reservoir.replacements() == 0u);
    CHECK(ctx, reservoir.skipped() == 0u);
    CHECK(ctx, reservoir.independentWindows() == 0u);
    CHECK(ctx, reservoir.excitedAxes() == 0u);
    CHECK(ctx, reservoir.partitionConfirmedAxes() == 0u);

    auto addPartitionEvidence = [&](uint8_t axis, uint16_t windowId) {
        for (uint8_t i = 0u; i < 8u; ++i) {
            Vec3 gyro = Vec3::zero();
            if (axis == 0u) gyro.x = 1.0f;
            else gyro.y = 1.0f;
            reservoir.consider(MagAxisAlignmentInterval{
                gyro, Vec3(1.0f, 0.0f, 0.2f), Vec3(0.99f, 0.02f, 0.2f), 0.02f, windowId});
        }
    };
    addPartitionEvidence(0u, 0u);
    addPartitionEvidence(1u, 2u);
    CHECK(ctx, reservoir.excitedAxes() == 2u);
    CHECK(ctx, reservoir.partitionConfirmedAxes() == 0u);
    addPartitionEvidence(0u, 1u);
    addPartitionEvidence(1u, 3u);
    CHECK(ctx, reservoir.partitionConfirmedAxes() == 2u);
}

static void testDynamicAxisSolverWithHardSoftAndNearOriginRawData(TestContext& ctx) {
    const Mat3 coarse(0,-1,0, 1,0,0, 0,0,1);
    const Mat3 residual = Quat::fromEulerXYZ(
        1.1f * MATH_DEG_TO_RAD,
       -0.8f * MATH_DEG_TO_RAD,
        1.4f * MATH_DEG_TO_RAD).toRotationMatrix();
    const Mat3 expected = residual * coarse;
    const Mat3 magFromImu = expected.transposed();
    const Vec3 hardIron(500.0f, 0.0f, 0.0f);
    const Mat3 softIron = Quat::fromEulerXYZ(0.15f, -0.21f, 0.27f).toRotationMatrix() *
        Mat3::diagonal(0.21f, 0.19f, 0.20f) *
        Quat::fromEulerXYZ(0.15f, -0.21f, 0.27f).toRotationMatrix().transposed();
    Mat3 softIronInv;
    CHECK(ctx, softIron.inverse(softIronInv));

    MagAxisAlignmentInterval intervals[64];
    Vec3 mImu = Vec3(0.45f, 0.20f, 0.87f).normalized() * 100.0f;
    for (uint16_t i = 0; i < 64u; ++i) {
        const Vec3 axis = (i % 4u == 0u) ? Vec3(1.0f, 0.2f, 0.1f).normalized()
                        : (i % 4u == 1u) ? Vec3(0.1f, 1.0f, 0.3f).normalized()
                        : (i % 4u == 2u) ? Vec3(0.25f, -0.1f, 1.0f).normalized()
                                         : Vec3(-0.7f, 0.5f, 0.5f).normalized();
        const Vec3 gyro = axis * (90.0f * MATH_DEG_TO_RAD);
        const float dt = 0.025f;
        const Vec3 nextImu = Quat::fromRotationVector(gyro * (-dt)).rotate(mImu);
        const Vec3 mag0Cal = magFromImu * mImu;
        const Vec3 mag1Cal = magFromImu * nextImu;
        intervals[i] = MagAxisAlignmentInterval{
            gyro,
            hardIron + softIronInv * mag0Cal,
            hardIron + softIronInv * mag1Cal,
            dt, static_cast<uint16_t>(i / 8u)};
        mImu = nextImu;
    }

    MagAxisAlignmentSolvePolicy policy;
    MagAxisAlignmentResult result;
    CHECK(ctx, solveMagAxisAlignmentDataset(
        intervals, 64u, hardIron, softIron,
        &coarse, policy, result));
    CHECK(ctx, result.valid);
    CHECK(ctx, result.failureReason == MagAxisAlignmentFailureReason::None);
    CHECK(ctx, result.excitedAxes == 3u);
    CHECK(ctx, result.partitionConfirmedAxes >= 2u);
    CHECK(ctx, result.independentWindows == 8u);
    CHECK(ctx, magAxisRotationDifferenceDeg(result.magToImu, expected) < 1.0f);
}

static void testSolverCountsUniqueReservoirWindows(TestContext& ctx) {
    MagAxisAlignmentInterval intervals[64];
    Mat3 coarse;
    Mat3 expected;
    makeRateDataset(90.0f, intervals, coarse, expected);
    for (uint16_t i = 0; i < 64u; ++i) intervals[i].windowId = static_cast<uint16_t>(i & 1u);
    MagAxisAlignmentSolvePolicy policy;
    policy.minIndependentWindows = 4u;
    policy.minExcitedAxes = 1u;
    policy.minTrainingWindows = 2u;
    policy.minValidationWindows = 2u;
    MagAxisAlignmentResult result;
    CHECK(ctx, !solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), Mat3::identity(),
        &coarse, policy, result));
    CHECK(ctx, result.independentWindows == 2u);
    CHECK(ctx, result.failureReason == MagAxisAlignmentFailureReason::InsufficientWindows);
}

static void testSolverRejectsInvalidHardSoftTransform(TestContext& ctx) {
    MagAxisAlignmentInterval intervals[64];
    Mat3 coarse;
    Mat3 expected;
    makeRateDataset(90.0f, intervals, coarse, expected);
    MagAxisAlignmentSolvePolicy policy;
    MagAxisAlignmentResult result;
    const Mat3 reflection(-1,0,0, 0,1,0, 0,0,1);
    CHECK(ctx, !solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), reflection,
        &coarse, policy, result));
    CHECK(ctx, result.failureReason == MagAxisAlignmentFailureReason::InvalidCalibration);
}

static void testRuntimeCollectorUsesCalibratedDirectionAndReservoir(TestContext& ctx) {
    MagAxisAlignmentCollector collector;
    Vec3 calibrated = Vec3(0.45f, 0.2f, 0.87f).normalized();
    uint64_t tUs = 100000u;
    uint32_t ms = 100u;
    for (uint32_t i = 0; i < 900u; ++i) {
        const Vec3 gyro = (i < 300u) ? Vec3(0.9f, 0.05f, 0.0f)
                         : (i < 600u) ? Vec3(0.0f, 0.95f, 0.05f)
                                      : Vec3(0.05f, 0.0f, 1.0f);
        const float dt = 0.017f;
        calibrated = Quat::fromRotationVector(gyro * (-dt)).rotate(calibrated).normalized();
        tUs += 17000u;
        ms += 17u;
        MagProcessedSample mag;
        mag.valid = mag.trusted = true;
        mag.raw = calibrated * 500.0f + Vec3(500.0f, 0.0f, 0.0f);
        mag.calibratedMagFrame = calibrated;
        mag.rawNorm = mag.raw.norm();
        mag.calibratedNorm = 1.0f;
        mag.seq = i + 1u;
        mag.t_us = tUs;
        mag.receivedMs = ms;
        collector.observe(gyro, tUs, mag, true);
    }
    CHECK(ctx, collector.intervalCount() == MagAxisAlignmentCollector::kMaxIntervals);
    CHECK(ctx, collector.excitedAxes() == 3u);
    CHECK(ctx, collector.partitionConfirmedAxes() == 3u);
    CHECK(ctx, collector.independentWindows() >= 4u);
    CHECK(ctx, collector.stats().intervalsReservoirReplaced > 0u);
    CHECK(ctx, collector.stats().intervalsReservoirSkipped > 0u);
    CHECK(ctx, collector.stats().intervalsRejectedCapacity == 0u);
}

static void testCalibratedRawOriginRemainsUsable(TestContext& ctx) {
    MagRuntimeProcessor processor;
    MagRuntimeConfig config;
    config.enabled = true;
    config.calibrationValid = true;
    config.axisAlignmentValid = true;
    config.hardIron = Vec3(500.0f, 0.0f, 0.0f);
    config.softIron = Mat3::identity();
    config.magToImu = Mat3::identity();
    config.expectedFieldNorm = 500.0f;
    config.minTrustNorm = 400.0f;
    config.maxTrustNorm = 600.0f;

    Lsm6dsvFifoReader::MagRawSample raw;
    raw.x = raw.y = raw.z = 0;
    raw.seq = 1u;
    raw.t_us = 1000u;
    MagProcessedSample processed;
    CHECK(ctx, processor.process(raw, config, 1u, processed));
    CHECK(ctx, processed.rawNorm == 0.0f);
    CHECK_NEAR(ctx, processed.calibratedNorm, 500.0f, 1.0e-5f);
    CHECK(ctx, (processed.rejectFlags & MAG_REJECT_ZERO_NORM) == 0u);
    CHECK(ctx, processed.trusted);
}

static void testMagRuntimeRetainsValidatedDeviceFrameDecision(TestContext& ctx) {
    MagRuntimeProcessor processor;
    MagRuntimeConfig config;
    config.enabled = true;
    config.calibrationValid = true;
    config.axisAlignmentValid = true;
    config.minTrustNorm = 0.1f;
    config.maxTrustNorm = 1000.0f;
    config.sensorToDeviceValid = true;
    config.sensorToDevice = Quat::fromEulerXYZ(0.0f, 0.0f, 0.5f * MATH_PI).toRotationMatrix();

    Lsm6dsvFifoReader::MagRawSample raw;
    raw.x = 100;
    raw.seq = 1u;
    raw.t_us = 1000u;
    MagProcessedSample processed;
    CHECK(ctx, processor.process(raw, config, 1u, processed));
    CHECK(ctx, processed.sensorToDeviceApplied);
    CHECK_NEAR(ctx, processed.body.x, 0.0f, 1.0e-4f);
    CHECK_NEAR(ctx, processed.body.y, 100.0f, 1.0e-4f);
    const SensorToDeviceFrame acceptedFrame = makeSensorToDeviceFrame(
        config.sensorToDeviceValid, config.sensorToDevice);
    CHECK(ctx, acceptedFrame.enabled);
    const Vec3 recovered = acceptedFrame.inverseApply(processed.body);
    CHECK_NEAR(ctx, recovered.x, 100.0f, 1.0e-4f);
    CHECK_NEAR(ctx, recovered.y, 0.0f, 1.0e-4f);

    config.sensorToDevice = Mat3::diagonal(2.0f, 1.0f, 1.0f);
    CHECK(ctx, processor.process(raw, config, 2u, processed));
    CHECK(ctx, !processed.sensorToDeviceApplied);
    CHECK_NEAR(ctx, processed.body.x, 100.0f, 1.0e-4f);
    CHECK_NEAR(ctx, processed.body.y, 0.0f, 1.0e-4f);
}

static void testDynamicAxisSolverKeepsPureRotationConstraint(TestContext& ctx) {
    const Mat3 reflection(-1,0,0, 0,1,0, 0,0,1);
    const Mat3 shear(1,0.1f,0, 0,1,0, 0,0,1);
    const Mat3 smallRotation = Quat::fromEulerXYZ(
        1.0f * MATH_DEG_TO_RAD,
       -2.0f * MATH_DEG_TO_RAD,
        0.5f * MATH_DEG_TO_RAD).toRotationMatrix();
    CHECK(ctx, !MagAxisAlignmentCollector::properRotation(reflection));
    CHECK(ctx, !MagAxisAlignmentCollector::properRotation(shear));
    CHECK(ctx, MagAxisAlignmentCollector::properRotation(smallRotation));
    CHECK_NEAR(ctx, smallRotation.determinant(), 1.0f, 1.0e-4f);
}

int main() {
    TestContext ctx;
    testFieldReliabilityFailsClosedAcrossChangedEnvironment(ctx);
    testHighDipHealthyFieldUsesAdaptiveHorizontalTrust(ctx);
    testHighDipLegacyHeadingStepGateScalesWithObservability(ctx);
    testUnobservableHorizontalHeadingSkipsDirectionalStepGate(ctx);
    testNearVerticalFieldRemainsFailClosedForYaw(ctx);
    testThirtyMinuteHighDipNoiseAndDisturbanceRecovery(ctx);
    testYawGateUsesSameAdaptiveHorizontalTrust(ctx);
    testModerateStationaryHeadingJumpStaysFailClosed(ctx);
    testAhrsYawFrameJumpDoesNotMasqueradeAsFieldJump(ctx);
    testStationaryFieldReturnRecoversAfterAhrsYawDrift(ctx);
    testStationaryFieldShortcutDisablesAfterPhysicalMotion(ctx);
    testStationaryJumpAtWindowBoundaryStillLatches(ctx);
    testSlowStationaryYawDriftRemainsCorrectable(ctx);
    testFastButPlausibleYawDriftDoesNotSelfLatch(ctx);
    testMaximumNormalYawCorrectionDoesNotSelfLatch(ctx);
    testRealRotationDoesNotTriggerStationaryJumpLatch(ctx);
    testVerySlowPhysicalRotationDoesNotTriggerJumpLatch(ctx);
    testFilteredHeadingRateAllowsNoisySixtyHertzReacquisition(ctx);
    testIntervalBuilderUsesTimestampCoherentGyroEndpoints(ctx);
    testAllProperSignedPermutationMountingsConverge(ctx);
    testReflectionAmbiguityRequiresRightHandedDriverContract(ctx);
    testOnlyProperSignedPermutationsAreAccepted(ctx);
    testAxisCollectorRejectsStaleGyroPair(ctx);
    testSixtyHertzCollectionSpansIndependentWindows(ctx);
    testRuntimeCollectorUsesSensorTimeAcrossProcessingBacklog(ctx);
    testDynamicAxisSolverRefinesMechanicalMisalignment(ctx);
    testSolverConfidenceIsRateNormalized(ctx);
    testSameCoarseWinnerFallsBackWhenRefinementsDisagree(ctx);
    testHoldoutRejectsTrainingOnlyFit(ctx);
    testGuidedAxisReservoirPreservesLateAxesAndPartitions(ctx);
    testDynamicAxisSolverWithHardSoftAndNearOriginRawData(ctx);
    testSolverCountsUniqueReservoirWindows(ctx);
    testSolverRejectsInvalidHardSoftTransform(ctx);
    testRuntimeCollectorUsesCalibratedDirectionAndReservoir(ctx);
    testCalibratedRawOriginRemainsUsable(ctx);
    testMagRuntimeRetainsValidatedDeviceFrameDecision(ctx);
    testDynamicAxisSolverKeepsPureRotationConstraint(ctx);
    return ctx.finish("test_mag_heading_reliability");
}
