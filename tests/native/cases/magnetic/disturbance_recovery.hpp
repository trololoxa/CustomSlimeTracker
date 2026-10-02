#pragma once
// disturbance recovery scenarios; single suite translation unit.
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
