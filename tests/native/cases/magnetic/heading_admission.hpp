#pragma once
// heading admission scenarios; single suite translation unit.
static void testHeadingRejectsInvalidQuaternionWithoutIdentityFallback(TestContext& ctx) {
    MagProcessedSample mag;
    mag.valid = true;
    mag.trusted = true;
    mag.body = Vec3(100.0f, 20.0f, 40.0f);
    mag.bodyNorm = mag.body.norm();
    MagHeadingEstimator estimator;
    MagHeadingSample heading;
    MagHeadingConfig cfg;

    CHECK(ctx, !estimator.update(
        mag, Quat(0.0f, 0.0f, 0.0f, 0.0f), cfg, 1u, heading));
    CHECK(ctx, (heading.rejectFlags & MAG_HEADING_REJECT_QUAT_INVALID) != 0u);
    CHECK(ctx, !heading.valid);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    CHECK(ctx, !estimator.update(
        mag, Quat(nan, 0.0f, 0.0f, 0.0f), cfg, 2u, heading));
    CHECK(ctx, (heading.rejectFlags & MAG_HEADING_REJECT_QUAT_INVALID) != 0u);
}

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
