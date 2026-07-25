#include "test_common.hpp"

#include "runtime/runtime_bias_types.hpp"
#include "sensor/mag_yaw_correction.hpp"

using namespace tracker;

static void testRuntimeBiasStateResets(TestContext& ctx) {
    RuntimeGyroBiasEstimator est;
    est.enabled = true;
    est.dryRun = true;
    est.runtimeTrimRadS = Vec3(0.1f, -0.2f, 0.3f);
    est.windows = 7;
    est.updates = 3;
    est.lastResidualDps = Vec3(1.0f, 2.0f, 3.0f);

    est.resetCounters();
    CHECK(ctx, est.enabled);
    CHECK(ctx, est.dryRun);
    CHECK(ctx, est.windows == 0);
    CHECK(ctx, est.updates == 0);
    CHECK_NEAR(ctx, est.runtimeTrimRadS.x, 0.1f, 1.0e-6f);

    est.resetAll();
    CHECK(ctx, !est.enabled);
    CHECK(ctx, !est.dryRun);
    CHECK_NEAR(ctx, est.runtimeTrimRadS.norm(), 0.0f, 1.0e-6f);
}

static MagYawCorrectionInput makeGoodMagYawInput(uint32_t nowMs) {
    MagYawCorrectionInput in;
    in.referenceValid = true;
    in.referenceWorldYawRad = 0.0f;

    in.mag.valid = true;
    in.mag.trusted = true;
    in.mag.receivedMs = nowMs;
    in.mag.t_us = static_cast<uint64_t>(nowMs) * 1000ULL;
    in.mag.seq = 42;

    in.heading.valid = true;
    in.heading.horizontalNorm = 300.0f;
    in.heading.magneticNorthWorldYawRad = 5.0f * MATH_DEG_TO_RAD;

    in.magTrustedForUse = true;
    in.magRejectFlagsForUse = MAG_REJECT_NONE;
    in.gyroNormDps = 0.5f;
    in.accelTrust = 1.0f;
    in.fieldReliable = true;
    in.fieldStableMs = 10000;
    in.magneticHeadingRateDegS = 0.0f;
    in.nowMs = nowMs;
    return in;
}

static void testMagYawGateOpenAndCorrectionDirection(TestContext& ctx) {
    MagYawCorrectionConfig cfg;
    cfg.applyEnabled = true;
    cfg.maxInnovationDeg = 25.0f;
    cfg.timeConstantS = 10.0f;
    cfg.maxCorrectionRateDegS = 2.0f;
    cfg.maxCorrectionStepDeg = 0.25f;
    cfg.fallbackDtS = 0.01f;

    MagYawCorrectionController controller;
    MagYawCorrectionOutput out;

    controller.update(makeGoodMagYawInput(1000), cfg, out);
    CHECK(ctx, out.gateOpen);
    CHECK(ctx, out.applyAllowed);
    CHECK(ctx, out.rejectFlags == MAG_YAW_REJECT_NONE);
    CHECK_NEAR(ctx, out.combinedTrust, 1.0f, 1.0e-6f);
    CHECK(ctx, out.correctionStepDeg < 0.0f);
    CHECK(ctx, std::fabs(out.correctionStepDeg) <= cfg.maxCorrectionStepDeg + 1.0e-6f);

    controller.markApplied(out.correctionStepDeg);
    CHECK(ctx, controller.stats().appliedCount == 1);
    CHECK(ctx, controller.last().applied);
}

static void testMagYawRejectsAndCooldown(TestContext& ctx) {
    MagYawCorrectionConfig cfg;
    cfg.applyEnabled = true;
    cfg.gyroMovingCooldownMs = 1000;

    MagYawCorrectionController controller;
    MagYawCorrectionOutput out;

    MagYawCorrectionInput moving = makeGoodMagYawInput(2000);
    moving.gyroNormDps = 100.0f;
    controller.update(moving, cfg, out);
    CHECK(ctx, (out.rejectFlags & MAG_YAW_REJECT_GYRO_MOVING) != 0);
    CHECK(ctx, !out.gateOpen);

    MagYawCorrectionInput afterMotion = makeGoodMagYawInput(2100);
    controller.update(afterMotion, cfg, out);
    CHECK(ctx, (out.rejectFlags & MAG_YAW_REJECT_COOLDOWN) != 0);
    CHECK(ctx, out.cooldownActive);
    CHECK(ctx, !out.gateOpen);

    MagYawCorrectionInput afterCooldown = makeGoodMagYawInput(3101);
    controller.update(afterCooldown, cfg, out);
    CHECK(ctx, out.rejectFlags == MAG_YAW_REJECT_NONE);
    CHECK(ctx, out.gateOpen);
}


static void testMagYawLargeErrorReacquiresWithoutPermanentCooldown(TestContext& ctx) {
    MagYawCorrectionConfig cfg;
    cfg.applyEnabled = true;
    cfg.maxInnovationDeg = 25.0f;
    cfg.reacquireInnovationMaxDeg = 170.0f;
    cfg.reacquireMinFieldStableMs = 8000;
    cfg.reacquireMaxHeadingRateDegS = 2.0f;

    MagYawCorrectionController controller;
    MagYawCorrectionOutput out;

    MagYawCorrectionInput pending = makeGoodMagYawInput(1000);
    pending.heading.magneticNorthWorldYawRad = 40.0f * MATH_DEG_TO_RAD;
    pending.fieldStableMs = 2000;
    controller.update(pending, cfg, out);
    CHECK(ctx, out.reacquirePending);
    CHECK(ctx, (out.rejectFlags & MAG_YAW_REJECT_REACQUIRE_PENDING) != 0u);
    CHECK(ctx, (out.rejectFlags & MAG_YAW_REJECT_COOLDOWN) == 0u);

    MagYawCorrectionInput ready = pending;
    ready.nowMs = 9001;
    ready.mag.receivedMs = ready.nowMs;
    ready.mag.t_us = static_cast<uint64_t>(ready.nowMs) * 1000ULL;
    ready.fieldStableMs = 9000;
    ready.magneticHeadingRateDegS = 0.2f;
    controller.update(ready, cfg, out);
    CHECK(ctx, out.gateOpen);
    CHECK(ctx, out.reacquireActive);
    CHECK(ctx, out.mode == MagYawCorrectionMode::Reacquiring);
    CHECK(ctx, out.applyAllowed);
    CHECK(ctx, out.correctionStepDeg < 0.0f);
    CHECK(ctx, std::fabs(out.correctionStepDeg) <= cfg.reacquireMaxCorrectionStepDeg + 1.0e-6f);
}

static void testMagRuntimeBodyNormExtremaInitializeFromFirstSample(TestContext& ctx) {
    MagRuntimeConfig cfg;
    cfg.enabled = true;
    cfg.calibrationValid = true;
    cfg.axisAlignmentValid = true;
    cfg.minTrustNorm = 0.0f;
    cfg.maxTrustNorm = 100.0f;

    MagRuntimeProcessor processor;
    Lsm6dsvFifoReader::MagRawSample raw;
    raw.t_us = 1000;
    raw.seq = 1;
    raw.x = 3;
    raw.y = 4;
    raw.z = 0;

    MagProcessedSample out;
    CHECK(ctx, processor.process(raw, cfg, 10, out));
    CHECK_NEAR(ctx, processor.stats().bodyNormMin, 5.0f, 1.0e-6f);
    CHECK_NEAR(ctx, processor.stats().bodyNormMax, 5.0f, 1.0e-6f);

    raw.t_us = 2000;
    raw.seq = 2;
    raw.x = 6;
    raw.y = 8;
    CHECK(ctx, processor.process(raw, cfg, 20, out));
    CHECK_NEAR(ctx, processor.stats().bodyNormMin, 5.0f, 1.0e-6f);
    CHECK_NEAR(ctx, processor.stats().bodyNormMax, 10.0f, 1.0e-6f);
    CHECK_NEAR(ctx, processor.stats().bodyNormMean(), 7.5f, 1.0e-6f);
}

static void testMagSampleAgeAcceptsZeroAndWrap(TestContext& ctx) {
    MagProcessedSample sample;
    sample.valid = true;
    sample.receivedMs = 0u;
    CHECK(ctx, MagRuntimeProcessor::ageMsForUse(sample, 7u) == 7u);

    sample.receivedMs = 0xFFFFFFFAu;
    CHECK(ctx, MagRuntimeProcessor::ageMsForUse(sample, 4u) == 10u);

    sample.valid = false;
    CHECK(ctx, MagRuntimeProcessor::ageMsForUse(sample, 4u) == 0xFFFFFFFFu);
}

static void testMagYawDtIsWrapSafe(TestContext& ctx) {
    MagYawCorrectionConfig cfg;
    cfg.applyEnabled = true;
    cfg.fallbackDtS = 0.25f;

    MagYawCorrectionController controller;
    MagYawCorrectionOutput out;

    controller.update(makeGoodMagYawInput(0xFFFFFFFAu), cfg, out);
    CHECK(ctx, out.dtMs == 0);

    controller.update(makeGoodMagYawInput(4u), cfg, out);
    CHECK(ctx, out.dtMs == 10u);
}

int main() {
    TestContext ctx;
    testRuntimeBiasStateResets(ctx);
    testMagYawGateOpenAndCorrectionDirection(ctx);
    testMagYawRejectsAndCooldown(ctx);
    testMagYawLargeErrorReacquiresWithoutPermanentCooldown(ctx);
    testMagRuntimeBodyNormExtremaInitializeFromFirstSample(ctx);
    testMagSampleAgeAcceptsZeroAndWrap(ctx);
    testMagYawDtIsWrapSafe(ctx);
    return ctx.finish("test_mag_yaw_runtime_bias");
}
