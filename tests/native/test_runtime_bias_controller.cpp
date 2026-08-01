#include "test_common.hpp"

#include "runtime/runtime_gyro_bias_controller.hpp"

using namespace tracker;

static GyroTempCompensator makeTempCompWithRange() {
    GyroTempCompensator comp;
    comp.reset(Vec3(0.10f, -0.20f, 0.05f) * MATH_DEG_TO_RAD, 30.0f);
    comp.setSlopeDpsPerC(Vec3(0.01f, 0.00f, -0.02f));
    comp.setQualityMetadata(25.0f, 35.0f, 0.8f, 0.2f, 0.05f);
    return comp;
}

static void testBaseAndCurrentBiasSelection(TestContext& ctx) {
    ImuCalibration imu;
    GyroTempCompensator emptyComp;
    RuntimeGyroBiasEstimator bias;

    CHECK(ctx, !runtimeBiasHasBaseGyroBiasModel(imu, emptyComp));
    CHECK_NEAR(ctx, runtimeBiasBaseGyroBiasRadS(imu, emptyComp, 30.0f).norm(), 0.0f, 1.0e-8f);

    imu.gyroBiasValid = true;
    imu.gyroBiasRadS = Vec3(1.0f, 2.0f, 3.0f) * MATH_DEG_TO_RAD;
    CHECK(ctx, runtimeBiasHasBaseGyroBiasModel(imu, emptyComp));
    CHECK_NEAR(ctx, runtimeBiasBaseGyroBiasRadS(imu, emptyComp, 30.0f).x * MATH_RAD_TO_DEG, 1.0f, 1.0e-6f);

    GyroTempCompensator comp = makeTempCompWithRange();
    const Vec3 tempBiasDps = runtimeBiasBaseGyroBiasRadS(imu, comp, 35.0f) * MATH_RAD_TO_DEG;
    CHECK_NEAR(ctx, tempBiasDps.x, 0.15f, 1.0e-6f);
    CHECK_NEAR(ctx, tempBiasDps.y, -0.20f, 1.0e-6f);
    CHECK_NEAR(ctx, tempBiasDps.z, -0.05f, 1.0e-6f);

    bias.runtimeTrimRadS = Vec3(0.01f, 0.02f, -0.03f) * MATH_DEG_TO_RAD;
    const Vec3 currentDps = runtimeBiasCurrentGyroBiasRadS(bias, imu, comp, 35.0f) * MATH_RAD_TO_DEG;
    const GyroTempCompRuntimeEval eval = comp.evaluateRuntime(35.0f);
    const Vec3 currentFromEvalDps =
        runtimeBiasCurrentGyroBiasRadS(bias, imu, eval) * MATH_RAD_TO_DEG;
    CHECK_NEAR(ctx, currentDps.x, 0.16f, 1.0e-6f);
    CHECK_NEAR(ctx, currentDps.y, -0.18f, 1.0e-6f);
    CHECK_NEAR(ctx, currentDps.z, -0.08f, 1.0e-6f);
    CHECK_NEAR(ctx, currentFromEvalDps.x, currentDps.x, 0.0f);
    CHECK_NEAR(ctx, currentFromEvalDps.y, currentDps.y, 0.0f);
    CHECK_NEAR(ctx, currentFromEvalDps.z, currentDps.z, 0.0f);
}

static void testTemperatureGateAndQualityFlags(TestContext& ctx) {
    GyroTempCompensator comp = makeTempCompWithRange();
    RuntimeGyroBiasEstimator bias;
    bias.tempExtrapolationMarginC = 5.0f;
    bias.outOfRangeGainScale = 0.25f;
    bias.requireTempCompRange = true;
    bias.allowOutOfRangeEstimator = true;

    RuntimeBiasTempGate inside = runtimeBiasTempGateFor(bias, comp, 30.0f);
    CHECK(ctx, inside.rangeRelevant);
    CHECK(ctx, !inside.outOfRange);
    CHECK(ctx, !inside.reject);
    CHECK_NEAR(ctx, inside.gainScale, 1.0f, 1.0e-6f);

    RuntimeBiasTempGate near = runtimeBiasTempGateFor(bias, comp, 38.0f);
    RuntimeBiasTempGate nearFromEval = runtimeBiasTempGateFor(
        bias, comp.evaluateRuntime(38.0f));
    CHECK(ctx, near.rangeRelevant);
    CHECK(ctx, near.outOfRange);
    CHECK(ctx, near.nearOutOfRange);
    CHECK(ctx, !near.reject);
    CHECK_NEAR(ctx, near.distanceToRangeC, 3.0f, 1.0e-6f);
    CHECK_NEAR(ctx, near.gainScale, 0.22f, 1.0e-6f);
    CHECK(ctx, nearFromEval.rangeRelevant == near.rangeRelevant);
    CHECK(ctx, nearFromEval.outOfRange == near.outOfRange);
    CHECK(ctx, nearFromEval.nearOutOfRange == near.nearOutOfRange);
    CHECK(ctx, nearFromEval.reject == near.reject);
    CHECK_NEAR(ctx, nearFromEval.distanceToRangeC, near.distanceToRangeC, 0.0f);
    CHECK_NEAR(ctx, nearFromEval.gainScale, near.gainScale, 0.0f);

    RuntimeBiasTempGate moderate = runtimeBiasTempGateFor(bias, comp, 44.0f);
    CHECK(ctx, moderate.outOfRange);
    CHECK(ctx, moderate.farOutOfRange);
    CHECK(ctx, moderate.nearOutOfRange);
    CHECK(ctx, !moderate.reject);
    CHECK_NEAR(ctx, moderate.distanceToRangeC, 9.0f, 1.0e-6f);
    CHECK(ctx, moderate.gainScale < near.gainScale);

    RuntimeBiasTempGate far = runtimeBiasTempGateFor(bias, comp, 60.0f);
    CHECK(ctx, far.outOfRange);
    CHECK(ctx, far.farOutOfRange);
    CHECK(ctx, far.reject);

    ImuQualityResult q;
    runtimeBiasApplyGyroTempQualityFlags(comp, q, 37.0f);
    CHECK(ctx, q.has(imu_quality_flags::TEMP_COMP_OUT_OF_RANGE));
    CHECK(ctx, q.gyroConfidence > 0.80f);
    CHECK(ctx, q.overallConfidence > 0.80f);
}

static void testClampAndDecisionFlags(TestContext& ctx) {
    RuntimeGyroBiasEstimator bias;
    bias.maxRuntimeTrimDps = 0.08f;
    const Vec3 clamped = clampRuntimeTrimDps(bias, Vec3(0.10f, -0.20f, 0.03f));
    CHECK_NEAR(ctx, clamped.x, 0.08f, 1.0e-6f);
    CHECK_NEAR(ctx, clamped.y, -0.08f, 1.0e-6f);
    CHECK_NEAR(ctx, clamped.z, 0.03f, 1.0e-6f);

    const uint32_t flags = runtimeBiasDecisionFlags(true, false, true, false, true, false, true, false, true, true, true);
    CHECK(ctx, (flags & (1u << 0)) != 0);  // accepted
    CHECK(ctx, (flags & (1u << 2)) != 0);  // saturated
    CHECK(ctx, (flags & (1u << 4)) != 0);  // temp bad
    CHECK(ctx, (flags & (1u << 6)) != 0);  // accel bad
    CHECK(ctx, (flags & (1u << 8)) != 0);  // priming
    CHECK(ctx, (flags & (1u << 9)) != 0);  // dry run
    CHECK(ctx, (flags & (1u << 10)) != 0); // temp cautious
}


static void testDeferredWindowFinalizationPreservesDecision(TestContext& ctx) {
    ImuCalibration imu;
    imu.gyroBiasValid = true;
    imu.gyroBiasRadS = Vec3::zero();
    imu.accelCalValid = true;

    GyroTempCompensator temp;
    RuntimeGyroBiasEstimator bias;
    bias.enabled = true;
    bias.requireTempCompRange = false;
    bias.windowSamplesRequired = 4u;
    bias.stationaryWindowsBeforeUpdate = 1u;
    bias.accelTrustMin = 0.0f;
    bias.gyroMeanMaxDps = 0.08f;
    bias.gyroStdNormMaxDps = 0.16f;
    bias.gyroStdAxisMaxDps = 0.12f;
    bias.accelNormMeanMaxErrG = 0.015f;
    bias.accelNormStdMaxG = 0.006f;
    bias.updateAlpha = 0.05f;
    bias.maxUpdateStepDps = 0.0015f;

    Ahrs6Dof ahrs;
    RuntimeGyroBiasUpdateDeps deps{bias, imu, temp, ahrs};
    ImuQualityResult quality;
    quality.flags = imu_quality_flags::OK;
    quality.shouldUpdateAhrs = true;
    quality.shouldUseAccelCorrection = true;
    quality.accelNormValid = true;
    quality.accelNormG = 1.0f;

    Lsm6dsv::Sample scaled;
    scaled.gyro_rad_s = Vec3(0.02f, 0.0f, 0.0f) * MATH_DEG_TO_RAD;
    scaled.accel_g = Vec3(0.0f, 0.0f, 1.0f);
    scaled.temp_c = 25.0f;
    Lsm6dsv::Sample calibrated = scaled;

    CHECK(ctx, !runtimeBiasUpdateEstimator(deps, scaled, calibrated, quality, 1000u));
    CHECK(ctx, !runtimeBiasUpdateEstimator(deps, scaled, calibrated, quality, 2000u));
    CHECK(ctx, !runtimeBiasUpdateEstimator(deps, scaled, calibrated, quality, 3000u));
    CHECK(ctx, runtimeBiasUpdateEstimator(deps, scaled, calibrated, quality, 4000u));
    CHECK(ctx, bias.completedWindowPending);
    CHECK(ctx, bias.windowsDeferred == 1u);
    CHECK(ctx, bias.windows == 0u);
    CHECK_NEAR(ctx, bias.runtimeTrimRadS.norm(), 0.0f, 1.0e-9f);
    CHECK(ctx, bias.calibratedGyroRadS.count == 0u);

    CHECK(ctx, runtimeBiasFinalizePendingWindow(deps));
    CHECK(ctx, !bias.completedWindowPending);
    CHECK(ctx, bias.windows == 1u);
    CHECK(ctx, bias.accepted == 1u);
    CHECK(ctx, bias.updates == 1u);
    CHECK_NEAR(ctx, bias.runtimeTrimRadS.x * MATH_RAD_TO_DEG, 0.001f, 1.0e-7f);
    CHECK(ctx, !runtimeBiasFinalizePendingWindow(deps));
}

int main() {
    TestContext ctx;
    testBaseAndCurrentBiasSelection(ctx);
    testTemperatureGateAndQualityFlags(ctx);
    testClampAndDecisionFlags(ctx);
    testDeferredWindowFinalizationPreservesDecision(ctx);
    return ctx.finish("test_runtime_bias_controller");
}
