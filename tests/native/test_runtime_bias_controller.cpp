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
    CHECK_NEAR(ctx, currentDps.x, 0.16f, 1.0e-6f);
    CHECK_NEAR(ctx, currentDps.y, -0.18f, 1.0e-6f);
    CHECK_NEAR(ctx, currentDps.z, -0.08f, 1.0e-6f);
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
    CHECK(ctx, near.rangeRelevant);
    CHECK(ctx, near.outOfRange);
    CHECK(ctx, near.nearOutOfRange);
    CHECK(ctx, !near.reject);
    CHECK_NEAR(ctx, near.distanceToRangeC, 3.0f, 1.0e-6f);
    CHECK_NEAR(ctx, near.gainScale, 0.25f, 1.0e-6f);

    RuntimeBiasTempGate far = runtimeBiasTempGateFor(bias, comp, 50.0f);
    CHECK(ctx, far.outOfRange);
    CHECK(ctx, far.farOutOfRange);
    CHECK(ctx, far.reject);

    ImuQualityResult q;
    runtimeBiasApplyGyroTempQualityFlags(comp, q, 50.0f);
    CHECK(ctx, q.has(imu_quality_flags::TEMP_COMP_OUT_OF_RANGE));
    CHECK_NEAR(ctx, q.gyroConfidence, 0.75f, 1.0e-6f);
    CHECK_NEAR(ctx, q.overallConfidence, 0.75f, 1.0e-6f);
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

int main() {
    TestContext ctx;
    testBaseAndCurrentBiasSelection(ctx);
    testTemperatureGateAndQualityFlags(ctx);
    testClampAndDecisionFlags(ctx);
    return ctx.finish("test_runtime_bias_controller");
}
