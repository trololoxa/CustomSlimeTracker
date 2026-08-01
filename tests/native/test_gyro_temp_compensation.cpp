#include "test_common.hpp"

#include "sensor/gyro_temperature_compensation.hpp"

using namespace tracker;

static void testBiasAtAndCorrectedGyro(TestContext& ctx) {
    GyroTempCompensator comp;
    comp.reset(Vec3(1.0f, -2.0f, 0.5f) * MATH_DEG_TO_RAD, 30.0f);
    comp.setSlopeDpsPerC(Vec3(0.05f, -0.10f, 0.00f));

    const Vec3 biasDps = comp.biasAt(35.0f) * MATH_RAD_TO_DEG;
    CHECK_NEAR(ctx, biasDps.x, 1.25f, 1.0e-5f);
    CHECK_NEAR(ctx, biasDps.y, -2.50f, 1.0e-5f);
    CHECK_NEAR(ctx, biasDps.z, 0.50f, 1.0e-5f);

    const Vec3 raw = Vec3(2.0f, -4.0f, 1.0f) * MATH_DEG_TO_RAD;
    const Vec3 correctedDps = comp.correctedGyro(raw, 35.0f) * MATH_RAD_TO_DEG;
    CHECK_NEAR(ctx, correctedDps.x, 0.75f, 1.0e-5f);
    CHECK_NEAR(ctx, correctedDps.y, -1.50f, 1.0e-5f);
    CHECK_NEAR(ctx, correctedDps.z, 0.50f, 1.0e-5f);

    comp.setEnabled(false);
    const Vec3 disabledBiasDps = comp.biasAt(35.0f) * MATH_RAD_TO_DEG;
    CHECK_NEAR(ctx, disabledBiasDps.x, 1.0f, 1.0e-5f);
    CHECK_NEAR(ctx, disabledBiasDps.y, -2.0f, 1.0e-5f);
    CHECK_NEAR(ctx, disabledBiasDps.z, 0.5f, 1.0e-5f);
}

static void testSnapshotRangeAndMetadata(TestContext& ctx) {
    GyroTempCompensator comp;
    comp.setModel(Vec3(0.0f, 0.0f, 0.0f), 25.0f, Vec3::zero());
    comp.setQualityMetadata(20.0f, 35.0f, 0.75f, 0.20f, 0.05f);

    GyroTempCompSnapshot inside = comp.snapshot(30.0f);
    CHECK(ctx, inside.valid);
    CHECK(ctx, inside.enabled);
    CHECK(ctx, inside.hasCalibratedRange);
    CHECK(ctx, !inside.tempOutOfRange);
    CHECK_NEAR(ctx, inside.fitQuality, 0.75f, 1.0e-6f);

    GyroTempCompSnapshot outside = comp.snapshot(40.0f);
    CHECK(ctx, outside.hasCalibratedRange);
    CHECK(ctx, outside.tempOutOfRange);
    CHECK(ctx, outside.tempSoftExtrapolated);
    CHECK(ctx, !outside.tempHardExtrapolated);
    CHECK_NEAR(ctx, outside.tempDistanceToRangeC, 5.0f, 1.0e-6f);
    CHECK(ctx, outside.extrapolationConfidence > 0.0f);
    CHECK(ctx, outside.extrapolationConfidence < 1.0f);

    GyroTempCompSnapshot farOutside = comp.snapshot(60.0f);
    CHECK(ctx, farOutside.tempOutOfRange);
    CHECK(ctx, farOutside.tempHardExtrapolated);
}

static void testRuntimeEvalMatchesSnapshot(TestContext& ctx) {
    GyroTempCompensator comp;
    comp.setModel(Vec3(0.1f, -0.2f, 0.05f) * MATH_DEG_TO_RAD,
                  30.0f,
                  Vec3(0.01f, 0.02f, -0.03f) * MATH_DEG_TO_RAD);
    comp.setQualityMetadata(25.0f, 35.0f, 0.8f, 0.2f, 0.05f);

    const float temperatures[] = {20.0f, 25.0f, 30.0f, 35.0f, 40.0f, 60.0f};
    for (float tempC : temperatures) {
        const GyroTempCompRuntimeEval eval = comp.evaluateRuntime(tempC);
        const GyroTempCompSnapshot snapshot = comp.snapshot(tempC);
        CHECK(ctx, eval.valid == snapshot.valid);
        CHECK(ctx, eval.temperatureModelValid == snapshot.temperatureModelValid);
        CHECK(ctx, eval.enabled == snapshot.enabled);
        CHECK(ctx, eval.hasCalibratedRange == snapshot.hasCalibratedRange);
        CHECK(ctx, eval.tempOutOfRange == snapshot.tempOutOfRange);
        CHECK(ctx, eval.tempSoftExtrapolated == snapshot.tempSoftExtrapolated);
        CHECK(ctx, eval.tempHardExtrapolated == snapshot.tempHardExtrapolated);
        CHECK_NEAR(ctx, eval.currentBiasRadS.x, snapshot.currentBiasRadS.x, 0.0f);
        CHECK_NEAR(ctx, eval.currentBiasRadS.y, snapshot.currentBiasRadS.y, 0.0f);
        CHECK_NEAR(ctx, eval.currentBiasRadS.z, snapshot.currentBiasRadS.z, 0.0f);
        CHECK_NEAR(ctx, eval.tempDistanceToRangeC, snapshot.tempDistanceToRangeC, 0.0f);
        CHECK_NEAR(ctx, eval.extrapolationConfidence, snapshot.extrapolationConfidence, 0.0f);
    }

    const GyroTempCompRuntimeEval nonfinite = comp.evaluateRuntime(NAN);
    const GyroTempCompSnapshot nonfiniteSnapshot = comp.snapshot(NAN);
    CHECK(ctx, nonfinite.valid == nonfiniteSnapshot.valid);
    CHECK(ctx, nonfinite.tempOutOfRange == nonfiniteSnapshot.tempOutOfRange);
    CHECK(ctx, nonfinite.tempSoftExtrapolated == nonfiniteSnapshot.tempSoftExtrapolated);
    CHECK(ctx, nonfinite.tempHardExtrapolated == nonfiniteSnapshot.tempHardExtrapolated);

    comp.setEnabled(false);
    const GyroTempCompRuntimeEval disabled = comp.evaluateRuntime(40.0f);
    CHECK(ctx, disabled.valid);
    CHECK(ctx, !disabled.enabled);
    CHECK_NEAR(ctx, disabled.currentBiasRadS.x,
               comp.referenceBiasRadS().x, 0.0f);

    GyroTempCompensator invalid;
    const GyroTempCompRuntimeEval invalidEval = invalid.evaluateRuntime(25.0f);
    const GyroTempCompSnapshot invalidSnapshot = invalid.snapshot(25.0f);
    CHECK(ctx, invalidEval.valid == invalidSnapshot.valid);
    CHECK(ctx, invalidEval.temperatureModelValid ==
               invalidSnapshot.temperatureModelValid);
    CHECK_NEAR(ctx, invalidEval.currentBiasRadS.norm(),
               invalidSnapshot.currentBiasRadS.norm(), 0.0f);
}

static void testSlopeAcceptanceLimit(TestContext& ctx) {
    GyroTempCompensator comp;
    const float limit = comp.config().maxAcceptedSlopeDpsPerC;
    CHECK(ctx, comp.acceptsSlopeDpsPerC(Vec3(limit, -limit, 0.0f)));
    CHECK(ctx, !comp.acceptsSlopeDpsPerC(Vec3(limit + 0.0001f, 0.0f, 0.0f)));
    CHECK(ctx, !comp.acceptsSlopeDpsPerC(Vec3(NAN, 0.0f, 0.0f)));
    CHECK(ctx, comp.acceptsSlopeRadSPerC(Vec3(limit, 0.0f, -limit) * MATH_DEG_TO_RAD));
}

static void testSetModelRejectsOversizedSlope(TestContext& ctx) {
    GyroTempCompensator comp;
    const float limit = comp.config().maxAcceptedSlopeDpsPerC;
    comp.setModel(Vec3(0.01f, 0.0f, 0.0f),
                  25.0f,
                  Vec3(limit + 0.01f, 0.0f, 0.0f) * MATH_DEG_TO_RAD);
    CHECK(ctx, comp.valid());
    CHECK(ctx, !comp.temperatureModelValid());
    CHECK_NEAR(ctx, comp.slopeRadSPerC().norm(), 0.0f, 1.0e-9f);
}

int main() {
    TestContext ctx;
    testBiasAtAndCorrectedGyro(ctx);
    testSnapshotRangeAndMetadata(ctx);
    testRuntimeEvalMatchesSnapshot(ctx);
    testSlopeAcceptanceLimit(ctx);
    testSetModelRejectsOversizedSlope(ctx);
    return ctx.finish("test_gyro_temp_compensation");
}
