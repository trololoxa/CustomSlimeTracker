#include "test_common.hpp"

#include <limits>

#include "runtime/static_test_types.hpp"

using namespace tracker;

static void testScalarStats(TestContext& ctx) {
    ScalarStats s;
    s.push(1.0f);
    s.push(2.0f);
    s.push(3.0f);
    s.push(std::numeric_limits<float>::quiet_NaN());

    CHECK(ctx, s.count == 3);
    CHECK_NEAR(ctx, s.mean(), 2.0f, 1.0e-6f);
    CHECK_NEAR(ctx, s.stddev(), std::sqrt(2.0f / 3.0f), 1.0e-6f);
    CHECK_NEAR(ctx, s.minValue, 1.0f, 1.0e-6f);
    CHECK_NEAR(ctx, s.maxValue, 3.0f, 1.0e-6f);

    s.reset();
    CHECK(ctx, s.count == 0);
    CHECK_NEAR(ctx, s.mean(), 0.0f, 1.0e-6f);
}

static void testVec3Stats(TestContext& ctx) {
    Vec3Stats v;
    v.push(Vec3(1.0f, 2.0f, 3.0f));
    v.push(Vec3(3.0f, 6.0f, 9.0f));

    CHECK(ctx, v.count == 2);
    const Vec3 mean = v.mean();
    CHECK_NEAR(ctx, mean.x, 2.0f, 1.0e-6f);
    CHECK_NEAR(ctx, mean.y, 4.0f, 1.0e-6f);
    CHECK_NEAR(ctx, mean.z, 6.0f, 1.0e-6f);

    const Vec3 sd = v.stddev();
    CHECK_NEAR(ctx, sd.x, 1.0f, 1.0e-6f);
    CHECK_NEAR(ctx, sd.y, 2.0f, 1.0e-6f);
    CHECK_NEAR(ctx, sd.z, 3.0f, 1.0e-6f);
}

static void testTempBinStatsRejectBadQuality(TestContext& ctx) {
    StaticTempBinStats b;
    b.push(25.0f, Vec3(1.0f, 2.0f, 3.0f), 1.0f, false);
    CHECK(ctx, b.badQualitySamples == 1);
    CHECK(ctx, b.tempC.count == 0);
    CHECK(ctx, b.gyroAfterRadS.count == 0);

    b.push(25.0f, Vec3(1.0f, 2.0f, 3.0f), 1.0f, true);
    CHECK(ctx, b.badQualitySamples == 1);
    CHECK(ctx, b.tempC.count == 1);
    CHECK(ctx, b.gyroAfterRadS.count == 1);
}

static void testTempBins(TestContext& ctx) {
    CHECK(ctx, staticTempBinIndex(9.99f) == -1);
    CHECK(ctx, staticTempBinIndex(10.00f) == 0);
    CHECK(ctx, staticTempBinIndex(10.99f) == 0);
    CHECK(ctx, staticTempBinIndex(11.00f) == 1);
    CHECK(ctx, staticTempBinIndex(57.99f) == 47);
    CHECK(ctx, staticTempBinIndex(58.00f) == -1);
    CHECK(ctx, staticTempBinIndex(std::numeric_limits<float>::quiet_NaN()) == -1);
}

static void testBlockedStatsPreserveSamples(TestContext& ctx) {
    CHECK(ctx, sizeof(StaticTestStatsBlock) <= 768u);
    StaticRuntimeTest target;
    StaticTestStatsBlock block;
    for (uint32_t i = 0; i < 64u; ++i) {
        const float value = static_cast<float>(i + 1u);
        CHECK(ctx, block.push(1000u + i,
                              0.75f,
                              25.25f,
                              Vec3(value, value * 2.0f, -value),
                              1.0f,
                              staticTempBinIndex(25.25f),
                              true));
    }
    block.mergeInto(target);
    CHECK(ctx, target.dtUs.count == 64u);
    CHECK(ctx, target.accelNormG.count == 64u);
    CHECK(ctx, target.gyroAfterRadS.count == 64u);
    const int bin = staticTempBinIndex(25.25f);
    CHECK(ctx, bin >= 0);
    CHECK(ctx, target.tempBins[bin].gyroAfterRadS.count == 64u);
    CHECK_NEAR(ctx, target.tempC.mean(), 25.25f, 0.0001f);
    CHECK_NEAR(ctx, target.gyroAfterRadS.mean().x, 32.5f, 0.001f);

    StaticTestStatsBlock sparse;
    for (int binIndex = 0; binIndex < StaticTestStatsBlock::TEMP_BIN_SLOTS; ++binIndex) {
        CHECK(ctx, sparse.push(1000u, 1.0f, 10.25f + static_cast<float>(binIndex),
                               Vec3::zero(), 1.0f, binIndex, true));
    }
    CHECK(ctx, !sparse.push(1000u, 1.0f, 20.25f, Vec3::zero(), 1.0f, 10, true));
    CHECK(ctx, sparse.bufferedSamples == StaticTestStatsBlock::TEMP_BIN_SLOTS);
}

static void testBlockedStatsNumericalBudget(TestContext& ctx) {
    StaticRuntimeTest blocked;
    ScalarStats directDt;
    ScalarStats directAccel;
    ScalarStats directTemp;
    Vec3Stats directGyro;
    StaticTestStatsBlock block;

    for (uint32_t i = 0u; i < 4096u; ++i) {
        const uint32_t dt = 1040u + (i % 9u);
        const float accel = 0.998f + static_cast<float>(i % 7u) * 0.0005f;
        const float temp = 25.20f + static_cast<float>(i % 11u) * 0.002f;
        const float phase = static_cast<float>(static_cast<int32_t>(i % 17u) - 8);
        const Vec3 gyro(0.0010f + phase * 0.00001f,
                        -0.0008f + phase * 0.000008f,
                        0.0002f - phase * 0.000006f);
        directDt.push(static_cast<float>(dt));
        directAccel.push(accel);
        directTemp.push(temp);
        directGyro.push(gyro);
        CHECK(ctx, block.push(dt, 0.95f, temp, gyro, accel,
                              staticTempBinIndex(temp), true));
        if (block.bufferedSamples == 64u) {
            block.mergeInto(blocked);
            block.reset();
        }
    }
    block.mergeInto(blocked);

    CHECK(ctx, blocked.dtUs.count == directDt.count);
    CHECK(ctx, blocked.gyroAfterRadS.count == directGyro.count);
    CHECK_NEAR(ctx, blocked.dtUs.mean(), directDt.mean(), 0.01f);
    CHECK_NEAR(ctx, blocked.dtUs.stddev(), directDt.stddev(), 0.02f);
    CHECK_NEAR(ctx, blocked.accelNormG.mean(), directAccel.mean(), 0.00001f);
    CHECK_NEAR(ctx, blocked.tempC.mean(), directTemp.mean(), 0.0001f);
    const Vec3 blockedMean = blocked.gyroAfterRadS.mean();
    const Vec3 directMean = directGyro.mean();
    CHECK_NEAR(ctx, blockedMean.x, directMean.x, 0.000001f);
    CHECK_NEAR(ctx, blockedMean.y, directMean.y, 0.000001f);
    CHECK_NEAR(ctx, blockedMean.z, directMean.z, 0.000001f);
}

int main() {
    TestContext ctx;
    testScalarStats(ctx);
    testVec3Stats(ctx);
    testTempBinStatsRejectBadQuality(ctx);
    testTempBins(ctx);
    testBlockedStatsPreserveSamples(ctx);
    testBlockedStatsNumericalBudget(ctx);
    return ctx.finish("test_static_stats");
}
