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

static void testTempBins(TestContext& ctx) {
    CHECK(ctx, staticTempBinIndex(9.99f) == -1);
    CHECK(ctx, staticTempBinIndex(10.00f) == 0);
    CHECK(ctx, staticTempBinIndex(10.99f) == 0);
    CHECK(ctx, staticTempBinIndex(11.00f) == 1);
    CHECK(ctx, staticTempBinIndex(57.99f) == 47);
    CHECK(ctx, staticTempBinIndex(58.00f) == -1);
    CHECK(ctx, staticTempBinIndex(std::numeric_limits<float>::quiet_NaN()) == -1);
}

int main() {
    TestContext ctx;
    testScalarStats(ctx);
    testVec3Stats(ctx);
    testTempBins(ctx);
    return ctx.finish("test_static_stats");
}
