#include "test_common.hpp"

#include "core/math.hpp"
#include "sensor/ahrs_6dof.hpp"

using namespace tracker;

static void testVecMatQuat(TestContext& ctx) {
    const Vec3 a(1.0f, 2.0f, 3.0f);
    const Vec3 b(-4.0f, 5.0f, -6.0f);

    CHECK_NEAR(ctx, dot(a, b), -12.0f, 1.0e-5f);

    const Vec3 c = cross(Vec3::unitX(), Vec3::unitY());
    CHECK_NEAR(ctx, c.x, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, c.y, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, c.z, 1.0f, 1.0e-6f);

    const Mat3 scale = Mat3::diagonal(2.0f, 3.0f, 4.0f);
    const Vec3 scaled = scale * Vec3(1.0f, -2.0f, 0.5f);
    CHECK_NEAR(ctx, scaled.x, 2.0f, 1.0e-6f);
    CHECK_NEAR(ctx, scaled.y, -6.0f, 1.0e-6f);
    CHECK_NEAR(ctx, scaled.z, 2.0f, 1.0e-6f);

    const Quat qz90 = Quat::fromAxisAngle(Vec3::unitZ(), 0.5f * MATH_PI).normalized();
    const Vec3 rotated = qz90.rotate(Vec3::unitX());
    CHECK_NEAR(ctx, rotated.x, 0.0f, 2.0e-5f);
    CHECK_NEAR(ctx, rotated.y, 1.0f, 2.0e-5f);
    CHECK_NEAR(ctx, rotated.z, 0.0f, 2.0e-5f);
    CHECK_NEAR(ctx, qz90.norm(), 1.0f, 1.0e-6f);

    const Quat zToX = Quat::fromTwoVectors(Vec3::unitZ(), Vec3::unitX());
    const Vec3 mapped = zToX.rotate(Vec3::unitZ());
    CHECK_NEAR(ctx, mapped.x, 1.0f, 2.0e-5f);
    CHECK_NEAR(ctx, mapped.y, 0.0f, 2.0e-5f);
    CHECK_NEAR(ctx, mapped.z, 0.0f, 2.0e-5f);

    CHECK_NEAR(ctx, wrapPi(3.0f * MATH_PI), MATH_PI, 1.0e-5f);
    CHECK_NEAR(ctx, wrapPi(-3.0f * MATH_PI), -MATH_PI, 1.0e-5f);
}

static void testAhrsStaticInvariants(TestContext& ctx) {
    Ahrs6DofConfig cfg;
    cfg.clampLargeDt = false;
    cfg.normalizeEvery = 8;

    Ahrs6Dof ahrs(cfg);
    CHECK(ctx, !ahrs.initialized());

    // First valid sample initializes from accel and intentionally returns false:
    // no gyro integration happens on that first timestamp.
    CHECK(ctx, !ahrs.update(Vec3::zero(), Vec3::unitZ(), 1000));
    CHECK(ctx, ahrs.initialized());

    for (uint32_t i = 2; i <= 1000; ++i) {
        CHECK(ctx, ahrs.update(Vec3::zero(), Vec3::unitZ(), static_cast<uint64_t>(i) * 1000ULL));
    }

    const Quat q = ahrs.quaternion();
    CHECK(ctx, q.isFinite());
    CHECK_NEAR(ctx, q.norm(), 1.0f, 2.0e-5f);

    const Vec3 up = q.rotate(Vec3::unitZ());
    CHECK_NEAR(ctx, up.x, 0.0f, 2.0e-4f);
    CHECK_NEAR(ctx, up.y, 0.0f, 2.0e-4f);
    CHECK_NEAR(ctx, up.z, 1.0f, 2.0e-4f);

    const uint32_t skippedBefore = ahrs.stats().skippedBadDt;
    CHECK(ctx, !ahrs.update(Vec3::zero(), Vec3::unitZ(), 1000ULL * 1000ULL));
    CHECK(ctx, ahrs.stats().skippedBadDt == skippedBefore + 1);

    CHECK(ctx, ahrs.stats().updateCount > 0);
    CHECK(ctx, ahrs.stats().accelUpdateCount > 0);
}


static void testAhrsStartupAndDtPolicy(TestContext& ctx) {
    Ahrs6DofConfig cfg;
    cfg.maxDtS = 0.010f;
    cfg.clampLargeDt = true;
    cfg.normalizeEvery = 1;

    Ahrs6Dof ahrs(cfg);
    CHECK(ctx, !ahrs.resetFromAccel(Vec3(0.0f, 0.0f, 2.0f), 1000));
    CHECK(ctx, !ahrs.initialized());

    // update() with impossible accel should count a startup rejection and stay uninitialized.
    CHECK(ctx, !ahrs.update(Vec3::zero(), Vec3(0.0f, 0.0f, 2.0f), 1000));
    CHECK(ctx, !ahrs.initialized());
    CHECK(ctx, ahrs.stats().startupAccelRejectedCount == 1);

    CHECK(ctx, !ahrs.update(Vec3::zero(), Vec3::unitZ(), 2000));
    CHECK(ctx, ahrs.initialized());

    CHECK(ctx, ahrs.update(Vec3(0.0f, 0.0f, 1.0f), Vec3::unitZ(), 102000));
    CHECK(ctx, ahrs.stats().clampedLargeDt == 1);
    CHECK_NEAR(ctx, ahrs.stats().lastDtS, 0.100f, 1.0e-6f);
    CHECK_NEAR(ctx, ahrs.stats().lastUsedDtS, 0.010f, 1.0e-6f);

    Ahrs6Dof rejectLargeDt(cfg);
    Ahrs6DofConfig rejectCfg = cfg;
    rejectCfg.clampLargeDt = false;
    rejectLargeDt.setConfig(rejectCfg);
    CHECK(ctx, !rejectLargeDt.update(Vec3::zero(), Vec3::unitZ(), 1000));
    CHECK(ctx, !rejectLargeDt.update(Vec3::zero(), Vec3::unitZ(), 101000));
    CHECK(ctx, rejectLargeDt.stats().skippedBadDt == 1);
    CHECK(ctx, rejectLargeDt.stats().lastIntegratedTimestampUs == 1000);
}

int main() {
    TestContext ctx;
    testVecMatQuat(ctx);
    testAhrsStaticInvariants(ctx);
    testAhrsStartupAndDtPolicy(ctx);
    return ctx.finish("test_core_math_ahrs");
}
