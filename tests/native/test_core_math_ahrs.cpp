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
    const uint32_t largeDtRebaseBefore = ahrs.stats().largeDtRebaseCount;
    CHECK(ctx, !ahrs.update(Vec3::zero(), Vec3::unitZ(), 2000ULL * 1000ULL));
    CHECK(ctx, ahrs.stats().skippedBadDt == skippedBefore + 1);
    CHECK(ctx, ahrs.stats().largeDtRebaseCount == largeDtRebaseBefore + 1);
    CHECK(ctx, ahrs.stats().lastRebaseTimestampUs == 2000ULL * 1000ULL);
    CHECK(ctx, ahrs.stats().lastTimestampUs == 2000ULL * 1000ULL);
    CHECK(ctx, ahrs.stats().lastIntegratedTimestampUs == 1000ULL * 1000ULL);
    CHECK(ctx, ahrs.stats().lastUsedDtS == 0.0f);

    // After a rejected multi-second gap, the next normal sample must integrate
    // again instead of staying permanently frozen against the pre-gap baseline.
    CHECK(ctx, ahrs.update(Vec3::zero(), Vec3::unitZ(), 2000ULL * 1000ULL + 1000ULL));

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
    CHECK(ctx, rejectLargeDt.stats().largeDtRebaseCount == 1);
    CHECK(ctx, rejectLargeDt.stats().lastRebaseTimestampUs == 101000);
    CHECK(ctx, rejectLargeDt.stats().lastTimestampUs == 101000);
    CHECK(ctx, rejectLargeDt.stats().lastIntegratedTimestampUs == 1000);
    CHECK(ctx, rejectLargeDt.stats().lastUsedDtS == 0.0f);
    CHECK(ctx, rejectLargeDt.update(Vec3::zero(), Vec3::unitZ(), 102000));
    CHECK(ctx, rejectLargeDt.stats().lastIntegratedTimestampUs == 102000);
}


static float horizontalHeading(const Vec3& v) {
    return std::atan2(v.y, v.x);
}

static void testAhrsTiltReacquisitionPreservesHeading(TestContext& ctx) {
    Ahrs6Dof ahrs;
    const Quat yaw = Quat::fromAxisAngle(Vec3::unitZ(), 60.0f * MATH_DEG_TO_RAD);
    ahrs.reset(yaw, 1000);

    const Quat trueQ = (yaw * Quat::fromAxisAngle(Vec3::unitX(), 28.0f * MATH_DEG_TO_RAD)).normalized();
    const Vec3 measuredAccel = trueQ.conjugated().rotate(Vec3::unitZ());
    const float headingBefore = horizontalHeading(ahrs.quaternion().rotate(Vec3::unitY()));

    CHECK(ctx, ahrs.reacquireTiltFromAccelPreserveHeading(measuredAccel, 5000));
    const Quat recovered = ahrs.quaternion();
    const Vec3 mappedUp = recovered.rotate(measuredAccel).normalized();
    CHECK_NEAR(ctx, mappedUp.x, 0.0f, 2.0e-4f);
    CHECK_NEAR(ctx, mappedUp.y, 0.0f, 2.0e-4f);
    CHECK_NEAR(ctx, mappedUp.z, 1.0f, 2.0e-4f);

    const float headingAfter = horizontalHeading(recovered.rotate(Vec3::unitY()));
    CHECK_NEAR(ctx, wrapPi(headingAfter - headingBefore), 0.0f, 2.0e-4f);
    CHECK(ctx, ahrs.stats().lastIntegratedTimestampUs == 5000);
    CHECK(ctx, !ahrs.reacquireTiltFromAccelPreserveHeading(Vec3(0.0f, 0.0f, 1.2f), 6000));

    // A 180-degree missed flip is gravity-observable but its horizontal axis
    // is ambiguous. The fallback keeps the current forward heading instead
    // of accepting the arbitrary axis chosen by a generic vector-to-vector
    // quaternion.
    Ahrs6Dof upsideDown;
    const Quat yaw35 = Quat::fromAxisAngle(Vec3::unitZ(), 35.0f * MATH_DEG_TO_RAD);
    upsideDown.reset(yaw35, 1000);
    const Vec3 flippedAccel = yaw35.conjugated().rotate(-Vec3::unitZ());
    const float flippedHeadingBefore = horizontalHeading(yaw35.rotate(Vec3::unitY()));
    CHECK(ctx, upsideDown.reacquireTiltFromAccelPreserveHeading(flippedAccel, 2000));
    const Quat flippedRecovered = upsideDown.quaternion();
    CHECK_NEAR(ctx, dot(flippedRecovered.rotate(flippedAccel), Vec3::unitZ()), 1.0f, 2.0e-4f);
    CHECK_NEAR(ctx,
               wrapPi(horizontalHeading(flippedRecovered.rotate(Vec3::unitY())) - flippedHeadingBefore),
               0.0f,
               2.0e-4f);
}

static void testAhrsRecoveryRebaseDiagnostics(TestContext& ctx) {
    Ahrs6DofConfig cfg;
    cfg.clampLargeDt = false;

    Ahrs6Dof ahrs(cfg);
    CHECK(ctx, !ahrs.update(Vec3::zero(), Vec3::unitZ(), 1000));
    CHECK(ctx, ahrs.update(Vec3::zero(), Vec3::unitZ(), 2000));

    const uint32_t updatesBefore = ahrs.stats().updateCount;
    ahrs.rebaseTimestamp(5000);
    CHECK(ctx, ahrs.stats().fifoRecoveryRebaseCount == 1);
    CHECK(ctx, ahrs.stats().lastRebaseTimestampUs == 5000);
    CHECK(ctx, ahrs.stats().postFifoRecoverySamples == 0);
    CHECK(ctx, ahrs.stats().lastTimestampUs == 5000);
    CHECK(ctx, ahrs.stats().lastIntegratedTimestampUs == 2000);

    CHECK(ctx, ahrs.update(Vec3::zero(), Vec3::unitZ(), 6000));
    CHECK(ctx, ahrs.stats().lastIntegratedTimestampUs == 6000);
    CHECK(ctx, ahrs.stats().postFifoRecoverySamples == 1);
    CHECK(ctx, ahrs.stats().updateCount == updatesBefore + 1);

    CHECK(ctx, ahrs.update(Vec3::zero(), Vec3::unitZ(), 7000));
    CHECK(ctx, ahrs.stats().postFifoRecoverySamples == 2);
}

static float quatAngularErrorRad(const Quat& aIn, const Quat& bIn) {
    const Quat a = aIn.normalized();
    const Quat b = bIn.normalized();
    const float d = std::fabs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z);
    return 2.0f * std::acos(clampf(d, 0.0f, 1.0f));
}

static void testFastQuaternionMathAccuracy(TestContext& ctx) {
    float maxStepError = 0.0f;
    for (int i = 0; i <= 400; ++i) {
        const float angle = -0.2f + 0.001f * static_cast<float>(i);
        const Vec3 rv = Vec3(0.31f, -0.72f, 0.62f).normalized() * angle;
        const Quat exact = Quat::fromRotationVector(rv);
        const Quat fast = quaternionFromRotationVectorFast(rv);
        const float err = quatAngularErrorRad(exact, fast);
        if (err > maxStepError) maxStepError = err;
        CHECK_NEAR(ctx, fast.norm(), 1.0f, 2.0e-6f);
    }
    CHECK(ctx, maxStepError < 1.0e-5f);

    float maxAcosError = 0.0f;
    for (int i = 0; i <= 2000; ++i) {
        const float x = -1.0f + 0.001f * static_cast<float>(i);
        const float err = std::fabs(std::acos(x) - acosFastUnitDot(x));
        if (err > maxAcosError) maxAcosError = err;
    }
    CHECK(ctx, maxAcosError < 8.0e-5f);
}

static void testFastGyroIntegrationTwoHours(TestContext& ctx) {
    constexpr uint32_t rateHz = 960;
    constexpr uint32_t seconds = 2u * 60u * 60u;
    const uint64_t samples = static_cast<uint64_t>(rateHz) * seconds;
    const float dt = 1.0f / static_cast<float>(rateHz);
    const Vec3 axis = Vec3(0.37f, -0.51f, 0.776f).normalized();
    const Vec3 gyro = axis * (1000.0f * MATH_DEG_TO_RAD);

    Quat fast = Quat::identity();
    Quat previousExact = Quat::identity();
    for (uint64_t i = 0; i < samples; ++i) {
        fast = integrateBodyRateFast(fast, gyro, dt);
        previousExact = integrateBodyRate(previousExact, gyro, dt);
        if ((i & 15u) == 15u) fast.normalizeInPlace();
    }
    fast.normalizeInPlace();

    // This is an intentionally extreme trajectory: two hours of continuous
    // 1000 dps rotation (20,000 full turns). The optimized path must remain
    // visually indistinguishable from the previous exact-per-sample path.
    const float differenceDeg = quatAngularErrorRad(previousExact, fast) * MATH_RAD_TO_DEG;
    CHECK_NEAR(ctx, fast.norm(), 1.0f, 2.0e-6f);
    CHECK(ctx, differenceDeg < 0.15f);
}

static void testDecimatedAccelCorrectionConverges(TestContext& ctx) {
    Ahrs6DofConfig cfg;
    cfg.normalizeEvery = 16;
    cfg.accelKp = 3.0f;
    Ahrs6Dof ahrs(cfg);
    ahrs.reset(Quat::fromAxisAngle(Vec3::unitX(), 25.0f * MATH_DEG_TO_RAD), 1000);

    uint64_t timestamp = 1000;
    for (uint32_t i = 0; i < 9600; ++i) {
        timestamp += 1042;
        CHECK(ctx, ahrs.update(Vec3::zero(), Vec3::unitZ(), 1.0f, timestamp));
    }

    const Vec3 mappedUp = ahrs.quaternion().rotate(Vec3::unitZ()).normalized();
    const float tiltErrorDeg = std::acos(clampf(dot(mappedUp, Vec3::unitZ()), -1.0f, 1.0f)) * MATH_RAD_TO_DEG;
    CHECK(ctx, tiltErrorDeg < 0.05f);
    CHECK_NEAR(ctx, ahrs.quaternion().norm(), 1.0f, 2.0e-5f);
    CHECK(ctx, ahrs.stats().accelUpdateCount > 2000u);
    CHECK(ctx, ahrs.stats().accelUpdateCount < 2500u);
}

static void testTwoHourStaticTiltRemainsBounded(TestContext& ctx) {
    Ahrs6DofConfig cfg;
    cfg.normalizeEvery = 16;
    cfg.accelKp = 3.0f;
    Ahrs6Dof ahrs(cfg);
    ahrs.reset(Quat::identity(), 1000);

    constexpr uint64_t samples = 960ULL * 2ULL * 60ULL * 60ULL;
    const Vec3 gyroBias(0.03f * MATH_DEG_TO_RAD,
                       -0.02f * MATH_DEG_TO_RAD,
                       0.01f * MATH_DEG_TO_RAD);
    uint64_t timestamp = 1000;
    uint32_t rng = 0x13579BDFu;
    for (uint64_t i = 0; i < samples; ++i) {
        timestamp += 1042;
        rng = rng * 1664525u + 1013904223u;
        const float nx = (static_cast<float>(rng >> 8) * (1.0f / 16777215.0f) - 0.5f) * 0.004f;
        rng = rng * 1664525u + 1013904223u;
        const float ny = (static_cast<float>(rng >> 8) * (1.0f / 16777215.0f) - 0.5f) * 0.004f;
        const Vec3 accel(nx, ny, 1.0f);
        CHECK(ctx, ahrs.update(gyroBias, accel, accel.norm(), timestamp));
    }

    const Vec3 mappedUp = ahrs.quaternion().rotate(Vec3::unitZ()).normalized();
    const float tiltErrorDeg =
        std::acos(clampf(dot(mappedUp, Vec3::unitZ()), -1.0f, 1.0f)) * MATH_RAD_TO_DEG;
    CHECK(ctx, tiltErrorDeg < 0.10f);
    CHECK_NEAR(ctx, ahrs.quaternion().norm(), 1.0f, 2.0e-5f);
}

int main() {
    TestContext ctx;
    testVecMatQuat(ctx);
    testAhrsStaticInvariants(ctx);
    testAhrsStartupAndDtPolicy(ctx);
    testAhrsRecoveryRebaseDiagnostics(ctx);
    testAhrsTiltReacquisitionPreservesHeading(ctx);
    testFastQuaternionMathAccuracy(ctx);
    testFastGyroIntegrationTwoHours(ctx);
    testTwoHourStaticTiltRemainsBounded(ctx);
    testDecimatedAccelCorrectionConverges(ctx);
    return ctx.finish("test_core_math_ahrs");
}
