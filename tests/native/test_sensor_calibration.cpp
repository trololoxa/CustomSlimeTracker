#include "test_common.hpp"

#include "sensor/calibration.hpp"

using namespace tracker;

static Lsm6dsv::Sample makeSample(const Vec3& gyroRadS, const Vec3& accelG) {
    Lsm6dsv::Sample s;
    s.gyro_rad_s = gyroRadS;
    s.accel_g = accelG;
    return s;
}

static void testImuCalibrationApply(TestContext& ctx) {
    ImuCalibration cal;
    cal.gyroBiasValid = true;
    cal.gyroBiasRadS = Vec3(0.1f, -0.2f, 0.3f);
    cal.accelCalValid = true;
    cal.accelBiasG = Vec3(0.01f, -0.02f, 0.03f);
    cal.accelScale = Mat3::diagonal(2.0f, 3.0f, 4.0f);

    const Lsm6dsv::Sample in = makeSample(Vec3(1.0f, 2.0f, 3.0f), Vec3(0.5f, 0.5f, 0.5f));
    const Lsm6dsv::Sample out = cal.apply(in);

    CHECK_NEAR(ctx, out.gyro_rad_s.x, 0.9f, 1.0e-6f);
    CHECK_NEAR(ctx, out.gyro_rad_s.y, 2.2f, 1.0e-6f);
    CHECK_NEAR(ctx, out.gyro_rad_s.z, 2.7f, 1.0e-6f);
    CHECK_NEAR(ctx, out.accel_g.x, 0.98f, 1.0e-6f);
    CHECK_NEAR(ctx, out.accel_g.y, 1.56f, 1.0e-6f);
    CHECK_NEAR(ctx, out.accel_g.z, 1.88f, 1.0e-6f);
}

static void testStationaryStats(TestContext& ctx) {
    StationaryStats stats;
    stats.push(makeSample(Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 1.0f)));
    stats.push(makeSample(Vec3(3.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 1.0f)));

    CHECK(ctx, stats.count == 2);
    CHECK_NEAR(ctx, stats.gyroMeanRadS.x, 2.0f, 1.0e-6f);
    CHECK_NEAR(ctx, stats.gyroVarianceRadS2().x, 2.0f, 1.0e-6f);
    CHECK_NEAR(ctx, stats.accelNormMeanG, 1.0f, 1.0e-6f);

    stats.reset();
    CHECK(ctx, stats.count == 0);
    CHECK_NEAR(ctx, stats.gyroVarianceNormRadS2(), 0.0f, 1.0e-6f);
}

static void testStationaryDetectorAndGyroStartup(TestContext& ctx) {
    GyroStartupCalibrationParams params;
    params.requiredStationarySamples = 8;
    params.maxTotalSamples = 64;
    params.stationary.warmupSamples = 2;
    params.stationary.maxGyroNormRadS = 1.0f * MATH_DEG_TO_RAD;
    params.stationary.maxAccelNormErrorG = 0.05f;
    params.stationary.maxGyroVarianceRadS2 = square(0.1f * MATH_DEG_TO_RAD);
    params.stationary.maxAccelVarianceG2 = square(0.002f);

    GyroStartupCalibrator cal(params);
    const Vec3 bias = Vec3(0.2f, -0.1f, 0.05f) * MATH_DEG_TO_RAD;
    bool done = false;
    for (int i = 0; i < 16; ++i) {
        done = cal.push(makeSample(bias, Vec3(0.0f, 0.0f, 1.0f)));
        if (done) break;
    }

    CHECK(ctx, done);
    CHECK(ctx, cal.done());
    CHECK(ctx, cal.result().success);
    CHECK(ctx, cal.result().stationarySamples >= params.requiredStationarySamples);
    CHECK_NEAR(ctx, cal.result().gyroBiasDps.x, 0.2f, 1.0e-5f);
    CHECK_NEAR(ctx, cal.result().gyroBiasDps.y, -0.1f, 1.0e-5f);
    CHECK_NEAR(ctx, cal.result().accelNormMeanG, 1.0f, 1.0e-6f);
}

static void testOnlineGyroBiasEstimator(TestContext& ctx) {
    OnlineGyroBiasEstimator estimator(0.5f);
    CHECK(ctx, !estimator.initialized());

    estimator.updateIfStationary(Vec3(1.0f, 0.0f, 0.0f), false);
    CHECK(ctx, !estimator.initialized());

    estimator.updateIfStationary(Vec3(1.0f, 0.0f, 0.0f), true);
    CHECK(ctx, estimator.initialized());
    CHECK_NEAR(ctx, estimator.biasRadS().x, 1.0f, 1.0e-6f);

    estimator.updateIfStationary(Vec3(3.0f, 0.0f, 0.0f), true);
    CHECK_NEAR(ctx, estimator.biasRadS().x, 2.0f, 1.0e-6f);
}

int main() {
    TestContext ctx;
    testImuCalibrationApply(ctx);
    testStationaryStats(ctx);
    testStationaryDetectorAndGyroStartup(ctx);
    testOnlineGyroBiasEstimator(ctx);
    return ctx.finish("test_sensor_calibration");
}
