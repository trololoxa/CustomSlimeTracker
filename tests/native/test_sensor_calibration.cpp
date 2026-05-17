#include "test_common.hpp"

#include "sensor/calibration.hpp"
#include "sensor/accel_6pos_calibration.hpp"

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


static void testAccel6PosFull3x3Calibration(TestContext& ctx) {
    Accel6PosCalibration cal;

    const Vec3 bias(0.035f, -0.020f, 0.045f);
    const Mat3 correction(
        1.040f, -0.030f,  0.020f,
        0.015f,  0.970f, -0.025f,
       -0.018f,  0.012f,  1.060f
    );
    Mat3 rawBasis;
    CHECK(ctx, correction.inverse(rawBasis));

    for (uint8_t i = 0; i < 6; ++i) {
        const auto face = static_cast<Accel6PosCalibration::Face>(i);
        const Vec3 expected = Accel6PosCalibration::expectedVector(face);
        const Vec3 raw = bias + rawBasis * expected;
        CHECK(ctx, cal.setFace(face, raw, 1024, Vec3(1.0e-6f, 1.0e-6f, 1.0e-6f)));
    }

    CHECK(ctx, cal.compute());
    const auto& r = cal.result();
    CHECK(ctx, r.valid);
    CHECK_NEAR(ctx, r.biasG.x, bias.x, 1.0e-5f);
    CHECK_NEAR(ctx, r.biasG.y, bias.y, 1.0e-5f);
    CHECK_NEAR(ctx, r.biasG.z, bias.z, 1.0e-5f);

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            CHECK_NEAR(ctx, r.scaleMatrix.m[row][col], correction.m[row][col], 2.0e-5f);
        }
    }

    const Vec3 rawTest = bias + rawBasis * Vec3(0.25f, -0.50f, 0.75f);
    const Vec3 corrected = Accel6PosCalibration::apply(rawTest, r);
    CHECK_NEAR(ctx, corrected.x, 0.25f, 2.0e-5f);
    CHECK_NEAR(ctx, corrected.y, -0.50f, 2.0e-5f);
    CHECK_NEAR(ctx, corrected.z, 0.75f, 2.0e-5f);
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
    testAccel6PosFull3x3Calibration(ctx);
    testStationaryStats(ctx);
    testStationaryDetectorAndGyroStartup(ctx);
    testOnlineGyroBiasEstimator(ctx);
    return ctx.finish("test_sensor_calibration");
}
