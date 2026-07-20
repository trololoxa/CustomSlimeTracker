#include "test_common.hpp"

#include "config/tracker_config_runtime.hpp"
#include "runtime/runtime_gyro_bias_controller.hpp"
#include "sensor/frame_transform.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/mag_runtime.hpp"

using namespace tracker;

static Mat3 rotateSensorXToDeviceY() {
    // +90 degrees around +Z: sensor +X -> device +Y.
    return Mat3(
         0.0f, -1.0f, 0.0f,
         1.0f,  0.0f, 0.0f,
         0.0f,  0.0f, 1.0f
    );
}

static void testFrameValidationAndThreeSensorAgreement(TestContext& ctx) {
    const Mat3 r = rotateSensorXToDeviceY();
    CHECK(ctx, isProperRotationMatrix(r));
    const Mat3 scale = Mat3::diagonal(2.0f, 1.0f, 1.0f);
    CHECK(ctx, !isProperRotationMatrix(scale));
    CHECK(ctx, !isProperRotationMatrix(Mat3::diagonal(-1.0f, 1.0f, 1.0f)));
    CHECK(ctx, !makeSensorToDeviceFrame(true, scale).enabled);

    TrackerConfig cfg;
    cfg.resetDefaults();
    cfg.data.frame.sensorToDeviceValid = true;
    cfg.data.frame.sensorToDevice = r;
    cfg.updateCrc();
    CHECK(ctx, cfg.validate());

    const SensorToDeviceFrame frame = makeSensorToDeviceFrame(true, r);
    const Vec3 gyroDevice = frame.apply(Vec3::unitX());
    const Vec3 accelDevice = frame.apply(Vec3::unitX());
    CHECK_NEAR(ctx, gyroDevice.x, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, gyroDevice.y, 1.0f, 1.0e-6f);
    CHECK_NEAR(ctx, accelDevice.y, 1.0f, 1.0e-6f);
    CHECK_NEAR(ctx, frame.inverseApply(gyroDevice).x, 1.0f, 1.0e-6f);

    MagRuntimeProcessor mag;
    MagRuntimeConfig mc;
    mc.enabled = true;
    mc.calibrationValid = true;
    mc.axisAlignmentValid = true;
    mc.sensorToDeviceValid = true;
    mc.sensorToDevice = r;
    mc.minTrustNorm = 0.1f;
    mc.maxTrustNorm = 10.0f;

    Lsm6dsvFifoReader::MagRawSample raw;
    raw.x = 1;
    raw.y = 0;
    raw.z = 0;
    raw.t_us = 1000;
    MagProcessedSample out;
    CHECK(ctx, mag.process(raw, mc, 10, out));
    CHECK(ctx, out.trusted);
    CHECK_NEAR(ctx, out.body.x, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, out.body.y, 1.0f, 1.0e-6f);
}

static void testInvalidFrameIsSanitizedWithoutTouchingCalibration(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();
    cfg.data.frame.sensorToDeviceValid = true;
    cfg.data.frame.sensorToDevice = Mat3::diagonal(2.0f, 1.0f, 1.0f);
    cfg.data.gyroCal.biasValid = true;
    cfg.data.gyroCal.biasRadS = Vec3(0.01f, 0.02f, 0.03f);
    cfg.updateCrc();
    // A finite legacy matrix remains loadable, but runtime ignores it until
    // sanitization replaces it with identity. Other calibration survives.
    CHECK(ctx, cfg.validate());
    CHECK(ctx, !makeSensorToDeviceFrame(true, cfg.data.frame.sensorToDevice).enabled);
    cfg.sanitize();

    CHECK(ctx, !cfg.data.frame.sensorToDeviceValid);
    CHECK_NEAR(ctx, cfg.data.frame.sensorToDevice.determinant(), 1.0f, 1.0e-6f);
    CHECK(ctx, cfg.data.gyroCal.biasValid);
    CHECK_NEAR(ctx, cfg.data.gyroCal.biasRadS.y, 0.02f, 1.0e-6f);
}

static void testGyroTemperatureLifecycleAndPersistence(TestContext& ctx) {
    GyroTempCompensator comp;
    comp.setModel(Vec3(1.0f, 2.0f, 3.0f) * MATH_DEG_TO_RAD,
                  30.0f,
                  Vec3(0.01f, -0.02f, 0.03f) * MATH_DEG_TO_RAD);
    comp.setQualityMetadata(20.0f, 40.0f, 0.9f, 0.2f, 0.03f);
    CHECK(ctx, comp.valid());
    CHECK(ctx, comp.temperatureModelValid());
    CHECK(ctx, comp.snapshot(35.0f).hasCalibratedRange);

    // Replacing the static bias must never silently retain old thermal slope
    // or quality metadata.
    comp.setStaticBias(Vec3(4.0f, 5.0f, 6.0f) * MATH_DEG_TO_RAD, 35.0f);
    CHECK(ctx, comp.valid());
    CHECK(ctx, !comp.temperatureModelValid());
    CHECK_NEAR(ctx, comp.slopeDpsPerC().norm(), 0.0f, 1.0e-7f);
    CHECK(ctx, !comp.snapshot(35.0f).hasCalibratedRange);

    TrackerConfig cfg;
    cfg.resetDefaults();
    cfg.captureFromGyroTempComp(comp);
    CHECK(ctx, cfg.data.gyroCal.biasValid);
    CHECK(ctx, !cfg.data.gyroCal.tempCompValid);
    CHECK_NEAR(ctx, cfg.data.gyroCal.tempSlopeRadSPerC.norm(), 0.0f, 1.0e-9f);
    CHECK_NEAR(ctx, cfg.data.gyroTempQuality.fitQuality, 0.0f, 1.0e-9f);

    GyroTempCompensator restored;
    cfg.applyToGyroTempComp(restored);
    CHECK(ctx, restored.valid());
    CHECK(ctx, !restored.temperatureModelValid());
    CHECK_NEAR(ctx, restored.referenceBiasDps().x, 4.0f, 1.0e-5f);

    // A missing runtime temperature model must not erase the independent
    // static gyro calibration captured by ImuCalibration. Explicit clear
    // commands own removal of the static bias.
    restored.clearAll();
    cfg.captureFromGyroTempComp(restored);
    CHECK(ctx, cfg.data.gyroCal.biasValid);
    CHECK(ctx, !cfg.data.gyroCal.tempCompValid);
    CHECK_NEAR(ctx, cfg.data.gyroCal.tempSlopeRadSPerC.norm(), 0.0f, 1.0e-9f);
}

static void testRuntimeTrimRemainsSensorFrame(TestContext& ctx) {
    ImuCalibration imu;
    imu.gyroBiasValid = true;
    imu.gyroBiasRadS = Vec3(0.10f, 0.0f, 0.0f);
    GyroTempCompensator temp;
    temp.setStaticBias(imu.gyroBiasRadS, 25.0f);
    RuntimeGyroBiasEstimator runtime;
    runtime.runtimeTrimRadS = Vec3(0.01f, 0.0f, 0.0f);

    const Vec3 sensorBias = runtimeBiasCurrentGyroBiasRadS(runtime, imu, temp, 25.0f);
    CHECK_NEAR(ctx, sensorBias.x, 0.11f, 1.0e-6f);
    const Vec3 deviceBias = rotateSensorXToDeviceY() * sensorBias;
    CHECK_NEAR(ctx, deviceBias.x, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, deviceBias.y, 0.11f, 1.0e-6f);
}

int main() {
    TestContext ctx;
    testFrameValidationAndThreeSensorAgreement(ctx);
    testInvalidFrameIsSanitizedWithoutTouchingCalibration(ctx);
    testGyroTemperatureLifecycleAndPersistence(ctx);
    testRuntimeTrimRemainsSensorFrame(ctx);
    return ctx.finish("test_frame_calibration_lifecycle");
}
