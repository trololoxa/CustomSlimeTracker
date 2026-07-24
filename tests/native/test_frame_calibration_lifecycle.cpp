#include "test_common.hpp"

#include <limits>

#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_storage.hpp"
#include "runtime/runtime_gyro_bias_controller.hpp"
#include "sensor/frame_transform.hpp"
#include "sensor/accel_6pos_calibration.hpp"
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
    restored.setEnabled(false);
    cfg.captureFromGyroTempComp(restored);
    CHECK(ctx, cfg.data.gyroCal.biasValid);
    CHECK(ctx, !cfg.data.gyroCal.tempCompValid);
    CHECK(ctx, !cfg.data.gyroCal.tempCompEnabled);
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

static void testCalibrationSnapshotAndRevisionSemantics(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();

    GyroTempCompensator comp;
    comp.setModel(Vec3(0.01f, -0.02f, 0.03f),
                  31.0f,
                  Vec3(0.0001f, -0.0002f, 0.0003f));
    comp.setQualityMetadata(20.0f, 42.0f, 0.95f, 0.20f, 0.03f);
    cfg.captureFromGyroTempCompUpdate(comp, 12345u, 777u);
    const uint32_t originalCrc = cfg.data.crc32;
    const uint32_t originalRevision = trackerCalibrationPayloadRevision(cfg);

    // A save/staging snapshot must not pretend that the existing model was
    // fitted again.
    cfg.captureFromGyroTempComp(comp);
    CHECK(ctx, cfg.data.gyroCalMeta.tempModelUpdatedUptimeMs == 12345u);
    CHECK(ctx, cfg.data.gyroCalMeta.tempModelSampleCount == 777u);
    CHECK(ctx, cfg.data.crc32 == originalCrc);
    CHECK(ctx, trackerCalibrationPayloadRevision(cfg) == originalRevision);

    GyroTempCompensator externallyChanged = comp;
    externallyChanged.setSlopeRadSPerC(Vec3(0.0002f, -0.0002f, 0.0003f));
    TrackerConfig unknownEvent = cfg;
    unknownEvent.captureFromGyroTempComp(externallyChanged);
    CHECK(ctx, unknownEvent.data.gyroCalMeta.tempModelUpdatedUptimeMs == 0u);
    CHECK(ctx, unknownEvent.data.gyroCalMeta.tempModelSampleCount == 0u);

    // Enable state and evidence are policy/history, not model freshness.
    TrackerConfig policyChanged = cfg;
    policyChanged.data.gyroCal.tempCompEnabled = false;
    policyChanged.data.gyroCalMeta.tempModelUpdatedUptimeMs = 99999u;
    policyChanged.data.gyroTempQuality.fitQuality = 0.91f;
    policyChanged.updateCrc();
    CHECK(ctx, trackerCalibrationModelEqual(cfg, policyChanged));
    CHECK(ctx, !trackerCalibrationEvidenceEqual(cfg, policyChanged));
    CHECK(ctx, trackerCalibrationPayloadRevision(cfg) ==
               trackerCalibrationPayloadRevision(policyChanged));
    CHECK(ctx, trackerCalibrationPayloadRevisionLegacyV2(cfg) !=
               trackerCalibrationPayloadRevisionLegacyV2(policyChanged));

    TrackerConfig modelChanged = cfg;
    modelChanged.data.gyroCal.tempSlopeRadSPerC.x += 0.00001f;
    modelChanged.updateCrc();
    CHECK(ctx, !trackerCalibrationModelEqual(cfg, modelChanged));
    CHECK(ctx, trackerCalibrationPayloadRevision(cfg) !=
               trackerCalibrationPayloadRevision(modelChanged));

    TrackerConfig active = cfg;
    active.data.gyroCal.tempCompEnabled = false;
    active.data.magCal.driverEnabled = true;
    active.data.magCal.minTrustNorm = 0.4f;
    active.data.magCal.maxTrustNorm = 1.8f;
    active.updateCrc();

    TrackerConfig candidate = cfg;
    candidate.data.gyroCal.tempCompEnabled = true;
    candidate.data.magCal.calibrationValid = true;
    candidate.data.magCal.expectedFieldNorm = 321.0f;
    candidate.data.magCal.minTrustNorm = 0.72f;
    candidate.data.magCal.maxTrustNorm = 1.28f;
    candidate.updateCrc();

    TrackerConfig composed = trackerComposeCalibrationCandidate(active, candidate);
    CHECK(ctx, !composed.data.gyroCal.tempCompEnabled);
    CHECK(ctx, composed.data.magCal.driverEnabled);
    CHECK_NEAR(ctx, composed.data.magCal.expectedFieldNorm, 321.0f, 1.0e-6f);
    CHECK_NEAR(ctx, composed.data.magCal.minTrustNorm, 0.72f, 1.0e-6f);
    CHECK_NEAR(ctx, composed.data.magCal.maxTrustNorm, 1.28f, 1.0e-6f);
}


static void testSanitizeInvalidatesNonFiniteMagCalibration(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.data.accelCal.valid = true;
    cfg.data.magCal.driverEnabled = true;
    cfg.data.magCal.calibrationValid = true;
    cfg.data.magCal.axisAlignmentValid = true;
    cfg.data.magCal.hardIron = Vec3(1.0f, std::numeric_limits<float>::quiet_NaN(), 3.0f);
    cfg.data.magCal.softIron = Mat3::identity();
    cfg.data.magCal.magToImu = Mat3::identity();
    cfg.data.magCal.expectedFieldNorm = 420.0f;
    cfg.data.magCalQuality.sampleCount = 1000u;
    cfg.data.magYaw.applyEnabled = true;
    cfg.sanitize();

    CHECK(ctx, cfg.data.magCal.driverEnabled);
    CHECK(ctx, !cfg.data.magCal.calibrationValid);
    CHECK(ctx, cfg.data.magCal.hardIron.x == 0.0f);
    CHECK(ctx, cfg.data.magCal.hardIron.y == 0.0f);
    CHECK(ctx, cfg.data.magCal.hardIron.z == 0.0f);
    CHECK(ctx, cfg.data.magCalQuality.sampleCount == 0u);
    CHECK(ctx, !cfg.data.magYaw.applyEnabled);

    cfg.data.magCal.calibrationValid = true;
    cfg.data.magCal.axisAlignmentValid = true;
    cfg.data.magCal.hardIron = Vec3::zero();
    cfg.data.magCal.softIron = Mat3::identity();
    cfg.data.magCal.magToImu = Mat3::identity();
    cfg.data.magCal.magToImu.m[0][0] = std::numeric_limits<float>::infinity();
    cfg.data.magYaw.applyEnabled = true;
    cfg.sanitize();
    CHECK(ctx, cfg.data.magCal.calibrationValid);
    CHECK(ctx, !cfg.data.magCal.axisAlignmentValid);
    CHECK(ctx, cfg.data.magCal.magToImu.m[0][0] == 1.0f);
    CHECK(ctx, !cfg.data.magYaw.applyEnabled);
}

static void testSanitizeRejectsTemperatureModelWithoutGyroBias(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();
    cfg.data.gyroCal.biasValid = false;
    cfg.data.gyroCal.tempCompValid = true;
    cfg.data.gyroCal.referenceTempC = 31.0f;
    cfg.data.gyroCal.tempSlopeRadSPerC = Vec3(0.001f, -0.002f, 0.003f);
    cfg.data.gyroTempQuality.fitQuality = 0.9f;
    cfg.data.gyroCalMeta.tempModelUpdatedUptimeMs = 1234u;
    cfg.data.gyroCalMeta.tempModelSampleCount = 567u;
    cfg.sanitize();

    CHECK(ctx, !cfg.data.gyroCal.tempCompValid);
    CHECK_NEAR(ctx, cfg.data.gyroCal.tempSlopeRadSPerC.x, 0.0f, 1.0e-12f);
    CHECK_NEAR(ctx, cfg.data.gyroCal.tempSlopeRadSPerC.y, 0.0f, 1.0e-12f);
    CHECK_NEAR(ctx, cfg.data.gyroCal.tempSlopeRadSPerC.z, 0.0f, 1.0e-12f);
    CHECK_NEAR(ctx, cfg.data.gyroTempQuality.fitQuality, 0.0f, 1.0e-12f);
    CHECK(ctx, cfg.data.gyroCalMeta.tempModelUpdatedUptimeMs == 0u);
    CHECK(ctx, cfg.data.gyroCalMeta.tempModelSampleCount == 0u);

    GyroTempCompensator runtime;
    cfg.applyToGyroTempComp(runtime);
    CHECK(ctx, !runtime.valid());
    CHECK(ctx, !runtime.temperatureModelValid());
}

static void testComponentScopedImuCapture(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();
    cfg.data.gyroCal.biasValid = true;
    cfg.data.gyroCal.biasRadS = Vec3(1.0f, 2.0f, 3.0f);
    cfg.data.accelCal.valid = true;
    cfg.data.accelCal.biasG = Vec3(4.0f, 5.0f, 6.0f);
    cfg.updateCrc();

    ImuCalibration runtime;
    runtime.gyroBiasValid = true;
    runtime.gyroBiasRadS = Vec3(7.0f, 8.0f, 9.0f);
    runtime.accelCalValid = true;
    runtime.accelBiasG = Vec3(10.0f, 11.0f, 12.0f);
    runtime.accelScale = Mat3::diagonal(1.1f, 1.2f, 1.3f);

    TrackerConfig gyroOnly = cfg;
    gyroOnly.captureGyroFromImuCalibration(runtime);
    CHECK_NEAR(ctx, gyroOnly.data.gyroCal.biasRadS.x, 7.0f, 1.0e-6f);
    CHECK_NEAR(ctx, gyroOnly.data.accelCal.biasG.x, 4.0f, 1.0e-6f);

    TrackerConfig accelOnly = cfg;
    accelOnly.captureAccelFromImuCalibration(runtime);
    CHECK_NEAR(ctx, accelOnly.data.gyroCal.biasRadS.x, 1.0f, 1.0e-6f);
    CHECK_NEAR(ctx, accelOnly.data.accelCal.biasG.x, 10.0f, 1.0e-6f);
}

static void testCalibrationClearPreservesPolicy(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();
    cfg.data.gyroCal.biasValid = true;
    cfg.data.gyroCal.biasRadS = Vec3(0.1f, 0.2f, 0.3f);
    cfg.data.gyroCal.tempCompValid = true;
    cfg.data.gyroCal.tempCompEnabled = false;
    cfg.data.gyroCal.tempSlopeRadSPerC = Vec3(0.001f, 0.0f, 0.0f);
    cfg.data.gyroTempQuality.fitQuality = 0.9f;
    cfg.data.gyroCalMeta.tempModelUpdatedUptimeMs = 123u;
    cfg.data.accelCal.valid = true;
    cfg.data.accelCal.biasG = Vec3(0.01f, 0.02f, 0.03f);
    cfg.data.magCal.driverEnabled = true;
    cfg.data.magCal.calibrationValid = true;
    cfg.data.magCal.axisAlignmentValid = true;
    cfg.data.magYaw.applyEnabled = true;
    cfg.data.magCalQuality.sampleCount = 42u;
    cfg.updateCrc();

    TrackerConfig accelOnlyClear = cfg;
    accelOnlyClear.clearAccelCalibration();
    CHECK(ctx, !accelOnlyClear.data.accelCal.valid);
    CHECK(ctx, !accelOnlyClear.data.magYaw.applyEnabled);

    cfg.clearAllCalibrationPreservingPolicy();
    CHECK(ctx, !cfg.data.gyroCal.biasValid);
    CHECK(ctx, !cfg.data.gyroCal.tempCompValid);
    CHECK(ctx, !cfg.data.gyroCal.tempCompEnabled);
    CHECK(ctx, cfg.data.gyroCalMeta.tempModelUpdatedUptimeMs == 0u);
    CHECK(ctx, !cfg.data.accelCal.valid);
    CHECK(ctx, cfg.data.magCal.driverEnabled);
    CHECK(ctx, !cfg.data.magCal.calibrationValid);
    CHECK(ctx, !cfg.data.magCal.axisAlignmentValid);
    CHECK(ctx, !cfg.data.magYaw.applyEnabled);
    CHECK(ctx, cfg.data.magCalQuality.sampleCount == 0u);
    CHECK(ctx, cfg.validate());
}

int main() {
    TestContext ctx;
    testFrameValidationAndThreeSensorAgreement(ctx);
    testInvalidFrameIsSanitizedWithoutTouchingCalibration(ctx);
    testGyroTemperatureLifecycleAndPersistence(ctx);
    testCalibrationSnapshotAndRevisionSemantics(ctx);
    testSanitizeInvalidatesNonFiniteMagCalibration(ctx);
    testSanitizeRejectsTemperatureModelWithoutGyroBias(ctx);
    testComponentScopedImuCapture(ctx);
    testCalibrationClearPreservesPolicy(ctx);
    testRuntimeTrimRemainsSensorFrame(ctx);
    return ctx.finish("test_frame_calibration_lifecycle");
}
