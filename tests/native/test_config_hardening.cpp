#include "test_common.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_detail.hpp"
#include "sensor/calibration.hpp"
#include "sensor/gyro_temperature_compensation.hpp"

using namespace tracker;

static void testDefaultRuntimeConfigValidates(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();

    CHECK(ctx, cfg.validate());
    CHECK(ctx, cfg.validateContent());
    CHECK(ctx, cfg.data.magic == tracker_config_detail::CONFIG_MAGIC);
    CHECK(ctx, cfg.data.version == tracker_config_detail::CONFIG_VERSION);
    CHECK(ctx, cfg.data.size == sizeof(TrackerConfigBlob));
    CHECK(ctx, cfg.data.crc32 == cfg.computeCrc());
}

static void testSanitizeRepairsInvalidRuntimeValues(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();

    cfg.data.hardware.spiHz = 1;
    cfg.data.fifo.watermarkWords = 0;
    cfg.data.fifo.maxWordsPerDrain = 0;
    cfg.data.fifo.maxDrainRoundsPerEvent = 0;
    cfg.data.fifo.samplePeriodUsOverride = std::nanf("");

    cfg.data.output.outputRateHz = 0;
    cfg.data.output.packetFormat = 9;

    cfg.data.quality.largeGapFactor = 0.1f;
    cfg.data.quality.accelNormOutlierMinG = -10.0f;
    cfg.data.quality.accelNormOutlierMaxG = -9.0f;

    cfg.data.magYaw.maxInnovationDeg = std::nanf("");
    cfg.data.magYaw.horizontalNormBad = -1.0f;
    cfg.data.magYaw.horizontalNormGood = 0.0f;
    cfg.data.magYaw.accelTrustBad = -5.0f;
    cfg.data.magYaw.accelTrustGood = -4.0f;
    cfg.data.magYaw.maxMagAgeMs = 100000U;

    cfg.sanitize();

    CHECK(ctx, cfg.validate());
    CHECK(ctx, cfg.data.hardware.spiHz >= tracker_config_detail::MIN_SPI_HZ);
    CHECK(ctx, cfg.data.fifo.watermarkWords != 0);
    CHECK(ctx, cfg.data.fifo.maxWordsPerDrain != 0);
    CHECK(ctx, cfg.data.fifo.maxDrainRoundsPerEvent != 0);
    CHECK_NEAR(ctx, cfg.data.fifo.samplePeriodUsOverride, 0.0f, 1.0e-6f);

    CHECK(ctx, cfg.data.output.outputRateHz != 0);
    CHECK(ctx, cfg.data.output.packetFormat == 0);

    CHECK(ctx, cfg.data.quality.largeGapFactor > 1.0f);
    CHECK(ctx, cfg.data.quality.accelNormOutlierMinG > 0.0f);
    CHECK(ctx, cfg.data.quality.accelNormOutlierMaxG > cfg.data.quality.accelNormOutlierMinG);

    CHECK(ctx, cfg.data.magYaw.maxInnovationDeg > 0.0f);
    CHECK(ctx, cfg.data.magYaw.horizontalNormBad > 0.0f);
    CHECK(ctx, cfg.data.magYaw.horizontalNormGood > cfg.data.magYaw.horizontalNormBad);
    CHECK(ctx, cfg.data.magYaw.accelTrustBad >= 0.0f);
    CHECK(ctx, cfg.data.magYaw.accelTrustGood > cfg.data.magYaw.accelTrustBad);
    CHECK(ctx, cfg.data.magYaw.maxMagAgeMs <= 5000U);
}

static void testSanitizeInvalidatesOnlyCorruptCalibrationBlocks(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();

    cfg.data.gyroCal.biasValid = true;
    cfg.data.gyroCal.biasRadS = Vec3(0.01f, -0.02f, 0.03f);
    cfg.data.accelCal.valid = true;
    cfg.data.accelCal.biasG = Vec3(1.0f, std::nanf(""), 3.0f);
    cfg.data.accelCal.scale = Mat3::identity();
    cfg.data.magCal.calibrationValid = true;
    cfg.data.magCal.hardIron = Vec3(4.0f, 5.0f, 6.0f);
    cfg.data.magCal.softIron = Mat3::identity();
    cfg.data.magCal.expectedFieldNorm = 50.0f;

    cfg.sanitize();

    CHECK(ctx, cfg.validate());
    CHECK(ctx, cfg.data.gyroCal.biasValid);
    CHECK_NEAR(ctx, cfg.data.gyroCal.biasRadS.x, 0.01f, 1.0e-6f);
    CHECK(ctx, !cfg.data.accelCal.valid);
    CHECK_NEAR(ctx, cfg.data.accelCal.biasG.x, 0.0f, 1.0e-6f);
    CHECK(ctx, cfg.data.magCal.calibrationValid);
    CHECK_NEAR(ctx, cfg.data.magCal.hardIron.x, 4.0f, 1.0e-6f);
}

static void testCrcDeterministicAndDetectsMutation(TestContext& ctx) {
    TrackerConfig a;
    TrackerConfig b;
    a.resetDefaults();
    b.resetDefaults();

    CHECK(ctx, a.data.crc32 == b.data.crc32);
    CHECK(ctx, a.computeCrc() == b.computeCrc());

    const uint32_t crc = a.data.crc32;
    a.data.output.outputRateHz = static_cast<uint16_t>(a.data.output.outputRateHz + 1U);
    CHECK(ctx, a.computeCrc() != crc);
    CHECK(ctx, !a.validate());

    a.updateCrc();
    CHECK(ctx, a.validate());
}

static void testApplyCaptureDoesNotTouchUnrelatedBlocks(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();

    cfg.data.magCal.driverEnabled = true;
    cfg.data.magCal.calibrationValid = true;
    cfg.data.magCal.hardIron = Vec3(7.0f, 8.0f, 9.0f);
    const uint8_t packetFormatBefore = cfg.data.output.packetFormat;

    ImuCalibration cal;
    cal.gyroBiasValid = true;
    cal.gyroBiasRadS = Vec3(0.001f, 0.002f, -0.003f);
    cal.accelCalValid = true;
    cal.accelBiasG = Vec3(0.01f, 0.02f, 0.03f);
    cal.accelScale = Mat3::diagonal(1.1f, 0.9f, 1.0f);

    cfg.captureFromImuCalibration(cal);

    CHECK(ctx, cfg.validate());
    CHECK(ctx, cfg.data.magCal.driverEnabled);
    CHECK(ctx, cfg.data.magCal.calibrationValid);
    CHECK_NEAR(ctx, cfg.data.magCal.hardIron.y, 8.0f, 1.0e-6f);
    CHECK(ctx, cfg.data.output.packetFormat == packetFormatBefore);

    ImuCalibration applied;
    cfg.applyToImuCalibration(applied);
    CHECK(ctx, applied.gyroBiasValid);
    CHECK(ctx, applied.accelCalValid);
    CHECK_NEAR(ctx, applied.gyroBiasRadS.z, -0.003f, 1.0e-6f);
    CHECK_NEAR(ctx, applied.accelBiasG.y, 0.02f, 1.0e-6f);
}

static void testSanitizeNeutralizesCompatibilityReservedFields(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();

    cfg.data.ahrs.reservedAccelTrustMinNormG = -123.0f;
    cfg.data.ahrs.reservedAccelTrustMaxNormG = 456.0f;
    cfg.data.ahrs.reservedMountingOffsetValid = true;
    cfg.data.ahrs.reservedMountingOffset = Quat(0.1f, 0.2f, 0.3f, 0.4f);
    cfg.data.gyroCal.reservedTempLearningEnabled = true;
    cfg.data.magCalQuality.reservedExpectedHorizontalNorm = 999.0f;
    cfg.data.frame.reservedApplyMountingOffsetInFirmware = true;
    cfg.data.frame.reservedOutputConvention = 7;
    cfg.data.frame.reservedFlags = 0xFF;
    cfg.data.reservedDevice.reservedDeviceId = 1234;
    cfg.data.reservedDevice.reservedSensorId = 9;
    std::strncpy(cfg.data.reservedDevice.reservedDeviceName, "ghost", sizeof(cfg.data.reservedDevice.reservedDeviceName) - 1);

    cfg.sanitize();

    CHECK(ctx, cfg.validate());
    CHECK_NEAR(ctx, cfg.data.ahrs.reservedAccelTrustMinNormG, 0.94f, 1.0e-6f);
    CHECK_NEAR(ctx, cfg.data.ahrs.reservedAccelTrustMaxNormG, 1.35f, 1.0e-6f);
    CHECK(ctx, !cfg.data.ahrs.reservedMountingOffsetValid);
    CHECK_NEAR(ctx, cfg.data.ahrs.reservedMountingOffset.w, 1.0f, 1.0e-6f);
    CHECK_NEAR(ctx, cfg.data.ahrs.reservedMountingOffset.x, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, cfg.data.ahrs.reservedMountingOffset.y, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, cfg.data.ahrs.reservedMountingOffset.z, 0.0f, 1.0e-6f);
    CHECK(ctx, !cfg.data.gyroCal.reservedTempLearningEnabled);
    CHECK_NEAR(ctx, cfg.data.magCalQuality.reservedExpectedHorizontalNorm, 0.0f, 1.0e-6f);
    CHECK(ctx, !cfg.data.frame.reservedApplyMountingOffsetInFirmware);
    CHECK(ctx, cfg.data.frame.reservedOutputConvention == 0);
    CHECK(ctx, cfg.data.frame.reservedFlags == 0);
    CHECK(ctx, cfg.data.reservedDevice.reservedDeviceId == 0);
    CHECK(ctx, cfg.data.reservedDevice.reservedSensorId == 0);
    CHECK(ctx, cfg.data.reservedDevice.reservedDeviceName[0] == '\0');
}

static void testPerformanceDefaultMigrationPreservesCurrentSettings(TestContext& ctx) {
    TrackerConfig legacy;
    legacy.resetDefaults();
    legacy.data.hardware.spiHz = tracker_config_detail::LEGACY_SPI_HZ;
    legacy.data.fifo.watermarkWords = cfg::LEGACY_FIFO_WATERMARK_WORDS;
    legacy.updateCrc();
    trackerMigratePerformanceDefaults(legacy);
    CHECK(ctx, legacy.data.hardware.spiHz == tracker_config_detail::DEFAULT_SPI_HZ);
    CHECK(ctx, legacy.data.fifo.watermarkWords == cfg::FIFO_WATERMARK_WORDS);
    CHECK(ctx, legacy.validate());

    TrackerConfig current;
    current.resetDefaults();
    current.data.hardware.spiHz = tracker_config_detail::PREVIOUS_SPI_HZ;
    current.data.fifo.watermarkWords = cfg::PREVIOUS_FIFO_WATERMARK_WORDS;
    current.updateCrc();
    trackerMigratePerformanceDefaults(current);
    CHECK(ctx, current.data.hardware.spiHz == tracker_config_detail::PREVIOUS_SPI_HZ);
    CHECK(ctx, current.data.fifo.watermarkWords == cfg::PREVIOUS_FIFO_WATERMARK_WORDS);
    CHECK(ctx, current.validate());
}

static void testOversizedTemperatureSlopeIsInvalidated(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();
    cfg.data.gyroCal.biasValid = true;
    cfg.data.gyroCal.biasRadS = Vec3(0.001f, -0.002f, 0.003f);
    cfg.data.gyroCal.tempCompValid = true;
    cfg.data.gyroCal.referenceTempC = 25.0f;
    cfg.data.gyroCal.tempSlopeRadSPerC = Vec3(0.11f, 0.0f, 0.0f) * MATH_DEG_TO_RAD;
    cfg.updateCrc();

    CHECK(ctx, !cfg.validateContent());
    cfg.sanitize();
    CHECK(ctx, cfg.validate());
    CHECK(ctx, cfg.data.gyroCal.biasValid);
    CHECK(ctx, !cfg.data.gyroCal.tempCompValid);
    CHECK_NEAR(ctx, cfg.data.gyroCal.tempSlopeRadSPerC.norm(), 0.0f, 1.0e-9f);

    GyroTempCompensator comp;
    cfg.applyToGyroTempComp(comp);
    CHECK(ctx, comp.valid());
    CHECK(ctx, !comp.temperatureModelValid());
}

static void testSanitizeClearsEvidenceForMissingModels(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();

    cfg.data.gyroCal.biasValid = false;
    cfg.data.gyroCal.biasRadS = Vec3(1.0f, 2.0f, 3.0f);
    cfg.data.gyroCalMeta.biasCalibrationUptimeMs = 123u;
    cfg.data.gyroCal.tempCompValid = false;
    cfg.data.gyroCal.tempCompEnabled = false;
    cfg.data.gyroCal.tempSlopeRadSPerC = Vec3(0.1f, 0.2f, 0.3f);
    cfg.data.gyroTempQuality.fitQuality = 0.9f;
    cfg.data.gyroCalMeta.tempModelUpdatedUptimeMs = 456u;
    cfg.data.gyroCalMeta.tempModelSampleCount = 789u;

    cfg.data.accelCal.valid = false;
    cfg.data.accelCal.biasG = Vec3(1.0f, 2.0f, 3.0f);
    cfg.data.accelCalQuality.calibrationUptimeMs = 10u;
    cfg.data.accelCalQuality.qualityScore = 0.8f;

    cfg.data.magCal.driverEnabled = true;
    cfg.data.magCal.calibrationValid = false;
    cfg.data.magCal.hardIron = Vec3(4.0f, 5.0f, 6.0f);
    cfg.data.magCal.minTrustNorm = 0.9f;
    cfg.data.magCalQuality.coverageScore = 0.7f;
    cfg.data.magCal.axisAlignmentValid = false;
    cfg.data.magCal.magToImu = Mat3::diagonal(-1.0f, 1.0f, -1.0f);

    cfg.data.frame.sensorToDeviceValid = false;
    cfg.data.frame.sensorToDevice = Mat3::diagonal(-1.0f, -1.0f, 1.0f);

    cfg.sanitize();

    CHECK(ctx, !cfg.data.gyroCal.biasValid);
    CHECK_NEAR(ctx, cfg.data.gyroCal.biasRadS.norm(), 0.0f, 1.0e-9f);
    CHECK(ctx, cfg.data.gyroCalMeta.biasCalibrationUptimeMs == 0u);
    CHECK(ctx, !cfg.data.gyroCal.tempCompValid);
    CHECK(ctx, !cfg.data.gyroCal.tempCompEnabled);
    CHECK_NEAR(ctx, cfg.data.gyroCal.tempSlopeRadSPerC.norm(), 0.0f, 1.0e-9f);
    CHECK_NEAR(ctx, cfg.data.gyroTempQuality.fitQuality, 0.0f, 1.0e-9f);
    CHECK(ctx, cfg.data.gyroCalMeta.tempModelUpdatedUptimeMs == 0u);
    CHECK(ctx, cfg.data.gyroCalMeta.tempModelSampleCount == 0u);

    CHECK(ctx, !cfg.data.accelCal.valid);
    CHECK_NEAR(ctx, cfg.data.accelCal.biasG.norm(), 0.0f, 1.0e-9f);
    CHECK(ctx, cfg.data.accelCalQuality.calibrationUptimeMs == 0u);
    CHECK_NEAR(ctx, cfg.data.accelCalQuality.qualityScore, 0.0f, 1.0e-9f);

    CHECK(ctx, cfg.data.magCal.driverEnabled);
    CHECK(ctx, !cfg.data.magCal.calibrationValid);
    CHECK_NEAR(ctx, cfg.data.magCal.hardIron.norm(), 0.0f, 1.0e-9f);
    CHECK_NEAR(ctx, cfg.data.magCal.minTrustNorm, 0.25f, 1.0e-6f);
    CHECK_NEAR(ctx, cfg.data.magCalQuality.coverageScore, 0.0f, 1.0e-9f);
    CHECK(ctx, !cfg.data.magCal.axisAlignmentValid);
    CHECK_NEAR(ctx, cfg.data.magCal.magToImu.determinant(), 1.0f, 1.0e-6f);
    CHECK(ctx, !cfg.data.frame.sensorToDeviceValid);
    CHECK_NEAR(ctx, cfg.data.frame.sensorToDevice.determinant(), 1.0f, 1.0e-6f);
}


static void testMagAxisSanitizePreservesContinuousProperRotation(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();
    cfg.data.magCal.axisAlignmentValid = true;
    cfg.data.magCal.magToImu = Quat::fromEulerXYZ(
        1.0f * MATH_DEG_TO_RAD,
       -2.0f * MATH_DEG_TO_RAD,
        0.5f * MATH_DEG_TO_RAD).toRotationMatrix();
    cfg.sanitize();
    CHECK(ctx, cfg.data.magCal.axisAlignmentValid);
    CHECK_NEAR(ctx, cfg.data.magCal.magToImu.determinant(), 1.0f, 1.0e-4f);
}

static void testFullCalibrationClearAlsoClearsFrame(TestContext& ctx) {
    TrackerConfig cfg;
    cfg.resetDefaults();
    cfg.data.frame.sensorToDeviceValid = true;
    cfg.data.frame.sensorToDevice = Mat3::diagonal(-1.0f, -1.0f, 1.0f);
    cfg.data.gyroCal.tempCompEnabled = false;
    cfg.data.magCal.driverEnabled = true;

    cfg.clearAllCalibrationPreservingPolicy();

    CHECK(ctx, !cfg.data.frame.sensorToDeviceValid);
    CHECK_NEAR(ctx, cfg.data.frame.sensorToDevice.determinant(), 1.0f, 1.0e-6f);
    CHECK(ctx, !cfg.data.gyroCal.tempCompEnabled);
    CHECK(ctx, cfg.data.magCal.driverEnabled);
}

int main() {
    TestContext ctx;
    testDefaultRuntimeConfigValidates(ctx);
    testSanitizeRepairsInvalidRuntimeValues(ctx);
    testSanitizeInvalidatesOnlyCorruptCalibrationBlocks(ctx);
    testCrcDeterministicAndDetectsMutation(ctx);
    testApplyCaptureDoesNotTouchUnrelatedBlocks(ctx);
    testSanitizeNeutralizesCompatibilityReservedFields(ctx);
    testPerformanceDefaultMigrationPreservesCurrentSettings(ctx);
    testOversizedTemperatureSlopeIsInvalidated(ctx);
    testSanitizeClearsEvidenceForMissingModels(ctx);
    testMagAxisSanitizePreservesContinuousProperRotation(ctx);
    testFullCalibrationClearAlsoClearsFrame(ctx);
    return ctx.finish("test_config_hardening");
}
