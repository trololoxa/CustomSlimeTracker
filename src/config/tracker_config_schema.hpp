#pragma once

#include <cstdint>

#include "defines.h"
#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/accel_6pos_calibration.hpp"
#include "config/tracker_config_detail.hpp"

namespace tracker {

struct TrackerConfigSchemaVersions {
    uint16_t hardware = tracker_config_detail::SCHEMA_HARDWARE_VERSION;
    uint16_t imu = tracker_config_detail::SCHEMA_IMU_VERSION;
    uint16_t fifo = tracker_config_detail::SCHEMA_FIFO_VERSION;
    uint16_t ahrs = tracker_config_detail::SCHEMA_AHRS_VERSION;
    uint16_t gyroCal = tracker_config_detail::SCHEMA_GYRO_CAL_VERSION;
    uint16_t accelCal = tracker_config_detail::SCHEMA_ACCEL_CAL_VERSION;
    uint16_t magCal = tracker_config_detail::SCHEMA_MAG_CAL_VERSION;
    uint16_t magYaw = tracker_config_detail::SCHEMA_MAG_YAW_VERSION;
    uint16_t quality = tracker_config_detail::SCHEMA_QUALITY_VERSION;
    uint16_t output = tracker_config_detail::SCHEMA_OUTPUT_VERSION;
    uint16_t frame = tracker_config_detail::SCHEMA_FRAME_VERSION;
    uint16_t reserved = 0;
};

struct TrackerHardwareConfig {
    int pinLsmSck = cfg::PIN_LSM_SCK;
    int pinLsmMiso = cfg::PIN_LSM_MISO;
    int pinLsmMosi = cfg::PIN_LSM_MOSI;
    int pinLsmCs = cfg::PIN_LSM_CS;
    int pinLsmInt1 = cfg::PIN_LSM_INT1;

    uint32_t serialBaud = cfg::SERIAL_BAUD;
    uint32_t spiHz = tracker_config_detail::DEFAULT_SPI_HZ;
    uint8_t spiMode = cfg::SPI_MODE;
};

struct TrackerImuConfig {
    Lsm6dsv::Odr imuOdr = Lsm6dsv::Odr::Hz960;
    Lsm6dsv::AccelFs accelFs = Lsm6dsv::AccelFs::G8;
    Lsm6dsv::GyroFs gyroFs = Lsm6dsv::GyroFs::Dps1000;
    Lsm6dsv::AccelMode accelMode = Lsm6dsv::AccelMode::HighPerformance;
    Lsm6dsv::GyroMode gyroMode = Lsm6dsv::GyroMode::HighPerformance;

    bool doSoftwareReset = true;
    bool disableI2cAndI3c = true;
    bool blockDataUpdate = true;
    bool autoIncrement = true;
};

struct TrackerFifoConfig {
    Lsm6dsv::Odr accelBdr = Lsm6dsv::Odr::Hz960;
    Lsm6dsv::Odr gyroBdr = Lsm6dsv::Odr::Hz960;

    uint8_t watermarkWords = cfg::FIFO_WATERMARK_WORDS;
    uint16_t maxWordsPerDrain = cfg::FIFO_MAX_WORDS_PER_DRAIN;
    uint8_t maxDrainRoundsPerEvent = cfg::FIFO_MAX_DRAIN_ROUNDS_PER_EVENT;

    Lsm6dsvFifoReader::FifoMode mode = Lsm6dsvFifoReader::FifoMode::Continuous;
    Lsm6dsvFifoReader::TimestampBatch timestampBatch = Lsm6dsvFifoReader::TimestampBatch::Decimation1;
    Lsm6dsvFifoReader::TemperatureBatch temperatureBatch = Lsm6dsvFifoReader::TemperatureBatch::Hz1_875;

    bool routeWatermarkToInt1 = true;
    bool routeOverrunToInt1 = true;
    bool routeFullToInt1 = true;

    bool enableTimestampCounter = true;
    bool useHardwareTimestamps = true;
    bool allowTimestampFallback = true;
    uint8_t maxWaitingSamplesBeforeFallback = cfg::FIFO_MAX_WAITING_SAMPLES_BEFORE_FALLBACK;

    // 0 => compute from ODR + INTERNAL_FREQ_FINE.
    float samplePeriodUsOverride = 0.0f;
};

struct TrackerAhrsConfig {
    // Legacy AHRS layout. accelCorrectionGain/useAccelCorrection remain active
    // compatibility controls; the two trust floats and quaternion bytes are
    // neutral reserved storage. Body mounting is server-side only.
    float accelCorrectionGain = 3.0f;
    float reservedAccelTrustMinNormG = 0.94f;
    float reservedAccelTrustMaxNormG = 1.35f;
    bool useAccelCorrection = true;
    bool reservedMountingOffsetValid = false;
    Quat reservedMountingOffset = Quat::identity();
};

struct TrackerAhrsRuntimeConfigPersisted {
    // This struct intentionally consumes bytes that were previously reserved in
    // TrackerConfigBlob. That preserves blob size and lets old NVS configs load;
    // sanitize() converts zero-filled legacy reserved bytes to these defaults.
    float minDtS = 0.0001f;
    float maxDtS = 0.0200f;
    float accelKp = 3.0f;
    float maxAccelCorrectionDegPerUpdate = 2.0f;

    float accelNormGoodErrorG = 0.06f;
    float accelNormBadErrorG = 0.35f;
    float accelInnovationGoodDeg = 8.0f;
    float accelInnovationBadDeg = 45.0f;

    float accelNormStdGoodG = 0.010f;
    float accelNormStdBadG = 0.080f;
    float accelNormVarianceAlpha = 0.02f;

    float gyroMotionGoodDps = 250.0f;
    float gyroMotionBadDps = 720.0f;

    uint32_t normalizeEvery = 16;

    bool clampLargeDt = false;
    bool accelCorrectionEnabled = true;
    bool adaptiveAccelCorrection = true;
    uint8_t reserved = 0;
};

static_assert(sizeof(TrackerAhrsRuntimeConfigPersisted) == 60,
              "TrackerAhrsRuntimeConfigPersisted must keep config blob size stable");

struct TrackerGyroCalibrationConfig {
    bool biasValid = false;
    Vec3 biasRadS = Vec3::zero();

    bool tempCompValid = false;
    bool tempCompEnabled = true;
    bool reservedTempLearningEnabled = false;
    float referenceTempC = 25.0f;
    Vec3 tempSlopeRadSPerC = Vec3::zero();
};

struct TrackerGyroTempQualityConfigPersisted {
    // Stored in the old reserved area to keep TrackerConfigBlob size stable.
    float tempRangeMinC = 0.0f;
    float tempRangeMaxC = 0.0f;
    float fitQuality = 0.0f;
    float residualBeforeDps = 0.0f;
    float residualAfterDps = 0.0f;
};

static_assert(sizeof(TrackerGyroTempQualityConfigPersisted) == 20,
              "TrackerGyroTempQualityConfigPersisted must replace reservedU32[5]");

struct TrackerGyroCalibrationMetaPersisted {
    uint32_t biasCalibrationUptimeMs = 0;
    uint32_t tempModelUpdatedUptimeMs = 0;
    uint32_t tempModelSampleCount = 0;
    uint16_t biasCalibrationVersion = tracker_config_detail::SCHEMA_GYRO_CAL_VERSION;
    uint16_t tempModelVersion = tracker_config_detail::SCHEMA_GYRO_CAL_VERSION;
};

struct TrackerAccelCalibrationConfig {
    bool valid = false;
    Vec3 biasG = Vec3::zero();
    Mat3 scale = Mat3::identity();
};

struct TrackerAccelCalibrationQualityPersisted {
    uint32_t calibrationUptimeMs = 0;
    uint32_t qualityFlags = accel_cal_quality_flags::MISSING_FACE;
    float qualityScore = 0.0f;
    float maxFaceNormErrorG = 0.0f;
    float maxAxisResidualG = 0.0f;
    uint32_t faceSamples[6] = {};
    float faceNormErrorG[6] = {};
    float faceAxisResidualG[6] = {};
};

struct TrackerMagCalibrationConfig {
    bool driverEnabled = false;
    bool calibrationValid = false;
    bool axisAlignmentValid = false;

    Vec3 hardIron = Vec3::zero();
    Mat3 softIron = Mat3::identity();
    Mat3 magToImu = Mat3::identity();

    float expectedFieldNorm = 1.0f;
    float minTrustNorm = 0.25f;
    float maxTrustNorm = 2.50f;
};

struct TrackerMagCalibrationQualityPersisted {
    uint32_t calibrationUptimeMs = 0;
    uint32_t sampleCount = 0;
    uint32_t rejectedSamples = 0;
    uint32_t saturatedSamples = 0;
    uint32_t qualityFlags = 0;
    float radiusX = 0.0f;
    float radiusY = 0.0f;
    float radiusZ = 0.0f;
    float normMin = 0.0f;
    float normMean = 0.0f;
    float normMax = 0.0f;
    float coverageScore = 0.0f;
    float residualRms = 0.0f;
    float reservedExpectedHorizontalNorm = 0.0f;
};

struct TrackerFrameConfigPersisted {
    // Only sensorToDevice is active. The remaining bytes are neutral reserved
    // compatibility storage; body mounting remains server-side and the current
    // wire convention is fixed.
    bool sensorToDeviceValid = false;
    bool reservedApplyMountingOffsetInFirmware = false;
    uint8_t reservedOutputConvention = 0;
    uint8_t reservedFlags = 0;
    Mat3 sensorToDevice = Mat3::identity();
};

struct TrackerReservedDeviceIdentityPersisted {
    // Abandoned duplicate identity storage retained only to preserve blob size.
    // Runtime identity lives exclusively in TrackerNetworkConfig.
    uint32_t reservedDeviceId = 0;
    uint8_t reservedSensorId = 0;
    uint8_t reserved0 = 0;
    uint16_t reserved1 = 0;
    char reservedDeviceName[32] = {};
};

struct TrackerMagYawCorrectionConfigPersisted {
    bool controllerEnabled = true;
    bool applyEnabled = false;
    bool requireAccelTrusted = true;
    uint8_t reservedFlags = 0;

    float maxInnovationDeg = 25.0f;
    float horizontalNormBad = 200.0f;
    float horizontalNormGood = 260.0f;
    float gyroNormGoodDps = 8.0f;
    float gyroNormBadDps = 35.0f;
    float accelTrustBad = 0.20f;
    float accelTrustGood = 0.70f;
    float timeConstantS = 30.0f;
    float maxCorrectionRateDegS = 2.0f;
    float maxCorrectionStepDeg = 0.25f;

    uint32_t maxMagAgeMs = 250;
    uint32_t gyroMovingCooldownMs = 1000;
    uint32_t accelBadCooldownMs = 750;
    uint32_t magDisturbanceCooldownMs = 3000;
    float fallbackDtS = 1.0f / 60.0f;
};

struct TrackerQualityConfigPersisted {
    float expectedDtUs = 0.0f;
    float largeGapFactor = 1.75f;
    float smallGapFactor = 0.25f;

    int16_t gyroNearSaturationAbsRaw = 30000;
    int16_t accelNearSaturationAbsRaw = 30000;

    float accelNormOutlierMinG = 0.50f;
    float accelNormOutlierMaxG = 1.50f;

    bool requestRecoveryOnFifoOverrun = true;
    bool requestRecoveryOnFifoFull = true;
    bool requestRecoveryOnUnknownTag = false;
    bool requestRecoveryOnTimestampBackwards = true;
    bool requestRecoveryOnTimestampQueueOverflow = true;

    bool skipAhrsOnBadTimestamp = true;
    bool skipAhrsOnGyroSaturation = true;
    bool disableAccelCorrectionOnAccelSaturation = true;
    bool disableAccelCorrectionOnAccelNormOutlier = true;
};

struct TrackerOutputConfig {
    bool serialDebugEnabled = true;
    bool quaternionOutputEnabled = false;
    uint16_t outputRateHz = cfg::OUTPUT_RATE_HZ;

    // Deprecated legacy field. SlimeVR UDP is controlled by the `slime`/network
    // runtime, not by the local serial output backend. Sanitization forces this
    // field to 0 so old configs with packetFormat=2 do not bind output to UDP.
    uint8_t packetFormat = 0;
};

struct TrackerConfigBlob {
    uint32_t magic = tracker_config_detail::CONFIG_MAGIC;
    uint16_t version = tracker_config_detail::CONFIG_VERSION;
    uint16_t size = sizeof(TrackerConfigBlob);
    uint32_t crc32 = 0;

    TrackerConfigSchemaVersions schemas;

    TrackerHardwareConfig hardware;
    TrackerImuConfig imu;
    TrackerFifoConfig fifo;
    TrackerAhrsConfig ahrs;
    TrackerGyroCalibrationConfig gyroCal;
    TrackerAccelCalibrationConfig accelCal;
    TrackerMagCalibrationConfig magCal;
    TrackerQualityConfigPersisted quality;
    TrackerOutputConfig output;
    TrackerMagYawCorrectionConfigPersisted magYaw;
    TrackerAhrsRuntimeConfigPersisted ahrsRuntime;
    TrackerGyroTempQualityConfigPersisted gyroTempQuality;

    TrackerGyroCalibrationMetaPersisted gyroCalMeta;
    TrackerAccelCalibrationQualityPersisted accelCalQuality;
    TrackerMagCalibrationQualityPersisted magCalQuality;
    TrackerFrameConfigPersisted frame;
    TrackerReservedDeviceIdentityPersisted reservedDevice;
};

} // namespace tracker
