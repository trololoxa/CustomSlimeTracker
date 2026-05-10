#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <cstdint>
#include <cstring>
#include <cmath>

#include "defines.h"
#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/calibration.hpp"
#include "sensor/accel_6pos_calibration.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"
#include "sensor/mag_calibration.hpp"

namespace tracker {

// ============================================================
// Tracker persistent config / calibration storage
// ============================================================
// Purpose:
//   - One canonical config blob for hardcoded defaults + NVS save/load.
//   - Safe persistent storage with magic/version/size/CRC.
//   - Command-protocol friendly interface:
//       load
//       save
//       erase
//       reset defaults
//       print summary
//   - Does not own commands; serial protocol can call this layer later.
//
// Storage backend:
//   ESP32 Preferences / NVS
//
// Notes:
//   - Pins are usually compile-time hardware constants. They are included here
//     for summary/debug, but runtime pin reassignment is not recommended.
//   - Calibration data is persisted here.
//   - Mag config fields are placeholders for the next phases.
// ============================================================

namespace tracker_config_detail {

static constexpr uint32_t CONFIG_MAGIC = 0x54364453UL; // 'T6DS' little-endian-ish
static constexpr uint16_t CONFIG_VERSION = 2;
static constexpr const char* NVS_NAMESPACE = "tracker";
static constexpr const char* NVS_KEY_CONFIG = "cfg";

static constexpr uint16_t SCHEMA_HARDWARE_VERSION = 1;
static constexpr uint16_t SCHEMA_IMU_VERSION = 1;
static constexpr uint16_t SCHEMA_FIFO_VERSION = 1;
static constexpr uint16_t SCHEMA_AHRS_VERSION = 2;
static constexpr uint16_t SCHEMA_GYRO_CAL_VERSION = 2;
static constexpr uint16_t SCHEMA_ACCEL_CAL_VERSION = 2;
static constexpr uint16_t SCHEMA_MAG_CAL_VERSION = 2;
static constexpr uint16_t SCHEMA_MAG_YAW_VERSION = 2;
static constexpr uint16_t SCHEMA_QUALITY_VERSION = 1;
static constexpr uint16_t SCHEMA_OUTPUT_VERSION = 2;
static constexpr uint16_t SCHEMA_FRAME_VERSION = 1;


static constexpr uint32_t DEFAULT_SPI_HZ = cfg::SPI_HZ;
static constexpr uint32_t LEGACY_SPI_HZ = cfg::LEGACY_SPI_HZ;
static constexpr uint32_t MIN_SPI_HZ = cfg::MIN_SPI_HZ;
static constexpr uint32_t MAX_SPI_HZ = cfg::MAX_SPI_HZ;

inline uint32_t fnv1a32(const uint8_t* data, size_t len) {
    uint32_t h = 2166136261UL;
    for (size_t i = 0; i < len; ++i) {
        h ^= static_cast<uint32_t>(data[i]);
        h *= 16777619UL;
    }
    return h;
}

inline bool finiteFloat(float x) {
    return std::isfinite(x);
}

inline bool finiteVec3(const Vec3& v) {
    return finiteFloat(v.x) && finiteFloat(v.y) && finiteFloat(v.z);
}

inline bool finiteQuat(const Quat& q) {
    return finiteFloat(q.w) && finiteFloat(q.x) && finiteFloat(q.y) && finiteFloat(q.z);
}

inline bool finiteMat3(const Mat3& m) {
    for (uint8_t r = 0; r < 3; ++r) {
        for (uint8_t c = 0; c < 3; ++c) {
            if (!finiteFloat(m.m[r][c])) return false;
        }
    }
    return true;
}

inline float clampFloat(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

} // namespace tracker_config_detail


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
    // Legacy/user-facing AHRS fields kept in place to preserve the persistent
    // config layout. Phase B effective AHRS parameters live in ahrsRuntime below.
    float accelCorrectionGain = 3.0f;
    float accelTrustMinNormG = 0.94f;
    float accelTrustMaxNormG = 1.35f;
    bool useAccelCorrection = true;

    // Mounting offset from sensor frame to tracker/body frame.
    bool mountingOffsetValid = false;
    Quat mountingOffset = Quat::identity();
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
    bool tempLearningEnabled = false;
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
    float expectedHorizontalNorm = 0.0f;
};

struct TrackerFrameConfigPersisted {
    // Reserved for explicit sensor/device/output frame conventions. Mounting
    // and body offsets should normally remain server-side for SlimeVR, but the
    // physical sensor-to-board/device transform belongs in firmware.
    bool sensorToDeviceValid = false;
    bool applyMountingOffsetInFirmware = false;
    uint8_t outputConvention = 0; // 0 = firmware-native quaternion convention.
    uint8_t reservedFlags = 0;
    Mat3 sensorToDevice = Mat3::identity();
};

struct TrackerDeviceIdentityPersisted {
    // Stable IDs/names for future network/SlimeVR transport. These are not used
    // by the current serial/debug output path yet.
    uint32_t deviceId = 0;
    uint8_t sensorId = 0;
    uint8_t reserved0 = 0;
    uint16_t reserved1 = 0;
    char deviceName[32] = "c3_6dsv_tracker";
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
    bool requestRecoveryOnUnknownTag = true;
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

    // Output packet format. Only 0 is implemented in this build.
    // 1 = reserved binary custom, 2 = reserved SlimeVR-compatible UDP.
    // Reserved formats are sanitized back to 0 until real backends exist.
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
    TrackerDeviceIdentityPersisted device;
};

class TrackerConfig {
public:
    TrackerConfigBlob data;

    void resetDefaults() {
        data = TrackerConfigBlob{};
        updateCrc();
    }

    bool validate() const {
        if (data.magic != tracker_config_detail::CONFIG_MAGIC) return false;
        if (data.version != tracker_config_detail::CONFIG_VERSION) return false;
        if (data.size != sizeof(TrackerConfigBlob)) return false;
        if (computeCrc() != data.crc32) return false;
        return validateContent();
    }

    bool validateContent() const {
        using namespace tracker_config_detail;

        if (data.schemas.hardware != SCHEMA_HARDWARE_VERSION) return false;
        if (data.schemas.imu != SCHEMA_IMU_VERSION) return false;
        if (data.schemas.fifo != SCHEMA_FIFO_VERSION) return false;
        if (data.schemas.ahrs != SCHEMA_AHRS_VERSION) return false;
        if (data.schemas.gyroCal != SCHEMA_GYRO_CAL_VERSION) return false;
        if (data.schemas.accelCal != SCHEMA_ACCEL_CAL_VERSION) return false;
        if (data.schemas.magCal != SCHEMA_MAG_CAL_VERSION) return false;
        if (data.schemas.magYaw != SCHEMA_MAG_YAW_VERSION) return false;
        if (data.schemas.quality != SCHEMA_QUALITY_VERSION) return false;
        if (data.schemas.output != SCHEMA_OUTPUT_VERSION) return false;
        if (data.schemas.frame != SCHEMA_FRAME_VERSION) return false;

        if (data.hardware.spiHz == 0) return false;
        if (data.hardware.serialBaud == 0) return false;

        if (data.fifo.watermarkWords == 0) return false;
        if (data.fifo.maxWordsPerDrain == 0) return false;
        if (data.fifo.maxDrainRoundsPerEvent == 0) return false;
        if (!finiteFloat(data.fifo.samplePeriodUsOverride)) return false;

        if (data.gyroCal.biasValid && !finiteVec3(data.gyroCal.biasRadS)) return false;
        if (data.gyroCal.tempCompValid) {
            if (!finiteFloat(data.gyroCal.referenceTempC)) return false;
            if (!finiteVec3(data.gyroCal.tempSlopeRadSPerC)) return false;
        }
        if (!finiteFloat(data.gyroTempQuality.tempRangeMinC)) return false;
        if (!finiteFloat(data.gyroTempQuality.tempRangeMaxC)) return false;
        if (!finiteFloat(data.gyroTempQuality.fitQuality)) return false;
        if (!finiteFloat(data.gyroTempQuality.residualBeforeDps)) return false;
        if (!finiteFloat(data.gyroTempQuality.residualAfterDps)) return false;
        if (data.gyroCalMeta.biasCalibrationVersion != SCHEMA_GYRO_CAL_VERSION) return false;
        if (data.gyroCalMeta.tempModelVersion != SCHEMA_GYRO_CAL_VERSION) return false;

        if (!finiteFloat(data.accelCalQuality.qualityScore)) return false;
        if (!finiteFloat(data.accelCalQuality.maxFaceNormErrorG)) return false;
        if (!finiteFloat(data.accelCalQuality.maxAxisResidualG)) return false;
        for (uint8_t i = 0; i < 6; ++i) {
            if (!finiteFloat(data.accelCalQuality.faceNormErrorG[i])) return false;
            if (!finiteFloat(data.accelCalQuality.faceAxisResidualG[i])) return false;
        }

        if (data.accelCal.valid) {
            if (!finiteVec3(data.accelCal.biasG)) return false;
            if (!finiteMat3(data.accelCal.scale)) return false;
        }

        if (data.magCal.calibrationValid) {
            if (!finiteVec3(data.magCal.hardIron)) return false;
            if (!finiteMat3(data.magCal.softIron)) return false;
            if (!finiteFloat(data.magCal.expectedFieldNorm)) return false;
        }

        if (data.magCal.axisAlignmentValid && !finiteMat3(data.magCal.magToImu)) return false;
        if (!finiteFloat(data.magCalQuality.radiusX)) return false;
        if (!finiteFloat(data.magCalQuality.radiusY)) return false;
        if (!finiteFloat(data.magCalQuality.radiusZ)) return false;
        if (!finiteFloat(data.magCalQuality.normMin)) return false;
        if (!finiteFloat(data.magCalQuality.normMean)) return false;
        if (!finiteFloat(data.magCalQuality.normMax)) return false;
        if (!finiteFloat(data.magCalQuality.coverageScore)) return false;
        if (!finiteFloat(data.magCalQuality.residualRms)) return false;
        if (!finiteFloat(data.magCalQuality.expectedHorizontalNorm)) return false;

        if (data.frame.sensorToDeviceValid && !finiteMat3(data.frame.sensorToDevice)) return false;

        // Keep these checks permissive so configs saved by older firmware, where
        // the magYaw region was reserved/zeroed, can still load and then be
        // repaired by sanitize().
        if (!finiteFloat(data.magYaw.maxInnovationDeg)) return false;
        if (!finiteFloat(data.magYaw.horizontalNormBad)) return false;
        if (!finiteFloat(data.magYaw.horizontalNormGood)) return false;
        if (!finiteFloat(data.magYaw.gyroNormGoodDps)) return false;
        if (!finiteFloat(data.magYaw.gyroNormBadDps)) return false;
        if (!finiteFloat(data.magYaw.accelTrustBad)) return false;
        if (!finiteFloat(data.magYaw.accelTrustGood)) return false;
        if (!finiteFloat(data.magYaw.timeConstantS)) return false;
        if (!finiteFloat(data.magYaw.maxCorrectionRateDegS)) return false;
        if (!finiteFloat(data.magYaw.maxCorrectionStepDeg)) return false;
        if (!finiteFloat(data.magYaw.fallbackDtS)) return false;

        if (data.ahrs.mountingOffsetValid && !finiteQuat(data.ahrs.mountingOffset)) return false;

        if (!finiteFloat(data.ahrsRuntime.minDtS)) return false;
        if (!finiteFloat(data.ahrsRuntime.maxDtS)) return false;
        if (!finiteFloat(data.ahrsRuntime.accelKp)) return false;
        if (!finiteFloat(data.ahrsRuntime.maxAccelCorrectionDegPerUpdate)) return false;
        if (!finiteFloat(data.ahrsRuntime.accelNormGoodErrorG)) return false;
        if (!finiteFloat(data.ahrsRuntime.accelNormBadErrorG)) return false;
        if (!finiteFloat(data.ahrsRuntime.accelInnovationGoodDeg)) return false;
        if (!finiteFloat(data.ahrsRuntime.accelInnovationBadDeg)) return false;
        if (!finiteFloat(data.ahrsRuntime.accelNormStdGoodG)) return false;
        if (!finiteFloat(data.ahrsRuntime.accelNormStdBadG)) return false;
        if (!finiteFloat(data.ahrsRuntime.accelNormVarianceAlpha)) return false;
        if (!finiteFloat(data.ahrsRuntime.gyroMotionGoodDps)) return false;
        if (!finiteFloat(data.ahrsRuntime.gyroMotionBadDps)) return false;

        if (!finiteFloat(data.quality.largeGapFactor) || data.quality.largeGapFactor <= 1.0f) return false;
        if (!finiteFloat(data.quality.accelNormOutlierMinG)) return false;
        if (!finiteFloat(data.quality.accelNormOutlierMaxG)) return false;
        if (data.quality.accelNormOutlierMinG <= 0.0f) return false;
        if (data.quality.accelNormOutlierMaxG <= data.quality.accelNormOutlierMinG) return false;

        if (data.output.outputRateHz == 0) return false;
        return true;
    }

    uint32_t computeCrc() const {
        TrackerConfigBlob tmp = data;
        tmp.crc32 = 0;
        return tracker_config_detail::fnv1a32(
            reinterpret_cast<const uint8_t*>(&tmp),
            sizeof(tmp)
        );
    }

    void updateCrc() {
        data.magic = tracker_config_detail::CONFIG_MAGIC;
        data.version = tracker_config_detail::CONFIG_VERSION;
        data.size = sizeof(TrackerConfigBlob);
        data.crc32 = 0;
        data.crc32 = computeCrc();
    }

    void sanitize() {
        using namespace tracker_config_detail;

        data.schemas = TrackerConfigSchemaVersions{};

        if (data.hardware.serialBaud == 0) data.hardware.serialBaud = cfg::SERIAL_BAUD;
        if (data.hardware.spiHz == 0) data.hardware.spiHz = DEFAULT_SPI_HZ;
        if (data.hardware.spiHz < MIN_SPI_HZ) data.hardware.spiHz = MIN_SPI_HZ;
        if (data.hardware.spiHz > MAX_SPI_HZ) data.hardware.spiHz = MAX_SPI_HZ;

        if (data.fifo.watermarkWords == 0) data.fifo.watermarkWords = cfg::FIFO_WATERMARK_WORDS;
        if (data.fifo.maxWordsPerDrain == 0) data.fifo.maxWordsPerDrain = cfg::FIFO_MAX_WORDS_PER_DRAIN;
        if (data.fifo.maxDrainRoundsPerEvent == 0) data.fifo.maxDrainRoundsPerEvent = cfg::FIFO_MAX_DRAIN_ROUNDS_PER_EVENT;
        if (!finiteFloat(data.fifo.samplePeriodUsOverride)) data.fifo.samplePeriodUsOverride = 0.0f;
        data.fifo.maxWaitingSamplesBeforeFallback = static_cast<uint8_t>(
            data.fifo.maxWaitingSamplesBeforeFallback == 0 ? cfg::FIFO_MAX_WAITING_SAMPLES_BEFORE_FALLBACK : data.fifo.maxWaitingSamplesBeforeFallback
        );

        if (!finiteVec3(data.gyroCal.biasRadS)) {
            data.gyroCal.biasRadS = Vec3::zero();
            data.gyroCal.biasValid = false;
        }
        if (!finiteFloat(data.gyroCal.referenceTempC)) data.gyroCal.referenceTempC = 25.0f;
        if (!finiteVec3(data.gyroCal.tempSlopeRadSPerC)) {
            data.gyroCal.tempSlopeRadSPerC = Vec3::zero();
            data.gyroCal.tempCompValid = false;
        }
        if (!finiteFloat(data.gyroTempQuality.tempRangeMinC)) data.gyroTempQuality.tempRangeMinC = 0.0f;
        if (!finiteFloat(data.gyroTempQuality.tempRangeMaxC)) data.gyroTempQuality.tempRangeMaxC = 0.0f;
        if (!finiteFloat(data.gyroTempQuality.fitQuality)) data.gyroTempQuality.fitQuality = 0.0f;
        if (!finiteFloat(data.gyroTempQuality.residualBeforeDps)) data.gyroTempQuality.residualBeforeDps = 0.0f;
        if (!finiteFloat(data.gyroTempQuality.residualAfterDps)) data.gyroTempQuality.residualAfterDps = 0.0f;
        data.gyroTempQuality.fitQuality = clampFloat(data.gyroTempQuality.fitQuality, 0.0f, 1.0f);
        if (data.gyroTempQuality.tempRangeMaxC < data.gyroTempQuality.tempRangeMinC) {
            data.gyroTempQuality.tempRangeMinC = 0.0f;
            data.gyroTempQuality.tempRangeMaxC = 0.0f;
        }
        data.gyroCalMeta.biasCalibrationVersion = SCHEMA_GYRO_CAL_VERSION;
        data.gyroCalMeta.tempModelVersion = SCHEMA_GYRO_CAL_VERSION;

        if (!finiteFloat(data.accelCalQuality.qualityScore)) data.accelCalQuality.qualityScore = 0.0f;
        if (!finiteFloat(data.accelCalQuality.maxFaceNormErrorG)) data.accelCalQuality.maxFaceNormErrorG = 0.0f;
        if (!finiteFloat(data.accelCalQuality.maxAxisResidualG)) data.accelCalQuality.maxAxisResidualG = 0.0f;
        data.accelCalQuality.qualityScore = clampFloat(data.accelCalQuality.qualityScore, 0.0f, 1.0f);
        for (uint8_t i = 0; i < 6; ++i) {
            if (!finiteFloat(data.accelCalQuality.faceNormErrorG[i])) data.accelCalQuality.faceNormErrorG[i] = 0.0f;
            if (!finiteFloat(data.accelCalQuality.faceAxisResidualG[i])) data.accelCalQuality.faceAxisResidualG[i] = 0.0f;
        }

        if (!finiteVec3(data.accelCal.biasG)) {
            data.accelCal.biasG = Vec3::zero();
            data.accelCal.valid = false;
        }
        if (!finiteMat3(data.accelCal.scale)) {
            data.accelCal.scale = Mat3::identity();
            data.accelCal.valid = false;
        }

        if (!finiteMat3(data.magCal.softIron)) data.magCal.softIron = Mat3::identity();
        if (!finiteMat3(data.magCal.magToImu)) data.magCal.magToImu = Mat3::identity();
        if (!finiteVec3(data.magCal.hardIron)) data.magCal.hardIron = Vec3::zero();
        if (!finiteFloat(data.magCal.expectedFieldNorm)) data.magCal.expectedFieldNorm = 1.0f;
        if (!finiteFloat(data.magCal.minTrustNorm)) data.magCal.minTrustNorm = 0.25f;
        if (!finiteFloat(data.magCal.maxTrustNorm)) data.magCal.maxTrustNorm = 2.50f;
        if (data.magCal.minTrustNorm <= 0.0f) data.magCal.minTrustNorm = 0.25f;
        if (data.magCal.maxTrustNorm <= data.magCal.minTrustNorm) data.magCal.maxTrustNorm = data.magCal.minTrustNorm * 2.0f;

        if (!finiteFloat(data.magCalQuality.radiusX)) data.magCalQuality.radiusX = 0.0f;
        if (!finiteFloat(data.magCalQuality.radiusY)) data.magCalQuality.radiusY = 0.0f;
        if (!finiteFloat(data.magCalQuality.radiusZ)) data.magCalQuality.radiusZ = 0.0f;
        if (!finiteFloat(data.magCalQuality.normMin)) data.magCalQuality.normMin = 0.0f;
        if (!finiteFloat(data.magCalQuality.normMean)) data.magCalQuality.normMean = 0.0f;
        if (!finiteFloat(data.magCalQuality.normMax)) data.magCalQuality.normMax = 0.0f;
        if (!finiteFloat(data.magCalQuality.coverageScore)) data.magCalQuality.coverageScore = 0.0f;
        if (!finiteFloat(data.magCalQuality.residualRms)) data.magCalQuality.residualRms = 0.0f;
        if (!finiteFloat(data.magCalQuality.expectedHorizontalNorm)) data.magCalQuality.expectedHorizontalNorm = 0.0f;
        data.magCalQuality.coverageScore = clampFloat(data.magCalQuality.coverageScore, 0.0f, 1.0f);

        if (!finiteMat3(data.frame.sensorToDevice)) {
            data.frame.sensorToDevice = Mat3::identity();
            data.frame.sensorToDeviceValid = false;
        }
        data.device.deviceName[sizeof(data.device.deviceName) - 1] = '\0';
        if (data.device.deviceName[0] == '\0') {
            std::strncpy(data.device.deviceName, "c3_6dsv_tracker", sizeof(data.device.deviceName) - 1);
            data.device.deviceName[sizeof(data.device.deviceName) - 1] = '\0';
        }

        // If this config was written by older firmware, magYaw fields live in
        // the previously-reserved zeroed region. Treat zero/invalid values as
        // defaults instead of invalidating the whole NVS blob.
        if (!finiteFloat(data.magYaw.maxInnovationDeg) || data.magYaw.maxInnovationDeg <= 0.0f) {
            data.magYaw.maxInnovationDeg = 25.0f;
        }

        if (!finiteFloat(data.magYaw.horizontalNormBad) || data.magYaw.horizontalNormBad <= 0.0f) {
            data.magYaw.horizontalNormBad = 200.0f;
        }

        if (!finiteFloat(data.magYaw.horizontalNormGood) || data.magYaw.horizontalNormGood <= data.magYaw.horizontalNormBad) {
            data.magYaw.horizontalNormGood = 260.0f;
        }

        if (!finiteFloat(data.magYaw.gyroNormGoodDps) || data.magYaw.gyroNormGoodDps < 0.0f) {
            data.magYaw.gyroNormGoodDps = 8.0f;
        }

        if (!finiteFloat(data.magYaw.gyroNormBadDps) || data.magYaw.gyroNormBadDps <= data.magYaw.gyroNormGoodDps) {
            data.magYaw.gyroNormBadDps = 35.0f;
        }

        if (!finiteFloat(data.magYaw.accelTrustBad) || data.magYaw.accelTrustBad < 0.0f) {
            data.magYaw.accelTrustBad = 0.20f;
        }

        if (!finiteFloat(data.magYaw.accelTrustGood) || data.magYaw.accelTrustGood <= data.magYaw.accelTrustBad) {
            data.magYaw.accelTrustGood = 0.70f;
        }

        data.magYaw.accelTrustBad = clampFloat(data.magYaw.accelTrustBad, 0.0f, 1.0f);
        data.magYaw.accelTrustGood = clampFloat(data.magYaw.accelTrustGood, data.magYaw.accelTrustBad + 0.01f, 1.0f);

        if (!finiteFloat(data.magYaw.timeConstantS) || data.magYaw.timeConstantS <= 0.001f) {
            data.magYaw.timeConstantS = 30.0f;
        }
        data.magYaw.timeConstantS = clampFloat(data.magYaw.timeConstantS, 1.0f, 300.0f);

        if (!finiteFloat(data.magYaw.maxCorrectionRateDegS) || data.magYaw.maxCorrectionRateDegS <= 0.0f) {
            data.magYaw.maxCorrectionRateDegS = 2.0f;
        }
        data.magYaw.maxCorrectionRateDegS = clampFloat(data.magYaw.maxCorrectionRateDegS, 0.01f, 45.0f);

        if (!finiteFloat(data.magYaw.maxCorrectionStepDeg) || data.magYaw.maxCorrectionStepDeg <= 0.0f) {
            data.magYaw.maxCorrectionStepDeg = 0.25f;
        }
        data.magYaw.maxCorrectionStepDeg = clampFloat(data.magYaw.maxCorrectionStepDeg, 0.001f, 5.0f);

        if (data.magYaw.maxMagAgeMs == 0) data.magYaw.maxMagAgeMs = 250;
        if (data.magYaw.maxMagAgeMs > 5000) data.magYaw.maxMagAgeMs = 5000;
        if (data.magYaw.gyroMovingCooldownMs > 60000UL) data.magYaw.gyroMovingCooldownMs = 1000;
        if (data.magYaw.accelBadCooldownMs > 60000UL) data.magYaw.accelBadCooldownMs = 750;
        if (data.magYaw.magDisturbanceCooldownMs > 120000UL) data.magYaw.magDisturbanceCooldownMs = 3000;
        if (!finiteFloat(data.magYaw.fallbackDtS) || data.magYaw.fallbackDtS <= 0.0f) data.magYaw.fallbackDtS = 1.0f / 60.0f;
        data.magYaw.fallbackDtS = clampFloat(data.magYaw.fallbackDtS, 0.001f, 1.0f);

        if (!finiteQuat(data.ahrs.mountingOffset)) {
            data.ahrs.mountingOffset = Quat::identity();
            data.ahrs.mountingOffsetValid = false;
        }

        // Phase B: old configs have zero-filled ahrsRuntime because this region
        // used to be reserved. Repair zeros/invalid values to production-safe
        // defaults instead of invalidating the whole calibration blob.
        if (!finiteFloat(data.ahrsRuntime.minDtS) || data.ahrsRuntime.minDtS <= 0.0f) data.ahrsRuntime.minDtS = 0.0001f;
        if (!finiteFloat(data.ahrsRuntime.maxDtS) || data.ahrsRuntime.maxDtS <= data.ahrsRuntime.minDtS) data.ahrsRuntime.maxDtS = 0.0200f;
        if (!finiteFloat(data.ahrsRuntime.accelKp) || data.ahrsRuntime.accelKp <= 0.0f) data.ahrsRuntime.accelKp = 3.0f;
        data.ahrsRuntime.accelKp = clampFloat(data.ahrsRuntime.accelKp, 0.0f, 20.0f);

        if (!finiteFloat(data.ahrsRuntime.maxAccelCorrectionDegPerUpdate) || data.ahrsRuntime.maxAccelCorrectionDegPerUpdate <= 0.0f) {
            data.ahrsRuntime.maxAccelCorrectionDegPerUpdate = 2.0f;
        }
        data.ahrsRuntime.maxAccelCorrectionDegPerUpdate = clampFloat(data.ahrsRuntime.maxAccelCorrectionDegPerUpdate, 0.01f, 20.0f);

        if (!finiteFloat(data.ahrsRuntime.accelNormGoodErrorG) || data.ahrsRuntime.accelNormGoodErrorG < 0.0f) data.ahrsRuntime.accelNormGoodErrorG = 0.06f;
        if (!finiteFloat(data.ahrsRuntime.accelNormBadErrorG) || data.ahrsRuntime.accelNormBadErrorG <= data.ahrsRuntime.accelNormGoodErrorG) data.ahrsRuntime.accelNormBadErrorG = 0.35f;
        data.ahrsRuntime.accelNormGoodErrorG = clampFloat(data.ahrsRuntime.accelNormGoodErrorG, 0.0f, 1.0f);
        data.ahrsRuntime.accelNormBadErrorG = clampFloat(data.ahrsRuntime.accelNormBadErrorG, data.ahrsRuntime.accelNormGoodErrorG + 0.001f, 2.0f);

        if (!finiteFloat(data.ahrsRuntime.accelInnovationGoodDeg) || data.ahrsRuntime.accelInnovationGoodDeg < 0.0f) data.ahrsRuntime.accelInnovationGoodDeg = 8.0f;
        if (!finiteFloat(data.ahrsRuntime.accelInnovationBadDeg) || data.ahrsRuntime.accelInnovationBadDeg <= data.ahrsRuntime.accelInnovationGoodDeg) data.ahrsRuntime.accelInnovationBadDeg = 45.0f;
        data.ahrsRuntime.accelInnovationGoodDeg = clampFloat(data.ahrsRuntime.accelInnovationGoodDeg, 0.0f, 90.0f);
        data.ahrsRuntime.accelInnovationBadDeg = clampFloat(data.ahrsRuntime.accelInnovationBadDeg, data.ahrsRuntime.accelInnovationGoodDeg + 0.1f, 180.0f);

        if (!finiteFloat(data.ahrsRuntime.accelNormStdGoodG) || data.ahrsRuntime.accelNormStdGoodG < 0.0f) data.ahrsRuntime.accelNormStdGoodG = 0.010f;
        if (!finiteFloat(data.ahrsRuntime.accelNormStdBadG) || data.ahrsRuntime.accelNormStdBadG <= data.ahrsRuntime.accelNormStdGoodG) data.ahrsRuntime.accelNormStdBadG = 0.080f;
        data.ahrsRuntime.accelNormStdGoodG = clampFloat(data.ahrsRuntime.accelNormStdGoodG, 0.0f, 0.50f);
        data.ahrsRuntime.accelNormStdBadG = clampFloat(data.ahrsRuntime.accelNormStdBadG, data.ahrsRuntime.accelNormStdGoodG + 0.001f, 2.0f);

        if (!finiteFloat(data.ahrsRuntime.accelNormVarianceAlpha) || data.ahrsRuntime.accelNormVarianceAlpha <= 0.0f) data.ahrsRuntime.accelNormVarianceAlpha = 0.02f;
        data.ahrsRuntime.accelNormVarianceAlpha = clampFloat(data.ahrsRuntime.accelNormVarianceAlpha, 0.001f, 1.0f);

        if (!finiteFloat(data.ahrsRuntime.gyroMotionGoodDps) || data.ahrsRuntime.gyroMotionGoodDps < 0.0f) data.ahrsRuntime.gyroMotionGoodDps = 250.0f;
        if (!finiteFloat(data.ahrsRuntime.gyroMotionBadDps) || data.ahrsRuntime.gyroMotionBadDps <= data.ahrsRuntime.gyroMotionGoodDps) data.ahrsRuntime.gyroMotionBadDps = 720.0f;
        data.ahrsRuntime.gyroMotionGoodDps = clampFloat(data.ahrsRuntime.gyroMotionGoodDps, 0.0f, 2000.0f);
        data.ahrsRuntime.gyroMotionBadDps = clampFloat(data.ahrsRuntime.gyroMotionBadDps, data.ahrsRuntime.gyroMotionGoodDps + 1.0f, 4000.0f);

        if (data.ahrsRuntime.normalizeEvery == 0 || data.ahrsRuntime.normalizeEvery > 4096) data.ahrsRuntime.normalizeEvery = 16;

        data.quality.largeGapFactor = clampFloat(data.quality.largeGapFactor, 1.01f, 10.0f);
        data.quality.accelNormOutlierMinG = clampFloat(data.quality.accelNormOutlierMinG, 0.01f, 2.0f);
        data.quality.accelNormOutlierMaxG = clampFloat(data.quality.accelNormOutlierMaxG, data.quality.accelNormOutlierMinG + 0.01f, 20.0f);

        if (data.output.outputRateHz == 0) data.output.outputRateHz = cfg::OUTPUT_RATE_HZ;
        if (data.output.outputRateHz > cfg::OUTPUT_RATE_HZ_MAX) data.output.outputRateHz = cfg::OUTPUT_RATE_HZ_MAX;
        if (data.output.packetFormat != 0) data.output.packetFormat = 0;

        updateCrc();
    }

    Lsm6dsv::Config makeLsmConfig() const {
        Lsm6dsv::Config cfg;
        cfg.accelOdr = data.imu.imuOdr;
        cfg.gyroOdr = data.imu.imuOdr;
        cfg.accelFs = data.imu.accelFs;
        cfg.gyroFs = data.imu.gyroFs;
        cfg.accelMode = data.imu.accelMode;
        cfg.gyroMode = data.imu.gyroMode;
        cfg.doSoftwareReset = data.imu.doSoftwareReset;
        cfg.disableI2cAndI3c = data.imu.disableI2cAndI3c;
        cfg.blockDataUpdate = data.imu.blockDataUpdate;
        cfg.autoIncrement = data.imu.autoIncrement;
        cfg.int1DrdyAccel = false;
        cfg.int1DrdyGyro = false;
        cfg.drdyPulsed = true;
        return cfg;
    }

    Lsm6dsvFifoReader::Config makeFifoConfig() const {
        Lsm6dsvFifoReader::Config cfg;
        cfg.accelBdr = data.fifo.accelBdr;
        cfg.gyroBdr = data.fifo.gyroBdr;
        cfg.watermarkWords = data.fifo.watermarkWords;
        cfg.mode = data.fifo.mode;
        cfg.routeWatermarkToInt1 = data.fifo.routeWatermarkToInt1;
        cfg.routeOverrunToInt1 = data.fifo.routeOverrunToInt1;
        cfg.routeFullToInt1 = data.fifo.routeFullToInt1;
        cfg.routeWatermarkToInt2 = false;
        cfg.routeOverrunToInt2 = false;
        cfg.routeFullToInt2 = false;
        cfg.timestampBatch = data.fifo.timestampBatch;
        cfg.temperatureBatch = data.fifo.temperatureBatch;
        cfg.enableTimestampCounter = data.fifo.enableTimestampCounter;
        cfg.useHardwareTimestamps = data.fifo.useHardwareTimestamps;
        cfg.allowTimestampFallback = data.fifo.allowTimestampFallback;
        cfg.maxWaitingSamplesBeforeFallback = data.fifo.maxWaitingSamplesBeforeFallback;
        cfg.samplePeriodUsOverride = data.fifo.samplePeriodUsOverride;

        // Phase 5: if magnetometer runtime is enabled in NVS/config,
        // accept QMC6309 XYZ words batched by LSM6DSV sensor hub slave 0.
        // The QMC stream is configured elsewhere; this only enables FIFO parsing.
        cfg.enableSensorHubSlave0 = data.magCal.driverEnabled;
        cfg.sensorHubSlave0PeriodUs = data.magCal.driverEnabled
            ? (1000000.0f / 60.0f)
            : 0.0f;
        return cfg;
    }

    Ahrs6DofConfig makeAhrsConfig() const {
        Ahrs6DofConfig cfg;
        cfg.minDtS = data.ahrsRuntime.minDtS;
        cfg.maxDtS = data.ahrsRuntime.maxDtS;
        cfg.clampLargeDt = data.ahrsRuntime.clampLargeDt;
        cfg.accelCorrectionEnabled = data.ahrs.useAccelCorrection && data.ahrsRuntime.accelCorrectionEnabled;
        cfg.adaptiveAccelCorrection = data.ahrsRuntime.adaptiveAccelCorrection;
        cfg.accelKp = data.ahrsRuntime.accelKp > 0.0f ? data.ahrsRuntime.accelKp : data.ahrs.accelCorrectionGain;
        if (!std::isfinite(cfg.accelKp) || cfg.accelKp <= 0.0f) cfg.accelKp = 3.0f;
        cfg.maxAccelCorrectionRadPerUpdate = data.ahrsRuntime.maxAccelCorrectionDegPerUpdate * MATH_DEG_TO_RAD;
        cfg.accelNormGoodErrorG = data.ahrsRuntime.accelNormGoodErrorG;
        cfg.accelNormBadErrorG = data.ahrsRuntime.accelNormBadErrorG;
        cfg.accelInnovationGoodRad = data.ahrsRuntime.accelInnovationGoodDeg * MATH_DEG_TO_RAD;
        cfg.accelInnovationBadRad = data.ahrsRuntime.accelInnovationBadDeg * MATH_DEG_TO_RAD;
        cfg.accelNormVarianceGoodG2 = square(data.ahrsRuntime.accelNormStdGoodG);
        cfg.accelNormVarianceBadG2 = square(data.ahrsRuntime.accelNormStdBadG);
        cfg.accelNormVarianceAlpha = data.ahrsRuntime.accelNormVarianceAlpha;
        cfg.gyroNormAccelTrustGoodRadS = data.ahrsRuntime.gyroMotionGoodDps * MATH_DEG_TO_RAD;
        cfg.gyroNormAccelTrustBadRadS = data.ahrsRuntime.gyroMotionBadDps * MATH_DEG_TO_RAD;
        cfg.normalizeEvery = data.ahrsRuntime.normalizeEvery;
        return cfg;
    }

    ImuQualityConfig makeQualityConfig() const {
        ImuQualityConfig cfg;
        cfg.expectedDtUs = data.quality.expectedDtUs;
        cfg.largeGapFactor = data.quality.largeGapFactor;
        cfg.smallGapFactor = data.quality.smallGapFactor;
        cfg.gyroNearSaturationAbsRaw = data.quality.gyroNearSaturationAbsRaw;
        cfg.accelNearSaturationAbsRaw = data.quality.accelNearSaturationAbsRaw;
        cfg.accelNormOutlierMinG = data.quality.accelNormOutlierMinG;
        cfg.accelNormOutlierMaxG = data.quality.accelNormOutlierMaxG;
        cfg.requestRecoveryOnFifoOverrun = data.quality.requestRecoveryOnFifoOverrun;
        cfg.requestRecoveryOnFifoFull = data.quality.requestRecoveryOnFifoFull;
        cfg.requestRecoveryOnUnknownTag = data.quality.requestRecoveryOnUnknownTag;
        cfg.requestRecoveryOnTimestampBackwards = data.quality.requestRecoveryOnTimestampBackwards;
        cfg.requestRecoveryOnTimestampQueueOverflow = data.quality.requestRecoveryOnTimestampQueueOverflow;
        cfg.skipAhrsOnBadTimestamp = data.quality.skipAhrsOnBadTimestamp;
        cfg.skipAhrsOnGyroSaturation = data.quality.skipAhrsOnGyroSaturation;
        cfg.disableAccelCorrectionOnAccelSaturation = data.quality.disableAccelCorrectionOnAccelSaturation;
        cfg.disableAccelCorrectionOnAccelNormOutlier = data.quality.disableAccelCorrectionOnAccelNormOutlier;
        return cfg;
    }

    void applyToImuCalibration(ImuCalibration& imuCal) const {
        imuCal.gyroBiasValid = data.gyroCal.biasValid;
        imuCal.gyroBiasRadS = data.gyroCal.biasRadS;

        imuCal.accelCalValid = data.accelCal.valid;
        imuCal.accelBiasG = data.accelCal.biasG;
        imuCal.accelScale = data.accelCal.scale;
    }

    void captureFromImuCalibration(const ImuCalibration& imuCal) {
        data.gyroCal.biasValid = imuCal.gyroBiasValid;
        data.gyroCal.biasRadS = imuCal.gyroBiasRadS;
        data.accelCal.valid = imuCal.accelCalValid;
        data.accelCal.biasG = imuCal.accelBiasG;
        data.accelCal.scale = imuCal.accelScale;
        updateCrc();
    }

    void noteGyroBiasCalibrationCaptured(uint32_t uptimeMs) {
        data.gyroCalMeta.biasCalibrationUptimeMs = uptimeMs;
        data.gyroCalMeta.biasCalibrationVersion = tracker_config_detail::SCHEMA_GYRO_CAL_VERSION;
        updateCrc();
    }

    void captureFromAccelCalibrationQuality(const Accel6PosCalibration& cal, uint32_t uptimeMs) {
        const Accel6PosCalibration::Result& r = cal.result();
        data.accelCalQuality.calibrationUptimeMs = uptimeMs;
        data.accelCalQuality.qualityFlags = r.qualityFlags;
        data.accelCalQuality.qualityScore = r.qualityScore;
        data.accelCalQuality.maxFaceNormErrorG = r.maxFaceNormErrorG;
        data.accelCalQuality.maxAxisResidualG = r.maxAxisResidualG;

        for (uint8_t i = 0; i < 6; ++i) {
            const auto face = static_cast<Accel6PosCalibration::Face>(i);
            data.accelCalQuality.faceSamples[i] = cal.faceData(face).samples;
            data.accelCalQuality.faceNormErrorG[i] = r.faceNormErrorG[i];
            data.accelCalQuality.faceAxisResidualG[i] = r.faceAxisResidualG[i];
        }
        updateCrc();
    }

    void captureFromMagCalibrationResult(const MagCalibrationResult& result,
                                         uint32_t sampleCount,
                                         uint32_t rejectedSamples,
                                         uint32_t saturatedSamples,
                                         float normMin,
                                         float normMean,
                                         float normMax,
                                         uint32_t uptimeMs) {
        data.magCal.calibrationValid = result.valid;
        if (result.valid) {
            data.magCal.hardIron = result.hardIron;
            data.magCal.softIron = result.softIron;
            data.magCal.expectedFieldNorm = result.expectedNorm;
            data.magCal.minTrustNorm = result.minTrustNorm;
            data.magCal.maxTrustNorm = result.maxTrustNorm;
        }

        data.magCalQuality.calibrationUptimeMs = uptimeMs;
        data.magCalQuality.sampleCount = sampleCount;
        data.magCalQuality.rejectedSamples = rejectedSamples;
        data.magCalQuality.saturatedSamples = saturatedSamples;
        data.magCalQuality.qualityFlags = result.valid ? 0u : 1u;
        data.magCalQuality.radiusX = result.radiusX;
        data.magCalQuality.radiusY = result.radiusY;
        data.magCalQuality.radiusZ = result.radiusZ;
        data.magCalQuality.normMin = normMin;
        data.magCalQuality.normMean = normMean;
        data.magCalQuality.normMax = normMax;
        float maxRadius = result.radiusX;
        if (result.radiusY > maxRadius) maxRadius = result.radiusY;
        if (result.radiusZ > maxRadius) maxRadius = result.radiusZ;
        float minRadius = result.radiusX;
        if (result.radiusY < minRadius) minRadius = result.radiusY;
        if (result.radiusZ < minRadius) minRadius = result.radiusZ;
        data.magCalQuality.coverageScore = (result.valid && maxRadius > 0.0f) ? (minRadius / maxRadius) : 0.0f;
        data.magCalQuality.residualRms = 0.0f;
        data.magCalQuality.expectedHorizontalNorm = 0.0f;
        updateCrc();
    }

    void applyToGyroTempComp(GyroTempCompensator& tempComp) const {
        GyroTempCompConfig cfg;
        cfg.enabled = data.gyroCal.tempCompEnabled;
        cfg.learningEnabled = data.gyroCal.tempLearningEnabled;
        cfg.calibratedTempMinC = data.gyroTempQuality.tempRangeMinC;
        cfg.calibratedTempMaxC = data.gyroTempQuality.tempRangeMaxC;
        cfg.fitQuality = data.gyroTempQuality.fitQuality;
        cfg.fitResidualBeforeDps = data.gyroTempQuality.residualBeforeDps;
        cfg.fitResidualAfterDps = data.gyroTempQuality.residualAfterDps;
        tempComp.setConfig(cfg);

        if (data.gyroCal.biasValid) {
            tempComp.reset(data.gyroCal.biasRadS, data.gyroCal.referenceTempC);
            if (data.gyroCal.tempCompValid) {
                tempComp.setSlopeRadSPerC(data.gyroCal.tempSlopeRadSPerC);
            }
        }
    }

    void captureFromGyroTempComp(const GyroTempCompensator& tempComp) {
        if (!tempComp.valid()) return;

        data.gyroCal.biasValid = true;
        data.gyroCal.biasRadS = tempComp.referenceBiasRadS();
        data.gyroCal.tempCompValid = true;
        data.gyroCal.tempCompEnabled = tempComp.config().enabled;
        data.gyroCal.tempLearningEnabled = tempComp.config().learningEnabled;
        data.gyroCal.referenceTempC = tempComp.referenceTempC();
        data.gyroCal.tempSlopeRadSPerC = tempComp.slopeRadSPerC();
        const GyroTempCompConfig& tc = tempComp.config();
        data.gyroTempQuality.tempRangeMinC = tc.calibratedTempMinC;
        data.gyroTempQuality.tempRangeMaxC = tc.calibratedTempMaxC;
        data.gyroTempQuality.fitQuality = tc.fitQuality;
        data.gyroTempQuality.residualBeforeDps = tc.fitResidualBeforeDps;
        data.gyroTempQuality.residualAfterDps = tc.fitResidualAfterDps;
        data.gyroCalMeta.tempModelUpdatedUptimeMs = millis();
        data.gyroCalMeta.tempModelVersion = tracker_config_detail::SCHEMA_GYRO_CAL_VERSION;
        updateCrc();
    }
};

enum class TrackerConfigError : uint8_t {
    None,
    NvsBeginFailed,
    NotFound,
    SizeMismatch,
    ReadFailed,
    WriteFailed,
    RemoveFailed,
    CrcOrValidationFailed,
};

struct TrackerConfigNvsInfo {
    bool beginOk = false;
    bool exists = false;
    size_t storedLen = 0;
    size_t expectedLen = sizeof(TrackerConfigBlob);
    uint32_t storedMagic = 0;
    uint16_t storedVersion = 0;
    uint16_t storedSize = 0;
    uint32_t storedCrc = 0;
    bool headerReadable = false;
    bool fullReadable = false;
    bool valid = false;
    TrackerConfigError error = TrackerConfigError::None;
};

class TrackerConfigStore {
public:
    explicit TrackerConfigStore(const char* nvsNamespace = tracker_config_detail::NVS_NAMESPACE,
                                const char* key = tracker_config_detail::NVS_KEY_CONFIG)
        : ns_(nvsNamespace), key_(key) {}

    TrackerConfigError lastError() const {
        return lastError_;
    }

    const char* lastErrorName() const {
        return errorName(lastError_);
    }

    static const char* errorName(TrackerConfigError e) {
        switch (e) {
            case TrackerConfigError::None:                  return "None";
            case TrackerConfigError::NvsBeginFailed:        return "NvsBeginFailed";
            case TrackerConfigError::NotFound:              return "NotFound";
            case TrackerConfigError::SizeMismatch:          return "SizeMismatch";
            case TrackerConfigError::ReadFailed:            return "ReadFailed";
            case TrackerConfigError::WriteFailed:           return "WriteFailed";
            case TrackerConfigError::RemoveFailed:          return "RemoveFailed";
            case TrackerConfigError::CrcOrValidationFailed: return "CrcOrValidationFailed";
        }
        return "Unknown";
    }

    bool inspect(TrackerConfigNvsInfo& info) {
        info = TrackerConfigNvsInfo{};
        Preferences prefs;
        if (!prefs.begin(ns_, true)) {
            info.error = TrackerConfigError::NvsBeginFailed;
            lastError_ = info.error;
            return false;
        }

        info.beginOk = true;
        info.storedLen = prefs.getBytesLength(key_);
        info.exists = info.storedLen > 0;

        if (!info.exists) {
            prefs.end();
            info.error = TrackerConfigError::NotFound;
            lastError_ = info.error;
            return true;
        }

        if (info.storedLen > sizeof(TrackerConfigBlob)) {
            prefs.end();
            info.error = TrackerConfigError::SizeMismatch;
            lastError_ = info.error;
            return true;
        }

        TrackerConfig tmp;
        const size_t readLen = prefs.getBytes(key_, &tmp.data, sizeof(tmp.data));
        prefs.end();

        if (readLen != info.storedLen) {
            info.error = TrackerConfigError::ReadFailed;
            lastError_ = info.error;
            return true;
        }

        if (readLen >= sizeof(tmp.data.magic) +
                       sizeof(tmp.data.version) +
                       sizeof(tmp.data.size) +
                       sizeof(tmp.data.crc32)) {
            info.headerReadable = true;
            info.storedMagic = tmp.data.magic;
            info.storedVersion = tmp.data.version;
            info.storedSize = tmp.data.size;
            info.storedCrc = tmp.data.crc32;
        }

        if (info.storedLen != sizeof(TrackerConfigBlob)) {
            info.error = TrackerConfigError::SizeMismatch;
            lastError_ = info.error;
            return true;
        }

        info.fullReadable = true;
        info.valid = tmp.validate();
        info.error = info.valid ? TrackerConfigError::None : TrackerConfigError::CrcOrValidationFailed;
        lastError_ = info.error;
        return true;
    }

    bool load(TrackerConfig& out) {
        Preferences prefs;
        if (!prefs.begin(ns_, true)) {
            lastError_ = TrackerConfigError::NvsBeginFailed;
            return false;
        }

        const size_t storedLen = prefs.getBytesLength(key_);
        if (storedLen == 0) {
            prefs.end();
            lastError_ = TrackerConfigError::NotFound;
            return false;
        }

        if (storedLen != sizeof(TrackerConfigBlob)) {
            prefs.end();
            lastError_ = TrackerConfigError::SizeMismatch;
            return false;
        }

        TrackerConfig tmp;
        const size_t readLen = prefs.getBytes(key_, &tmp.data, sizeof(tmp.data));
        prefs.end();

        if (readLen != sizeof(tmp.data)) {
            lastError_ = TrackerConfigError::ReadFailed;
            return false;
        }

        if (!tmp.validate()) {
            lastError_ = TrackerConfigError::CrcOrValidationFailed;
            return false;
        }

        out = tmp;
        lastError_ = TrackerConfigError::None;
        return true;
    }

    bool loadOrDefaults(TrackerConfig& out, bool* loadedFromNvs = nullptr) {
        if (load(out)) {
            if (loadedFromNvs) *loadedFromNvs = true;
            return true;
        }

        out.resetDefaults();
        if (loadedFromNvs) *loadedFromNvs = false;
        return true;
    }

    bool save(TrackerConfig config) {
        config.sanitize();
        config.updateCrc();

        if (!config.validate()) {
            lastError_ = TrackerConfigError::CrcOrValidationFailed;
            return false;
        }

        Preferences prefs;
        if (!prefs.begin(ns_, false)) {
            lastError_ = TrackerConfigError::NvsBeginFailed;
            return false;
        }

        const size_t written = prefs.putBytes(key_, &config.data, sizeof(config.data));
        prefs.end();

        if (written != sizeof(config.data)) {
            lastError_ = TrackerConfigError::WriteFailed;
            return false;
        }

        lastError_ = TrackerConfigError::None;
        return true;
    }

    bool erase() {
        Preferences prefs;
        if (!prefs.begin(ns_, false)) {
            lastError_ = TrackerConfigError::NvsBeginFailed;
            return false;
        }

        const bool ok = prefs.remove(key_);
        prefs.end();

        if (!ok) {
            lastError_ = TrackerConfigError::RemoveFailed;
            return false;
        }

        lastError_ = TrackerConfigError::None;
        return true;
    }

    bool exists() {
        Preferences prefs;
        if (!prefs.begin(ns_, true)) {
            lastError_ = TrackerConfigError::NvsBeginFailed;
            return false;
        }
        const size_t len = prefs.getBytesLength(key_);
        prefs.end();
        lastError_ = TrackerConfigError::None;
        return len == sizeof(TrackerConfigBlob);
    }

private:
    const char* ns_;
    const char* key_;
    TrackerConfigError lastError_ = TrackerConfigError::None;
};

// ============================================================
// Future network / SlimeVR persistent storage
// ============================================================
// Kept in a separate NVS namespace so normal config dumps can stay safe and
// calibration resets do not have to imply Wi-Fi credential resets. This is not
// wired to runtime yet; it reserves the schema before the next calibration pass.

namespace tracker_network_detail {
static constexpr uint32_t CONFIG_MAGIC = 0x544E4554UL; // 'TNET'
static constexpr uint16_t CONFIG_VERSION = 1;
static constexpr const char* NVS_NAMESPACE = "tracker_net";
static constexpr const char* NVS_KEY_CONFIG = "netcfg";
static constexpr uint16_t DEFAULT_SLIMEVR_PORT = 6969;
}

struct TrackerNetworkConfigBlob {
    uint32_t magic = tracker_network_detail::CONFIG_MAGIC;
    uint16_t version = tracker_network_detail::CONFIG_VERSION;
    uint16_t size = sizeof(TrackerNetworkConfigBlob);
    uint32_t crc32 = 0;

    bool wifiEnabled = false;
    bool credentialsValid = false;
    bool manualServerEnabled = false;
    bool discoveryEnabled = true;

    char ssid[33] = "";
    char password[65] = "";
    char serverHost[64] = "";
    uint16_t serverPort = tracker_network_detail::DEFAULT_SLIMEVR_PORT;

    uint32_t deviceId = 0;
    uint8_t sensorId = 0;
    uint8_t reserved0 = 0;
    uint16_t reserved1 = 0;
    char deviceName[32] = "c3_6dsv_tracker";
};

class TrackerNetworkConfig {
public:
    TrackerNetworkConfigBlob data;

    void resetDefaults() {
        data = TrackerNetworkConfigBlob{};
        updateCrc();
    }

    uint32_t computeCrc() const {
        TrackerNetworkConfigBlob tmp = data;
        tmp.crc32 = 0;
        return tracker_config_detail::fnv1a32(
            reinterpret_cast<const uint8_t*>(&tmp),
            sizeof(tmp)
        );
    }

    void updateCrc() {
        data.magic = tracker_network_detail::CONFIG_MAGIC;
        data.version = tracker_network_detail::CONFIG_VERSION;
        data.size = sizeof(TrackerNetworkConfigBlob);
        data.crc32 = 0;
        data.crc32 = computeCrc();
    }

    void sanitize() {
        data.ssid[sizeof(data.ssid) - 1] = '\0';
        data.password[sizeof(data.password) - 1] = '\0';
        data.serverHost[sizeof(data.serverHost) - 1] = '\0';
        data.deviceName[sizeof(data.deviceName) - 1] = '\0';
        if (data.deviceName[0] == '\0') {
            std::strncpy(data.deviceName, "c3_6dsv_tracker", sizeof(data.deviceName) - 1);
            data.deviceName[sizeof(data.deviceName) - 1] = '\0';
        }
        if (data.serverPort == 0) data.serverPort = tracker_network_detail::DEFAULT_SLIMEVR_PORT;
        if (!data.credentialsValid) {
            data.wifiEnabled = false;
        }
        updateCrc();
    }

    bool validate() const {
        if (data.magic != tracker_network_detail::CONFIG_MAGIC) return false;
        if (data.version != tracker_network_detail::CONFIG_VERSION) return false;
        if (data.size != sizeof(TrackerNetworkConfigBlob)) return false;
        if (computeCrc() != data.crc32) return false;
        if (data.serverPort == 0) return false;
        if (data.credentialsValid && data.ssid[0] == '\0') return false;
        return true;
    }
};

class TrackerNetworkConfigStore {
public:
    explicit TrackerNetworkConfigStore(const char* nvsNamespace = tracker_network_detail::NVS_NAMESPACE,
                                       const char* key = tracker_network_detail::NVS_KEY_CONFIG)
        : ns_(nvsNamespace), key_(key) {}

    TrackerConfigError lastError() const { return lastError_; }
    const char* lastErrorName() const { return TrackerConfigStore::errorName(lastError_); }

    bool load(TrackerNetworkConfig& out) {
        Preferences prefs;
        if (!prefs.begin(ns_, true)) {
            lastError_ = TrackerConfigError::NvsBeginFailed;
            return false;
        }
        const size_t storedLen = prefs.getBytesLength(key_);
        if (storedLen == 0) {
            prefs.end();
            lastError_ = TrackerConfigError::NotFound;
            return false;
        }
        if (storedLen != sizeof(TrackerNetworkConfigBlob)) {
            prefs.end();
            lastError_ = TrackerConfigError::SizeMismatch;
            return false;
        }
        TrackerNetworkConfig tmp;
        const size_t readLen = prefs.getBytes(key_, &tmp.data, sizeof(tmp.data));
        prefs.end();
        if (readLen != sizeof(tmp.data)) {
            lastError_ = TrackerConfigError::ReadFailed;
            return false;
        }
        if (!tmp.validate()) {
            lastError_ = TrackerConfigError::CrcOrValidationFailed;
            return false;
        }
        out = tmp;
        lastError_ = TrackerConfigError::None;
        return true;
    }

    bool save(TrackerNetworkConfig config) {
        config.sanitize();
        if (!config.validate()) {
            lastError_ = TrackerConfigError::CrcOrValidationFailed;
            return false;
        }
        Preferences prefs;
        if (!prefs.begin(ns_, false)) {
            lastError_ = TrackerConfigError::NvsBeginFailed;
            return false;
        }
        const size_t written = prefs.putBytes(key_, &config.data, sizeof(config.data));
        prefs.end();
        if (written != sizeof(config.data)) {
            lastError_ = TrackerConfigError::WriteFailed;
            return false;
        }
        lastError_ = TrackerConfigError::None;
        return true;
    }

    bool erase() {
        Preferences prefs;
        if (!prefs.begin(ns_, false)) {
            lastError_ = TrackerConfigError::NvsBeginFailed;
            return false;
        }
        const bool ok = prefs.remove(key_);
        prefs.end();
        lastError_ = ok ? TrackerConfigError::None : TrackerConfigError::RemoveFailed;
        return ok;
    }

private:
    const char* ns_;
    const char* key_;
    TrackerConfigError lastError_ = TrackerConfigError::None;
};

// ============================================================
// Serial-friendly summary helpers
// ============================================================

inline void printTrackerConfigSummary(Stream& out, const TrackerConfig& cfg) {
    out.println("==============================================================================");
    out.println("TRACKER CONFIG SUMMARY");
    out.println("==============================================================================");
    out.print("magic=0x"); out.print(cfg.data.magic, HEX);
    out.print(" version="); out.print(cfg.data.version);
    out.print(" size="); out.print(cfg.data.size);
    out.print(" crc=0x"); out.println(cfg.data.crc32, HEX);
    out.print("schema hardware/imu/fifo/ahrs/gyro/accel/mag/magyaw/quality/output/frame=");
    out.print(cfg.data.schemas.hardware); out.print('/');
    out.print(cfg.data.schemas.imu); out.print('/');
    out.print(cfg.data.schemas.fifo); out.print('/');
    out.print(cfg.data.schemas.ahrs); out.print('/');
    out.print(cfg.data.schemas.gyroCal); out.print('/');
    out.print(cfg.data.schemas.accelCal); out.print('/');
    out.print(cfg.data.schemas.magCal); out.print('/');
    out.print(cfg.data.schemas.magYaw); out.print('/');
    out.print(cfg.data.schemas.quality); out.print('/');
    out.print(cfg.data.schemas.output); out.print('/');
    out.println(cfg.data.schemas.frame);

    out.println("-- hardware --");
    out.print("pins sck/miso/mosi/cs/int1=");
    out.print(cfg.data.hardware.pinLsmSck); out.print('/');
    out.print(cfg.data.hardware.pinLsmMiso); out.print('/');
    out.print(cfg.data.hardware.pinLsmMosi); out.print('/');
    out.print(cfg.data.hardware.pinLsmCs); out.print('/');
    out.println(cfg.data.hardware.pinLsmInt1);
    out.print("serialBaud="); out.print(cfg.data.hardware.serialBaud);
    out.print(" spiHz="); out.println(cfg.data.hardware.spiHz);

    out.println("-- imu/fifo --");
    out.print("imuOdrHz="); out.println(Lsm6dsv::odrHz(cfg.data.imu.imuOdr), 3);
    out.print("fifoAccelBdrHz="); out.println(Lsm6dsv::odrHz(cfg.data.fifo.accelBdr), 3);
    out.print("fifoGyroBdrHz="); out.println(Lsm6dsv::odrHz(cfg.data.fifo.gyroBdr), 3);
    out.print("samplePeriodUsOverride="); out.println(cfg.data.fifo.samplePeriodUsOverride, 3);
    out.print("fifoWatermarkWords="); out.println(cfg.data.fifo.watermarkWords);
    out.print("maxWordsPerDrain="); out.println(cfg.data.fifo.maxWordsPerDrain);
    out.print("maxDrainRoundsPerEvent="); out.println(cfg.data.fifo.maxDrainRoundsPerEvent);
    out.print("useHardwareTimestamps="); out.println(cfg.data.fifo.useHardwareTimestamps ? "yes" : "no");
    out.print("timestampFallback="); out.println(cfg.data.fifo.allowTimestampFallback ? "yes" : "no");

    out.println("-- ahrs effective config --");
    out.print("ahrsAccelCorrectionEnabled="); out.println((cfg.data.ahrs.useAccelCorrection && cfg.data.ahrsRuntime.accelCorrectionEnabled) ? "yes" : "no");
    out.print("ahrsAdaptiveAccelCorrection="); out.println(cfg.data.ahrsRuntime.adaptiveAccelCorrection ? "yes" : "no");
    out.print("ahrsAccelKp="); out.println(cfg.data.ahrsRuntime.accelKp, 6);
    out.print("ahrsDtMinMaxS="); out.print(cfg.data.ahrsRuntime.minDtS, 7); out.print(','); out.println(cfg.data.ahrsRuntime.maxDtS, 7);
    out.print("ahrsClampLargeDt="); out.println(cfg.data.ahrsRuntime.clampLargeDt ? "yes" : "no");
    out.print("ahrsMaxAccelCorrectionDeg="); out.println(cfg.data.ahrsRuntime.maxAccelCorrectionDegPerUpdate, 6);
    out.print("ahrsAccelNormGoodBadErrG="); out.print(cfg.data.ahrsRuntime.accelNormGoodErrorG, 6); out.print(','); out.println(cfg.data.ahrsRuntime.accelNormBadErrorG, 6);
    out.print("ahrsAccelInnovationGoodBadDeg="); out.print(cfg.data.ahrsRuntime.accelInnovationGoodDeg, 3); out.print(','); out.println(cfg.data.ahrsRuntime.accelInnovationBadDeg, 3);
    out.print("ahrsAccelNormStdGoodBadG="); out.print(cfg.data.ahrsRuntime.accelNormStdGoodG, 6); out.print(','); out.println(cfg.data.ahrsRuntime.accelNormStdBadG, 6);
    out.print("ahrsGyroMotionGoodBadDps="); out.print(cfg.data.ahrsRuntime.gyroMotionGoodDps, 3); out.print(','); out.println(cfg.data.ahrsRuntime.gyroMotionBadDps, 3);
    out.print("ahrsNormalizeEvery="); out.println(cfg.data.ahrsRuntime.normalizeEvery);

    out.println("-- gyro calibration --");
    out.print("gyroBiasValid="); out.println(cfg.data.gyroCal.biasValid ? "yes" : "no");
    out.print("gyroBiasRadS=");
    out.print(cfg.data.gyroCal.biasRadS.x, 8); out.print(',');
    out.print(cfg.data.gyroCal.biasRadS.y, 8); out.print(',');
    out.println(cfg.data.gyroCal.biasRadS.z, 8);
    out.print("gyroBiasDps=");
    const Vec3 gyroBiasDps = cfg.data.gyroCal.biasRadS * MATH_RAD_TO_DEG;
    out.print(gyroBiasDps.x, 5); out.print(',');
    out.print(gyroBiasDps.y, 5); out.print(',');
    out.println(gyroBiasDps.z, 5);
    out.print("tempCompValid="); out.println(cfg.data.gyroCal.tempCompValid ? "yes" : "no");
    out.print("tempCompEnabled="); out.println(cfg.data.gyroCal.tempCompEnabled ? "yes" : "no");
    out.print("referenceTempC="); out.println(cfg.data.gyroCal.referenceTempC, 3);
    const Vec3 tempSlopeDps = cfg.data.gyroCal.tempSlopeRadSPerC * MATH_RAD_TO_DEG;
    out.print("tempSlopeDpsPerC=");
    out.print(tempSlopeDps.x, 8); out.print(',');
    out.print(tempSlopeDps.y, 8); out.print(',');
    out.println(tempSlopeDps.z, 8);
    out.print("tempRangeMinMaxC="); out.print(cfg.data.gyroTempQuality.tempRangeMinC, 3); out.print(','); out.println(cfg.data.gyroTempQuality.tempRangeMaxC, 3);
    out.print("tempFitQuality="); out.println(cfg.data.gyroTempQuality.fitQuality, 6);
    out.print("tempFitResidualBeforeAfterDps="); out.print(cfg.data.gyroTempQuality.residualBeforeDps, 6); out.print(','); out.println(cfg.data.gyroTempQuality.residualAfterDps, 6);
    out.print("gyroBiasCalibrationUptimeMs="); out.println(cfg.data.gyroCalMeta.biasCalibrationUptimeMs);
    out.print("gyroTempModelUpdatedUptimeMs="); out.println(cfg.data.gyroCalMeta.tempModelUpdatedUptimeMs);
    out.print("gyroTempModelSampleCount="); out.println(cfg.data.gyroCalMeta.tempModelSampleCount);

    out.println("-- accel calibration --");
    out.print("accelCalValid="); out.println(cfg.data.accelCal.valid ? "yes" : "no");
    out.print("accelBiasG=");
    out.print(cfg.data.accelCal.biasG.x, 8); out.print(',');
    out.print(cfg.data.accelCal.biasG.y, 8); out.print(',');
    out.println(cfg.data.accelCal.biasG.z, 8);
    out.print("accelScaleDiag=");
    out.print(cfg.data.accelCal.scale.m[0][0], 8); out.print(',');
    out.print(cfg.data.accelCal.scale.m[1][1], 8); out.print(',');
    out.println(cfg.data.accelCal.scale.m[2][2], 8);
    out.print("accelScaleMatrix=");
    out.print(cfg.data.accelCal.scale.m[0][0], 8); out.print(',');
    out.print(cfg.data.accelCal.scale.m[0][1], 8); out.print(',');
    out.print(cfg.data.accelCal.scale.m[0][2], 8); out.print(';');
    out.print(cfg.data.accelCal.scale.m[1][0], 8); out.print(',');
    out.print(cfg.data.accelCal.scale.m[1][1], 8); out.print(',');
    out.print(cfg.data.accelCal.scale.m[1][2], 8); out.print(';');
    out.print(cfg.data.accelCal.scale.m[2][0], 8); out.print(',');
    out.print(cfg.data.accelCal.scale.m[2][1], 8); out.print(',');
    out.println(cfg.data.accelCal.scale.m[2][2], 8);
    out.print("accelCalQualityScore="); out.println(cfg.data.accelCalQuality.qualityScore, 6);
    out.print("accelCalQualityFlags=0x"); out.println(cfg.data.accelCalQuality.qualityFlags, HEX);
    out.print("accelCalUptimeMs="); out.println(cfg.data.accelCalQuality.calibrationUptimeMs);
    out.print("accelCalMaxFaceNormErrorG="); out.println(cfg.data.accelCalQuality.maxFaceNormErrorG, 8);
    out.print("accelCalMaxAxisResidualG="); out.println(cfg.data.accelCalQuality.maxAxisResidualG, 8);
    out.print("accelCalFaceSamples=");
    for (uint8_t i = 0; i < 6; ++i) { if (i) out.print(','); out.print(cfg.data.accelCalQuality.faceSamples[i]); }
    out.println();

    out.println("-- magnetometer --");
    out.print("magDriverEnabled="); out.println(cfg.data.magCal.driverEnabled ? "yes" : "no");
    out.print("magCalibrationValid="); out.println(cfg.data.magCal.calibrationValid ? "yes" : "no");
    out.print("magAxisAlignmentValid="); out.println(cfg.data.magCal.axisAlignmentValid ? "yes" : "no");
    out.print("magHardIron=");
    out.print(cfg.data.magCal.hardIron.x, 6); out.print(',');
    out.print(cfg.data.magCal.hardIron.y, 6); out.print(',');
    out.println(cfg.data.magCal.hardIron.z, 6);
    out.print("magExpectedFieldNorm="); out.println(cfg.data.magCal.expectedFieldNorm, 6);
    out.print("magTrustNormMinMax=");
    out.print(cfg.data.magCal.minTrustNorm, 6); out.print(',');
    out.println(cfg.data.magCal.maxTrustNorm, 6);
    out.print("magCalUptimeMs="); out.println(cfg.data.magCalQuality.calibrationUptimeMs);
    out.print("magCalSamplesRejectedSaturated="); out.print(cfg.data.magCalQuality.sampleCount); out.print(','); out.print(cfg.data.magCalQuality.rejectedSamples); out.print(','); out.println(cfg.data.magCalQuality.saturatedSamples);
    out.print("magCalRadiusXYZ="); out.print(cfg.data.magCalQuality.radiusX, 3); out.print(','); out.print(cfg.data.magCalQuality.radiusY, 3); out.print(','); out.println(cfg.data.magCalQuality.radiusZ, 3);
    out.print("magCalNormMinMeanMax="); out.print(cfg.data.magCalQuality.normMin, 3); out.print(','); out.print(cfg.data.magCalQuality.normMean, 3); out.print(','); out.println(cfg.data.magCalQuality.normMax, 3);
    out.print("magCalCoverageScore="); out.println(cfg.data.magCalQuality.coverageScore, 6);

    out.println("-- frame/device --");
    out.print("sensorToDeviceValid="); out.println(cfg.data.frame.sensorToDeviceValid ? "yes" : "no");
    out.print("applyMountingOffsetInFirmware="); out.println(cfg.data.frame.applyMountingOffsetInFirmware ? "yes" : "no");
    out.print("outputConvention="); out.println(cfg.data.frame.outputConvention);
    out.print("deviceId="); out.println(cfg.data.device.deviceId);
    out.print("sensorId="); out.println(cfg.data.device.sensorId);
    out.print("deviceName="); out.println(cfg.data.device.deviceName);

    out.println("-- mag yaw correction --");
    out.print("magYawControllerEnabled="); out.println(cfg.data.magYaw.controllerEnabled ? "yes" : "no");
    out.print("magYawApplyEnabled="); out.println(cfg.data.magYaw.applyEnabled ? "yes" : "no");
    out.print("magYawRequireAccelTrusted="); out.println(cfg.data.magYaw.requireAccelTrusted ? "yes" : "no");

    out.print("magYawMaxInnovationDeg="); out.println(cfg.data.magYaw.maxInnovationDeg, 3);

    out.print("magYawHorizontalBadGood=");
    out.print(cfg.data.magYaw.horizontalNormBad, 3); out.print(',');
    out.println(cfg.data.magYaw.horizontalNormGood, 3);

    out.print("magYawGyroGoodBadDps=");
    out.print(cfg.data.magYaw.gyroNormGoodDps, 3); out.print(',');
    out.println(cfg.data.magYaw.gyroNormBadDps, 3);

    out.print("magYawAccelTrustBadGood=");
    out.print(cfg.data.magYaw.accelTrustBad, 3); out.print(',');
    out.println(cfg.data.magYaw.accelTrustGood, 3);

    out.print("magYawMaxAgeMs="); out.println(cfg.data.magYaw.maxMagAgeMs);
    out.print("magYawTimeConstantS="); out.println(cfg.data.magYaw.timeConstantS, 3);
    out.print("magYawMaxRateDegS="); out.println(cfg.data.magYaw.maxCorrectionRateDegS, 3);
    out.print("magYawMaxStepDeg="); out.println(cfg.data.magYaw.maxCorrectionStepDeg, 3);
    out.print("magYawCooldownGyroAccelMagMs="); out.print(cfg.data.magYaw.gyroMovingCooldownMs); out.print(','); out.print(cfg.data.magYaw.accelBadCooldownMs); out.print(','); out.println(cfg.data.magYaw.magDisturbanceCooldownMs);
    out.print("magYawFallbackDtS="); out.println(cfg.data.magYaw.fallbackDtS, 6);

    out.println("-- quality --");
    out.print("largeGapFactor="); out.println(cfg.data.quality.largeGapFactor, 3);
    out.print("gyroNearSatRaw="); out.println(cfg.data.quality.gyroNearSaturationAbsRaw);
    out.print("accelNearSatRaw="); out.println(cfg.data.quality.accelNearSaturationAbsRaw);

    out.println("-- output --");
    out.print("serialDebugEnabled="); out.println(cfg.data.output.serialDebugEnabled ? "yes" : "no");
    out.print("quaternionOutputEnabled="); out.println(cfg.data.output.quaternionOutputEnabled ? "yes" : "no");
    out.print("outputRateHz="); out.println(cfg.data.output.outputRateHz);
    out.print("packetFormat="); out.println(cfg.data.output.packetFormat);

    out.println("-- future network storage --");
    out.print("networkNvsNamespace="); out.println(tracker_network_detail::NVS_NAMESPACE);
    out.print("networkNvsKey="); out.println(tracker_network_detail::NVS_KEY_CONFIG);
    out.println("networkRuntimeWired=no");
    out.println("==============================================================================");
}

inline void printTrackerNetworkConfigSummary(Stream& out, const TrackerNetworkConfig& cfg, bool revealSecrets = false) {
    out.println("==============================================================================");
    out.println("TRACKER NETWORK CONFIG SUMMARY");
    out.println("==============================================================================");
    out.print("magic=0x"); out.print(cfg.data.magic, HEX);
    out.print(" version="); out.print(cfg.data.version);
    out.print(" size="); out.print(cfg.data.size);
    out.print(" crc=0x"); out.println(cfg.data.crc32, HEX);
    out.print("wifiEnabled="); out.println(cfg.data.wifiEnabled ? "yes" : "no");
    out.print("credentialsValid="); out.println(cfg.data.credentialsValid ? "yes" : "no");
    out.print("ssid="); out.println(cfg.data.ssid);
    out.print("password="); out.println(revealSecrets ? cfg.data.password : "<hidden>");
    out.print("discoveryEnabled="); out.println(cfg.data.discoveryEnabled ? "yes" : "no");
    out.print("manualServerEnabled="); out.println(cfg.data.manualServerEnabled ? "yes" : "no");
    out.print("serverHost="); out.println(cfg.data.serverHost);
    out.print("serverPort="); out.println(cfg.data.serverPort);
    out.print("deviceId="); out.println(cfg.data.deviceId);
    out.print("sensorId="); out.println(cfg.data.sensorId);
    out.print("deviceName="); out.println(cfg.data.deviceName);
    out.println("==============================================================================");
}

} // namespace tracker