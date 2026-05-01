#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <cstdint>
#include <cstring>
#include <cmath>

#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/calibration.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"

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
static constexpr uint16_t CONFIG_VERSION = 1;
static constexpr const char* NVS_NAMESPACE = "tracker";
static constexpr const char* NVS_KEY_CONFIG = "cfg";

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

struct TrackerHardwareConfig {
    int pinLsmSck = 3;
    int pinLsmMiso = 0;
    int pinLsmMosi = 2;
    int pinLsmCs = 1;
    int pinLsmInt1 = 10;

    uint32_t serialBaud = 921600;
    uint32_t spiHz = 1000000;
    uint8_t spiMode = SPI_MODE0;
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

    uint8_t watermarkWords = 48;
    uint16_t maxWordsPerDrain = 384;
    uint8_t maxDrainRoundsPerEvent = 6;

    Lsm6dsvFifoReader::FifoMode mode = Lsm6dsvFifoReader::FifoMode::Continuous;
    Lsm6dsvFifoReader::TimestampBatch timestampBatch = Lsm6dsvFifoReader::TimestampBatch::Decimation1;
    Lsm6dsvFifoReader::TemperatureBatch temperatureBatch = Lsm6dsvFifoReader::TemperatureBatch::Hz1_875;

    bool routeWatermarkToInt1 = true;
    bool routeOverrunToInt1 = true;
    bool routeFullToInt1 = true;

    bool enableTimestampCounter = true;
    bool useHardwareTimestamps = true;
    bool allowTimestampFallback = true;
    uint8_t maxWaitingSamplesBeforeFallback = 32;

    // 0 => compute from ODR + INTERNAL_FREQ_FINE.
    float samplePeriodUsOverride = 0.0f;
};

struct TrackerAhrsConfig {
    // These are generic placeholders. Keep actual AHRS defaults inside Ahrs6Dof
    // until that class exposes a public Config struct.
    float accelCorrectionGain = 0.0f;
    float accelTrustMinNormG = 0.50f;
    float accelTrustMaxNormG = 1.50f;
    bool useAccelCorrection = true;

    // Mounting offset from sensor frame to tracker/body frame.
    bool mountingOffsetValid = false;
    Quat mountingOffset = Quat::identity();
};

struct TrackerGyroCalibrationConfig {
    bool biasValid = false;
    Vec3 biasRadS = Vec3::zero();

    bool tempCompValid = false;
    bool tempCompEnabled = true;
    bool tempLearningEnabled = false;
    float referenceTempC = 25.0f;
    Vec3 tempSlopeRadSPerC = Vec3::zero();
};

struct TrackerAccelCalibrationConfig {
    bool valid = false;
    Vec3 biasG = Vec3::zero();
    Mat3 scale = Mat3::identity();
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
    uint16_t outputRateHz = 100;

    // Future packet format enum placeholder:
    // 0 = debug text, 1 = binary custom, 2 = SlimeVR-compatible.
    uint8_t packetFormat = 0;
};

struct TrackerConfigBlob {
    uint32_t magic = tracker_config_detail::CONFIG_MAGIC;
    uint16_t version = tracker_config_detail::CONFIG_VERSION;
    uint16_t size = sizeof(TrackerConfigBlob);
    uint32_t crc32 = 0;

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

    uint32_t reservedU32[16] = {};
    float reservedF32[4] = {};
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

        if (data.ahrs.mountingOffsetValid && !finiteQuat(data.ahrs.mountingOffset)) return false;

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

        if (data.hardware.serialBaud == 0) data.hardware.serialBaud = 921600;
        if (data.hardware.spiHz == 0) data.hardware.spiHz = 1000000;

        if (data.fifo.watermarkWords == 0) data.fifo.watermarkWords = 48;
        if (data.fifo.maxWordsPerDrain == 0) data.fifo.maxWordsPerDrain = 384;
        if (data.fifo.maxDrainRoundsPerEvent == 0) data.fifo.maxDrainRoundsPerEvent = 6;
        if (!finiteFloat(data.fifo.samplePeriodUsOverride)) data.fifo.samplePeriodUsOverride = 0.0f;
        data.fifo.maxWaitingSamplesBeforeFallback = static_cast<uint8_t>(
            data.fifo.maxWaitingSamplesBeforeFallback == 0 ? 32 : data.fifo.maxWaitingSamplesBeforeFallback
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

        if (!finiteQuat(data.ahrs.mountingOffset)) {
            data.ahrs.mountingOffset = Quat::identity();
            data.ahrs.mountingOffsetValid = false;
        }

        data.quality.largeGapFactor = clampFloat(data.quality.largeGapFactor, 1.01f, 10.0f);
        data.quality.accelNormOutlierMinG = clampFloat(data.quality.accelNormOutlierMinG, 0.01f, 2.0f);
        data.quality.accelNormOutlierMaxG = clampFloat(data.quality.accelNormOutlierMaxG, data.quality.accelNormOutlierMinG + 0.01f, 20.0f);

        if (data.output.outputRateHz == 0) data.output.outputRateHz = 100;
        if (data.output.outputRateHz > 1000) data.output.outputRateHz = 1000;

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

    void applyToGyroTempComp(GyroTempCompensator& tempComp) const {
        GyroTempCompConfig cfg;
        cfg.enabled = data.gyroCal.tempCompEnabled;
        cfg.learningEnabled = data.gyroCal.tempLearningEnabled;
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
    out.print("fifoWatermarkWords="); out.println(cfg.data.fifo.watermarkWords);
    out.print("maxWordsPerDrain="); out.println(cfg.data.fifo.maxWordsPerDrain);
    out.print("maxDrainRoundsPerEvent="); out.println(cfg.data.fifo.maxDrainRoundsPerEvent);
    out.print("useHardwareTimestamps="); out.println(cfg.data.fifo.useHardwareTimestamps ? "yes" : "no");
    out.print("timestampFallback="); out.println(cfg.data.fifo.allowTimestampFallback ? "yes" : "no");

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
    out.print("referenceTempC="); out.println(cfg.data.gyroCal.referenceTempC, 3);

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

    out.println("-- quality --");
    out.print("largeGapFactor="); out.println(cfg.data.quality.largeGapFactor, 3);
    out.print("gyroNearSatRaw="); out.println(cfg.data.quality.gyroNearSaturationAbsRaw);
    out.print("accelNearSatRaw="); out.println(cfg.data.quality.accelNearSaturationAbsRaw);

    out.println("-- output --");
    out.print("serialDebugEnabled="); out.println(cfg.data.output.serialDebugEnabled ? "yes" : "no");
    out.print("quaternionOutputEnabled="); out.println(cfg.data.output.quaternionOutputEnabled ? "yes" : "no");
    out.print("outputRateHz="); out.println(cfg.data.output.outputRateHz);
    out.println("==============================================================================");
}

} // namespace tracker