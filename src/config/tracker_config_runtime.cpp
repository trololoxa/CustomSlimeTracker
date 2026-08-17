#include "config/tracker_config_runtime.hpp"

#include <cstring>
#include <cmath>

#include "sensor/calibration.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/frame_transform.hpp"
#include "sensor/mag_calibration.hpp"

namespace tracker {

namespace {

bool validSlimeVRMotionPacketPolicyValue(uint8_t value) {
    return value <= static_cast<uint8_t>(SlimeVRMotionPacketPolicy::RotationAcceleration23);
}

uint8_t encodeSlimeVRMotionPacketPolicy(SlimeVRMotionPacketPolicy policy) {
    return static_cast<uint8_t>(tracker_config_detail::OUTPUT_PACKET_MODE_MARKER |
        (static_cast<uint8_t>(policy) & tracker_config_detail::OUTPUT_PACKET_MODE_VALUE_MASK));
}

} // namespace


SlimeVRMotionPacketPolicy TrackerConfig::slimevrMotionPacketPolicy() const {
    const uint8_t encoded = data.output.packetFormat;
    if ((encoded & tracker_config_detail::OUTPUT_PACKET_MODE_MARKER) == 0u ||
        (encoded & static_cast<uint8_t>(~tracker_config_detail::OUTPUT_PACKET_MODE_ALLOWED_MASK)) != 0u) {
        return SlimeVRMotionPacketPolicy::QuaternionOnly;
    }
    const uint8_t value = encoded & tracker_config_detail::OUTPUT_PACKET_MODE_VALUE_MASK;
    if (!validSlimeVRMotionPacketPolicyValue(value)) {
        return SlimeVRMotionPacketPolicy::QuaternionOnly;
    }
    return static_cast<SlimeVRMotionPacketPolicy>(value);
}

void TrackerConfig::setSlimeVRMotionPacketPolicy(SlimeVRMotionPacketPolicy policy) {
    const uint8_t value = static_cast<uint8_t>(policy);
    data.output.packetFormat = encodeSlimeVRMotionPacketPolicy(
        validSlimeVRMotionPacketPolicyValue(value) ? policy : SlimeVRMotionPacketPolicy::QuaternionOnly);
    updateCrc();
}

void TrackerConfig::resetDefaults() {
    data = TrackerConfigBlob{};
    updateCrc();
}

bool TrackerConfig::validate() const {
    if (data.magic != tracker_config_detail::CONFIG_MAGIC) return false;
    if (data.version != tracker_config_detail::CONFIG_VERSION) return false;
    if (data.size != sizeof(TrackerConfigBlob)) return false;
    if (computeCrc() != data.crc32) return false;
    return validateContent();
}

bool TrackerConfig::validateContent() const {
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
        const float maxSlopeRadSPerC = GyroTempCompConfig{}.maxAcceptedSlopeDpsPerC * MATH_DEG_TO_RAD;
        if (std::fabs(data.gyroCal.tempSlopeRadSPerC.x) > maxSlopeRadSPerC ||
            std::fabs(data.gyroCal.tempSlopeRadSPerC.y) > maxSlopeRadSPerC ||
            std::fabs(data.gyroCal.tempSlopeRadSPerC.z) > maxSlopeRadSPerC) {
            return false;
        }
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

    // Keep finite legacy frame values loadable so one stale frame does not
    // discard the whole NVS blob. Runtime enables only proper rotations and
    // sanitize() clears invalid scale/shear/reflection matrices on next save.
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

uint32_t TrackerConfig::computeCrc() const {
    TrackerConfigBlob tmp = data;
    tmp.crc32 = 0;
    return tracker_config_detail::fnv1a32(
        reinterpret_cast<const uint8_t*>(&tmp),
        sizeof(tmp)
    );
}

void TrackerConfig::updateCrc() {
    data.magic = tracker_config_detail::CONFIG_MAGIC;
    data.version = tracker_config_detail::CONFIG_VERSION;
    data.size = sizeof(TrackerConfigBlob);
    data.crc32 = 0;
    data.crc32 = computeCrc();
}

void TrackerConfig::sanitize() {
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
    if (!data.gyroCal.biasValid) {
        data.gyroCal.biasRadS = Vec3::zero();
        data.gyroCalMeta.biasCalibrationUptimeMs = 0;
    }
    if (!finiteFloat(data.gyroCal.referenceTempC)) data.gyroCal.referenceTempC = 25.0f;
    // A temperature slope has no standalone meaning without the reference
    // gyro bias it modifies. Older/corrupted blobs could represent this
    // impossible state; normalize it instead of letting storage revision,
    // diagnostics and runtime disagree about which model is active.
    if (!data.gyroCal.biasValid && data.gyroCal.tempCompValid) {
        data.gyroCal.tempCompValid = false;
        data.gyroCal.tempSlopeRadSPerC = Vec3::zero();
        data.gyroTempQuality = TrackerGyroTempQualityConfigPersisted{};
        data.gyroCalMeta.tempModelUpdatedUptimeMs = 0;
        data.gyroCalMeta.tempModelSampleCount = 0;
    }
    const float maxSlopeRadSPerC = GyroTempCompConfig{}.maxAcceptedSlopeDpsPerC * MATH_DEG_TO_RAD;
    if (!finiteVec3(data.gyroCal.tempSlopeRadSPerC) ||
        std::fabs(data.gyroCal.tempSlopeRadSPerC.x) > maxSlopeRadSPerC ||
        std::fabs(data.gyroCal.tempSlopeRadSPerC.y) > maxSlopeRadSPerC ||
        std::fabs(data.gyroCal.tempSlopeRadSPerC.z) > maxSlopeRadSPerC) {
        data.gyroCal.tempSlopeRadSPerC = Vec3::zero();
        data.gyroCal.tempCompValid = false;
        data.gyroTempQuality = TrackerGyroTempQualityConfigPersisted{};
        data.gyroCalMeta.tempModelUpdatedUptimeMs = 0;
        data.gyroCalMeta.tempModelSampleCount = 0;
    }
    if (!data.gyroCal.tempCompValid) {
        data.gyroCal.tempSlopeRadSPerC = Vec3::zero();
        data.gyroTempQuality = TrackerGyroTempQualityConfigPersisted{};
        data.gyroCalMeta.tempModelUpdatedUptimeMs = 0;
        data.gyroCalMeta.tempModelSampleCount = 0;
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
    if (!data.accelCal.valid) {
        data.accelCal.biasG = Vec3::zero();
        data.accelCal.scale = Mat3::identity();
        data.accelCalQuality = TrackerAccelCalibrationQualityPersisted{};
    }

    const bool invalidMagFieldModel =
        !finiteMat3(data.magCal.softIron) ||
        !finiteVec3(data.magCal.hardIron) ||
        !finiteFloat(data.magCal.expectedFieldNorm);
    if (invalidMagFieldModel) {
        data.magCal.calibrationValid = false;
    }
    if (!data.magCal.calibrationValid) {
        data.magCal.hardIron = Vec3::zero();
        data.magCal.softIron = Mat3::identity();
        data.magCal.expectedFieldNorm = 1.0f;
        data.magCal.minTrustNorm = 0.25f;
        data.magCal.maxTrustNorm = 2.50f;
        data.magCalQuality = TrackerMagCalibrationQualityPersisted{};
    }
    if (!finiteMat3(data.magCal.magToImu)) {
        data.magCal.axisAlignmentValid = false;
    }
    if (!data.magCal.axisAlignmentValid) {
        data.magCal.magToImu = Mat3::identity();
    }
    if (!data.accelCal.valid || !data.magCal.calibrationValid ||
        !data.magCal.axisAlignmentValid) {
        data.magYaw.applyEnabled = false;
    }

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
    data.magCalQuality.coverageScore = clampFloat(data.magCalQuality.coverageScore, 0.0f, 1.0f);

    if (!isProperRotationMatrix(data.frame.sensorToDevice)) {
        data.frame.sensorToDeviceValid = false;
    }
    if (!data.frame.sensorToDeviceValid) {
        data.frame.sensorToDevice = Mat3::identity();
    }
    // Compatibility bytes from abandoned firmware-side mounting/output/identity
    // concepts are always neutral. Keeping their layout preserves existing NVS
    // blobs without advertising a non-functional capability.
    data.ahrs.reservedAccelTrustMinNormG = 0.94f;
    data.ahrs.reservedAccelTrustMaxNormG = 1.35f;
    data.ahrs.reservedMountingOffsetValid = false;
    data.ahrs.reservedMountingOffset = Quat::identity();
    data.gyroCal.reservedTempLearningEnabled = false;
    data.magCalQuality.reservedExpectedHorizontalNorm = 0.0f;
    data.frame.reservedApplyMountingOffsetInFirmware = false;
    data.frame.reservedOutputConvention = 0;
    data.frame.reservedFlags = 0;
    data.reservedDevice = TrackerReservedDeviceIdentityPersisted{};

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
    const uint8_t packetModeValue = data.output.packetFormat &
        tracker_config_detail::OUTPUT_PACKET_MODE_VALUE_MASK;
    const bool packetModeMarked =
        (data.output.packetFormat & tracker_config_detail::OUTPUT_PACKET_MODE_MARKER) != 0u;
    const bool packetModeReservedBitsClear =
        (data.output.packetFormat &
         static_cast<uint8_t>(~tracker_config_detail::OUTPUT_PACKET_MODE_ALLOWED_MASK)) == 0u;
    if (!packetModeMarked || !packetModeReservedBitsClear ||
        !validSlimeVRMotionPacketPolicyValue(packetModeValue)) {
        // Every historical packetFormat value is unmarked. Do not reinterpret
        // old local-output values as packet 23 or acceleration output.
        data.output.packetFormat = encodeSlimeVRMotionPacketPolicy(
            SlimeVRMotionPacketPolicy::QuaternionOnly);
    }

    updateCrc();
}

Lsm6dsv::Config TrackerConfig::makeLsmConfig() const {
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

Lsm6dsvFifoReader::Config TrackerConfig::makeFifoConfig() const {
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
    cfg.sensorHubSlave0SaturationAbs = data.magCal.driverEnabled ? 31900u : 0u;
    return cfg;
}

Ahrs6DofConfig TrackerConfig::makeAhrsConfig() const {
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

ImuQualityConfig TrackerConfig::makeQualityConfig() const {
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

void TrackerConfig::applyToImuCalibration(ImuCalibration& imuCal) const {
    imuCal.gyroBiasValid = data.gyroCal.biasValid;
    imuCal.gyroBiasRadS = data.gyroCal.biasRadS;

    imuCal.accelCalValid = data.accelCal.valid;
    imuCal.accelBiasG = data.accelCal.biasG;
    imuCal.accelScale = data.accelCal.scale;
}

void TrackerConfig::captureGyroFromImuCalibration(const ImuCalibration& imuCal) {
    data.gyroCal.biasValid = imuCal.gyroBiasValid;
    data.gyroCal.biasRadS = imuCal.gyroBiasRadS;
    updateCrc();
}

void TrackerConfig::captureAccelFromImuCalibration(const ImuCalibration& imuCal) {
    data.accelCal.valid = imuCal.accelCalValid;
    data.accelCal.biasG = imuCal.accelBiasG;
    data.accelCal.scale = imuCal.accelScale;
    updateCrc();
}

void TrackerConfig::captureFromImuCalibration(const ImuCalibration& imuCal) {
    captureGyroFromImuCalibration(imuCal);
    captureAccelFromImuCalibration(imuCal);
    updateCrc();
}

void TrackerConfig::clearGyroCalibrationPreservingPolicy() {
    const bool tempCompEnabled = data.gyroCal.tempCompEnabled;
    const bool tempLearningEnabled = data.gyroCal.reservedTempLearningEnabled;
    data.gyroCal = TrackerGyroCalibrationConfig{};
    data.gyroCal.tempCompEnabled = tempCompEnabled;
    data.gyroCal.reservedTempLearningEnabled = tempLearningEnabled;
    data.gyroTempQuality = TrackerGyroTempQualityConfigPersisted{};
    data.gyroCalMeta = TrackerGyroCalibrationMetaPersisted{};
    updateCrc();
}

void TrackerConfig::clearAccelCalibration() {
    data.accelCal = TrackerAccelCalibrationConfig{};
    data.accelCalQuality = TrackerAccelCalibrationQualityPersisted{};
    // Magnetic heading projection is not valid without calibrated gravity.
    data.magYaw.applyEnabled = false;
    updateCrc();
}

void TrackerConfig::clearMagCalibrationPreservingDriver() {
    const bool driverEnabled = data.magCal.driverEnabled;
    data.magCal = TrackerMagCalibrationConfig{};
    data.magCal.driverEnabled = driverEnabled;
    data.magCalQuality = TrackerMagCalibrationQualityPersisted{};
    // Yaw correction cannot remain active without a trusted field model.
    data.magYaw.applyEnabled = false;
    updateCrc();
}

void TrackerConfig::clearAllCalibrationPreservingPolicy() {
    clearGyroCalibrationPreservingPolicy();
    clearAccelCalibration();
    clearMagCalibrationPreservingDriver();
    data.frame.sensorToDevice = Mat3::identity();
    data.frame.sensorToDeviceValid = false;
    updateCrc();
}

void TrackerConfig::noteGyroBiasCalibrationCaptured(uint32_t uptimeMs) {
    data.gyroCalMeta.biasCalibrationUptimeMs = uptimeMs;
    data.gyroCalMeta.biasCalibrationVersion = tracker_config_detail::SCHEMA_GYRO_CAL_VERSION;
    updateCrc();
}

void TrackerConfig::captureFromMagCalibrationResult(const MagCalibrationResult& result,
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
    data.magCalQuality.coverageScore = result.valid ? result.coverageScore : 0.0f;
    data.magCalQuality.residualRms = result.valid ? result.residualRms : 0.0f;
    data.magCalQuality.reservedExpectedHorizontalNorm = 0.0f;
    updateCrc();
}

void TrackerConfig::applyToGyroTempComp(GyroTempCompensator& tempComp) const {
    GyroTempCompConfig cfg;
    cfg.enabled = data.gyroCal.tempCompEnabled;
    if (data.gyroCal.tempCompValid) {
        cfg.calibratedTempMinC = data.gyroTempQuality.tempRangeMinC;
        cfg.calibratedTempMaxC = data.gyroTempQuality.tempRangeMaxC;
        cfg.fitQuality = data.gyroTempQuality.fitQuality;
        cfg.fitResidualBeforeDps = data.gyroTempQuality.residualBeforeDps;
        cfg.fitResidualAfterDps = data.gyroTempQuality.residualAfterDps;
    }

    tempComp.setConfig(cfg);
    if (!data.gyroCal.biasValid) {
        tempComp.clearAll();
        tempComp.setConfig(cfg);
        return;
    }

    if (data.gyroCal.tempCompValid) {
        tempComp.setModel(data.gyroCal.biasRadS,
                          data.gyroCal.referenceTempC,
                          data.gyroCal.tempSlopeRadSPerC);
    } else {
        tempComp.setStaticBias(data.gyroCal.biasRadS, data.gyroCal.referenceTempC);
    }
}

namespace {

void captureGyroTempCompSnapshot(TrackerConfig& config,
                                 const GyroTempCompensator& tempComp) {
    if (!tempComp.valid()) {
        // Temperature runtime owns policy even before a bias/model exists.
        // Preserve the independent static bias captured from ImuCalibration,
        // while making a deliberately invalidated runtime model visible to
        // calibration-aware staging/saves.
        config.data.gyroCal.tempCompEnabled = tempComp.config().enabled;
        config.data.gyroCal.reservedTempLearningEnabled = false;
        config.data.gyroCal.tempCompValid = false;
        config.data.gyroCal.tempSlopeRadSPerC = Vec3::zero();
        config.data.gyroTempQuality = TrackerGyroTempQualityConfigPersisted{};
        config.data.gyroCalMeta.tempModelUpdatedUptimeMs = 0;
        config.data.gyroCalMeta.tempModelSampleCount = 0;
        return;
    }

    const TrackerGyroCalibrationConfig previousModel = config.data.gyroCal;
    const uint32_t previousUpdatedUptimeMs =
        config.data.gyroCalMeta.tempModelUpdatedUptimeMs;
    const uint32_t previousSampleCount =
        config.data.gyroCalMeta.tempModelSampleCount;

    config.data.gyroCal.biasValid = true;
    config.data.gyroCal.biasRadS = tempComp.referenceBiasRadS();
    config.data.gyroCal.tempCompValid = tempComp.temperatureModelValid();
    config.data.gyroCal.tempCompEnabled = tempComp.config().enabled;
    config.data.gyroCal.reservedTempLearningEnabled = false;
    config.data.gyroCal.referenceTempC = tempComp.referenceTempC();
    config.data.gyroCal.tempSlopeRadSPerC = config.data.gyroCal.tempCompValid
        ? tempComp.slopeRadSPerC()
        : Vec3::zero();

    if (config.data.gyroCal.tempCompValid) {
        const GyroTempCompConfig& tc = tempComp.config();
        config.data.gyroTempQuality.tempRangeMinC = tc.calibratedTempMinC;
        config.data.gyroTempQuality.tempRangeMaxC = tc.calibratedTempMaxC;
        config.data.gyroTempQuality.fitQuality = tc.fitQuality;
        config.data.gyroTempQuality.residualBeforeDps = tc.fitResidualBeforeDps;
        config.data.gyroTempQuality.residualAfterDps = tc.fitResidualAfterDps;
        const bool sameBias = previousModel.biasValid == config.data.gyroCal.biasValid &&
            previousModel.biasRadS.x == config.data.gyroCal.biasRadS.x &&
            previousModel.biasRadS.y == config.data.gyroCal.biasRadS.y &&
            previousModel.biasRadS.z == config.data.gyroCal.biasRadS.z;
        const bool sameTempModel = previousModel.tempCompValid == config.data.gyroCal.tempCompValid &&
            (!config.data.gyroCal.tempCompValid ||
             (previousModel.referenceTempC == config.data.gyroCal.referenceTempC &&
              previousModel.tempSlopeRadSPerC.x == config.data.gyroCal.tempSlopeRadSPerC.x &&
              previousModel.tempSlopeRadSPerC.y == config.data.gyroCal.tempSlopeRadSPerC.y &&
              previousModel.tempSlopeRadSPerC.z == config.data.gyroCal.tempSlopeRadSPerC.z));
        // A snapshot must not claim that an existing model was fitted again,
        // and must not attach old evidence to an externally changed model.
        const bool sameModel = sameBias && sameTempModel;
        config.data.gyroCalMeta.tempModelUpdatedUptimeMs =
            sameModel ? previousUpdatedUptimeMs : 0u;
        config.data.gyroCalMeta.tempModelSampleCount =
            sameModel ? previousSampleCount : 0u;
    } else {
        config.data.gyroTempQuality = TrackerGyroTempQualityConfigPersisted{};
        config.data.gyroCalMeta.tempModelUpdatedUptimeMs = 0;
        config.data.gyroCalMeta.tempModelSampleCount = 0;
    }
    config.data.gyroCalMeta.tempModelVersion =
        tracker_config_detail::SCHEMA_GYRO_CAL_VERSION;
}

} // namespace

void TrackerConfig::captureFromGyroTempComp(const GyroTempCompensator& tempComp) {
    captureGyroTempCompSnapshot(*this, tempComp);
    updateCrc();
}

void TrackerConfig::captureFromGyroTempCompUpdate(const GyroTempCompensator& tempComp,
                                                   uint32_t uptimeMs,
                                                   uint32_t sampleCount) {
    captureGyroTempCompSnapshot(*this, tempComp);
    data.gyroCalMeta.tempModelUpdatedUptimeMs =
        data.gyroCal.tempCompValid ? uptimeMs : 0u;
    data.gyroCalMeta.tempModelSampleCount =
        data.gyroCal.tempCompValid ? sampleCount : 0u;
    updateCrc();
}

void trackerMigratePerformanceDefaults(TrackerConfig& config) {
    // Preserve all current user/NVS settings. Only unmistakable historical
    // defaults are upgraded; the previous 4 MHz / 12-word production settings
    // stay intact until the user explicitly A/B tests and saves new values.
    if (config.data.hardware.spiHz == tracker_config_detail::LEGACY_SPI_HZ) {
        config.data.hardware.spiHz = tracker_config_detail::DEFAULT_SPI_HZ;
    }
    if (config.data.fifo.watermarkWords == cfg::LEGACY_FIFO_WATERMARK_WORDS) {
        config.data.fifo.watermarkWords = cfg::FIFO_WATERMARK_WORDS;
    }

#if TRACKER_BUILD_IS_SLIM
    if (config.data.fifo.watermarkWords == 0u ||
        config.data.fifo.watermarkWords > cfg::FIFO_WATERMARK_WORDS) {
        config.data.fifo.watermarkWords = cfg::FIFO_WATERMARK_WORDS;
    }
    if (config.data.output.outputRateHz == 0u ||
        config.data.output.outputRateHz > cfg::OUTPUT_RATE_HZ_MAX) {
        config.data.output.outputRateHz = cfg::OUTPUT_RATE_HZ;
    }
#endif

    config.sanitize();
    config.updateCrc();
}

} // namespace tracker
