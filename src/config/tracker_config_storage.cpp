#include "config/tracker_config_storage.hpp"

#include <cmath>
#include <cstring>

namespace tracker {

namespace {

float clamp01(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

uint32_t crcBytesWithZeroedTail(const void* value, size_t size, size_t crcOffset) {
    if (!value || crcOffset + sizeof(uint32_t) > size) return 0;
    const auto* bytes = static_cast<const uint8_t*>(value);
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < size; ++i) {
        const uint8_t byte = (i >= crcOffset && i < crcOffset + sizeof(uint32_t)) ? 0u : bytes[i];
        hash ^= byte;
        hash *= 16777619u;
    }
    return hash;
}

uint32_t signatureCrc(const TrackerSensorSignature& signature) {
    return crcBytesWithZeroedTail(
        &signature,
        sizeof(signature),
        offsetof(TrackerSensorSignature, crc32)
    );
}

uint32_t schemaHash(const TrackerConfigSchemaVersions& schemas) {
    return tracker_config_detail::fnv1a32(
        reinterpret_cast<const uint8_t*>(&schemas),
        sizeof(schemas)
    );
}

bool finiteScore(float value) {
    return std::isfinite(value) && value >= 0.0f && value <= 1.0f;
}

uint32_t fnvAppend(uint32_t hash, const void* data, size_t len) {
    const auto* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < len; ++i) {
        hash ^= static_cast<uint32_t>(bytes[i]);
        hash *= 16777619u;
    }
    return hash;
}

uint32_t calibrationRevisionLegacyV2(const TrackerConfigBlob& data) {
    uint32_t hash = 2166136261u;
    hash = fnvAppend(hash, &data.gyroCal, sizeof(data.gyroCal));
    hash = fnvAppend(hash, &data.gyroTempQuality, sizeof(data.gyroTempQuality));
    hash = fnvAppend(hash, &data.gyroCalMeta, sizeof(data.gyroCalMeta));
    hash = fnvAppend(hash, &data.accelCal, sizeof(data.accelCal));
    hash = fnvAppend(hash, &data.accelCalQuality, sizeof(data.accelCalQuality));
    hash = fnvAppend(hash, &data.magCal.calibrationValid, sizeof(data.magCal.calibrationValid));
    hash = fnvAppend(hash, &data.magCal.axisAlignmentValid, sizeof(data.magCal.axisAlignmentValid));
    hash = fnvAppend(hash, &data.magCal.hardIron, sizeof(data.magCal.hardIron));
    hash = fnvAppend(hash, &data.magCal.softIron, sizeof(data.magCal.softIron));
    hash = fnvAppend(hash, &data.magCal.magToImu, sizeof(data.magCal.magToImu));
    hash = fnvAppend(hash, &data.magCal.expectedFieldNorm, sizeof(data.magCal.expectedFieldNorm));
    hash = fnvAppend(hash, &data.magCalQuality, sizeof(data.magCalQuality));
    return hash == 0u ? 1u : hash;
}

uint32_t canonicalFloatBits(float value) {
    if (value == 0.0f) return 0u; // +0 and -0 are the same calibration value.
    uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "float must be 32-bit");
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

uint32_t appendFloat(uint32_t hash, float value) {
    const uint32_t bits = canonicalFloatBits(value);
    return fnvAppend(hash, &bits, sizeof(bits));
}

uint32_t appendVec3(uint32_t hash, const Vec3& value) {
    hash = appendFloat(hash, value.x);
    hash = appendFloat(hash, value.y);
    return appendFloat(hash, value.z);
}

uint32_t appendMat3(uint32_t hash, const Mat3& value) {
    for (uint8_t row = 0; row < 3; ++row) {
        for (uint8_t col = 0; col < 3; ++col) {
            hash = appendFloat(hash, value.m[row][col]);
        }
    }
    return hash;
}

bool vecEqual(const Vec3& a, const Vec3& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool matEqual(const Mat3& a, const Mat3& b) {
    for (uint8_t row = 0; row < 3; ++row) {
        for (uint8_t col = 0; col < 3; ++col) {
            if (a.m[row][col] != b.m[row][col]) return false;
        }
    }
    return true;
}

uint32_t calibrationModelRevision(const TrackerConfigBlob& data) {
    uint32_t hash = 2166136261u;

    hash = fnvAppend(hash, &data.gyroCal.biasValid, sizeof(data.gyroCal.biasValid));
    if (data.gyroCal.biasValid) {
        hash = appendVec3(hash, data.gyroCal.biasRadS);
    }
    hash = fnvAppend(hash, &data.gyroCal.tempCompValid, sizeof(data.gyroCal.tempCompValid));
    if (data.gyroCal.tempCompValid) {
        hash = appendFloat(hash, data.gyroCal.referenceTempC);
        hash = appendVec3(hash, data.gyroCal.tempSlopeRadSPerC);
    }

    hash = fnvAppend(hash, &data.accelCal.valid, sizeof(data.accelCal.valid));
    if (data.accelCal.valid) {
        hash = appendVec3(hash, data.accelCal.biasG);
        hash = appendMat3(hash, data.accelCal.scale);
    }

    hash = fnvAppend(hash, &data.magCal.calibrationValid, sizeof(data.magCal.calibrationValid));
    if (data.magCal.calibrationValid) {
        hash = appendVec3(hash, data.magCal.hardIron);
        hash = appendMat3(hash, data.magCal.softIron);
        hash = appendFloat(hash, data.magCal.expectedFieldNorm);
        hash = appendFloat(hash, data.magCal.minTrustNorm);
        hash = appendFloat(hash, data.magCal.maxTrustNorm);
    }
    hash = fnvAppend(hash, &data.magCal.axisAlignmentValid, sizeof(data.magCal.axisAlignmentValid));
    if (data.magCal.axisAlignmentValid) {
        hash = appendMat3(hash, data.magCal.magToImu);
    }
    return hash == 0u ? 1u : hash;
}


} // namespace

TrackerSensorSignature trackerMakeSensorSignature(const TrackerConfig& config) {
    TrackerSensorSignature signature{};
    signature.imuOdr = static_cast<uint8_t>(config.data.imu.imuOdr);
    signature.accelFs = static_cast<uint8_t>(config.data.imu.accelFs);
    signature.gyroFs = static_cast<uint8_t>(config.data.imu.gyroFs);
    signature.accelMode = static_cast<uint8_t>(config.data.imu.accelMode);
    signature.gyroMode = static_cast<uint8_t>(config.data.imu.gyroMode);
    signature.accelBdr = static_cast<uint8_t>(config.data.fifo.accelBdr);
    signature.gyroBdr = static_cast<uint8_t>(config.data.fifo.gyroBdr);
    signature.sensorToDeviceValid = config.data.frame.sensorToDeviceValid ? 1u : 0u;
    signature.sensorToDeviceHash = signature.sensorToDeviceValid
        ? tracker_config_detail::fnv1a32(
            reinterpret_cast<const uint8_t*>(&config.data.frame.sensorToDevice),
            sizeof(config.data.frame.sensorToDevice))
        : 0u;
    signature.calibrationSchemaHash = schemaHash(config.data.schemas);
    signature.crc32 = 0;
    signature.crc32 = signatureCrc(signature);
    return signature;
}

bool trackerValidateSensorSignature(const TrackerSensorSignature& signature) {
    if (signature.magic != tracker_config_storage_detail::SIGNATURE_MAGIC) return false;
    if (signature.version != tracker_config_storage_detail::SIGNATURE_VERSION) return false;
    if (signature.size != sizeof(TrackerSensorSignature)) return false;
    if (signature.boardType != tracker_config_storage_detail::BOARD_TYPE_LOLIN_C3_MINI) return false;
    if (signature.imuType != tracker_config_storage_detail::IMU_TYPE_LSM6DSV) return false;
    if (signature.mcuType != tracker_config_storage_detail::MCU_TYPE_ESP32_C3) return false;
    if (signature.magType != tracker_config_storage_detail::MAG_TYPE_QMC6309) return false;
    return signature.crc32 == signatureCrc(signature);
}

bool trackerSensorSignaturesEqual(const TrackerSensorSignature& a,
                                  const TrackerSensorSignature& b) {
    if (!trackerValidateSensorSignature(a) || !trackerValidateSensorSignature(b)) return false;
    TrackerSensorSignature aa = a;
    TrackerSensorSignature bb = b;
    aa.crc32 = 0;
    bb.crc32 = 0;
    // A disabled frame is an identity runtime transform. Older records may
    // still contain a stale matrix hash, so dormant bytes are not part of the
    // logical sensor contract until the validity flag is enabled.
    if (aa.sensorToDeviceValid == 0u && bb.sensorToDeviceValid == 0u) {
        aa.sensorToDeviceHash = 0u;
        bb.sensorToDeviceHash = 0u;
    }
    return std::memcmp(&aa, &bb, sizeof(aa)) == 0;
}

TrackerCalibrationQualitySummary trackerCalibrationQualityFromConfig(const TrackerConfig& config) {
    TrackerCalibrationQualitySummary quality{};

    quality.gyroScore = config.data.gyroCal.biasValid ? 0.70f : 0.0f;
    if (config.data.gyroCal.tempCompValid) {
        quality.gyroScore = clamp01(quality.gyroScore + 0.30f * clamp01(config.data.gyroTempQuality.fitQuality));
        quality.gyroResidualDps = std::max(0.0f, config.data.gyroTempQuality.residualAfterDps);
    }

    quality.accelScore = config.data.accelCal.valid
        ? clamp01(config.data.accelCalQuality.qualityScore)
        : 0.0f;
    quality.accelResidualG = std::max(
        std::max(0.0f, config.data.accelCalQuality.maxFaceNormErrorG),
        std::max(0.0f, config.data.accelCalQuality.maxAxisResidualG)
    );

    quality.coverageScore = config.data.magCal.calibrationValid
        ? clamp01(config.data.magCalQuality.coverageScore)
        : 0.0f;
    quality.magResidual = std::max(0.0f, config.data.magCalQuality.residualRms);
    quality.magScore = config.data.magCal.calibrationValid
        ? clamp01(0.75f * quality.coverageScore + 0.25f / (1.0f + quality.magResidual))
        : 0.0f;
    quality.alignmentScore = config.data.magCal.axisAlignmentValid ? 1.0f : 0.0f;

    trackerCalibrationQualityRecomputeOverall(config, quality);
    quality.qualityFlags |= tracker_calibration_quality_flags::SOURCE_DERIVED;
    return quality;
}

void trackerCalibrationQualityRecomputeOverall(const TrackerConfigBlob& payload,
                                               TrackerCalibrationQualitySummary& quality) {
    float weighted = 0.0f;
    float weight = 0.0f;
    if (payload.gyroCal.biasValid) {
        weighted += clamp01(quality.gyroScore) * 0.35f;
        weight += 0.35f;
    }
    if (payload.accelCal.valid) {
        weighted += clamp01(quality.accelScore) * 0.35f;
        weight += 0.35f;
    }
    if (payload.magCal.calibrationValid) {
        weighted += clamp01(quality.magScore) * 0.20f;
        weight += 0.20f;
    }
    if (payload.magCal.axisAlignmentValid) {
        weighted += clamp01(quality.alignmentScore) * 0.10f;
        weight += 0.10f;
    }
    quality.overallScore = weight > 0.0f ? clamp01(weighted / weight) : 0.0f;
}

void trackerCalibrationQualityRecomputeOverall(const TrackerConfig& config,
                                               TrackerCalibrationQualitySummary& quality) {
    trackerCalibrationQualityRecomputeOverall(config.data, quality);
}

void trackerApplyCalibrationCandidateToConfig(TrackerConfig& active,
                                                const TrackerConfig& candidate) {
    const bool tempCompEnabled = active.data.gyroCal.tempCompEnabled;
    const bool tempLearningEnabled = active.data.gyroCal.reservedTempLearningEnabled;

    active.data.gyroCal = candidate.data.gyroCal;
    // These are runtime/product policy, not learned calibration model state.
    active.data.gyroCal.tempCompEnabled = tempCompEnabled;
    active.data.gyroCal.reservedTempLearningEnabled = tempLearningEnabled;
    active.data.gyroTempQuality = candidate.data.gyroTempQuality;
    active.data.gyroCalMeta = candidate.data.gyroCalMeta;
    active.data.accelCal = candidate.data.accelCal;
    active.data.accelCalQuality = candidate.data.accelCalQuality;

    // Calibration owns the measured field model, derived trust bounds and
    // mag-to-IMU alignment. Driver/runtime enable policy stays active-owned.
    active.data.magCal.calibrationValid = candidate.data.magCal.calibrationValid;
    active.data.magCal.axisAlignmentValid = candidate.data.magCal.axisAlignmentValid;
    active.data.magCal.hardIron = candidate.data.magCal.hardIron;
    active.data.magCal.softIron = candidate.data.magCal.softIron;
    active.data.magCal.magToImu = candidate.data.magCal.magToImu;
    active.data.magCal.expectedFieldNorm = candidate.data.magCal.expectedFieldNorm;
    active.data.magCal.minTrustNorm = candidate.data.magCal.minTrustNorm;
    active.data.magCal.maxTrustNorm = candidate.data.magCal.maxTrustNorm;
    active.data.magCalQuality = candidate.data.magCalQuality;
    if (!active.data.accelCal.valid || !active.data.magCal.calibrationValid ||
        !active.data.magCal.axisAlignmentValid) {
        active.data.magYaw.applyEnabled = false;
    }
    active.sanitize();
    active.updateCrc();
}

TrackerConfig trackerComposeCalibrationCandidate(const TrackerConfig& active,
                                                 const TrackerConfig& candidate) {
    TrackerConfig composed = active;
    trackerApplyCalibrationCandidateToConfig(composed, candidate);
    return composed;
}

bool trackerCalibrationModelEqual(const TrackerConfig& a, const TrackerConfig& b) {
    if (a.data.gyroCal.biasValid != b.data.gyroCal.biasValid) return false;
    if (a.data.gyroCal.biasValid &&
        !vecEqual(a.data.gyroCal.biasRadS, b.data.gyroCal.biasRadS)) return false;
    if (a.data.gyroCal.tempCompValid != b.data.gyroCal.tempCompValid) return false;
    if (a.data.gyroCal.tempCompValid &&
        (a.data.gyroCal.referenceTempC != b.data.gyroCal.referenceTempC ||
         !vecEqual(a.data.gyroCal.tempSlopeRadSPerC, b.data.gyroCal.tempSlopeRadSPerC))) return false;

    if (a.data.accelCal.valid != b.data.accelCal.valid) return false;
    if (a.data.accelCal.valid &&
        (!vecEqual(a.data.accelCal.biasG, b.data.accelCal.biasG) ||
         !matEqual(a.data.accelCal.scale, b.data.accelCal.scale))) return false;

    if (a.data.magCal.calibrationValid != b.data.magCal.calibrationValid) return false;
    if (a.data.magCal.calibrationValid &&
        (!vecEqual(a.data.magCal.hardIron, b.data.magCal.hardIron) ||
         !matEqual(a.data.magCal.softIron, b.data.magCal.softIron) ||
         a.data.magCal.expectedFieldNorm != b.data.magCal.expectedFieldNorm ||
         a.data.magCal.minTrustNorm != b.data.magCal.minTrustNorm ||
         a.data.magCal.maxTrustNorm != b.data.magCal.maxTrustNorm)) return false;
    if (a.data.magCal.axisAlignmentValid != b.data.magCal.axisAlignmentValid) return false;
    if (a.data.magCal.axisAlignmentValid &&
        !matEqual(a.data.magCal.magToImu, b.data.magCal.magToImu)) return false;
    return true;
}

bool trackerCalibrationEvidenceEqual(const TrackerConfig& a, const TrackerConfig& b) {
    return std::memcmp(&a.data.gyroTempQuality, &b.data.gyroTempQuality,
                       sizeof(a.data.gyroTempQuality)) == 0 &&
           std::memcmp(&a.data.gyroCalMeta, &b.data.gyroCalMeta,
                       sizeof(a.data.gyroCalMeta)) == 0 &&
           std::memcmp(&a.data.accelCalQuality, &b.data.accelCalQuality,
                       sizeof(a.data.accelCalQuality)) == 0 &&
           std::memcmp(&a.data.magCalQuality, &b.data.magCalQuality,
                       sizeof(a.data.magCalQuality)) == 0;
}

bool trackerCalibrationPayloadEqual(const TrackerConfig& a, const TrackerConfig& b) {
    return trackerCalibrationModelEqual(a, b) && trackerCalibrationEvidenceEqual(a, b);
}

uint32_t trackerCalibrationPayloadRevision(const TrackerConfigBlob& payload) {
    return calibrationModelRevision(payload);
}

uint32_t trackerCalibrationPayloadRevision(const TrackerConfig& config) {
    return calibrationModelRevision(config.data);
}

uint32_t trackerCalibrationPayloadRevisionLegacyV2(const TrackerConfigBlob& payload) {
    return calibrationRevisionLegacyV2(payload);
}

uint32_t trackerCalibrationPayloadRevisionLegacyV2(const TrackerConfig& config) {
    return calibrationRevisionLegacyV2(config.data);
}

void trackerCalibrationQualitySetProvenance(TrackerCalibrationQualitySummary& quality,
                                            TrackerCalibrationProvenance provenance) {
    const uint32_t encoded = static_cast<uint32_t>(provenance) & 0x7u;
    quality.qualityFlags = (quality.qualityFlags &
                            ~tracker_calibration_quality_flags::PROVENANCE_MASK) |
                           (encoded << tracker_calibration_quality_flags::PROVENANCE_SHIFT);
}

TrackerCalibrationProvenance trackerCalibrationQualityProvenance(
    const TrackerCalibrationQualitySummary& quality) {
    const uint32_t encoded = (quality.qualityFlags &
                              tracker_calibration_quality_flags::PROVENANCE_MASK) >>
                             tracker_calibration_quality_flags::PROVENANCE_SHIFT;
    if (encoded > static_cast<uint32_t>(TrackerCalibrationProvenance::ImportedLegacy)) {
        return TrackerCalibrationProvenance::Unknown;
    }
    return static_cast<TrackerCalibrationProvenance>(encoded);
}

bool trackerValidateCalibrationQuality(const TrackerCalibrationQualitySummary& quality) {
    if (!finiteScore(quality.overallScore) ||
        !finiteScore(quality.gyroScore) ||
        !finiteScore(quality.accelScore) ||
        !finiteScore(quality.magScore) ||
        !finiteScore(quality.alignmentScore) ||
        !finiteScore(quality.coverageScore)) {
        return false;
    }
    return std::isfinite(quality.gyroResidualDps) && quality.gyroResidualDps >= 0.0f &&
           std::isfinite(quality.accelResidualG) && quality.accelResidualG >= 0.0f &&
           std::isfinite(quality.magResidual) && quality.magResidual >= 0.0f;
}

uint32_t trackerConfigSlotRecordCrc(const TrackerConfigSlotRecord& record) {
    return crcBytesWithZeroedTail(
        &record,
        sizeof(record),
        offsetof(TrackerConfigSlotRecord, crc32)
    );
}

bool trackerValidateConfigSlotRecord(const TrackerConfigSlotRecord& record) {
    if (record.magic != tracker_config_storage_detail::SLOT_MAGIC ||
        (record.version != tracker_config_storage_detail::LEGACY_SLOT_VERSION &&
         record.version != tracker_config_storage_detail::SLOT_VERSION) ||
        record.size != sizeof(TrackerConfigSlotRecord) ||
        record.generation == 0 ||
        record.crc32 != trackerConfigSlotRecordCrc(record) ||
        !trackerValidateSensorSignature(record.signature) ||
        !trackerValidateCalibrationQuality(record.quality)) {
        return false;
    }
    TrackerConfig config;
    config.data = record.payload;
    if (!config.validate()) return false;
    return trackerSensorSignaturesEqual(record.signature, trackerMakeSensorSignature(config));
}

uint32_t trackerConfigCommitRecordCrc(const TrackerConfigCommitRecord& record) {
    return crcBytesWithZeroedTail(
        &record,
        sizeof(record),
        offsetof(TrackerConfigCommitRecord, crc32)
    );
}

bool trackerValidateConfigCommitRecord(const TrackerConfigCommitRecord& record) {
    if (record.magic != tracker_config_storage_detail::COMMIT_MAGIC ||
        record.version != tracker_config_storage_detail::COMMIT_VERSION ||
        record.size != sizeof(TrackerConfigCommitRecord)) {
        return false;
    }
    if (record.slot != TrackerConfigSlot::A && record.slot != TrackerConfigSlot::B) return false;
    if (record.generation == 0u) return false;
    return record.crc32 == trackerConfigCommitRecordCrc(record);
}

uint32_t trackerConfigSelectorRecordCrc(const TrackerConfigSelectorRecord& record) {
    return crcBytesWithZeroedTail(
        &record,
        sizeof(record),
        offsetof(TrackerConfigSelectorRecord, crc32)
    );
}

bool trackerValidateConfigSelectorRecord(const TrackerConfigSelectorRecord& record) {
    if (record.magic != tracker_config_storage_detail::SELECTOR_MAGIC ||
        record.version != tracker_config_storage_detail::SELECTOR_VERSION ||
        record.size != sizeof(TrackerConfigSelectorRecord)) {
        return false;
    }
    if (record.activeSlot != TrackerConfigSlot::A && record.activeSlot != TrackerConfigSlot::B) return false;
    if (record.activeGeneration == 0) return false;
    return record.crc32 == trackerConfigSelectorRecordCrc(record);
}

uint32_t trackerCalibrationCandidateRecordCrc(const TrackerCalibrationCandidateRecord& record) {
    return crcBytesWithZeroedTail(
        &record,
        sizeof(record),
        offsetof(TrackerCalibrationCandidateRecord, crc32)
    );
}

bool trackerValidateCalibrationCandidateRecord(const TrackerCalibrationCandidateRecord& record) {
    if (record.magic != tracker_config_storage_detail::CANDIDATE_MAGIC ||
        (record.version != tracker_config_storage_detail::GENERATION_CANDIDATE_VERSION &&
         record.version != tracker_config_storage_detail::METADATA_REVISION_CANDIDATE_VERSION &&
         record.version != tracker_config_storage_detail::CANDIDATE_VERSION) ||
        record.size != sizeof(TrackerCalibrationCandidateRecord) ||
        record.candidateGeneration == 0 ||
        record.crc32 != trackerCalibrationCandidateRecordCrc(record) ||
        !trackerValidateSensorSignature(record.signature) ||
        !trackerValidateCalibrationQuality(record.metadata.quality)) {
        return false;
    }
    TrackerConfig config;
    config.data = record.payload;
    if (!config.validate()) return false;
    return trackerSensorSignaturesEqual(record.signature, trackerMakeSensorSignature(config));
}

bool trackerGenerationIsNewer(uint32_t a, uint32_t b) {
    return a != b && static_cast<int32_t>(a - b) > 0;
}

const char* trackerConfigSlotName(TrackerConfigSlot slot) {
    switch (slot) {
        case TrackerConfigSlot::A: return "A";
        case TrackerConfigSlot::B: return "B";
        case TrackerConfigSlot::None: break;
    }
    return "none";
}

const char* trackerCalibrationProvenanceName(TrackerCalibrationProvenance provenance) {
    switch (provenance) {
        case TrackerCalibrationProvenance::Manual: return "manual";
        case TrackerCalibrationProvenance::Setup: return "setup";
        case TrackerCalibrationProvenance::Background: return "background";
        case TrackerCalibrationProvenance::ImportedLegacy: return "imported_legacy";
        case TrackerCalibrationProvenance::Unknown: break;
    }
    return "unknown";
}

const char* trackerCalibrationComparisonName(TrackerCalibrationComparisonResult result) {
    switch (result) {
        case TrackerCalibrationComparisonResult::Better: return "better";
        case TrackerCalibrationComparisonResult::NotEnoughImprovement: return "not_enough_improvement";
        case TrackerCalibrationComparisonResult::SignatureMismatch: return "signature_mismatch";
        case TrackerCalibrationComparisonResult::InvalidCandidate: return "invalid_candidate";
        case TrackerCalibrationComparisonResult::Promoted: return "promoted";
        case TrackerCalibrationComparisonResult::Rejected: return "rejected";
        case TrackerCalibrationComparisonResult::StaleActiveGeneration: return "stale_active_generation";
        case TrackerCalibrationComparisonResult::AlreadyPromoted: return "already_promoted";
        case TrackerCalibrationComparisonResult::NotCompared: break;
    }
    return "not_compared";
}

} // namespace tracker
