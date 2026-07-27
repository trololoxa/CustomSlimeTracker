#pragma once

#include <cstddef>
#include <cstdint>

#include "config/tracker_config_runtime.hpp"

namespace tracker {

namespace tracker_config_storage_detail {

static constexpr uint32_t SLOT_MAGIC = 0x53474643UL;      // 'CFGS'
static constexpr uint16_t LEGACY_SLOT_VERSION = 1;
static constexpr uint16_t SLOT_VERSION = 2;
static constexpr uint32_t SELECTOR_MAGIC = 0x4C454343UL;  // 'CCEL'
static constexpr uint16_t SELECTOR_VERSION = 1;
static constexpr uint32_t CANDIDATE_MAGIC = 0x444E4143UL; // 'CAND'
static constexpr uint16_t GENERATION_CANDIDATE_VERSION = 1;
static constexpr uint16_t METADATA_REVISION_CANDIDATE_VERSION = 2;
static constexpr uint16_t CANDIDATE_VERSION = 3;
// Source compatibility for older tests/callers that intentionally construct
// the generation-bound v1 record.
static constexpr uint16_t LEGACY_CANDIDATE_VERSION = GENERATION_CANDIDATE_VERSION;
static constexpr uint32_t SIGNATURE_MAGIC = 0x47495343UL; // 'CSIG'
static constexpr uint16_t SIGNATURE_VERSION = 1;
static constexpr uint32_t COMMIT_MAGIC = 0x544D4343UL;    // 'CCMT'
static constexpr uint16_t COMMIT_VERSION = 1;

static constexpr uint16_t BOARD_TYPE_LOLIN_C3_MINI = 10;
static constexpr uint16_t IMU_TYPE_LSM6DSV = 13;
static constexpr uint16_t MCU_TYPE_ESP32_C3 = 6;
static constexpr uint16_t MAG_TYPE_QMC6309 = 1;

static constexpr uint32_t DEFAULT_CANDIDATE_MIN_WRITE_INTERVAL_MS = 5UL * 60UL * 1000UL;
static constexpr float DEFAULT_CANDIDATE_MIN_QUALITY_IMPROVEMENT = 0.01f;

} // namespace tracker_config_storage_detail

enum class TrackerConfigSlot : uint8_t {
    None = 0,
    A = 1,
    B = 2,
};

enum class TrackerCalibrationProvenance : uint8_t {
    Unknown = 0,
    Manual = 1,
    Setup = 2,
    Background = 3,
    ImportedLegacy = 4,
};

enum class TrackerCalibrationComparisonResult : uint8_t {
    NotCompared = 0,
    Better = 1,
    NotEnoughImprovement = 2,
    SignatureMismatch = 3,
    InvalidCandidate = 4,
    Promoted = 5,
    Rejected = 6,
    StaleActiveGeneration = 7,
    AlreadyPromoted = 8,
};

namespace tracker_calibration_comparison_flags {
static constexpr uint32_t NONE = 0;
static constexpr uint32_t ACTIVE_MISSING = 1u << 0;
static constexpr uint32_t SIGNATURE_MISMATCH = 1u << 1;
static constexpr uint32_t QUALITY_NONFINITE = 1u << 2;
static constexpr uint32_t BELOW_MIN_IMPROVEMENT = 1u << 3;
static constexpr uint32_t GYRO_REGRESSION = 1u << 4;
static constexpr uint32_t ACCEL_REGRESSION = 1u << 5;
static constexpr uint32_t MAG_REGRESSION = 1u << 6;
static constexpr uint32_t ALIGNMENT_REGRESSION = 1u << 7;
static constexpr uint32_t COVERAGE_REGRESSION = 1u << 8;
static constexpr uint32_t STALE_ACTIVE_GENERATION = 1u << 9;
static constexpr uint32_t ALREADY_PROMOTED = 1u << 10;
}

namespace tracker_calibration_quality_flags {
static constexpr uint32_t PROVENANCE_SHIFT = 24u;
static constexpr uint32_t PROVENANCE_MASK = 0x7u << PROVENANCE_SHIFT;
static constexpr uint32_t AUTONOMY_0022 = 1u << 22;
static constexpr uint32_t AUTONOMY_0023 = 1u << 27;
static constexpr uint32_t GYRO_MEASURED = 1u << 28;
static constexpr uint32_t ALIGNMENT_MEASURED = 1u << 29;
// ACCEL_MEASURED shares the low provenance-free bit used only by measured
// background candidates. It remains outside the persisted provenance mask.
static constexpr uint32_t ACCEL_MEASURED = 1u << 23;
static constexpr uint32_t SOURCE_DERIVED = 1u << 30;
static constexpr uint32_t SOURCE_MEASURED = 1u << 31;
}

struct TrackerSensorSignature {
    uint32_t magic = tracker_config_storage_detail::SIGNATURE_MAGIC;
    uint16_t version = tracker_config_storage_detail::SIGNATURE_VERSION;
    uint16_t size = sizeof(TrackerSensorSignature);

    uint16_t boardType = tracker_config_storage_detail::BOARD_TYPE_LOLIN_C3_MINI;
    uint16_t imuType = tracker_config_storage_detail::IMU_TYPE_LSM6DSV;
    uint16_t mcuType = tracker_config_storage_detail::MCU_TYPE_ESP32_C3;
    uint16_t magType = tracker_config_storage_detail::MAG_TYPE_QMC6309;

    uint8_t imuOdr = 0;
    uint8_t accelFs = 0;
    uint8_t gyroFs = 0;
    uint8_t accelMode = 0;
    uint8_t gyroMode = 0;
    uint8_t accelBdr = 0;
    uint8_t gyroBdr = 0;
    uint8_t sensorToDeviceValid = 0;

    uint32_t sensorToDeviceHash = 0;
    uint32_t calibrationSchemaHash = 0;
    uint32_t crc32 = 0;
};

struct TrackerCalibrationQualitySummary {
    float overallScore = 0.0f;
    float gyroScore = 0.0f;
    float accelScore = 0.0f;
    float magScore = 0.0f;
    float alignmentScore = 0.0f;
    float coverageScore = 0.0f;

    float gyroResidualDps = 0.0f;
    float accelResidualG = 0.0f;
    float magResidual = 0.0f;
    uint32_t qualityFlags = 0;
};

struct TrackerConfigSlotRecord {
    uint32_t magic = tracker_config_storage_detail::SLOT_MAGIC;
    uint16_t version = tracker_config_storage_detail::SLOT_VERSION;
    uint16_t size = sizeof(TrackerConfigSlotRecord);
    uint32_t generation = 0;
    TrackerSensorSignature signature;
    TrackerCalibrationQualitySummary quality;
    TrackerConfigBlob payload;
    uint32_t crc32 = 0;
};

struct TrackerConfigCommitRecord {
    uint32_t magic = tracker_config_storage_detail::COMMIT_MAGIC;
    uint16_t version = tracker_config_storage_detail::COMMIT_VERSION;
    uint16_t size = sizeof(TrackerConfigCommitRecord);
    TrackerConfigSlot slot = TrackerConfigSlot::None;
    uint8_t reserved0[3] = {};
    uint32_t generation = 0;
    uint32_t crc32 = 0;
};

struct TrackerConfigSelectorRecord {
    uint32_t magic = tracker_config_storage_detail::SELECTOR_MAGIC;
    uint16_t version = tracker_config_storage_detail::SELECTOR_VERSION;
    uint16_t size = sizeof(TrackerConfigSelectorRecord);
    TrackerConfigSlot activeSlot = TrackerConfigSlot::None;
    uint8_t reserved0[3] = {};
    uint32_t activeGeneration = 0;
    uint32_t successfulActiveWrites = 0;
    uint32_t successfulMigrations = 0;
    uint32_t successfulPromotions = 0;
    uint32_t crc32 = 0;
};

struct TrackerCalibrationCandidateMetadata {
    TrackerCalibrationProvenance provenance = TrackerCalibrationProvenance::Unknown;
    TrackerCalibrationComparisonResult lastComparison = TrackerCalibrationComparisonResult::NotCompared;
    uint16_t reserved0 = 0;
    uint32_t comparisonFlags = tracker_calibration_comparison_flags::NONE;
    uint32_t createdUptimeMs = 0;
    uint32_t activeCalibrationRevisionAtCreation = 0;
    uint32_t sampleCount = 0;
    uint32_t independentWindowCount = 0;
    TrackerCalibrationQualitySummary quality;
};

struct TrackerCalibrationCandidateRecord {
    uint32_t magic = tracker_config_storage_detail::CANDIDATE_MAGIC;
    uint16_t version = tracker_config_storage_detail::CANDIDATE_VERSION;
    uint16_t size = sizeof(TrackerCalibrationCandidateRecord);
    uint32_t candidateGeneration = 0;
    uint32_t persistedWriteCount = 0;
    TrackerSensorSignature signature;
    TrackerCalibrationCandidateMetadata metadata;
    TrackerConfigBlob payload;
    uint32_t crc32 = 0;
};

struct TrackerConfigSlotInfo {
    bool exists = false;
    bool readable = false;
    bool valid = false;
    bool legacyCommitted = false;
    bool commitMarkerExists = false;
    bool commitMarkerValid = false;
    uint32_t generation = 0;
    TrackerSensorSignature signature;
    TrackerCalibrationQualitySummary quality;
};

struct TrackerCalibrationCandidateInfo {
    bool exists = false;
    bool readable = false;
    bool valid = false;
    bool dirtyInRam = false;
    uint32_t generation = 0;
    uint32_t persistedWriteCount = 0;
    TrackerCalibrationCandidateMetadata metadata;
};

struct TrackerConfigStorageInfo {
    TrackerConfigSlotInfo slotA;
    TrackerConfigSlotInfo slotB;
    bool selectorExists = false;
    bool selectorValid = false;
    TrackerConfigSlot selectedSlot = TrackerConfigSlot::None;
    uint32_t selectedGeneration = 0;
    bool selectedByFallback = false;
    bool legacyExists = false;
    bool legacyValid = false;
    TrackerCalibrationCandidateInfo candidate;
    uint32_t successfulActiveWrites = 0;
    uint32_t successfulMigrations = 0;
    uint32_t successfulPromotions = 0;
    uint32_t loadFallbacks = 0;
    uint32_t selectorRepairFailures = 0;
    uint32_t candidateStageCount = 0;
    uint32_t candidateFlushCount = 0;
    uint32_t candidateFlushThrottled = 0;
    uint32_t candidateRejectedCount = 0;
    bool legacyCleanupPending = false;
    uint32_t legacyCleanupFailures = 0;
    uint32_t commitUncertainCount = 0;
    uint32_t commitMarkerRepairFailures = 0;
    uint32_t noOpSaveCount = 0;
    uint32_t degradedWriteBlocks = 0;
    bool storageDegradedLatched = false;
    bool authoritativeApplyPending = false;
    uint32_t applyPendingWriteBlocks = 0;
    uint32_t candidatePromotionStateWrites = 0;
    uint32_t candidatePromotionStateWriteFailures = 0;
};

struct TrackerCalibrationWearPolicy {
    uint32_t minCandidateWriteIntervalMs =
        tracker_config_storage_detail::DEFAULT_CANDIDATE_MIN_WRITE_INTERVAL_MS;
    float minQualityImprovement =
        tracker_config_storage_detail::DEFAULT_CANDIDATE_MIN_QUALITY_IMPROVEMENT;
};

struct TrackerPreparedConfigPromotion {
    bool valid = false;
    TrackerConfigSlot targetSlot = TrackerConfigSlot::None;
    uint32_t targetGeneration = 0;
    uint32_t activeGenerationAtPreparation = 0;
    TrackerConfigSelectorRecord previousSelector;
};

TrackerSensorSignature trackerMakeSensorSignature(const TrackerConfig& config);
bool trackerValidateSensorSignature(const TrackerSensorSignature& signature);
bool trackerSensorSignaturesEqual(const TrackerSensorSignature& a,
                                  const TrackerSensorSignature& b);

TrackerCalibrationQualitySummary trackerCalibrationQualityFromConfig(const TrackerConfig& config);
void trackerCalibrationQualityRecomputeOverall(const TrackerConfigBlob& payload,
                                               TrackerCalibrationQualitySummary& quality);
void trackerCalibrationQualityRecomputeOverall(const TrackerConfig& config,
                                               TrackerCalibrationQualitySummary& quality);
void trackerApplyCalibrationCandidateToConfig(TrackerConfig& active,
                                                const TrackerConfig& candidate);
TrackerConfig trackerComposeCalibrationCandidate(const TrackerConfig& active,
                                                 const TrackerConfig& candidate);
bool trackerCalibrationModelEqual(const TrackerConfig& a, const TrackerConfig& b);
bool trackerCalibrationEvidenceEqual(const TrackerConfig& a, const TrackerConfig& b);
// Compatibility alias: calibration payload means model + evidence, not product policy.
bool trackerCalibrationPayloadEqual(const TrackerConfig& a, const TrackerConfig& b);
uint32_t trackerCalibrationPayloadRevision(const TrackerConfig& config);
uint32_t trackerCalibrationPayloadRevision(const TrackerConfigBlob& payload);
uint32_t trackerCalibrationPayloadRevisionLegacyV2(const TrackerConfig& config);
uint32_t trackerCalibrationPayloadRevisionLegacyV2(const TrackerConfigBlob& payload);
bool trackerValidateCalibrationQuality(const TrackerCalibrationQualitySummary& quality);
void trackerCalibrationQualitySetProvenance(TrackerCalibrationQualitySummary& quality,
                                            TrackerCalibrationProvenance provenance);
TrackerCalibrationProvenance trackerCalibrationQualityProvenance(
    const TrackerCalibrationQualitySummary& quality);

uint32_t trackerConfigSlotRecordCrc(const TrackerConfigSlotRecord& record);
bool trackerValidateConfigSlotRecord(const TrackerConfigSlotRecord& record);
uint32_t trackerConfigCommitRecordCrc(const TrackerConfigCommitRecord& record);
bool trackerValidateConfigCommitRecord(const TrackerConfigCommitRecord& record);
uint32_t trackerConfigSelectorRecordCrc(const TrackerConfigSelectorRecord& record);
bool trackerValidateConfigSelectorRecord(const TrackerConfigSelectorRecord& record);
uint32_t trackerCalibrationCandidateRecordCrc(const TrackerCalibrationCandidateRecord& record);
bool trackerValidateCalibrationCandidateRecord(const TrackerCalibrationCandidateRecord& record);

bool trackerGenerationIsNewer(uint32_t a, uint32_t b);
const char* trackerConfigSlotName(TrackerConfigSlot slot);
const char* trackerCalibrationProvenanceName(TrackerCalibrationProvenance provenance);
const char* trackerCalibrationComparisonName(TrackerCalibrationComparisonResult result);

} // namespace tracker
