#pragma once

#include <cstddef>
#include <cstdint>

#include "build_config/feature_flags.hpp"
#include "config/tracker_config_storage.hpp"

namespace tracker {

enum class TrackerConfigError : uint8_t {
    None,
    NvsBeginFailed,
    NotFound,
    SizeMismatch,
    ReadFailed,
    WriteFailed,
    RemoveFailed,
    CrcOrValidationFailed,
    SelectorInvalid,
    SignatureMismatch,
    CandidateInvalid,
    CandidateNotBetter,
    WriteThrottled,
    PromotionNotPrepared,
    OutOfMemory,
    CandidateStale,
    CommitUncertain,
    StorageDegraded,
    ApplyPending,
    CandidateAlreadyPromoted,
    RuntimeCalibrationDiverged,
    RuntimeSensorSignatureDiverged,
};

enum class TrackerConfigLoadStatus : uint8_t {
    None = 0,
    Loaded,
    Migrated,
    DefaultsNotFound,
    DefaultsStorageError,
};

struct TrackerConfigNvsInfo {
    // Compatibility summary for the currently selected active config.
    bool beginOk = false;
    bool exists = false;
    size_t storedLen = 0;
    size_t expectedLen = sizeof(TrackerConfigSlotRecord);
    uint32_t storedMagic = 0;
    uint16_t storedVersion = 0;
    uint16_t storedSize = 0;
    uint32_t storedCrc = 0;
    bool headerReadable = false;
    bool fullReadable = false;
    bool valid = false;
    TrackerConfigError error = TrackerConfigError::None;

    TrackerConfigStorageInfo storage;
};

class TrackerConfigStore {
public:
    explicit TrackerConfigStore(const char* nvsNamespace = tracker_config_detail::NVS_NAMESPACE,
                                const char* key = tracker_config_detail::NVS_KEY_CONFIG);

    TrackerConfigError lastError() const;
    const char* lastErrorName() const;
    TrackerConfigLoadStatus lastLoadStatus() const;
    const char* lastLoadStatusName() const;
    TrackerConfigError lastLoadError() const;
    static const char* errorName(TrackerConfigError e);
    static const char* loadStatusName(TrackerConfigLoadStatus status);

    bool inspect(TrackerConfigNvsInfo& info);
    bool inspectStorage(TrackerConfigStorageInfo& info);
    bool load(TrackerConfig& out);
    // Read the authoritative active slot without migration, cleanup, repair,
    // or clearing recovery latches. Intended for diagnostics only.
    bool verify(TrackerConfig& out);
    // Recovery latches are cleared only after the loaded config has been
    // applied successfully to hardware and runtime.
    void confirmAuthoritativeConfigApplied();
    void markAuthoritativeConfigApplyFailed();
    bool loadOrDefaults(TrackerConfig& out, bool* loadedFromNvs = nullptr);
    // Writes the sanitized config to the inactive slot, verifies read-back,
    // then atomically switches the CRC-protected selector. The previous slot is
    // retained. A selector verification failure is reconciled against NVS;
    // CommitUncertain means the caller must reload/reboot before assuming
    // either generation is authoritative.
    bool save(TrackerConfig& config,
              TrackerCalibrationProvenance provenance = TrackerCalibrationProvenance::Unknown);
    bool erase();
    bool exists();

    bool migrateLegacy();

    void setWearPolicy(const TrackerCalibrationWearPolicy& policy);
    const TrackerCalibrationWearPolicy& wearPolicy() const;

    // Stage a complete calibration/config candidate in RAM. The active config
    // is never modified. Future background collectors can call this repeatedly
    // without wearing NVS.
    bool stageCandidate(const TrackerConfig& candidate,
                        TrackerCalibrationCandidateMetadata metadata,
                        uint32_t nowMs);
    bool candidateDirty() const;
    bool flushCandidate(uint32_t nowMs, bool force = false);
    bool loadCandidate(TrackerCalibrationCandidateRecord& out);
    bool discardCandidate();
    bool compareCandidate(TrackerCalibrationCandidateRecord& outCandidate,
                          TrackerCalibrationComparisonResult& outResult,
                          uint32_t& outFlags);

    // Promotion is two-phase so the caller can apply calibration-owned state
    // to runtime before the selector changes. Sensor signature equality means
    // IMU/FIFO hardware modes are unchanged and need no reconfiguration. An
    // aborted prepare restores the inactive slot to the previous active record.
    bool prepareCandidatePromotion(TrackerPreparedConfigPromotion& out,
                                   TrackerConfig& outCandidate,
                                   bool force = false,
                                   const TrackerConfig* runtimeCalibration = nullptr);
    bool commitPreparedPromotion(TrackerPreparedConfigPromotion& prepared,
                                 TrackerConfig& outActive);
    void abortPreparedPromotion(TrackerPreparedConfigPromotion& prepared);

private:
    const char* ns_ = nullptr;
    char legacyKey_[16] = {};
    char slotAKey_[16] = {};
    char slotBKey_[16] = {};
    char selectorKey_[16] = {};
    char candidateKey_[16] = {};
    char slotACommitKey_[16] = {};
    char slotBCommitKey_[16] = {};

    TrackerConfigError lastError_ = TrackerConfigError::None;
    TrackerConfigLoadStatus lastLoadStatus_ = TrackerConfigLoadStatus::None;
    TrackerConfigError lastLoadError_ = TrackerConfigError::None;
    TrackerCalibrationWearPolicy wearPolicy_;
#if TRACKER_ENABLE_CALIBRATION_CANDIDATES
    TrackerCalibrationCandidateRecord ramCandidate_{};
    bool ramCandidateValid_ = false;
    bool ramCandidateDirty_ = false;
#endif
    uint32_t lastCandidateWriteMs_ = 0;
    bool candidateWriteTimestampValid_ = false;

    uint32_t loadFallbacks_ = 0;
    uint32_t selectorRepairFailures_ = 0;
    uint32_t candidateStageCount_ = 0;
    uint32_t candidateFlushCount_ = 0;
    uint32_t candidateFlushThrottled_ = 0;
    uint32_t candidateRejectedCount_ = 0;
    bool legacyCleanupPending_ = false;
    uint32_t legacyCleanupFailures_ = 0;
    uint32_t commitUncertainCount_ = 0;
    bool commitUncertainLatched_ = false;
    bool storageDegradedLatched_ = false;
    bool authoritativeApplyPending_ = false;
    uint32_t applyPendingWriteBlocks_ = 0;
    uint32_t degradedWriteBlocks_ = 0;
    uint32_t noOpSaveCount_ = 0;
    uint32_t commitMarkerRepairFailures_ = 0;
    uint32_t candidatePromotionStateWrites_ = 0;
    uint32_t candidatePromotionStateWriteFailures_ = 0;

    const char* slotKey(TrackerConfigSlot slot) const;
    const char* commitKey(TrackerConfigSlot slot) const;
    static TrackerConfigSlot otherSlot(TrackerConfigSlot slot);

    bool readSlot(TrackerConfigSlot slot,
                  TrackerConfigSlotRecord& out,
                  bool& exists,
                  bool& readable);
    bool writeSlotVerified(TrackerConfigSlot slot,
                           const TrackerConfigSlotRecord& record);
    bool readCommit(TrackerConfigSlot slot,
                    TrackerConfigCommitRecord& out,
                    bool& exists,
                    bool& readable);
    bool writeCommitVerified(TrackerConfigSlot slot, uint32_t generation);
    bool removeCommitMarker(TrackerConfigSlot slot);
    bool slotHasCommitAuthority(TrackerConfigSlot slot,
                                const TrackerConfigSlotRecord& record,
                                const TrackerConfigCommitRecord& commit,
                                bool commitExists,
                                bool commitReadable) const;
    bool readSelector(TrackerConfigSelectorRecord& out,
                      bool& exists,
                      bool& readable);
    bool writeSelectorVerified(const TrackerConfigSelectorRecord& selector);
    bool reconcileSelectorCommit(const TrackerConfigSelectorRecord& intended,
                                 const TrackerConfigSelectorRecord& previous,
                                 bool& outCommitted);
    bool invalidatePreparedSlot(const TrackerPreparedConfigPromotion& prepared);
    bool cleanupLegacyKey();
    bool clearUncommittedActiveArtifactsForLegacyRecovery();
    bool readLegacy(TrackerConfig& out, bool& exists);
    bool readCandidateFromNvs(TrackerCalibrationCandidateRecord& out,
                              bool& exists,
                              bool& readable);
    bool writeCandidateVerified(const TrackerCalibrationCandidateRecord& candidate);

    bool resolveActive(TrackerConfigSlotRecord& outRecord,
                       TrackerConfigSlot& outSlot,
                       TrackerConfigSelectorRecord& outSelector,
                       bool& outUsedFallback,
                       bool allowRepair = true);
    bool saveInternal(TrackerConfig& config,
                      bool migration,
                      bool promotion,
                      TrackerConfigSlot forcedTarget = TrackerConfigSlot::None,
                      bool switchSelector = true,
                      TrackerPreparedConfigPromotion* prepared = nullptr,
                      const TrackerCalibrationQualitySummary* qualityOverride = nullptr,
                      TrackerCalibrationProvenance provenance = TrackerCalibrationProvenance::Unknown);
    bool candidateQualityComparison(TrackerCalibrationCandidateRecord& candidate,
                                    const TrackerConfigSlotRecord* active,
                                    TrackerCalibrationComparisonResult& result,
                                    uint32_t& flags) const;
};

} // namespace tracker
