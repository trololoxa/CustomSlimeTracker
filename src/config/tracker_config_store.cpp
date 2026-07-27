#include "config/tracker_config_store.hpp"

#include <Preferences.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>

namespace tracker {

namespace {

bool writeRecord(Preferences& prefs, const char* key, const void* data, size_t size) {
    return prefs.putBytes(key, data, size) == size;
}

bool removeIfPresent(Preferences& prefs, const char* key) {
    if (!prefs.isKey(key)) return true;
    return prefs.remove(key);
}

uint32_t nextGeneration(uint32_t current) {
    const uint32_t next = current + 1u;
    return next == 0u ? 1u : next;
}

bool qualityRegressed(float candidate, float active, float tolerance = 0.001f) {
    return candidate + tolerance < active;
}

uint32_t candidateRevisionForPayload(uint16_t candidateVersion,
                                     const TrackerConfigBlob& payload) {
    if (candidateVersion == tracker_config_storage_detail::METADATA_REVISION_CANDIDATE_VERSION) {
        return trackerCalibrationPayloadRevisionLegacyV2(payload);
    }
    return trackerCalibrationPayloadRevision(payload);
}

template <typename T>
std::unique_ptr<T> makeScratch(TrackerConfigError& error) {
    std::unique_ptr<T> scratch(new (std::nothrow) T{});
    if (!scratch) error = TrackerConfigError::OutOfMemory;
    return scratch;
}

struct ResolveActiveScratch {
    TrackerConfigSlotRecord a;
    TrackerConfigSlotRecord b;
    TrackerConfigCommitRecord aCommit;
    TrackerConfigCommitRecord bCommit;
};

struct SaveInternalScratch {
    TrackerConfig candidate;
    TrackerConfig activeConfig;
    TrackerConfigSlotRecord activeRecord;
    TrackerConfigSlotRecord record;
};

struct InspectStorageScratch {
    TrackerConfigSlotRecord a;
    TrackerConfigSlotRecord b;
    TrackerConfigCommitRecord aCommit;
    TrackerConfigCommitRecord bCommit;
    TrackerConfig legacy;
    TrackerCalibrationCandidateRecord candidate;
};

struct CandidateStageScratch {
    TrackerConfig candidate;
    TrackerCalibrationCandidateRecord previous;
    TrackerConfigSlotRecord active;
    TrackerCalibrationCandidateRecord staged;
    TrackerConfigSelectorRecord selector;
};

struct CandidateFlushScratch {
    TrackerCalibrationCandidateRecord compared;
    TrackerConfigSlotRecord active;
};

struct PromotionScratch {
    TrackerCalibrationCandidateRecord candidate;
    TrackerConfigSlotRecord active;
    TrackerConfig candidateSnapshot;
    TrackerConfig activeConfig;
    TrackerConfig composedConfig;
    TrackerConfigSelectorRecord selector;
    TrackerCalibrationQualitySummary promotedQuality;
};

} // namespace

TrackerConfigStore::TrackerConfigStore(const char* nvsNamespace,
                                       const char* key)
    : ns_(nvsNamespace) {
    const char* base = (key && *key) ? key : tracker_config_detail::NVS_KEY_CONFIG;
    std::snprintf(legacyKey_, sizeof(legacyKey_), "%.*s", 15, base);
    std::snprintf(slotAKey_, sizeof(slotAKey_), "%.*s_a", 13, base);
    std::snprintf(slotBKey_, sizeof(slotBKey_), "%.*s_b", 13, base);
    std::snprintf(selectorKey_, sizeof(selectorKey_), "%.*s_s", 13, base);
    std::snprintf(candidateKey_, sizeof(candidateKey_), "%.*s_c", 13, base);
    std::snprintf(slotACommitKey_, sizeof(slotACommitKey_), "%.*s_ac", 12, base);
    std::snprintf(slotBCommitKey_, sizeof(slotBCommitKey_), "%.*s_bc", 12, base);
}

TrackerConfigError TrackerConfigStore::lastError() const {
    return lastError_;
}

TrackerConfigLoadStatus TrackerConfigStore::lastLoadStatus() const {
    return lastLoadStatus_;
}

TrackerConfigError TrackerConfigStore::lastLoadError() const {
    return lastLoadError_;
}

const char* TrackerConfigStore::lastLoadStatusName() const {
    return loadStatusName(lastLoadStatus_);
}

const char* TrackerConfigStore::lastErrorName() const {
    return errorName(lastError_);
}

const char* TrackerConfigStore::errorName(TrackerConfigError e) {
    switch (e) {
        case TrackerConfigError::None:                  return "None";
        case TrackerConfigError::NvsBeginFailed:        return "NvsBeginFailed";
        case TrackerConfigError::NotFound:              return "NotFound";
        case TrackerConfigError::SizeMismatch:          return "SizeMismatch";
        case TrackerConfigError::ReadFailed:            return "ReadFailed";
        case TrackerConfigError::WriteFailed:           return "WriteFailed";
        case TrackerConfigError::RemoveFailed:          return "RemoveFailed";
        case TrackerConfigError::CrcOrValidationFailed: return "CrcOrValidationFailed";
        case TrackerConfigError::SelectorInvalid:       return "SelectorInvalid";
        case TrackerConfigError::SignatureMismatch:     return "SignatureMismatch";
        case TrackerConfigError::CandidateInvalid:      return "CandidateInvalid";
        case TrackerConfigError::CandidateNotBetter:    return "CandidateNotBetter";
        case TrackerConfigError::WriteThrottled:        return "WriteThrottled";
        case TrackerConfigError::PromotionNotPrepared:  return "PromotionNotPrepared";
        case TrackerConfigError::OutOfMemory:             return "OutOfMemory";
        case TrackerConfigError::CandidateStale:          return "CandidateStale";
        case TrackerConfigError::CommitUncertain:         return "CommitUncertain";
        case TrackerConfigError::StorageDegraded:          return "StorageDegraded";
        case TrackerConfigError::ApplyPending:              return "ApplyPending";
        case TrackerConfigError::CandidateAlreadyPromoted: return "CandidateAlreadyPromoted";
        case TrackerConfigError::RuntimeCalibrationDiverged: return "RuntimeCalibrationDiverged";
        case TrackerConfigError::RuntimeSensorSignatureDiverged: return "RuntimeSensorSignatureDiverged";
    }
    return "Unknown";
}

const char* TrackerConfigStore::loadStatusName(TrackerConfigLoadStatus status) {
    switch (status) {
        case TrackerConfigLoadStatus::Loaded: return "loaded";
        case TrackerConfigLoadStatus::Migrated: return "migrated";
        case TrackerConfigLoadStatus::DefaultsNotFound: return "defaults_not_found";
        case TrackerConfigLoadStatus::DefaultsStorageError: return "defaults_storage_error";
        case TrackerConfigLoadStatus::None: break;
    }
    return "none";
}

const char* TrackerConfigStore::slotKey(TrackerConfigSlot slot) const {
    switch (slot) {
        case TrackerConfigSlot::A: return slotAKey_;
        case TrackerConfigSlot::B: return slotBKey_;
        case TrackerConfigSlot::None: break;
    }
    return nullptr;
}

const char* TrackerConfigStore::commitKey(TrackerConfigSlot slot) const {
    switch (slot) {
        case TrackerConfigSlot::A: return slotACommitKey_;
        case TrackerConfigSlot::B: return slotBCommitKey_;
        case TrackerConfigSlot::None: break;
    }
    return nullptr;
}

TrackerConfigSlot TrackerConfigStore::otherSlot(TrackerConfigSlot slot) {
    return slot == TrackerConfigSlot::A ? TrackerConfigSlot::B : TrackerConfigSlot::A;
}

bool TrackerConfigStore::readSlot(TrackerConfigSlot slot,
                                  TrackerConfigSlotRecord& out,
                                  bool& exists,
                                  bool& readable) {
    exists = false;
    readable = false;
    out = TrackerConfigSlotRecord{};
    const char* key = slotKey(slot);
    if (!key) return false;

    Preferences prefs;
    if (!prefs.begin(ns_, true)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    exists = prefs.isKey(key);
    if (!exists) {
        prefs.end();
        return true;
    }
    const size_t len = prefs.getBytesLength(key);
    if (len != sizeof(out)) {
        prefs.end();
        return true;
    }
    readable = prefs.getBytes(key, &out, sizeof(out)) == sizeof(out);
    prefs.end();
    return true;
}

bool TrackerConfigStore::writeSlotVerified(TrackerConfigSlot slot,
                                           const TrackerConfigSlotRecord& record) {
    const char* key = slotKey(slot);
    if (!key || !trackerValidateConfigSlotRecord(record)) {
        lastError_ = TrackerConfigError::CrcOrValidationFailed;
        return false;
    }

    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    const bool written = writeRecord(prefs, key, &record, sizeof(record));
    prefs.end();
    if (!written) {
        lastError_ = TrackerConfigError::WriteFailed;
        return false;
    }

    auto verify = makeScratch<TrackerConfigSlotRecord>(lastError_);
    if (!verify) return false;
    bool exists = false;
    bool readable = false;
    if (!readSlot(slot, *verify, exists, readable) || !exists || !readable ||
        !trackerValidateConfigSlotRecord(*verify) ||
        std::memcmp(verify.get(), &record, sizeof(record)) != 0) {
        lastError_ = TrackerConfigError::ReadFailed;
        return false;
    }
    return true;
}

bool TrackerConfigStore::readCommit(TrackerConfigSlot slot,
                                    TrackerConfigCommitRecord& out,
                                    bool& exists,
                                    bool& readable) {
    exists = false;
    readable = false;
    out = TrackerConfigCommitRecord{};
    const char* key = commitKey(slot);
    if (!key) return false;

    Preferences prefs;
    if (!prefs.begin(ns_, true)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    exists = prefs.isKey(key);
    if (!exists) {
        prefs.end();
        return true;
    }
    const size_t len = prefs.getBytesLength(key);
    if (len != sizeof(out)) {
        prefs.end();
        return true;
    }
    readable = prefs.getBytes(key, &out, sizeof(out)) == sizeof(out);
    prefs.end();
    return true;
}

bool TrackerConfigStore::writeCommitVerified(TrackerConfigSlot slot, uint32_t generation) {
    const char* key = commitKey(slot);
    if (!key || generation == 0u) {
        lastError_ = TrackerConfigError::CrcOrValidationFailed;
        return false;
    }
    TrackerConfigCommitRecord record{};
    record.slot = slot;
    record.generation = generation;
    record.crc32 = 0;
    record.crc32 = trackerConfigCommitRecordCrc(record);

    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    const bool written = writeRecord(prefs, key, &record, sizeof(record));
    prefs.end();
    if (!written) {
        lastError_ = TrackerConfigError::WriteFailed;
        return false;
    }

    TrackerConfigCommitRecord verify{};
    bool exists = false;
    bool readable = false;
    if (!readCommit(slot, verify, exists, readable) || !exists || !readable ||
        !trackerValidateConfigCommitRecord(verify) ||
        std::memcmp(&verify, &record, sizeof(record)) != 0) {
        lastError_ = TrackerConfigError::ReadFailed;
        return false;
    }
    return true;
}

bool TrackerConfigStore::removeCommitMarker(TrackerConfigSlot slot) {
    const char* key = commitKey(slot);
    if (!key) return false;
    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    const bool removed = removeIfPresent(prefs, key);
    prefs.end();
    if (!removed) {
        lastError_ = TrackerConfigError::RemoveFailed;
        return false;
    }
    return true;
}

bool TrackerConfigStore::slotHasCommitAuthority(
    TrackerConfigSlot slot,
    const TrackerConfigSlotRecord& record,
    const TrackerConfigCommitRecord& commit,
    bool commitExists,
    bool commitReadable) const {
    if (record.version == tracker_config_storage_detail::LEGACY_SLOT_VERSION) return true;
    return commitExists && commitReadable && trackerValidateConfigCommitRecord(commit) &&
           commit.slot == slot && commit.generation == record.generation;
}

bool TrackerConfigStore::readSelector(TrackerConfigSelectorRecord& out,
                                      bool& exists,
                                      bool& readable) {
    exists = false;
    readable = false;
    out = TrackerConfigSelectorRecord{};
    Preferences prefs;
    if (!prefs.begin(ns_, true)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    exists = prefs.isKey(selectorKey_);
    if (!exists) {
        prefs.end();
        return true;
    }
    const size_t len = prefs.getBytesLength(selectorKey_);
    if (len != sizeof(out)) {
        prefs.end();
        return true;
    }
    readable = prefs.getBytes(selectorKey_, &out, sizeof(out)) == sizeof(out);
    prefs.end();
    return true;
}

bool TrackerConfigStore::writeSelectorVerified(const TrackerConfigSelectorRecord& selector) {
    if (!trackerValidateConfigSelectorRecord(selector)) {
        lastError_ = TrackerConfigError::SelectorInvalid;
        return false;
    }
    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    const bool written = writeRecord(prefs, selectorKey_, &selector, sizeof(selector));
    prefs.end();
    if (!written) {
        lastError_ = TrackerConfigError::WriteFailed;
        return false;
    }

    TrackerConfigSelectorRecord verify{};
    bool exists = false;
    bool readable = false;
    if (!readSelector(verify, exists, readable) || !exists || !readable ||
        !trackerValidateConfigSelectorRecord(verify) ||
        std::memcmp(&verify, &selector, sizeof(selector)) != 0) {
        lastError_ = TrackerConfigError::ReadFailed;
        return false;
    }
    return true;
}

bool TrackerConfigStore::reconcileSelectorCommit(
    const TrackerConfigSelectorRecord& intended,
    const TrackerConfigSelectorRecord& previous,
    bool& outCommitted) {
    outCommitted = false;
    for (uint8_t attempt = 0; attempt < 3u; ++attempt) {
        TrackerConfigSelectorRecord actual{};
        bool exists = false;
        bool readable = false;
        if (!readSelector(actual, exists, readable) || !exists || !readable ||
            !trackerValidateConfigSelectorRecord(actual)) {
            continue;
        }
        if (actual.activeSlot == intended.activeSlot &&
            actual.activeGeneration == intended.activeGeneration) {
            outCommitted = true;
            lastError_ = TrackerConfigError::None;
            return true;
        }
        if (actual.activeSlot == previous.activeSlot &&
            actual.activeGeneration == previous.activeGeneration) {
            lastError_ = TrackerConfigError::WriteFailed;
            return true;
        }
        break;
    }
    ++commitUncertainCount_;
    commitUncertainLatched_ = true;
    lastError_ = TrackerConfigError::CommitUncertain;
    return false;
}

bool TrackerConfigStore::invalidatePreparedSlot(
    const TrackerPreparedConfigPromotion& prepared) {
    if (prepared.targetSlot == TrackerConfigSlot::None || prepared.targetGeneration == 0) return true;
    auto record = makeScratch<TrackerConfigSlotRecord>(lastError_);
    if (!record) return false;
    bool exists = false;
    bool readable = false;
    if (!readSlot(prepared.targetSlot, *record, exists, readable)) return false;
    if (!exists) return true;
    if (!readable || !trackerValidateConfigSlotRecord(*record) ||
        record->generation != prepared.targetGeneration) {
        return true;
    }

    // Restore redundancy after an aborted prepare. Copy the still-authoritative
    // active record into the target slot instead of merely deleting it.
    if (trackerValidateConfigSelectorRecord(prepared.previousSelector)) {
        auto previous = makeScratch<TrackerConfigSlotRecord>(lastError_);
        if (!previous) return false;
        bool previousExists = false;
        bool previousReadable = false;
        if (readSlot(prepared.previousSelector.activeSlot,
                     *previous,
                     previousExists,
                     previousReadable) &&
            previousExists && previousReadable &&
            trackerValidateConfigSlotRecord(*previous) &&
            previous->generation == prepared.previousSelector.activeGeneration) {
            if (!writeSlotVerified(prepared.targetSlot, *previous)) return false;
            if (previous->version == tracker_config_storage_detail::SLOT_VERSION) {
                return writeCommitVerified(prepared.targetSlot, previous->generation);
            }
            return removeCommitMarker(prepared.targetSlot);
        }
    }

    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    const char* key = slotKey(prepared.targetSlot);
    const char* markerKey = commitKey(prepared.targetSlot);
    const bool markerRemoved = markerKey && removeIfPresent(prefs, markerKey);
    const bool removed = key && removeIfPresent(prefs, key);
    prefs.end();
    if (!markerRemoved || !removed) {
        lastError_ = TrackerConfigError::RemoveFailed;
        return false;
    }
    return true;
}

bool TrackerConfigStore::cleanupLegacyKey() {
    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        legacyCleanupPending_ = true;
        ++legacyCleanupFailures_;
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    const bool removed = removeIfPresent(prefs, legacyKey_);
    prefs.end();
    if (!removed) {
        legacyCleanupPending_ = true;
        ++legacyCleanupFailures_;
        lastError_ = TrackerConfigError::RemoveFailed;
        return false;
    }
    legacyCleanupPending_ = false;
    return true;
}

bool TrackerConfigStore::readLegacy(TrackerConfig& out, bool& exists) {
    exists = false;
    Preferences prefs;
    if (!prefs.begin(ns_, true)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    exists = prefs.isKey(legacyKey_);
    if (!exists) {
        prefs.end();
        return true;
    }
    const size_t len = prefs.getBytesLength(legacyKey_);
    if (len != sizeof(TrackerConfigBlob)) {
        prefs.end();
        lastError_ = TrackerConfigError::SizeMismatch;
        return false;
    }
    const bool ok = prefs.getBytes(legacyKey_, &out.data, sizeof(out.data)) == sizeof(out.data);
    prefs.end();
    if (!ok) {
        lastError_ = TrackerConfigError::ReadFailed;
        return false;
    }
    if (!out.validate()) {
        lastError_ = TrackerConfigError::CrcOrValidationFailed;
        return false;
    }
    return true;
}

bool TrackerConfigStore::readCandidateFromNvs(TrackerCalibrationCandidateRecord& out,
                                              bool& exists,
                                              bool& readable) {
    exists = false;
    readable = false;
    out = TrackerCalibrationCandidateRecord{};
    Preferences prefs;
    if (!prefs.begin(ns_, true)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    exists = prefs.isKey(candidateKey_);
    if (!exists) {
        prefs.end();
        return true;
    }
    const size_t len = prefs.getBytesLength(candidateKey_);
    if (len != sizeof(out)) {
        prefs.end();
        return true;
    }
    readable = prefs.getBytes(candidateKey_, &out, sizeof(out)) == sizeof(out);
    prefs.end();
    return true;
}

bool TrackerConfigStore::writeCandidateVerified(const TrackerCalibrationCandidateRecord& candidate) {
    if (!trackerValidateCalibrationCandidateRecord(candidate)) {
        lastError_ = TrackerConfigError::CandidateInvalid;
        return false;
    }
    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    const bool written = writeRecord(prefs, candidateKey_, &candidate, sizeof(candidate));
    prefs.end();
    if (!written) {
        lastError_ = TrackerConfigError::WriteFailed;
        return false;
    }

    auto verify = makeScratch<TrackerCalibrationCandidateRecord>(lastError_);
    if (!verify) return false;
    bool exists = false;
    bool readable = false;
    if (!readCandidateFromNvs(*verify, exists, readable) || !exists || !readable ||
        !trackerValidateCalibrationCandidateRecord(*verify) ||
        std::memcmp(verify.get(), &candidate, sizeof(candidate)) != 0) {
        lastError_ = TrackerConfigError::ReadFailed;
        return false;
    }
    return true;
}

bool TrackerConfigStore::resolveActive(TrackerConfigSlotRecord& outRecord,
                                       TrackerConfigSlot& outSlot,
                                       TrackerConfigSelectorRecord& outSelector,
                                       bool& outUsedFallback,
                                       bool allowRepair) {
    outRecord = TrackerConfigSlotRecord{};
    outSlot = TrackerConfigSlot::None;
    outSelector = TrackerConfigSelectorRecord{};
    outUsedFallback = false;

    auto scratch = makeScratch<ResolveActiveScratch>(lastError_);
    if (!scratch) return false;
    auto& a = scratch->a;
    auto& b = scratch->b;
    auto& aCommit = scratch->aCommit;
    auto& bCommit = scratch->bCommit;
    TrackerConfigSelectorRecord selector{};
    bool aExists = false, aReadable = false;
    bool bExists = false, bReadable = false;
    bool aCommitExists = false, aCommitReadable = false;
    bool bCommitExists = false, bCommitReadable = false;
    bool selectorExists = false, selectorReadable = false;
    if (!readSlot(TrackerConfigSlot::A, a, aExists, aReadable) ||
        !readSlot(TrackerConfigSlot::B, b, bExists, bReadable) ||
        !readCommit(TrackerConfigSlot::A, aCommit, aCommitExists, aCommitReadable) ||
        !readCommit(TrackerConfigSlot::B, bCommit, bCommitExists, bCommitReadable) ||
        !readSelector(selector, selectorExists, selectorReadable)) {
        return false;
    }

    const bool aValid = aExists && aReadable && trackerValidateConfigSlotRecord(a);
    const bool bValid = bExists && bReadable && trackerValidateConfigSlotRecord(b);
    const bool aCommitted = aValid && slotHasCommitAuthority(
        TrackerConfigSlot::A, a, aCommit, aCommitExists, aCommitReadable);
    const bool bCommitted = bValid && slotHasCommitAuthority(
        TrackerConfigSlot::B, b, bCommit, bCommitExists, bCommitReadable);
    const bool selectorValid = selectorExists && selectorReadable &&
        trackerValidateConfigSelectorRecord(selector);

    if (selectorValid) {
        const TrackerConfigSlotRecord& selected = selector.activeSlot == TrackerConfigSlot::A ? a : b;
        const bool selectedValid = selector.activeSlot == TrackerConfigSlot::A ? aValid : bValid;
        if (selectedValid && selected.generation == selector.activeGeneration) {
            outRecord = selected;
            outSlot = selector.activeSlot;
            outSelector = selector;
            // The selector is the authority. A v2 commit marker is only the
            // recovery proof used when the selector is later lost, so repair it
            // best-effort without rejecting the selected config.
            const bool selectedCommitted = selector.activeSlot == TrackerConfigSlot::A
                ? aCommitted : bCommitted;
            if (allowRepair && !selectedCommitted &&
                selected.version == tracker_config_storage_detail::SLOT_VERSION) {
                const TrackerConfigError before = lastError_;
                if (!writeCommitVerified(selector.activeSlot, selected.generation)) {
                    ++commitMarkerRepairFailures_;
                    lastError_ = before;
                }
            }
            return true;
        }

        const TrackerConfigSlot fallbackSlot = otherSlot(selector.activeSlot);
        const TrackerConfigSlotRecord& fallback = fallbackSlot == TrackerConfigSlot::A ? a : b;
        const bool fallbackCommitted = fallbackSlot == TrackerConfigSlot::A ? aCommitted : bCommitted;
        // A newer non-selected slot may be a fully written but uncommitted
        // prepare. Only a committed/legacy slot may be a fallback.
        if (fallbackCommitted &&
            !trackerGenerationIsNewer(fallback.generation, selector.activeGeneration)) {
            outRecord = fallback;
            outSlot = fallbackSlot;
            outSelector = selector;
            outUsedFallback = true;
        }
    }

    if (outSlot == TrackerConfigSlot::None && !selectorValid) {
        if (aCommitted && bCommitted) {
            const bool bothV2 =
                a.version == tracker_config_storage_detail::SLOT_VERSION &&
                b.version == tracker_config_storage_detail::SLOT_VERSION;
            if (bothV2) {
                // Both commit markers prove that both generations passed a
                // selector switch. Recover the newest committed generation.
                if (trackerGenerationIsNewer(a.generation, b.generation)) {
                    outRecord = a;
                    outSlot = TrackerConfigSlot::A;
                } else {
                    outRecord = b;
                    outSlot = TrackerConfigSlot::B;
                }
            } else if (a.version == tracker_config_storage_detail::SLOT_VERSION) {
                // A v2 marker is explicit commit proof and is stronger than an
                // ambiguous selector-less v1 record.
                outRecord = a;
                outSlot = TrackerConfigSlot::A;
            } else if (b.version == tracker_config_storage_detail::SLOT_VERSION) {
                outRecord = b;
                outSlot = TrackerConfigSlot::B;
            } else {
                // Both are v1 slots. Their newer record may have been written
                // immediately before power loss, so choose the older.
                if (trackerGenerationIsNewer(a.generation, b.generation)) {
                    outRecord = b;
                    outSlot = TrackerConfigSlot::B;
                } else {
                    outRecord = a;
                    outSlot = TrackerConfigSlot::A;
                }
            }
            outUsedFallback = true;
        } else if (aCommitted) {
            outRecord = a;
            outSlot = TrackerConfigSlot::A;
            outUsedFallback = selectorExists || a.version == tracker_config_storage_detail::SLOT_VERSION;
        } else if (bCommitted) {
            outRecord = b;
            outSlot = TrackerConfigSlot::B;
            outUsedFallback = selectorExists || b.version == tracker_config_storage_detail::SLOT_VERSION;
        }
    }

    if (outSlot != TrackerConfigSlot::None) {
        TrackerConfigSelectorRecord repaired = selectorValid ? selector : TrackerConfigSelectorRecord{};
        repaired.activeSlot = outSlot;
        repaired.activeGeneration = outRecord.generation;
        repaired.crc32 = 0;
        repaired.crc32 = trackerConfigSelectorRecordCrc(repaired);
        if (allowRepair && !writeSelectorVerified(repaired)) {
            // A committed slot remains usable. Selector repair is best-effort.
            ++selectorRepairFailures_;
        }
        outSelector = repaired;
        if (outUsedFallback) ++loadFallbacks_;
        lastError_ = TrackerConfigError::None;
        return true;
    }

    const bool anyActiveStorage = aExists || bExists || selectorExists ||
                                  aCommitExists || bCommitExists;
    lastError_ = anyActiveStorage
        ? TrackerConfigError::CrcOrValidationFailed
        : TrackerConfigError::NotFound;
    return false;
}

bool TrackerConfigStore::saveInternal(TrackerConfig& config,
                                      bool migration,
                                      bool promotion,
                                      TrackerConfigSlot forcedTarget,
                                      bool switchSelector,
                                      TrackerPreparedConfigPromotion* prepared,
                                      const TrackerCalibrationQualitySummary* qualityOverride,
                                      TrackerCalibrationProvenance provenance) {
    if (commitUncertainLatched_) {
        lastError_ = TrackerConfigError::CommitUncertain;
        return false;
    }
    if (storageDegradedLatched_) {
        ++degradedWriteBlocks_;
        lastError_ = TrackerConfigError::StorageDegraded;
        return false;
    }
    if (authoritativeApplyPending_) {
        ++applyPendingWriteBlocks_;
        lastError_ = TrackerConfigError::ApplyPending;
        return false;
    }
    auto scratch = makeScratch<SaveInternalScratch>(lastError_);
    if (!scratch) return false;
    TrackerConfig& candidate = scratch->candidate;
    candidate = config;
    candidate.sanitize();
    candidate.updateCrc();
    if (!candidate.validate()) {
        lastError_ = TrackerConfigError::CrcOrValidationFailed;
        return false;
    }

    TrackerConfigSlotRecord& activeRecord = scratch->activeRecord;
    activeRecord = TrackerConfigSlotRecord{};
    TrackerConfigSlot activeSlot = TrackerConfigSlot::None;
    TrackerConfigSelectorRecord selector{};
    bool fallback = false;
    const bool hasActive = resolveActive(activeRecord, activeSlot, selector, fallback);
    if (!hasActive && lastError_ != TrackerConfigError::NotFound) return false;
    if (!hasActive) {
        activeSlot = TrackerConfigSlot::None;
        selector = TrackerConfigSelectorRecord{};
    }

    bool activeHasDurableV2Commit = false;
    if (hasActive && activeRecord.version == tracker_config_storage_detail::SLOT_VERSION) {
        TrackerConfigCommitRecord activeCommit{};
        bool commitExists = false;
        bool commitReadable = false;
        if (!readCommit(activeSlot, activeCommit, commitExists, commitReadable)) return false;
        activeHasDurableV2Commit = slotHasCommitAuthority(
            activeSlot,
            activeRecord,
            activeCommit,
            commitExists,
            commitReadable
        );
    }

    if (!migration && !promotion && forcedTarget == TrackerConfigSlot::None &&
        switchSelector && qualityOverride == nullptr && hasActive &&
        activeHasDurableV2Commit &&
        std::memcmp(&activeRecord.payload, &candidate.data, sizeof(candidate.data)) == 0) {
        config = candidate;
        ++noOpSaveCount_;
        lastError_ = TrackerConfigError::None;
        return true;
    }

    if (autonomyProbationWriteBarrier_ && !migration && !promotion) {
        // A non-noop save would overwrite the inactive generation that is the
        // current autonomous rollback anchor. Fail closed until probation is
        // accepted or rolled back. The no-op path above remains available.
        lastError_ = TrackerConfigError::ApplyPending;
        return false;
    }

    const TrackerConfigSlot target = forcedTarget != TrackerConfigSlot::None
        ? forcedTarget
        : (activeSlot == TrackerConfigSlot::None ? TrackerConfigSlot::A : otherSlot(activeSlot));
    const uint32_t generation = nextGeneration(hasActive ? activeRecord.generation : 0u);

    TrackerConfigSlotRecord& record = scratch->record;
    record = TrackerConfigSlotRecord{};
    record.generation = generation;
    record.signature = trackerMakeSensorSignature(candidate);
    if (qualityOverride) {
        record.quality = *qualityOverride;
        const uint32_t sourceMask = tracker_calibration_quality_flags::SOURCE_DERIVED |
                                    tracker_calibration_quality_flags::SOURCE_MEASURED;
        if ((record.quality.qualityFlags & sourceMask) == 0u) {
            record.quality.qualityFlags |= tracker_calibration_quality_flags::SOURCE_MEASURED;
        }
        trackerCalibrationQualitySetProvenance(record.quality, provenance);
    } else if (hasActive) {
        TrackerConfig& activeConfig = scratch->activeConfig;
        activeConfig.data = activeRecord.payload;
        if (trackerCalibrationModelEqual(activeConfig, candidate) &&
            trackerCalibrationEvidenceEqual(activeConfig, candidate)) {
            // Product/runtime policy changed, but the learned model and its
            // evidence did not. Preserve the accepted quality and provenance.
            record.quality = activeRecord.quality;
        } else {
            record.quality = trackerCalibrationQualityFromConfig(candidate);
            trackerCalibrationQualitySetProvenance(record.quality, provenance);
        }
    } else {
        record.quality = trackerCalibrationQualityFromConfig(candidate);
        trackerCalibrationQualitySetProvenance(
            record.quality,
            migration ? TrackerCalibrationProvenance::ImportedLegacy : provenance
        );
    }
    record.payload = candidate.data;
    record.crc32 = 0;
    record.crc32 = trackerConfigSlotRecordCrc(record);
    // A stale commit marker from the previous generation of this physical
    // slot must be removed before the prepared record is written.
    if (!removeCommitMarker(target)) return false;
    if (!writeSlotVerified(target, record)) return false;

    if (prepared) {
        prepared->valid = true;
        prepared->targetSlot = target;
        prepared->targetGeneration = generation;
        prepared->activeGenerationAtPreparation = hasActive ? activeRecord.generation : 0u;
        prepared->previousSelector = selector;
    }

    if (!switchSelector) {
        lastError_ = TrackerConfigError::None;
        return true;
    }

    TrackerConfigSelectorRecord nextSelector = selector;
    nextSelector.activeSlot = target;
    nextSelector.activeGeneration = generation;
    nextSelector.successfulActiveWrites += 1u;
    if (migration) nextSelector.successfulMigrations += 1u;
    if (promotion) nextSelector.successfulPromotions += 1u;
    nextSelector.crc32 = 0;
    nextSelector.crc32 = trackerConfigSelectorRecordCrc(nextSelector);
    if (!writeSelectorVerified(nextSelector)) {
        bool committed = false;
        const TrackerConfigError initialError = lastError_;
        if (!reconcileSelectorCommit(nextSelector, selector, committed)) return false;
        if (!committed) {
            lastError_ = initialError;
            return false;
        }
    }

    const TrackerConfigError beforeMarker = lastError_;
    if (!writeCommitVerified(target, generation)) {
        // Selector already committed the generation. A missing marker only
        // weakens selector-loss recovery and is repaired on the next load.
        ++commitMarkerRepairFailures_;
        lastError_ = beforeMarker;
    }

    config = candidate;
    lastError_ = TrackerConfigError::None;
    return true;
}

bool TrackerConfigStore::inspectStorage(TrackerConfigStorageInfo& info) {
    info = TrackerConfigStorageInfo{};
    auto scratch = makeScratch<InspectStorageScratch>(lastError_);
    if (!scratch) return false;
    auto& a = scratch->a;
    auto& b = scratch->b;
    auto& aCommit = scratch->aCommit;
    auto& bCommit = scratch->bCommit;
    TrackerConfigSelectorRecord selector{};
    bool aExists = false, aReadable = false;
    bool bExists = false, bReadable = false;
    bool aCommitExists = false, aCommitReadable = false;
    bool bCommitExists = false, bCommitReadable = false;
    bool selectorExists = false, selectorReadable = false;
    if (!readSlot(TrackerConfigSlot::A, a, aExists, aReadable) ||
        !readSlot(TrackerConfigSlot::B, b, bExists, bReadable) ||
        !readCommit(TrackerConfigSlot::A, aCommit, aCommitExists, aCommitReadable) ||
        !readCommit(TrackerConfigSlot::B, bCommit, bCommitExists, bCommitReadable) ||
        !readSelector(selector, selectorExists, selectorReadable)) {
        return false;
    }

    info.slotA.exists = aExists;
    info.slotA.readable = aReadable;
    info.slotA.valid = aExists && aReadable && trackerValidateConfigSlotRecord(a);
    info.slotA.legacyCommitted = info.slotA.valid &&
        a.version == tracker_config_storage_detail::LEGACY_SLOT_VERSION;
    info.slotA.commitMarkerExists = aCommitExists;
    info.slotA.commitMarkerValid = info.slotA.valid && slotHasCommitAuthority(
        TrackerConfigSlot::A, a, aCommit, aCommitExists, aCommitReadable);
    if (info.slotA.valid) {
        info.slotA.generation = a.generation;
        info.slotA.signature = a.signature;
        info.slotA.quality = a.quality;
    }
    info.slotB.exists = bExists;
    info.slotB.readable = bReadable;
    info.slotB.valid = bExists && bReadable && trackerValidateConfigSlotRecord(b);
    info.slotB.legacyCommitted = info.slotB.valid &&
        b.version == tracker_config_storage_detail::LEGACY_SLOT_VERSION;
    info.slotB.commitMarkerExists = bCommitExists;
    info.slotB.commitMarkerValid = info.slotB.valid && slotHasCommitAuthority(
        TrackerConfigSlot::B, b, bCommit, bCommitExists, bCommitReadable);
    if (info.slotB.valid) {
        info.slotB.generation = b.generation;
        info.slotB.signature = b.signature;
        info.slotB.quality = b.quality;
    }

    info.selectorExists = selectorExists;
    info.selectorValid = selectorExists && selectorReadable && trackerValidateConfigSelectorRecord(selector);
    if (info.selectorValid) {
        info.selectedSlot = selector.activeSlot;
        info.selectedGeneration = selector.activeGeneration;
        info.successfulActiveWrites = selector.successfulActiveWrites;
        info.successfulMigrations = selector.successfulMigrations;
        info.successfulPromotions = selector.successfulPromotions;
    }

    Preferences prefs;
    if (!prefs.begin(ns_, true)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    info.legacyExists = prefs.isKey(legacyKey_);
    prefs.end();
    if (info.legacyExists) {
        TrackerConfig& legacy = scratch->legacy;
        legacy = TrackerConfig{};
        bool exists = false;
        info.legacyValid = readLegacy(legacy, exists);
        if (!info.legacyValid) {
            if (lastError_ == TrackerConfigError::SizeMismatch ||
                lastError_ == TrackerConfigError::CrcOrValidationFailed) {
                lastError_ = TrackerConfigError::None;
            } else {
                return false;
            }
        }
    }

    TrackerCalibrationCandidateRecord& candidate = scratch->candidate;
    candidate = TrackerCalibrationCandidateRecord{};
    bool candidateExists = false;
    bool candidateReadable = false;
    if (!readCandidateFromNvs(candidate, candidateExists, candidateReadable)) return false;
#if TRACKER_ENABLE_CALIBRATION_CANDIDATES
    info.candidate.exists = candidateExists || ramCandidateValid_;
    info.candidate.readable = candidateReadable;
    info.candidate.valid = ramCandidateValid_ ||
        (candidateExists && candidateReadable && trackerValidateCalibrationCandidateRecord(candidate));
    info.candidate.dirtyInRam = ramCandidateDirty_;
    const TrackerCalibrationCandidateRecord* candidateView = ramCandidateValid_ ? &ramCandidate_ : &candidate;
#else
    info.candidate.exists = candidateExists;
    info.candidate.readable = candidateReadable;
    info.candidate.valid = candidateExists && candidateReadable &&
        trackerValidateCalibrationCandidateRecord(candidate);
    info.candidate.dirtyInRam = false;
    const TrackerCalibrationCandidateRecord* candidateView = &candidate;
#endif
    if (info.candidate.valid) {
        info.candidate.generation = candidateView->candidateGeneration;
        info.candidate.persistedWriteCount = candidateView->persistedWriteCount;
        info.candidate.metadata = candidateView->metadata;
    }

    info.loadFallbacks = loadFallbacks_;
    info.selectorRepairFailures = selectorRepairFailures_;
    info.candidateStageCount = candidateStageCount_;
    info.candidateFlushCount = candidateFlushCount_;
    info.candidateFlushThrottled = candidateFlushThrottled_;
    info.candidateRejectedCount = candidateRejectedCount_;
    info.legacyCleanupPending = legacyCleanupPending_;
    info.legacyCleanupFailures = legacyCleanupFailures_;
    info.commitUncertainCount = commitUncertainCount_;
    info.commitMarkerRepairFailures = commitMarkerRepairFailures_;
    info.noOpSaveCount = noOpSaveCount_;
    info.degradedWriteBlocks = degradedWriteBlocks_;
    info.storageDegradedLatched = storageDegradedLatched_;
    info.authoritativeApplyPending = authoritativeApplyPending_;
    info.applyPendingWriteBlocks = applyPendingWriteBlocks_;
    info.candidatePromotionStateWrites = candidatePromotionStateWrites_;
    info.candidatePromotionStateWriteFailures = candidatePromotionStateWriteFailures_;
    lastError_ = TrackerConfigError::None;
    return true;
}

bool TrackerConfigStore::inspect(TrackerConfigNvsInfo& info) {
    info = TrackerConfigNvsInfo{};
    info.beginOk = inspectStorage(info.storage);
    if (!info.beginOk) {
        info.error = lastError_;
        return false;
    }

    auto record = makeScratch<TrackerConfigSlotRecord>(lastError_);
    if (!record) {
        info.error = lastError_;
        return false;
    }
    TrackerConfigSlot slot = TrackerConfigSlot::None;
    TrackerConfigSelectorRecord selector{};
    bool fallback = false;
    if (!resolveActive(*record, slot, selector, fallback)) {
        info.error = lastError_;
        return true;
    }
    info.exists = true;
    info.storedLen = sizeof(*record);
    info.headerReadable = true;
    info.fullReadable = true;
    info.valid = true;
    info.storedMagic = record->magic;
    info.storedVersion = record->version;
    info.storedSize = record->size;
    info.storedCrc = record->crc32;
    info.storage.selectedByFallback = fallback;
    info.storage.selectedSlot = slot;
    info.storage.selectedGeneration = record->generation;
    TrackerConfigSelectorRecord actualSelector{};
    bool selectorExists = false;
    bool selectorReadable = false;
    if (readSelector(actualSelector, selectorExists, selectorReadable)) {
        info.storage.selectorExists = selectorExists;
        info.storage.selectorValid = selectorExists && selectorReadable &&
            trackerValidateConfigSelectorRecord(actualSelector);
        if (info.storage.selectorValid) {
            if (actualSelector.activeSlot == slot &&
                actualSelector.activeGeneration == record->generation) {
                info.storage.selectedSlot = actualSelector.activeSlot;
                info.storage.selectedGeneration = actualSelector.activeGeneration;
            }
            info.storage.successfulActiveWrites = actualSelector.successfulActiveWrites;
            info.storage.successfulMigrations = actualSelector.successfulMigrations;
            info.storage.successfulPromotions = actualSelector.successfulPromotions;
        }
    }
    info.error = TrackerConfigError::None;
    lastError_ = TrackerConfigError::None;
    return true;
}

bool TrackerConfigStore::load(TrackerConfig& out) {
    lastLoadStatus_ = TrackerConfigLoadStatus::None;
    lastLoadError_ = TrackerConfigError::None;
    auto record = makeScratch<TrackerConfigSlotRecord>(lastError_);
    if (!record) {
        lastLoadError_ = lastError_;
        return false;
    }
    TrackerConfigSlot slot = TrackerConfigSlot::None;
    TrackerConfigSelectorRecord selector{};
    bool fallback = false;
    if (!resolveActive(*record, slot, selector, fallback)) {
        if (lastError_ != TrackerConfigError::NotFound &&
            lastError_ != TrackerConfigError::CrcOrValidationFailed) {
            lastLoadError_ = lastError_;
            return false;
        }
        if (!migrateLegacy()) {
            lastLoadError_ = lastError_;
            return false;
        }
        if (!resolveActive(*record, slot, selector, fallback)) {
            lastLoadError_ = lastError_;
            return false;
        }
        lastLoadStatus_ = TrackerConfigLoadStatus::Migrated;
    } else {
        lastLoadStatus_ = TrackerConfigLoadStatus::Loaded;
    }
    out.data = record->payload;
    Preferences legacyProbe;
    bool staleLegacyExists = false;
    if (legacyProbe.begin(ns_, true)) {
        staleLegacyExists = legacyProbe.isKey(legacyKey_);
        legacyProbe.end();
    }
    if (staleLegacyExists) (void)cleanupLegacyKey();
    authoritativeApplyPending_ = true;
    lastLoadError_ = TrackerConfigError::None;
    lastError_ = TrackerConfigError::None;
    return true;
}

bool TrackerConfigStore::verify(TrackerConfig& out) {
    auto record = makeScratch<TrackerConfigSlotRecord>(lastError_);
    if (!record) return false;
    TrackerConfigSlot slot = TrackerConfigSlot::None;
    TrackerConfigSelectorRecord selector{};
    bool fallback = false;
    if (!resolveActive(*record, slot, selector, fallback, false)) return false;
    out.data = record->payload;
    lastError_ = TrackerConfigError::None;
    return true;
}

void TrackerConfigStore::confirmAuthoritativeConfigApplied() {
    commitUncertainLatched_ = false;
    storageDegradedLatched_ = false;
    authoritativeApplyPending_ = false;
    lastError_ = TrackerConfigError::None;
}

void TrackerConfigStore::markAuthoritativeConfigApplyFailed() {
    authoritativeApplyPending_ = false;
    storageDegradedLatched_ = true;
}

bool TrackerConfigStore::loadOrDefaults(TrackerConfig& out, bool* loadedFromNvs) {
    if (load(out)) {
        if (loadedFromNvs) *loadedFromNvs = true;
        return true;
    }
    const TrackerConfigError loadError = lastError_;
    out.resetDefaults();
    if (loadedFromNvs) *loadedFromNvs = false;
    lastLoadError_ = loadError;
    lastLoadStatus_ = loadError == TrackerConfigError::NotFound
        ? TrackerConfigLoadStatus::DefaultsNotFound
        : TrackerConfigLoadStatus::DefaultsStorageError;
    authoritativeApplyPending_ = false;
    storageDegradedLatched_ = loadError != TrackerConfigError::NotFound;
    lastError_ = loadError;
    return true;
}

bool TrackerConfigStore::save(TrackerConfig& config,
                              TrackerCalibrationProvenance provenance) {
    return saveInternal(config, false, false, TrackerConfigSlot::None, true,
                        nullptr, nullptr, provenance);
}

bool TrackerConfigStore::clearUncommittedActiveArtifactsForLegacyRecovery() {
    auto scratch = makeScratch<ResolveActiveScratch>(lastError_);
    if (!scratch) return false;
    bool aExists = false, aReadable = false;
    bool bExists = false, bReadable = false;
    bool aCommitExists = false, aCommitReadable = false;
    bool bCommitExists = false, bCommitReadable = false;
    bool selectorExists = false, selectorReadable = false;
    TrackerConfigSelectorRecord selector{};
    if (!readSlot(TrackerConfigSlot::A, scratch->a, aExists, aReadable) ||
        !readSlot(TrackerConfigSlot::B, scratch->b, bExists, bReadable) ||
        !readCommit(TrackerConfigSlot::A, scratch->aCommit, aCommitExists, aCommitReadable) ||
        !readCommit(TrackerConfigSlot::B, scratch->bCommit, bCommitExists, bCommitReadable) ||
        !readSelector(selector, selectorExists, selectorReadable)) {
        return false;
    }

    // Never erase evidence that could merely be temporarily unreadable.
    if ((aExists && !aReadable) || (bExists && !bReadable) ||
        (aCommitExists && !aCommitReadable) || (bCommitExists && !bCommitReadable) ||
        (selectorExists && !selectorReadable)) {
        lastError_ = TrackerConfigError::ReadFailed;
        return false;
    }

    const bool aValid = aExists && trackerValidateConfigSlotRecord(scratch->a);
    const bool bValid = bExists && trackerValidateConfigSlotRecord(scratch->b);
    const bool aCommitted = aValid && slotHasCommitAuthority(
        TrackerConfigSlot::A, scratch->a, scratch->aCommit, aCommitExists, aCommitReadable);
    const bool bCommitted = bValid && slotHasCommitAuthority(
        TrackerConfigSlot::B, scratch->b, scratch->bCommit, bCommitExists, bCommitReadable);
    const bool selectorValid = selectorExists && trackerValidateConfigSelectorRecord(selector);
    const bool selectorHasValidTarget = selectorValid &&
        ((selector.activeSlot == TrackerConfigSlot::A && aValid &&
          scratch->a.generation == selector.activeGeneration) ||
         (selector.activeSlot == TrackerConfigSlot::B && bValid &&
          scratch->b.generation == selector.activeGeneration));
    if (aCommitted || bCommitted || selectorHasValidTarget) {
        lastError_ = TrackerConfigError::SelectorInvalid;
        return false;
    }

    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    const bool ok = removeIfPresent(prefs, selectorKey_) &&
                    removeIfPresent(prefs, slotAKey_) &&
                    removeIfPresent(prefs, slotBKey_) &&
                    removeIfPresent(prefs, slotACommitKey_) &&
                    removeIfPresent(prefs, slotBCommitKey_);
    prefs.end();
    if (!ok) {
        lastError_ = TrackerConfigError::RemoveFailed;
        return false;
    }
    lastError_ = TrackerConfigError::None;
    return true;
}

bool TrackerConfigStore::migrateLegacy() {
    if (authoritativeApplyPending_) {
        ++applyPendingWriteBlocks_;
        lastError_ = TrackerConfigError::ApplyPending;
        return false;
    }
    // Never let a stale legacy key overwrite an already authoritative dual-slot
    // generation. In that state migration is only pending-key cleanup.
    auto active = makeScratch<TrackerConfigSlotRecord>(lastError_);
    if (!active) return false;
    TrackerConfigSlot activeSlot = TrackerConfigSlot::None;
    TrackerConfigSelectorRecord selector{};
    bool fallback = false;
    if (resolveActive(*active, activeSlot, selector, fallback)) {
        const bool cleaned = cleanupLegacyKey();
        if (!cleaned) {
            // Active config is still valid and authoritative. Cleanup failure is
            // diagnostic, not a reason to boot defaults.
            lastError_ = TrackerConfigError::None;
        }
        return true;
    }
    const TrackerConfigError activeError = lastError_;
    if (activeError != TrackerConfigError::NotFound &&
        activeError != TrackerConfigError::CrcOrValidationFailed) {
        return false;
    }

    auto legacy = makeScratch<TrackerConfig>(lastError_);
    if (!legacy) return false;
    bool legacyExists = false;
    if (!readLegacy(*legacy, legacyExists)) return false;
    if (!legacyExists) {
        lastError_ = activeError;
        return false;
    }

    if (activeError == TrackerConfigError::CrcOrValidationFailed) {
        // A failed first migration may leave a fully written but uncommitted v2
        // slot. The still-valid legacy blob is the only authoritative source.
        // Clear only artifacts that are fully readable and have no commit proof.
        if (!clearUncommittedActiveArtifactsForLegacyRecovery()) return false;
    }

    if (!saveInternal(*legacy, true, false, TrackerConfigSlot::A, true, nullptr, nullptr,
                      TrackerCalibrationProvenance::ImportedLegacy)) return false;
    (void)cleanupLegacyKey();
    // Cleanup may be retried later; the committed active slot remains usable.
    lastError_ = TrackerConfigError::None;
    return true;
}

bool TrackerConfigStore::erase() {
    TrackerConfigSelectorRecord selector{};
    bool selectorExists = false;
    bool selectorReadable = false;
    const bool selectorReadOk = readSelector(selector, selectorExists, selectorReadable);
    const bool selectorValid = selectorReadOk && selectorExists && selectorReadable &&
        trackerValidateConfigSelectorRecord(selector);
    const TrackerConfigSlot selected = selectorValid ? selector.activeSlot : TrackerConfigSlot::None;

    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    bool ok = true;
    const auto removeKey = [&](const char* key) {
        if (!removeIfPresent(prefs, key)) ok = false;
    };

    // Remove non-authoritative state first. If any early removal fails, the
    // selected slot and selector are still intact. For a selected config,
    // remove selector before the selected slot and its commit marker last; any
    // interrupted erase therefore still leaves a recoverable slot proof.
    removeKey(candidateKey_);
    removeKey(legacyKey_);
    if (selected == TrackerConfigSlot::A) {
        removeKey(slotBCommitKey_);
        removeKey(slotBKey_);
        removeKey(selectorKey_);
        removeKey(slotAKey_);
        removeKey(slotACommitKey_);
    } else if (selected == TrackerConfigSlot::B) {
        removeKey(slotACommitKey_);
        removeKey(slotAKey_);
        removeKey(selectorKey_);
        removeKey(slotBKey_);
        removeKey(slotBCommitKey_);
    } else {
        removeKey(selectorKey_);
        removeKey(slotAKey_);
        removeKey(slotBKey_);
        removeKey(slotACommitKey_);
        removeKey(slotBCommitKey_);
    }
    prefs.end();
    if (!ok) {
        lastError_ = TrackerConfigError::RemoveFailed;
        return false;
    }
#if TRACKER_ENABLE_CALIBRATION_CANDIDATES
    ramCandidate_ = TrackerCalibrationCandidateRecord{};
    ramCandidateValid_ = false;
    ramCandidateDirty_ = false;
#endif
    commitUncertainLatched_ = false;
    storageDegradedLatched_ = false;
    authoritativeApplyPending_ = false;
    legacyCleanupPending_ = false;
    lastError_ = TrackerConfigError::None;
    return true;
}

bool TrackerConfigStore::exists() {
    auto record = makeScratch<TrackerConfigSlotRecord>(lastError_);
    if (!record) return false;
    TrackerConfigSlot slot = TrackerConfigSlot::None;
    TrackerConfigSelectorRecord selector{};
    bool fallback = false;
    const bool ok = resolveActive(*record, slot, selector, fallback);
    if (!ok && lastError_ == TrackerConfigError::NotFound) return false;
    return ok;
}

void TrackerConfigStore::setWearPolicy(const TrackerCalibrationWearPolicy& policy) {
    wearPolicy_ = policy;
    if (!std::isfinite(wearPolicy_.minQualityImprovement) ||
        wearPolicy_.minQualityImprovement < 0.0f) {
        wearPolicy_.minQualityImprovement =
            tracker_config_storage_detail::DEFAULT_CANDIDATE_MIN_QUALITY_IMPROVEMENT;
    }
}

const TrackerCalibrationWearPolicy& TrackerConfigStore::wearPolicy() const {
    return wearPolicy_;
}

bool TrackerConfigStore::candidateQualityComparison(TrackerCalibrationCandidateRecord& candidate,
                                                    const TrackerConfigSlotRecord* active,
                                                    TrackerCalibrationComparisonResult& result,
                                                    uint32_t& flags) const {
    flags = tracker_calibration_comparison_flags::NONE;
    if (!trackerValidateCalibrationCandidateRecord(candidate)) {
        result = TrackerCalibrationComparisonResult::InvalidCandidate;
        flags |= tracker_calibration_comparison_flags::QUALITY_NONFINITE;
        return false;
    }
    if (!active) {
        flags |= tracker_calibration_comparison_flags::ACTIVE_MISSING;
        result = TrackerCalibrationComparisonResult::Better;
        return true;
    }
    if (!trackerSensorSignaturesEqual(candidate.signature, active->signature)) {
        flags |= tracker_calibration_comparison_flags::SIGNATURE_MISMATCH;
        result = TrackerCalibrationComparisonResult::SignatureMismatch;
        return false;
    }
    const uint32_t activeCalibrationRevision =
        candidateRevisionForPayload(candidate.version, active->payload);
    if (candidate.version != tracker_config_storage_detail::GENERATION_CANDIDATE_VERSION &&
        candidate.metadata.lastComparison == TrackerCalibrationComparisonResult::Promoted &&
        candidate.metadata.activeCalibrationRevisionAtCreation == activeCalibrationRevision) {
        flags |= tracker_calibration_comparison_flags::ALREADY_PROMOTED;
        result = TrackerCalibrationComparisonResult::AlreadyPromoted;
        return false;
    }
    const bool staleBase =
        candidate.version == tracker_config_storage_detail::GENERATION_CANDIDATE_VERSION
            ? candidate.metadata.activeCalibrationRevisionAtCreation != active->generation
            : candidate.metadata.activeCalibrationRevisionAtCreation != activeCalibrationRevision;
    if (staleBase) {
        flags |= tracker_calibration_comparison_flags::STALE_ACTIVE_GENERATION;
        result = TrackerCalibrationComparisonResult::StaleActiveGeneration;
        return false;
    }

    const auto& cq = candidate.metadata.quality;
    TrackerCalibrationQualitySummary effectiveActiveQuality = active->quality;
    const bool candidateHasMeasuredGyro =
        (cq.qualityFlags & tracker_calibration_quality_flags::GYRO_MEASURED) != 0u;
    const bool activeHasMeasuredGyro =
        (effectiveActiveQuality.qualityFlags &
         tracker_calibration_quality_flags::GYRO_MEASURED) != 0u;
    const bool candidateHasMeasuredAccel =
        (cq.qualityFlags & tracker_calibration_quality_flags::ACCEL_MEASURED) != 0u;
    const bool activeHasMeasuredAccel =
        (effectiveActiveQuality.qualityFlags &
         tracker_calibration_quality_flags::ACCEL_MEASURED) != 0u;
    const bool candidateHasMeasuredAlignment =
        (cq.qualityFlags & tracker_calibration_quality_flags::ALIGNMENT_MEASURED) != 0u;
    const bool activeHasMeasuredAlignment =
        (effectiveActiveQuality.qualityFlags &
         tracker_calibration_quality_flags::ALIGNMENT_MEASURED) != 0u;
    if (candidateHasMeasuredAlignment && !activeHasMeasuredAlignment &&
        active->payload.magCal.axisAlignmentValid) {
        // Older active records only know that an axis matrix is valid and
        // therefore persisted a binary alignmentScore=1. That value is not a
        // measured fit score and cannot be compared to a gyro-assisted solve.
        // Treat the unmeasured-but-valid baseline as neutral; the runtime
        // solver separately proves pairwise improvement on the exact same
        // intervals before it is allowed to stage this candidate.
        effectiveActiveQuality.alignmentScore = 0.50f;
        trackerCalibrationQualityRecomputeOverall(active->payload, effectiveActiveQuality);
    }
    if (candidateHasMeasuredGyro && !activeHasMeasuredGyro &&
        active->payload.gyroCal.biasValid) {
        // Legacy active records have a derived 0.70/1.00 score, not a measured
        // held-out residual. The background learner already proves strict
        // per-axis and train/validation improvement before staging, so compare
        // the unmeasured baseline conservatively below the measured candidate.
        effectiveActiveQuality.gyroScore =
            std::max(0.0f, cq.gyroScore - 0.05f);
        trackerCalibrationQualityRecomputeOverall(active->payload, effectiveActiveQuality);
    }
    if (candidateHasMeasuredAccel && !activeHasMeasuredAccel &&
        active->payload.accelCal.valid) {
        effectiveActiveQuality.accelScore =
            std::max(0.0f, cq.accelScore - 0.05f);
        trackerCalibrationQualityRecomputeOverall(active->payload, effectiveActiveQuality);
    }
    const auto& aq = effectiveActiveQuality;
    if (qualityRegressed(cq.gyroScore, aq.gyroScore))
        flags |= tracker_calibration_comparison_flags::GYRO_REGRESSION;
    if (qualityRegressed(cq.accelScore, aq.accelScore))
        flags |= tracker_calibration_comparison_flags::ACCEL_REGRESSION;
    if (qualityRegressed(cq.magScore, aq.magScore))
        flags |= tracker_calibration_comparison_flags::MAG_REGRESSION;
    if (qualityRegressed(cq.alignmentScore, aq.alignmentScore))
        flags |= tracker_calibration_comparison_flags::ALIGNMENT_REGRESSION;
    if (qualityRegressed(cq.coverageScore, aq.coverageScore))
        flags |= tracker_calibration_comparison_flags::COVERAGE_REGRESSION;

    const float improvement = cq.overallScore - aq.overallScore;
    if (improvement < wearPolicy_.minQualityImprovement) {
        flags |= tracker_calibration_comparison_flags::BELOW_MIN_IMPROVEMENT;
    }

    const uint32_t regressionMask =
        tracker_calibration_comparison_flags::GYRO_REGRESSION |
        tracker_calibration_comparison_flags::ACCEL_REGRESSION |
        tracker_calibration_comparison_flags::MAG_REGRESSION |
        tracker_calibration_comparison_flags::ALIGNMENT_REGRESSION |
        tracker_calibration_comparison_flags::COVERAGE_REGRESSION;
    if ((flags & (regressionMask | tracker_calibration_comparison_flags::BELOW_MIN_IMPROVEMENT)) != 0u) {
        result = TrackerCalibrationComparisonResult::NotEnoughImprovement;
        return false;
    }
    result = TrackerCalibrationComparisonResult::Better;
    return true;
}

bool TrackerConfigStore::stageCandidate(const TrackerConfig& candidateInput,
                                        TrackerCalibrationCandidateMetadata metadata,
                                        uint32_t nowMs) {
#if !TRACKER_ENABLE_CALIBRATION_CANDIDATES
    (void)candidateInput;
    (void)metadata;
    (void)nowMs;
    lastError_ = TrackerConfigError::CandidateInvalid;
    return false;
#else
    if (storageDegradedLatched_) {
        ++degradedWriteBlocks_;
        lastError_ = TrackerConfigError::StorageDegraded;
        return false;
    }
    if (authoritativeApplyPending_) {
        ++applyPendingWriteBlocks_;
        lastError_ = TrackerConfigError::ApplyPending;
        return false;
    }
    auto scratch = makeScratch<CandidateStageScratch>(lastError_);
    if (!scratch) return false;
    TrackerConfig& candidate = scratch->candidate;
    candidate = candidateInput;
    candidate.sanitize();
    candidate.updateCrc();
    if (!candidate.validate()) {
        lastError_ = TrackerConfigError::CandidateInvalid;
        return false;
    }
    if (!trackerValidateCalibrationQuality(metadata.quality)) {
        metadata.quality = trackerCalibrationQualityFromConfig(candidate);
    }
    if (!trackerValidateCalibrationQuality(metadata.quality)) {
        lastError_ = TrackerConfigError::CandidateInvalid;
        return false;
    }

    TrackerCalibrationCandidateRecord& previous = scratch->previous;
    previous = TrackerCalibrationCandidateRecord{};
    bool previousExists = false;
    bool previousReadable = false;
    uint32_t previousGeneration = 0;
    uint32_t previousWrites = 0;
    if (ramCandidateValid_) {
        previousGeneration = ramCandidate_.candidateGeneration;
        previousWrites = ramCandidate_.persistedWriteCount;
    } else if (readCandidateFromNvs(previous, previousExists, previousReadable) &&
               previousExists && previousReadable && trackerValidateCalibrationCandidateRecord(previous)) {
        previousGeneration = previous.candidateGeneration;
        previousWrites = previous.persistedWriteCount;
    }

    TrackerConfigSlotRecord& active = scratch->active;
    active = TrackerConfigSlotRecord{};
    TrackerConfigSlot activeSlot = TrackerConfigSlot::None;
    TrackerConfigSelectorRecord& selector = scratch->selector;
    bool fallback = false;
    const bool hasActive = resolveActive(active, activeSlot, selector, fallback);
    if (!hasActive && lastError_ != TrackerConfigError::NotFound) return false;

    TrackerCalibrationCandidateRecord& staged = scratch->staged;
    staged = TrackerCalibrationCandidateRecord{};
    staged.version = tracker_config_storage_detail::CANDIDATE_VERSION;
    staged.candidateGeneration = nextGeneration(previousGeneration);
    staged.persistedWriteCount = previousWrites;
    staged.signature = trackerMakeSensorSignature(candidate);
    metadata.createdUptimeMs = nowMs;
    const uint32_t activeRevision = hasActive
        ? trackerCalibrationPayloadRevision(active.payload)
        : 0u;
    if (metadata.activeCalibrationRevisionAtCreation != 0u &&
        metadata.activeCalibrationRevisionAtCreation != activeRevision) {
        lastError_ = TrackerConfigError::CandidateStale;
        return false;
    }
    metadata.activeCalibrationRevisionAtCreation = activeRevision;
    staged.metadata = metadata;
    staged.payload = candidate.data;
    staged.crc32 = 0;
    staged.crc32 = trackerCalibrationCandidateRecordCrc(staged);

    TrackerCalibrationComparisonResult comparison = TrackerCalibrationComparisonResult::NotCompared;
    uint32_t flags = 0;
    (void)candidateQualityComparison(staged, hasActive ? &active : nullptr, comparison, flags);
    staged.metadata.lastComparison = comparison;
    staged.metadata.comparisonFlags = flags;
    staged.crc32 = 0;
    staged.crc32 = trackerCalibrationCandidateRecordCrc(staged);
    if (!trackerValidateCalibrationCandidateRecord(staged)) {
        lastError_ = TrackerConfigError::CandidateInvalid;
        return false;
    }

    ramCandidate_ = staged;
    ramCandidateValid_ = true;
    ramCandidateDirty_ = true;
    ++candidateStageCount_;
    if (comparison != TrackerCalibrationComparisonResult::Better) ++candidateRejectedCount_;
    lastError_ = TrackerConfigError::None;
    return true;
#endif
}

bool TrackerConfigStore::candidateDirty() const {
#if !TRACKER_ENABLE_CALIBRATION_CANDIDATES
    return false;
#else
    return ramCandidateDirty_;
#endif
}

bool TrackerConfigStore::candidateExists(bool& outExists) {
    outExists = false;
#if !TRACKER_ENABLE_CALIBRATION_CANDIDATES
    lastError_ = TrackerConfigError::None;
    return true;
#else
    if (ramCandidateValid_) {
        outExists = true;
        lastError_ = TrackerConfigError::None;
        return true;
    }
    Preferences prefs;
    if (!prefs.begin(ns_, true)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    outExists = prefs.isKey(candidateKey_);
    prefs.end();
    lastError_ = TrackerConfigError::None;
    return true;
#endif
}

bool TrackerConfigStore::flushCandidate(uint32_t nowMs, bool force) {
#if !TRACKER_ENABLE_CALIBRATION_CANDIDATES
    (void)nowMs;
    (void)force;
    lastError_ = TrackerConfigError::CandidateInvalid;
    return false;
#else
    if (storageDegradedLatched_) {
        ++degradedWriteBlocks_;
        lastError_ = TrackerConfigError::StorageDegraded;
        return false;
    }
    if (authoritativeApplyPending_) {
        ++applyPendingWriteBlocks_;
        lastError_ = TrackerConfigError::ApplyPending;
        return false;
    }
    if (!ramCandidateValid_ || !ramCandidateDirty_) {
        lastError_ = TrackerConfigError::NotFound;
        return false;
    }

    auto scratch = makeScratch<CandidateFlushScratch>(lastError_);
    if (!scratch) return false;
    TrackerCalibrationCandidateRecord& compared = scratch->compared;
    compared = ramCandidate_;
    TrackerCalibrationComparisonResult result = TrackerCalibrationComparisonResult::NotCompared;
    uint32_t flags = 0;
    TrackerConfigSlotRecord& active = scratch->active;
    active = TrackerConfigSlotRecord{};
    TrackerConfigSlot activeSlot = TrackerConfigSlot::None;
    TrackerConfigSelectorRecord selector{};
    bool fallback = false;
    const bool hasActive = resolveActive(active, activeSlot, selector, fallback);
    if (!hasActive && lastError_ != TrackerConfigError::NotFound) return false;
    const bool better = candidateQualityComparison(compared, hasActive ? &active : nullptr, result, flags);
    const bool promotedMetadataRetry =
        result == TrackerCalibrationComparisonResult::AlreadyPromoted && ramCandidateDirty_;
    if (promotedMetadataRetry) {
        compared.metadata.lastComparison = TrackerCalibrationComparisonResult::Promoted;
        compared.metadata.comparisonFlags = tracker_calibration_comparison_flags::NONE;
    } else {
        compared.metadata.lastComparison = result;
        compared.metadata.comparisonFlags = flags;
    }
    const bool hardReject = result == TrackerCalibrationComparisonResult::SignatureMismatch ||
                            result == TrackerCalibrationComparisonResult::StaleActiveGeneration ||
                            (result == TrackerCalibrationComparisonResult::AlreadyPromoted &&
                             !promotedMetadataRetry) ||
                            result == TrackerCalibrationComparisonResult::InvalidCandidate;
    if (hardReject || (!promotedMetadataRetry && !force && !better)) {
        ramCandidate_ = compared;
        ramCandidate_.crc32 = 0;
        ramCandidate_.crc32 = trackerCalibrationCandidateRecordCrc(ramCandidate_);
        ++candidateRejectedCount_;
        if (result == TrackerCalibrationComparisonResult::SignatureMismatch)
            lastError_ = TrackerConfigError::SignatureMismatch;
        else if (result == TrackerCalibrationComparisonResult::StaleActiveGeneration)
            lastError_ = TrackerConfigError::CandidateStale;
        else if (result == TrackerCalibrationComparisonResult::AlreadyPromoted)
            lastError_ = TrackerConfigError::CandidateAlreadyPromoted;
        else if (result == TrackerCalibrationComparisonResult::InvalidCandidate)
            lastError_ = TrackerConfigError::CandidateInvalid;
        else
            lastError_ = TrackerConfigError::CandidateNotBetter;
        return false;
    }

    if (!promotedMetadataRetry && !force && candidateWriteTimestampValid_ &&
        static_cast<uint32_t>(nowMs - lastCandidateWriteMs_) < wearPolicy_.minCandidateWriteIntervalMs) {
        ++candidateFlushThrottled_;
        lastError_ = TrackerConfigError::WriteThrottled;
        return false;
    }

    if (!promotedMetadataRetry) compared.persistedWriteCount += 1u;
    compared.crc32 = 0;
    compared.crc32 = trackerCalibrationCandidateRecordCrc(compared);
    if (!writeCandidateVerified(compared)) return false;

    ramCandidate_ = compared;
    ramCandidateValid_ = true;
    ramCandidateDirty_ = false;
    lastCandidateWriteMs_ = nowMs;
    candidateWriteTimestampValid_ = true;
    ++candidateFlushCount_;
    lastError_ = TrackerConfigError::None;
    return true;
#endif
}

bool TrackerConfigStore::loadCandidate(TrackerCalibrationCandidateRecord& out) {
#if !TRACKER_ENABLE_CALIBRATION_CANDIDATES
    (void)out;
    lastError_ = TrackerConfigError::CandidateInvalid;
    return false;
#else
    if (ramCandidateValid_) {
        out = ramCandidate_;
        lastError_ = TrackerConfigError::None;
        return true;
    }
    bool exists = false;
    bool readable = false;
    if (!readCandidateFromNvs(out, exists, readable)) return false;
    if (!exists) {
        lastError_ = TrackerConfigError::NotFound;
        return false;
    }
    if (!readable || !trackerValidateCalibrationCandidateRecord(out)) {
        lastError_ = TrackerConfigError::CandidateInvalid;
        return false;
    }
    ramCandidate_ = out;
    ramCandidateValid_ = true;
    ramCandidateDirty_ = false;
    // Persisted candidates receive a boot-time wear grace. Without this, a
    // reboot resets the interval and allows immediate repeated flash writes.
    lastCandidateWriteMs_ = 0;
    candidateWriteTimestampValid_ = true;
    lastError_ = TrackerConfigError::None;
    return true;
#endif
}

bool TrackerConfigStore::discardCandidate() {
    if (commitUncertainLatched_) {
        lastError_ = TrackerConfigError::CommitUncertain;
        return false;
    }
    if (storageDegradedLatched_) {
        ++degradedWriteBlocks_;
        lastError_ = TrackerConfigError::StorageDegraded;
        return false;
    }
    if (authoritativeApplyPending_) {
        ++applyPendingWriteBlocks_;
        lastError_ = TrackerConfigError::ApplyPending;
        return false;
    }
    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    const bool ok = removeIfPresent(prefs, candidateKey_);
    prefs.end();
    if (!ok) {
        lastError_ = TrackerConfigError::RemoveFailed;
        return false;
    }
#if TRACKER_ENABLE_CALIBRATION_CANDIDATES
    ramCandidate_ = TrackerCalibrationCandidateRecord{};
    ramCandidateValid_ = false;
    ramCandidateDirty_ = false;
#endif
    lastError_ = TrackerConfigError::None;
    return true;
}

bool TrackerConfigStore::compareCandidate(TrackerCalibrationCandidateRecord& outCandidate,
                                          TrackerCalibrationComparisonResult& outResult,
                                          uint32_t& outFlags) {
#if !TRACKER_ENABLE_CALIBRATION_CANDIDATES
    outCandidate = TrackerCalibrationCandidateRecord{};
    outResult = TrackerCalibrationComparisonResult::InvalidCandidate;
    outFlags = tracker_calibration_comparison_flags::QUALITY_NONFINITE;
    lastError_ = TrackerConfigError::CandidateInvalid;
    return false;
#else
    if (!loadCandidate(outCandidate)) return false;
    TrackerConfigSlotRecord active{};
    TrackerConfigSlot activeSlot = TrackerConfigSlot::None;
    TrackerConfigSelectorRecord selector{};
    bool fallback = false;
    const bool hasActive = resolveActive(active, activeSlot, selector, fallback);
    if (!hasActive && lastError_ != TrackerConfigError::NotFound) return false;
    const bool better = candidateQualityComparison(
        outCandidate,
        hasActive ? &active : nullptr,
        outResult,
        outFlags
    );
    const bool comparisonChanged =
        outCandidate.metadata.lastComparison != outResult ||
        outCandidate.metadata.comparisonFlags != outFlags;
    outCandidate.metadata.lastComparison = outResult;
    outCandidate.metadata.comparisonFlags = outFlags;
    outCandidate.crc32 = 0;
    outCandidate.crc32 = trackerCalibrationCandidateRecordCrc(outCandidate);
    ramCandidate_ = outCandidate;
    ramCandidateValid_ = true;
    ramCandidateDirty_ = ramCandidateDirty_ || comparisonChanged;
    lastError_ = better ? TrackerConfigError::None : TrackerConfigError::CandidateNotBetter;
    return true;
#endif
}

bool TrackerConfigStore::prepareCandidatePromotion(TrackerPreparedConfigPromotion& out,
                                                   TrackerConfig& outCandidate,
                                                   bool force,
                                                   const TrackerConfig* runtimeCalibration) {
#if !TRACKER_ENABLE_CALIBRATION_CANDIDATES
    (void)force;
    out = TrackerPreparedConfigPromotion{};
    outCandidate = TrackerConfig{};
    lastError_ = TrackerConfigError::CandidateInvalid;
    return false;
#else
    out = TrackerPreparedConfigPromotion{};
    if (storageDegradedLatched_) {
        ++degradedWriteBlocks_;
        lastError_ = TrackerConfigError::StorageDegraded;
        return false;
    }
    if (authoritativeApplyPending_) {
        ++applyPendingWriteBlocks_;
        lastError_ = TrackerConfigError::ApplyPending;
        return false;
    }
    auto scratch = makeScratch<PromotionScratch>(lastError_);
    if (!scratch) return false;
    TrackerCalibrationCandidateRecord& candidate = scratch->candidate;
    if (!loadCandidate(candidate)) return false;

    TrackerConfigSlotRecord& active = scratch->active;
    TrackerConfigSlot activeSlot = TrackerConfigSlot::None;
    TrackerConfigSelectorRecord& selector = scratch->selector;
    bool fallback = false;
    const bool hasActive = resolveActive(active, activeSlot, selector, fallback);
    if (!hasActive && lastError_ != TrackerConfigError::NotFound) return false;

    TrackerCalibrationComparisonResult result = TrackerCalibrationComparisonResult::NotCompared;
    uint32_t flags = 0;
    const bool better = candidateQualityComparison(candidate, hasActive ? &active : nullptr, result, flags);
    candidate.metadata.lastComparison = result;
    candidate.metadata.comparisonFlags = flags;
    candidate.crc32 = 0;
    candidate.crc32 = trackerCalibrationCandidateRecordCrc(candidate);
    ramCandidate_ = candidate;
    ramCandidateValid_ = true;
    ramCandidateDirty_ = true;

    if (result == TrackerCalibrationComparisonResult::SignatureMismatch) {
        lastError_ = TrackerConfigError::SignatureMismatch;
        return false;
    }
    if (result == TrackerCalibrationComparisonResult::StaleActiveGeneration) {
        lastError_ = TrackerConfigError::CandidateStale;
        return false;
    }
    if (result == TrackerCalibrationComparisonResult::AlreadyPromoted) {
        lastError_ = TrackerConfigError::CandidateAlreadyPromoted;
        return false;
    }
    if (result == TrackerCalibrationComparisonResult::InvalidCandidate) {
        lastError_ = TrackerConfigError::CandidateInvalid;
        return false;
    }
    if (!force && !better) {
        lastError_ = TrackerConfigError::CandidateNotBetter;
        return false;
    }

    TrackerConfig& candidateSnapshot = scratch->candidateSnapshot;
    candidateSnapshot.data = candidate.payload;
    TrackerConfig& composed = scratch->composedConfig;
    TrackerConfig& activeConfig = scratch->activeConfig;
    if (hasActive) {
        activeConfig.data = active.payload;
        // Compose directly into the heap-backed promotion workspace. Returning
        // TrackerConfig by value here creates a 756-byte ABI-dependent return
        // temporary on some host/embedded compilers and can exceed the bounded
        // promotion stack budget even though the persistent records are already
        // off-stack.
        composed = activeConfig;
        trackerApplyCalibrationCandidateToConfig(composed, candidateSnapshot);
    } else {
        composed = candidateSnapshot;
        composed.sanitize();
        composed.updateCrc();
    }

    if (runtimeCalibration && hasActive &&
        !trackerSensorSignaturesEqual(trackerMakeSensorSignature(*runtimeCalibration), active.signature)) {
        // Promotion is calibration-only and deliberately avoids IMU/FIFO
        // reconfiguration. A live unsaved ODR/full-scale/FIFO/frame contract
        // must therefore be saved or rolled back before promotion.
        lastError_ = TrackerConfigError::RuntimeSensorSignatureDiverged;
        return false;
    }

    if (runtimeCalibration && hasActive &&
        !trackerCalibrationModelEqual(*runtimeCalibration, activeConfig) &&
        !trackerCalibrationModelEqual(*runtimeCalibration, composed)) {
        // The caller has a third, unsaved calibration model in RAM. Promoting
        // either persisted model would silently destroy it, so require the
        // user/background owner to save or discard that runtime work first.
        lastError_ = TrackerConfigError::RuntimeCalibrationDiverged;
        return false;
    }

    const TrackerConfigSlot target = hasActive ? otherSlot(activeSlot) : TrackerConfigSlot::A;
    TrackerCalibrationQualitySummary& promotedQuality = scratch->promotedQuality;
    promotedQuality = candidate.metadata.quality;
    trackerCalibrationQualitySetProvenance(promotedQuality, candidate.metadata.provenance);
    if (!saveInternal(composed, false, true, target, false, &out, &promotedQuality,
                      candidate.metadata.provenance)) {
        return false;
    }
    out.previousSelector = selector;
    out.activeGenerationAtPreparation = hasActive ? active.generation : 0u;
    outCandidate = composed;
    lastError_ = TrackerConfigError::None;
    return true;
#endif
}

bool TrackerConfigStore::commitPreparedPromotion(TrackerPreparedConfigPromotion& prepared,
                                                 TrackerConfig& outActive) {
#if !TRACKER_ENABLE_CALIBRATION_CANDIDATES
    (void)prepared;
    (void)outActive;
    lastError_ = TrackerConfigError::CandidateInvalid;
    return false;
#else
    if (commitUncertainLatched_) {
        lastError_ = TrackerConfigError::CommitUncertain;
        return false;
    }
    if (storageDegradedLatched_) {
        ++degradedWriteBlocks_;
        lastError_ = TrackerConfigError::StorageDegraded;
        return false;
    }
    if (authoritativeApplyPending_) {
        ++applyPendingWriteBlocks_;
        lastError_ = TrackerConfigError::ApplyPending;
        return false;
    }
    if (!prepared.valid || prepared.targetSlot == TrackerConfigSlot::None || prepared.targetGeneration == 0) {
        lastError_ = TrackerConfigError::PromotionNotPrepared;
        return false;
    }

    auto target = makeScratch<TrackerConfigSlotRecord>(lastError_);
    if (!target) return false;
    bool exists = false;
    bool readable = false;
    if (!readSlot(prepared.targetSlot, *target, exists, readable) || !exists || !readable ||
        !trackerValidateConfigSlotRecord(*target) ||
        target->generation != prepared.targetGeneration) {
        lastError_ = TrackerConfigError::ReadFailed;
        return false;
    }

    // The active generation may not change between prepare and commit.
    auto current = makeScratch<TrackerConfigSlotRecord>(lastError_);
    if (!current) return false;
    TrackerConfigSlot currentSlot = TrackerConfigSlot::None;
    TrackerConfigSelectorRecord currentSelector{};
    bool fallback = false;
    if (prepared.activeGenerationAtPreparation != 0u) {
        if (!resolveActive(*current, currentSlot, currentSelector, fallback) ||
            current->generation != prepared.activeGenerationAtPreparation) {
            lastError_ = TrackerConfigError::CandidateStale;
            return false;
        }
    }

    TrackerConfigSelectorRecord selector = prepared.previousSelector;
    selector.activeSlot = prepared.targetSlot;
    selector.activeGeneration = prepared.targetGeneration;
    selector.successfulActiveWrites += 1u;
    selector.successfulPromotions += 1u;
    selector.crc32 = 0;
    selector.crc32 = trackerConfigSelectorRecordCrc(selector);
    if (!writeSelectorVerified(selector)) {
        bool committed = false;
        const TrackerConfigError initialError = lastError_;
        if (!reconcileSelectorCommit(selector, prepared.previousSelector, committed)) return false;
        if (!committed) {
            lastError_ = initialError;
            return false;
        }
    }

    const TrackerConfigError beforeMarker = lastError_;
    if (!writeCommitVerified(prepared.targetSlot, prepared.targetGeneration)) {
        ++commitMarkerRepairFailures_;
        lastError_ = beforeMarker;
    }

    outActive.data = target->payload;
    prepared.valid = false;
    if (ramCandidateValid_) {
        const bool wasPersisted = ramCandidate_.persistedWriteCount != 0u;
        ramCandidate_.version = tracker_config_storage_detail::CANDIDATE_VERSION;
        ramCandidate_.metadata.activeCalibrationRevisionAtCreation =
            trackerCalibrationPayloadRevision(target->payload);
        ramCandidate_.metadata.lastComparison = TrackerCalibrationComparisonResult::Promoted;
        ramCandidate_.metadata.comparisonFlags = tracker_calibration_comparison_flags::NONE;
        if (wasPersisted) ++ramCandidate_.persistedWriteCount;
        ramCandidate_.crc32 = 0;
        ramCandidate_.crc32 = trackerCalibrationCandidateRecordCrc(ramCandidate_);
        ramCandidateDirty_ = wasPersisted;
        if (wasPersisted) {
            const TrackerConfigError beforeCandidateUpdate = lastError_;
            if (writeCandidateVerified(ramCandidate_)) {
                ++candidatePromotionStateWrites_;
                ramCandidateDirty_ = false;
            } else {
                ++candidatePromotionStateWriteFailures_;
                lastError_ = beforeCandidateUpdate;
            }
        }
    }
    lastError_ = TrackerConfigError::None;
    return true;
#endif
}

bool TrackerConfigStore::restoreAuthoritativeGeneration(TrackerConfigSlot slot,
                                                        uint32_t generation,
                                                        TrackerConfig& outActive) {
    if (slot == TrackerConfigSlot::None || generation == 0u) {
        lastError_ = TrackerConfigError::SelectorInvalid;
        return false;
    }
    // Exact rollback is the recovery operation for an uncertain promotion.
    // It is allowed to reconcile a latched selector uncertainty, but still
    // remains blocked by independently degraded storage.
    if (storageDegradedLatched_) {
        ++degradedWriteBlocks_;
        lastError_ = TrackerConfigError::StorageDegraded;
        return false;
    }

    TrackerConfigSlotRecord record{};
    bool exists = false;
    bool readable = false;
    if (!readSlot(slot, record, exists, readable) || !exists || !readable ||
        !trackerValidateConfigSlotRecord(record) || record.generation != generation) {
        lastError_ = TrackerConfigError::ReadFailed;
        return false;
    }
    TrackerConfigCommitRecord commit{};
    bool commitExists = false;
    bool commitReadable = false;
    if (!readCommit(slot, commit, commitExists, commitReadable) ||
        !slotHasCommitAuthority(slot, record, commit, commitExists, commitReadable)) {
        lastError_ = TrackerConfigError::SelectorInvalid;
        return false;
    }

    TrackerConfigSelectorRecord current{};
    bool selectorExists = false;
    bool selectorReadable = false;
    if (!readSelector(current, selectorExists, selectorReadable) ||
        !selectorExists || !selectorReadable || !trackerValidateConfigSelectorRecord(current)) {
        lastError_ = TrackerConfigError::SelectorInvalid;
        return false;
    }
    TrackerConfigSelectorRecord intended = current;
    intended.activeSlot = slot;
    intended.activeGeneration = generation;
    intended.successfulActiveWrites += 1u;
    intended.crc32 = 0u;
    intended.crc32 = trackerConfigSelectorRecordCrc(intended);
    if (!writeSelectorVerified(intended)) {
        bool committed = false;
        const TrackerConfigError initialError = lastError_;
        if (!reconcileSelectorCommit(intended, current, committed) || !committed) {
            lastError_ = initialError;
            return false;
        }
    }

    outActive.data = record.payload;
    outActive.sanitize();
    outActive.updateCrc();
    authoritativeApplyPending_ = true;
    lastError_ = TrackerConfigError::None;
    return true;
}

void TrackerConfigStore::abortPreparedPromotion(TrackerPreparedConfigPromotion& prepared) {
    const TrackerConfigError before = lastError_;
    if (!invalidatePreparedSlot(prepared)) {
        // Resolver rules still prevent a newer uncommitted slot from becoming
        // fallback. Keep the cleanup error visible for diagnostics.
        prepared.valid = false;
        return;
    }
    prepared.valid = false;
    lastError_ = before == TrackerConfigError::None ? TrackerConfigError::None : before;
}

} // namespace tracker
