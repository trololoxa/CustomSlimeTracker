#include "runtime/calibration_autonomy_store.hpp"

#include <cstring>
#include <memory>
#include <new>

namespace tracker {

CalibrationAutonomyStore::CalibrationAutonomyStore(const char* nvsNamespace)
    : ns_(nvsNamespace) {}

template <typename T>
uint32_t CalibrationAutonomyStore::crcRecord(const T& record, size_t crcOffset) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(&record);
    uint32_t h = 2166136261UL;
    for (size_t i = 0; i < sizeof(T); ++i) {
        uint8_t value = bytes[i];
        if (i >= crcOffset && i < crcOffset + sizeof(uint32_t)) value = 0;
        h ^= static_cast<uint32_t>(value);
        h *= 16777619UL;
    }
    return h;
}

uint32_t CalibrationAutonomyStore::journalCrc(const CalibrationAutonomyJournalRecord& record) {
    return crcRecord(record, offsetof(CalibrationAutonomyJournalRecord, crc32));
}

uint32_t CalibrationAutonomyStore::legacyJournalV1Crc(
    const CalibrationAutonomyJournalRecordV1& record) {
    return crcRecord(record, offsetof(CalibrationAutonomyJournalRecordV1, crc32));
}

uint32_t CalibrationAutonomyStore::preferencesCrc(const CalibrationAutonomyPreferencesRecord& record) {
    return crcRecord(record, offsetof(CalibrationAutonomyPreferencesRecord, crc32));
}

uint32_t CalibrationAutonomyStore::rejectionCrc(const CalibrationAutonomyRejectionRecord& record) {
    return crcRecord(record, offsetof(CalibrationAutonomyRejectionRecord, crc32));
}

uint32_t CalibrationAutonomyStore::eraseRecoveryCrc(
    const CalibrationAutonomyEraseRecoveryRecord& record) {
    return crcRecord(record, offsetof(CalibrationAutonomyEraseRecoveryRecord, crc32));
}

bool CalibrationAutonomyStore::valid(const CalibrationAutonomyJournalRecord& record) {
    return record.magic == calibration_autonomy_storage_detail::JOURNAL_MAGIC &&
           record.version == calibration_autonomy_storage_detail::JOURNAL_VERSION &&
           record.size == sizeof(record) &&
           record.crc32 == journalCrc(record);
}

bool CalibrationAutonomyStore::valid(const CalibrationAutonomyJournalRecordV1& record) {
    return record.magic == calibration_autonomy_storage_detail::JOURNAL_MAGIC &&
           record.version == 1u &&
           record.size == sizeof(record) &&
           record.crc32 == legacyJournalV1Crc(record);
}

bool CalibrationAutonomyStore::valid(const CalibrationAutonomyPreferencesRecord& record) {
    return record.magic == calibration_autonomy_storage_detail::PREFS_MAGIC &&
           record.version == calibration_autonomy_storage_detail::PREFS_VERSION &&
           record.size == sizeof(record) &&
           record.wave0022Enabled <= 1u && record.wave0023Enabled <= 1u &&
           record.crc32 == preferencesCrc(record);
}

bool CalibrationAutonomyStore::valid(const CalibrationAutonomyRejectionRecord& record) {
    return record.magic == calibration_autonomy_storage_detail::REJECT_MAGIC &&
           record.version == calibration_autonomy_storage_detail::REJECT_VERSION &&
           record.size == sizeof(record) &&
           record.crc32 == rejectionCrc(record);
}

bool CalibrationAutonomyStore::valid(const CalibrationAutonomyEraseRecoveryRecord& record) {
    return record.magic == calibration_autonomy_storage_detail::ERASE_MAGIC &&
           record.version == calibration_autonomy_storage_detail::ERASE_VERSION &&
           record.size == sizeof(record) &&
           record.wave0022Enabled <= 1u && record.wave0023Enabled <= 1u &&
           record.crc32 == eraseRecoveryCrc(record);
}

bool CalibrationAutonomyStore::sequenceNewer(uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a - b) > 0;
}

template <typename T>
bool CalibrationAutonomyStore::readRecord(Preferences& prefs, const char* key, T& out) {
    if (!prefs.isKey(key)) return false;
    if (prefs.getBytesLength(key) != sizeof(T)) return false;
    return prefs.getBytes(key, &out, sizeof(T)) == sizeof(T);
}

template <typename T>
bool CalibrationAutonomyStore::writeRecordVerified(Preferences& prefs,
                                                     const char* key,
                                                     const T& record,
                                                     T& verify) {
    if (prefs.putBytes(key, &record, sizeof(T)) != sizeof(T)) return false;
    verify = T{};
    if (!readRecord(prefs, key, verify)) return false;
    return std::memcmp(&record, &verify, sizeof(T)) == 0;
}

bool CalibrationAutonomyStore::loadPreferences(CalibrationAutonomyPreferencesRecord& out) {
    Preferences prefs;
    if (!prefs.begin(ns_, true)) {
        lastError_ = "nvs_begin_failed";
        return false;
    }
    const bool read = readRecord(prefs, calibration_autonomy_storage_detail::KEY_PREFERENCES,
                                 prefsScratch_);
    prefs.end();
    if (!read) {
        out = CalibrationAutonomyPreferencesRecord{};
        lastError_ = "not_found";
        return false;
    }
    if (!valid(prefsScratch_)) {
        lastError_ = "invalid_preferences";
        return false;
    }
    out = prefsScratch_;
    lastError_ = "none";
    return true;
}

bool CalibrationAutonomyStore::savePreferences(bool wave0022Enabled,
                                                bool wave0023Enabled) {
    prefsVerify_ = CalibrationAutonomyPreferencesRecord{};
    (void)loadPreferences(prefsVerify_);
    prefsScratch_ = CalibrationAutonomyPreferencesRecord{};
    prefsScratch_.sequence = prefsVerify_.sequence + 1u;
    if (prefsScratch_.sequence == 0u) prefsScratch_.sequence = 1u;
    prefsScratch_.wave0022Enabled = wave0022Enabled ? 1u : 0u;
    prefsScratch_.wave0023Enabled = wave0023Enabled ? 1u : 0u;
    prefsScratch_.crc32 = preferencesCrc(prefsScratch_);

    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = "nvs_begin_failed";
        return false;
    }
    const bool ok = writeRecordVerified(
        prefs, calibration_autonomy_storage_detail::KEY_PREFERENCES,
        prefsScratch_, prefsVerify_);
    prefs.end();
    lastError_ = ok ? "none" : "write_verify_failed";
    return ok;
}

bool CalibrationAutonomyStore::loadJournal(CalibrationAutonomyJournalRecord& out) {
    Preferences prefs;
    if (!prefs.begin(ns_, true)) {
        lastError_ = "nvs_begin_failed";
        return false;
    }
    const bool presentA = prefs.isKey(calibration_autonomy_storage_detail::KEY_JOURNAL_A);
    const bool presentB = prefs.isKey(calibration_autonomy_storage_detail::KEY_JOURNAL_B);
    const bool readA = readRecord(prefs, calibration_autonomy_storage_detail::KEY_JOURNAL_A,
                                  journalA_);
    const bool readB = readRecord(prefs, calibration_autonomy_storage_detail::KEY_JOURNAL_B,
                                  journalB_);
    prefs.end();
    const bool validA = readA && valid(journalA_);
    const bool validB = readB && valid(journalB_);
    if (!validA && !validB) {
        out = CalibrationAutonomyJournalRecord{};
        lastError_ = (presentA || presentB) ? "invalid_journal" : "not_found";
        return false;
    }
    out = validA && (!validB || sequenceNewer(journalA_.sequence, journalB_.sequence))
        ? journalA_ : journalB_;
    lastError_ = "none";
    return true;
}

bool CalibrationAutonomyStore::loadLegacyJournalV1(
    CalibrationAutonomyJournalRecordV1& out) {
    Preferences prefs;
    if (!prefs.begin(ns_, true)) {
        lastError_ = "nvs_begin_failed";
        return false;
    }
    const bool presentA = prefs.isKey(calibration_autonomy_storage_detail::KEY_JOURNAL_A);
    const bool presentB = prefs.isKey(calibration_autonomy_storage_detail::KEY_JOURNAL_B);
    const bool readA = readRecord(prefs, calibration_autonomy_storage_detail::KEY_JOURNAL_A,
                                  legacyJournalA_);
    const bool readB = readRecord(prefs, calibration_autonomy_storage_detail::KEY_JOURNAL_B,
                                  legacyJournalB_);
    prefs.end();
    const bool validA = readA && valid(legacyJournalA_);
    const bool validB = readB && valid(legacyJournalB_);
    if (!validA && !validB) {
        out = CalibrationAutonomyJournalRecordV1{};
        lastError_ = (presentA || presentB) ? "invalid_journal" : "not_found";
        return false;
    }
    out = validA && (!validB || sequenceNewer(legacyJournalA_.sequence, legacyJournalB_.sequence))
        ? legacyJournalA_ : legacyJournalB_;
    lastError_ = "legacy_journal_v1";
    return true;
}

bool CalibrationAutonomyStore::writeJournal(CalibrationAutonomyJournalRecord& input) {
    journalCurrent_ = CalibrationAutonomyJournalRecord{};
    const bool haveCurrent = loadJournal(journalCurrent_);
    journalVerify_ = input;
    journalVerify_.magic = calibration_autonomy_storage_detail::JOURNAL_MAGIC;
    journalVerify_.version = calibration_autonomy_storage_detail::JOURNAL_VERSION;
    journalVerify_.size = sizeof(journalVerify_);
    journalVerify_.sequence = haveCurrent ? journalCurrent_.sequence + 1u : 1u;
    if (journalVerify_.sequence == 0u) journalVerify_.sequence = 1u;
    journalVerify_.crc32 = journalCrc(journalVerify_);
    const char* key = (journalVerify_.sequence & 1u)
        ? calibration_autonomy_storage_detail::KEY_JOURNAL_A
        : calibration_autonomy_storage_detail::KEY_JOURNAL_B;

    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = "nvs_begin_failed";
        return false;
    }
    const bool ok = writeRecordVerified(prefs, key, journalVerify_, journalCurrent_);
    prefs.end();
    if (ok) input = journalVerify_;
    lastError_ = ok ? "none" : "write_verify_failed";
    return ok;
}

bool CalibrationAutonomyStore::clearJournal() {
    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = "nvs_begin_failed";
        return false;
    }
    bool ok = true;
    if (prefs.isKey(calibration_autonomy_storage_detail::KEY_JOURNAL_A)) {
        ok = prefs.remove(calibration_autonomy_storage_detail::KEY_JOURNAL_A) && ok;
    }
    if (prefs.isKey(calibration_autonomy_storage_detail::KEY_JOURNAL_B)) {
        ok = prefs.remove(calibration_autonomy_storage_detail::KEY_JOURNAL_B) && ok;
    }
    prefs.end();
    lastError_ = ok ? "none" : "remove_failed";
    return ok;
}

bool CalibrationAutonomyStore::loadRejection(CalibrationAutonomyRejectionRecord& out) {
    Preferences prefs;
    if (!prefs.begin(ns_, true)) {
        lastError_ = "nvs_begin_failed";
        return false;
    }
    const bool read = readRecord(prefs, calibration_autonomy_storage_detail::KEY_REJECTION,
                                 rejectionScratch_);
    prefs.end();
    if (!read) {
        out = CalibrationAutonomyRejectionRecord{};
        lastError_ = "not_found";
        return false;
    }
    if (!valid(rejectionScratch_)) {
        lastError_ = "invalid_rejection";
        return false;
    }
    out = rejectionScratch_;
    lastError_ = "none";
    return true;
}

bool CalibrationAutonomyStore::writeRejection(CalibrationAutonomyRejectionRecord& input) {
    rejectionVerify_ = CalibrationAutonomyRejectionRecord{};
    (void)loadRejection(rejectionVerify_);
    rejectionScratch_ = input;
    rejectionScratch_.magic = calibration_autonomy_storage_detail::REJECT_MAGIC;
    rejectionScratch_.version = calibration_autonomy_storage_detail::REJECT_VERSION;
    rejectionScratch_.size = sizeof(rejectionScratch_);
    rejectionScratch_.sequence = rejectionVerify_.sequence + 1u;
    if (rejectionScratch_.sequence == 0u) rejectionScratch_.sequence = 1u;
    rejectionScratch_.crc32 = rejectionCrc(rejectionScratch_);

    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = "nvs_begin_failed";
        return false;
    }
    const bool ok = writeRecordVerified(
        prefs, calibration_autonomy_storage_detail::KEY_REJECTION,
        rejectionScratch_, rejectionVerify_);
    prefs.end();
    if (ok) input = rejectionVerify_;
    lastError_ = ok ? "none" : "write_verify_failed";
    return ok;
}

bool CalibrationAutonomyStore::clearRejection() {
    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = "nvs_begin_failed";
        return false;
    }
    bool ok = true;
    if (prefs.isKey(calibration_autonomy_storage_detail::KEY_REJECTION)) {
        ok = prefs.remove(calibration_autonomy_storage_detail::KEY_REJECTION);
    }
    prefs.end();
    lastError_ = ok ? "none" : "remove_failed";
    return ok;
}

bool CalibrationAutonomyStore::loadEraseRecovery(
    CalibrationAutonomyEraseRecoveryRecord& out) {
    auto scratch = std::unique_ptr<CalibrationAutonomyEraseRecoveryRecord>(
        new (std::nothrow) CalibrationAutonomyEraseRecoveryRecord{});
    if (!scratch) {
        lastError_ = "out_of_memory";
        return false;
    }
    Preferences prefs;
    if (!prefs.begin(ns_, true)) {
        lastError_ = "nvs_begin_failed";
        return false;
    }
    const bool present = prefs.isKey(calibration_autonomy_storage_detail::KEY_ERASE_RECOVERY);
    const bool read = readRecord(
        prefs, calibration_autonomy_storage_detail::KEY_ERASE_RECOVERY, *scratch);
    prefs.end();
    if (!read || !valid(*scratch)) {
        out = CalibrationAutonomyEraseRecoveryRecord{};
        lastError_ = present ? "invalid_erase_recovery" : "not_found";
        return false;
    }
    out = *scratch;
    lastError_ = "none";
    return true;
}

bool CalibrationAutonomyStore::writeEraseRecovery(
    const TrackerConfig& cleanConfig, bool wave0022Enabled, bool wave0023Enabled) {
    auto record = std::unique_ptr<CalibrationAutonomyEraseRecoveryRecord>(
        new (std::nothrow) CalibrationAutonomyEraseRecoveryRecord{});
    auto verify = std::unique_ptr<CalibrationAutonomyEraseRecoveryRecord>(
        new (std::nothrow) CalibrationAutonomyEraseRecoveryRecord{});
    if (!record || !verify) {
        lastError_ = "out_of_memory";
        return false;
    }
    record->wave0022Enabled = wave0022Enabled ? 1u : 0u;
    record->wave0023Enabled = wave0023Enabled ? 1u : 0u;
    record->cleanPayload = cleanConfig.data;
    record->crc32 = eraseRecoveryCrc(*record);
    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = "nvs_begin_failed";
        return false;
    }
    const bool ok = writeRecordVerified(
        prefs, calibration_autonomy_storage_detail::KEY_ERASE_RECOVERY, *record, *verify);
    prefs.end();
    lastError_ = ok ? "none" : "write_verify_failed";
    return ok;
}

bool CalibrationAutonomyStore::clearEraseRecovery() {
    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = "nvs_begin_failed";
        return false;
    }
    bool ok = true;
    if (prefs.isKey(calibration_autonomy_storage_detail::KEY_ERASE_RECOVERY)) {
        ok = prefs.remove(calibration_autonomy_storage_detail::KEY_ERASE_RECOVERY);
    }
    prefs.end();
    lastError_ = ok ? "none" : "remove_failed";
    return ok;
}

const char* calibrationAutonomySubsystemName(CalibrationAutonomySubsystem subsystem) {
    switch (subsystem) {
        case CalibrationAutonomySubsystem::None: return "none";
        case CalibrationAutonomySubsystem::MagToImu: return "mag_to_imu";
        case CalibrationAutonomySubsystem::GyroBias: return "gyro_bias";
        case CalibrationAutonomySubsystem::GyroTemperature: return "gyro_temperature";
        case CalibrationAutonomySubsystem::Accelerometer: return "accelerometer";
    }
    return "unknown";
}

const char* calibrationAutonomyJournalStateName(CalibrationAutonomyJournalState state) {
    switch (state) {
        case CalibrationAutonomyJournalState::Empty: return "empty";
        case CalibrationAutonomyJournalState::PromotionPending: return "promotion_pending";
        case CalibrationAutonomyJournalState::Probation: return "probation";
        case CalibrationAutonomyJournalState::AcceptPending: return "accept_pending";
        case CalibrationAutonomyJournalState::RollbackPending: return "rollback_pending";
    }
    return "unknown";
}

} // namespace tracker
