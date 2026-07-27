#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <Preferences.h>

#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_storage.hpp"

namespace tracker {

enum class CalibrationAutonomySubsystem : uint8_t {
    None = 0,
    MagToImu = 1,
    GyroBias = 2,
    GyroTemperature = 3,
    Accelerometer = 4,
};

enum class CalibrationAutonomyJournalState : uint8_t {
    Empty = 0,
    PromotionPending = 1,
    Probation = 2,
    AcceptPending = 3,
    RollbackPending = 4,
};

namespace calibration_autonomy_storage_detail {
static constexpr const char* NVS_NAMESPACE = "cal_auto";
static constexpr const char* KEY_JOURNAL_A = "journal_a";
static constexpr const char* KEY_JOURNAL_B = "journal_b";
static constexpr const char* KEY_PREFERENCES = "prefs";
static constexpr const char* KEY_REJECTION = "reject";
static constexpr const char* KEY_ERASE_RECOVERY = "erase_pending";
static constexpr uint32_t JOURNAL_MAGIC = 0x4A413323UL; // '#3AJ'
static constexpr uint16_t JOURNAL_VERSION = 2;
static constexpr uint32_t PREFS_MAGIC = 0x50413323UL;   // '#3AP'
static constexpr uint16_t PREFS_VERSION = 1;
static constexpr uint32_t REJECT_MAGIC = 0x52413323UL;  // '#3AR'
static constexpr uint16_t REJECT_VERSION = 1;
static constexpr uint32_t ERASE_MAGIC = 0x45413323UL;   // '#3AE'
static constexpr uint16_t ERASE_VERSION = 1;
}

struct CalibrationAutonomyPreferencesRecord {
    uint32_t magic = calibration_autonomy_storage_detail::PREFS_MAGIC;
    uint16_t version = calibration_autonomy_storage_detail::PREFS_VERSION;
    uint16_t size = sizeof(CalibrationAutonomyPreferencesRecord);
    uint32_t sequence = 0;
    uint8_t wave0022Enabled = 1;
    uint8_t wave0023Enabled = 1;
    uint8_t reserved[2] = {};
    uint32_t crc32 = 0;
};

struct CalibrationAutonomyRejectionRecord {
    uint32_t magic = calibration_autonomy_storage_detail::REJECT_MAGIC;
    uint16_t version = calibration_autonomy_storage_detail::REJECT_VERSION;
    uint16_t size = sizeof(CalibrationAutonomyRejectionRecord);
    uint32_t sequence = 0;
    CalibrationAutonomySubsystem subsystem = CalibrationAutonomySubsystem::None;
    uint8_t reason = 0;
    uint16_t reserved = 0;
    uint32_t candidateFingerprint = 0;
    uint32_t activeCalibrationRevision = 0;
    uint32_t rejectedUptimeMs = 0;
    uint32_t crc32 = 0;
};

// This record is deliberately independent from TrackerConfig schema 2 and
// candidate format 3. It stores a write-ahead rollback anchor for the rare
// promotion transaction, without changing or migrating the main config blob.


struct CalibrationAutonomyEraseRecoveryRecord {
    uint32_t magic = calibration_autonomy_storage_detail::ERASE_MAGIC;
    uint16_t version = calibration_autonomy_storage_detail::ERASE_VERSION;
    uint16_t size = sizeof(CalibrationAutonomyEraseRecoveryRecord);
    uint32_t sequence = 1u;
    uint8_t wave0022Enabled = 0u;
    uint8_t wave0023Enabled = 0u;
    uint8_t reserved[2] = {};
    TrackerConfigBlob cleanPayload;
    uint32_t crc32 = 0u;
};

// Read-only compatibility layout written by the first 0023 release. 0023a
// added exact slot/generation rollback anchors, so v1 records must never be
// treated as current v2 records. They are used only for conservative rollback
// during upgrade and are then removed.
struct CalibrationAutonomyJournalRecordV1 {
    uint32_t magic = calibration_autonomy_storage_detail::JOURNAL_MAGIC;
    uint16_t version = 1u;
    uint16_t size = sizeof(CalibrationAutonomyJournalRecordV1);
    uint32_t sequence = 0;
    CalibrationAutonomyJournalState state = CalibrationAutonomyJournalState::Empty;
    CalibrationAutonomySubsystem subsystem = CalibrationAutonomySubsystem::None;
    uint16_t reserved = 0;
    uint32_t previousCalibrationRevision = 0;
    uint32_t targetCalibrationRevision = 0;
    uint32_t candidateFingerprint = 0;
    uint32_t transactionStartedUptimeMs = 0;
    uint32_t probationAcceptedWindows = 0;
    TrackerConfigBlob previousPayload;
    uint32_t crc32 = 0;
};

struct CalibrationAutonomyJournalRecord {
    uint32_t magic = calibration_autonomy_storage_detail::JOURNAL_MAGIC;
    uint16_t version = calibration_autonomy_storage_detail::JOURNAL_VERSION;
    uint16_t size = sizeof(CalibrationAutonomyJournalRecord);
    uint32_t sequence = 0;
    CalibrationAutonomyJournalState state = CalibrationAutonomyJournalState::Empty;
    CalibrationAutonomySubsystem subsystem = CalibrationAutonomySubsystem::None;
    uint16_t reserved = 0;
    TrackerConfigSlot previousSlot = TrackerConfigSlot::None;
    uint8_t reservedSlot[3] = {};
    uint32_t previousGeneration = 0;
    uint32_t previousCalibrationRevision = 0;
    uint32_t targetCalibrationRevision = 0;
    uint32_t candidateFingerprint = 0;
    uint32_t transactionStartedUptimeMs = 0;
    uint32_t probationAcceptedWindows = 0;
    TrackerConfigBlob previousPayload;
    uint32_t crc32 = 0;
};

class CalibrationAutonomyStore {
public:
    explicit CalibrationAutonomyStore(
        const char* nvsNamespace = calibration_autonomy_storage_detail::NVS_NAMESPACE);

    bool loadPreferences(CalibrationAutonomyPreferencesRecord& out);
    bool savePreferences(bool wave0022Enabled, bool wave0023Enabled);

    bool loadJournal(CalibrationAutonomyJournalRecord& out);
    bool loadLegacyJournalV1(CalibrationAutonomyJournalRecordV1& out);
    bool writeJournal(CalibrationAutonomyJournalRecord& record);
    bool clearJournal();

    bool loadRejection(CalibrationAutonomyRejectionRecord& out);
    bool writeRejection(CalibrationAutonomyRejectionRecord& record);
    bool clearRejection();

    bool loadEraseRecovery(CalibrationAutonomyEraseRecoveryRecord& out);
    bool writeEraseRecovery(const TrackerConfig& cleanConfig,
                            bool wave0022Enabled,
                            bool wave0023Enabled);
    bool clearEraseRecovery();

    const char* lastErrorName() const { return lastError_; }
    bool lastErrorIsNotFound() const { return std::strcmp(lastError_, "not_found") == 0; }

    static uint32_t journalCrc(const CalibrationAutonomyJournalRecord& record);
    static uint32_t legacyJournalV1Crc(const CalibrationAutonomyJournalRecordV1& record);
    static uint32_t preferencesCrc(const CalibrationAutonomyPreferencesRecord& record);
    static uint32_t rejectionCrc(const CalibrationAutonomyRejectionRecord& record);
    static uint32_t eraseRecoveryCrc(const CalibrationAutonomyEraseRecoveryRecord& record);
    static bool valid(const CalibrationAutonomyJournalRecord& record);
    static bool valid(const CalibrationAutonomyJournalRecordV1& record);
    static bool valid(const CalibrationAutonomyPreferencesRecord& record);
    static bool valid(const CalibrationAutonomyRejectionRecord& record);
    static bool valid(const CalibrationAutonomyEraseRecoveryRecord& record);

private:
    const char* ns_;
    const char* lastError_ = "none";
    CalibrationAutonomyJournalRecord journalA_{};
    CalibrationAutonomyJournalRecord journalB_{};
    CalibrationAutonomyJournalRecord journalCurrent_{};
    CalibrationAutonomyJournalRecord journalVerify_{};
    CalibrationAutonomyJournalRecordV1 legacyJournalA_{};
    CalibrationAutonomyJournalRecordV1 legacyJournalB_{};
    CalibrationAutonomyPreferencesRecord prefsScratch_{};
    CalibrationAutonomyPreferencesRecord prefsVerify_{};
    CalibrationAutonomyRejectionRecord rejectionScratch_{};
    CalibrationAutonomyRejectionRecord rejectionVerify_{};

    static bool sequenceNewer(uint32_t a, uint32_t b);
    template <typename T>
    static uint32_t crcRecord(const T& record, size_t crcOffset);
    template <typename T>
    bool readRecord(Preferences& prefs, const char* key, T& out);
    template <typename T>
    bool writeRecordVerified(Preferences& prefs, const char* key, const T& record, T& verify);
};

const char* calibrationAutonomySubsystemName(CalibrationAutonomySubsystem subsystem);
const char* calibrationAutonomyJournalStateName(CalibrationAutonomyJournalState state);

} // namespace tracker
