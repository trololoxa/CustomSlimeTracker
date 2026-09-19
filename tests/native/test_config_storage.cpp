#include "test_common.hpp"

#include <cstddef>
#include <cstdint>

#include "Preferences.h"
#include "config/tracker_config_store.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/mag_axis_alignment.hpp"

using namespace tracker;

namespace {

TrackerConfig makeConfig(uint16_t outputRateHz,
                         float accelQuality = 0.0f,
                         bool withCalibration = false) {
    TrackerConfig config;
    config.resetDefaults();
    config.data.output.outputRateHz = outputRateHz;
    if (withCalibration) {
        config.data.gyroCal.biasValid = true;
        config.data.gyroCal.biasRadS = Vec3(0.001f, -0.002f, 0.003f);
        config.data.accelCal.valid = true;
        config.data.accelCal.biasG = Vec3(0.01f, -0.01f, 0.005f);
        config.data.accelCal.scale = Mat3::identity();
        config.data.accelCalQuality.qualityScore = accelQuality;
        config.data.accelCalQuality.maxFaceNormErrorG = 0.01f;
        config.data.accelCalQuality.maxAxisResidualG = 0.01f;
    }
    config.updateCrc();
    return config;
}

TrackerCalibrationCandidateMetadata candidateMetadata(float overall,
                                                       float gyro,
                                                       float accel) {
    TrackerCalibrationCandidateMetadata metadata;
    metadata.provenance = TrackerCalibrationProvenance::Manual;
    metadata.sampleCount = 4096;
    metadata.independentWindowCount = 8;
    metadata.quality.overallScore = overall;
    metadata.quality.gyroScore = gyro;
    metadata.quality.accelScore = accel;
    metadata.quality.magScore = 0.0f;
    metadata.quality.alignmentScore = 0.0f;
    metadata.quality.coverageScore = 0.0f;
    metadata.quality.gyroResidualDps = 0.01f;
    metadata.quality.accelResidualG = 0.01f;
    metadata.quality.magResidual = 0.0f;
    return metadata;
}


template <typename T>
bool readStoredRecord(const char* name, const char* key, T& out) {
    const auto bytes = Preferences::getTestBytes(name, key);
    if (bytes.size() != sizeof(T)) return false;
    std::memcpy(&out, bytes.data(), sizeof(T));
    return true;
}

bool removeStoredKey(const char* name, const char* key) {
    Preferences prefs;
    if (!prefs.begin(name, false)) return false;
    const bool ok = !prefs.isKey(key) || prefs.remove(key);
    prefs.end();
    return ok;
}

bool convertSlotToLegacyV1(const char* name,
                           const char* slotKey,
                           const char* commitKey) {
    TrackerConfigSlotRecord record;
    if (!readStoredRecord(name, slotKey, record)) return false;
    record.version = tracker_config_storage_detail::LEGACY_SLOT_VERSION;
    record.crc32 = 0;
    record.crc32 = trackerConfigSlotRecordCrc(record);
    Preferences::putTestBytes(name, slotKey, &record, sizeof(record));
    return removeStoredKey(name, commitKey);
}


void testDualSlotSaveAndLoad(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_dual", "cfg");

    TrackerConfig first = makeConfig(50);
    CHECK(ctx, store.save(first));
    TrackerConfigStorageInfo info;
    CHECK(ctx, store.inspectStorage(info));
    CHECK(ctx, info.selectorValid);
    CHECK(ctx, info.selectedSlot == TrackerConfigSlot::A);
    CHECK(ctx, info.slotA.valid);
    CHECK(ctx, !info.slotB.valid);
    CHECK(ctx, info.selectedGeneration == 1u);

    TrackerConfig second = makeConfig(100);
    CHECK(ctx, store.save(second));
    CHECK(ctx, store.inspectStorage(info));
    CHECK(ctx, info.selectedSlot == TrackerConfigSlot::B);
    CHECK(ctx, info.slotA.valid);
    CHECK(ctx, info.slotB.valid);
    CHECK(ctx, info.slotA.generation == 1u);
    CHECK(ctx, info.slotB.generation == 2u);
    CHECK(ctx, info.successfulActiveWrites == 2u);

    TrackerConfig loaded;
    CHECK(ctx, store.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 100u);
}

void testTornInactiveWriteKeepsOldSelector(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_torn", "cfg");
    TrackerConfig active = makeConfig(50);
    CHECK(ctx, store.save(active));

    Preferences::setNextPutLimit(32);
    TrackerConfig failed = makeConfig(100);
    CHECK(ctx, !store.save(failed));
    CHECK(ctx, store.lastError() == TrackerConfigError::WriteFailed);

    TrackerConfigStore rebooted("cfg_torn", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, rebooted.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 50u);
}

void testTornSelectorKeepsPreviousActive(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_torn_selector", "cfg");
    TrackerConfig first = makeConfig(50);
    CHECK(ctx, store.save(first));

    Preferences::setPutLimitForKey("cfg_torn_selector", "cfg_s", 8);
    TrackerConfig second = makeConfig(100);
    CHECK(ctx, !store.save(second));
    CHECK(ctx, store.lastError() == TrackerConfigError::CommitUncertain);

    TrackerConfigStore rebooted("cfg_torn_selector", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, rebooted.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 50u);

    TrackerConfigStorageInfo info;
    CHECK(ctx, rebooted.inspectStorage(info));
    CHECK(ctx, info.slotA.valid);
    CHECK(ctx, info.slotB.valid);
    CHECK(ctx, info.slotB.generation == 2u);
    CHECK(ctx, info.selectedSlot == TrackerConfigSlot::A);
    CHECK(ctx, info.selectedGeneration == 1u);
}

void testCorruptSelectedSlotFallsBack(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_fallback", "cfg");
    TrackerConfig first = makeConfig(50);
    TrackerConfig second = makeConfig(100);
    CHECK(ctx, store.save(first));
    CHECK(ctx, store.save(second));

    CHECK(ctx, Preferences::corruptTestByte(
        "cfg_fallback",
        "cfg_b",
        offsetof(TrackerConfigSlotRecord, payload) + offsetof(TrackerConfigBlob, output)
    ));

    TrackerConfigStore rebooted("cfg_fallback", "cfg");
    TrackerConfigNvsInfo nvsInfo;
    CHECK(ctx, rebooted.inspect(nvsInfo));
    CHECK(ctx, nvsInfo.storage.selectedByFallback);
    CHECK(ctx, nvsInfo.storage.selectedSlot == TrackerConfigSlot::A);
    CHECK(ctx, nvsInfo.storage.selectedGeneration == 1u);

    TrackerConfig loaded;
    CHECK(ctx, rebooted.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 50u);
    TrackerConfigStorageInfo info;
    CHECK(ctx, rebooted.inspectStorage(info));
    CHECK(ctx, info.selectedSlot == TrackerConfigSlot::A);
    CHECK(ctx, info.loadFallbacks == 1u);
}

void testSelectorLossUsesNewestExplicitlyCommittedGeneration(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_selector", "cfg");
    TrackerConfig first = makeConfig(50);
    TrackerConfig second = makeConfig(100);
    CHECK(ctx, store.save(first));
    CHECK(ctx, store.save(second));
    CHECK(ctx, Preferences::corruptTestByte("cfg_selector", "cfg_s", 0));

    TrackerConfigStore rebooted("cfg_selector", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, rebooted.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 100u);
}

void testLegacyV1SelectorLossUsesConservativeOlderGeneration(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_selector_v1", "cfg");
    TrackerConfig first = makeConfig(50);
    TrackerConfig second = makeConfig(100);
    CHECK(ctx, store.save(first));
    CHECK(ctx, store.save(second));
    CHECK(ctx, convertSlotToLegacyV1("cfg_selector_v1", "cfg_a", "cfg_ac"));
    CHECK(ctx, convertSlotToLegacyV1("cfg_selector_v1", "cfg_b", "cfg_bc"));
    CHECK(ctx, Preferences::corruptTestByte("cfg_selector_v1", "cfg_s", 0));

    TrackerConfigStore rebooted("cfg_selector_v1", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, rebooted.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 50u);
}

void testLegacyMigration(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig legacy = makeConfig(77);
    Preferences::putTestBytes("cfg_migrate", "cfg", &legacy.data, sizeof(legacy.data));

    TrackerConfigStore store("cfg_migrate", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, store.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 77u);

    TrackerConfigStorageInfo info;
    CHECK(ctx, store.inspectStorage(info));
    CHECK(ctx, info.selectorValid);
    CHECK(ctx, info.selectedSlot == TrackerConfigSlot::A);
    CHECK(ctx, info.successfulMigrations == 1u);
    CHECK(ctx, trackerCalibrationQualityProvenance(info.slotA.quality) ==
               TrackerCalibrationProvenance::ImportedLegacy);
    CHECK(ctx, !info.legacyExists);
}

void testCandidateDoesNotBecomeActiveBeforeCommit(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_candidate", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));

    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.x = 0.005f;
    candidate.updateCrc();
    auto metadata = candidateMetadata(0.95f, 0.90f, 0.95f);
    CHECK(ctx, store.stageCandidate(candidate, metadata, 1000u));
    CHECK(ctx, store.candidateDirty());
    CHECK(ctx, store.flushCandidate(1000u));

    TrackerConfig loadedBefore;
    CHECK(ctx, store.verify(loadedBefore));
    CHECK_NEAR(ctx, loadedBefore.data.accelCal.biasG.x, 0.01f, 1.0e-6f);

    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, store.prepareCandidatePromotion(prepared, preparedConfig));
    TrackerConfig loadedPrepared;
    CHECK(ctx, store.verify(loadedPrepared));
    CHECK_NEAR(ctx, loadedPrepared.data.accelCal.biasG.x, 0.01f, 1.0e-6f);

    TrackerConfig promoted;
    CHECK(ctx, store.commitPreparedPromotion(prepared, promoted));
    CHECK_NEAR(ctx, promoted.data.accelCal.biasG.x, 0.005f, 1.0e-6f);
    TrackerConfig loadedAfter;
    CHECK(ctx, store.verify(loadedAfter));
    CHECK_NEAR(ctx, loadedAfter.data.accelCal.biasG.x, 0.005f, 1.0e-6f);

    TrackerConfigStorageInfo info;
    CHECK(ctx, store.inspectStorage(info));
    CHECK(ctx, info.successfulPromotions == 1u);

    TrackerPreparedConfigPromotion repeated;
    TrackerConfig repeatedConfig;
    CHECK(ctx, !store.prepareCandidatePromotion(repeated, repeatedConfig, true));
    CHECK(ctx, store.lastError() == TrackerConfigError::CandidateAlreadyPromoted);

    TrackerConfigStore rebooted("cfg_candidate", "cfg");
    TrackerConfig activeAfterReboot;
    CHECK(ctx, rebooted.verify(activeAfterReboot));
    TrackerCalibrationCandidateRecord promotedCandidate;
    CHECK(ctx, rebooted.loadCandidate(promotedCandidate));
    CHECK(ctx, promotedCandidate.metadata.lastComparison ==
               TrackerCalibrationComparisonResult::Promoted);
    CHECK(ctx, promotedCandidate.metadata.activeCalibrationRevisionAtCreation ==
               trackerCalibrationPayloadRevision(activeAfterReboot));
    TrackerPreparedConfigPromotion rebootedRepeat;
    TrackerConfig rebootedRepeatConfig;
    CHECK(ctx, !rebooted.prepareCandidatePromotion(rebootedRepeat, rebootedRepeatConfig, true));
    CHECK(ctx, rebooted.lastError() == TrackerConfigError::CandidateAlreadyPromoted);
}


void testPromotedCandidateMetadataWriteCanRetryWithoutRepromotion(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_promoted_retry", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));
    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.x = 0.004f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, candidateMetadata(0.95f, 0.90f, 0.95f), 1000u));
    CHECK(ctx, store.flushCandidate(1000u, true));

    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, store.prepareCandidatePromotion(prepared, preparedConfig));
    Preferences::setPutLimitForKey("cfg_promoted_retry", "cfg_c", 0u);
    TrackerConfig promoted;
    CHECK(ctx, store.commitPreparedPromotion(prepared, promoted));
    CHECK(ctx, store.candidateDirty());
    TrackerConfigStorageInfo info;
    CHECK(ctx, store.inspectStorage(info));
    CHECK(ctx, info.candidatePromotionStateWriteFailures == 1u);

    CHECK(ctx, store.flushCandidate(2000u, true));
    CHECK(ctx, !store.candidateDirty());
    TrackerConfigStore rebooted("cfg_promoted_retry", "cfg");
    TrackerCalibrationCandidateRecord persisted;
    CHECK(ctx, rebooted.loadCandidate(persisted));
    CHECK(ctx, persisted.metadata.lastComparison ==
               TrackerCalibrationComparisonResult::Promoted);
}

void testPreparedPromotionAbortedByRebootKeepsActive(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_abort", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));

    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.y = -0.005f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, candidateMetadata(0.95f, 0.90f, 0.95f), 1000u));
    CHECK(ctx, store.flushCandidate(1000u));
    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, store.prepareCandidatePromotion(prepared, preparedConfig));

    TrackerConfigStore rebooted("cfg_abort", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, rebooted.load(loaded));
    CHECK_NEAR(ctx, loaded.data.accelCal.biasG.y, -0.01f, 1.0e-6f);
}

void testCandidateSignatureMismatchBlocksPromotion(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_sig", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));

    TrackerConfig candidate = active;
    candidate.data.imu.imuOdr = Lsm6dsv::Odr::Hz480;
    candidate.data.fifo.accelBdr = Lsm6dsv::Odr::Hz480;
    candidate.data.fifo.gyroBdr = Lsm6dsv::Odr::Hz480;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, candidateMetadata(0.99f, 0.99f, 0.99f), 1000u));

    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, !store.prepareCandidatePromotion(prepared, preparedConfig, true));
    CHECK(ctx, store.lastError() == TrackerConfigError::SignatureMismatch);
}

void testCandidateQualityAndWearGates(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_wear", "cfg");
    TrackerCalibrationWearPolicy policy;
    policy.minCandidateWriteIntervalMs = 10000u;
    policy.minQualityImprovement = 0.05f;
    store.setWearPolicy(policy);

    TrackerConfig active = makeConfig(100, 0.70f, true);
    CHECK(ctx, store.save(active));

    TrackerConfig weak = active;
    weak.data.accelCal.biasG.z = 0.004f;
    weak.updateCrc();
    CHECK(ctx, store.stageCandidate(weak, candidateMetadata(0.71f, 0.71f, 0.71f), 1000u));
    CHECK(ctx, !store.flushCandidate(1000u));
    CHECK(ctx, store.lastError() == TrackerConfigError::CandidateNotBetter);

    TrackerConfig good = active;
    good.data.accelCal.biasG.z = 0.003f;
    good.updateCrc();
    CHECK(ctx, store.stageCandidate(good, candidateMetadata(0.95f, 0.90f, 0.95f), 2000u));
    CHECK(ctx, store.flushCandidate(2000u));

    TrackerConfig newer = good;
    newer.data.accelCal.biasG.z = 0.002f;
    newer.updateCrc();
    CHECK(ctx, store.stageCandidate(newer, candidateMetadata(0.97f, 0.92f, 0.97f), 3000u));
    CHECK(ctx, !store.flushCandidate(3000u));
    CHECK(ctx, store.lastError() == TrackerConfigError::WriteThrottled);
    CHECK(ctx, store.flushCandidate(3000u, true));
}


void testCandidateSurvivesUnrelatedActiveConfigChange(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_stale", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));

    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.x = 0.004f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, candidateMetadata(0.95f, 0.90f, 0.95f), 1000u));

    TrackerConfig newer = active;
    newer.data.output.outputRateHz = 80;
    newer.updateCrc();
    CHECK(ctx, store.save(newer));

    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, store.prepareCandidatePromotion(prepared, preparedConfig, true));
    CHECK(ctx, preparedConfig.data.output.outputRateHz == 80u);
    CHECK_NEAR(ctx, preparedConfig.data.accelCal.biasG.x, 0.004f, 1.0e-6f);
}

void testCandidateBecomesStaleAfterActiveCalibrationChange(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_stale_cal", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));

    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.x = 0.004f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, candidateMetadata(0.95f, 0.90f, 0.95f), 1000u));

    TrackerConfig recalibrated = active;
    recalibrated.data.accelCal.biasG.y = -0.004f;
    recalibrated.updateCrc();
    CHECK(ctx, store.save(recalibrated));

    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, !store.prepareCandidatePromotion(prepared, preparedConfig, true));
    CHECK(ctx, store.lastError() == TrackerConfigError::CandidateStale);
}

void testPreparedNewerSlotNeverBecomesFallback(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_prepared_fallback", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));
    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.y = -0.004f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, candidateMetadata(0.95f, 0.90f, 0.95f), 1000u));

    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, store.prepareCandidatePromotion(prepared, preparedConfig));
    CHECK(ctx, Preferences::corruptTestByte(
        "cfg_prepared_fallback", "cfg_a", offsetof(TrackerConfigSlotRecord, payload)
    ));

    TrackerConfigStore rebooted("cfg_prepared_fallback", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, !rebooted.load(loaded));
    CHECK(ctx, rebooted.lastError() == TrackerConfigError::CrcOrValidationFailed);
}

void testAbortInvalidatesPreparedSlot(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_abort_cleanup", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));
    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.z = 0.004f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, candidateMetadata(0.95f, 0.90f, 0.95f), 1000u));
    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, store.prepareCandidatePromotion(prepared, preparedConfig));
    store.abortPreparedPromotion(prepared);
    TrackerConfigStorageInfo info;
    CHECK(ctx, store.inspectStorage(info));
    CHECK(ctx, info.slotB.valid);
    CHECK(ctx, info.slotB.generation == info.selectedGeneration);
}

void testPromotionRejectsDivergedRuntimeCalibration(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_runtime_diverged", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));

    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.x += 0.004f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(
        candidate, candidateMetadata(0.95f, 0.90f, 0.95f), 1000u));

    TrackerConfig runtime = active;
    runtime.data.gyroCal.biasRadS.x += 0.002f;
    runtime.updateCrc();
    TrackerPreparedConfigPromotion rejected;
    TrackerConfig rejectedConfig;
    CHECK(ctx, !store.prepareCandidatePromotion(
        rejected, rejectedConfig, true, &runtime));
    CHECK(ctx, store.lastError() == TrackerConfigError::RuntimeCalibrationDiverged);

    TrackerConfigStorageInfo info;
    CHECK(ctx, store.inspectStorage(info));
    CHECK(ctx, info.selectedGeneration == 1u);
    CHECK(ctx, !info.slotB.exists);

    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, store.prepareCandidatePromotion(
        prepared, preparedConfig, true, &active));
    store.abortPreparedPromotion(prepared);
}

void testPromotionPreservesUnrelatedActiveSettingsAndMeasuredQuality(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_compose", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    active.data.magCal.driverEnabled = true;
    active.data.magCal.calibrationValid = true;
    active.data.magCal.expectedFieldNorm = 1.0f;
    active.data.magCal.minTrustNorm = 0.31f;
    active.data.magCal.maxTrustNorm = 1.8f;
    active.updateCrc();
    CHECK(ctx, store.save(active));

    TrackerConfig candidate = active;
    candidate.data.output.outputRateHz = 50;
    candidate.data.ahrsRuntime.accelKp = 9.0f;
    candidate.data.magCal.driverEnabled = false;
    candidate.data.magCal.minTrustNorm = 0.9f;
    candidate.data.accelCal.biasG.x = 0.004f;
    candidate.updateCrc();
    auto metadata = candidateMetadata(0.95f, 0.90f, 0.95f);
    CHECK(ctx, store.stageCandidate(candidate, metadata, 1000u));

    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, store.prepareCandidatePromotion(prepared, preparedConfig, true));
    CHECK(ctx, preparedConfig.data.output.outputRateHz == 100u);
    CHECK_NEAR(ctx, preparedConfig.data.ahrsRuntime.accelKp, active.data.ahrsRuntime.accelKp, 1e-6f);
    CHECK(ctx, preparedConfig.data.magCal.driverEnabled);
    CHECK_NEAR(ctx, preparedConfig.data.magCal.minTrustNorm, 0.9f, 1e-6f);
    CHECK_NEAR(ctx, preparedConfig.data.accelCal.biasG.x, 0.004f, 1e-6f);

    TrackerConfig promoted;
    CHECK(ctx, store.commitPreparedPromotion(prepared, promoted));
    TrackerConfigStorageInfo info;
    CHECK(ctx, store.inspectStorage(info));
    const auto& quality = info.selectedSlot == TrackerConfigSlot::A ? info.slotA.quality : info.slotB.quality;
    CHECK_NEAR(ctx, quality.overallScore, 0.95f, 1e-6f);
    CHECK(ctx, (quality.qualityFlags & tracker_calibration_quality_flags::SOURCE_MEASURED) != 0u);
    CHECK(ctx, trackerCalibrationQualityProvenance(quality) ==
               TrackerCalibrationProvenance::Manual);

    TrackerConfig unrelated = promoted;
    unrelated.data.output.outputRateHz = 75u;
    unrelated.updateCrc();
    CHECK(ctx, store.save(unrelated));
    CHECK(ctx, store.inspectStorage(info));
    const auto& preserved = info.selectedSlot == TrackerConfigSlot::A ? info.slotA.quality : info.slotB.quality;
    CHECK_NEAR(ctx, preserved.overallScore, 0.95f, 1e-6f);
    CHECK(ctx, (preserved.qualityFlags & tracker_calibration_quality_flags::SOURCE_MEASURED) != 0u);
}

void testMigrationCleanupFailureKeepsMigratedConfig(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig legacy = makeConfig(77);
    Preferences::putTestBytes("cfg_migrate_cleanup", "cfg", &legacy.data, sizeof(legacy.data));
    Preferences::setRemoveFailureForKey("cfg_migrate_cleanup", "cfg");

    TrackerConfigStore store("cfg_migrate_cleanup", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, store.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 77u);
    TrackerConfigStorageInfo info;
    CHECK(ctx, store.inspectStorage(info));
    CHECK(ctx, !info.legacyCleanupPending);
    CHECK(ctx, info.legacyCleanupFailures == 1u);
    CHECK(ctx, !info.legacyExists);
}

void testManualMigrationNeverOverwritesActive(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_no_remigrate", "cfg");
    TrackerConfig active = makeConfig(100);
    CHECK(ctx, store.save(active));
    TrackerConfig legacy = makeConfig(50);
    Preferences::putTestBytes("cfg_no_remigrate", "cfg", &legacy.data, sizeof(legacy.data));
    CHECK(ctx, store.migrateLegacy());
    TrackerConfig loaded;
    CHECK(ctx, store.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 100u);
    CHECK(ctx, Preferences::getTestBytes("cfg_no_remigrate", "cfg").empty());
}

void testSelectorReadbackFailureReconcilesCommittedWrite(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_selector_reconcile", "cfg");
    TrackerConfig first = makeConfig(50);
    CHECK(ctx, store.save(first));
    Preferences::setGetFailuresForKey("cfg_selector_reconcile", "cfg_s", 1u);
    TrackerConfig second = makeConfig(100);
    CHECK(ctx, store.save(second));
    TrackerConfig loaded;
    CHECK(ctx, store.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 100u);
}


void testPersistentSelectorReadFailureReportsUncertain(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_selector_uncertain", "cfg");
    TrackerConfig first = makeConfig(50);
    CHECK(ctx, store.save(first));
    Preferences::setGetFailuresForKey("cfg_selector_uncertain", "cfg_s", 100u);
    TrackerConfig second = makeConfig(100);
    CHECK(ctx, !store.save(second));
    CHECK(ctx, store.lastError() == TrackerConfigError::CommitUncertain);
    TrackerConfig blocked = makeConfig(77);
    CHECK(ctx, !store.save(blocked));
    CHECK(ctx, store.lastError() == TrackerConfigError::CommitUncertain);
    Preferences::setGetFailuresForKey("cfg_selector_uncertain", "cfg_s", 0u);
    TrackerConfig reconciled;
    CHECK(ctx, store.load(reconciled));
    CHECK(ctx, reconciled.data.output.outputRateHz == 100u);
    CHECK(ctx, !store.save(blocked));
    CHECK(ctx, store.lastError() == TrackerConfigError::CommitUncertain);
    store.confirmAuthoritativeConfigApplied();
    CHECK(ctx, store.save(blocked));
    TrackerConfigStorageInfo info;
    CHECK(ctx, store.inspectStorage(info));
    CHECK(ctx, info.commitUncertainCount == 1u);
}

void testLoadStatusDistinguishesStorageFailure(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_load_status", "cfg");
    Preferences::setNextBeginFailures(1u);
    TrackerConfig loaded;
    bool fromNvs = true;
    CHECK(ctx, store.loadOrDefaults(loaded, &fromNvs));
    CHECK(ctx, !fromNvs);
    CHECK(ctx, store.lastLoadStatus() == TrackerConfigLoadStatus::DefaultsStorageError);
    CHECK(ctx, store.lastLoadError() == TrackerConfigError::NvsBeginFailed);

    TrackerConfigStore emptyStore("cfg_load_empty", "cfg");
    TrackerConfig emptyLoaded;
    CHECK(ctx, emptyStore.loadOrDefaults(emptyLoaded, &fromNvs));
    CHECK(ctx, !fromNvs);
    CHECK(ctx, emptyStore.lastLoadStatus() == TrackerConfigLoadStatus::DefaultsNotFound);
    CHECK(ctx, emptyStore.lastLoadError() == TrackerConfigError::NotFound);
}


void testEmptyNvsLifecycle(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_empty", "cfg");
    TrackerConfig config;
    bool loadedFromNvs = true;
    CHECK(ctx, store.loadOrDefaults(config, &loadedFromNvs));
    CHECK(ctx, !loadedFromNvs);
    CHECK(ctx, store.lastLoadStatus() == TrackerConfigLoadStatus::DefaultsNotFound);
    CHECK(ctx, store.lastLoadError() == TrackerConfigError::NotFound);
    CHECK(ctx, store.save(config));

    TrackerConfigStorageInfo info;
    CHECK(ctx, store.inspectStorage(info));
    CHECK(ctx, info.selectorValid);
    CHECK(ctx, info.selectedSlot == TrackerConfigSlot::A);
    CHECK(ctx, info.slotA.valid);
    CHECK(ctx, info.slotA.commitMarkerValid);
    CHECK(ctx, info.selectedGeneration == 1u);

    TrackerConfigStore rebooted("cfg_empty", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, rebooted.load(loaded));
    CHECK(ctx, rebooted.lastLoadStatus() == TrackerConfigLoadStatus::Loaded);
}

void testCorruptActiveStorageIsNotClassifiedAsEmpty(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_corrupt_all", "cfg");
    TrackerConfig active = makeConfig(77, 0.8f, true);
    CHECK(ctx, store.save(active));
    CHECK(ctx, Preferences::corruptTestByte(
        "cfg_corrupt_all", "cfg_a", offsetof(TrackerConfigSlotRecord, payload)));

    TrackerConfigStore rebooted("cfg_corrupt_all", "cfg");
    TrackerConfig fallback;
    bool loadedFromNvs = true;
    CHECK(ctx, rebooted.loadOrDefaults(fallback, &loadedFromNvs));
    CHECK(ctx, !loadedFromNvs);
    CHECK(ctx, rebooted.lastLoadStatus() == TrackerConfigLoadStatus::DefaultsStorageError);
    CHECK(ctx, rebooted.lastLoadError() == TrackerConfigError::CrcOrValidationFailed);
    TrackerConfigStorageInfo info;
    CHECK(ctx, rebooted.inspectStorage(info));
    CHECK(ctx, info.storageDegradedLatched);
}

void testTransientBootReadFailureBlocksWritesUntilAuthoritativeLoad(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore writer("cfg_transient", "cfg");
    TrackerConfig good = makeConfig(77, 0.8f, true);
    CHECK(ctx, writer.save(good));

    Preferences::setNextBeginFailures(1u);
    TrackerConfigStore booted("cfg_transient", "cfg");
    TrackerConfig runtime;
    bool loadedFromNvs = true;
    CHECK(ctx, booted.loadOrDefaults(runtime, &loadedFromNvs));
    CHECK(ctx, !loadedFromNvs);
    CHECK(ctx, booted.lastLoadStatus() == TrackerConfigLoadStatus::DefaultsStorageError);
    CHECK(ctx, !runtime.data.gyroCal.biasValid);

    TrackerConfig verified;
    CHECK(ctx, booted.verify(verified));
    CHECK(ctx, verified.data.output.outputRateHz == 77u);
    CHECK(ctx, verified.data.gyroCal.biasValid);
    CHECK(ctx, !booted.save(runtime));
    CHECK(ctx, booted.lastError() == TrackerConfigError::StorageDegraded);

    TrackerConfig loaded;
    CHECK(ctx, booted.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 77u);
    CHECK(ctx, loaded.data.gyroCal.biasValid);
    CHECK(ctx, !booted.save(loaded));
    CHECK(ctx, booted.lastError() == TrackerConfigError::StorageDegraded);
    booted.confirmAuthoritativeConfigApplied();
    CHECK(ctx, booted.save(loaded));
}


void testAuthoritativeLoadRequiresApplyConfirmationBeforeWrites(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore writer("cfg_apply_pending", "cfg");
    TrackerConfig active = makeConfig(77, 0.8f, true);
    CHECK(ctx, writer.save(active));

    TrackerConfigStore rebooted("cfg_apply_pending", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, rebooted.load(loaded));
    TrackerConfigStorageInfo pendingInfo;
    CHECK(ctx, rebooted.inspectStorage(pendingInfo));
    CHECK(ctx, pendingInfo.authoritativeApplyPending);

    TrackerConfig changed = loaded;
    changed.data.output.outputRateHz = 88u;
    changed.updateCrc();
    CHECK(ctx, !rebooted.save(changed));
    CHECK(ctx, rebooted.lastError() == TrackerConfigError::ApplyPending);

    rebooted.markAuthoritativeConfigApplyFailed();
    CHECK(ctx, !rebooted.save(changed));
    CHECK(ctx, rebooted.lastError() == TrackerConfigError::StorageDegraded);

    TrackerConfig reloaded;
    CHECK(ctx, rebooted.load(reloaded));
    rebooted.confirmAuthoritativeConfigApplied();
    CHECK(ctx, rebooted.save(changed));
}

void testFirstSaveReadbackFailureNeverBecomesAuthoritative(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_first_readback", "cfg");
    Preferences::setGetFailuresForKey("cfg_first_readback", "cfg_a", 1u);
    TrackerConfig candidate = makeConfig(66, 0.8f, true);
    CHECK(ctx, !store.save(candidate));
    CHECK(ctx, store.lastError() == TrackerConfigError::ReadFailed);
    CHECK(ctx, Preferences::getTestBytes("cfg_first_readback", "cfg_s").empty());
    CHECK(ctx, Preferences::getTestBytes("cfg_first_readback", "cfg_ac").empty());

    TrackerConfigStore rebooted("cfg_first_readback", "cfg");
    TrackerConfig loaded;
    bool fromNvs = true;
    CHECK(ctx, rebooted.loadOrDefaults(loaded, &fromNvs));
    CHECK(ctx, !fromNvs);
    CHECK(ctx, rebooted.lastLoadStatus() == TrackerConfigLoadStatus::DefaultsStorageError);
    CHECK(ctx, rebooted.lastLoadError() == TrackerConfigError::CrcOrValidationFailed);
}


void testInterruptedLegacyMigrationRecoversFromStillValidLegacyBlob(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig legacy = makeConfig(77, 0.8f, true);
    Preferences::putTestBytes("cfg_migrate_retry", "cfg", &legacy.data, sizeof(legacy.data));
    Preferences::setGetFailuresForKey("cfg_migrate_retry", "cfg_a", 1u);

    TrackerConfigStore firstBoot("cfg_migrate_retry", "cfg");
    TrackerConfig failed;
    CHECK(ctx, !firstBoot.load(failed));
    CHECK(ctx, firstBoot.lastError() == TrackerConfigError::ReadFailed);
    CHECK(ctx, !Preferences::getTestBytes("cfg_migrate_retry", "cfg").empty());
    CHECK(ctx, Preferences::getTestBytes("cfg_migrate_retry", "cfg_s").empty());

    TrackerConfigStore rebooted("cfg_migrate_retry", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, rebooted.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 77u);
    CHECK(ctx, loaded.data.gyroCal.biasValid);
    CHECK(ctx, rebooted.lastLoadStatus() == TrackerConfigLoadStatus::Migrated);
    TrackerConfigStorageInfo info;
    CHECK(ctx, rebooted.inspectStorage(info));
    CHECK(ctx, info.selectorValid);
    CHECK(ctx, info.slotA.commitMarkerValid);
    CHECK(ctx, !info.legacyExists);
}

void testSelectorIsAuthoritativeWhenCommitMarkerWriteFails(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_marker_repair", "cfg");
    Preferences::setPutLimitForKey("cfg_marker_repair", "cfg_ac", 0u);
    TrackerConfig active = makeConfig(77, 0.8f, true);
    CHECK(ctx, store.save(active));

    TrackerConfigStore rebooted("cfg_marker_repair", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, rebooted.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 77u);
    TrackerConfigStorageInfo info;
    CHECK(ctx, rebooted.inspectStorage(info));
    CHECK(ctx, info.slotA.commitMarkerValid);
}

void testNoOpSavePreservesGenerationAndCandidate(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_noop", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));
    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.x = 0.004f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, candidateMetadata(0.95f, 0.90f, 0.95f), 1000u));

    TrackerConfigStorageInfo before;
    CHECK(ctx, store.inspectStorage(before));
    TrackerConfig identical = active;
    CHECK(ctx, store.save(identical));
    TrackerConfigStorageInfo after;
    CHECK(ctx, store.inspectStorage(after));
    CHECK(ctx, after.selectedGeneration == before.selectedGeneration);
    CHECK(ctx, after.noOpSaveCount == before.noOpSaveCount + 1u);

    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, store.prepareCandidatePromotion(prepared, preparedConfig, true));
}


void testLoadUpgradesLegacyV1ActiveSlotToV3(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_upgrade_v1", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));
    CHECK(ctx, convertSlotToLegacyV1("cfg_upgrade_v1", "cfg_a", "cfg_ac"));

    TrackerConfigStore upgraded("cfg_upgrade_v1", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, upgraded.load(loaded));
    CHECK(ctx, upgraded.lastLoadStatus() == TrackerConfigLoadStatus::Migrated);
    upgraded.confirmAuthoritativeConfigApplied();
    TrackerConfigStorageInfo after;
    CHECK(ctx, upgraded.inspectStorage(after));
    CHECK(ctx, after.selectedGeneration == 2u);
    const TrackerConfigSlotInfo& selected = after.selectedSlot == TrackerConfigSlot::A
        ? after.slotA : after.slotB;
    CHECK(ctx, selected.valid);
    CHECK(ctx, !selected.legacyCommitted);
    CHECK(ctx, selected.commitMarkerValid);
}

void testLegacyV1CandidateUsesGenerationFreshnessContract(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_candidate_v1", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));
    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.x = 0.004f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, candidateMetadata(0.95f, 0.90f, 0.95f), 1000u));
    CHECK(ctx, store.flushCandidate(1000u, true));

    TrackerConfigStorageInfo info;
    CHECK(ctx, store.inspectStorage(info));
    TrackerCalibrationCandidateRecord record;
    CHECK(ctx, readStoredRecord("cfg_candidate_v1", "cfg_c", record));
    record.version = tracker_config_storage_detail::LEGACY_CANDIDATE_VERSION;
    record.metadata.activeCalibrationRevisionAtCreation = info.selectedGeneration;
    record.crc32 = 0;
    record.crc32 = trackerCalibrationCandidateRecordCrc(record);
    Preferences::putTestBytes("cfg_candidate_v1", "cfg_c", &record, sizeof(record));

    TrackerConfigStore rebooted("cfg_candidate_v1", "cfg");
    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, rebooted.prepareCandidatePromotion(prepared, preparedConfig, true));
}

void testCorruptCandidateDoesNotBreakActiveConfig(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_bad_candidate", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));
    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.x = 0.004f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, candidateMetadata(0.95f, 0.90f, 0.95f), 1000u));
    CHECK(ctx, store.flushCandidate(1000u, true));
    CHECK(ctx, Preferences::corruptTestByte(
        "cfg_bad_candidate", "cfg_c", offsetof(TrackerCalibrationCandidateRecord, payload)));

    TrackerConfigStore rebooted("cfg_bad_candidate", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, rebooted.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 100u);
    TrackerCalibrationCandidateRecord bad;
    CHECK(ctx, !rebooted.loadCandidate(bad));
    CHECK(ctx, rebooted.lastError() == TrackerConfigError::CandidateInvalid);
}

void testDegradedStateBlocksCandidateDiscardButAllowsFullErase(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore writer("cfg_degraded_erase", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, writer.save(active));
    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.x = 0.004f;
    candidate.updateCrc();
    CHECK(ctx, writer.stageCandidate(candidate, candidateMetadata(0.95f, 0.90f, 0.95f), 1000u));
    CHECK(ctx, writer.flushCandidate(1000u, true));

    Preferences::setNextBeginFailures(1u);
    TrackerConfigStore booted("cfg_degraded_erase", "cfg");
    TrackerConfig defaults;
    CHECK(ctx, booted.loadOrDefaults(defaults));
    CHECK(ctx, !booted.discardCandidate());
    CHECK(ctx, booted.lastError() == TrackerConfigError::StorageDegraded);
    CHECK(ctx, booted.erase());

    TrackerConfigStore empty("cfg_degraded_erase", "cfg");
    TrackerConfig loaded;
    bool fromNvs = true;
    CHECK(ctx, empty.loadOrDefaults(loaded, &fromNvs));
    CHECK(ctx, !fromNvs);
    CHECK(ctx, empty.lastLoadStatus() == TrackerConfigLoadStatus::DefaultsNotFound);
}

void testRuntimeSnapshotNoOpAndProvenancePreservation(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_snapshot_noop", "cfg");

    TrackerConfig active = makeConfig(100, 0.91f, true);
    GyroTempCompensator temp;
    temp.setModel(Vec3(0.001f, -0.002f, 0.003f),
                  32.0f,
                  Vec3(0.00001f, -0.00002f, 0.00003f));
    temp.setQualityMetadata(18.0f, 45.0f, 0.96f, 0.15f, 0.02f);
    active.captureFromGyroTempCompUpdate(temp, 1234u);
    CHECK(ctx, store.save(active, TrackerCalibrationProvenance::ImportedLegacy));

    TrackerConfig snapshot = active;
    snapshot.captureFromGyroTempComp(temp);
    CHECK(ctx, snapshot.data.gyroCalMeta.tempModelUpdatedUptimeMs == 1234u);
    CHECK(ctx, snapshot.data.crc32 == active.data.crc32);
    CHECK(ctx, store.save(snapshot, TrackerCalibrationProvenance::Manual));

    TrackerConfigStorageInfo info;
    CHECK(ctx, store.inspectStorage(info));
    CHECK(ctx, info.selectedGeneration == 1u);
    CHECK(ctx, info.noOpSaveCount == 1u);
    const TrackerConfigSlotInfo& selected =
        info.selectedSlot == TrackerConfigSlot::A ? info.slotA : info.slotB;
    CHECK(ctx, trackerCalibrationQualityProvenance(selected.quality) ==
               TrackerCalibrationProvenance::ImportedLegacy);

    // A policy-only save writes a new config generation but must preserve the
    // accepted calibration evidence/provenance and model revision.
    TrackerConfig policyChanged = snapshot;
    policyChanged.data.gyroCal.tempCompEnabled = false;
    policyChanged.updateCrc();
    CHECK(ctx, store.save(policyChanged, TrackerCalibrationProvenance::Manual));
    CHECK(ctx, store.inspectStorage(info));
    CHECK(ctx, info.selectedGeneration == 2u);
    const TrackerConfigSlotInfo& selectedPolicy =
        info.selectedSlot == TrackerConfigSlot::A ? info.slotA : info.slotB;
    CHECK(ctx, trackerCalibrationQualityProvenance(selectedPolicy.quality) ==
               TrackerCalibrationProvenance::ImportedLegacy);
}

void testV3CandidateFreshAcrossEvidenceAndTempPolicyChanges(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_v3_fresh", "cfg");
    TrackerConfig active = makeConfig(100, 0.80f, true);
    active.data.gyroCal.tempCompValid = true;
    active.data.gyroCal.referenceTempC = 30.0f;
    active.data.gyroCal.tempSlopeRadSPerC = Vec3(0.0001f, 0.0002f, -0.0001f);
    active.data.gyroTempQuality.fitQuality = 0.8f;
    active.data.gyroCalMeta.tempModelUpdatedUptimeMs = 100u;
    active.updateCrc();
    CHECK(ctx, store.save(active, TrackerCalibrationProvenance::Setup));

    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.x += 0.001f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, candidateMetadata(0.95f, 0.90f, 0.95f), 1000u));
    CHECK(ctx, store.flushCandidate(1000u, true));

    TrackerCalibrationCandidateRecord staged;
    CHECK(ctx, store.loadCandidate(staged));
    CHECK(ctx, staged.version == tracker_config_storage_detail::CANDIDATE_VERSION);

    TrackerConfig evidencePolicyChanged = active;
    evidencePolicyChanged.data.gyroCal.tempCompEnabled = false;
    evidencePolicyChanged.data.gyroCalMeta.tempModelUpdatedUptimeMs = 999u;
    evidencePolicyChanged.data.gyroTempQuality.fitQuality = 0.79f;
    evidencePolicyChanged.updateCrc();
    CHECK(ctx, store.save(evidencePolicyChanged, TrackerCalibrationProvenance::Manual));

    TrackerCalibrationComparisonResult result = TrackerCalibrationComparisonResult::NotCompared;
    uint32_t flags = 0;
    CHECK(ctx, store.compareCandidate(staged, result, flags));
    CHECK(ctx, result != TrackerCalibrationComparisonResult::StaleActiveGeneration);
    CHECK(ctx, (flags & tracker_calibration_comparison_flags::STALE_ACTIVE_GENERATION) == 0u);
}

void testV2CandidateRevisionCompatibility(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_v2_candidate", "cfg");
    TrackerConfig active = makeConfig(100, 0.80f, true);
    active.data.gyroCalMeta.biasCalibrationUptimeMs = 123u;
    active.updateCrc();
    CHECK(ctx, store.save(active));

    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.x += 0.001f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, candidateMetadata(0.95f, 0.90f, 0.95f), 1000u));
    CHECK(ctx, store.flushCandidate(1000u, true));

    TrackerCalibrationCandidateRecord record;
    CHECK(ctx, readStoredRecord("cfg_v2_candidate", "cfg_c", record));
    record.version = tracker_config_storage_detail::METADATA_REVISION_CANDIDATE_VERSION;
    record.metadata.activeCalibrationRevisionAtCreation =
        trackerCalibrationPayloadRevisionLegacyV2(active);
    record.crc32 = 0;
    record.crc32 = trackerCalibrationCandidateRecordCrc(record);
    Preferences::putTestBytes("cfg_v2_candidate", "cfg_c", &record, sizeof(record));

    TrackerConfigStore rebooted("cfg_v2_candidate", "cfg");
    TrackerCalibrationCandidateRecord loaded;
    TrackerCalibrationComparisonResult result = TrackerCalibrationComparisonResult::NotCompared;
    uint32_t flags = 0;
    CHECK(ctx, rebooted.compareCandidate(loaded, result, flags));
    CHECK(ctx, loaded.version == tracker_config_storage_detail::METADATA_REVISION_CANDIDATE_VERSION);
    CHECK(ctx, result != TrackerCalibrationComparisonResult::StaleActiveGeneration);
}

void testDormantFrameBytesDoNotChangeSensorSignature(TestContext& ctx) {
    TrackerConfig a;
    TrackerConfig b;
    a.resetDefaults();
    b.resetDefaults();
    a.data.frame.sensorToDeviceValid = false;
    b.data.frame.sensorToDeviceValid = false;
    b.data.frame.sensorToDevice = Mat3::diagonal(-1.0f, -1.0f, 1.0f);

    const TrackerSensorSignature sa = trackerMakeSensorSignature(a);
    const TrackerSensorSignature sb = trackerMakeSensorSignature(b);
    CHECK(ctx, trackerSensorSignaturesEqual(sa, sb));

    b.data.frame.sensorToDeviceValid = true;
    const TrackerSensorSignature enabled = trackerMakeSensorSignature(b);
    CHECK(ctx, !trackerSensorSignaturesEqual(sa, enabled));
}

void testPromotionRejectsDivergedRuntimeSensorContract(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_runtime_sig", "cfg");
    TrackerConfig active = makeConfig(100, 0.60f, true);
    CHECK(ctx, store.save(active));

    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.x += 0.002f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, candidateMetadata(0.95f, 0.95f, 0.95f), 1000u));

    TrackerConfig runtime = active;
    runtime.data.imu.accelFs = Lsm6dsv::AccelFs::G4;
    runtime.updateCrc();

    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, !store.prepareCandidatePromotion(prepared, preparedConfig, true, &runtime));
    CHECK(ctx, store.lastError() == TrackerConfigError::RuntimeSensorSignatureDiverged);
}


void testMeasuredAxisCandidateBeatsUnmeasuredValidAlignment(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_axis_quality", "cfg");

    TrackerConfig active = makeConfig(100, 0.90f, true);
    active.data.magCal.calibrationValid = true;
    active.data.magCal.axisAlignmentValid = true;
    active.data.magCal.magToImu = Mat3::identity();
    active.data.magCalQuality.coverageScore = 0.90f;
    active.data.magCalQuality.residualRms = 0.02f;
    active.updateCrc();
    CHECK(ctx, store.save(active));

    TrackerConfig candidate = active;
    candidate.data.magCal.magToImu = Quat::fromEulerXYZ(
        0.5f * MATH_DEG_TO_RAD,
       -1.0f * MATH_DEG_TO_RAD,
        1.5f * MATH_DEG_TO_RAD).toRotationMatrix();
    candidate.updateCrc();

    TrackerCalibrationCandidateMetadata metadata;
    metadata.provenance = TrackerCalibrationProvenance::Background;
    metadata.sampleCount = 64u;
    metadata.independentWindowCount = 5u;
    metadata.quality = trackerCalibrationQualityFromConfig(candidate);
    metadata.quality.alignmentScore = 0.80f;
    metadata.quality.qualityFlags |=
        tracker_calibration_quality_flags::ALIGNMENT_MEASURED |
        tracker_calibration_quality_flags::SOURCE_MEASURED;
    trackerCalibrationQualityRecomputeOverall(candidate, metadata.quality);
    trackerCalibrationQualitySetProvenance(
        metadata.quality, TrackerCalibrationProvenance::Background);

    CHECK(ctx, store.stageCandidate(candidate, metadata, 1000u));
    bool exists = false;
    CHECK(ctx, store.candidateExists(exists));
    CHECK(ctx, exists);

    TrackerCalibrationCandidateRecord compared;
    TrackerCalibrationComparisonResult result = TrackerCalibrationComparisonResult::NotCompared;
    uint32_t flags = 0u;
    CHECK(ctx, store.compareCandidate(compared, result, flags));
    CHECK(ctx, result == TrackerCalibrationComparisonResult::Better);
    CHECK(ctx, (flags & tracker_calibration_comparison_flags::BELOW_MIN_IMPROVEMENT) == 0u);
    CHECK(ctx, (flags & tracker_calibration_comparison_flags::ALIGNMENT_REGRESSION) == 0u);

    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, store.prepareCandidatePromotion(prepared, preparedConfig));
    CHECK(ctx, prepared.valid);
    CHECK(ctx, preparedConfig.data.magCal.axisAlignmentValid);
}

void testSolverMeasuredAxisCandidateUsesNormalStorePromotionPath(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_axis_solver", "cfg");

    TrackerConfig active = makeConfig(100, 0.90f, true);
    active.data.magCal.calibrationValid = true;
    active.data.magCal.hardIron = Vec3::zero();
    active.data.magCal.softIron = Mat3::identity();
    active.data.magCal.axisAlignmentValid = true;
    const Mat3 coarse(0,-1,0, 1,0,0, 0,0,1);
    active.data.magCal.magToImu = coarse;
    active.updateCrc();
    CHECK(ctx, store.save(active));

    const Mat3 residual = Quat::fromEulerXYZ(
        0.7f * MATH_DEG_TO_RAD,
       -1.1f * MATH_DEG_TO_RAD,
        1.6f * MATH_DEG_TO_RAD).toRotationMatrix();
    const Mat3 expected = residual * coarse;
    const Mat3 inverse = expected.transposed();
    MagAxisAlignmentInterval intervals[64];
    Vec3 field = Vec3(0.45f, 0.20f, 0.87f).normalized();
    const float rate = 90.0f * MATH_DEG_TO_RAD;
    for (uint16_t i = 0; i < 64u; ++i) {
        const Vec3 axis = (i % 4u == 0u) ? Vec3(1.0f, 0.2f, 0.1f).normalized()
                        : (i % 4u == 1u) ? Vec3(0.1f, 1.0f, 0.3f).normalized()
                        : (i % 4u == 2u) ? Vec3(0.25f, -0.1f, 1.0f).normalized()
                                         : Vec3(-0.7f, 0.5f, 0.5f).normalized();
        const Vec3 gyro = axis * rate;
        const float dt = 0.017f;
        const Vec3 next = Quat::fromRotationVector(gyro * (-dt)).rotate(field).normalized();
        intervals[i] = MagAxisAlignmentInterval{
            gyro, inverse * field, inverse * next, dt,
            static_cast<uint16_t>(i / 8u)};
        field = next;
    }

    MagAxisAlignmentSolvePolicy policy;
    MagAxisAlignmentResult solved;
    CHECK(ctx, solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), Mat3::identity(),
        &coarse, policy, solved));
    CHECK(ctx, solved.valid);
    CHECK(ctx, solved.validationPassed);
    CHECK(ctx, solved.improvesActive);
    CHECK(ctx, solved.qualityScore >= 0.62f);

    TrackerConfig candidate = active;
    candidate.data.magCal.magToImu = solved.magToImu;
    candidate.updateCrc();

    TrackerCalibrationCandidateMetadata metadata;
    metadata.provenance = TrackerCalibrationProvenance::Background;
    metadata.sampleCount = solved.usedIntervals;
    metadata.independentWindowCount = solved.independentWindows;
    metadata.quality = trackerCalibrationQualityFromConfig(candidate);
    metadata.quality.alignmentScore = solved.qualityScore;
    metadata.quality.qualityFlags |=
        tracker_calibration_quality_flags::ALIGNMENT_MEASURED |
        tracker_calibration_quality_flags::SOURCE_MEASURED;
    trackerCalibrationQualityRecomputeOverall(candidate, metadata.quality);
    trackerCalibrationQualitySetProvenance(
        metadata.quality, TrackerCalibrationProvenance::Background);

    CHECK(ctx, store.stageCandidate(candidate, metadata, 1000u));
    TrackerCalibrationCandidateRecord compared;
    TrackerCalibrationComparisonResult result =
        TrackerCalibrationComparisonResult::NotCompared;
    uint32_t flags = 0u;
    CHECK(ctx, store.compareCandidate(compared, result, flags));
    CHECK(ctx, result == TrackerCalibrationComparisonResult::Better);
    CHECK(ctx, (flags & tracker_calibration_comparison_flags::BELOW_MIN_IMPROVEMENT) == 0u);

    TrackerPreparedConfigPromotion prepared;
    TrackerConfig preparedConfig;
    CHECK(ctx, store.prepareCandidatePromotion(prepared, preparedConfig));
    CHECK(ctx, prepared.valid);
    CHECK(ctx, magAxisRotationDifferenceDeg(
        preparedConfig.data.magCal.magToImu, expected) < 1.0f);
}

void testGenerationWrapComparison(TestContext& ctx) {
    CHECK(ctx, trackerGenerationIsNewer(1u, UINT32_MAX));
    CHECK(ctx, !trackerGenerationIsNewer(UINT32_MAX, 1u));
    CHECK(ctx, !trackerGenerationIsNewer(5u, 5u));
}

bool convertSlotToTransitionalV2(const char* name,
                                 const char* slotKey,
                                 bool addHistoricalFields = true) {
    TrackerConfigSlotRecord record;
    if (!readStoredRecord(name, slotKey, record)) return false;
    record.version = tracker_config_storage_detail::TRANSITIONAL_SLOT_VERSION;
    if (addHistoricalFields) {
        // These values were valid/persisted before the 0028 semantic contract.
        // sanitize() has deterministic compatibility rules for each of them.
        record.payload.output.packetFormat = 0u;
        record.payload.ahrs.reservedAccelTrustMinNormG = 0.0f;
        record.payload.ahrs.reservedAccelTrustMaxNormG = 0.0f;
        record.payload.magYaw.maxInnovationDeg = 0.0f;
        record.payload.ahrsRuntime.minDtS = 0.0f;
        record.payload.crc32 = 0u;
        TrackerConfig payload;
        payload.data = record.payload;
        payload.updateCrc();
        record.payload = payload.data;
    }
    record.crc32 = 0;
    record.crc32 = trackerConfigSlotRecordCrc(record);
    Preferences::putTestBytes(name, slotKey, &record, sizeof(record));
    return true;
}

TrackerConfig makeMagEnabledCalibratedConfig() {
    TrackerConfig config = makeConfig(100u, 0.95f, true);
    config.data.hardware.spiHz = 4000000u;
    config.data.fifo.watermarkWords = 12u;
    config.data.frame.sensorToDeviceValid = true;
    config.data.frame.sensorToDevice = Mat3::identity();
    config.data.magCal.driverEnabled = true;
    config.data.magCal.calibrationValid = true;
    config.data.magCal.axisAlignmentValid = true;
    config.data.magCal.hardIron = Vec3(120.0f, -80.0f, 45.0f);
    config.data.magCal.softIron = Mat3::diagonal(1.10f, 0.95f, 1.02f);
    config.data.magCal.magToImu = Mat3::identity();
    config.data.magCal.expectedFieldNorm = 900.0f;
    config.data.magCal.minTrustNorm = 450.0f;
    config.data.magCal.maxTrustNorm = 1600.0f;
    config.data.magYaw.applyEnabled = true;
    config.updateCrc();
    return config;
}

void testDeployedV3MigrationKeepsCalibrations(TestContext& ctx) {
    for (unsigned variant = 0u; variant < 5u; ++variant) {
        Preferences::clearTestStorage();
        TrackerConfigStore writer("cfg_v3", "cfg");
        auto good = makeMagEnabledCalibratedConfig();
        CHECK(ctx, writer.save(good));
        TrackerConfigSlotRecord record;
        CHECK(ctx, readStoredRecord("cfg_v3", "cfg_a", record));
        record.version = tracker_config_storage_detail::DEPLOYED_V3_SLOT_VERSION;
        TrackerConfig old;
        old.data = record.payload;
        old.data.hardware.serialBaud /= 2u;
        old.data.output.quaternionOutputEnabled = true;
        old.data.output.serialDebugEnabled = true;
        old.data.quality.expectedDtUs = 10000.0f;
        old.updateCrc();
        CHECK(ctx, old.validate());
        CHECK(ctx, !old.validateSemanticConfig());
        record.payload = old.data;
        record.crc32 = trackerConfigSlotRecordCrc(record);
        Preferences::putTestBytes("cfg_v3", "cfg_a", &record, sizeof(record));
        const auto oldBytes = Preferences::getTestBytes("cfg_v3", "cfg_a");
        const auto selector = Preferences::getTestBytes("cfg_v3", "cfg_s");
        TrackerConfigStore reader("cfg_v3", "cfg");
        if (variant == 1u) reader.setWriteInhibited(true);
        if (variant == 2u) Preferences::setGetFailuresForKey("cfg_v3", "cfg_b", 1u);
        if (variant == 3u) Preferences::setPutLimitForKey("cfg_v3", "cfg_b", 0u);
        if (variant == 4u) Preferences::setPutLimitForKey("cfg_v3", "cfg_s", 0u);
        TrackerConfig loaded;
        if (variant < 2u) {
            CHECK(ctx, reader.load(loaded));
            CHECK(ctx, loaded.validateSemanticConfig());
            CHECK(ctx, loaded.data.hardware.serialBaud == cfg::SERIAL_BAUD);
            CHECK(ctx, !loaded.data.output.serialDebugEnabled);
            CHECK(ctx, loaded.data.output.quaternionOutputEnabled);
            CHECK(ctx, loaded.data.quality.expectedDtUs == 0.0f);
            CHECK(ctx, loaded.data.magCal.driverEnabled);
            CHECK(ctx, trackerCalibrationModelEqual(good, loaded));
            CHECK(ctx, std::memcmp(&good.data.magCal, &loaded.data.magCal, sizeof(good.data.magCal)) == 0);
            CHECK(ctx, Preferences::getTestBytes("cfg_v3", "cfg_a") == oldBytes);
            if (variant == 1u) {
                CHECK(ctx, reader.lastLoadStatus() == TrackerConfigLoadStatus::LoadedLegacyReadOnly);
                CHECK(ctx, Preferences::getTestBytes("cfg_v3", "cfg_s") == selector);
                CHECK(ctx, Preferences::getTestBytes("cfg_v3", "cfg_b").empty());
            } else {
                CHECK(ctx, readStoredRecord("cfg_v3", "cfg_b", record));
                CHECK(ctx, record.version == tracker_config_storage_detail::SLOT_VERSION);
                const auto committed = Preferences::getTestBytes("cfg_v3", "cfg_s");
                TrackerConfigStore rebooted("cfg_v3", "cfg");
                CHECK(ctx, rebooted.load(loaded));
                CHECK(ctx, Preferences::getTestBytes("cfg_v3", "cfg_s") == committed);
            }
        } else {
            CHECK(ctx, !reader.load(loaded));
            CHECK(ctx, Preferences::getTestBytes("cfg_v3", "cfg_a") == oldBytes);
            if (variant != 4u) CHECK(ctx, Preferences::getTestBytes("cfg_v3", "cfg_s") == selector);
            // A torn selector is repaired from committed old-good authority.
            Preferences::setPutLimitForKey("cfg_v3", "cfg_s", 4096u);
            Preferences::setPutLimitForKey("cfg_v3", "cfg_b", 4096u);
            TrackerConfigStore rebooted("cfg_v3", "cfg");
            CHECK(ctx, rebooted.load(loaded));
            CHECK(ctx, trackerCalibrationModelEqual(good, loaded));
            CHECK(ctx, loaded.data.magCal.driverEnabled);
        }
    }
    // Versioned normalization never repairs a broken calibration matrix.
    Preferences::clearTestStorage();
    auto invalid = makeMagEnabledCalibratedConfig();
    invalid.data.accelCal.scale = Mat3{};
    invalid.data.accelCal.scale.m[0][0] = 0.0f;
    invalid.updateCrc();
    CHECK(ctx, !trackerNormalizeDeployedV3(invalid));
}

void testSafeModeWriteInhibit(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_safe_mode", "cfg");
    TrackerConfig config = makeConfig(100u);
    store.setWriteInhibited(true);
    CHECK(ctx, !store.save(config));
    CHECK(ctx, store.lastError() == TrackerConfigError::WriteInhibited);
    CHECK(ctx, !store.erase());
    CHECK(ctx, store.lastError() == TrackerConfigError::WriteInhibited);
    store.setWriteInhibited(false);
    CHECK(ctx, store.save(config));
}

void testSafeModeLoadPerformsNoRepairOrMigrationWrites(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig legacy = makeConfig(73u);
    Preferences::putTestBytes("cfg_safe_legacy", "cfg", &legacy.data, sizeof(legacy.data));

    TrackerConfigStore legacyReader("cfg_safe_legacy", "cfg");
    legacyReader.setWriteInhibited(true);
    TrackerConfig loaded;
    CHECK(ctx, legacyReader.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 73u);
    CHECK(ctx, legacyReader.lastLoadStatus() ==
                   TrackerConfigLoadStatus::LoadedLegacyReadOnly);
    CHECK(ctx, !Preferences::getTestBytes("cfg_safe_legacy", "cfg").empty());
    CHECK(ctx, Preferences::getTestBytes("cfg_safe_legacy", "cfg_a").empty());
    CHECK(ctx, Preferences::getTestBytes("cfg_safe_legacy", "cfg_s").empty());

    Preferences::clearTestStorage();
    Preferences::setPutLimitForKey("cfg_safe_repair", "cfg_ac", 0u);
    TrackerConfigStore writer("cfg_safe_repair", "cfg");
    TrackerConfig active = makeConfig(81u);
    CHECK(ctx, writer.save(active));
    CHECK(ctx, Preferences::getTestBytes("cfg_safe_repair", "cfg_ac").empty());

    TrackerConfigStore activeReader("cfg_safe_repair", "cfg");
    activeReader.setWriteInhibited(true);
    CHECK(ctx, activeReader.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 81u);
    CHECK(ctx, Preferences::getTestBytes("cfg_safe_repair", "cfg_ac").empty());
}

void testTransitionalV2MigratesWithoutLosingMagOrCalibration(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore writer("cfg_v2_compat", "cfg");
    TrackerConfig active = makeMagEnabledCalibratedConfig();
    CHECK(ctx, active.validateSemanticConfig());
    CHECK(ctx, writer.save(active));
    CHECK(ctx, convertSlotToTransitionalV2("cfg_v2_compat", "cfg_a"));

    TrackerConfigSlotRecord source;
    CHECK(ctx, readStoredRecord("cfg_v2_compat", "cfg_a", source));
    CHECK(ctx, source.version == tracker_config_storage_detail::TRANSITIONAL_SLOT_VERSION);
    CHECK(ctx, trackerValidateConfigSlotRecord(source));
    TrackerConfig preMigration;
    preMigration.data = source.payload;
    CHECK(ctx, !preMigration.validateSemanticConfig());

    TrackerConfigStore rebooted("cfg_v2_compat", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, rebooted.load(loaded));
    CHECK(ctx, rebooted.lastLoadStatus() == TrackerConfigLoadStatus::Migrated);
    CHECK(ctx, loaded.validateSemanticConfig());
    CHECK(ctx, loaded.data.hardware.spiHz == 4000000u);
    CHECK(ctx, loaded.data.fifo.watermarkWords == 12u);
    CHECK(ctx, loaded.data.gyroCal.biasValid);
    CHECK(ctx, loaded.data.accelCal.valid);
    CHECK(ctx, loaded.data.frame.sensorToDeviceValid);
    CHECK(ctx, loaded.data.magCal.driverEnabled);
    CHECK(ctx, loaded.data.magCal.calibrationValid);
    CHECK(ctx, loaded.data.magCal.axisAlignmentValid);
    CHECK(ctx, loaded.data.magYaw.applyEnabled);
    CHECK(ctx, loaded.data.magCal.hardIron.x == active.data.magCal.hardIron.x);
    CHECK(ctx, loaded.data.magCal.expectedFieldNorm == active.data.magCal.expectedFieldNorm);
    CHECK(ctx, trackerCalibrationModelEqual(loaded, active));
    CHECK(ctx, trackerCalibrationEvidenceEqual(loaded, active));
    CHECK(ctx, (loaded.data.output.packetFormat &
                tracker_config_detail::OUTPUT_PACKET_MODE_MARKER) != 0u);

    TrackerConfigStorageInfo info;
    CHECK(ctx, rebooted.inspectStorage(info));
    CHECK(ctx, info.selectedGeneration == 2u);
    const TrackerConfigSlotInfo& selected = info.selectedSlot == TrackerConfigSlot::A
        ? info.slotA : info.slotB;
    const TrackerConfigSlotInfo& oldGood = info.selectedSlot == TrackerConfigSlot::A
        ? info.slotB : info.slotA;
    CHECK(ctx, selected.version == tracker_config_storage_detail::SLOT_VERSION);
    CHECK(ctx, selected.semanticValid);
    CHECK(ctx, !selected.migrationRequired);
    CHECK(ctx, selected.commitMarkerValid);
    CHECK(ctx, oldGood.version == tracker_config_storage_detail::TRANSITIONAL_SLOT_VERSION);
    CHECK(ctx, oldGood.migrationRequired);
}

void testTransitionalV2ReadOnlyMigrationIsVolatileAndPreservesSource(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore writer("cfg_v2_read_only", "cfg");
    TrackerConfig active = makeMagEnabledCalibratedConfig();
    CHECK(ctx, writer.save(active));
    CHECK(ctx, convertSlotToTransitionalV2("cfg_v2_read_only", "cfg_a"));
    const auto sourceBefore = Preferences::getTestBytes("cfg_v2_read_only", "cfg_a");

    TrackerConfigStore reader("cfg_v2_read_only", "cfg");
    reader.setWriteInhibited(true);
    TrackerConfig loaded;
    CHECK(ctx, reader.load(loaded));
    CHECK(ctx, reader.lastLoadStatus() == TrackerConfigLoadStatus::LoadedLegacyReadOnly);
    CHECK(ctx, loaded.validateSemanticConfig());
    CHECK(ctx, loaded.data.magCal.driverEnabled);
    CHECK(ctx, loaded.data.magCal.calibrationValid);
    CHECK(ctx, loaded.data.magYaw.applyEnabled);
    CHECK(ctx, Preferences::getTestBytes("cfg_v2_read_only", "cfg_a") == sourceBefore);
    CHECK(ctx, Preferences::getTestBytes("cfg_v2_read_only", "cfg_b").empty());
}

void testInterruptedTransitionalV2MigrationKeepsOldGood(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore writer("cfg_v2_interrupted", "cfg");
    TrackerConfig active = makeMagEnabledCalibratedConfig();
    CHECK(ctx, writer.save(active));
    CHECK(ctx, convertSlotToTransitionalV2("cfg_v2_interrupted", "cfg_a"));
    const auto oldGood = Preferences::getTestBytes("cfg_v2_interrupted", "cfg_a");
    Preferences::setGetFailuresForKey("cfg_v2_interrupted", "cfg_b", 1u);

    TrackerConfigStore interrupted("cfg_v2_interrupted", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, !interrupted.load(loaded));
    CHECK(ctx, interrupted.lastError() == TrackerConfigError::ReadFailed);
    CHECK(ctx, Preferences::getTestBytes("cfg_v2_interrupted", "cfg_a") == oldGood);

    TrackerConfigStore rebooted("cfg_v2_interrupted", "cfg");
    CHECK(ctx, rebooted.load(loaded));
    CHECK(ctx, rebooted.lastLoadStatus() == TrackerConfigLoadStatus::Migrated);
    CHECK(ctx, loaded.data.magCal.driverEnabled);
    CHECK(ctx, loaded.data.magCal.calibrationValid);
    CHECK(ctx, loaded.data.magYaw.applyEnabled);
    CHECK(ctx, Preferences::getTestBytes("cfg_v2_interrupted", "cfg_a") == oldGood);
}

void testTransitionalV2ImpossibleCalibrationStillFailsClosed(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore writer("cfg_v2_impossible", "cfg");
    TrackerConfig active = makeMagEnabledCalibratedConfig();
    CHECK(ctx, writer.save(active));
    CHECK(ctx, convertSlotToTransitionalV2("cfg_v2_impossible", "cfg_a", false));

    TrackerConfigSlotRecord record;
    CHECK(ctx, readStoredRecord("cfg_v2_impossible", "cfg_a", record));
    TrackerConfig impossible;
    impossible.data = record.payload;
    impossible.data.accelCal.scale = Mat3::diagonal(100.0f, 1.0f, 1.0f);
    impossible.updateCrc();
    record.payload = impossible.data;
    record.crc32 = 0u;
    record.crc32 = trackerConfigSlotRecordCrc(record);
    Preferences::putTestBytes("cfg_v2_impossible", "cfg_a", &record, sizeof(record));

    TrackerConfigStore rebooted("cfg_v2_impossible", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, !rebooted.load(loaded));
    CHECK(ctx, rebooted.lastError() == TrackerConfigError::CrcOrValidationFailed);
    CHECK(ctx, Preferences::getTestBytes("cfg_v2_impossible", "cfg_b").empty());
}

void testRollbackFailureDoesNotApplyOutput(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_rb_output", "cfg");
    auto old = makeConfig(50u);
    auto active = makeConfig(100u);
    CHECK(ctx, store.save(old));
    CHECK(ctx, store.save(active));
    const auto before = active.data;
    Preferences::setPutLimitForKey("cfg_rb_output", "cfg_s", 0u);
    CHECK(ctx, !store.restoreAuthoritativeGeneration(TrackerConfigSlot::A, 1u, active));
    CHECK(ctx, std::memcmp(&before, &active.data, sizeof(before)) == 0);
}

void testCurrentSlotRequiresSemanticAdmission(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore writer("cfg_semantic_load", "cfg");
    TrackerConfig good = makeConfig(100u);
    CHECK(ctx, writer.save(good));

    TrackerConfigSlotRecord record;
    CHECK(ctx, readStoredRecord("cfg_semantic_load", "cfg_a", record));
    TrackerConfig impossible;
    impossible.data = record.payload;
    impossible.data.fifo.maxWordsPerDrain = 65535u;
    impossible.updateCrc();
    record.payload = impossible.data;
    record.crc32 = 0u;
    record.crc32 = trackerConfigSlotRecordCrc(record);
    Preferences::putTestBytes("cfg_semantic_load", "cfg_a", &record, sizeof(record));

    TrackerConfigStore rebooted("cfg_semantic_load", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, !rebooted.load(loaded));
    CHECK(ctx, rebooted.lastError() == TrackerConfigError::CrcOrValidationFailed);
}

void testPreparedAuthoritativeCommitKeepsOldGoodUntilSelectorCommit(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("cfg_hw_prepare", "cfg");
    TrackerConfig active = makeConfig(50u);
    CHECK(ctx, store.save(active));

    TrackerConfig candidate = makeConfig(100u);
    TrackerPreparedConfigCommit prepared;
    CHECK(ctx, store.prepareAuthoritativeCommit(candidate, prepared));
    CHECK(ctx, prepared.valid);

    TrackerConfigStore beforeCommit("cfg_hw_prepare", "cfg");
    TrackerConfig loaded;
    CHECK(ctx, beforeCommit.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 50u);

    TrackerConfig committed;
    CHECK(ctx, store.commitPreparedAuthoritative(prepared, committed));
    CHECK(ctx, committed.data.output.outputRateHz == 100u);
    TrackerConfigStore afterCommit("cfg_hw_prepare", "cfg");
    CHECK(ctx, afterCommit.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 100u);

    TrackerConfig abortedCandidate = makeConfig(75u);
    CHECK(ctx, store.prepareAuthoritativeCommit(abortedCandidate, prepared));
    CHECK(ctx, store.abortPreparedAuthoritative(prepared));
    TrackerConfigStore afterAbort("cfg_hw_prepare", "cfg");
    CHECK(ctx, afterAbort.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == 100u);
}

} // namespace

int main() {
    TestContext ctx;
    testDualSlotSaveAndLoad(ctx);
    testTornInactiveWriteKeepsOldSelector(ctx);
    testTornSelectorKeepsPreviousActive(ctx);
    testCorruptSelectedSlotFallsBack(ctx);
    testSelectorLossUsesNewestExplicitlyCommittedGeneration(ctx);
    testLegacyV1SelectorLossUsesConservativeOlderGeneration(ctx);
    testLegacyMigration(ctx);
    testCandidateDoesNotBecomeActiveBeforeCommit(ctx);
    testPromotedCandidateMetadataWriteCanRetryWithoutRepromotion(ctx);
    testPreparedPromotionAbortedByRebootKeepsActive(ctx);
    testCandidateSignatureMismatchBlocksPromotion(ctx);
    testCandidateQualityAndWearGates(ctx);
    testCandidateSurvivesUnrelatedActiveConfigChange(ctx);
    testCandidateBecomesStaleAfterActiveCalibrationChange(ctx);
    testPreparedNewerSlotNeverBecomesFallback(ctx);
    testAbortInvalidatesPreparedSlot(ctx);
    testPromotionRejectsDivergedRuntimeCalibration(ctx);
    testPromotionPreservesUnrelatedActiveSettingsAndMeasuredQuality(ctx);
    testMigrationCleanupFailureKeepsMigratedConfig(ctx);
    testManualMigrationNeverOverwritesActive(ctx);
    testSelectorReadbackFailureReconcilesCommittedWrite(ctx);
    testPersistentSelectorReadFailureReportsUncertain(ctx);
    testLoadStatusDistinguishesStorageFailure(ctx);
    testEmptyNvsLifecycle(ctx);
    testCorruptActiveStorageIsNotClassifiedAsEmpty(ctx);
    testTransientBootReadFailureBlocksWritesUntilAuthoritativeLoad(ctx);
    testAuthoritativeLoadRequiresApplyConfirmationBeforeWrites(ctx);
    testFirstSaveReadbackFailureNeverBecomesAuthoritative(ctx);
    testInterruptedLegacyMigrationRecoversFromStillValidLegacyBlob(ctx);
    testSelectorIsAuthoritativeWhenCommitMarkerWriteFails(ctx);
    testNoOpSavePreservesGenerationAndCandidate(ctx);
    testRuntimeSnapshotNoOpAndProvenancePreservation(ctx);
    testV3CandidateFreshAcrossEvidenceAndTempPolicyChanges(ctx);
    testV2CandidateRevisionCompatibility(ctx);
    testLoadUpgradesLegacyV1ActiveSlotToV3(ctx);
    testLegacyV1CandidateUsesGenerationFreshnessContract(ctx);
    testCorruptCandidateDoesNotBreakActiveConfig(ctx);
    testDegradedStateBlocksCandidateDiscardButAllowsFullErase(ctx);
    testDormantFrameBytesDoNotChangeSensorSignature(ctx);
    testPromotionRejectsDivergedRuntimeSensorContract(ctx);
    testMeasuredAxisCandidateBeatsUnmeasuredValidAlignment(ctx);
    testSolverMeasuredAxisCandidateUsesNormalStorePromotionPath(ctx);
    testGenerationWrapComparison(ctx);
    testDeployedV3MigrationKeepsCalibrations(ctx);
    testSafeModeWriteInhibit(ctx);
    testSafeModeLoadPerformsNoRepairOrMigrationWrites(ctx);
    testTransitionalV2MigratesWithoutLosingMagOrCalibration(ctx);
    testTransitionalV2ReadOnlyMigrationIsVolatileAndPreservesSource(ctx);
    testInterruptedTransitionalV2MigrationKeepsOldGood(ctx);
    testTransitionalV2ImpossibleCalibrationStillFailsClosed(ctx);
    testRollbackFailureDoesNotApplyOutput(ctx);
    testCurrentSlotRequiresSemanticAdmission(ctx);
    testPreparedAuthoritativeCommitKeepsOldGoodUntilSelectorCommit(ctx);
    return ctx.finish("test_config_storage");
}
