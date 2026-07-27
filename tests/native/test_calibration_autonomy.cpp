#include "test_common.hpp"

#include <cstdint>

#include "Preferences.h"
#include "config/tracker_config_store.hpp"
#include "runtime/calibration_autonomy_controller.hpp"
#include "runtime/calibration_autonomy_store.hpp"
#include "runtime/runtime_gyro_bias_controller.hpp"

using namespace tracker;

namespace {

struct ApplyContext {
    TrackerConfig* config = nullptr;
    ImuCalibration* imuCal = nullptr;
    GyroTempCompensator* temp = nullptr;
    RuntimeGyroBiasEstimator* runtimeBias = nullptr;
};

bool allowRealtime(MagDeferredServiceGate& gate, void*) {
    gate = MagDeferredServiceGate{};
    gate.allowed = true;
    return true;
}

void applyCalibration(const TrackerConfig& promoted, void* user) {
    auto& ctx = *static_cast<ApplyContext*>(user);
    trackerApplyCalibrationCandidateToConfig(*ctx.config, promoted);
    ctx.config->applyToImuCalibration(*ctx.imuCal);
    ctx.config->applyToGyroTempComp(*ctx.temp);
    runtimeBiasReset(*ctx.runtimeBias);
}

TrackerConfig makeBaselineConfig() {
    TrackerConfig config;
    config.resetDefaults();
    config.data.gyroCal.biasValid = true;
    config.data.gyroCal.biasRadS = Vec3::zero();
    config.data.accelCal.valid = true;
    config.data.accelCal.biasG = Vec3::zero();
    config.data.accelCal.scale = Mat3::identity();
    config.data.accelCalQuality.qualityScore = 0.95f;
    config.data.accelCalQuality.maxFaceNormErrorG = 0.01f;
    config.data.accelCalQuality.maxAxisResidualG = 0.01f;
    config.sanitize();
    config.updateCrc();
    return config;
}

void feedWindow(CalibrationAutonomyController& controller,
                float gyroBiasRadS,
                uint32_t nowMs,
                uint64_t& timestampUs) {
    ImuQualityResult quality;
    quality.flags = imu_quality_flags::OK;
    quality.gyroConfidence = 1.0f;
    quality.accelConfidence = 1.0f;
    quality.overallConfidence = 1.0f;
    quality.accelNormG = 1.0f;
    quality.accelNormValid = true;

    Lsm6dsv::Sample sample;
    sample.gyro_rad_s = Vec3(gyroBiasRadS, 0.0f, 0.0f);
    sample.accel_g = Vec3(0.0f, 0.0f, 1.0f);
    sample.temp_c = 25.0f;
    for (uint32_t i = 0; i < 1024u; ++i) {
        timestampUs += 1042u;
        sample.t_us = timestampUs;
        controller.observeImuSample(sample, quality, timestampUs, nowMs);
    }
}

void driveGyroCandidateToProbation(TestContext& ctx,
                                   CalibrationAutonomyController& controller,
                                   float gyroBiasRadS,
                                   uint64_t& timestampUs) {
    feedWindow(controller, gyroBiasRadS, 1u, timestampUs);
    feedWindow(controller, gyroBiasRadS, 60002u, timestampUs);
    feedWindow(controller, gyroBiasRadS, 120003u, timestampUs);
    CHECK(ctx, controller.state() == CalibrationAutonomyState::CandidateReady);
    CHECK(ctx, controller.service(120300u));
    CHECK(ctx, controller.service(120600u));
    CHECK(ctx, controller.service(120900u));
    CHECK(ctx, controller.service(121200u));
    CHECK(ctx, controller.service(121500u));
    CHECK(ctx, controller.service(121800u));
    CHECK(ctx, controller.state() == CalibrationAutonomyState::PromotedProbation);
}

void testJournalFallbackAndPreferences(TestContext& ctx) {
    Preferences::clearTestStorage();
    CalibrationAutonomyStore store("auto_journal");

    CalibrationAutonomyJournalRecord first;
    first.state = CalibrationAutonomyJournalState::PromotionPending;
    first.subsystem = CalibrationAutonomySubsystem::GyroBias;
    first.previousCalibrationRevision = 11u;
    first.targetCalibrationRevision = 22u;
    CHECK(ctx, store.writeJournal(first));

    CalibrationAutonomyJournalRecord second = first;
    second.state = CalibrationAutonomyJournalState::Probation;
    second.probationAcceptedWindows = 2u;
    CHECK(ctx, store.writeJournal(second));

    CalibrationAutonomyJournalRecord loaded;
    CHECK(ctx, store.loadJournal(loaded));
    CHECK(ctx, loaded.state == CalibrationAutonomyJournalState::Probation);
    CHECK(ctx, loaded.probationAcceptedWindows == 2u);

    // Sequence 2 is stored in journal_b. Corrupting it must expose the older,
    // still-valid write-ahead anchor from journal_a.
    CHECK(ctx, Preferences::corruptTestByte("auto_journal", "journal_b", 12u));
    CHECK(ctx, store.loadJournal(loaded));
    CHECK(ctx, loaded.state == CalibrationAutonomyJournalState::PromotionPending);
    CHECK(ctx, loaded.previousCalibrationRevision == 11u);

    CHECK(ctx, Preferences::corruptTestByte("auto_journal", "journal_a", 16u));
    CHECK(ctx, !store.loadJournal(loaded));
    CHECK(ctx, std::string(store.lastErrorName()) == "invalid_journal");

    CHECK(ctx, store.savePreferences(false, true));
    CalibrationAutonomyPreferencesRecord prefs;
    CHECK(ctx, store.loadPreferences(prefs));
    CHECK(ctx, prefs.wave0022Enabled == 0u);
    CHECK(ctx, prefs.wave0023Enabled == 1u);
}

void testProbationWriteBarrier(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("auto_barrier", "cfg");
    TrackerConfig active = makeBaselineConfig();
    CHECK(ctx, store.save(active));

    store.setAutonomyProbationWriteBarrier(true);
    TrackerConfig noOp = active;
    CHECK(ctx, store.save(noOp));

    TrackerConfig changed = active;
    changed.data.output.outputRateHz = 50u;
    changed.updateCrc();
    CHECK(ctx, !store.save(changed));
    CHECK(ctx, store.lastError() == TrackerConfigError::ApplyPending);

    store.setAutonomyProbationWriteBarrier(false);
    CHECK(ctx, store.save(changed));
}

void testDisabledAutonomyHasColdHotPath(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig config = makeBaselineConfig();
    TrackerConfigStore configStore("auto_cold", "cfg");
    CHECK(ctx, configStore.save(config));
    CalibrationAutonomyStore autonomyStore("auto_cold_j");
    CHECK(ctx, autonomyStore.savePreferences(false, false));

    ImuCalibration imuCal;
    config.applyToImuCalibration(imuCal);
    GyroTempCompensator temp;
    config.applyToGyroTempComp(temp);
    RuntimeGyroBiasEstimator runtimeBias;
    runtimeBiasReset(runtimeBias);

    CalibrationAutonomyDeps deps;
    deps.config = &config;
    deps.configStore = &configStore;
    deps.autonomyStore = &autonomyStore;
    deps.imuCal = &imuCal;
    deps.gyroTempComp = &temp;
    deps.runtimeBias = &runtimeBias;

    CalibrationAutonomyController controller;
    controller.begin(deps, 1u);
    CHECK(ctx, !controller.wave0022Enabled());
    CHECK(ctx, !controller.wave0023Enabled());
    CHECK(ctx, !controller.imuObservationRequired());
    CHECK(ctx, !controller.deferredServiceRequired());

    ImuQualityResult quality;
    quality.flags = imu_quality_flags::OK;
    quality.accelConfidence = 1.0f;
    quality.overallConfidence = 1.0f;
    Lsm6dsv::Sample sample;
    sample.gyro_rad_s = Vec3::zero();
    sample.accel_g = Vec3(0.0f, 0.0f, 1.0f);
    sample.temp_c = 25.0f;
    controller.observeImuSample(sample, quality, 1000u, 1u);
    CHECK(ctx, controller.stats().samplesObserved == 0u);
    CHECK(ctx, !controller.service(300u));
    CHECK(ctx, controller.stats().serviceCalls == 0u);

    CHECK(ctx, controller.setWave0023Enabled(true, false, 400u));
    CHECK(ctx, controller.imuObservationRequired());
    CHECK(ctx, controller.deferredServiceRequired());
}

void testExactGenerationRollback(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("auto_exact_rb", "cfg");
    TrackerConfig baseline = makeBaselineConfig();
    CHECK(ctx, store.save(baseline));
    TrackerConfigStorageInfo before;
    CHECK(ctx, store.inspectStorage(before));
    const TrackerConfigSlot oldSlot = before.selectedSlot;
    const uint32_t oldGeneration = before.selectedGeneration;

    TrackerConfig changed = baseline;
    changed.data.output.outputRateHz = 50u;
    changed.data.gyroCal.biasRadS = Vec3(0.01f, 0.0f, 0.0f);
    changed.updateCrc();
    CHECK(ctx, store.save(changed));

    TrackerConfig restored;
    CHECK(ctx, store.restoreAuthoritativeGeneration(oldSlot, oldGeneration, restored));
    CHECK(ctx, restored.data.output.outputRateHz == baseline.data.output.outputRateHz);
    CHECK_NEAR(ctx, restored.data.gyroCal.biasRadS.x, baseline.data.gyroCal.biasRadS.x, 1.0e-8f);
    store.confirmAuthoritativeConfigApplied();

    TrackerConfig loaded;
    CHECK(ctx, store.load(loaded));
    CHECK(ctx, loaded.data.output.outputRateHz == baseline.data.output.outputRateHz);
    CHECK_NEAR(ctx, loaded.data.gyroCal.biasRadS.x, baseline.data.gyroCal.biasRadS.x, 1.0e-8f);
}

void testManualOwnershipInvalidatesProposal(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig config = makeBaselineConfig();
    TrackerConfigStore configStore("auto_manual", "cfg");
    CHECK(ctx, configStore.save(config));
    CalibrationAutonomyStore autonomyStore("auto_manual_j");
    ImuCalibration imuCal;
    config.applyToImuCalibration(imuCal);
    GyroTempCompensator temp;
    config.applyToGyroTempComp(temp);
    RuntimeGyroBiasEstimator runtimeBias;
    runtimeBiasReset(runtimeBias);
    ApplyContext apply{&config, &imuCal, &temp, &runtimeBias};

    CalibrationAutonomyDeps deps;
    deps.config = &config;
    deps.configStore = &configStore;
    deps.autonomyStore = &autonomyStore;
    deps.imuCal = &imuCal;
    deps.gyroTempComp = &temp;
    deps.runtimeBias = &runtimeBias;
    deps.callbacks.evaluateRealtimeGate = allowRealtime;
    deps.callbacks.applyCalibrationConfig = applyCalibration;
    deps.callbacks.applyCalibrationConfigUser = &apply;

    CalibrationAutonomyController controller;
    controller.begin(deps, 1u);
    uint64_t timestampUs = 0u;
    constexpr float kBias = 0.10f * MATH_DEG_TO_RAD;
    feedWindow(controller, kBias, 1u, timestampUs);
    feedWindow(controller, kBias, 60002u, timestampUs);
    feedWindow(controller, kBias, 120003u, timestampUs);
    CHECK(ctx, controller.state() == CalibrationAutonomyState::CandidateReady);

    CHECK(ctx, controller.beginManualCalibration(120100u));
    config.data.gyroCal.biasRadS = Vec3(0.02f, 0.0f, 0.0f);
    config.updateCrc();
    CHECK(ctx, configStore.save(config, TrackerCalibrationProvenance::Setup));
    controller.endManualCalibration(true, 120200u);

    CHECK(ctx, !controller.service(120500u));
    bool candidateExists = true;
    CHECK(ctx, configStore.candidateExists(candidateExists));
    CHECK(ctx, !candidateExists);
    CHECK(ctx, controller.state() == CalibrationAutonomyState::Observing);
}

void testBootRecoveryBeforeSelectorCommit(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig config = makeBaselineConfig();
    TrackerConfigStore configStore("auto_boot_pre", "cfg");
    CHECK(ctx, configStore.save(config));
    CalibrationAutonomyStore autonomyStore("auto_boot_pre_j");
    ImuCalibration imuCal;
    config.applyToImuCalibration(imuCal);
    GyroTempCompensator temp;
    config.applyToGyroTempComp(temp);
    RuntimeGyroBiasEstimator runtimeBias;
    runtimeBiasReset(runtimeBias);
    ApplyContext apply{&config, &imuCal, &temp, &runtimeBias};

    CalibrationAutonomyDeps deps;
    deps.config = &config;
    deps.configStore = &configStore;
    deps.autonomyStore = &autonomyStore;
    deps.imuCal = &imuCal;
    deps.gyroTempComp = &temp;
    deps.runtimeBias = &runtimeBias;
    deps.callbacks.evaluateRealtimeGate = allowRealtime;
    deps.callbacks.applyCalibrationConfig = applyCalibration;
    deps.callbacks.applyCalibrationConfigUser = &apply;

    CalibrationAutonomyController first;
    first.begin(deps, 1u);
    uint64_t timestampUs = 0u;
    constexpr float kBias = 0.10f * MATH_DEG_TO_RAD;
    feedWindow(first, kBias, 1u, timestampUs);
    feedWindow(first, kBias, 60002u, timestampUs);
    feedWindow(first, kBias, 120003u, timestampUs);
    CHECK(ctx, first.service(120300u));
    CHECK(ctx, first.service(120600u));
    CHECK(ctx, first.service(120900u));
    CHECK(ctx, first.service(121200u)); // journal durable, selector not switched
    CHECK_NEAR(ctx, config.data.gyroCal.biasRadS.x, 0.0f, 1.0e-8f);

    CalibrationAutonomyController recovered;
    recovered.begin(deps, 121300u);
    CHECK(ctx, recovered.state() == CalibrationAutonomyState::Observing);
    CHECK(ctx, !configStore.autonomyProbationWriteBarrier());
    CalibrationAutonomyJournalRecord journal;
    CHECK(ctx, !autonomyStore.loadJournal(journal));

    // A manual/setup command owns the shared candidate slot and removes the
    // abandoned autonomy candidate before it mutates calibration state.
    CHECK(ctx, recovered.beginManualCalibration(121600u));
    bool candidateExists = true;
    CHECK(ctx, configStore.candidateExists(candidateExists));
    CHECK(ctx, !candidateExists);
    recovered.endManualCalibration(false, 121700u);
}

void testBootRecoveryAfterSelectorCommitThenManualRollback(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig config = makeBaselineConfig();
    TrackerConfigStore configStore("auto_boot_post", "cfg");
    CHECK(ctx, configStore.save(config));
    CalibrationAutonomyStore autonomyStore("auto_boot_post_j");
    ImuCalibration imuCal;
    config.applyToImuCalibration(imuCal);
    GyroTempCompensator temp;
    config.applyToGyroTempComp(temp);
    RuntimeGyroBiasEstimator runtimeBias;
    runtimeBiasReset(runtimeBias);
    ApplyContext apply{&config, &imuCal, &temp, &runtimeBias};

    CalibrationAutonomyDeps deps;
    deps.config = &config;
    deps.configStore = &configStore;
    deps.autonomyStore = &autonomyStore;
    deps.imuCal = &imuCal;
    deps.gyroTempComp = &temp;
    deps.runtimeBias = &runtimeBias;
    deps.callbacks.evaluateRealtimeGate = allowRealtime;
    deps.callbacks.applyCalibrationConfig = applyCalibration;
    deps.callbacks.applyCalibrationConfigUser = &apply;

    CalibrationAutonomyController first;
    first.begin(deps, 1u);
    uint64_t timestampUs = 0u;
    constexpr float kBias = 0.10f * MATH_DEG_TO_RAD;
    feedWindow(first, kBias, 1u, timestampUs);
    feedWindow(first, kBias, 60002u, timestampUs);
    feedWindow(first, kBias, 120003u, timestampUs);
    CHECK(ctx, first.service(120300u));
    CHECK(ctx, first.service(120600u));
    CHECK(ctx, first.service(120900u));
    CHECK(ctx, first.service(121200u)); // write-ahead journal
    CHECK(ctx, first.service(121500u)); // selector switched, probation journal not yet written
    CHECK_NEAR(ctx, config.data.gyroCal.biasRadS.x, kBias, 1.0e-7f);

    CalibrationAutonomyController recovered;
    recovered.begin(deps, 121600u);
    CHECK(ctx, recovered.state() == CalibrationAutonomyState::PromotedProbation);
    CHECK(ctx, configStore.autonomyProbationWriteBarrier());

    // Setup/manual entry must not merely request rollback. It must finish the
    // exact selector restore, candidate cleanup and journal cleanup before the
    // command receives ownership.
    CHECK(ctx, recovered.beginManualCalibration(121900u));
    CHECK(ctx, recovered.manualCalibrationLocked());
    CHECK_NEAR(ctx, config.data.gyroCal.biasRadS.x, 0.0f, 1.0e-8f);
    CHECK(ctx, !configStore.autonomyProbationWriteBarrier());
    bool candidateExists = true;
    CHECK(ctx, configStore.candidateExists(candidateExists));
    CHECK(ctx, !candidateExists);
    CalibrationAutonomyJournalRecord journal;
    CHECK(ctx, !autonomyStore.loadJournal(journal));
    recovered.endManualCalibration(false, 122000u);
}

void testPowerLossDuringAcceptCleanup(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig config = makeBaselineConfig();
    TrackerConfigStore configStore("auto_accept_power", "cfg");
    CHECK(ctx, configStore.save(config));
    CalibrationAutonomyStore autonomyStore("auto_accept_power_j");
    ImuCalibration imuCal;
    config.applyToImuCalibration(imuCal);
    GyroTempCompensator temp;
    config.applyToGyroTempComp(temp);
    RuntimeGyroBiasEstimator runtimeBias;
    runtimeBiasReset(runtimeBias);
    ApplyContext apply{&config, &imuCal, &temp, &runtimeBias};

    CalibrationAutonomyDeps deps;
    deps.config = &config;
    deps.configStore = &configStore;
    deps.autonomyStore = &autonomyStore;
    deps.imuCal = &imuCal;
    deps.gyroTempComp = &temp;
    deps.runtimeBias = &runtimeBias;
    deps.callbacks.evaluateRealtimeGate = allowRealtime;
    deps.callbacks.applyCalibrationConfig = applyCalibration;
    deps.callbacks.applyCalibrationConfigUser = &apply;

    constexpr float kBias = 0.10f * MATH_DEG_TO_RAD;
    uint64_t timestampUs = 0u;
    CalibrationAutonomyController controller;
    controller.begin(deps, 1u);
    driveGyroCandidateToProbation(ctx, controller, kBias, timestampUs);

    feedWindow(controller, kBias, 180100u, timestampUs);
    feedWindow(controller, kBias, 240200u, timestampUs);
    feedWindow(controller, kBias, 300300u, timestampUs);
    CHECK(ctx, controller.service(300600u)); // choose acceptance
    CHECK(ctx, controller.service(300900u)); // durable AcceptPending
    CHECK(ctx, controller.service(301200u)); // candidate removed, journal remains

    bool candidateExists = true;
    CHECK(ctx, configStore.candidateExists(candidateExists));
    CHECK(ctx, !candidateExists);
    CalibrationAutonomyJournalRecord journal;
    CHECK(ctx, autonomyStore.loadJournal(journal));
    CHECK(ctx, journal.state == CalibrationAutonomyJournalState::AcceptPending);

    // Simulated power loss after candidate cleanup but before journal cleanup.
    CalibrationAutonomyController recovered;
    recovered.begin(deps, 301300u);
    CHECK(ctx, recovered.state() == CalibrationAutonomyState::AcceptedCleanup);
    CHECK(ctx, configStore.autonomyProbationWriteBarrier());
    CHECK(ctx, recovered.service(301600u));
    CHECK(ctx, recovered.state() == CalibrationAutonomyState::Observing);
    CHECK(ctx, !configStore.autonomyProbationWriteBarrier());
    CHECK(ctx, !autonomyStore.loadJournal(journal));
    CHECK_NEAR(ctx, config.data.gyroCal.biasRadS.x, kBias, 1.0e-7f);
}

void testPowerLossDuringRollbackCleanup(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig config = makeBaselineConfig();
    TrackerConfigStore configStore("auto_rollback_power", "cfg");
    CHECK(ctx, configStore.save(config));
    CalibrationAutonomyStore autonomyStore("auto_rollback_power_j");
    ImuCalibration imuCal;
    config.applyToImuCalibration(imuCal);
    GyroTempCompensator temp;
    config.applyToGyroTempComp(temp);
    RuntimeGyroBiasEstimator runtimeBias;
    runtimeBiasReset(runtimeBias);
    ApplyContext apply{&config, &imuCal, &temp, &runtimeBias};

    CalibrationAutonomyDeps deps;
    deps.config = &config;
    deps.configStore = &configStore;
    deps.autonomyStore = &autonomyStore;
    deps.imuCal = &imuCal;
    deps.gyroTempComp = &temp;
    deps.runtimeBias = &runtimeBias;
    deps.callbacks.evaluateRealtimeGate = allowRealtime;
    deps.callbacks.applyCalibrationConfig = applyCalibration;
    deps.callbacks.applyCalibrationConfigUser = &apply;

    constexpr float kBias = 0.10f * MATH_DEG_TO_RAD;
    uint64_t timestampUs = 0u;
    CalibrationAutonomyController controller;
    controller.begin(deps, 1u);
    driveGyroCandidateToProbation(ctx, controller, kBias, timestampUs);

    CHECK(ctx, controller.requestRollback(
        122100u, CalibrationAutonomyRejectReason::UserDisabled));
    CHECK(ctx, controller.state() == CalibrationAutonomyState::RollbackPending);
    CHECK(ctx, controller.service(122400u)); // exact previous selector restored
    CHECK_NEAR(ctx, config.data.gyroCal.biasRadS.x, 0.0f, 1.0e-8f);

    // Power loss after selector restore, before candidate cleanup.
    CalibrationAutonomyController afterRestore;
    afterRestore.begin(deps, 122500u);
    CHECK(ctx, afterRestore.state() == CalibrationAutonomyState::RollbackPending);
    CHECK(ctx, afterRestore.service(122800u)); // candidate removed

    bool candidateExists = true;
    CHECK(ctx, configStore.candidateExists(candidateExists));
    CHECK(ctx, !candidateExists);

    // Power loss after candidate cleanup, before rejection record.
    CalibrationAutonomyController afterCandidateCleanup;
    afterCandidateCleanup.begin(deps, 122900u);
    CHECK(ctx, afterCandidateCleanup.state() == CalibrationAutonomyState::RollbackPending);
    CHECK(ctx, afterCandidateCleanup.service(123200u)); // rejection durable

    // Power loss after rejection record, before journal cleanup.
    CalibrationAutonomyController afterRejection;
    afterRejection.begin(deps, 123300u);
    CHECK(ctx, afterRejection.state() == CalibrationAutonomyState::RollbackPending);
    CHECK(ctx, afterRejection.service(123600u)); // journal cleanup + barrier release
    CHECK(ctx, afterRejection.state() == CalibrationAutonomyState::RejectedCooldown);
    CHECK(ctx, !configStore.autonomyProbationWriteBarrier());
    CalibrationAutonomyJournalRecord journal;
    CHECK(ctx, !autonomyStore.loadJournal(journal));
    CHECK_NEAR(ctx, config.data.gyroCal.biasRadS.x, 0.0f, 1.0e-8f);
}

void testGyroLifecycleAcceptsOnlyAfterFreshProbation(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig config = makeBaselineConfig();
    TrackerConfigStore configStore("auto_lifecycle", "cfg");
    CHECK(ctx, configStore.save(config));

    CalibrationAutonomyStore autonomyStore("auto_life_j");
    ImuCalibration imuCal;
    config.applyToImuCalibration(imuCal);
    GyroTempCompensator temp;
    config.applyToGyroTempComp(temp);
    RuntimeGyroBiasEstimator runtimeBias;
    runtimeBiasReset(runtimeBias);

    ApplyContext apply{&config, &imuCal, &temp, &runtimeBias};
    CalibrationAutonomyDeps deps;
    deps.config = &config;
    deps.configStore = &configStore;
    deps.autonomyStore = &autonomyStore;
    deps.imuCal = &imuCal;
    deps.gyroTempComp = &temp;
    deps.runtimeBias = &runtimeBias;
    deps.callbacks.evaluateRealtimeGate = allowRealtime;
    deps.callbacks.applyCalibrationConfig = applyCalibration;
    deps.callbacks.applyCalibrationConfigUser = &apply;

    CalibrationAutonomyController controller;
    controller.begin(deps, 1u);

    constexpr float kTrueBiasRadS = 0.10f * MATH_DEG_TO_RAD;
    uint64_t timestampUs = 0u;
    feedWindow(controller, kTrueBiasRadS, 1u, timestampUs);
    feedWindow(controller, kTrueBiasRadS, 60002u, timestampUs);
    feedWindow(controller, kTrueBiasRadS, 120003u, timestampUs);
    CHECK(ctx, controller.state() == CalibrationAutonomyState::CandidateReady);

    CHECK(ctx, controller.service(120300u)); // stage RAM candidate
    CHECK(ctx, controller.service(120600u)); // persist candidate
    CHECK(ctx, controller.service(120900u)); // compare + prepare inactive slot
    CHECK(ctx, controller.state() == CalibrationAutonomyState::PromotionPending);
    CHECK(ctx, controller.blocksMotionLightSleep());
    CHECK(ctx, controller.service(121200u)); // durable write-ahead journal
    CHECK(ctx, controller.state() == CalibrationAutonomyState::PromotionPending);
    CHECK(ctx, controller.service(121500u)); // authoritative selector commit
    CHECK(ctx, controller.state() == CalibrationAutonomyState::PromotionCommitPending);
    CHECK(ctx, controller.service(121800u)); // probation journal/checkpoint
    CHECK(ctx, controller.state() == CalibrationAutonomyState::PromotedProbation);
    CHECK_NEAR(ctx, config.data.gyroCal.biasRadS.x, kTrueBiasRadS, 1.0e-7f);

    bool candidateExists = false;
    CHECK(ctx, configStore.candidateExists(candidateExists));
    CHECK(ctx, candidateExists);
    CHECK(ctx, configStore.autonomyProbationWriteBarrier());

    // Promotion is provisional. Three new, independently separated windows
    // are required before candidate cleanup and acceptance.
    feedWindow(controller, kTrueBiasRadS, 180100u, timestampUs);
    feedWindow(controller, kTrueBiasRadS, 240200u, timestampUs);
    feedWindow(controller, kTrueBiasRadS, 300300u, timestampUs);
    CHECK(ctx, controller.state() == CalibrationAutonomyState::PromotedProbation);
    CHECK(ctx, controller.stats().probationWindows == 3u);

    CHECK(ctx, controller.service(300600u)); // enter accepted cleanup
    CHECK(ctx, controller.state() == CalibrationAutonomyState::AcceptedCleanup);
    CHECK(ctx, controller.service(300900u)); // durable AcceptPending journal
    CHECK(ctx, controller.state() == CalibrationAutonomyState::AcceptedCleanup);
    CHECK(ctx, controller.service(301200u)); // remove autonomy-owned candidate
    CHECK(ctx, controller.state() == CalibrationAutonomyState::AcceptedCleanup);
    CHECK(ctx, controller.service(301500u)); // remove journal and release barrier
    CHECK(ctx, controller.state() == CalibrationAutonomyState::Observing);
    CHECK(ctx, controller.stats().accepts == 1u);
    CHECK(ctx, !configStore.autonomyProbationWriteBarrier());
    CHECK(ctx, configStore.candidateExists(candidateExists));
    CHECK(ctx, !candidateExists);

    CalibrationAutonomyJournalRecord journal;
    CHECK(ctx, !autonomyStore.loadJournal(journal));
}


void testLegacyV1JournalRollsBackConservatively(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig baseline = makeBaselineConfig();
    TrackerConfig active = baseline;
    active.data.gyroCal.biasRadS = Vec3(0.025f, 0.0f, 0.0f);
    active.updateCrc();

    TrackerConfigStore configStore("auto_v1_cfg", "cfg");
    CHECK(ctx, configStore.save(active));
    CalibrationAutonomyStore autonomyStore("auto_v1_j");
    CHECK(ctx, autonomyStore.savePreferences(false, false));

    CalibrationAutonomyJournalRecordV1 legacy;
    legacy.sequence = 7u;
    legacy.state = CalibrationAutonomyJournalState::Probation;
    legacy.subsystem = CalibrationAutonomySubsystem::GyroBias;
    legacy.previousCalibrationRevision = trackerCalibrationPayloadRevision(baseline);
    legacy.targetCalibrationRevision = trackerCalibrationPayloadRevision(active);
    legacy.previousPayload = baseline.data;
    legacy.crc32 = CalibrationAutonomyStore::legacyJournalV1Crc(legacy);
    Preferences::putTestBytes("auto_v1_j", "journal_a", &legacy, sizeof(legacy));

    ImuCalibration imuCal;
    active.applyToImuCalibration(imuCal);
    GyroTempCompensator temp;
    active.applyToGyroTempComp(temp);
    RuntimeGyroBiasEstimator runtimeBias;
    runtimeBiasReset(runtimeBias);
    ApplyContext apply{&active, &imuCal, &temp, &runtimeBias};
    CalibrationAutonomyDeps deps;
    deps.config = &active;
    deps.configStore = &configStore;
    deps.autonomyStore = &autonomyStore;
    deps.imuCal = &imuCal;
    deps.gyroTempComp = &temp;
    deps.runtimeBias = &runtimeBias;
    deps.callbacks.evaluateRealtimeGate = allowRealtime;
    deps.callbacks.applyCalibrationConfig = applyCalibration;
    deps.callbacks.applyCalibrationConfigUser = &apply;

    CalibrationAutonomyController controller;
    controller.begin(deps, 100u);
    CHECK(ctx, controller.state() == CalibrationAutonomyState::RejectedCooldown);
    CHECK_NEAR(ctx, active.data.gyroCal.biasRadS.x, 0.0f, 1.0e-8f);
    CalibrationAutonomyJournalRecord current;
    CHECK(ctx, !autonomyStore.loadJournal(current));
    CHECK(ctx, autonomyStore.lastErrorIsNotFound());
}


void testPowerLossDuringFullEraseRecoversCleanPolicy(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig config = makeBaselineConfig();
    config.data.output.outputRateHz = 50u;
    config.updateCrc();
    TrackerConfigStore configStore("auto_erase_cfg", "cfg");
    CHECK(ctx, configStore.save(config));
    CalibrationAutonomyStore autonomyStore("auto_erase_j");
    CHECK(ctx, autonomyStore.savePreferences(false, false));

    ImuCalibration imuCal;
    config.applyToImuCalibration(imuCal);
    GyroTempCompensator temp;
    config.applyToGyroTempComp(temp);
    RuntimeGyroBiasEstimator runtimeBias;
    runtimeBiasReset(runtimeBias);
    CalibrationAutonomyDeps deps;
    deps.config = &config;
    deps.configStore = &configStore;
    deps.autonomyStore = &autonomyStore;
    deps.imuCal = &imuCal;
    deps.gyroTempComp = &temp;
    deps.runtimeBias = &runtimeBias;

    CalibrationAutonomyController first;
    first.begin(deps, 1u);
    TrackerConfig clean = config;
    clean.clearAllCalibrationPreservingPolicy();
    clean.sanitize();
    clean.updateCrc();
    CHECK(ctx, first.preparePersistentCalibrationErase(clean));

    // Power disappears after destructive slot erase but before the CLI can save
    // the clean config or clear the marker.
    CHECK(ctx, configStore.erase());

    TrackerConfig bootConfig;
    bool loaded = true;
    TrackerConfigStore bootStore("auto_erase_cfg", "cfg");
    CHECK(ctx, bootStore.loadOrDefaults(bootConfig, &loaded));
    CHECK(ctx, !loaded);
    ImuCalibration bootImu;
    bootConfig.applyToImuCalibration(bootImu);
    GyroTempCompensator bootTemp;
    bootConfig.applyToGyroTempComp(bootTemp);
    RuntimeGyroBiasEstimator bootBias;
    runtimeBiasReset(bootBias);
    CalibrationAutonomyDeps bootDeps;
    bootDeps.config = &bootConfig;
    bootDeps.configStore = &bootStore;
    bootDeps.autonomyStore = &autonomyStore;
    bootDeps.imuCal = &bootImu;
    bootDeps.gyroTempComp = &bootTemp;
    bootDeps.runtimeBias = &bootBias;

    CalibrationAutonomyController rebooted;
    rebooted.begin(bootDeps, 2u);
    CHECK(ctx, rebooted.state() == CalibrationAutonomyState::Observing);
    CHECK(ctx, !bootConfig.data.gyroCal.biasValid);
    CHECK(ctx, !bootConfig.data.accelCal.valid);
    CHECK(ctx, bootConfig.data.output.outputRateHz == 50u);
    CalibrationAutonomyEraseRecoveryRecord marker;
    CHECK(ctx, !autonomyStore.loadEraseRecovery(marker));
    CHECK(ctx, autonomyStore.lastErrorIsNotFound());
    TrackerConfig persisted;
    CHECK(ctx, bootStore.load(persisted));
    CHECK(ctx, !persisted.data.gyroCal.biasValid);
    CHECK(ctx, persisted.data.output.outputRateHz == 50u);
}

void testForceEraseRecoveryClearsInvalidJournal(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfig config = makeBaselineConfig();
    TrackerConfigStore configStore("auto_force_cfg", "cfg");
    CHECK(ctx, configStore.save(config));
    CalibrationAutonomyStore autonomyStore("auto_force_j");
    CHECK(ctx, autonomyStore.savePreferences(false, false));
    const uint32_t garbage = 0xDEADBEEFu;
    Preferences::putTestBytes("auto_force_j", "journal_a", &garbage, sizeof(garbage));

    ImuCalibration imuCal;
    config.applyToImuCalibration(imuCal);
    GyroTempCompensator temp;
    config.applyToGyroTempComp(temp);
    RuntimeGyroBiasEstimator runtimeBias;
    runtimeBiasReset(runtimeBias);
    CalibrationAutonomyDeps deps;
    deps.config = &config;
    deps.configStore = &configStore;
    deps.autonomyStore = &autonomyStore;
    deps.imuCal = &imuCal;
    deps.gyroTempComp = &temp;
    deps.runtimeBias = &runtimeBias;

    CalibrationAutonomyController controller;
    controller.begin(deps, 1u);
    CHECK(ctx, controller.state() == CalibrationAutonomyState::SuspendedStorage);
    // Corrupt/obsolete storage is fail-closed for autonomous writes, but it is
    // passive: light sleep preserves RAM and must remain available for battery
    // protection while the user has not yet issued destructive recovery.
    CHECK(ctx, !controller.blocksMotionLightSleep());
    CHECK(ctx, !controller.deferredServiceRequired());
    CHECK(ctx, !controller.service(300u));
    CHECK(ctx, controller.stats().serviceCalls == 0u);
    CHECK(ctx, controller.forceClearPersistentCalibrationStateForErase(2u));
    CHECK(ctx, controller.state() == CalibrationAutonomyState::Observing);
    CalibrationAutonomyJournalRecord journal;
    CHECK(ctx, !autonomyStore.loadJournal(journal));
    CHECK(ctx, autonomyStore.lastErrorIsNotFound());
    CalibrationAutonomyPreferencesRecord prefs;
    CHECK(ctx, autonomyStore.loadPreferences(prefs));
    CHECK(ctx, prefs.wave0022Enabled == 0u);
    CHECK(ctx, prefs.wave0023Enabled == 0u);
}

} // namespace

int main() {
    TestContext ctx;
    testJournalFallbackAndPreferences(ctx);
    testLegacyV1JournalRollsBackConservatively(ctx);
    testForceEraseRecoveryClearsInvalidJournal(ctx);
    testPowerLossDuringFullEraseRecoversCleanPolicy(ctx);
    testProbationWriteBarrier(ctx);
    testDisabledAutonomyHasColdHotPath(ctx);
    testExactGenerationRollback(ctx);
    testManualOwnershipInvalidatesProposal(ctx);
    testBootRecoveryBeforeSelectorCommit(ctx);
    testBootRecoveryAfterSelectorCommitThenManualRollback(ctx);
    testPowerLossDuringAcceptCleanup(ctx);
    testPowerLossDuringRollbackCleanup(ctx);
    testGyroLifecycleAcceptsOnlyAfterFreshProbation(ctx);
    return ctx.finish("calibration_autonomy");
}
