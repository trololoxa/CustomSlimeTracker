#pragma once
// Storage candidates scenarios; each complete test retains its original call order.
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
