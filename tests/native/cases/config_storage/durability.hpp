#pragma once
// Storage durability scenarios; each complete test retains its original call order.
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

void testGenerationWrapComparison(TestContext& ctx) {
    CHECK(ctx, trackerGenerationIsNewer(1u, UINT32_MAX));
    CHECK(ctx, !trackerGenerationIsNewer(UINT32_MAX, 1u));
    CHECK(ctx, !trackerGenerationIsNewer(5u, 5u));
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
