#pragma once
// Storage admission scenarios; each complete test retains its original call order.
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
