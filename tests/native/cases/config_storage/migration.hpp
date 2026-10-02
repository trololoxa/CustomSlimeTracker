#pragma once
// Storage migration scenarios; each complete test retains its original call order.
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
