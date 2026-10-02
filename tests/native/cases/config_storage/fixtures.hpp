#pragma once
// Shared storage fixtures; included only by test_config_storage.cpp.
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
