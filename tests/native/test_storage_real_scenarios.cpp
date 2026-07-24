#include "test_common.hpp"

#include <cstddef>

#include "Preferences.h"
#include "config/tracker_config_store.hpp"

using namespace tracker;

namespace {

TrackerConfig makeScenarioConfig(uint16_t rate, bool calibrated = true) {
    TrackerConfig config;
    config.resetDefaults();
    config.data.output.outputRateHz = rate;
    if (calibrated) {
        config.data.gyroCal.biasValid = true;
        config.data.gyroCal.biasRadS = Vec3(0.1f, 0.2f, 0.3f);
        config.data.accelCal.valid = true;
        config.data.accelCal.scale = Mat3::identity();
        config.data.accelCal.biasG = Vec3(0.01f, 0.02f, 0.03f);
        config.data.accelCalQuality.qualityScore = 0.8f;
    }
    config.updateCrc();
    return config;
}

TrackerCalibrationCandidateMetadata goodCandidateMetadata() {
    TrackerCalibrationCandidateMetadata metadata;
    metadata.provenance = TrackerCalibrationProvenance::Background;
    metadata.quality.overallScore = 0.95f;
    metadata.quality.gyroScore = 0.95f;
    metadata.quality.accelScore = 0.95f;
    metadata.quality.magScore = 0.0f;
    metadata.quality.alignmentScore = 0.0f;
    metadata.quality.coverageScore = 0.0f;
    metadata.quality.gyroResidualDps = 0.01f;
    metadata.quality.accelResidualG = 0.01f;
    metadata.quality.magResidual = 0.0f;
    return metadata;
}

void emptyNvs(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("life_empty", "cfg");
    TrackerConfig config;
    bool loaded = true;
    CHECK(ctx, store.loadOrDefaults(config, &loaded));
    CHECK(ctx, !loaded);
    CHECK(ctx, store.lastLoadStatus() == TrackerConfigLoadStatus::DefaultsNotFound);
    CHECK(ctx, store.save(config));
}

void corruptedNvs(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore writer("life_corrupt", "cfg");
    TrackerConfig good = makeScenarioConfig(77);
    CHECK(ctx, writer.save(good));
    CHECK(ctx, Preferences::corruptTestByte(
        "life_corrupt", "cfg_a", offsetof(TrackerConfigSlotRecord, payload)));

    TrackerConfigStore boot("life_corrupt", "cfg");
    TrackerConfig defaults;
    bool loaded = true;
    CHECK(ctx, boot.loadOrDefaults(defaults, &loaded));
    CHECK(ctx, !loaded);
    CHECK(ctx, boot.lastLoadStatus() == TrackerConfigLoadStatus::DefaultsStorageError);
    CHECK(ctx, boot.lastLoadError() == TrackerConfigError::CrcOrValidationFailed);
    CHECK(ctx, !boot.save(defaults));
    CHECK(ctx, boot.lastError() == TrackerConfigError::StorageDegraded);
}

void transientReadFailure(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore writer("life_transient", "cfg");
    TrackerConfig good = makeScenarioConfig(77);
    CHECK(ctx, writer.save(good));

    Preferences::setNextBeginFailures(1u);
    TrackerConfigStore boot("life_transient", "cfg");
    TrackerConfig runtime;
    CHECK(ctx, boot.loadOrDefaults(runtime));
    CHECK(ctx, boot.lastLoadStatus() == TrackerConfigLoadStatus::DefaultsStorageError);

    TrackerConfig probe;
    CHECK(ctx, boot.verify(probe));
    CHECK(ctx, probe.data.gyroCal.biasValid);
    CHECK(ctx, !boot.save(runtime));
    CHECK(ctx, boot.lastError() == TrackerConfigError::StorageDegraded);

    TrackerConfig recovered;
    CHECK(ctx, boot.load(recovered));
    CHECK(ctx, !boot.save(recovered));
    CHECK(ctx, boot.lastError() == TrackerConfigError::StorageDegraded);
    boot.confirmAuthoritativeConfigApplied();
    CHECK(ctx, boot.save(recovered));
}

void firstSavePowerLossWindow(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("life_first", "cfg");
    Preferences::setGetFailuresForKey("life_first", "cfg_a", 1u);
    TrackerConfig good = makeScenarioConfig(66);
    CHECK(ctx, !store.save(good));

    TrackerConfigStore rebooted("life_first", "cfg");
    TrackerConfig defaults;
    bool loaded = true;
    CHECK(ctx, rebooted.loadOrDefaults(defaults, &loaded));
    CHECK(ctx, !loaded);
    CHECK(ctx, rebooted.lastLoadStatus() == TrackerConfigLoadStatus::DefaultsStorageError);
}

void noOpAndCandidate(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore store("life_noop", "cfg");
    TrackerConfig active = makeScenarioConfig(100);
    CHECK(ctx, store.save(active));

    TrackerConfig candidate = active;
    candidate.data.accelCal.biasG.x = 0.005f;
    candidate.updateCrc();
    CHECK(ctx, store.stageCandidate(candidate, goodCandidateMetadata(), 1000u));

    TrackerConfigStorageInfo before;
    CHECK(ctx, store.inspectStorage(before));
    CHECK(ctx, store.save(active));
    TrackerConfigStorageInfo after;
    CHECK(ctx, store.inspectStorage(after));
    CHECK(ctx, before.selectedGeneration == after.selectedGeneration);

    TrackerPreparedConfigPromotion prepared;
    TrackerConfig composed;
    CHECK(ctx, store.prepareCandidatePromotion(prepared, composed, true));
}

} // namespace

int main() {
    TestContext ctx;
    emptyNvs(ctx);
    corruptedNvs(ctx);
    transientReadFailure(ctx);
    firstSavePowerLossWindow(ctx);
    noOpAndCandidate(ctx);
    return ctx.finish("storage_real_scenarios");
}
