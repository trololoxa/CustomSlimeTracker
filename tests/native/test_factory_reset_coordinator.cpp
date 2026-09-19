#include "test_common.hpp"

#include <Preferences.h>

#include <cstring>

#include "config/factory_reset_coordinator.hpp"

using namespace tracker;

static TrackerConfig calibratedConfig() {
    TrackerConfig config;
    config.resetDefaults();
    config.data.gyroCal.biasValid = true;
    config.data.gyroCal.biasRadS = Vec3(0.01f, -0.02f, 0.03f);
    config.data.accelCal.valid = true;
    config.data.accelCal.biasG = Vec3(0.01f, 0.02f, -0.01f);
    config.data.accelCal.scale = Mat3::identity();
    config.updateCrc();
    return config;
}

static TrackerNetworkConfig networkConfig() {
    TrackerNetworkConfig network;
    network.resetDefaults();
    std::strncpy(network.data.ssid, "test", sizeof(network.data.ssid) - 1u);
    network.data.credentialsValid = true;
    network.data.wifiEnabled = true;
    network.updateCrc();
    return network;
}

static void testCalibrationScopePreservesNetwork(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore configStore("fr_cfg", "cfg");
    TrackerNetworkConfigStore networkStore("fr_net", "net");
    CalibrationAutonomyStore autonomyStore("fr_auto");
    FactoryResetCoordinator coordinator(configStore, networkStore, &autonomyStore, "fr_marker");

    TrackerConfig config = calibratedConfig();
    CHECK(ctx, configStore.save(config));
    TrackerNetworkConfig network = networkConfig();
    CHECK(ctx, networkStore.save(network));
    CHECK(ctx, autonomyStore.savePreferences(false, true));

    CHECK(ctx, coordinator.request(FactoryResetScope::Calibration));
    bool pending = true;
    CHECK(ctx, coordinator.pending(pending));
    CHECK(ctx, !pending);

    TrackerConfig clean;
    CHECK(ctx, configStore.load(clean));
    CHECK(ctx, !clean.data.gyroCal.biasValid);
    CHECK(ctx, !clean.data.accelCal.valid);
    TrackerNetworkConfig retained;
    CHECK(ctx, networkStore.load(retained));
    CHECK(ctx, std::strcmp(retained.data.ssid, "test") == 0);
    CalibrationAutonomyPreferencesRecord preferences;
    CHECK(ctx, !autonomyStore.loadPreferences(preferences));
    CHECK(ctx, autonomyStore.lastErrorIsNotFound());
}

static void testInterruptedFullResetResumes(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore configStore("fr_cfg", "cfg");
    TrackerNetworkConfigStore networkStore("fr_net", "net");
    CalibrationAutonomyStore autonomyStore("fr_auto");
    FactoryResetCoordinator coordinator(configStore, networkStore, &autonomyStore, "fr_marker");

    TrackerConfig config = calibratedConfig();
    CHECK(ctx, configStore.save(config));
    TrackerNetworkConfig network = networkConfig();
    CHECK(ctx, networkStore.save(network));
    CHECK(ctx, autonomyStore.savePreferences(true, false));

    Preferences::setRemoveFailureForKey("fr_net", "net");
    CHECK(ctx, !coordinator.request(FactoryResetScope::Full));
    CHECK(ctx, coordinator.lastError() == FactoryResetError::NetworkResetFailed);
    bool pending = false;
    CHECK(ctx, coordinator.pending(pending));
    CHECK(ctx, pending);
    const auto markerBytes = Preferences::getTestBytes("fr_marker", "pending");
    CHECK(ctx, markerBytes.size() == factory_reset_detail::MARKER_ENCODED_SIZE);

    FactoryResetCoordinator afterReboot(
        configStore, networkStore, &autonomyStore, "fr_marker");
    CHECK(ctx, afterReboot.resumePending());
    CHECK(ctx, afterReboot.pending(pending));
    CHECK(ctx, !pending);
    TrackerConfig missingConfig;
    CHECK(ctx, !configStore.load(missingConfig));
    CHECK(ctx, configStore.lastError() == TrackerConfigError::NotFound);
    TrackerNetworkConfig missingNetwork;
    CHECK(ctx, !networkStore.load(missingNetwork));
    CHECK(ctx, networkStore.lastError() == TrackerConfigError::NotFound);
    CalibrationAutonomyPreferencesRecord missingPreferences;
    CHECK(ctx, !autonomyStore.loadPreferences(missingPreferences));
    CHECK(ctx, autonomyStore.lastErrorIsNotFound());
}

static void testCorruptMarkerIsFailClosedButReplaceable(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore configStore("fr_cfg", "cfg");
    TrackerNetworkConfigStore networkStore("fr_net", "net");
    CalibrationAutonomyStore autonomyStore("fr_auto");
    FactoryResetCoordinator coordinator(configStore, networkStore, &autonomyStore, "fr_marker");

    TrackerConfig config = calibratedConfig();
    CHECK(ctx, configStore.save(config));
    TrackerNetworkConfig network = networkConfig();
    CHECK(ctx, networkStore.save(network));

    Preferences::setRemoveFailureForKey("fr_net", "net");
    CHECK(ctx, !coordinator.request(FactoryResetScope::Full));
    CHECK(ctx, Preferences::corruptTestByte("fr_marker", "pending", 10u, 0x01u));

    FactoryResetCoordinator afterReboot(
        configStore, networkStore, &autonomyStore, "fr_marker");
    CHECK(ctx, !afterReboot.resumePending());
    CHECK(ctx, afterReboot.lastError() == FactoryResetError::MarkerInvalid);
    TrackerNetworkConfig retained;
    CHECK(ctx, networkStore.load(retained));

    // A new explicit, confirmed request may replace an unexecutable marker.
    CHECK(ctx, afterReboot.request(FactoryResetScope::Network));
    CHECK(ctx, !networkStore.load(retained));
    CHECK(ctx, networkStore.lastError() == TrackerConfigError::NotFound);
}

static void testPendingScopeCannotBeSilentlyReinterpreted(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore configStore("fr_cfg", "cfg");
    TrackerNetworkConfigStore networkStore("fr_net", "net");
    CalibrationAutonomyStore autonomyStore("fr_auto");
    FactoryResetCoordinator coordinator(configStore, networkStore, &autonomyStore, "fr_marker");

    TrackerConfig config = calibratedConfig();
    CHECK(ctx, configStore.save(config));
    TrackerNetworkConfig network = networkConfig();
    CHECK(ctx, networkStore.save(network));
    Preferences::setRemoveFailureForKey("fr_net", "net");
    CHECK(ctx, !coordinator.request(FactoryResetScope::Full));

    FactoryResetScope pendingScope = FactoryResetScope::Main;
    bool pending = false;
    CHECK(ctx, coordinator.pending(pending, &pendingScope));
    CHECK(ctx, pending);
    CHECK(ctx, pendingScope == FactoryResetScope::Full);
    CHECK(ctx, !coordinator.request(FactoryResetScope::Network));
    CHECK(ctx, coordinator.lastError() == FactoryResetError::PendingScopeMismatch);
}

static void testConfirmedResetIsSafeModeEscape(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore configStore("fr_cfg", "cfg");
    TrackerNetworkConfigStore networkStore("fr_net", "net");
    CalibrationAutonomyStore autonomyStore("fr_auto");
    FactoryResetCoordinator coordinator(configStore, networkStore, &autonomyStore, "fr_marker");
    TrackerConfig config = calibratedConfig();
    CHECK(ctx, configStore.save(config));

    configStore.setWriteInhibited(true);
    configStore.setAutonomyProbationWriteBarrier(true);
    networkStore.setWriteInhibited(true);
    autonomyStore.setWriteInhibited(true);
    CHECK(ctx, coordinator.request(FactoryResetScope::Main));
    CHECK(ctx, configStore.writeInhibited());
    CHECK(ctx, configStore.autonomyProbationWriteBarrier());
    CHECK(ctx, networkStore.writeInhibited());
    CHECK(ctx, autonomyStore.writeInhibited());
    TrackerConfig missing;
    CHECK(ctx, !configStore.load(missing));
    CHECK(ctx, configStore.lastError() == TrackerConfigError::NotFound);
}

static void testPendingResetResumesThroughSafeMode(TestContext& ctx) {
    Preferences::clearTestStorage();
    TrackerConfigStore configStore("fr_cfg", "cfg");
    TrackerNetworkConfigStore networkStore("fr_net", "net");
    CalibrationAutonomyStore autonomyStore("fr_auto");
    FactoryResetCoordinator coordinator(configStore, networkStore, &autonomyStore, "fr_marker");
    TrackerConfig config = calibratedConfig();
    CHECK(ctx, configStore.save(config));
    TrackerNetworkConfig network = networkConfig();
    CHECK(ctx, networkStore.save(network));

    Preferences::setRemoveFailureForKey("fr_net", "net");
    CHECK(ctx, !coordinator.request(FactoryResetScope::Full));

    configStore.setWriteInhibited(true);
    configStore.setAutonomyProbationWriteBarrier(true);
    networkStore.setWriteInhibited(true);
    autonomyStore.setWriteInhibited(true);
    FactoryResetCoordinator afterReboot(
        configStore, networkStore, &autonomyStore, "fr_marker");
    CHECK(ctx, afterReboot.resumePending());
    CHECK(ctx, configStore.writeInhibited());
    CHECK(ctx, configStore.autonomyProbationWriteBarrier());
    CHECK(ctx, networkStore.writeInhibited());
    CHECK(ctx, autonomyStore.writeInhibited());
    bool pending = true;
    CHECK(ctx, afterReboot.pending(pending));
    CHECK(ctx, !pending);
}

int main() {
    TestContext ctx;
    testCalibrationScopePreservesNetwork(ctx);
    testInterruptedFullResetResumes(ctx);
    testCorruptMarkerIsFailClosedButReplaceable(ctx);
    testConfirmedResetIsSafeModeEscape(ctx);
    testPendingResetResumesThroughSafeMode(ctx);
    testPendingScopeCannotBeSilentlyReinterpreted(ctx);
    return ctx.finish("factory_reset_coordinator");
}
