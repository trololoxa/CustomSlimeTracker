#include "test_common.hpp"

#include <Preferences.h>

#include <cstring>

#include "config/tracker_network_config.hpp"

using namespace tracker;

int main() {
    TestContext ctx;
    Preferences::clearTestStorage();

    TrackerNetworkConfig cfg;
    cfg.resetDefaults();
    CHECK(ctx, cfg.validate());
    CHECK(ctx, cfg.tapUserAction() == SlimeVRUserAction::None);

    cfg.setTapUserAction(SlimeVRUserAction::YawReset);
    CHECK(ctx, cfg.validate());
    CHECK(ctx, cfg.tapUserAction() == SlimeVRUserAction::YawReset);

    TrackerNetworkConfigStore store("test_net", "cfg");
    CHECK(ctx, store.save(cfg));
    CHECK(ctx, store.lastError() == TrackerConfigError::None);

    TrackerNetworkConfig loaded;
    loaded.resetDefaults();
    CHECK(ctx, store.load(loaded));
    CHECK(ctx, loaded.validate());
    CHECK(ctx, loaded.tapUserAction() == SlimeVRUserAction::YawReset);
    CHECK(ctx, loaded.data.crc32 == cfg.data.crc32);

    TrackerNetworkConfig invalid = loaded;
    std::memset(invalid.data.deviceName, 'x', sizeof(invalid.data.deviceName));
    const TrackerNetworkConfig beforeInvalidSave = invalid;
    CHECK(ctx, !store.save(invalid));
    CHECK(ctx, std::memcmp(&invalid.data, &beforeInvalidSave.data, sizeof(invalid.data)) == 0);
    TrackerNetworkConfig stillStored;
    CHECK(ctx, store.load(stillStored));
    CHECK(ctx, stillStored.data.crc32 == loaded.data.crc32);

    // Historical default hostname is normalized only after its stored CRC has
    // been proved, then the current semantic contract is applied.
    TrackerNetworkConfig legacy;
    legacy.resetDefaults();
    std::strncpy(legacy.data.deviceName, "c3_6dsv_tracker", sizeof(legacy.data.deviceName) - 1u);
    legacy.updateCrc();
    Preferences::putTestBytes("legacy_net", "cfg", &legacy.data, sizeof(legacy.data));
    TrackerNetworkConfigStore legacyStore("legacy_net", "cfg");
    TrackerNetworkConfig migrated;
    CHECK(ctx, legacyStore.load(migrated));
    CHECK(ctx, std::strcmp(migrated.data.deviceName, "c3-6dsv-tracker") == 0);
    CHECK(ctx, migrated.validateSemanticConfig());

    TrackerNetworkConfig impossibleStored;
    impossibleStored.resetDefaults();
    std::strncpy(
        impossibleStored.data.deviceName, "invalid_name", sizeof(impossibleStored.data.deviceName) - 1u);
    impossibleStored.updateCrc();
    Preferences::putTestBytes(
        "invalid_net", "cfg", &impossibleStored.data, sizeof(impossibleStored.data));
    TrackerNetworkConfigStore invalidStore("invalid_net", "cfg");
    TrackerNetworkConfig rejected;
    CHECK(ctx, !invalidStore.load(rejected));
    CHECK(ctx, invalidStore.lastError() == TrackerConfigError::CrcOrValidationFailed);

    loaded.data.tapUserAction = 0xffu;
    loaded.updateCrc();
    CHECK(ctx, !loaded.validate());

    loaded.data.reserved1 = 1u;
    loaded.updateCrc();
    CHECK(ctx, !loaded.validateSemanticConfig());

    store.setWriteInhibited(true);
    CHECK(ctx, !store.save(cfg));
    CHECK(ctx, store.lastError() == TrackerConfigError::WriteInhibited);
    CHECK(ctx, !store.erase());
    CHECK(ctx, store.lastError() == TrackerConfigError::WriteInhibited);
    store.setWriteInhibited(false);
    CHECK(ctx, store.erase());
    CHECK(ctx, !store.load(loaded));
    CHECK(ctx, store.lastError() == TrackerConfigError::NotFound);

    return ctx.finish("tracker_network_config");
}
