#include "test_common.hpp"

#include <Preferences.h>

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

    loaded.data.tapUserAction = 0xffu;
    loaded.updateCrc();
    CHECK(ctx, !loaded.validate());
    loaded.sanitize();
    CHECK(ctx, loaded.validate());
    CHECK(ctx, loaded.tapUserAction() == SlimeVRUserAction::None);

    CHECK(ctx, store.erase());
    CHECK(ctx, !store.load(loaded));
    CHECK(ctx, store.lastError() == TrackerConfigError::NotFound);

    return ctx.finish("tracker_network_config");
}
