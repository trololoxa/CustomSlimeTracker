#include "test_common.hpp"

// Regression test for an ABI/configuration bug: tracker_command_wiring.cpp used
// to see TRACKER_HAS_MOTION_LIGHT_SLEEP as undefined because its public header
// did not include the feature contract. The source then compiled a smaller
// TrackerSerialCommandContext and silently omitted this callback.
#define ARDUINO_ARCH_ESP32 1
#define TRACKER_ENABLE_MOTION_LIGHT_SLEEP 1
#include <Arduino.h>
Stream Serial;
#include "../../src/app/tracker_command_wiring.cpp"

namespace {

bool g_requested = false;

bool requestSleep(void* user) {
    g_requested = user == reinterpret_cast<void*>(0x1234);
    return g_requested;
}

} // namespace

int main() {
    TestContext ctx;

    tracker::TrackerCommandRuntimeObjects objects;
    tracker::TrackerCommandRuntimeHooks hooks;
    hooks.requestMotionLightSleep = requestSleep;
    hooks.requestMotionLightSleepUser = reinterpret_cast<void*>(0x1234);

    tracker::TrackerSerialCommandContext commandCtx;
    tracker::wireTrackerCommandContext(commandCtx, objects, hooks);

    CHECK(ctx, commandCtx.requestMotionLightSleep == requestSleep);
    CHECK(ctx, commandCtx.requestMotionLightSleepUser == reinterpret_cast<void*>(0x1234));
    CHECK(ctx, commandCtx.requestMotionLightSleep(commandCtx.requestMotionLightSleepUser));
    CHECK(ctx, g_requested);

    return ctx.finish("tracker_command_wiring_motion_sleep");
}
