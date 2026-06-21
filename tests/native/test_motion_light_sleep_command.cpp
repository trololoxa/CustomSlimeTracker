#include "test_common.hpp"

// Compile this one handler with the feature enabled. The normal native suite
// uses the Debug default (sleep disabled), which also verifies that production
// builds do not acquire this CLI surface accidentally.
#define ARDUINO_ARCH_ESP32 1
#define TRACKER_ENABLE_MOTION_LIGHT_SLEEP 1
#include <Arduino.h>
Stream Serial;
#include "../../src/serial/tracker_sleep_commands.cpp"

namespace {

int g_requests = 0;
bool g_accept = true;

bool requestSleep(void*) {
    ++g_requests;
    return g_accept;
}

} // namespace

int main() {
    TestContext ctx;
    Stream out;
    tracker::TrackerSerialCommandContext commandCtx;
    commandCtx.io = &out;
    commandCtx.requestMotionLightSleep = requestSleep;

    char command[] = "sleep";
    char* argv[] = {command};
    tracker::trackerSerialDispatchMotionLightSleepCommand(commandCtx, 1, argv);
    CHECK(ctx, g_requests == 1);

    g_accept = false;
    tracker::trackerSerialDispatchMotionLightSleepCommand(commandCtx, 1, argv);
    CHECK(ctx, g_requests == 2);

    char invalidCommand[] = "sleep";
    char extra[] = "now";
    char* invalidArgv[] = {invalidCommand, extra};
    tracker::trackerSerialDispatchMotionLightSleepCommand(commandCtx, 2, invalidArgv);
    CHECK(ctx, g_requests == 2);

    return ctx.finish("motion_light_sleep_command");
}
