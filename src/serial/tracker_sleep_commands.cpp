#include "serial/tracker_sleep_commands.hpp"

#if TRACKER_HAS_MOTION_LIGHT_SLEEP

#include <Arduino.h>

namespace tracker {

void trackerSerialDispatchMotionLightSleepCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = ctx.io ? *ctx.io : Serial;

    if (argc != 1) {
        tracker_serial_detail::printErr(out, "usage: sleep");
        return;
    }

    if (ctx.requestMotionLightSleep == nullptr) {
        tracker_serial_detail::printErr(out, "motion light sleep is not wired");
        return;
    }

    if (!ctx.requestMotionLightSleep(ctx.requestMotionLightSleepUser)) {
        tracker_serial_detail::printErr(out, "motion light sleep is unavailable while tracker is recovering or testing");
        return;
    }

    // The app performs the destructive transition after cli->poll() returns.
    // Do not call esp_light_sleep_start() from this dispatcher callback.
    out.println("# OK motion_light_sleep=queued; move tracker to wake");
}

} // namespace tracker

#endif // TRACKER_HAS_MOTION_LIGHT_SLEEP
