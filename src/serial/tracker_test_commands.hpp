#pragma once

#include <Arduino.h>

#include "serial/tracker_serial_context.hpp"

namespace tracker {

inline Stream& trackerSerialTestStream(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

inline bool trackerSerialTestIs(const char* a, const char* b) {
    return tracker_serial_detail::eqIgnoreCase(a, b);
}

inline void trackerSerialDispatchTestCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = trackerSerialTestStream(ctx);
    if (argc < 2) {
        tracker_serial_detail::printErr(out, "usage: test static <seconds>|stop|status");
        return;
    }

    if (trackerSerialTestIs(argv[1], "status")) {
        if (ctx.printStaticTestStatus) {
            ctx.printStaticTestStatus(out, ctx.printStaticTestStatusUser);
        } else {
            tracker_serial_detail::printErr(out, "static test status hook not available");
        }
        return;
    }

    if (trackerSerialTestIs(argv[1], "stop")) {
        if (!ctx.stopStaticTest) {
            tracker_serial_detail::printErr(out, "static test stop hook not available");
            return;
        }
        const bool ok = ctx.stopStaticTest(ctx.stopStaticTestUser);
        if (ok) tracker_serial_detail::printOk(out, "static test stop requested");
        else tracker_serial_detail::printErr(out, "static test was not running");
        return;
    }

    if (trackerSerialTestIs(argv[1], "static")) {
        if (argc < 3) {
            tracker_serial_detail::printErr(out, "usage: test static <seconds>");
            return;
        }
        if (!ctx.startStaticTest) {
            tracker_serial_detail::printErr(out, "static test start hook not available");
            return;
        }
        uint32_t seconds = 0;
        if (!tracker_serial_detail::parseU32(argv[2], seconds) || seconds == 0 || seconds > 21600UL) {
            tracker_serial_detail::printErr(out, "invalid duration; expected 1..21600 seconds");
            return;
        }
        const bool ok = ctx.startStaticTest(seconds * 1000UL, ctx.startStaticTestUser);
        if (ok) tracker_serial_detail::printOk(out, "static test started");
        else tracker_serial_detail::printErr(out, "static test already running");
        return;
    }

    tracker_serial_detail::printErr(out, "unknown test command");
}

} // namespace tracker
