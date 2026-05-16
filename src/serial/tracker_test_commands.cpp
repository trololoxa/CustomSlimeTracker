#include "serial/tracker_test_commands.hpp"

#include <Arduino.h>

#include "serial/tracker_serial_context.hpp"

namespace tracker {

Stream& trackerSerialTestStream(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

bool trackerSerialTestIs(const char* a, const char* b) {
    return tracker_serial_detail::eqIgnoreCase(a, b);
}

void trackerSerialDispatchTestCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = trackerSerialTestStream(ctx);
    if (argc < 2) {
        tracker_serial_detail::printErr(out, "usage: test static <seconds>|runtime <seconds>|stop|status");
        return;
    }

    if (trackerSerialTestIs(argv[1], "status")) {
        if (ctx.printStaticTestStatus) {
            out.println("# STATIC TEST STATUS");
            ctx.printStaticTestStatus(out, ctx.printStaticTestStatusUser);
        } else {
            tracker_serial_detail::printErr(out, "static test status hook not available");
        }
        if (ctx.printRuntimeTestStatus) {
            out.println("# RUNTIME TEST STATUS");
            ctx.printRuntimeTestStatus(out, ctx.printRuntimeTestStatusUser);
        }
        return;
    }

    if (trackerSerialTestIs(argv[1], "stop")) {
        bool stopped = false;
        if (ctx.stopStaticTest) {
            stopped = ctx.stopStaticTest(ctx.stopStaticTestUser) || stopped;
        }
        if (ctx.stopRuntimeTest) {
            stopped = ctx.stopRuntimeTest(ctx.stopRuntimeTestUser) || stopped;
        }
        if (stopped) tracker_serial_detail::printOk(out, "test stop requested");
        else tracker_serial_detail::printErr(out, "no test was running");
        return;
    }

    if (trackerSerialTestIs(argv[1], "static") || trackerSerialTestIs(argv[1], "runtime")) {
        if (argc < 3) {
            tracker_serial_detail::printErr(out, trackerSerialTestIs(argv[1], "static") ? "usage: test static <seconds>" : "usage: test runtime <seconds>");
            return;
        }
        uint32_t seconds = 0;
        if (!tracker_serial_detail::parseU32(argv[2], seconds) || seconds == 0 || seconds > 21600UL) {
            tracker_serial_detail::printErr(out, "invalid duration; expected 1..21600 seconds");
            return;
        }

        if (trackerSerialTestIs(argv[1], "static")) {
            if (!ctx.startStaticTest) {
                tracker_serial_detail::printErr(out, "static test start hook not available");
                return;
            }
            const bool ok = ctx.startStaticTest(seconds * 1000UL, ctx.startStaticTestUser);
            if (ok) tracker_serial_detail::printOk(out, "static test started");
            else tracker_serial_detail::printErr(out, "static test already running");
            return;
        }

        if (!ctx.startRuntimeTest) {
            tracker_serial_detail::printErr(out, "runtime test start hook not available");
            return;
        }
        const bool ok = ctx.startRuntimeTest(seconds * 1000UL, ctx.startRuntimeTestUser);
        if (ok) tracker_serial_detail::printOk(out, "runtime test started");
        else tracker_serial_detail::printErr(out, "runtime test already running or unavailable");
        return;
    }

    tracker_serial_detail::printErr(out, "unknown test command");
}

} // namespace tracker
