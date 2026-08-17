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
        tracker_serial_detail::printErr(out, "usage: test static <seconds>|runtime <seconds>|summary static|runtime|stop|status");
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

    if (trackerSerialTestIs(argv[1], "summary")) {
        if (argc != 3) {
            tracker_serial_detail::printErr(out, "usage: test summary static|runtime");
            return;
        }
        if (trackerSerialTestIs(argv[2], "static")) {
            if (!ctx.printStaticTestSummary) {
                tracker_serial_detail::printErr(out, "static test summary hook not available");
                return;
            }
            (void)ctx.printStaticTestSummary(out, ctx.printStaticTestSummaryUser);
            return;
        }
        if (trackerSerialTestIs(argv[2], "runtime")) {
            if (!ctx.printRuntimeTestSummary) {
                tracker_serial_detail::printErr(out, "runtime test summary hook not available");
                return;
            }
            (void)ctx.printRuntimeTestSummary(out, ctx.printRuntimeTestSummaryUser);
            return;
        }
        tracker_serial_detail::printErr(out, "usage: test summary static|runtime");
        return;
    }

    if (trackerSerialTestIs(argv[1], "report")) {
        if (argc != 3) {
            tracker_serial_detail::printErr(out, "usage: test report static|runtime");
            return;
        }
        if (trackerSerialTestIs(argv[2], "static")) {
            if (!ctx.printStaticTestReport) {
                tracker_serial_detail::printErr(out, "static test report hook not available");
                return;
            }
            (void)ctx.printStaticTestReport(out, ctx.printStaticTestReportUser);
            return;
        }
        if (trackerSerialTestIs(argv[2], "runtime")) {
            if (!ctx.printRuntimeTestReport) {
                tracker_serial_detail::printErr(out, "runtime test report hook not available");
                return;
            }
            (void)ctx.printRuntimeTestReport(out, ctx.printRuntimeTestReportUser);
            return;
        }
        tracker_serial_detail::printErr(out, "usage: test report static|runtime");
        return;
    }

    if (trackerSerialTestIs(argv[1], "stop")) {
        bool stopped = false;
        constexpr bool force = true;
        if (ctx.stopStaticTest) {
            stopped = ctx.stopStaticTest(out, force, ctx.stopStaticTestUser) || stopped;
        }
        if (ctx.stopRuntimeTest) {
            stopped = ctx.stopRuntimeTest(out, force, ctx.stopRuntimeTestUser) || stopped;
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
        constexpr uint32_t maxSeconds = 21600UL;
        if (!tracker_serial_detail::parseU32(argv[2], seconds) ||
            seconds == 0 || seconds > maxSeconds) {
            tracker_serial_detail::printErr(out, "invalid duration; expected 1..21600 seconds");
            return;
        }

        if (trackerSerialTestIs(argv[1], "static")) {
            if (!ctx.startStaticTest) {
                tracker_serial_detail::printErr(out, "static test start hook not available");
                return;
            }
            const bool ok = ctx.startStaticTest(seconds * 1000UL, out, ctx.startStaticTestUser);
            if (ok) tracker_serial_detail::printOk(out, "static test started");
            else tracker_serial_detail::printErr(out, "static test already running");
            return;
        }

        if (!ctx.startRuntimeTest) {
            tracker_serial_detail::printErr(out, "runtime test start hook not available");
            return;
        }
        const bool ok = ctx.startRuntimeTest(seconds * 1000UL, out, ctx.startRuntimeTestUser);
        if (ok) tracker_serial_detail::printOk(out, "runtime test started");
        else tracker_serial_detail::printErr(out, "runtime test already running or unavailable");
        return;
    }

    tracker_serial_detail::printErr(out, "unknown test command");
}

} // namespace tracker
