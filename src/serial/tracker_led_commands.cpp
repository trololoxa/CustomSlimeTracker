#include "serial/tracker_led_commands.hpp"

#include "runtime/status_led_runtime.hpp"
#include "serial/tracker_serial_parse.hpp"
#include "serial/tracker_serial_print.hpp"

namespace tracker {
namespace {

void printLedHelp(Stream& out) {
    out.println("led status");
    out.println("led auto              - clear manual override");
    out.println("led on | off          - manual override");
    out.println("led identify [ms]     - fast blink for locating this tracker");
    out.println("led test <mode>       - manual pattern: normal|wifi|server|connection_error|sensor_error|hardware_error");
    out.println("led reset             - reset write counter");
}

void printLedStatus(Stream& out, const StatusLedRuntime& led) {
    const StatusLedRuntimeStatus s = led.status();
    out.println("# LED STATUS");
    out.print("configured="); out.println(s.configured ? "yes" : "no");
    out.print("enabled="); out.println(s.enabled ? "yes" : "no");
    out.print("pin="); out.println(static_cast<int>(s.pin));
    out.print("active_low="); out.println(s.activeLow ? "yes" : "no");
    out.print("output_on="); out.println(s.outputOn ? "yes" : "no");
    out.print("manual_override="); out.println(s.manualOverride ? "yes" : "no");
    out.print("requested_mode="); out.println(trackerStatusLedModeName(s.requestedMode));
    out.print("effective_mode="); out.println(trackerStatusLedModeName(s.effectiveMode));
    out.print("last_update_ms="); out.println(s.lastUpdateMs);
    out.print("mode_since_ms="); out.println(s.modeSinceMs);
    out.print("identify_until_ms="); out.println(s.identifyUntilMs);
    out.print("writes="); out.println(s.writes);
}

} // namespace

void trackerSerialDispatchLedCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = ctx.io ? *ctx.io : Serial;
    if (!ctx.statusLedRuntime) {
        tracker_serial_detail::printErr(out, "status LED runtime is not wired");
        return;
    }

    if (argc < 2 || tracker_serial_detail::eqIgnoreCase(argv[1], "status")) {
        printLedStatus(out, *ctx.statusLedRuntime);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "help")) {
        printLedHelp(out);
        return;
    }

    const uint32_t nowMs = millis();

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "auto")) {
        ctx.statusLedRuntime->clearManualOverride(nowMs);
        tracker_serial_detail::printOk(out, "status LED auto mode restored");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "on")) {
        ctx.statusLedRuntime->setManualOverride(TrackerStatusLedMode::ManualOn, nowMs);
        ctx.statusLedRuntime->update(nowMs);
        tracker_serial_detail::printOk(out, "status LED forced on");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "off")) {
        ctx.statusLedRuntime->setManualOverride(TrackerStatusLedMode::ManualOff, nowMs);
        ctx.statusLedRuntime->update(nowMs);
        tracker_serial_detail::printOk(out, "status LED forced off");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "identify") ||
        tracker_serial_detail::eqIgnoreCase(argv[1], "id")) {
        uint32_t durationMs = TRACKER_STATUS_LED_IDENTIFY_DEFAULT_MS;
        if (argc >= 3 && !tracker_serial_detail::parseU32(argv[2], durationMs)) {
            tracker_serial_detail::printErr(out, "usage: led identify [ms]");
            return;
        }
        ctx.statusLedRuntime->identify(nowMs, durationMs);
        ctx.statusLedRuntime->update(nowMs);
        out.print("# OK status LED identify ms="); out.println(durationMs);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "test")) {
        if (argc < 3) {
            tracker_serial_detail::printErr(out, "usage: led test <mode>");
            return;
        }
        TrackerStatusLedMode mode = TrackerStatusLedMode::Off;
        if (!trackerStatusLedModeFromName(argv[2], mode)) {
            tracker_serial_detail::printErr(out, "unknown LED mode");
            return;
        }
        ctx.statusLedRuntime->setManualOverride(mode, nowMs);
        ctx.statusLedRuntime->update(nowMs);
        out.print("# OK status LED test mode="); out.println(trackerStatusLedModeName(mode));
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "reset")) {
        ctx.statusLedRuntime->resetCounters();
        tracker_serial_detail::printOk(out, "status LED counters reset");
        return;
    }

    tracker_serial_detail::printErr(out, "unknown led command; use led help");
}

} // namespace tracker
