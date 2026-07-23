#include "serial/tracker_tap_commands.hpp"

#include "runtime/tap_runtime_controller.hpp"
#include "serial/tracker_serial_parse.hpp"
#include "serial/tracker_serial_print.hpp"

namespace tracker {
namespace {

uint8_t parseTapCount(const char* token) {
    if (!token || token[0] == '\0') return TRACKER_TAP_VALUE_MIN;
    if (tracker_serial_detail::eqIgnoreCase(token, "double") ||
        tracker_serial_detail::eqIgnoreCase(token, "doubletap")) {
        return 2;
    }
    if (tracker_serial_detail::eqIgnoreCase(token, "single") ||
        tracker_serial_detail::eqIgnoreCase(token, "tap")) {
        return 1;
    }
    uint32_t parsed = 0;
    if (!tracker_serial_detail::parseU32(token, parsed) || parsed > 255u) return 0;
    return static_cast<uint8_t>(parsed);
}

void printHexByte(Stream& out, uint8_t value) {
    out.print("0x");
    if (value < 0x10u) out.print('0');
    out.print(static_cast<unsigned int>(value), HEX);
}

void printMaskedReg(Stream& out, const char* name, uint8_t actual, uint8_t expected, uint8_t mask) {
    out.print(name); out.print("="); printHexByte(out, actual);
    out.print(" expected="); printHexByte(out, expected);
    out.print(" mask="); printHexByte(out, mask);
    out.print(" ok="); out.println(((actual & mask) == (expected & mask)) ? "yes" : "no");
}

void printTapHelp(Stream& out) {
    out.println("tap status");
    out.println("tap on | off");
#if TRACKER_ENABLE_TAP_DIAGNOSTICS
    out.println("tap log on | off | status - emit physical tap diagnostics to Serial/telnet");
#endif
    out.println("tap test [1..10]       - send one SlimeVR Tap packet directly");
    out.println("tap inject <1..10>     - emulate physical taps through accumulator");
    out.println("tap reset");
}

void printTapStatus(Stream& out, const TapRuntimeController& tap) {
    const TapRuntimeStatus s = tap.status();
    out.println("# TAP STATUS");
    out.print("enabled="); out.println(s.enabled ? "yes" : "no");
    out.print("hardware_configured="); out.println(s.hardwareConfigured ? "yes" : "no");
    out.print("last_read_ok="); out.println(s.lastReadOk ? "yes" : "no");
    out.print("last_raw_source="); printHexByte(out, s.lastRawSource); out.println();
    out.print("last_physical_count="); out.println(s.lastPhysicalCount);
    out.print("last_value="); out.println(s.lastValue);
    out.print("last_sent_ok="); out.println(s.lastSentOk ? "yes" : "no");
    out.print("physical_tap_user_action="); out.println(slimevrUserActionName(s.physicalTapUserAction));
    out.print("user_actions_sent="); out.println(s.userActionsSent);
    out.print("user_action_failures="); out.println(s.userActionFailures);
#if TRACKER_ENABLE_TAP_DIAGNOSTICS
    out.print("diagnostic_logging="); out.println(s.diagnosticLogging ? "yes" : "no");
    out.print("diagnostic_events="); out.println(s.diagnosticEvents);
#endif
    out.print("single_detected="); out.println(s.singleDetected);
    out.print("double_detected="); out.println(s.doubleDetected);
    out.print("tap_detected_no_type="); out.println(s.tapDetectedNoType);
    out.print("pending_count="); out.println(s.pendingCount);
    out.print("pending_first_ms="); out.println(s.pendingFirstMs);
    out.print("pending_last_ms="); out.println(s.pendingLastMs);
    out.print("flush_deadline_ms="); out.println(s.flushDeadlineMs);
    out.print("last_physical_tap_ms="); out.println(s.lastPhysicalTapMs);
    out.print("post_send_lockout_until_ms="); out.println(s.postSendLockoutUntilMs);
    out.print("physical_tap_events="); out.println(s.physicalTapEvents);
    out.print("physical_tap_count="); out.println(s.physicalTapCount);
    out.print("windows_started="); out.println(s.windowsStarted);
    out.print("windows_flushed="); out.println(s.windowsFlushed);
    out.print("suppressed_below_min="); out.println(s.suppressedBelowMin);
    out.print("suppressed_duplicate="); out.println(s.suppressedDuplicate);
    out.print("suppressed_lockout="); out.println(s.suppressedLockout);
    out.print("clamped_overflow="); out.println(s.clampedOverflow);
    out.print("sent="); out.println(s.sent);
    out.print("send_failures="); out.println(s.sendFailures);
    out.print("no_server="); out.println(s.noServer);
    out.print("read_failures="); out.println(s.readFailures);
    out.print("configure_failures="); out.println(s.configureFailures);
    out.print("last_event_ms="); out.println(s.lastEventMs);
    out.print("last_sent_ms="); out.println(s.lastSentMs);
    out.print("register_verify_ok="); out.println(s.registerVerifyOk ? "yes" : "no");
    out.print("last_register_read_ok="); out.println(s.lastRegisterReadOk ? "yes" : "no");
    out.print("register_verify_failures="); out.println(s.registerVerifyFailures);
    out.print("register_mismatch_count="); out.println(s.registerMismatchCount);
    out.print("last_register_verify_ms="); out.println(s.lastRegisterVerifyMs);

    const auto& r = s.lastRegisterVerification;
    printMaskedReg(out, "reg_functions_enable", r.actual.functionsEnable, r.expected.functionsEnable, r.mask.functionsEnable);
    printMaskedReg(out, "reg_tap_cfg0", r.actual.tapCfg0, r.expected.tapCfg0, r.mask.tapCfg0);
    printMaskedReg(out, "reg_tap_cfg1", r.actual.tapCfg1, r.expected.tapCfg1, r.mask.tapCfg1);
    printMaskedReg(out, "reg_tap_cfg2", r.actual.tapCfg2, r.expected.tapCfg2, r.mask.tapCfg2);
    printMaskedReg(out, "reg_tap_ths_6d", r.actual.tapThs6d, r.expected.tapThs6d, r.mask.tapThs6d);
    printMaskedReg(out, "reg_tap_dur", r.actual.tapDur, r.expected.tapDur, r.mask.tapDur);
    printMaskedReg(out, "reg_wake_up_ths", r.actual.wakeUpThs, r.expected.wakeUpThs, r.mask.wakeUpThs);
    printMaskedReg(out, "reg_md1_cfg", r.actual.md1Cfg, r.expected.md1Cfg, r.mask.md1Cfg);
}

} // namespace

void trackerSerialDispatchTapCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = ctx.io ? *ctx.io : Serial;
    if (!ctx.tapRuntime) {
        tracker_serial_detail::printErr(out, "tap runtime is not wired");
        return;
    }

    if (argc < 2 || tracker_serial_detail::eqIgnoreCase(argv[1], "status")) {
        printTapStatus(out, *ctx.tapRuntime);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "help")) {
        printTapHelp(out);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "on") ||
        tracker_serial_detail::eqIgnoreCase(argv[1], "enable")) {
        const bool ok = ctx.tapRuntime->setEnabled(true);
        out.print("# "); out.println(ok ? "OK tap runtime enabled" : "WARN tap runtime enable failed");
        return;
    }

#if TRACKER_ENABLE_TAP_DIAGNOSTICS
    if (tracker_serial_detail::eqIgnoreCase(argv[1], "log")) {
        if (argc < 3 || tracker_serial_detail::eqIgnoreCase(argv[2], "status")) {
            const TapRuntimeStatus s = ctx.tapRuntime->status();
            out.print("# tap_log="); out.println(s.diagnosticLogging ? "on" : "off");
            out.print("# tap_log_events="); out.println(s.diagnosticEvents);
            return;
        }
        if (tracker_serial_detail::eqIgnoreCase(argv[2], "on") ||
            tracker_serial_detail::eqIgnoreCase(argv[2], "enable")) {
            ctx.tapRuntime->setDiagnosticLogging(true);
            const TapRuntimeConfig cfg = ctx.tapRuntime->config();
            out.print("# OK tap log enabled poll_ms="); out.print(cfg.pollIntervalMs);
            out.print(" threshold="); out.print(cfg.threshold);
            out.print(" min_count="); out.print(cfg.minCount);
            out.print(" window_ms="); out.println(cfg.aggregationWindowMs);
            return;
        }
        if (tracker_serial_detail::eqIgnoreCase(argv[2], "off") ||
            tracker_serial_detail::eqIgnoreCase(argv[2], "disable")) {
            ctx.tapRuntime->setDiagnosticLogging(false);
            out.println("# OK tap log disabled");
            return;
        }
        tracker_serial_detail::printErr(out, "usage: tap log on|off|status");
        return;
    }
#endif

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "off") ||
        tracker_serial_detail::eqIgnoreCase(argv[1], "disable")) {
        const bool ok = ctx.tapRuntime->setEnabled(false);
        out.print("# "); out.println(ok ? "OK tap runtime disabled" : "WARN tap runtime disable failed");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "reset")) {
        ctx.tapRuntime->resetCounters();
        out.println("# OK tap counters reset");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "test")) {
        const uint8_t value = parseTapCount(argc >= 3 ? argv[2] : nullptr);
        if (value < TRACKER_TAP_VALUE_MIN || value > TRACKER_TAP_VALUE_MAX) {
            tracker_serial_detail::printErr(out, "usage: tap test [1..10]");
            return;
        }
        const bool ok = ctx.tapRuntime->sendManualTap(value, millis());
        out.print("# tap test value=");
        out.print(value);
        out.print(" sent=");
        out.println(ok ? "yes" : "no");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "inject")) {
        const uint8_t count = parseTapCount(argc >= 3 ? argv[2] : nullptr);
        if (count == 0 || count > TRACKER_TAP_VALUE_MAX) {
            tracker_serial_detail::printErr(out, "usage: tap inject <1..10>");
            return;
        }
        const bool ok = ctx.tapRuntime->injectPhysicalTaps(count, millis());
        out.print("# tap inject count=");
        out.print(count);
        out.print(" sent=");
        out.println(ok ? "yes" : "no");
        return;
    }

    tracker_serial_detail::printErr(out, "unknown tap command; use tap help");
}

} // namespace tracker
