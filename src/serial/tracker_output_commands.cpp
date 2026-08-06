#include "serial/tracker_output_commands.hpp"

#include <Arduino.h>

#include "config/tracker_config_runtime.hpp"
#include "config/tracker_network_config.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

Stream& trackerSerialOutputStream(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

bool trackerSerialOutputIs(const char* a, const char* b) {
    return tracker_serial_detail::eqIgnoreCase(a, b);
}

const char* trackerSerialStreamModeName(TrackerStreamMode mode) {
    switch (mode) {
        case TrackerStreamMode::Off:        return "off";
        case TrackerStreamMode::Heartbeat:  return "heartbeat";
        case TrackerStreamMode::Raw:        return "raw";
        case TrackerStreamMode::Scaled:     return "scaled";
        case TrackerStreamMode::Quat:       return "quat";
        case TrackerStreamMode::Debug:      return "debug";
    }
    return "unknown";
}

const char* trackerSerialLogModeName(TrackerLogMode mode) {
    switch (mode) {
        case TrackerLogMode::Off:   return "off";
        case TrackerLogMode::Basic: return "basic";
        case TrackerLogMode::Full:  return "full";
    }
    return "unknown";
}

bool trackerSerialParseStreamMode(const char* s, TrackerStreamMode& mode) {
    if (trackerSerialOutputIs(s, "off"))       { mode = TrackerStreamMode::Off; return true; }
    if (trackerSerialOutputIs(s, "heartbeat")) { mode = TrackerStreamMode::Heartbeat; return true; }
    if (trackerSerialOutputIs(s, "raw"))       { mode = TrackerStreamMode::Raw; return true; }
    if (trackerSerialOutputIs(s, "scaled"))    { mode = TrackerStreamMode::Scaled; return true; }
    if (trackerSerialOutputIs(s, "quat"))      { mode = TrackerStreamMode::Quat; return true; }
    if (trackerSerialOutputIs(s, "debug"))     { mode = TrackerStreamMode::Debug; return true; }
    return false;
}

bool trackerSerialParseLogMode(const char* s, TrackerLogMode& mode) {
    if (trackerSerialOutputIs(s, "off"))   { mode = TrackerLogMode::Off; return true; }
    if (trackerSerialOutputIs(s, "basic")) { mode = TrackerLogMode::Basic; return true; }
    if (trackerSerialOutputIs(s, "full"))  { mode = TrackerLogMode::Full; return true; }
    return false;
}

void trackerSerialDispatchStreamCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = trackerSerialOutputStream(ctx);
    if (!ctx.streamState) {
        tracker_serial_detail::printErr(out, "stream state not available");
        return;
    }

    if (argc < 2) {
        out.print("stream_mode="); out.println(trackerSerialStreamModeName(ctx.streamState->mode));
        out.print("stream_rate_hz="); out.println(ctx.streamState->rateHz);
        return;
    }

    if (trackerSerialOutputIs(argv[1], "rate")) {
        if (argc < 3) {
            tracker_serial_detail::printErr(out, "usage: stream rate <hz>");
            return;
        }
        uint32_t hz = 0;
        if (!tracker_serial_detail::parseU32(argv[2], hz) || hz == 0 || hz > 1000) {
            tracker_serial_detail::printErr(out, "invalid stream rate; expected 1..1000");
            return;
        }
        ctx.streamState->rateHz = static_cast<uint16_t>(hz);
        if (ctx.config) {
            ctx.config->data.output.outputRateHz = static_cast<uint16_t>(hz);
            ctx.config->updateCrc();
        }
        tracker_serial_detail::printOk(out, "stream rate set");
        return;
    }

    TrackerStreamMode mode;
    if (!trackerSerialParseStreamMode(argv[1], mode)) {
        tracker_serial_detail::printErr(out, "unknown stream mode; use off|heartbeat|raw|scaled|quat|debug");
        return;
    }
    ctx.streamState->mode = mode;
    ctx.streamState->lastEmitUs = 0;

    if (ctx.config) {
        ctx.config->data.output.quaternionOutputEnabled = (mode == TrackerStreamMode::Quat);
        ctx.config->data.output.serialDebugEnabled = (mode == TrackerStreamMode::Debug);
        ctx.config->updateCrc();
    }

    out.print("# OK stream mode ");
    out.println(trackerSerialStreamModeName(mode));
}

void trackerSerialDispatchLogCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = trackerSerialOutputStream(ctx);
    if (!ctx.logState) {
        tracker_serial_detail::printErr(out, "log state not available");
        return;
    }
    const bool wasEnabled = ctx.logState->enabled();

    if (argc < 2) {
        out.print("log_mode="); out.println(trackerSerialLogModeName(ctx.logState->mode));
        out.print("log_rate_hz="); out.println(ctx.logState->rateHz);
        out.print("log_sequence="); out.println(ctx.logState->sequence);
        out.print("log_finishing="); out.println(ctx.logState->finishing ? "yes" : "no");
        out.print("log_owner_origin="); out.println(trackerCommandOriginName(ctx.logState->ownerOrigin));
        out.print("log_owner_session="); out.println(ctx.logState->ownerSessionId);
        return;
    }

    const bool canControl = !ctx.logState->enabled() || ctx.logState->ownedBy(ctx) ||
                            ctx.origin == TrackerCommandOrigin::UsbSerial;

    if (trackerSerialOutputIs(argv[1], "rate")) {
        if (!canControl) {
            tracker_serial_detail::printErr(out, "log is owned by another session");
            return;
        }
        if (argc < 3) {
            tracker_serial_detail::printErr(out, "usage: log rate <hz>");
            return;
        }
        uint32_t hz = 0;
        const uint32_t maxHz = ctx.origin == TrackerCommandOrigin::RemoteTcp ? 20u : 200u;
        if (!tracker_serial_detail::parseU32(argv[2], hz) || hz == 0 || hz > maxHz) {
            tracker_serial_detail::printErr(
                out,
                ctx.origin == TrackerCommandOrigin::RemoteTcp
                    ? "invalid remote log rate; expected 1..20"
                    : "invalid log rate; expected 1..200");
            return;
        }
        ctx.logState->rateHz = static_cast<uint16_t>(hz);
        ctx.logState->lastEmitUs = 0;
        ctx.logState->lastMagEmitUs = 0;
        ctx.logState->lastNetworkEmitUs = 0;
        tracker_serial_detail::printOk(out, "log rate set");
        return;
    }

    if (trackerSerialOutputIs(argv[1], "header")) {
        if (ctx.emitLogHeader) ctx.emitLogHeader(out, ctx.emitLogHeaderUser);
        else tracker_serial_detail::printErr(out, "log header hook not available");
        return;
    }

    if (trackerSerialOutputIs(argv[1], "summary")) {
        if (ctx.printLogSummary) ctx.printLogSummary(out, ctx.printLogSummaryUser);
        else tracker_serial_detail::printErr(out, "log summary hook not available");
        return;
    }

    if (trackerSerialOutputIs(argv[1], "finish")) {
        if (!canControl || !ctx.logState->enabled()) {
            tracker_serial_detail::printErr(
                out,
                ctx.logState->enabled()
                    ? "log is owned by another session"
                    : "log is not running");
            return;
        }
        ctx.logState->finishing = true;
        tracker_serial_detail::printOk(out, "log producer stopped; wait for queued=0, then summary/off");
        return;
    }

    if (trackerSerialOutputIs(argv[1], "reset")) {
        if (!canControl) {
            tracker_serial_detail::printErr(out, "log is owned by another session");
            return;
        }
        ctx.logState->sequence = 0;
        ctx.logState->lastEmitUs = 0;
        ctx.logState->lastMagEmitUs = 0;
        ctx.logState->lastNetworkEmitUs = 0;
        ctx.logState->finishing = false;
        if (ctx.resetLogPipeline) {
            ctx.resetLogPipeline(false, ctx.resetLogPipelineUser);
        }
        if (ctx.resetLogCounters) ctx.resetLogCounters(ctx.resetLogCountersUser);
        tracker_serial_detail::printOk(out, "log counters reset");
        return;
    }

    if (trackerSerialOutputIs(argv[1], "off") || trackerSerialOutputIs(argv[1], "stop")) {
        if (!canControl) {
            tracker_serial_detail::printErr(out, "log is owned by another session");
            return;
        }
        if (ctx.resetLogPipeline) {
            ctx.resetLogPipeline(true, ctx.resetLogPipelineUser);
        }
        ctx.logState->release();
        tracker_serial_detail::printOk(out, "log stopped");
        return;
    }

    TrackerLogMode mode = TrackerLogMode::Off;
    bool start = false;
    if (trackerSerialOutputIs(argv[1], "start")) {
        start = true;
        if (argc >= 3) {
            if (!trackerSerialParseLogMode(argv[2], mode) || mode == TrackerLogMode::Off) {
                tracker_serial_detail::printErr(out, "usage: log start [basic|full]");
                return;
            }
        } else {
            mode = TrackerLogMode::Basic;
        }
    } else if (trackerSerialParseLogMode(argv[1], mode)) {
        start = (mode != TrackerLogMode::Off);
    } else {
        tracker_serial_detail::printErr(out, "unknown log command; use off|basic|full|start|stop|finish|rate|header|summary|reset");
        return;
    }

    if (!canControl) {
        tracker_serial_detail::printErr(out, "log is owned by another session");
        return;
    }

    ctx.logState->mode = mode;
    ctx.logState->lastEmitUs = 0;
    ctx.logState->lastMagEmitUs = 0;
    ctx.logState->lastNetworkEmitUs = 0;
    if (mode == TrackerLogMode::Off) {
        if (ctx.resetLogPipeline) {
            ctx.resetLogPipeline(true, ctx.resetLogPipelineUser);
        }
        ctx.logState->release();
        tracker_serial_detail::printOk(out, "log stopped");
        return;
    }
    if (!wasEnabled && ctx.resetLogPipeline) {
        ctx.resetLogPipeline(false, ctx.resetLogPipelineUser);
    }
    ctx.logState->bind(out, ctx.origin, ctx.sessionId);

    out.print("# OK log mode ");
    out.println(trackerSerialLogModeName(mode));
    if (start && ctx.emitLogHeader) {
        ctx.emitLogHeader(out, ctx.emitLogHeaderUser);
    }
}

void trackerSerialDispatchOutputCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = trackerSerialOutputStream(ctx);
    if (!ctx.config) {
        tracker_serial_detail::printErr(out, "config not available");
        return;
    }
    if (argc < 2) {
        tracker_serial_detail::printErr(out, "usage: output mode|rate|start|stop");
        return;
    }

    if (trackerSerialOutputIs(argv[1], "rate")) {
        if (argc < 3) {
            tracker_serial_detail::printErr(out, "usage: output rate <hz>");
            return;
        }
        uint32_t hz = 0;
        if (!tracker_serial_detail::parseU32(argv[2], hz) || hz == 0 || hz > 1000) {
            tracker_serial_detail::printErr(out, "invalid output rate; expected 1..1000");
            return;
        }
        ctx.config->data.output.outputRateHz = static_cast<uint16_t>(hz);
        if (ctx.streamState) ctx.streamState->rateHz = static_cast<uint16_t>(hz);
        ctx.config->updateCrc();
        tracker_serial_detail::printOk(out, "output rate set");
        return;
    }

    if (trackerSerialOutputIs(argv[1], "mode")) {
        if (argc < 3) {
            tracker_serial_detail::printErr(out, "usage: output mode debug");
            return;
        }
        if (trackerSerialOutputIs(argv[2], "debug")) {
            ctx.config->data.output.packetFormat = 0;
            ctx.config->data.output.serialDebugEnabled = true;
            ctx.config->data.output.quaternionOutputEnabled = false;
            if (ctx.streamState) {
                ctx.streamState->mode = TrackerStreamMode::Debug;
                ctx.streamState->lastEmitUs = 0;
            }
            ctx.config->updateCrc();
            tracker_serial_detail::printOk(out, "output mode debug");
            return;
        }
        if (trackerSerialOutputIs(argv[2], "slimevr")) {
            tracker_serial_detail::printErr(out, "output mode slimevr was removed; use: slime start");
            return;
        }
        if (trackerSerialOutputIs(argv[2], "binary")) {
            tracker_serial_detail::printErr(out, "NOT_IMPLEMENTED: binary output backend is not available in this build");
            return;
        }
        tracker_serial_detail::printErr(out, "unknown output mode; SlimeVR is controlled with: slime start|stop|status");
        return;
    }

    if (trackerSerialOutputIs(argv[1], "start")) {
        ctx.config->data.output.quaternionOutputEnabled = true;
        ctx.config->data.output.serialDebugEnabled = false;
        ctx.config->data.output.packetFormat = 0;
        if (ctx.streamState) {
            ctx.streamState->mode = TrackerStreamMode::Quat;
            ctx.streamState->lastEmitUs = 0;
        }
        ctx.config->updateCrc();
        tracker_serial_detail::printOk(out, "local quaternion output started");
        out.println("# SlimeVR UDP is independent; use: slime start");
        return;
    }

    if (trackerSerialOutputIs(argv[1], "stop")) {
        ctx.config->data.output.quaternionOutputEnabled = false;
        ctx.config->data.output.serialDebugEnabled = false;
        ctx.config->data.output.packetFormat = 0;
        if (ctx.streamState) ctx.streamState->mode = TrackerStreamMode::Off;
        ctx.config->updateCrc();
        tracker_serial_detail::printOk(out, "local output stopped");
        return;
    }

    tracker_serial_detail::printErr(out, "unknown output command");
}

} // namespace tracker
