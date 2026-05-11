#pragma once

#include <Arduino.h>

#include "serial/tracker_serial_context.hpp"

namespace tracker {

inline Stream& trackerSerialBiasStream(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

inline bool trackerSerialBiasIs(const char* a, const char* b) {
    return tracker_serial_detail::eqIgnoreCase(a, b);
}

inline void trackerSerialDispatchBiasCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = trackerSerialBiasStream(ctx);

    if (argc < 2 || trackerSerialBiasIs(argv[1], "status")) {
        if (ctx.printRuntimeGyroBiasStatus) ctx.printRuntimeGyroBiasStatus(out, ctx.printRuntimeGyroBiasStatusUser);
        else tracker_serial_detail::printErr(out, "runtime gyro bias status hook not available");
        return;
    }

    if (trackerSerialBiasIs(argv[1], "on") || trackerSerialBiasIs(argv[1], "enable")) {
        if (!ctx.setRuntimeGyroBiasEnabled || !ctx.setRuntimeGyroBiasEnabled(true, ctx.setRuntimeGyroBiasEnabledUser)) {
            tracker_serial_detail::printErr(out, "runtime gyro bias enable failed");
            return;
        }
        tracker_serial_detail::printOk(out, "runtime gyro bias estimator enabled in RAM");
        return;
    }

    if (trackerSerialBiasIs(argv[1], "off") || trackerSerialBiasIs(argv[1], "disable")) {
        if (!ctx.setRuntimeGyroBiasEnabled || !ctx.setRuntimeGyroBiasEnabled(false, ctx.setRuntimeGyroBiasEnabledUser)) {
            tracker_serial_detail::printErr(out, "runtime gyro bias disable failed");
            return;
        }
        tracker_serial_detail::printOk(out, "runtime gyro bias estimator disabled");
        return;
    }

    if (trackerSerialBiasIs(argv[1], "reset")) {
        if (ctx.resetRuntimeGyroBiasEstimator) {
            ctx.resetRuntimeGyroBiasEstimator(ctx.resetRuntimeGyroBiasEstimatorUser);
            tracker_serial_detail::printOk(out, "runtime gyro bias estimator counters and runtime trim reset");
        } else {
            tracker_serial_detail::printErr(out, "runtime gyro bias reset hook not available");
        }
        return;
    }

    tracker_serial_detail::printErr(out, "unknown bias command; use status|on|off|reset");
}

} // namespace tracker
