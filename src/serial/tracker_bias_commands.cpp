#include "serial/tracker_bias_commands.hpp"

#include <Arduino.h>

#include "serial/tracker_serial_context.hpp"
#include "serial/tracker_config_commands.hpp"
#include "config/tracker_config.hpp"
#include "runtime/runtime_bias_types.hpp"

namespace tracker {

Stream& trackerSerialBiasStream(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

bool trackerSerialBiasIs(const char* a, const char* b) {
    return tracker_serial_detail::eqIgnoreCase(a, b);
}

bool trackerSerialBiasSaveAutostart(TrackerSerialCommandContext& ctx, bool enabled) {
    Stream& out = trackerSerialBiasStream(ctx);
    if (!ctx.config || !ctx.configStore) {
        tracker_serial_detail::printErr(out, "runtime gyro bias save failed: config store not available");
        return false;
    }

    TrackerConfig candidate = *ctx.config;
    if (enabled) {
        candidate.data.ahrsRuntime.reserved |=
            tracker_config_detail::AHRS_RUNTIME_FLAG_RUNTIME_BIAS_ENABLED;
    } else {
        candidate.data.ahrsRuntime.reserved &=
            static_cast<uint8_t>(~tracker_config_detail::AHRS_RUNTIME_FLAG_RUNTIME_BIAS_ENABLED);
    }
    candidate.sanitize();
    candidate.updateCrc();
    if (!ctx.configStore->save(candidate, TrackerCalibrationProvenance::Manual)) {
        out.print("# ERR runtime gyro bias save failed: ");
        out.println(ctx.configStore->lastErrorName());
        return false;
    }
    *ctx.config = candidate;
    tracker_serial_detail::printOk(out, enabled
        ? "runtime gyro bias autostart enabled and saved to NVS"
        : "runtime gyro bias autostart disabled and saved to NVS");
    return true;
}

void trackerSerialDispatchBiasCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = trackerSerialBiasStream(ctx);

    if (argc < 2 || trackerSerialBiasIs(argv[1], "status")) {
        if (ctx.printRuntimeGyroBiasStatus) ctx.printRuntimeGyroBiasStatus(out, ctx.printRuntimeGyroBiasStatusUser);
        else tracker_serial_detail::printErr(out, "runtime gyro bias status hook not available");
        return;
    }

    if (trackerSerialBiasIs(argv[1], "on") || trackerSerialBiasIs(argv[1], "enable")) {
        const bool saveRequested = argc >= 3 && trackerSerialBiasIs(argv[2], "save");
        const bool oldEnabled = ctx.runtimeBias ? ctx.runtimeBias->enabled : false;
        if (!ctx.setRuntimeGyroBiasEnabled || !ctx.setRuntimeGyroBiasEnabled(true, ctx.setRuntimeGyroBiasEnabledUser)) {
            tracker_serial_detail::printErr(out, "runtime gyro bias enable failed");
            return;
        }
        if (saveRequested && !trackerSerialBiasSaveAutostart(ctx, true)) {
            (void)ctx.setRuntimeGyroBiasEnabled(oldEnabled, ctx.setRuntimeGyroBiasEnabledUser);
            return;
        }
        tracker_serial_detail::printOk(out, saveRequested
            ? "runtime gyro bias estimator enabled and autostart saved"
            : "runtime gyro bias estimator enabled in RAM; use 'bias on save' to persist autostart");
        return;
    }

    if (trackerSerialBiasIs(argv[1], "off") || trackerSerialBiasIs(argv[1], "disable")) {
        const bool saveRequested = argc >= 3 && trackerSerialBiasIs(argv[2], "save");
        const bool oldEnabled = ctx.runtimeBias ? ctx.runtimeBias->enabled : true;
        if (!ctx.setRuntimeGyroBiasEnabled || !ctx.setRuntimeGyroBiasEnabled(false, ctx.setRuntimeGyroBiasEnabledUser)) {
            tracker_serial_detail::printErr(out, "runtime gyro bias disable failed");
            return;
        }
        if (saveRequested && !trackerSerialBiasSaveAutostart(ctx, false)) {
            (void)ctx.setRuntimeGyroBiasEnabled(oldEnabled, ctx.setRuntimeGyroBiasEnabledUser);
            return;
        }
        tracker_serial_detail::printOk(out, saveRequested
            ? "runtime gyro bias estimator disabled and autostart saved"
            : "runtime gyro bias estimator disabled in RAM; use 'bias off save' to persist autostart");
        return;
    }

    if (trackerSerialBiasIs(argv[1], "save")) {
        if (!ctx.runtimeBias) {
            tracker_serial_detail::printErr(out, "runtime gyro bias save failed: estimator not available");
            return;
        }
        (void)trackerSerialBiasSaveAutostart(ctx, ctx.runtimeBias->enabled);
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

    tracker_serial_detail::printErr(out, "unknown bias command; use status|on [save]|off [save]|save|reset");
}

} // namespace tracker
