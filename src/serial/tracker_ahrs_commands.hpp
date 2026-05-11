#pragma once

#include <Arduino.h>
#include <cmath>

#include "core/math.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "config/tracker_config.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

class TrackerAhrsCommandDispatcher {
public:
    static void dispatch(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.ahrs) {
            tracker_serial_detail::printErr(out, "ahrs not available");
            return;
        }

        if (argc < 2 || is(argv[1], "status")) {
            const Vec3 e = ctx.ahrs->eulerDeg();
            const Quat q = ctx.ahrs->quaternionPositiveW();
            tracker_serial_detail::printQuatLine(out, "quat", q, 7);
            tracker_serial_detail::printVec3Line(out, "euler_deg", e, 3);
            const auto& st = ctx.ahrs->stats();
            out.print("ahrs_updates="); out.println(st.updateCount);
            out.print("accel_updates="); out.println(st.accelUpdateCount);
            out.print("accel_rejects="); out.println(st.accelRejectedCount);
            out.print("last_seen_t_us="); tracker_serial_detail::printU64Dec(out, st.lastSeenTimestampUs); out.println();
            out.print("last_integrated_t_us="); tracker_serial_detail::printU64Dec(out, st.lastIntegratedTimestampUs); out.println();
            out.print("bad_dt_rejects="); out.println(st.skippedBadDt);
            out.print("startup_accel_rejects="); out.println(st.startupAccelRejectedCount);
            out.print("large_dt_clamps="); out.println(st.clampedLargeDt);
            out.print("last_accel_trust="); out.println(st.lastAccelGate.trust, 6);
            out.print("last_accel_norm_trust="); out.println(st.lastAccelGate.normTrust, 6);
            out.print("last_accel_innovation_trust="); out.println(st.lastAccelGate.innovationTrust, 6);
            out.print("last_accel_variance_trust="); out.println(st.lastAccelGate.varianceTrust, 6);
            out.print("last_gyro_motion_trust="); out.println(st.lastAccelGate.gyroMotionTrust, 6);
            out.print("accel_norm_variance_g2="); out.println(st.accelNormVarianceG2, 9);
            return;
        }

        if (is(argv[1], "config")) {
            printAhrsConfig(out, ctx.ahrs->config());
            return;
        }

        if (is(argv[1], "reset")) {
            ctx.ahrs->reset();
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
            tracker_serial_detail::printOk(out, "ahrs reset");
            return;
        }

        if (!ctx.config) {
            tracker_serial_detail::printErr(out, "config not available");
            return;
        }

        auto applyAndMaybeSave = [&](bool saveRequested, const char* okMsg, const char* saveErr) -> void {
            ctx.config->sanitize();
            ctx.ahrs->setConfig(ctx.config->makeAhrsConfig());
            ctx.config->updateCrc();
            if (saveRequested) {
                if (!ctx.configStore || !ctx.configStore->save(*ctx.config)) {
                    tracker_serial_detail::printErr(out, saveErr);
                    return;
                }
            }
            tracker_serial_detail::printOk(out, okMsg);
        };

        if (is(argv[1], "defaults")) {
            const bool saveRequested = argc >= 3 && is(argv[2], "save");
            ctx.config->data.ahrsRuntime = TrackerAhrsRuntimeConfigPersisted{};
            ctx.config->data.ahrs.accelCorrectionGain = 3.0f;
            ctx.config->data.ahrs.useAccelCorrection = true;
            applyAndMaybeSave(saveRequested,
                              saveRequested ? "ahrs defaults applied and saved" : "ahrs defaults applied",
                              "ahrs defaults save failed");
            return;
        }

        if (is(argv[1], "accel") || is(argv[1], "adaptive")) {
            if (argc < 3) {
                tracker_serial_detail::printErr(out, "usage: ahrs accel|adaptive on|off [save]");
                return;
            }
            bool enabled = false;
            if (!tracker_serial_detail::parseBool(argv[2], enabled)) {
                tracker_serial_detail::printErr(out, "expected on/off");
                return;
            }
            const bool saveRequested = argc >= 4 && is(argv[3], "save");
            if (is(argv[1], "accel")) {
                ctx.config->data.ahrsRuntime.accelCorrectionEnabled = enabled;
                ctx.config->data.ahrs.useAccelCorrection = enabled;
            } else {
                ctx.config->data.ahrsRuntime.adaptiveAccelCorrection = enabled;
            }
            applyAndMaybeSave(saveRequested,
                              saveRequested ? "ahrs gate saved" : "ahrs gate set",
                              "ahrs gate save failed");
            return;
        }

        if (is(argv[1], "accel_kp") || is(argv[1], "max_step")) {
            if (argc < 3) {
                tracker_serial_detail::printErr(out, "usage: ahrs accel_kp <gain> [save] | max_step <deg> [save]");
                return;
            }
            float v = 0.0f;
            if (!tracker_serial_detail::parseFloat(argv[2], v)) {
                tracker_serial_detail::printErr(out, "invalid value");
                return;
            }
            const bool saveRequested = argc >= 4 && is(argv[3], "save");
            if (is(argv[1], "accel_kp")) {
                ctx.config->data.ahrsRuntime.accelKp = v;
                ctx.config->data.ahrs.accelCorrectionGain = v;
            } else {
                ctx.config->data.ahrsRuntime.maxAccelCorrectionDegPerUpdate = v;
            }
            applyAndMaybeSave(saveRequested,
                              saveRequested ? "ahrs parameter saved" : "ahrs parameter set",
                              "ahrs parameter save failed");
            return;
        }

        if (is(argv[1], "accel_norm") || is(argv[1], "accel_innovation") ||
            is(argv[1], "accel_var") || is(argv[1], "gyro_gate") || is(argv[1], "dt")) {
            if (argc < 4) {
                tracker_serial_detail::printErr(out, "usage: ahrs accel_norm|accel_innovation|accel_var|gyro_gate|dt <good/min> <bad/max> [save]");
                return;
            }
            float a = 0.0f;
            float b = 0.0f;
            if (!tracker_serial_detail::parseFloat(argv[2], a) ||
                !tracker_serial_detail::parseFloat(argv[3], b)) {
                tracker_serial_detail::printErr(out, "invalid values");
                return;
            }
            const bool saveRequested = argc >= 5 && is(argv[4], "save");
            if (is(argv[1], "accel_norm")) {
                ctx.config->data.ahrsRuntime.accelNormGoodErrorG = a;
                ctx.config->data.ahrsRuntime.accelNormBadErrorG = b;
            } else if (is(argv[1], "accel_innovation")) {
                ctx.config->data.ahrsRuntime.accelInnovationGoodDeg = a;
                ctx.config->data.ahrsRuntime.accelInnovationBadDeg = b;
            } else if (is(argv[1], "accel_var")) {
                ctx.config->data.ahrsRuntime.accelNormStdGoodG = a;
                ctx.config->data.ahrsRuntime.accelNormStdBadG = b;
            } else if (is(argv[1], "gyro_gate")) {
                ctx.config->data.ahrsRuntime.gyroMotionGoodDps = a;
                ctx.config->data.ahrsRuntime.gyroMotionBadDps = b;
            } else {
                ctx.config->data.ahrsRuntime.minDtS = a * 0.001f;
                ctx.config->data.ahrsRuntime.maxDtS = b * 0.001f;
            }
            applyAndMaybeSave(saveRequested,
                              saveRequested ? "ahrs gate saved" : "ahrs gate set",
                              "ahrs gate save failed");
            return;
        }

        tracker_serial_detail::printErr(out, "unknown ahrs command");
    }

private:
    static void printAhrsConfig(Stream& out, const Ahrs6DofConfig& cfg) {
        out.println("# AHRS CONFIG");
        out.print("accelCorrectionEnabled="); out.println(cfg.accelCorrectionEnabled ? "yes" : "no");
        out.print("adaptiveAccelCorrection="); out.println(cfg.adaptiveAccelCorrection ? "yes" : "no");
        out.print("accelKp="); out.println(cfg.accelKp, 6);
        out.print("minDtS="); out.println(cfg.minDtS, 7);
        out.print("maxDtS="); out.println(cfg.maxDtS, 7);
        out.print("clampLargeDt="); out.println(cfg.clampLargeDt ? "yes" : "no");
        out.print("maxAccelCorrectionDegPerUpdate="); out.println(cfg.maxAccelCorrectionRadPerUpdate * MATH_RAD_TO_DEG, 6);
        out.print("accelNormGoodBadErrorG="); out.print(cfg.accelNormGoodErrorG, 6); out.print(','); out.println(cfg.accelNormBadErrorG, 6);
        out.print("accelInnovationGoodBadDeg="); out.print(cfg.accelInnovationGoodRad * MATH_RAD_TO_DEG, 3); out.print(','); out.println(cfg.accelInnovationBadRad * MATH_RAD_TO_DEG, 3);
        out.print("accelNormStdGoodBadG="); out.print(std::sqrt(cfg.accelNormVarianceGoodG2), 6); out.print(','); out.println(std::sqrt(cfg.accelNormVarianceBadG2), 6);
        out.print("accelNormVarianceAlpha="); out.println(cfg.accelNormVarianceAlpha, 6);
        out.print("gyroMotionGoodBadDps="); out.print(cfg.gyroNormAccelTrustGoodRadS * MATH_RAD_TO_DEG, 3); out.print(','); out.println(cfg.gyroNormAccelTrustBadRadS * MATH_RAD_TO_DEG, 3);
        out.print("normalizeEvery="); out.println(cfg.normalizeEvery);
    }


private:
    static Stream& stream(TrackerSerialCommandContext& ctx) {
        return ctx.io ? *ctx.io : Serial;
    }

    static bool is(const char* a, const char* b) {
        return tracker_serial_detail::eqIgnoreCase(a, b);
    }
};

inline void trackerSerialDispatchAhrsCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    TrackerAhrsCommandDispatcher::dispatch(ctx, argc, argv);
}

} // namespace tracker
