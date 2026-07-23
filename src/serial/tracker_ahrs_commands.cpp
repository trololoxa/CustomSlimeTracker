#include "serial/tracker_ahrs_commands.hpp"

#include <Arduino.h>
#include <cmath>

#include "core/math.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "serial/tracker_serial_context.hpp"
#include "serial/tracker_config_transaction.hpp"

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
            out.print("large_dt_rebase_count="); out.println(st.largeDtRebaseCount);
            out.print("fifo_rebase_count="); out.println(st.fifoRecoveryRebaseCount);
            out.print("last_rebase_t_us="); tracker_serial_detail::printU64Dec(out, st.lastRebaseTimestampUs); out.println();
            out.print("post_fifo_recovery_samples="); out.println(st.postFifoRecoverySamples);
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

        auto applyAndMaybeSave = [&](TrackerConfig candidate,
                                     bool saveRequested,
                                     const char* okMsg,
                                     const char* saveErr) -> void {
            const bool ok = trackerCommitConfigCandidate(
                ctx,
                candidate,
                saveRequested,
                [&]() { ctx.ahrs->setConfig(ctx.config->makeAhrsConfig()); }
            );
            if (!ok) {
                tracker_serial_detail::printErr(out, saveErr);
                return;
            }
            tracker_serial_detail::printOk(out, okMsg);
        };

        if (is(argv[1], "defaults")) {
            const bool saveRequested = argc >= 3 && is(argv[2], "save");
            TrackerConfig candidate = *ctx.config;
            candidate.data.ahrsRuntime = TrackerAhrsRuntimeConfigPersisted{};
            candidate.data.ahrs.accelCorrectionGain = 3.0f;
            candidate.data.ahrs.useAccelCorrection = true;
            applyAndMaybeSave(candidate, saveRequested,
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
            TrackerConfig candidate = *ctx.config;
            if (is(argv[1], "accel")) {
                candidate.data.ahrsRuntime.accelCorrectionEnabled = enabled;
                candidate.data.ahrs.useAccelCorrection = enabled;
            } else {
                candidate.data.ahrsRuntime.adaptiveAccelCorrection = enabled;
            }
            applyAndMaybeSave(candidate, saveRequested,
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
            TrackerConfig candidate = *ctx.config;
            if (is(argv[1], "accel_kp")) {
                candidate.data.ahrsRuntime.accelKp = v;
                candidate.data.ahrs.accelCorrectionGain = v;
            } else {
                candidate.data.ahrsRuntime.maxAccelCorrectionDegPerUpdate = v;
            }
            applyAndMaybeSave(candidate, saveRequested,
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
            TrackerConfig candidate = *ctx.config;
            if (is(argv[1], "accel_norm")) {
                candidate.data.ahrsRuntime.accelNormGoodErrorG = a;
                candidate.data.ahrsRuntime.accelNormBadErrorG = b;
            } else if (is(argv[1], "accel_innovation")) {
                candidate.data.ahrsRuntime.accelInnovationGoodDeg = a;
                candidate.data.ahrsRuntime.accelInnovationBadDeg = b;
            } else if (is(argv[1], "accel_var")) {
                candidate.data.ahrsRuntime.accelNormStdGoodG = a;
                candidate.data.ahrsRuntime.accelNormStdBadG = b;
            } else if (is(argv[1], "gyro_gate")) {
                candidate.data.ahrsRuntime.gyroMotionGoodDps = a;
                candidate.data.ahrsRuntime.gyroMotionBadDps = b;
            } else {
                candidate.data.ahrsRuntime.minDtS = a * 0.001f;
                candidate.data.ahrsRuntime.maxDtS = b * 0.001f;
            }
            applyAndMaybeSave(candidate, saveRequested,
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

void trackerSerialDispatchAhrsCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    TrackerAhrsCommandDispatcher::dispatch(ctx, argc, argv);
}

} // namespace tracker
