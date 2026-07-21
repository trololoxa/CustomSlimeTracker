#include "serial/tracker_mag_commands.hpp"

#include <Arduino.h>
#include <cstdint>
#include <cmath>

#include "core/math.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "connection/lsm6dsv_sensorhub.hpp"
#include "sensor/qmc6309.hpp"
#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

namespace tracker_serial_mag_detail {

Stream& stream(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

bool is(const char* a, const char* b) {
    return tracker_serial_detail::eqIgnoreCase(a, b);
}

} // namespace tracker_serial_mag_detail

bool magSaveConfigIfRequested(TrackerSerialCommandContext& ctx, bool saveRequested) {
        if (!saveRequested) return true;
        if (!ctx.config || !ctx.configStore) return false;
        ctx.config->updateCrc();
        return ctx.configStore->save(*ctx.config);
    }

bool rearmMagIfNeeded(TrackerSerialCommandContext& ctx) {
        if (!ctx.config || !ctx.config->data.magCal.driverEnabled) return true;
        if (!ctx.setMagRuntimeEnabled) return false;
        return ctx.setMagRuntimeEnabled(true, false, ctx.setMagRuntimeEnabledUser);
    }

char magAxisLower(char c) {
        if (c >= 'A' && c <= 'Z') return static_cast<char>(c - 'A' + 'a');
        return c;
    }

bool parseMagAxisToken(const char* token, uint8_t& axis, float& sign) {
        if (token == nullptr || token[0] == '\0') return false;

        sign = 1.0f;
        uint8_t pos = 0;

        if (token[pos] == '+') {
            sign = 1.0f;
            pos++;
        } else if (token[pos] == '-') {
            sign = -1.0f;
            pos++;
        }

        const char a = magAxisLower(token[pos]);
        if (a == '\0' || token[pos + 1] != '\0') return false;

        if (a == 'x') {
            axis = 0;
            return true;
        }
        if (a == 'y') {
            axis = 1;
            return true;
        }
        if (a == 'z') {
            axis = 2;
            return true;
        }

        return false;
    }

const char* magAxisNameFromIndex(uint8_t axis) {
        switch (axis) {
            case 0: return "x";
            case 1: return "y";
            case 2: return "z";
        }
        return "?";
    }

void printMagAxisToken(Stream& out, const Mat3& m, uint8_t row) {
        uint8_t nonZeroCount = 0;
        uint8_t axis = 0;
        float sign = 1.0f;

        for (uint8_t col = 0; col < 3; ++col) {
            const float v = m.m[row][col];
            if (std::fabs(v) > 0.5f) {
                nonZeroCount++;
                axis = col;
                sign = v >= 0.0f ? 1.0f : -1.0f;
            }
        }

        if (nonZeroCount != 1) {
            out.print("?");
            return;
        }

        out.print(sign >= 0.0f ? "+" : "-");
        out.print(magAxisNameFromIndex(axis));
    }

bool makeMagAxisMatrixFromTokens(const char* bodyXToken,
                                            const char* bodyYToken,
                                            const char* bodyZToken,
                                            Mat3& out) {
        uint8_t axis[3] = {};
        float sign[3] = {};

        if (!parseMagAxisToken(bodyXToken, axis[0], sign[0])) return false;
        if (!parseMagAxisToken(bodyYToken, axis[1], sign[1])) return false;
        if (!parseMagAxisToken(bodyZToken, axis[2], sign[2])) return false;

        // Must be a pure permutation/sign matrix:
        // body.x, body.y, body.z must use each mag axis exactly once.
        bool used[3] = {false, false, false};
        for (uint8_t i = 0; i < 3; ++i) {
            if (axis[i] > 2) return false;
            if (used[axis[i]]) return false;
            used[axis[i]] = true;
        }

        out = Mat3::zero();
        for (uint8_t row = 0; row < 3; ++row) {
            out.m[row][axis[row]] = sign[row];
        }

        return true;
    }

bool saveConfigIfRequested(TrackerSerialCommandContext& ctx, bool saveRequested) {
        if (!saveRequested) return true;
        if (!ctx.config || !ctx.configStore) return false;
        ctx.config->updateCrc();
        return ctx.configStore->save(*ctx.config);
    }

void printMagAxisMatrix(Stream& out, const TrackerConfig& config) {
        const Mat3& m = config.data.magCal.magToImu;

        out.println("# MAG AXIS");
        out.print("axisAlignmentValid=");
        out.println(config.data.magCal.axisAlignmentValid ? "yes" : "no");

        out.print("mapping=");
        printMagAxisToken(out, m, 0);
        out.print(' ');
        printMagAxisToken(out, m, 1);
        out.print(' ');
        printMagAxisToken(out, m, 2);
        out.println();

        out.print("meaning=");
        out.print("body.x=");
        printMagAxisToken(out, m, 0);
        out.print(" body.y=");
        printMagAxisToken(out, m, 1);
        out.print(" body.z=");
        printMagAxisToken(out, m, 2);
        out.println();

        out.print("magToImu_row0=");
        out.print(m.m[0][0], 6); out.print(',');
        out.print(m.m[0][1], 6); out.print(',');
        out.println(m.m[0][2], 6);

        out.print("magToImu_row1=");
        out.print(m.m[1][0], 6); out.print(',');
        out.print(m.m[1][1], 6); out.print(',');
        out.println(m.m[1][2], 6);

        out.print("magToImu_row2=");
        out.print(m.m[2][0], 6); out.print(',');
        out.print(m.m[2][1], 6); out.print(',');
        out.println(m.m[2][2], 6);
    }

    void trackerSerialDispatchMagCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = tracker_serial_mag_detail::stream(ctx);

        if (argc < 2 || tracker_serial_mag_detail::is(argv[1], "status")) {
            out.println("# MAG STATUS");
            if (ctx.config) {
                out.print("mag_config_enabled="); out.println(ctx.config->data.magCal.driverEnabled ? "yes" : "no");
                out.print("mag_cal_valid="); out.println(ctx.config->data.magCal.calibrationValid ? "yes" : "no");
                out.print("mag_axis_valid="); out.println(ctx.config->data.magCal.axisAlignmentValid ? "yes" : "no");
            }
            if (ctx.printMagRuntimeStatus) {
                ctx.printMagRuntimeStatus(out, ctx.printMagRuntimeStatusUser);
            }
            if (ctx.fifo) {
                const auto& fs = ctx.fifo->stats();
                out.print("sensorhub0_words="); out.println(fs.sensorHubSlave0Words);
                out.print("sensorhub_nack_words="); out.println(fs.sensorHubNackWords);
                out.print("mag_samples_produced="); out.println(fs.magSamplesProduced);
                out.print("mag_last_xyz="); out.print(fs.lastMagX); out.print(','); out.print(fs.lastMagY); out.print(','); out.println(fs.lastMagZ);
                out.print("mag_last_norm_raw="); out.println(fs.lastMagRawNorm, 3);
            }
            return;
        }

        if (tracker_serial_mag_detail::is(argv[1], "enable") || tracker_serial_mag_detail::is(argv[1], "on")) {
            const bool saveRequested = argc >= 3 && tracker_serial_mag_detail::is(argv[2], "save");
            if (ctx.config) {
                ctx.config->data.magCal.driverEnabled = true;
                ctx.config->updateCrc();
            }
            bool ok = true;
            if (ctx.setMagRuntimeEnabled) {
                ok = ctx.setMagRuntimeEnabled(true, saveRequested, ctx.setMagRuntimeEnabledUser);
            }
            if (ok && !ctx.setMagRuntimeEnabled) {
                ok = magSaveConfigIfRequested(ctx, saveRequested);
            }
            if (ok) tracker_serial_detail::printOk(out, saveRequested ? "mag enabled and saved" : "mag enabled in RAM");
            else tracker_serial_detail::printErr(out, "mag enable failed");
            return;
        }

        if (tracker_serial_mag_detail::is(argv[1], "disable") || tracker_serial_mag_detail::is(argv[1], "off")) {
            const bool saveRequested = argc >= 3 && tracker_serial_mag_detail::is(argv[2], "save");
            if (ctx.config) {
                ctx.config->data.magCal.driverEnabled = false;
                ctx.config->updateCrc();
            }
            bool ok = true;
            if (ctx.setMagRuntimeEnabled) {
                ok = ctx.setMagRuntimeEnabled(false, saveRequested, ctx.setMagRuntimeEnabledUser);
            }
            if (ok && !ctx.setMagRuntimeEnabled) {
                ok = magSaveConfigIfRequested(ctx, saveRequested);
            }
            if (ok) tracker_serial_detail::printOk(out, saveRequested ? "mag disabled and saved" : "mag disabled in RAM");
            else tracker_serial_detail::printErr(out, "mag disable failed");
            return;
        }

        if (tracker_serial_mag_detail::is(argv[1], "cal")) {
            if (argc < 3 || tracker_serial_mag_detail::is(argv[2], "status") || tracker_serial_mag_detail::is(argv[2], "print")) {
                if (ctx.printMagCalibrationStatus) {
                    ctx.printMagCalibrationStatus(out, ctx.printMagCalibrationStatusUser);
                } else {
                    tracker_serial_detail::printErr(out, "mag calibration status hook not available");
                }
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "start")) {
                if (!ctx.config || !ctx.config->data.magCal.driverEnabled) {
                    tracker_serial_detail::printErr(out, "enable mag first: mag enable");
                    return;
                }
                if (!ctx.startMagCalibration) {
                    tracker_serial_detail::printErr(out, "mag calibration start hook not available");
                    return;
                }
                const bool ok = ctx.startMagCalibration(ctx.startMagCalibrationUser);
                if (ok) tracker_serial_detail::printOk(out, "mag calibration collection started");
                else tracker_serial_detail::printErr(out, "mag calibration start failed");
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "stop")) {
                if (ctx.stopMagCalibration) {
                    ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
                    tracker_serial_detail::printOk(out, "mag calibration collection stopped");
                } else {
                    tracker_serial_detail::printErr(out, "mag calibration stop hook not available");
                }
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "reset")) {
                if (ctx.resetMagCalibration) {
                    ctx.resetMagCalibration(ctx.resetMagCalibrationUser);
                    tracker_serial_detail::printOk(out, "mag calibration collector reset");
                } else {
                    tracker_serial_detail::printErr(out, "mag calibration reset hook not available");
                }
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "apply")) {
                const bool saveRequested = argc >= 4 && tracker_serial_mag_detail::is(argv[3], "save");
                if (!ctx.applyMagCalibration) {
                    tracker_serial_detail::printErr(out, "mag calibration apply hook not available");
                    return;
                }
                const bool ok = ctx.applyMagCalibration(saveRequested, ctx.applyMagCalibrationUser);
                if (ok) tracker_serial_detail::printOk(out, saveRequested ? "mag calibration applied and saved" : "mag calibration applied in RAM");
                else tracker_serial_detail::printErr(out, "mag calibration apply failed");
                return;
            }

            tracker_serial_detail::printErr(out, "unknown mag cal command");
            return;
        }

        if (tracker_serial_mag_detail::is(argv[1], "processed") || tracker_serial_mag_detail::is(argv[1], "trust")) {
            if (ctx.printMagProcessedStatus) {
                ctx.printMagProcessedStatus(out, ctx.printMagProcessedStatusUser);
            } else {
                tracker_serial_detail::printErr(out, "mag processed hook not available");
            }
            return;
        }

        if (tracker_serial_mag_detail::is(argv[1], "heading")) {
            if (argc < 3 || tracker_serial_mag_detail::is(argv[2], "status") || tracker_serial_mag_detail::is(argv[2], "print")) {
                if (ctx.printMagHeadingStatus) {
                    ctx.printMagHeadingStatus(out, ctx.printMagHeadingStatusUser);
                } else {
                    tracker_serial_detail::printErr(out, "mag heading hook not available");
                }
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "ref")) {
                if (!ctx.setMagHeadingReference) {
                    tracker_serial_detail::printErr(out, "mag heading ref hook not available");
                    return;
                }

                const bool ok = ctx.setMagHeadingReference(ctx.setMagHeadingReferenceUser);
                if (ok) {
                    tracker_serial_detail::printOk(out, "mag heading reference set");
                    if (ctx.printMagHeadingStatus) {
                        ctx.printMagHeadingStatus(out, ctx.printMagHeadingStatusUser);
                    }
                } else {
                    tracker_serial_detail::printErr(out, "mag heading reference set failed");
                }
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "clear")) {
                if (!ctx.clearMagHeadingReference) {
                    tracker_serial_detail::printErr(out, "mag heading clear hook not available");
                    return;
                }

                ctx.clearMagHeadingReference(ctx.clearMagHeadingReferenceUser);
                tracker_serial_detail::printOk(out, "mag heading reference cleared");

                if (ctx.printMagHeadingStatus) {
                    ctx.printMagHeadingStatus(out, ctx.printMagHeadingStatusUser);
                }
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "auto")) {
                if (argc < 4 || tracker_serial_mag_detail::is(argv[3], "status")) {
                    if (ctx.printMagHeadingStatus) {
                        ctx.printMagHeadingStatus(out, ctx.printMagHeadingStatusUser);
                    } else {
                        tracker_serial_detail::printErr(out, "mag heading status hook not available");
                    }
                    return;
                }

                if (tracker_serial_mag_detail::is(argv[3], "on") || tracker_serial_mag_detail::is(argv[3], "enable")) {
                    if (!ctx.setMagHeadingAutoReferenceEnabled) {
                        tracker_serial_detail::printErr(out, "mag heading auto hook not available");
                        return;
                    }

                    const bool ok = ctx.setMagHeadingAutoReferenceEnabled(true, ctx.setMagHeadingAutoReferenceEnabledUser);
                    if (ok) tracker_serial_detail::printOk(out, "mag heading auto-ref enabled");
                    else tracker_serial_detail::printErr(out, "mag heading auto-ref enable failed");
                    return;
                }

                if (tracker_serial_mag_detail::is(argv[3], "off") || tracker_serial_mag_detail::is(argv[3], "disable")) {
                    if (!ctx.setMagHeadingAutoReferenceEnabled) {
                        tracker_serial_detail::printErr(out, "mag heading auto hook not available");
                        return;
                    }

                    const bool ok = ctx.setMagHeadingAutoReferenceEnabled(false, ctx.setMagHeadingAutoReferenceEnabledUser);
                    if (ok) tracker_serial_detail::printOk(out, "mag heading auto-ref disabled");
                    else tracker_serial_detail::printErr(out, "mag heading auto-ref disable failed");
                    return;
                }

                tracker_serial_detail::printErr(out, "usage: mag heading auto on|off|status");
                return;
            }

            tracker_serial_detail::printErr(out, "unknown mag heading command");
            return;
        }

        if (tracker_serial_mag_detail::is(argv[1], "yaw")) {
            if (argc < 3 || tracker_serial_mag_detail::is(argv[2], "status")) {
                if (ctx.printMagYawCorrectionStatus) {
                    ctx.printMagYawCorrectionStatus(out, ctx.printMagYawCorrectionStatusUser);
                } else {
                    tracker_serial_detail::printErr(out, "mag yaw status hook not available");
                }
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "reset")) {
                if (ctx.resetMagYawCorrection) {
                    ctx.resetMagYawCorrection(ctx.resetMagYawCorrectionUser);
                    tracker_serial_detail::printOk(out, "mag yaw correction stats reset");
                } else {
                    tracker_serial_detail::printErr(out, "mag yaw reset hook not available");
                }
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "enable")) {
                const bool saveRequested = argc >= 4 && tracker_serial_mag_detail::is(argv[3], "save");

                if (ctx.setMagYawCorrectionApplyEnabled) {
                    const bool ok = ctx.setMagYawCorrectionApplyEnabled(true, saveRequested, ctx.setMagYawCorrectionApplyEnabledUser);
                    if (ok) tracker_serial_detail::printOk(out, saveRequested ? "mag yaw correction enabled and saved" : "mag yaw correction enabled");
                    else tracker_serial_detail::printErr(out, "mag yaw correction enable failed");
                } else {
                    tracker_serial_detail::printErr(out, "mag yaw enable hook not available");
                }
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "disable")) {
                const bool saveRequested = argc >= 4 && tracker_serial_mag_detail::is(argv[3], "save");

                if (ctx.setMagYawCorrectionApplyEnabled) {
                    const bool ok = ctx.setMagYawCorrectionApplyEnabled(false, saveRequested, ctx.setMagYawCorrectionApplyEnabledUser);
                    if (ok) tracker_serial_detail::printOk(out, saveRequested ? "mag yaw correction disabled and saved" : "mag yaw correction disabled");
                    else tracker_serial_detail::printErr(out, "mag yaw correction disable failed");
                } else {
                    tracker_serial_detail::printErr(out, "mag yaw disable hook not available");
                }
                return;
            }

            if (!ctx.config) {
                tracker_serial_detail::printErr(out, "config not available");
                return;
            }

            auto saveYawConfigIfRequested = [&](bool saveRequested) -> bool {
                ctx.config->sanitize();
                ctx.config->updateCrc();

                if (!saveRequested) return true;

                if (!ctx.configStore) return false;
                return ctx.configStore->save(*ctx.config);
            };

            auto resetYawStats = [&]() {
                if (ctx.resetMagYawCorrection) {
                    ctx.resetMagYawCorrection(ctx.resetMagYawCorrectionUser);
                }
            };

            if (tracker_serial_mag_detail::is(argv[2], "defaults")) {
                const bool saveRequested = argc >= 4 && tracker_serial_mag_detail::is(argv[3], "save");

                const bool keepApply = ctx.config->data.magYaw.applyEnabled;
                ctx.config->data.magYaw = TrackerMagYawCorrectionConfigPersisted{};
                ctx.config->data.magYaw.applyEnabled = keepApply;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw defaults save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw defaults applied and saved" : "mag yaw defaults applied");
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "tc")) {
                if (argc < 4) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw tc <seconds> [save]");
                    return;
                }

                float v = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], v)) {
                    tracker_serial_detail::printErr(out, "invalid time constant");
                    return;
                }

                const bool saveRequested = argc >= 5 && tracker_serial_mag_detail::is(argv[4], "save");
                ctx.config->data.magYaw.timeConstantS = v;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw tc save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw tc saved" : "mag yaw tc set");
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "innovation")) {
                if (argc < 4) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw innovation <deg> [save]");
                    return;
                }

                float v = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], v)) {
                    tracker_serial_detail::printErr(out, "invalid innovation");
                    return;
                }

                const bool saveRequested = argc >= 5 && tracker_serial_mag_detail::is(argv[4], "save");
                ctx.config->data.magYaw.maxInnovationDeg = v;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw innovation save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw innovation saved" : "mag yaw innovation set");
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "gyro_gate")) {
                if (argc < 5) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw gyro_gate <goodDps> <badDps> [save]");
                    return;
                }

                float good = 0.0f;
                float bad = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], good) ||
                    !tracker_serial_detail::parseFloat(argv[4], bad)) {
                    tracker_serial_detail::printErr(out, "invalid gyro gate");
                    return;
                }

                const bool saveRequested = argc >= 6 && tracker_serial_mag_detail::is(argv[5], "save");
                ctx.config->data.magYaw.gyroNormGoodDps = good;
                ctx.config->data.magYaw.gyroNormBadDps = bad;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw gyro gate save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw gyro gate saved" : "mag yaw gyro gate set");
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "horiz_gate")) {
                if (argc < 5) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw horiz_gate <bad> <good> [save]");
                    return;
                }

                float bad = 0.0f;
                float good = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], bad) ||
                    !tracker_serial_detail::parseFloat(argv[4], good)) {
                    tracker_serial_detail::printErr(out, "invalid horizontal gate");
                    return;
                }

                const bool saveRequested = argc >= 6 && tracker_serial_mag_detail::is(argv[5], "save");
                ctx.config->data.magYaw.horizontalNormBad = bad;
                ctx.config->data.magYaw.horizontalNormGood = good;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw horizontal gate save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw horizontal gate saved" : "mag yaw horizontal gate set");
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "accel_gate")) {
                if (argc < 5) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw accel_gate <badTrust> <goodTrust> [save]");
                    return;
                }

                float bad = 0.0f;
                float good = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], bad) ||
                    !tracker_serial_detail::parseFloat(argv[4], good)) {
                    tracker_serial_detail::printErr(out, "invalid accel gate");
                    return;
                }

                const bool saveRequested = argc >= 6 && tracker_serial_mag_detail::is(argv[5], "save");
                ctx.config->data.magYaw.accelTrustBad = bad;
                ctx.config->data.magYaw.accelTrustGood = good;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw accel gate save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw accel gate saved" : "mag yaw accel gate set");
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "age")) {
                if (argc < 4) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw age <ms> [save]");
                    return;
                }

                uint32_t v = 0;
                if (!tracker_serial_detail::parseU32(argv[3], v)) {
                    tracker_serial_detail::printErr(out, "invalid age");
                    return;
                }

                const bool saveRequested = argc >= 5 && tracker_serial_mag_detail::is(argv[4], "save");
                ctx.config->data.magYaw.maxMagAgeMs = v;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw age save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw age saved" : "mag yaw age set");
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "rate")) {
                if (argc < 4) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw rate <deg_s> [save]");
                    return;
                }

                float v = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], v)) {
                    tracker_serial_detail::printErr(out, "invalid max rate");
                    return;
                }

                const bool saveRequested = argc >= 5 && tracker_serial_mag_detail::is(argv[4], "save");
                ctx.config->data.magYaw.maxCorrectionRateDegS = v;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw rate save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw rate saved" : "mag yaw rate set");
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "step")) {
                if (argc < 4) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw step <deg> [save]");
                    return;
                }

                float v = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], v)) {
                    tracker_serial_detail::printErr(out, "invalid max step");
                    return;
                }

                const bool saveRequested = argc >= 5 && tracker_serial_mag_detail::is(argv[4], "save");
                ctx.config->data.magYaw.maxCorrectionStepDeg = v;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw step save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw step saved" : "mag yaw step set");
                return;
            }

            tracker_serial_detail::printErr(out, "unknown mag yaw command");
            return;
        }

        if (tracker_serial_mag_detail::is(argv[1], "axis")) {
            if (!ctx.config) {
                tracker_serial_detail::printErr(out, "config not available");
                return;
            }

            if (argc < 3 || tracker_serial_mag_detail::is(argv[2], "print")) {
                printMagAxisMatrix(out, *ctx.config);
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "set")) {
                if (argc < 6) {
                    tracker_serial_detail::printErr(out, "usage: mag axis set <bodyX> <bodyY> <bodyZ> [save]");
                    out.println("# examples:");
                    out.println("#   mag axis set +x +y +z");
                    out.println("#   mag axis set +y -x +z");
                    out.println("#   mag axis set -y +x +z save");
                    return;
                }

                Mat3 m = Mat3::identity();
                if (!makeMagAxisMatrixFromTokens(argv[3], argv[4], argv[5], m)) {
                    tracker_serial_detail::printErr(out, "invalid axis mapping; use each of x/y/z exactly once, with optional +/-");
                    out.println("# valid examples:");
                    out.println("#   mag axis set +x +y +z");
                    out.println("#   mag axis set +y -x +z");
                    out.println("#   mag axis set -x +z +y");
                    return;
                }

                const bool saveRequested = argc >= 7 && tracker_serial_mag_detail::is(argv[6], "save");

                ctx.config->data.magCal.magToImu = m;
                ctx.config->data.magCal.axisAlignmentValid = true;
                ctx.config->updateCrc();

                if (!saveConfigIfRequested(ctx, saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag axis save failed");
                    return;
                }

                if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
                tracker_serial_detail::printOk(out, saveRequested ? "mag axis mapping saved" : "mag axis mapping set in RAM");
                printMagAxisMatrix(out, *ctx.config);
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "identity")) {
                const bool saveRequested = argc >= 4 && tracker_serial_mag_detail::is(argv[3], "save");

                ctx.config->data.magCal.magToImu = Mat3::identity();
                ctx.config->data.magCal.axisAlignmentValid = true;
                ctx.config->updateCrc();

                if (!saveConfigIfRequested(ctx, saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag axis identity save failed");
                    return;
                }

                if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
                tracker_serial_detail::printOk(out, saveRequested ? "mag axis identity saved" : "mag axis identity set in RAM");
                printMagAxisMatrix(out, *ctx.config);
                return;
            }

            if (tracker_serial_mag_detail::is(argv[2], "clear")) {
                const bool saveRequested = argc >= 4 && tracker_serial_mag_detail::is(argv[3], "save");

                ctx.config->data.magCal.magToImu = Mat3::identity();
                ctx.config->data.magCal.axisAlignmentValid = false;
                ctx.config->updateCrc();

                if (!saveConfigIfRequested(ctx, saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag axis clear save failed");
                    return;
                }

                if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
                tracker_serial_detail::printOk(out, saveRequested ? "mag axis cleared and saved" : "mag axis cleared in RAM");
                printMagAxisMatrix(out, *ctx.config);
                return;
            }

            tracker_serial_detail::printErr(out, "unknown mag axis command");
            return;
        }

        if (!ctx.mag) {
            tracker_serial_detail::printErr(out, "mag driver not available");
            return;
        }

        if (tracker_serial_mag_detail::is(argv[1], "id")) {
            uint8_t id = 0;
            const bool ok = ctx.mag->probe(&id);
            out.print(ok ? "# OK " : "# ERR ");
            out.print("qmc_id=0x"); out.print(id, HEX);
            out.print(" expected=0x"); out.print(Qmc6309::EXPECTED_CHIP_ID, HEX);
            out.print(" qmcErr="); out.print(ctx.mag->lastErrorName());
            if (ctx.sensorHub) { out.print(" hubErr="); out.print(ctx.sensorHub->lastErrorName()); }
            out.println();
            (void)rearmMagIfNeeded(ctx);
            return;
        }

        if (tracker_serial_mag_detail::is(argv[1], "qmcstatus")) {
            Qmc6309::Status st;
            const bool ok = ctx.mag->readStatus(st);
            out.print(ok ? "# OK " : "# ERR ");
            out.print("qmc_status=0x"); out.print(st.raw, HEX);
            out.print(" drdy="); out.print(st.dataReady ? 1 : 0);
            out.print(" ovfl="); out.print(st.overflow ? 1 : 0);
            out.print(" nvm_ready="); out.print(st.nvmReady ? 1 : 0);
            out.print(" nvm_load_done="); out.print(st.nvmLoadDone ? 1 : 0);
            out.print(" qmcErr="); out.print(ctx.mag->lastErrorName());
            if (ctx.sensorHub) { out.print(" hubErr="); out.print(ctx.sensorHub->lastErrorName()); }
            out.println();
            (void)rearmMagIfNeeded(ctx);
            return;
        }

        if (tracker_serial_mag_detail::is(argv[1], "regs")) {
            out.println("# QMC REGS 00..0B");
            for (uint8_t r = 0; r <= 0x0B; ++r) {
                uint8_t v = 0;
                const bool ok = ctx.mag->readReg(r, v);
                out.print("0x"); if (r < 0x10) out.print('0'); out.print(r, HEX);
                out.print('=');
                if (ok) { out.print("0x"); if (v < 0x10) out.print('0'); out.print(v, HEX); }
                else out.print("ERR");
                out.println();
            }
            (void)rearmMagIfNeeded(ctx);
            return;
        }

        if (tracker_serial_mag_detail::is(argv[1], "hub")) {
            if (!ctx.sensorHub) {
                tracker_serial_detail::printErr(out, "sensor hub not available");
                return;
            }
            Lsm6dsvSensorHub::MasterStatus st;
            const bool ok = ctx.sensorHub->readMasterStatus(st);
            out.print(ok ? "# OK " : "# ERR ");
            out.print("hub_status=0x"); out.print(st.raw, HEX);
            out.print(" endop="); out.print(st.endop ? 1 : 0);
            out.print(" write_once_done="); out.print(st.writeOnceDone ? 1 : 0);
            out.print(" nack0="); out.print(st.slave0Nack ? 1 : 0);
            out.print(" nack1="); out.print(st.slave1Nack ? 1 : 0);
            out.print(" nack2="); out.print(st.slave2Nack ? 1 : 0);
            out.print(" nack3="); out.print(st.slave3Nack ? 1 : 0);
            out.print(" hubErr="); out.println(ctx.sensorHub->lastErrorName());
            return;
        }

        if (tracker_serial_mag_detail::is(argv[1], "fifo")) {
            if (!ctx.fifo) {
                tracker_serial_detail::printErr(out, "fifo not available");
                return;
            }
            const auto& fs = ctx.fifo->stats();
            out.println("# MAG FIFO");
            out.print("sensorhub0_words="); out.println(fs.sensorHubSlave0Words);
            out.print("sensorhub_nack_words="); out.println(fs.sensorHubNackWords);
            out.print("mag_samples_produced="); out.println(fs.magSamplesProduced);
            out.print("mag_queue_overflow="); out.println(fs.magQueueOverflow);
            out.print("mag_tag_counter_jumps="); out.println(fs.magTagCounterJumps);
            out.print("mag_last_xyz="); out.print(fs.lastMagX); out.print(','); out.print(fs.lastMagY); out.print(','); out.println(fs.lastMagZ);
            out.print("mag_last_norm_raw="); out.println(fs.lastMagRawNorm, 3);
            out.print("mag_dt_last_us="); out.println(fs.lastMagDtUs);
            out.print("mag_dt_min_us="); out.println(fs.minMagDtUs);
            out.print("mag_dt_max_us="); out.println(fs.maxMagDtUs);
            return;
        }

        tracker_serial_detail::printErr(out, "unknown mag command");
    }


} // namespace tracker
