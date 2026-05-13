#include "serial/tracker_calibration_commands.hpp"

#include <Arduino.h>
#include <cstdint>

#include "core/math.hpp"
#include "sensor/accel_6pos_calibration.hpp"
#include "sensor/fifo_calibrations.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "serial/tracker_serial_context.hpp"
#include "serial/tracker_config_commands.hpp"

namespace tracker {

class TrackerCalibrationCommandDispatcher {
public:
    static void dispatch(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (argc < 2) {
            tracker_serial_detail::printErr(out, "usage: cal gyro|accel|temp|save|clear_all");
            return;
        }

        if (is(argv[1], "gyro")) {
            cmdCalGyro(ctx, argc, argv);
            return;
        }
        if (is(argv[1], "accel")) {
            cmdCalAccel(ctx, argc, argv);
            return;
        }
        if (is(argv[1], "temp")) {
            cmdCalTemp(ctx, argc, argv);
            return;
        }
        if (is(argv[1], "save")) {
            if (!ctx.config || !ctx.configStore) {
                tracker_serial_detail::printErr(out, "config/configStore not available");
                return;
            }
            trackerSerialCaptureRuntimeToConfig(ctx);
            if (ctx.configStore->save(*ctx.config)) tracker_serial_detail::printOk(out, "calibration saved");
            else {
                out.print("# ERR calibration save failed: ");
                out.println(ctx.configStore->lastErrorName());
            }
            return;
        }
        if (is(argv[1], "clear_all")) {
            if (ctx.imuCal) {
                ctx.imuCal->gyroBiasValid = false;
                ctx.imuCal->gyroBiasRadS = Vec3::zero();
                ctx.imuCal->accelCalValid = false;
                ctx.imuCal->accelBiasG = Vec3::zero();
                ctx.imuCal->accelScale = Mat3::identity();
            }
            if (ctx.config) {
                ctx.config->data.gyroCal = TrackerGyroCalibrationConfig{};
                ctx.config->data.gyroCalMeta = TrackerGyroCalibrationMetaPersisted{};
                ctx.config->data.accelCal = TrackerAccelCalibrationConfig{};
                ctx.config->data.accelCalQuality = TrackerAccelCalibrationQualityPersisted{};
                ctx.config->data.magCal = TrackerMagCalibrationConfig{};
                ctx.config->data.magCalQuality = TrackerMagCalibrationQualityPersisted{};
                ctx.config->updateCrc();
            }
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
            tracker_serial_detail::printOk(out, "all calibration cleared in RAM");
            return;
        }

        tracker_serial_detail::printErr(out, "unknown cal command");
    }

private:
    static void cmdCalGyro(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (argc >= 3 && is(argv[2], "save")) {
            if (!ctx.config || !ctx.configStore || !ctx.imuCal) {
                tracker_serial_detail::printErr(out, "config or calibration not available");
                return;
            }
            ctx.config->captureFromImuCalibration(*ctx.imuCal);
            ctx.config->noteGyroBiasCalibrationCaptured(millis());
            if (ctx.gyroTempComp) ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
            if (ctx.configStore->save(*ctx.config)) tracker_serial_detail::printOk(out, "gyro calibration saved");
            else {
                out.print("# ERR gyro save failed: ");
                out.println(ctx.configStore->lastErrorName());
            }
            return;
        }

        if (argc >= 3 && is(argv[2], "clear")) {
            if (ctx.imuCal) {
                ctx.imuCal->gyroBiasValid = false;
                ctx.imuCal->gyroBiasRadS = Vec3::zero();
            }
            if (ctx.config) {
                ctx.config->data.gyroCal = TrackerGyroCalibrationConfig{};
                ctx.config->data.gyroCalMeta = TrackerGyroCalibrationMetaPersisted{};
                ctx.config->updateCrc();
            }
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
            tracker_serial_detail::printOk(out, "gyro calibration cleared in RAM");
            return;
        }

        if (!ctx.calibrationIo || !ctx.imuCal) {
            tracker_serial_detail::printErr(out, "calibration IO or imuCal not available");
            return;
        }

        out.println("# gyro calibration started; keep tracker still");

        FifoGyroStartupCalibrationParams params;
        FifoGyroStartupCalibrator cal(params);
        GyroStartupCalibrationResult result;

        GyroProgressPrinter pp;
        pp.out = &out;
        pp.lastPrintMs = 0;

        const bool ok = cal.run(*ctx.calibrationIo, result, &gyroProgressCallback, &pp);
        printGyroResult(out, result, ctx.calibrationIo->latestTempC);

        if (!ok) {
            tracker_serial_detail::printErr(out, "gyro calibration failed");
            return;
        }

        FifoGyroStartupCalibrator::applyResultToCalibration(
            result,
            *ctx.imuCal,
            ctx.gyroTempComp,
            ctx.calibrationIo->latestTempC
        );

        if (ctx.config) {
            ctx.config->captureFromImuCalibration(*ctx.imuCal);
            ctx.config->noteGyroBiasCalibrationCaptured(millis());
            if (ctx.gyroTempComp) ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
        }
        if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);

        tracker_serial_detail::printOk(out, "gyro calibration applied to RAM; use cal gyro save or config save");
    }

    static void cmdCalAccel(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.accelCalRunner) {
            tracker_serial_detail::printErr(out, "accel calibration runner not available");
            return;
        }

        if (argc < 3) {
            tracker_serial_detail::printErr(out, "usage: cal accel face|compute|dump|save|clear");
            return;
        }

        if (is(argv[2], "face")) {
            if (argc < 4) {
                tracker_serial_detail::printErr(out, "usage: cal accel face XP|XN|YP|YN|ZP|ZN");
                return;
            }
            if (!ctx.calibrationIo) {
                tracker_serial_detail::printErr(out, "calibration IO not available");
                return;
            }
            const Accel6PosCalibration::Face face = Accel6PosCalibration::parseFace(argv[3]);
            if (face == Accel6PosCalibration::Face::Invalid) {
                tracker_serial_detail::printErr(out, "invalid face; use XP XN YP YN ZP ZN");
                return;
            }

            out.print("# accel face capture started: ");
            out.println(Accel6PosCalibration::faceName(face));

            AccelProgressPrinter pp;
            pp.out = &out;
            pp.lastPrintMs = 0;
            const bool ok = ctx.accelCalRunner->captureFace(*ctx.calibrationIo, face, &accelProgressCallback, &pp);
            if (ok) {
                out.print("# OK accel face captured: ");
                out.println(Accel6PosCalibration::faceName(face));
            } else {
                tracker_serial_detail::printErr(out, "accel face capture failed");
            }
            return;
        }

        if (is(argv[2], "compute")) {
            if (!ctx.accelCalRunner->compute()) {
                printAccelCal(out, ctx.accelCalRunner->calibration());
                tracker_serial_detail::printErr(out, "accel calibration rejected by quality gates; see quality_flags");
                return;
            }
            printAccelCal(out, ctx.accelCalRunner->calibration());
            if (ctx.imuCal) ctx.accelCalRunner->applyToImuCalibration(*ctx.imuCal);
            if (ctx.config && ctx.imuCal) {
                ctx.config->captureFromImuCalibration(*ctx.imuCal);
                ctx.config->captureFromAccelCalibrationQuality(ctx.accelCalRunner->calibration(), millis());
            }
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
            tracker_serial_detail::printOk(out, "accel calibration computed and applied to RAM");
            return;
        }

        if (is(argv[2], "dump")) {
            printAccelCal(out, ctx.accelCalRunner->calibration());
            return;
        }

        if (is(argv[2], "save")) {
            if (!ctx.config || !ctx.configStore || !ctx.imuCal) {
                tracker_serial_detail::printErr(out, "config/configStore/imuCal not available");
                return;
            }
            ctx.config->captureFromImuCalibration(*ctx.imuCal);
            if (ctx.accelCalRunner->calibration().result().valid) {
                ctx.config->captureFromAccelCalibrationQuality(ctx.accelCalRunner->calibration(), millis());
            }
            if (ctx.configStore->save(*ctx.config)) tracker_serial_detail::printOk(out, "accel calibration saved");
            else {
                out.print("# ERR accel save failed: ");
                out.println(ctx.configStore->lastErrorName());
            }
            return;
        }

        if (is(argv[2], "clear")) {
            ctx.accelCalRunner->reset();
            if (ctx.imuCal) {
                ctx.imuCal->accelCalValid = false;
                ctx.imuCal->accelBiasG = Vec3::zero();
                ctx.imuCal->accelScale = Mat3::identity();
            }
            if (ctx.config) {
                ctx.config->data.accelCal = TrackerAccelCalibrationConfig{};
                ctx.config->data.accelCalQuality = TrackerAccelCalibrationQualityPersisted{};
                ctx.config->updateCrc();
            }
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
            tracker_serial_detail::printOk(out, "accel calibration cleared in RAM");
            return;
        }

        tracker_serial_detail::printErr(out, "unknown cal accel command");
    }

    static void cmdCalTemp(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);

        if (!ctx.gyroTempComp) {
            tracker_serial_detail::printErr(out, "gyro temp comp not available");
            return;
        }

        auto saveTempConfigIfRequested = [&](bool saveRequested) -> bool {
            if (!ctx.config) return false;

            ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
            ctx.config->sanitize();
            ctx.config->updateCrc();

            if (!saveRequested) return true;

            if (!ctx.configStore) return false;
            return ctx.configStore->save(*ctx.config);
        };

        const float currentTempC =
            ctx.calibrationIo ? ctx.calibrationIo->latestTempC : 25.0f;

        if (argc < 3 || is(argv[2], "print")) {
            const auto s = ctx.gyroTempComp->snapshot(currentTempC);

            out.println("# GYRO TEMP COMP");
            out.print("temp_comp_valid="); out.println(s.valid ? "yes" : "no");
            out.print("temp_comp_enabled="); out.println(s.enabled ? "yes" : "no");
            out.print("temp_learning_enabled="); out.println(s.learningEnabled ? "yes" : "no");

            out.print("current_temp_c="); out.println(s.currentTempC, 3);
            out.print("reference_temp_c="); out.println(s.referenceTempC, 3);
            out.print("delta_temp_c="); out.println(s.deltaTempC, 3);

            tracker_serial_detail::printVec3Line(out, "reference_bias_dps", s.referenceBiasDps, 6);
            tracker_serial_detail::printVec3Line(out, "slope_dps_per_c", s.slopeDpsPerC, 8);
            tracker_serial_detail::printVec3Line(out, "current_bias_dps", s.currentBiasDps, 6);
            out.print("calibrated_range_valid="); out.println(s.hasCalibratedRange ? "yes" : "no");
            out.print("calibrated_temp_min_c="); out.println(s.calibratedTempMinC, 3);
            out.print("calibrated_temp_max_c="); out.println(s.calibratedTempMaxC, 3);
            out.print("temp_out_of_range="); out.println(s.tempOutOfRange ? "yes" : "no");
            out.print("fit_quality="); out.println(s.fitQuality, 6);
            out.print("fit_residual_before_dps="); out.println(s.fitResidualBeforeDps, 6);
            out.print("fit_residual_after_dps="); out.println(s.fitResidualAfterDps, 6);

            out.print("learn_accepted="); out.println(s.learnAccepted);
            out.print("learn_rejected="); out.println(s.learnRejected);
            return;
        }

        if (is(argv[2], "enable")) {
            const bool saveRequested = argc >= 4 && is(argv[3], "save");

            ctx.gyroTempComp->setEnabled(true);

            if (!saveTempConfigIfRequested(saveRequested)) {
                tracker_serial_detail::printErr(out, "cal temp enable save failed");
                return;
            }

            tracker_serial_detail::printOk(out, saveRequested ? "gyro temp compensation enabled and saved" : "gyro temp compensation enabled");
            return;
        }

        if (is(argv[2], "disable")) {
            const bool saveRequested = argc >= 4 && is(argv[3], "save");

            ctx.gyroTempComp->setEnabled(false);

            if (!saveTempConfigIfRequested(saveRequested)) {
                tracker_serial_detail::printErr(out, "cal temp disable save failed");
                return;
            }

            tracker_serial_detail::printOk(out, saveRequested ? "gyro temp compensation disabled and saved" : "gyro temp compensation disabled");
            return;
        }

        if (is(argv[2], "set_slope")) {
            if (argc < 6) {
                tracker_serial_detail::printErr(out, "usage: cal temp set_slope X Y Z [save] ; values in dps/C");
                return;
            }

            float x = 0, y = 0, z = 0;
            if (!tracker_serial_detail::parseFloat(argv[3], x) ||
                !tracker_serial_detail::parseFloat(argv[4], y) ||
                !tracker_serial_detail::parseFloat(argv[5], z)) {
                tracker_serial_detail::printErr(out, "invalid slope values");
                return;
            }

            const bool saveRequested = argc >= 7 && is(argv[6], "save");

            ctx.gyroTempComp->setSlopeDpsPerC(Vec3(x, y, z));

            if (!saveTempConfigIfRequested(saveRequested)) {
                tracker_serial_detail::printErr(out, "cal temp set_slope save failed");
                return;
            }

            tracker_serial_detail::printOk(out, saveRequested ? "temperature slope applied and saved" : "temperature slope applied to RAM");
            return;
        }

        if (is(argv[2], "fit_static")) {
            const bool saveRequested = argc >= 4 && is(argv[3], "save");

            if (!ctx.fitGyroTempFromLastStatic) {
                tracker_serial_detail::printErr(out, "fit_static hook not available");
                return;
            }

            const bool ok = ctx.fitGyroTempFromLastStatic(
                saveRequested,
                out,
                ctx.fitGyroTempFromLastStaticUser
            );

            if (!ok) {
                tracker_serial_detail::printErr(out, "temperature fit from static test failed");
            }
            return;
        }

        if (is(argv[2], "clear")) {
            const bool saveRequested = argc >= 4 && is(argv[3], "save");

            ctx.gyroTempComp->setSlopeRadSPerC(Vec3::zero());

            if (ctx.config) {
                ctx.config->data.gyroCal.tempCompValid = false;
                ctx.config->data.gyroCal.tempSlopeRadSPerC = Vec3::zero();
                ctx.config->data.gyroTempQuality = TrackerGyroTempQualityConfigPersisted{};
                ctx.config->sanitize();
                ctx.config->updateCrc();

                if (saveRequested) {
                    if (!ctx.configStore || !ctx.configStore->save(*ctx.config)) {
                        tracker_serial_detail::printErr(out, "cal temp clear save failed");
                        return;
                    }
                }
            }

            tracker_serial_detail::printOk(out, saveRequested ? "temperature compensation cleared and saved" : "temperature compensation cleared in RAM");
            return;
        }

        tracker_serial_detail::printErr(out, "unknown cal temp command");
    }

    struct GyroProgressPrinter {
        Stream* out = nullptr;
        uint32_t lastPrintMs = 0;
    };

    static void gyroProgressCallback(const FifoGyroStartupCalibrationProgress& p, void* user) {
        GyroProgressPrinter* pp = static_cast<GyroProgressPrinter*>(user);
        if (!pp || !pp->out) return;
        const uint32_t now = millis();
        if (now - pp->lastPrintMs < 500) return;
        pp->lastPrintMs = now;

        pp->out->print("# cal gyro still=");
        pp->out->print(p.stationarySamples);
        pp->out->print('/');
        pp->out->print(p.requiredStationarySamples);
        pp->out->print(" total=");
        pp->out->print(p.totalSamples);
        pp->out->print(" rejected=");
        pp->out->print(p.rejectedSamples);
        pp->out->print(" temp_c=");
        pp->out->println(p.latestTempC, 3);
    }

    static void printGyroResult(Stream& out, const GyroStartupCalibrationResult& r, float tempC) {
        out.println("# GYRO CAL RESULT");
        out.print("success="); out.println(r.success ? "yes" : "no");
        out.print("stationary_samples="); out.println(r.stationarySamples);
        out.print("total_samples="); out.println(r.totalSamples);
        tracker_serial_detail::printVec3Line(out, "gyro_bias_rad_s", r.gyroBiasRadS, 8);
        tracker_serial_detail::printVec3Line(out, "gyro_bias_dps", r.gyroBiasDps, 5);
        tracker_serial_detail::printVec3(out, "accel_mean_g", r.accelMeanG, 6);
        out.print(" norm="); out.println(r.accelNormMeanG, 6);
        out.print("gyro_noise_norm_rad_s2="); out.println(r.gyroNoiseNormRadS2, 10);
        out.print("accel_noise_norm_g2="); out.println(r.accelNoiseNormG2, 10);
        out.print("reference_temp_c="); out.println(tempC, 3);
    }

    struct AccelProgressPrinter {
        Stream* out = nullptr;
        uint32_t lastPrintMs = 0;
    };

    static void accelProgressCallback(const FifoAccel6PosCaptureProgress& p, void* user) {
        AccelProgressPrinter* pp = static_cast<AccelProgressPrinter*>(user);
        if (!pp || !pp->out) return;
        const uint32_t now = millis();
        if (now - pp->lastPrintMs < 250) return;
        pp->lastPrintMs = now;

        pp->out->print("# cal accel face=");
        pp->out->print(Accel6PosCalibration::faceName(p.face));
        pp->out->print(" accepted=");
        pp->out->print(p.acceptedSamples);
        pp->out->print('/');
        pp->out->print(p.requiredSamples);
        pp->out->print(" rejected=");
        pp->out->print(p.rejectedSamples);
        pp->out->print(" mean=");
        pp->out->print(p.meanG.x, 5); pp->out->print(',');
        pp->out->print(p.meanG.y, 5); pp->out->print(',');
        pp->out->print(p.meanG.z, 5);
        pp->out->print(" norm=");
        pp->out->println(p.meanNormG, 6);
    }

    static void printAccelCal(Stream& out, const Accel6PosCalibration& cal) {
        out.println("# ACCEL CAL DUMP");
        printAccelFace(out, cal, Accel6PosCalibration::Face::XP);
        printAccelFace(out, cal, Accel6PosCalibration::Face::XN);
        printAccelFace(out, cal, Accel6PosCalibration::Face::YP);
        printAccelFace(out, cal, Accel6PosCalibration::Face::YN);
        printAccelFace(out, cal, Accel6PosCalibration::Face::ZP);
        printAccelFace(out, cal, Accel6PosCalibration::Face::ZN);

        const auto& r = cal.result();
        out.print("result_valid="); out.println(r.valid ? "yes" : "no");
        out.print("quality_score="); out.println(r.qualityScore, 6);
        out.print("quality_flags=0x"); out.println(r.qualityFlags, HEX);
        printAccelQualityFlags(out, r.qualityFlags);
        tracker_serial_detail::printVec3Line(out, "accel_bias_g", r.biasG, 8);
        tracker_serial_detail::printVec3Line(out, "accel_scale_diag", r.scale, 8);
        out.print("accel_matrix_row0="); out.print(r.scaleMatrix.m[0][0], 8); out.print(','); out.print(r.scaleMatrix.m[0][1], 8); out.print(','); out.println(r.scaleMatrix.m[0][2], 8);
        out.print("accel_matrix_row1="); out.print(r.scaleMatrix.m[1][0], 8); out.print(','); out.print(r.scaleMatrix.m[1][1], 8); out.print(','); out.println(r.scaleMatrix.m[1][2], 8);
        out.print("accel_matrix_row2="); out.print(r.scaleMatrix.m[2][0], 8); out.print(','); out.print(r.scaleMatrix.m[2][1], 8); out.print(','); out.println(r.scaleMatrix.m[2][2], 8);
        out.print("max_face_norm_error_g="); out.println(r.maxFaceNormErrorG, 8);
        out.print("max_axis_residual_g="); out.println(r.maxAxisResidualG, 8);
        out.print("face_norm_errors_g=");
        for (uint8_t i = 0; i < 6; ++i) {
            if (i) out.print(',');
            out.print(r.faceNormErrorG[i], 8);
        }
        out.println();
        out.print("face_axis_residuals_g=");
        for (uint8_t i = 0; i < 6; ++i) {
            if (i) out.print(',');
            out.print(r.faceAxisResidualG[i], 8);
        }
        out.println();
    }

    static void printAccelQualityFlags(Stream& out, uint32_t flags) {
        if (flags == accel_cal_quality_flags::OK) {
            out.println("quality_flag_names=OK");
            return;
        }

        out.print("quality_flag_names=");
        bool first = true;
        const uint32_t known[] = {
            accel_cal_quality_flags::MISSING_FACE,
            accel_cal_quality_flags::TOO_FEW_SAMPLES,
            accel_cal_quality_flags::FACE_VARIANCE_HIGH,
            accel_cal_quality_flags::FACE_NORM_IMPLAUSIBLE,
            accel_cal_quality_flags::FACE_DIRECTION_BAD,
            accel_cal_quality_flags::AXIS_SEPARATION_LOW,
            accel_cal_quality_flags::BIAS_IMPLAUSIBLE,
            accel_cal_quality_flags::SCALE_IMPLAUSIBLE,
            accel_cal_quality_flags::NORM_RESIDUAL_HIGH,
            accel_cal_quality_flags::AXIS_RESIDUAL_HIGH,
        };
        for (uint8_t i = 0; i < sizeof(known) / sizeof(known[0]); ++i) {
            if ((flags & known[i]) == 0) continue;
            if (!first) out.print('|');
            first = false;
            out.print(Accel6PosCalibration::qualityFlagName(known[i]));
        }
        out.println();
    }

    static void printAccelFace(Stream& out, const Accel6PosCalibration& cal, Accel6PosCalibration::Face face) {
        out.print(Accel6PosCalibration::faceName(face));
        out.print('=');
        if (!cal.hasFace(face)) {
            out.println("missing");
            return;
        }
        const auto& d = cal.faceData(face);
        out.print("samples:"); out.print(d.samples);
        out.print(",mean:");
        out.print(d.meanG.x, 6); out.print(',');
        out.print(d.meanG.y, 6); out.print(',');
        out.print(d.meanG.z, 6);
        out.print(",norm:"); out.println(d.meanNormG, 6);
    }


private:
    static Stream& stream(TrackerSerialCommandContext& ctx) {
        return ctx.io ? *ctx.io : Serial;
    }

    static bool is(const char* a, const char* b) {
        return tracker_serial_detail::eqIgnoreCase(a, b);
    }
};

void trackerSerialDispatchCalibrationCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    TrackerCalibrationCommandDispatcher::dispatch(ctx, argc, argv);
}

} // namespace tracker
