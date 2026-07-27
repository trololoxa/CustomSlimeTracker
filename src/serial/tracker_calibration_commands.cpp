#include "serial/tracker_calibration_commands.hpp"

#include <Arduino.h>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>

#include "core/math.hpp"
#include "sensor/accel_6pos_calibration.hpp"
#include "sensor/fifo_calibrations.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "runtime/runtime_gyro_bias_controller.hpp"
#if TRACKER_HAS_CALIBRATION_AUTONOMY
#include "runtime/calibration_autonomy_controller.hpp"
#endif
#include "runtime/gyro_temp_calibration_capture.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "serial/tracker_serial_context.hpp"
#include "serial/tracker_config_commands.hpp"
#include "serial/tracker_fifo_config_control.hpp"

namespace tracker {

void trackerSerialResetCalibrationWorkspaces(TrackerSerialCommandContext& ctx, uint8_t mask) {
    if ((mask & TRACKER_CAL_WORKSPACE_ACCEL) != 0u && ctx.accelCalRunner) {
        ctx.accelCalRunner->reset();
    }
    if ((mask & TRACKER_CAL_WORKSPACE_GYRO_TEMP) != 0u && ctx.gyroTempCapture) {
        ctx.gyroTempCapture->reset();
    }
    if ((mask & TRACKER_CAL_WORKSPACE_MAG) != 0u && ctx.resetMagCalibration) {
        ctx.resetMagCalibration(ctx.resetMagCalibrationUser);
    }
}

static bool gyroTempRuntimeModelEqual(const GyroTempCompensator& a,
                                      const GyroTempCompensator& b) {
    return a.valid() == b.valid() &&
        a.temperatureModelValid() == b.temperatureModelValid() &&
        a.config().enabled == b.config().enabled &&
        a.referenceTempC() == b.referenceTempC() &&
        a.referenceBiasRadS().x == b.referenceBiasRadS().x &&
        a.referenceBiasRadS().y == b.referenceBiasRadS().y &&
        a.referenceBiasRadS().z == b.referenceBiasRadS().z &&
        a.slopeRadSPerC().x == b.slopeRadSPerC().x &&
        a.slopeRadSPerC().y == b.slopeRadSPerC().y &&
        a.slopeRadSPerC().z == b.slopeRadSPerC().z;
}


class CalibrationManualOwnershipScope {
public:
    CalibrationManualOwnershipScope(TrackerSerialCommandContext& ctx, bool required)
        : ctx_(ctx), required_(required) {
#if TRACKER_HAS_CALIBRATION_AUTONOMY
        if (required_ && ctx_.calibrationAutonomy) {
            acquired_ = ctx_.calibrationAutonomy->beginManualCalibration(millis());
        } else {
            acquired_ = true;
        }
#else
        acquired_ = true;
#endif
    }

    ~CalibrationManualOwnershipScope() {
#if TRACKER_HAS_CALIBRATION_AUTONOMY
        if (required_ && acquired_ && ctx_.calibrationAutonomy) {
            // Conservatively invalidate all background evidence even when a
            // command exits early: blocking calibration commands may have
            // already changed RAM or persisted a checkpoint.
            ctx_.calibrationAutonomy->endManualCalibration(true, millis());
        }
#endif
    }

    bool acquired() const { return acquired_; }

private:
    TrackerSerialCommandContext& ctx_;
    bool required_ = false;
    bool acquired_ = false;
};

class TrackerCalibrationCommandDispatcher {
public:
    static void dispatch(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (argc < 2) {
            tracker_serial_detail::printErr(out, "usage: cal status|autonomy|gyro|accel|temp|candidate|save|clear_all|erase_all confirm");
            return;
        }

        if (is(argv[1], "autonomy")) {
            cmdCalAutonomy(ctx, argc, argv);
            return;
        }
        if (is(argv[1], "status")) {
            cmdCalStatus(ctx);
            return;
        }
        // Full erase is the recovery command for damaged/obsolete autonomy
        // state. It must not pass through beginManualCalibration(), because that
        // path intentionally refuses ambiguous journals.
        if (is(argv[1], "erase_all")) {
            cmdCalEraseAll(ctx, argc, argv);
            return;
        }

        CalibrationManualOwnershipScope ownership(ctx, commandMutatesCalibration(argc, argv));
        if (!ownership.acquired()) {
            tracker_serial_detail::printErr(
                out,
                "manual calibration blocked: autonomous transaction could not be resolved");
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
        if (is(argv[1], "candidate")) {
            cmdCalCandidate(ctx, argc, argv);
            return;
        }
        if (is(argv[1], "save")) {
            if (!ctx.config || !ctx.configStore) {
                tracker_serial_detail::printErr(out, "config/configStore not available");
                return;
            }
            trackerSerialCaptureRuntimeToConfig(ctx);
            if (ctx.configStore->save(*ctx.config, TrackerCalibrationProvenance::Manual)) tracker_serial_detail::printOk(out, "calibration saved");
            else {
                out.print("# ERR calibration save failed: ");
                out.println(ctx.configStore->lastErrorName());
            }
            return;
        }
        if (is(argv[1], "clear_all")) {
            clearCalibrationRuntime(ctx);
            tracker_serial_detail::printOk(out, "all calibration cleared in RAM");
            return;
        }


        tracker_serial_detail::printErr(out, "unknown cal command");
    }

private:
    static void cmdCalStatus(TrackerSerialCommandContext& ctx) {
        Stream& out = stream(ctx);
        out.println("# CALIBRATION STATUS");
        if (ctx.config) {
            out.print("gyro_bias_valid=");
            out.println(ctx.config->data.gyroCal.biasValid ? "yes" : "no");
            out.print("gyro_temp_valid=");
            out.println(ctx.config->data.gyroCal.tempCompValid ? "yes" : "no");
            out.print("gyro_temp_enabled=");
            out.println(ctx.config->data.gyroCal.tempCompEnabled ? "yes" : "no");
            out.print("accel_cal_valid=");
            out.println(ctx.config->data.accelCal.valid ? "yes" : "no");
            out.print("sensor_to_device_valid=");
            out.println(ctx.config->data.frame.sensorToDeviceValid ? "yes" : "no");
            out.print("mag_cal_valid=");
            out.println(ctx.config->data.magCal.calibrationValid ? "yes" : "no");
            out.print("mag_to_imu_valid=");
            out.println(ctx.config->data.magCal.axisAlignmentValid ? "yes" : "no");
        } else {
            out.println("config_available=no");
        }
#if TRACKER_HAS_CALIBRATION_AUTONOMY
        if (ctx.calibrationAutonomy) {
            ctx.calibrationAutonomy->printStatus(out, millis());
        }
#endif
    }

    static void cmdCalEraseAll(TrackerSerialCommandContext& ctx,
                               int argc,
                               char** argv) {
        Stream& out = stream(ctx);
        if (argc != 3 || !is(argv[2], "confirm")) {
            tracker_serial_detail::printErr(out, "usage: cal erase_all confirm");
            return;
        }
        if (!ctx.config || !ctx.configStore) {
            tracker_serial_detail::printErr(out, "config/configStore not available");
            return;
        }

        TrackerConfig clean = *ctx.config;
        clean.clearAllCalibrationPreservingPolicy();
        clean.sanitize();
        clean.updateCrc();

        // Write a verified recovery marker before touching config slots. It
        // contains the calibrationless payload and current autonomy preferences,
        // so a power cut at any later step is completed idempotently at boot.
#if TRACKER_HAS_CALIBRATION_AUTONOMY
        if (ctx.calibrationAutonomy &&
            !ctx.calibrationAutonomy->preparePersistentCalibrationErase(clean)) {
            tracker_serial_detail::printErr(
                out, "failed to persist calibration erase recovery marker");
            return;
        }
#endif
        if (!ctx.configStore->erase()) {
            out.print("# ERR calibration storage erase failed: ");
            out.println(ctx.configStore->lastErrorName());
            return;
        }
        if (!ctx.configStore->save(clean, TrackerCalibrationProvenance::Manual)) {
            out.print("# ERR clean policy config save failed after erase: ");
            out.println(ctx.configStore->lastErrorName());
            return;
        }
        *ctx.config = clean;
        clearCalibrationRuntime(ctx);

#if TRACKER_HAS_CALIBRATION_AUTONOMY
        if (ctx.calibrationAutonomy &&
            !ctx.calibrationAutonomy->forceClearPersistentCalibrationStateForErase(millis())) {
            tracker_serial_detail::printErr(
                out,
                "calibration erased, but autonomy metadata cleanup failed; retry erase_all");
            return;
        }
#endif
        tracker_serial_detail::printOk(
            out,
            "all saved calibration, candidates, rollback journal and rejection memory erased");
    }

    static bool commandMutatesCalibration(int argc, char** argv) {
        if (argc < 2) return false;
        if (is(argv[1], "candidate")) {
            return !(argc >= 3 && (is(argv[2], "status") || is(argv[2], "compare")));
        }
        if (is(argv[1], "accel")) {
            return !(argc >= 3 && is(argv[2], "dump"));
        }
        if (is(argv[1], "temp")) {
            return !(argc < 3 || is(argv[2], "print"));
        }
        return is(argv[1], "gyro") || is(argv[1], "save") ||
            is(argv[1], "clear_all");
    }

    static void clearCalibrationRuntime(TrackerSerialCommandContext& ctx) {
        if (ctx.imuCal) {
            ctx.imuCal->gyroBiasValid = false;
            ctx.imuCal->gyroBiasRadS = Vec3::zero();
            ctx.imuCal->accelCalValid = false;
            ctx.imuCal->accelBiasG = Vec3::zero();
            ctx.imuCal->accelScale = Mat3::identity();
        }
        if (ctx.gyroTempComp) ctx.gyroTempComp->clearAll();
        if (ctx.runtimeBias) runtimeBiasReset(*ctx.runtimeBias);
        trackerSerialResetCalibrationWorkspaces(ctx);
        if (ctx.config) ctx.config->clearAllCalibrationPreservingPolicy();
        if (ctx.clearMagHeadingReference) {
            ctx.clearMagHeadingReference(ctx.clearMagHeadingReferenceUser);
        }
        if (ctx.resetMagYawCorrection) {
            ctx.resetMagYawCorrection(ctx.resetMagYawCorrectionUser);
        }
        if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
        if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();
    }

    static void cmdCalAutonomy(TrackerSerialCommandContext& ctx,
                               int argc,
                               char** argv) {
        Stream& out = stream(ctx);
#if TRACKER_HAS_CALIBRATION_AUTONOMY
        if (!ctx.calibrationAutonomy) {
            tracker_serial_detail::printErr(out, "calibration autonomy not available");
            return;
        }
        if (argc == 2 || is(argv[2], "status")) {
            ctx.calibrationAutonomy->printStatus(out, millis());
            return;
        }
        if (is(argv[2], "rollback")) {
            if (argc != 3) {
                tracker_serial_detail::printErr(out, "usage: cal autonomy rollback");
                return;
            }
            if (!ctx.calibrationAutonomy->requestRollback(millis())) {
                tracker_serial_detail::printErr(out, "no autonomy probation to roll back");
                return;
            }
            tracker_serial_detail::printOk(out, "calibration autonomy rollback complete");
            return;
        }
        if (is(argv[2], "reset")) {
            if (argc != 3) {
                tracker_serial_detail::printErr(out, "usage: cal autonomy reset");
                return;
            }
            ctx.calibrationAutonomy->resetRuntimeEvidence();
            tracker_serial_detail::printOk(out, "calibration autonomy runtime evidence reset");
            return;
        }
        if (is(argv[2], "clear_rejections")) {
            if (argc != 3) {
                tracker_serial_detail::printErr(out, "usage: cal autonomy clear_rejections");
                return;
            }
            if (!ctx.calibrationAutonomy->clearRejectionMemory()) {
                tracker_serial_detail::printErr(out, "failed to clear autonomy rejection memory");
                return;
            }
            tracker_serial_detail::printOk(out, "calibration autonomy rejection memory cleared");
            return;
        }
        if (!is(argv[2], "0022") && !is(argv[2], "0023")) {
            tracker_serial_detail::printErr(
                out,
                "usage: cal autonomy 0022|0023 on|off [save]");
            return;
        }
        if (argc < 4 || argc > 5 ||
            (!is(argv[3], "on") && !is(argv[3], "off")) ||
            (argc == 5 && !is(argv[4], "save"))) {
            tracker_serial_detail::printErr(
                out,
                "usage: cal autonomy 0022|0023 on|off [save]");
            return;
        }
        const bool enabled = is(argv[3], "on");
        const bool persist = argc == 5;
        const bool ok = is(argv[2], "0022")
            ? ctx.calibrationAutonomy->setWave0022Enabled(enabled, persist, millis())
            : ctx.calibrationAutonomy->setWave0023Enabled(enabled, persist, millis());
        if (!ok) {
            tracker_serial_detail::printErr(out, "calibration autonomy state change failed");
            return;
        }
        out.print("# OK calibration autonomy ");
        out.print(argv[2]);
        out.print(' ');
        out.print(enabled ? "enabled" : "disabled");
        out.println(persist ? " and saved" : " until reboot");
#else
        (void)ctx;
        (void)argc;
        (void)argv;
        tracker_serial_detail::printErr(out, "calibration autonomy not compiled");
#endif
    }

    static TrackerCalibrationProvenance parseCandidateProvenance(const char* value) {
        if (is(value, "manual")) return TrackerCalibrationProvenance::Manual;
        if (is(value, "setup")) return TrackerCalibrationProvenance::Setup;
        if (is(value, "background")) return TrackerCalibrationProvenance::Background;
        return TrackerCalibrationProvenance::Unknown;
    }

    static void printCandidateQuality(Stream& out,
                                      const TrackerCalibrationQualitySummary& quality) {
        out.print("quality_overall="); out.println(quality.overallScore, 6);
        out.print("quality_gyro="); out.println(quality.gyroScore, 6);
        out.print("quality_accel="); out.println(quality.accelScore, 6);
        out.print("quality_mag="); out.println(quality.magScore, 6);
        out.print("quality_alignment="); out.println(quality.alignmentScore, 6);
        out.print("quality_coverage="); out.println(quality.coverageScore, 6);
        out.print("gyro_residual_dps="); out.println(quality.gyroResidualDps, 6);
        out.print("accel_residual_g="); out.println(quality.accelResidualG, 6);
        out.print("mag_residual="); out.println(quality.magResidual, 6);
        out.print("quality_flags=0x"); out.println(quality.qualityFlags, HEX);
    }

    static void printCandidate(Stream& out,
                               const TrackerCalibrationCandidateRecord& candidate,
                               bool dirty) {
        out.println("# CALIBRATION CANDIDATE");
        out.print("candidate_format_version="); out.println(candidate.version);
        out.print("candidate_generation="); out.println(candidate.candidateGeneration);
        out.print("candidate_dirty="); out.println(dirty ? "yes" : "no");
        out.print("candidate_persisted_writes="); out.println(candidate.persistedWriteCount);
        out.print("candidate_provenance=");
        out.println(trackerCalibrationProvenanceName(candidate.metadata.provenance));
        out.print("candidate_created_uptime_ms="); out.println(candidate.metadata.createdUptimeMs);
        if (candidate.version == tracker_config_storage_detail::LEGACY_CANDIDATE_VERSION) {
            out.print("candidate_active_generation_at_creation=");
        } else {
            out.print("candidate_active_calibration_revision=0x");
        }
        if (candidate.version == tracker_config_storage_detail::LEGACY_CANDIDATE_VERSION) {
            out.println(candidate.metadata.activeCalibrationRevisionAtCreation);
        } else {
            out.println(candidate.metadata.activeCalibrationRevisionAtCreation, HEX);
        }
        out.print("candidate_sample_count="); out.println(candidate.metadata.sampleCount);
        out.print("candidate_independent_windows=");
        out.println(candidate.metadata.independentWindowCount);
        out.print("candidate_comparison=");
        out.println(trackerCalibrationComparisonName(candidate.metadata.lastComparison));
        out.print("candidate_comparison_flags=0x");
        out.println(candidate.metadata.comparisonFlags, HEX);
        out.print("candidate_signature_crc=0x"); out.println(candidate.signature.crc32, HEX);
        printCandidateQuality(out, candidate.metadata.quality);
    }

    static bool accelRunnerMatchesConfig(
        const Accel6PosCalibration::Result& result,
        const TrackerConfig& config
    ) {
        if (!result.valid || !config.data.accelCal.valid) return false;
        if (result.biasG.x != config.data.accelCal.biasG.x ||
            result.biasG.y != config.data.accelCal.biasG.y ||
            result.biasG.z != config.data.accelCal.biasG.z) {
            return false;
        }
        for (uint8_t row = 0; row < 3; ++row) {
            for (uint8_t col = 0; col < 3; ++col) {
                if (result.scaleMatrix.m[row][col] !=
                    config.data.accelCal.scale.m[row][col]) {
                    return false;
                }
            }
        }
        return true;
    }

    template <typename T>
    static std::unique_ptr<T> candidateScratch(Stream& out) {
        std::unique_ptr<T> value(new (std::nothrow) T{});
        if (!value) tracker_serial_detail::printErr(out, "out of memory for candidate command");
        return value;
    }

    static void applyCalibrationConfig(TrackerSerialCommandContext& ctx,
                                       const TrackerConfig& config) {
        if (!ctx.config) return;

        // Promotion owns learned calibration state only. Preserve current RAM
        // product policy (output, AHRS tuning, FIFO/SPI and temp-comp enable)
        // even when it has not been persisted yet.
        trackerApplyCalibrationCandidateToConfig(*ctx.config, config);
        if (ctx.imuCal) ctx.config->applyToImuCalibration(*ctx.imuCal);
        if (ctx.gyroTempComp) ctx.config->applyToGyroTempComp(*ctx.gyroTempComp);
        if (ctx.runtimeBias) runtimeBiasReset(*ctx.runtimeBias);
        trackerSerialResetCalibrationWorkspaces(ctx);

        // A changed bias/scale/alignment invalidates state learned against the
        // previous model, but does not require an IMU/FIFO reconfiguration.
        if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
        if (ctx.clearMagHeadingReference) {
            ctx.clearMagHeadingReference(ctx.clearMagHeadingReferenceUser);
        }
        if (ctx.resetMagYawCorrection) {
            ctx.resetMagYawCorrection(ctx.resetMagYawCorrectionUser);
        }
    }

    static void cmdCalCandidate(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.config || !ctx.configStore) {
            tracker_serial_detail::printErr(out, "config/configStore not available");
            return;
        }
        if (argc < 3 || is(argv[2], "status")) {
            auto candidate = candidateScratch<TrackerCalibrationCandidateRecord>(out);
            if (!candidate) return;
            if (!ctx.configStore->loadCandidate(*candidate)) {
                if (ctx.configStore->lastError() == TrackerConfigError::NotFound) {
                    out.println("# CALIBRATION CANDIDATE");
                    out.println("candidate_exists=no");
                    return;
                }
                out.print("# ERR candidate load failed: ");
                out.println(ctx.configStore->lastErrorName());
                return;
            }
            out.println("candidate_exists=yes");
            printCandidate(out, *candidate, ctx.configStore->candidateDirty());
            return;
        }

        if (is(argv[2], "stage")) {
            TrackerCalibrationProvenance provenance = TrackerCalibrationProvenance::Manual;
            bool flush = false;
            bool forceFlush = false;
            for (int i = 3; i < argc; ++i) {
                const TrackerCalibrationProvenance parsed = parseCandidateProvenance(argv[i]);
                if (parsed != TrackerCalibrationProvenance::Unknown) {
                    provenance = parsed;
                } else if (is(argv[i], "flush")) {
                    flush = true;
                } else if (is(argv[i], "force")) {
                    flush = true;
                    forceFlush = true;
                } else {
                    tracker_serial_detail::printErr(out, "usage: cal candidate stage [manual|setup|background] [flush|force]");
                    return;
                }
            }

            auto staged = candidateScratch<TrackerConfig>(out);
            if (!staged) return;
            *staged = *ctx.config;
            trackerSerialCaptureRuntimeToConfig(ctx, *staged);
            TrackerCalibrationCandidateMetadata metadata;
            metadata.provenance = provenance;
            metadata.quality = trackerCalibrationQualityFromConfig(*staged);
            if (ctx.accelCalRunner && accelRunnerMatchesConfig(
                    ctx.accelCalRunner->calibration().result(), *staged)) {
                metadata.sampleCount = 0;
                for (uint8_t i = 0;
                     i < static_cast<uint8_t>(Accel6PosCalibration::Face::Count);
                     ++i) {
                    metadata.sampleCount += ctx.accelCalRunner->calibration().faceData(
                        static_cast<Accel6PosCalibration::Face>(i)
                    ).samples;
                }
                metadata.independentWindowCount = 6;
            }
            if (!ctx.configStore->stageCandidate(*staged, metadata, millis())) {
                out.print("# ERR candidate stage failed: ");
                out.println(ctx.configStore->lastErrorName());
                return;
            }
            if (flush && !ctx.configStore->flushCandidate(millis(), forceFlush)) {
                out.print("# ERR candidate flush failed: ");
                out.println(ctx.configStore->lastErrorName());
                return;
            }
            tracker_serial_detail::printOk(out, flush ? "calibration candidate staged and flushed" : "calibration candidate staged in RAM");
            return;
        }

        if (is(argv[2], "flush")) {
            const bool force = argc >= 4 && is(argv[3], "force");
            if (argc >= 4 && !force) {
                tracker_serial_detail::printErr(out, "usage: cal candidate flush [force]");
                return;
            }
            if (!ctx.configStore->flushCandidate(millis(), force)) {
                out.print("# ERR candidate flush failed: ");
                out.println(ctx.configStore->lastErrorName());
                return;
            }
            tracker_serial_detail::printOk(out, "calibration candidate flushed to NVS");
            return;
        }

        if (is(argv[2], "compare")) {
            auto candidate = candidateScratch<TrackerCalibrationCandidateRecord>(out);
            if (!candidate) return;
            TrackerCalibrationComparisonResult result;
            uint32_t flags = 0;
            if (!ctx.configStore->compareCandidate(*candidate, result, flags)) {
                out.print("# ERR candidate compare failed: ");
                out.println(ctx.configStore->lastErrorName());
                return;
            }
            printCandidate(out, *candidate, ctx.configStore->candidateDirty());
            return;
        }

        if (is(argv[2], "discard")) {
            if (!ctx.configStore->discardCandidate()) {
                out.print("# ERR candidate discard failed: ");
                out.println(ctx.configStore->lastErrorName());
                return;
            }
            tracker_serial_detail::printOk(out, "calibration candidate discarded");
            return;
        }

        if (is(argv[2], "promote")) {
            const bool force = argc >= 4 && is(argv[3], "force");
            if (argc >= 4 && !force) {
                tracker_serial_detail::printErr(out, "usage: cal candidate promote [force]");
                return;
            }

            auto prepared = candidateScratch<TrackerPreparedConfigPromotion>(out);
            auto candidate = candidateScratch<TrackerConfig>(out);
            auto previous = candidateScratch<TrackerConfig>(out);
            auto promoted = candidateScratch<TrackerConfig>(out);
            if (!prepared || !candidate || !previous || !promoted) return;

            *previous = *ctx.config;
            trackerSerialCaptureRuntimeToConfig(ctx, *previous);
            if (!ctx.configStore->prepareCandidatePromotion(
                    *prepared, *candidate, force, previous.get())) {
                out.print("# ERR candidate promotion prepare failed: ");
                out.println(ctx.configStore->lastErrorName());
                return;
            }

            // Signature equality guarantees identical IMU/FIFO modes. Apply
            // only calibration-owned runtime state; no sensor/FIFO restart.
            applyCalibrationConfig(ctx, *candidate);

            if (!ctx.configStore->commitPreparedPromotion(*prepared, *promoted)) {
                const TrackerConfigError commitError = ctx.configStore->lastError();
                if (commitError == TrackerConfigError::CommitUncertain) {
                    auto resolved = candidateScratch<TrackerConfig>(out);
                    if (resolved && ctx.configStore->load(*resolved)) {
                        const bool candidateCommitted =
                            std::memcmp(&resolved->data, &candidate->data, sizeof(resolved->data)) == 0;
                        applyCalibrationConfig(ctx, *resolved);
                        ctx.configStore->confirmAuthoritativeConfigApplied();
                        if (candidateCommitted) {
                            prepared->valid = false;
                            if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();
                            out.println("# WARN selector verification was uncertain, but active generation reconciled to candidate");
                            tracker_serial_detail::printOk(out, "calibration candidate promoted after reconciliation");
                            return;
                        }
                        ctx.configStore->abortPreparedPromotion(*prepared);
                        out.println("# WARN selector verification was uncertain; reconciled to previous active config");
                        return;
                    }
                    out.println("# ERR selector commit state is uncertain; runtime kept on candidate, reboot required before further config writes");
                    return;
                }
                applyCalibrationConfig(ctx, *previous);
                ctx.configStore->abortPreparedPromotion(*prepared);
                out.print("# ERR candidate selector commit failed; runtime rolled back: ");
                out.println(TrackerConfigStore::errorName(commitError));
                return;
            }

            applyCalibrationConfig(ctx, *promoted);
            if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();
            tracker_serial_detail::printOk(out, "calibration candidate promoted atomically");
            return;
        }

        tracker_serial_detail::printErr(out, "usage: cal candidate status|stage|flush|compare|discard|promote");
    }

    static void cmdCalGyro(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (argc >= 3 && is(argv[2], "save")) {
            if (!ctx.config || !ctx.configStore || !ctx.imuCal) {
                tracker_serial_detail::printErr(out, "config or calibration not available");
                return;
            }
            ctx.config->captureGyroFromImuCalibration(*ctx.imuCal);
            if (ctx.gyroTempComp) ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
            if (ctx.configStore->save(*ctx.config, TrackerCalibrationProvenance::Manual)) tracker_serial_detail::printOk(out, "gyro calibration saved");
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
            if (ctx.gyroTempComp) ctx.gyroTempComp->clearAll();
            if (ctx.runtimeBias) runtimeBiasReset(*ctx.runtimeBias);
            trackerSerialResetCalibrationWorkspaces(ctx, TRACKER_CAL_WORKSPACE_GYRO_TEMP);
            if (ctx.config) ctx.config->clearGyroCalibrationPreservingPolicy();
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
            if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();
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

        if (ctx.runtimeBias) runtimeBiasReset(*ctx.runtimeBias);
        trackerSerialResetCalibrationWorkspaces(ctx, TRACKER_CAL_WORKSPACE_GYRO_TEMP);
        if (ctx.config) {
            ctx.config->captureGyroFromImuCalibration(*ctx.imuCal);
            ctx.config->noteGyroBiasCalibrationCaptured(millis());
            if (ctx.gyroTempComp) ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
        }
        if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
        if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();

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
                ctx.config->captureAccelFromImuCalibration(*ctx.imuCal);
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
            ctx.config->captureAccelFromImuCalibration(*ctx.imuCal);
            if (ctx.configStore->save(*ctx.config, TrackerCalibrationProvenance::Manual)) tracker_serial_detail::printOk(out, "accel calibration saved");
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
            if (ctx.config) ctx.config->clearAccelCalibration();
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
            if (ctx.clearMagHeadingReference) {
                ctx.clearMagHeadingReference(ctx.clearMagHeadingReferenceUser);
            }
            if (ctx.resetMagYawCorrection) {
                ctx.resetMagYawCorrection(ctx.resetMagYawCorrectionUser);
            }
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

        enum class TempCaptureKind : uint8_t { Snapshot, ModelUpdate };
        auto commitTempCandidate = [&](const GyroTempCompensator& candidateTempComp,
                                       bool saveRequested,
                                       bool resetRuntimeTrim,
                                       TempCaptureKind captureKind) -> bool {
            if (!ctx.config) return false;

            const bool modelChanged = !gyroTempRuntimeModelEqual(*ctx.gyroTempComp, candidateTempComp);
            const bool oldBiasValid = ctx.gyroTempComp->valid();
            const bool newBiasValid = candidateTempComp.valid();

            TrackerConfig candidateConfig = *ctx.config;
            if (captureKind == TempCaptureKind::ModelUpdate) {
                candidateConfig.captureFromGyroTempCompUpdate(candidateTempComp, millis());
            } else {
                candidateConfig.captureFromGyroTempComp(candidateTempComp);
            }
            candidateConfig.sanitize();
            candidateConfig.updateCrc();

            if (saveRequested) {
                if (!ctx.configStore || !ctx.configStore->save(candidateConfig, TrackerCalibrationProvenance::Manual)) return false;
            }

            *ctx.gyroTempComp = candidateTempComp;
            *ctx.config = candidateConfig;
            if (ctx.runtimeBias && modelChanged && resetRuntimeTrim) runtimeBiasReset(*ctx.runtimeBias);
            if (modelChanged) {
                trackerSerialResetCalibrationWorkspaces(ctx, TRACKER_CAL_WORKSPACE_GYRO_TEMP);
                if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
                if (oldBiasValid != newBiasValid && ctx.slimevrRuntime) {
                    ctx.slimevrRuntime->requestSensorInfoRefresh();
                }
            }
            return true;
        };

        const float currentTempC =
            ctx.calibrationIo ? ctx.calibrationIo->latestTempC : 25.0f;

        if (argc < 3 || is(argv[2], "print")) {
            const auto s = ctx.gyroTempComp->snapshot(currentTempC);

            out.println("# GYRO TEMP COMP");
            out.print("gyro_bias_model_valid="); out.println(s.valid ? "yes" : "no");
            out.print("temp_comp_valid="); out.println(s.temperatureModelValid ? "yes" : "no");
            out.print("temp_comp_enabled="); out.println(s.enabled ? "yes" : "no");

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

            return;
        }

        if (is(argv[2], "enable")) {
            const bool saveRequested = argc >= 4 && is(argv[3], "save");

            GyroTempCompensator candidateTempComp = *ctx.gyroTempComp;
            candidateTempComp.setEnabled(true);

            if (!commitTempCandidate(candidateTempComp, saveRequested, true, TempCaptureKind::Snapshot)) {
                tracker_serial_detail::printErr(out, "cal temp enable save failed");
                return;
            }

            tracker_serial_detail::printOk(out, saveRequested ? "gyro temp compensation enabled and saved" : "gyro temp compensation enabled");
            return;
        }

        if (is(argv[2], "disable")) {
            const bool saveRequested = argc >= 4 && is(argv[3], "save");

            GyroTempCompensator candidateTempComp = *ctx.gyroTempComp;
            candidateTempComp.setEnabled(false);

            if (!commitTempCandidate(candidateTempComp, saveRequested, true, TempCaptureKind::Snapshot)) {
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
            const Vec3 requestedSlopeDpsPerC(x, y, z);
            if (!ctx.gyroTempComp->acceptsSlopeDpsPerC(requestedSlopeDpsPerC)) {
                out.print("# ERR slope exceeds max_accepted_slope_dps_per_c=");
                out.println(ctx.gyroTempComp->config().maxAcceptedSlopeDpsPerC, 6);
                return;
            }

            if (!ctx.gyroTempComp->valid()) {
                tracker_serial_detail::printErr(out, "temperature slope requires a valid gyro bias/reference model");
                return;
            }

            GyroTempCompensator candidateTempComp = *ctx.gyroTempComp;
            candidateTempComp.setSlopeDpsPerC(requestedSlopeDpsPerC);
            if (!candidateTempComp.temperatureModelValid()) {
                tracker_serial_detail::printErr(out, "temperature slope was rejected by the runtime model");
                return;
            }

            if (!commitTempCandidate(candidateTempComp, saveRequested, true, TempCaptureKind::ModelUpdate)) {
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

            GyroTempCompensator candidateTempComp = *ctx.gyroTempComp;
            candidateTempComp.invalidateTemperatureModel();
            if (!commitTempCandidate(candidateTempComp, saveRequested, true, TempCaptureKind::ModelUpdate)) {
                tracker_serial_detail::printErr(out, "cal temp clear save failed");
                return;
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
        pp->out->flush();
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
        tracker_serial_detail::printVec3Line(out, "gyro_std_dps", r.gyroStdDps, 6);
        tracker_serial_detail::printVec3Line(out, "gyro_mean_std_error_dps", r.gyroMeanStdErrorDps, 6);
        tracker_serial_detail::printVec3Line(out, "accel_std_g", r.accelStdG, 6);
        tracker_serial_detail::printVec3Line(out, "validation_residual_dps", r.validationResidualDps, 6);
        tracker_serial_detail::printVec3Line(out, "validation_gyro_std_dps", r.validationGyroStdDps, 6);
        tracker_serial_detail::printVec3Line(out, "validation_gyro_mean_std_error_dps", r.validationGyroMeanStdErrorDps, 6);
        tracker_serial_detail::printVec3Line(out, "validation_accel_std_g", r.validationAccelStdG, 6);
        out.print("validation_accel_norm_mean_g="); out.println(r.validationAccelNormMeanG, 6);
        out.print("validation_accel_mean_delta_g="); out.println(r.validationAccelMeanDeltaG, 6);
        out.print("validation_samples="); out.println(r.validationSamples);
        out.print("temperature_span_c="); out.println(r.temperatureSpanC, 4);
        out.print("train_noise_gate="); out.println(r.trainNoiseGatePassed ? "pass" : "fail");
        out.print("train_mean_precision_gate="); out.println(r.trainMeanPrecisionGatePassed ? "pass" : "fail");
        out.print("validation_gate="); out.println(r.validationGatePassed ? "pass" : "fail");
        out.print("temperature_gate="); out.println(r.temperatureGatePassed ? "pass" : "fail");
        out.print("reference_temp_c="); out.println(tempC, 3);
        out.flush();
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
        pp->out->flush();
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
        out.print("max_pair_center_residual_g="); out.println(r.maxPairCenterResidualG, 8);
        out.print("accel_raw_basis_det="); out.println(r.matrixDeterminant, 8);
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
            accel_cal_quality_flags::PAIR_CENTER_RESIDUAL_HIGH,
            accel_cal_quality_flags::MATRIX_SINGULAR,
            accel_cal_quality_flags::INDEPENDENT_VALIDATION_FAILED,
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
