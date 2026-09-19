#pragma once

#include "serial/tracker_setup_commands.hpp"
#include "serial/tracker_config_commands.hpp"
#include "serial/tracker_calibration_commands.hpp"
#include "serial/tracker_serial_print.hpp"
#include "sensor/calibration.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/imu_quality.hpp"
#include "runtime/runtime_bias_types.hpp"
#include "runtime/runtime_gyro_bias_controller.hpp"
#include "runtime/slimevr_output_runtime.hpp"

namespace tracker {

// Synchronous manual/setup owner. Autonomy retains its reboot journal and uses
// the same config-store prepare/commit/rollback primitives. Its sensor probation
// remains asynchronous and journaled across reboot.
struct TrackerCalibrationTransaction {
    static Stream& transactionOutput(TrackerSerialCommandContext& ctx) { return ctx.io ? *ctx.io : Serial; }
    TrackerConfig configSnapshot;
    TrackerConfig candidateSnapshot;
    ImuCalibration imuSnapshot;
    bool haveConfig = false;
    bool haveImu = false;
    RuntimeGyroBiasEstimator runtimeBiasSnapshot;
    bool haveRuntimeBias = false;
    GyroTempCompensator tempSnapshot;
    bool haveTemperature = false;
    bool originalMagDriverEnabled = false;
    bool originalMagYawApplyEnabled = false;
    bool committed = false;
    bool persistentResultUncertain = false;
    bool rollbackNeeded = true;
    uint8_t rollbackWorkspaceMask = TRACKER_CAL_WORKSPACE_ALL;

    TrackerCalibrationTransaction() = default;

    void begin(TrackerSerialCommandContext& ctx) {
        transactionOutput(ctx).println("# calibration_transaction=volatile_preview_until_commit");
        configSnapshot = TrackerConfig{};
        imuSnapshot = ImuCalibration{};
        runtimeBiasSnapshot = RuntimeGyroBiasEstimator{};
        haveConfig = false;
        haveImu = false;
        haveRuntimeBias = false;
        haveTemperature = ctx.gyroTempComp != nullptr;
        if (haveTemperature) tempSnapshot = *ctx.gyroTempComp;
        originalMagDriverEnabled = false;
        originalMagYawApplyEnabled = false;
        committed = false;
        persistentResultUncertain = false;
        rollbackWorkspaceMask = TRACKER_CAL_WORKSPACE_ALL;
        rollbackNeeded = true;

        if (ctx.config) {
            configSnapshot = *ctx.config;
            haveConfig = true;
            originalMagDriverEnabled = ctx.config->data.magCal.driverEnabled;
            originalMagYawApplyEnabled = ctx.config->data.magYaw.applyEnabled;
        }
        if (ctx.imuCal) {
            imuSnapshot = *ctx.imuCal;
            haveImu = true;
        }
        if (ctx.runtimeBias) {
            runtimeBiasSnapshot = *ctx.runtimeBias;
            haveRuntimeBias = true;
        }
    }

    void rollback(TrackerSerialCommandContext& ctx, const char* reason) {
        Stream& s = transactionOutput(ctx);
        if (committed || !rollbackNeeded) return;

        if ((rollbackWorkspaceMask & TRACKER_CAL_WORKSPACE_MAG) != 0u && ctx.stopMagCalibration) {
            ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
        }
        trackerSerialResetCalibrationWorkspaces(ctx, rollbackWorkspaceMask);

        if (haveConfig && ctx.config) {
            *ctx.config = configSnapshot;
        }
        if (haveImu && ctx.imuCal) {
            *ctx.imuCal = imuSnapshot;
        } else if (ctx.config && ctx.imuCal) {
            ctx.config->applyToImuCalibration(*ctx.imuCal);
        }
        if (haveTemperature && ctx.gyroTempComp) *ctx.gyroTempComp = tempSnapshot;

        if (haveRuntimeBias && ctx.runtimeBias) {
            *ctx.runtimeBias = runtimeBiasSnapshot;
        } else if (ctx.resetRuntimeGyroBiasEstimator) {
            ctx.resetRuntimeGyroBiasEstimator(ctx.resetRuntimeGyroBiasEstimatorUser);
        }
        if (ctx.ahrs && ctx.config) ctx.ahrs->setConfig(ctx.config->makeAhrsConfig());
        if (ctx.quality && ctx.config) {
            ctx.quality->setConfig(ctx.config->makeQualityConfig());
            ctx.quality->reset();
            if (ctx.fifo) ctx.quality->syncFifoStats(ctx.fifo->stats());
        }
        if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
        if (ctx.resetMagYawCorrection) ctx.resetMagYawCorrection(ctx.resetMagYawCorrectionUser);

        if (ctx.setMagRuntimeEnabled) {
            (void)ctx.setMagRuntimeEnabled(originalMagDriverEnabled, false, ctx.setMagRuntimeEnabledUser);
        }
        if (ctx.setMagYawCorrectionApplyEnabled) {
            (void)ctx.setMagYawCorrectionApplyEnabled(originalMagYawApplyEnabled, false, ctx.setMagYawCorrectionApplyEnabledUser);
        }
        if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();

        s.print("# CALIBRATION TRANSACTION ROLLBACK");
        if (reason && reason[0]) {
            s.print(" reason=");
            s.print(reason);
        }
        s.println();
        s.println("# Current setup transaction restored to its pre-stage snapshot; earlier committed checkpoints, if any, remain authoritative.");
    }

    bool commit(TrackerSerialCommandContext& ctx, uint8_t workspaceMask = TRACKER_CAL_WORKSPACE_ALL,
                TrackerCalibrationProvenance provenance = TrackerCalibrationProvenance::Setup,
                const TrackerConfig* staged = nullptr,
                TrackerPreparedConfigCommit* existingPrepared = nullptr) {
        Stream& s = transactionOutput(ctx);
        if (!ctx.config || !ctx.configStore) {
            tracker_serial_detail::printErr(s, "setup calibration commit failed: config store not available");
            return false;
        }
        rollbackWorkspaceMask = workspaceMask;
        if (staged) rollbackNeeded = false;
        TrackerConfig& candidate = candidateSnapshot;
        candidate = staged ? *staged : *ctx.config;
        if ((!staged && !trackerSerialCaptureRuntimeToConfig(ctx, candidate)) ||
            !candidate.validateSemanticConfig()) {
            tracker_serial_detail::printErr(s, "setup calibration produced an invalid config candidate");
            return false;
        }
        candidate.updateCrc();
        TrackerPreparedConfigCommit localPrepared;
        TrackerPreparedConfigCommit& prepared = existingPrepared ? *existingPrepared : localPrepared;
        if (!existingPrepared && !ctx.configStore->prepareAuthoritativeCommit(candidate, prepared, provenance)) return false;
        if (!prepared.valid) return false;
        const bool previousBarrier = ctx.configStore->autonomyProbationWriteBarrier();
        ctx.configStore->setAutonomyProbationWriteBarrier(true);
        s.println("# calibration_transaction=persisted_candidate_probation");
        const bool modelChanged = !trackerCalibrationModelEqual(configSnapshot, candidate);
        const bool tempPolicyChanged = configSnapshot.data.gyroCal.tempCompEnabled != candidate.data.gyroCal.tempCompEnabled;
        rollbackNeeded = true;
        *ctx.config = candidate;
        if (staged && ctx.gyroTempComp) candidate.applyToGyroTempComp(*ctx.gyroTempComp);
        if (modelChanged || tempPolicyChanged) {
            if (ctx.imuCal) candidate.applyToImuCalibration(*ctx.imuCal);
            if (!staged && ctx.gyroTempComp) candidate.applyToGyroTempComp(*ctx.gyroTempComp);
            if (ctx.runtimeBias) runtimeBiasReset(*ctx.runtimeBias);
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
            if (ctx.clearMagHeadingReference) ctx.clearMagHeadingReference(ctx.clearMagHeadingReferenceUser);
            if (ctx.resetMagYawCorrection) ctx.resetMagYawCorrection(ctx.resetMagYawCorrectionUser);
        }
        // candidate is frozen after inactive-slot write/read-back. The live
        // model is an explicitly named preview until this sensor verdict.
        if (!trackerSerialVerifyCalibrationCandidate(ctx, candidate,
                provenance == TrackerCalibrationProvenance::Setup)) {
            s.println("# ERR calibration sensor probation rejected; candidate not committed");
            (void)ctx.configStore->abortPreparedAuthoritative(prepared);
            ctx.configStore->setAutonomyProbationWriteBarrier(previousBarrier);
            return false;
        }
        const bool commitOk = existingPrepared
            ? ctx.configStore->commitPreparedPromotion(prepared, candidate)
            : ctx.configStore->commitPreparedAuthoritative(prepared, candidate);
        if (!commitOk) {
            persistentResultUncertain = ctx.configStore->lastError() == TrackerConfigError::CommitUncertain;
            if (persistentResultUncertain) {
                // Keep the proven working preview. Neither RAM nor authority
                // may be guessed when selector read-back is uncertain.
                committed = true;
                // The store's CommitUncertain latch now owns write inhibition.
                // Do not leave a foreign probation barrier after verified load.
                ctx.configStore->setAutonomyProbationWriteBarrier(previousBarrier);
                s.println("# ERR calibration commit uncertain; proven preview retained, config load or reboot required");
            } else {
                (void)ctx.configStore->abortPreparedAuthoritative(prepared);
                ctx.configStore->setAutonomyProbationWriteBarrier(previousBarrier);
            }
            return false;
        }
        ctx.configStore->setAutonomyProbationWriteBarrier(previousBarrier);
        *ctx.config = candidate;
        committed = true;
        ctx.configStore->confirmAuthoritativeConfigApplied();
        trackerSerialResetCalibrationWorkspaces(ctx, workspaceMask);
        tracker_serial_detail::printOk(s, "calibration transaction committed to NVS");
        if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();
        return true;
    }
};

} // namespace tracker
