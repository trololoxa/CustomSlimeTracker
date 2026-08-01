#include "runtime/mag_runtime_controller.hpp"

#include "sensor/imu_quality.hpp"

#include <algorithm>
#include <cmath>

#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "connection/lsm6dsv_sensorhub.hpp"
#include "runtime/output_runtime.hpp"
#include "runtime/mag_status_reporter.hpp"
#include "runtime/tracker_console_suppress.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/imu_quality.hpp"
#include "sensor/frame_transform.hpp"
#include "sensor/mag_calibration.hpp"
#include "sensor/qmc6309.hpp"

namespace tracker {

#if defined(__GNUC__) || defined(__clang__)
#define TRACKER_MAG_RUNTIME_NOINLINE __attribute__((noinline))
#else
#define TRACKER_MAG_RUNTIME_NOINLINE
#endif

namespace {

Vec3 inverseApplyAcceptedSensorToDevice(const Mat3& rotation,
                                        const Vec3& deviceVector) {
    return Vec3(
        rotation.m[0][0] * deviceVector.x +
            rotation.m[1][0] * deviceVector.y +
            rotation.m[2][0] * deviceVector.z,
        rotation.m[0][1] * deviceVector.x +
            rotation.m[1][1] * deviceVector.y +
            rotation.m[2][1] * deviceVector.z,
        rotation.m[0][2] * deviceVector.x +
            rotation.m[1][2] * deviceVector.y +
            rotation.m[2][2] * deviceVector.z);
}

} // namespace

void MagRuntimeController::begin(const MagRuntimeControllerDeps& deps) {
    deps_ = deps;
    runtimeConfigCache_ = MagRuntimeConfig{};
    runtimeConfigCacheRevision_ = 0u;
    runtimeConfigCacheValid_ = false;
    clearAxisAlignmentEvidence();
}

const MagRuntimeConfig& MagRuntimeController::runtimeConfig() const {
    if (!deps_.config) {
        runtimeConfigCache_ = MagRuntimeConfig{};
        runtimeConfigCacheRevision_ = 0u;
        runtimeConfigCacheValid_ = true;
        return runtimeConfigCache_;
    }

    const uint32_t revision = deps_.config->data.crc32;
    if (runtimeConfigCacheValid_ && runtimeConfigCacheRevision_ == revision) {
        return runtimeConfigCache_;
    }

    MagRuntimeConfig c;
    const auto& magCal = deps_.config->data.magCal;
    c.enabled = magCal.driverEnabled;
    c.calibrationValid = magCal.calibrationValid;
    c.axisAlignmentValid = magCal.axisAlignmentValid;

    c.hardIron = magCal.hardIron;
    c.softIron = magCal.softIron;
    c.magToImu = magCal.magToImu;
    c.sensorToDeviceValid = deps_.config->data.frame.sensorToDeviceValid;
    c.sensorToDevice = deps_.config->data.frame.sensorToDevice;
    if (deps_.sensorToDeviceFrameCache != nullptr) {
        c.sensorToDeviceFrame = deps_.sensorToDeviceFrameCache->resolve(
            revision,
            c.sensorToDeviceValid,
            c.sensorToDevice);
    } else {
        c.sensorToDeviceFrame = makeSensorToDeviceFrame(
            c.sensorToDeviceValid,
            c.sensorToDevice);
    }
    c.sensorToDevicePrevalidated = true;

    c.expectedFieldNorm = magCal.expectedFieldNorm;
    c.minTrustNorm = magCal.minTrustNorm;
    c.maxTrustNorm = magCal.maxTrustNorm;

    c.maxSampleAgeMs = 250;
    c.minUsableNorm = 1.0e-6f;
    runtimeConfigCache_ = c;
    runtimeConfigCacheRevision_ = revision;
    runtimeConfigCacheValid_ = true;
    return runtimeConfigCache_;
}

MagHeadingConfig MagRuntimeController::headingConfig() const {
    MagHeadingConfig c;
    c.requireTrustedMag = true;
    c.minHorizontalNorm = 1.0e-6f;
    return c;
}

MagYawCorrectionConfig MagRuntimeController::yawConfig() const {
    MagYawCorrectionConfig c;
    if (!deps_.config) return c;

    const auto& y = deps_.config->data.magYaw;
    const bool accelCalReady = accelReady();
    const bool recoverySafe = !isRecoveryActive();

    c.enabled = y.controllerEnabled &&
                deps_.config->data.magCal.driverEnabled &&
                deps_.config->data.magCal.calibrationValid &&
                accelCalReady &&
                recoverySafe;
    c.applyEnabled = y.applyEnabled && accelCalReady && recoverySafe;

    c.maxInnovationDeg = y.maxInnovationDeg;
    c.maxMagAgeMs = y.maxMagAgeMs;

    c.horizontalNormGood = y.horizontalNormGood;
    c.horizontalNormBad = y.horizontalNormBad;

    c.gyroNormGoodDps = y.gyroNormGoodDps;
    c.gyroNormBadDps = y.gyroNormBadDps;

    c.accelTrustGood = y.accelTrustGood;
    c.accelTrustBad = y.accelTrustBad;
    c.requireAccelTrusted = y.requireAccelTrusted;

    c.timeConstantS = y.timeConstantS;
    c.maxCorrectionRateDegS = y.maxCorrectionRateDegS;
    c.maxCorrectionStepDeg = y.maxCorrectionStepDeg;
    c.fallbackDtS = y.fallbackDtS;

    c.gyroMovingCooldownMs = y.gyroMovingCooldownMs;
    c.accelBadCooldownMs = y.accelBadCooldownMs;
    c.magDisturbanceCooldownMs = y.magDisturbanceCooldownMs;

    return c;
}

MagFieldReliabilityConfig MagRuntimeController::fieldReliabilityConfig() const {
    return MagFieldReliabilityConfig{};
}

float MagRuntimeController::headingErrorToReferenceRad(const MagHeadingSample& heading) const {
    if (!deps_.headingRef || !deps_.headingRef->valid || !heading.valid) {
        return 0.0f;
    }
    return wrapPi(heading.magneticNorthWorldYawRad - deps_.headingRef->worldYawRad);
}

float MagRuntimeController::headingErrorToReferenceDeg(const MagHeadingSample& heading) const {
    return headingErrorToReferenceRad(heading) * MATH_RAD_TO_DEG;
}

void MagRuntimeController::resetYawCorrectionRuntime() {
    if (deps_.yawCorrection) deps_.yawCorrection->reset();
    if (deps_.lastYawCorrection) *deps_.lastYawCorrection = MagYawCorrectionOutput{};
}

void MagRuntimeController::resetAxisAlignmentCandidate() {
    clearAxisAlignmentEvidence();
#if TRACKER_ENABLE_CALIBRATION_CANDIDATES
    if (deps_.axisAlignmentCollector) deps_.axisAlignmentCollector->reset();
    if (deps_.axisAlignmentState) deps_.axisAlignmentState->reset();
#endif
}

void MagRuntimeController::setAxisAlignmentLearningEnabled(bool enabled) {
    axisAlignmentLearningEnabled_ = enabled;
    if (!enabled) resetAxisAlignmentCandidate();
}

void MagRuntimeController::resetOrientationState(const char* reason, uint64_t timestampUs, bool rebaseAhrsTimebase) {
    if (deps_.headingEstimator) deps_.headingEstimator->reset();
    if (deps_.lastHeading) *deps_.lastHeading = MagHeadingSample{};
    if (deps_.fieldReliability) deps_.fieldReliability->reset();
    if (deps_.lastFieldReliability) *deps_.lastFieldReliability = MagFieldReliabilityOutput{};
    resetAxisAlignmentCandidate();

    if (deps_.headingRef) deps_.headingRef->clear();
    if (deps_.headingAutoRef) {
        deps_.headingAutoRef->done = false;
        deps_.headingAutoRef->resetCandidate();
    }

    resetYawCorrectionRuntime();

    if (rebaseAhrsTimebase && timestampUs != 0 && deps_.ahrs) {
        deps_.ahrs->rebaseTimestamp(timestampUs);
    }

    if (!trackerConsoleTrackingMessagesSuppressed(millis())) {
        Stream& out = stream();
        out.print("# TRACKING orientation-dependent state reset");
        if (reason && reason[0] != '\0') {
            out.print(" reason=");
            out.print(reason);
        }
        if (timestampUs != 0) {
            out.print(" t_us=");
            outputPrintU64Dec(out, timestampUs);
        }
        out.println();
    }

    if (deps_.callbacks.emitStateEvent) {
        deps_.callbacks.emitStateEvent("ORIENTATION_RESET",
                                       reason,
                                       timestampUs,
                                       0,
                                       outputConfidence(),
                                       deps_.callbacks.emitStateEventUser);
    }
}

bool MagRuntimeController::setEnabled(bool enabled, bool persist) {
    if (!deps_.config || !deps_.hub || !deps_.qmc || !deps_.fifo || !deps_.state) {
        return false;
    }
    if (persist && !deps_.configStore) {
        stream().println("# ERR mag persistence requested but config store is unavailable");
        return false;
    }

    const uint64_t fifoTs = deps_.fifo->stats().lastAssignedTimestampUs;
    const uint64_t keepTs = fifoTs != 0
        ? fifoTs
        : (deps_.fallbackTimestampUs ? *deps_.fallbackTimestampUs : 0);

    const TrackerConfig oldConfig = *deps_.config;
    const bool oldHardwareEnabled = deps_.state->runtimeEnabled && deps_.state->fifoArmed;

    TrackerConfig candidateConfig = oldConfig;
    candidateConfig.data.magCal.driverEnabled = enabled;
    candidateConfig.sanitize();
    candidateConfig.updateCrc();

    const bool hardwareAlreadyMatches = enabled
        ? (deps_.state->runtimeEnabled && deps_.state->fifoArmed)
        : (!deps_.state->runtimeEnabled && !deps_.state->fifoArmed);
    if (oldConfig.data.magCal.driverEnabled == enabled && hardwareAlreadyMatches) {
        if (persist && !deps_.configStore->save(candidateConfig)) {
            stream().print("# ERR mag no-op save failed: ");
            stream().println(deps_.configStore->lastErrorName());
            return false;
        }
        *deps_.config = candidateConfig;
        stream().print("# OK QMC6309 runtime already ");
        stream().println(enabled ? "enabled" : "disabled");
        return true;
    }

    // Reconfigure hardware while the active config still describes the old
    // state.  The helper takes the target explicitly, so a failed transition
    // cannot leave RAM claiming that the magnetometer was enabled.
    if (!applyHardwareEnabledState(enabled, keepTs)) {
        return false;
    }

    if (persist && !deps_.configStore->save(candidateConfig)) {
        stream().print("# ERR mag ");
        stream().print(enabled ? "enable" : "disable");
        stream().print(" save failed: ");
        stream().println(deps_.configStore->lastErrorName());

        const bool rollbackOk = applyHardwareEnabledState(oldHardwareEnabled, keepTs);
        if (!rollbackOk) {
            stream().println("# ERR mag hardware rollback failed after persistence error");
            deps_.state->lastInitOk = false;
        }
        *deps_.config = oldConfig;
        return false;
    }

    *deps_.config = candidateConfig;
    stream().print("# OK QMC6309 runtime ");
    stream().println(enabled ? "enabled" : "disabled");
    return true;
}


bool MagRuntimeController::startFromPreconfiguredFifo() {
    if (!deps_.config || !deps_.hub || !deps_.qmc || !deps_.fifo || !deps_.state) return false;
    if (!deps_.config->data.magCal.driverEnabled) return true;
    if (deps_.state->runtimeEnabled && deps_.state->fifoArmed) return true;

    const uint64_t fifoTs = deps_.fifo->stats().lastAssignedTimestampUs;
    const uint64_t keepTs = fifoTs != 0
        ? fifoTs
        : (deps_.fallbackTimestampUs ? *deps_.fallbackTimestampUs : 0);

    auto failSafeDisable = [&]() {
        deps_.hub->stopMaster();
        deps_.state->runtimeEnabled = false;
        deps_.state->fifoArmed = false;
        // The preconfigured FIFO expects mag words. If QMC startup fails, move
        // back to a coherent 6DoF configuration even though this may require a
        // one-time startup recovery on the failure path.
        (void)reconfigureFifoForMagEnabled(false, keepTs);
        deps_.state->lastInitOk = false;
        resetRuntimeCounters();
    };

    if (!initSensorHub()) {
        ++deps_.state->enableFailures;
        failSafeDisable();
        return false;
    }
    if (!deps_.qmc->configureNormal100Hz()) {
        stream().print("# ERR QMC6309 boot init100 failed qmcErr=");
        stream().print(deps_.qmc->lastErrorName());
        stream().print(" hubErr=");
        stream().println(deps_.hub->lastErrorName());
        ++deps_.state->enableFailures;
        failSafeDisable();
        return false;
    }
    if (!deps_.qmc->armHubFifoRead(Lsm6dsvSensorHub::ShubOdr::Hz60)) {
        stream().print("# ERR QMC6309 boot arm FIFO failed qmcErr=");
        stream().print(deps_.qmc->lastErrorName());
        stream().print(" hubErr=");
        stream().println(deps_.hub->lastErrorName());
        ++deps_.state->enableFailures;
        failSafeDisable();
        return false;
    }

    deps_.state->runtimeEnabled = true;
    deps_.state->fifoArmed = true;
    deps_.state->lastInitOk = true;
    deps_.state->lastEnableMs = millis();
    resetRuntimeCounters();
    stream().println("# OK QMC6309 boot stream started without FIFO reconfigure");
    return true;
}

bool MagRuntimeController::setHeadingReference(const char* reason, bool verbose) {
    if (!deps_.lastHeading || !deps_.lastProcessed || !deps_.headingRef || !deps_.headingAutoRef) {
        return false;
    }

    if (!deps_.lastHeading->valid) {
        if (verbose) stream().println("# ERR cannot set mag heading reference: last heading is invalid");
        return false;
    }

    if (!MagRuntimeProcessor::trustedForUse(*deps_.lastProcessed, runtimeConfig(), millis())) {
        if (verbose) stream().println("# ERR cannot set mag heading reference: last mag is not trusted");
        return false;
    }

    deps_.headingRef->valid = true;
    deps_.headingRef->worldYawRad = deps_.lastHeading->magneticNorthWorldYawRad;
    deps_.headingRef->setMs = millis();
    deps_.headingRef->magSeq = deps_.lastHeading->magSeq;
    deps_.headingRef->magTimestampUs = deps_.lastHeading->magTimestampUs;

    deps_.headingAutoRef->done = true;
    deps_.headingAutoRef->setCount++;
    deps_.headingAutoRef->lastSetMs = millis();
    deps_.headingAutoRef->resetCandidate();

    resetYawCorrectionRuntime();

    if (verbose && !trackerConsoleTrackingMessagesSuppressed(millis())) {
        stream().print("# OK mag heading ref");
        if (reason && reason[0] != '\0') {
            stream().print(" reason=");
            stream().print(reason);
        }
        stream().print(" world_yaw_deg=");
        stream().println(deps_.headingRef->worldYawDeg(), 6);
    }

    return true;
}

void MagRuntimeController::clearHeadingReference() {
    if (deps_.headingRef) deps_.headingRef->clear();
    if (deps_.headingAutoRef) {
        deps_.headingAutoRef->done = false;
        deps_.headingAutoRef->resetCandidate();
    }
    // An explicit clear is also the safe user action for accepting a genuinely
    // new magnetic environment. The monitor must reacquire norm/dip/heading as
    // one coherent reference before auto-reference or yaw correction can resume.
    if (deps_.fieldReliability) deps_.fieldReliability->restartAcquisition();
    if (deps_.lastFieldReliability) {
        *deps_.lastFieldReliability = MagFieldReliabilityOutput{};
    }
    resetYawCorrectionRuntime();
}

bool MagRuntimeController::setAutoReferenceEnabled(bool enabled) {
    if (!deps_.headingAutoRef) return false;

    deps_.headingAutoRef->enabled = enabled;

    if (!enabled) {
        deps_.headingAutoRef->resetCandidate();
    } else if (!deps_.headingRef || !deps_.headingRef->valid) {
        deps_.headingAutoRef->done = false;
        deps_.headingAutoRef->resetCandidate();
    }

    stream().print("# OK mag heading auto-ref ");
    stream().println(enabled ? "enabled" : "disabled");
    return true;
}

bool MagRuntimeController::setYawCorrectionApplyEnabled(bool enabled, bool persist) {
    if (!deps_.config) return false;
    if (persist && !deps_.configStore) {
        stream().println("# ERR mag yaw persistence requested but config store is unavailable");
        return false;
    }

    if (enabled && (!deps_.config->data.accelCal.valid ||
                    !deps_.config->data.magCal.calibrationValid ||
                    !deps_.config->data.magCal.axisAlignmentValid)) {
        stream().println("# ERR mag yaw correction requires valid accel, field and axis calibration");
        return false;
    }

    TrackerConfig candidateConfig = *deps_.config;
    candidateConfig.data.magYaw.applyEnabled = enabled;
    candidateConfig.sanitize();
    candidateConfig.updateCrc();
    if (candidateConfig.data.magYaw.applyEnabled != enabled) {
        stream().println("# ERR mag yaw correction request was rejected by config invariants");
        return false;
    }

    const bool changed = deps_.config->data.magYaw.applyEnabled != enabled;
    if (persist && !deps_.configStore->save(candidateConfig)) {
        stream().print("# ERR mag yaw correction save failed: ");
        stream().println(deps_.configStore->lastErrorName());
        return false;
    }

    *deps_.config = candidateConfig;
    if (changed) resetYawCorrectionRuntime();

    stream().print("# OK mag yaw correction apply=");
    stream().println(candidateConfig.data.magYaw.applyEnabled ? "enabled" : "disabled");
    return true;
}

bool MagRuntimeController::startCalibration() {
    if (!deps_.state || !deps_.calibrationCollector) return false;
    if (!deps_.state->runtimeEnabled || !deps_.state->fifoArmed) {
        return false;
    }
    deps_.calibrationCollector->start(millis());
    return true;
}

void MagRuntimeController::stopCalibration() {
    if (deps_.calibrationCollector) deps_.calibrationCollector->stop();
}

void MagRuntimeController::resetCalibration() {
    if (deps_.calibrationCollector) deps_.calibrationCollector->reset();
}

bool MagRuntimeController::applyCalibration(bool persist) {
    if (!deps_.config || !deps_.calibrationCollector) return false;

    MagCalibrationResult result;
    if (!deps_.calibrationCollector->compute(result)) {
        stream().println("# ERR mag calibration compute failed");
        stream().print("# mag_cal_failure_reason=");
        stream().println(deps_.calibrationCollector->lastFailureReasonName());
        const MagCalibrationFitSetDiagnostics fitSet =
            deps_.calibrationCollector->fitSetDiagnostics();
        stream().print("# mag_cal_samples="); stream().println(deps_.calibrationCollector->samples());
        stream().print("# mag_cal_stored_fit_samples="); stream().println(fitSet.samples);
        stream().print("# mag_cal_reservoir_replacements="); stream().println(deps_.calibrationCollector->reservoirReplacements());
        stream().print("# mag_cal_reservoir_skipped="); stream().println(deps_.calibrationCollector->reservoirSkipped());
        stream().print("# mag_cal_capture_span_xyz=");
        stream().print(deps_.calibrationCollector->spanX(), 3); stream().print(',');
        stream().print(deps_.calibrationCollector->spanY(), 3); stream().print(',');
        stream().println(deps_.calibrationCollector->spanZ(), 3);
        stream().print("# mag_cal_fit_span_xyz=");
        stream().print(fitSet.max.x - fitSet.min.x, 3); stream().print(',');
        stream().print(fitSet.max.y - fitSet.min.y, 3); stream().print(',');
        stream().println(fitSet.max.z - fitSet.min.z, 3);
        stream().print("# mag_cal_capture_norm_min_mean_max=");
        stream().print(deps_.calibrationCollector->normMin(), 3); stream().print(',');
        stream().print(deps_.calibrationCollector->normMean(), 3); stream().print(',');
        stream().println(deps_.calibrationCollector->normMax(), 3);
        stream().print("# mag_cal_fit_norm_min_mean_max=");
        stream().print(fitSet.normMin, 3); stream().print(',');
        stream().print(fitSet.normMean, 3); stream().print(',');
        stream().println(fitSet.normMax, 3);
        magStatusPrintCalibrationFitQuality(stream(), *deps_.calibrationCollector, "# mag_cal_");
        stream().println("# Continue only if a physical quality metric is below coverage/quality limits; otherwise rotate in place away from metal/magnets and retry.");
        return false;
    }

    if (persist && !deps_.configStore) {
        stream().println("# ERR mag calibration persistence requested but config store is unavailable");
        return false;
    }

    const TrackerMagCalibrationConfig oldMagCal = deps_.config->data.magCal;
    TrackerConfig candidateConfig = *deps_.config;
    candidateConfig.captureFromMagCalibrationResult(
        result,
        deps_.calibrationCollector->samples(),
        deps_.calibrationCollector->rejected(),
        deps_.calibrationCollector->saturated(),
        deps_.calibrationCollector->normMin(),
        deps_.calibrationCollector->normMean(),
        deps_.calibrationCollector->normMax(),
        millis()
    );

    bool fieldModelChanged =
        oldMagCal.calibrationValid != candidateConfig.data.magCal.calibrationValid ||
        oldMagCal.expectedFieldNorm != candidateConfig.data.magCal.expectedFieldNorm ||
        oldMagCal.minTrustNorm != candidateConfig.data.magCal.minTrustNorm ||
        oldMagCal.maxTrustNorm != candidateConfig.data.magCal.maxTrustNorm;
    fieldModelChanged = fieldModelChanged ||
        oldMagCal.hardIron.x != candidateConfig.data.magCal.hardIron.x ||
        oldMagCal.hardIron.y != candidateConfig.data.magCal.hardIron.y ||
        oldMagCal.hardIron.z != candidateConfig.data.magCal.hardIron.z;
    for (uint8_t row = 0; row < 3 && !fieldModelChanged; ++row) {
        for (uint8_t col = 0; col < 3; ++col) {
            if (oldMagCal.softIron.m[row][col] != candidateConfig.data.magCal.softIron.m[row][col]) {
                fieldModelChanged = true;
                break;
            }
        }
    }
    if (fieldModelChanged) {
        // Hard/soft-iron changes alter calibrated vector directions. A previous
        // mag-to-IMU solution is no longer proven compatible.
        candidateConfig.data.magCal.magToImu = Mat3::identity();
        candidateConfig.data.magCal.axisAlignmentValid = false;
        candidateConfig.data.magYaw.applyEnabled = false;
        candidateConfig.sanitize();
        candidateConfig.updateCrc();
    }

    if (persist && !deps_.configStore->save(candidateConfig, TrackerCalibrationProvenance::Manual)) {
        stream().print("# ERR mag calibration save failed: ");
        stream().println(deps_.configStore->lastErrorName());
        return false;
    }

    *deps_.config = candidateConfig;
    const uint64_t ts = deps_.fifo ? deps_.fifo->stats().lastAssignedTimestampUs : 0;
    resetOrientationState("mag_calibration_changed", ts, false);

    stream().print("# OK mag hardIron=");
    stream().print(result.hardIron.x, 6); stream().print(',');
    stream().print(result.hardIron.y, 6); stream().print(',');
    stream().println(result.hardIron.z, 6);

    stream().print("# OK mag softIron=");
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (r != 0 || c != 0) stream().print(',');
            stream().print(result.softIron.m[r][c], 6);
        }
    }
    stream().println();

    stream().print("# OK mag expectedNorm=");
    stream().println(result.expectedNorm, 6);
    stream().print("# OK mag residualRms=");
    stream().println(result.residualRms, 6);
    stream().print("# OK mag geometricResidualRms=");
    stream().println(result.geometricResidualRms, 6);
    stream().print("# OK mag normalizedResidualRms=");
    stream().println(result.normalizedResidualRms, 6);
    stream().print("# OK mag coverageScore=");
    stream().println(result.coverageScore, 6);
    stream().print("# OK mag directionalCoverageScore=");
    stream().println(result.directionalCoverageScore, 6);
    stream().print("# OK mag inlierRatio=");
    stream().println(result.inlierRatio, 6);

    return true;
}

TRACKER_MAG_RUNTIME_NOINLINE void MagRuntimeController::captureGyroEndpoint(
    MagProcessedSample& processed) const {
    if (!deps_.lastCalibratedSample || !deps_.lastImuTimestampUs || !deps_.config) {
        return;
    }

    const uint64_t gyroTimestampUs = *deps_.lastImuTimestampUs;
    const uint64_t skewUs = gyroTimestampUs > processed.t_us
        ? gyroTimestampUs - processed.t_us
        : processed.t_us - gyroTimestampUs;
    processed.gyroEndpointSkewUs = static_cast<uint32_t>(
        std::min<uint64_t>(skewUs, 0xFFFFFFFFULL));

    Vec3 gyroSensor = deps_.lastCalibratedSample->gyro_rad_s;
    // MagRuntimeProcessor already validated this exact config matrix before
    // applying it to the magnetic vector. Reuse that decision and only do
    // the inverse multiply here; repeating makeSensorToDeviceFrame() would
    // redo three norms, three dot products and a determinant every mag tick.
    if (processed.sensorToDeviceApplied) {
        gyroSensor = inverseApplyAcceptedSensorToDevice(
            deps_.config->data.frame.sensorToDevice, gyroSensor);
    }
    if (gyroTimestampUs != 0u && processed.t_us != 0u &&
        skewUs <= MAG_AXIS_MAX_GYRO_MAG_SKEW_US && gyroSensor.isFinite()) {
        processed.gyroEndpointValid = true;
        processed.gyroSensorRadS = gyroSensor;
        processed.gyroTimestampUs = gyroTimestampUs;
    }
}

TRACKER_MAG_RUNTIME_NOINLINE void MagRuntimeController::updateHeadingSnapshot(
    uint32_t nowMs) {
    deps_.headingEstimator->update(
        *deps_.lastProcessed,
        deps_.ahrs->quaternionPositiveW(),
        headingConfig(),
        nowMs,
        *deps_.lastHeading
    );
}

TRACKER_MAG_RUNTIME_NOINLINE void MagRuntimeController::updateFieldReliabilitySnapshot(
    uint32_t nowMs,
    float gyroNormDps,
    float accelTrust,
    bool processorTrustedForUse,
    MagFieldReliabilityOutput& reliability) {
    reliability = MagFieldReliabilityOutput{};
    if (deps_.fieldReliability) {
        MagFieldReliabilityInput fieldIn;
        fieldIn.mag = *deps_.lastProcessed;
        fieldIn.heading = *deps_.lastHeading;
        fieldIn.processorTrustedForUse = processorTrustedForUse;
        fieldIn.gyroNormDps = gyroNormDps;
        fieldIn.accelTrust = accelTrust;
        fieldIn.nowMs = nowMs;
        deps_.fieldReliability->update(fieldIn, fieldReliabilityConfig(), reliability);
    }
    if (deps_.lastFieldReliability && deps_.lastFieldReliability != &reliability) {
        *deps_.lastFieldReliability = reliability;
    }
}

TRACKER_MAG_RUNTIME_NOINLINE void MagRuntimeController::updateYawCorrectionSnapshot(
    uint32_t nowMs,
    float gyroNormDps,
    float accelTrust,
    bool magTrustedForUse,
    uint32_t magRejectFlagsForUse,
    const MagFieldReliabilityOutput& reliability) {
    MagYawCorrectionInput yawIn;
    yawIn.mag = *deps_.lastProcessed;
    yawIn.heading = *deps_.lastHeading;
    yawIn.referenceValid = deps_.headingRef && deps_.headingRef->valid;
    yawIn.referenceWorldYawRad = deps_.headingRef ? deps_.headingRef->worldYawRad : 0.0f;
    yawIn.magTrustedForUse = magTrustedForUse;
    yawIn.magRejectFlagsForUse = magRejectFlagsForUse;
    yawIn.gyroNormDps = gyroNormDps;
    yawIn.accelTrust = accelTrust;
    yawIn.fieldReliable = reliability.trustedForYaw;
    yawIn.fieldStableMs = reliability.stableMs;
    yawIn.magneticHeadingRateDegS = reliability.headingRateDegS;
    yawIn.nowMs = nowMs;

    MagYawCorrectionOutput& yawOut = *deps_.lastYawCorrection;
    deps_.yawCorrection->update(yawIn, yawConfig(), yawOut);

    if (applyYawCorrectionToAhrs(yawOut)) {
        yawOut.applied = true;
        deps_.yawCorrection->markApplied(yawOut.correctionStepDeg);
    }
}

TRACKER_MAG_RUNTIME_NOINLINE void MagRuntimeController::processRawSample(
    const Lsm6dsvFifoReader::MagRawSample& mag) {
    if (!deps_.state || !deps_.processor || !deps_.headingEstimator ||
        !deps_.lastProcessed || !deps_.lastHeading || !deps_.lastYawCorrection ||
        !deps_.ahrs || !deps_.yawCorrection) {
        return;
    }

    // One coherent wall-clock observation per magnetic sample avoids repeated
    // Arduino clock calls and prevents a sample from crossing millisecond
    // boundaries between processing, heading, reliability and yaw stages.
    const uint32_t nowMs = millis();

    deps_.state->samples++;
    deps_.state->queuePops++;
    deps_.state->lastSampleMs = nowMs;
    deps_.state->lastRaw = mag;
    deps_.state->lastNormRaw = magRawNorm(mag);

    if (deps_.calibrationCollector) {
        deps_.calibrationCollector->push(mag, deps_.state->lastNormRaw, nowMs);
    }

    // Build the relatively large runtime config once. 0023ge previously built
    // it twice in this 60 Hz callback and also kept all downstream workspaces
    // in one compiler-visible frame, which exceeded the MSYS2 stack ceiling.
    const MagRuntimeConfig& magCfg = runtimeConfig();
    MagProcessedSample& processed = *deps_.lastProcessed;
    deps_.processor->process(mag, magCfg, nowMs, processed);
    captureGyroEndpoint(processed);
    updateHeadingSnapshot(nowMs);

    const Ahrs6DofStats& ahrsStats = deps_.ahrs->stats();
    const float gyroNormDps = ahrsStats.lastGyroRadS.norm() * MATH_RAD_TO_DEG;
    const float accelTrust = ahrsStats.lastAccelGate.trust;
    const bool processorTrustedForUse =
        MagRuntimeProcessor::trustedForUse(processed, magCfg, nowMs);
    const uint32_t magRejectFlagsForUse =
        MagRuntimeProcessor::rejectFlagsForUse(processed, magCfg, nowMs);

    MagFieldReliabilityOutput fallbackReliability;
    MagFieldReliabilityOutput& reliability = deps_.lastFieldReliability
        ? *deps_.lastFieldReliability
        : fallbackReliability;
    updateFieldReliabilitySnapshot(nowMs,
                                   gyroNormDps,
                                   accelTrust,
                                   processorTrustedForUse,
                                   reliability);
    const bool magTrustedForUse = processorTrustedForUse && reliability.trustedForYaw;

    (void)enqueueAxisAlignmentEvidence(nowMs);
    updateAutoReference(nowMs, gyroNormDps, accelTrust, magTrustedForUse);
    updateYawCorrectionSnapshot(nowMs,
                                gyroNormDps,
                                accelTrust,
                                magTrustedForUse,
                                magRejectFlagsForUse,
                                reliability);

    if (deps_.callbacks.emitMagFrame) {
        deps_.callbacks.emitMagFrame(processed,
                                     *deps_.lastHeading,
                                     reliability,
                                     *deps_.lastYawCorrection,
                                     magRejectFlagsForUse,
                                     magTrustedForUse,
                                     deps_.callbacks.emitMagFrameUser);
    }

    const float magHeadingErrorDeg =
        (deps_.headingRef && deps_.headingRef->valid && deps_.lastHeading->valid)
            ? headingErrorToReferenceDeg(*deps_.lastHeading)
            : 0.0f;

    if (deps_.callbacks.recordStaticMagYawSample) {
        deps_.callbacks.recordStaticMagYawSample(magHeadingErrorDeg,
                                                 *deps_.lastHeading,
                                                 *deps_.lastYawCorrection,
                                                 deps_.callbacks.recordStaticMagYawSampleUser);
    }

    const uint32_t nacks = deps_.fifo ? deps_.fifo->stats().sensorHubNackWords : 0;
    if (nacks != deps_.state->nacksSeen) {
        deps_.state->nacksSeen = nacks;
    }
}

Stream& MagRuntimeController::stream() const {
    return deps_.out ? *deps_.out : Serial;
}

bool MagRuntimeController::accelReady() const {
    return deps_.accelCalibrationReady ? *deps_.accelCalibrationReady : false;
}

bool MagRuntimeController::isRecoveryActive() const {
    return deps_.recoveryActive ? deps_.recoveryActive(deps_.recoveryActiveUser) : false;
}

float MagRuntimeController::outputConfidence() const {
    return deps_.lastOutputConfidence ? *deps_.lastOutputConfidence : 0.0f;
}

bool MagRuntimeController::initSensorHub() {
    if (!deps_.state || !deps_.hub) return false;
    if (deps_.state->hubInitialized) return true;

    Lsm6dsvSensorHub::Config hubCfg;
    hubCfg.resetMasterOnBegin = true;
    hubCfg.enableInternalShubPullups = true;
    hubCfg.forceDisablePrimaryI2cI3c = true;
    hubCfg.transactionOdr = Lsm6dsvSensorHub::ShubOdr::Hz120;
    hubCfg.transactionTimeoutMs = 150;

    if (!deps_.hub->begin(hubCfg)) {
        stream().print("# ERR sensor hub init failed error=");
        stream().println(deps_.hub->lastErrorName());
        deps_.state->hubInitialized = false;
        return false;
    }

    deps_.state->hubInitialized = true;
    stream().println("# OK LSM6DSV sensor hub init for QMC6309");
    return true;
}

void MagRuntimeController::resetRuntimeCounters() {
    if (!deps_.state) return;

    deps_.state->samples = 0;
    deps_.state->queuePops = 0;
    deps_.state->lastSampleMs = 0;
    deps_.state->nacksSeen = deps_.fifo ? deps_.fifo->stats().sensorHubNackWords : 0;
    deps_.state->lastRaw = Lsm6dsvFifoReader::MagRawSample{};
    deps_.state->lastNormRaw = 0.0f;

    if (deps_.processor) deps_.processor->reset();
    if (deps_.lastProcessed) *deps_.lastProcessed = MagProcessedSample{};

    if (deps_.headingEstimator) deps_.headingEstimator->reset();
    if (deps_.lastHeading) *deps_.lastHeading = MagHeadingSample{};
    if (deps_.fieldReliability) deps_.fieldReliability->reset();
    if (deps_.lastFieldReliability) *deps_.lastFieldReliability = MagFieldReliabilityOutput{};
    resetAxisAlignmentCandidate();

    resetYawCorrectionRuntime();

    if (deps_.headingRef) deps_.headingRef->clear();
    if (deps_.headingAutoRef) deps_.headingAutoRef->resetAll();
}

bool MagRuntimeController::applyHardwareEnabledState(bool enabled, uint64_t keepTimestampUs) {
    if (!deps_.state || !deps_.hub || !deps_.qmc || !deps_.fifo) return false;

    if (!enabled) {
        deps_.hub->stopMaster();
        deps_.state->runtimeEnabled = false;
        deps_.state->fifoArmed = false;
        const bool fifoOk = reconfigureFifoForMagEnabled(false, keepTimestampUs);
        deps_.state->lastInitOk = fifoOk;
        resetRuntimeCounters();
        return fifoOk;
    }

    auto failSafeDisable = [&]() {
        deps_.hub->stopMaster();
        deps_.state->runtimeEnabled = false;
        deps_.state->fifoArmed = false;
        (void)reconfigureFifoForMagEnabled(false, keepTimestampUs);
        deps_.state->lastInitOk = false;
        resetRuntimeCounters();
    };

    if (!initSensorHub()) {
        deps_.state->enableFailures++;
        failSafeDisable();
        return false;
    }

    if (!deps_.qmc->configureNormal100Hz()) {
        stream().print("# ERR QMC6309 init100 failed qmcErr=");
        stream().print(deps_.qmc->lastErrorName());
        stream().print(" hubErr=");
        stream().println(deps_.hub->lastErrorName());
        deps_.state->enableFailures++;
        failSafeDisable();
        return false;
    }

    if (!reconfigureFifoForMagEnabled(true, keepTimestampUs)) {
        deps_.state->enableFailures++;
        failSafeDisable();
        return false;
    }

    if (!deps_.qmc->armHubFifoRead(Lsm6dsvSensorHub::ShubOdr::Hz60)) {
        stream().print("# ERR QMC6309 arm FIFO failed qmcErr=");
        stream().print(deps_.qmc->lastErrorName());
        stream().print(" hubErr=");
        stream().println(deps_.hub->lastErrorName());
        deps_.state->enableFailures++;
        failSafeDisable();
        return false;
    }

    deps_.state->runtimeEnabled = true;
    deps_.state->fifoArmed = true;
    deps_.state->lastInitOk = true;
    deps_.state->lastEnableMs = millis();
    resetRuntimeCounters();
    return true;
}

bool MagRuntimeController::reconfigureFifoForMagEnabled(bool enabled, uint64_t keepTimestampUs) {
    if (!deps_.fifo || !deps_.config) return false;

    Lsm6dsvFifoReader::Config fifoCfg = deps_.config->makeFifoConfig();
    fifoCfg.enableSensorHubSlave0 = enabled;
    fifoCfg.sensorHubSlave0PeriodUs = enabled ? deps_.magHubPeriodUs : 0.0f;

    if (!deps_.fifo->configure(fifoCfg)) {
        stream().println("# ERR FIFO reconfigure for mag failed");
        return false;
    }

    deps_.fifo->resetTimestampReconstruction(keepTimestampUs);
    if (deps_.quality && deps_.quality->counters().samples > 0) {
        deps_.quality->reset();
        deps_.quality->syncFifoStats(deps_.fifo->stats());
    }
    if (deps_.callbacks.resetFifoRuntime) {
        deps_.callbacks.resetFifoRuntime(deps_.callbacks.resetFifoRuntimeUser);
    }
    if (deps_.callbacks.requestTrackingRecovery) {
        deps_.callbacks.requestTrackingRecovery(
            imu_quality_flags::FIFO_RECOVERY_REQUESTED,
            "mag_fifo_reconfigure",
            keepTimestampUs,
            deps_.callbacks.requestTrackingRecoveryUser
        );
    }
    return true;
}

float MagRuntimeController::magRawNorm(const Lsm6dsvFifoReader::MagRawSample& m) {
    return std::sqrt(static_cast<float>(m.x) * static_cast<float>(m.x) +
                     static_cast<float>(m.y) * static_cast<float>(m.y) +
                     static_cast<float>(m.z) * static_cast<float>(m.z));
}

float MagRuntimeController::rampUp(float x, float bad, float good) {
    if (x >= good) return 1.0f;
    if (x <= bad) return 0.0f;
    if (good <= bad) return 0.0f;
    return (x - bad) / (good - bad);
}

bool MagRuntimeController::deferredServiceAllowed(MagDeferredServiceGate& gate) const {
    gate = MagDeferredServiceGate{};
    if (!deps_.callbacks.evaluateDeferredServiceGate) {
        gate.allowed = true;
        return true;
    }
    return deps_.callbacks.evaluateDeferredServiceGate(
        gate, deps_.callbacks.evaluateDeferredServiceGateUser);
}

void MagRuntimeController::clearAxisAlignmentEvidence() {
    for (AxisAlignmentEvidence& evidence : axisEvidence_) {
        evidence = AxisAlignmentEvidence{};
    }
    axisEvidenceHead_ = 0u;
    axisEvidenceTail_ = 0u;
    axisEvidenceCount_ = 0u;
}

bool MagRuntimeController::enqueueAxisAlignmentEvidence(uint32_t nowMs) {
#if !TRACKER_ENABLE_CALIBRATION_CANDIDATES
    (void)nowMs;
    return false;
#else
    if (!axisAlignmentLearningEnabled_ || !deps_.axisAlignmentCollector ||
        !deps_.axisAlignmentState || !deps_.config || !deps_.configStore ||
        !deps_.lastProcessed || !deps_.config->data.magCal.calibrationValid ||
        (deps_.calibrationCollector && deps_.calibrationCollector->active())) {
        return false;
    }
    if (deps_.axisAlignmentState->candidateStaged ||
        deps_.axisAlignmentState->blockedByExistingCandidate) {
        deps_.axisAlignmentState->pendingAction =
            MagAxisAlignmentDeferredAction::CheckCandidateSlot;
        return false;
    }
    const uint32_t axisIndependentRejects = deps_.lastProcessed->rejectFlags &
        ~(static_cast<uint32_t>(MAG_REJECT_AXIS_NOT_ALIGNED));
    if (!deps_.lastProcessed->valid || axisIndependentRejects != MAG_REJECT_NONE ||
        !deps_.lastProcessed->gyroEndpointValid) {
        return false;
    }
    MagAxisAlignmentRuntimeState& state = *deps_.axisAlignmentState;
    if (axisEvidenceCount_ >= kAxisEvidenceCapacity) {
        ++state.evidenceDropped;
        return false;
    }
    AxisAlignmentEvidence& evidence = axisEvidence_[axisEvidenceTail_];
    evidence.nowMs = nowMs;
    evidence.gyroSensorRadS = deps_.lastProcessed->gyroSensorRadS;
    evidence.gyroTimestampUs = deps_.lastProcessed->gyroTimestampUs;
    evidence.configCrc = deps_.config->data.crc32;
    evidence.mag = *deps_.lastProcessed;
    axisEvidenceTail_ = static_cast<uint8_t>((axisEvidenceTail_ + 1u) % kAxisEvidenceCapacity);
    ++axisEvidenceCount_;
    ++state.evidenceQueued;
    if (axisEvidenceCount_ > state.evidenceQueueHighWater) {
        state.evidenceQueueHighWater = axisEvidenceCount_;
    }
    return true;
#endif
}

bool MagRuntimeController::serviceOneAxisAlignmentEvidence() {
#if !TRACKER_ENABLE_CALIBRATION_CANDIDATES
    return false;
#else
    if (axisEvidenceCount_ == 0u) return false;
    const AxisAlignmentEvidence evidence = axisEvidence_[axisEvidenceHead_];
    axisEvidence_[axisEvidenceHead_] = AxisAlignmentEvidence{};
    axisEvidenceHead_ = static_cast<uint8_t>((axisEvidenceHead_ + 1u) % kAxisEvidenceCapacity);
    --axisEvidenceCount_;
    if (!deps_.config || deps_.config->data.crc32 != evidence.configCrc) {
        if (deps_.axisAlignmentState) ++deps_.axisAlignmentState->evidenceStaleDropped;
        return true;
    }
    updateAxisAlignmentCandidate(evidence);
    if (deps_.axisAlignmentState) ++deps_.axisAlignmentState->evidenceProcessed;
    return true;
#endif
}

void MagRuntimeController::updateAxisAlignmentCandidate(
    const AxisAlignmentEvidence& evidence) {
#if TRACKER_ENABLE_CALIBRATION_CANDIDATES
    if (!axisAlignmentLearningEnabled_) return;
    if (!deps_.axisAlignmentCollector || !deps_.axisAlignmentState || !deps_.config ||
        !deps_.configStore) {
        return;
    }
    if (!deps_.config->data.magCal.calibrationValid) return;
    if (deps_.calibrationCollector && deps_.calibrationCollector->active()) return;
    const uint32_t nowMs = evidence.nowMs;
    if (deps_.axisAlignmentState->nextCollectionAllowedMs != 0u &&
        static_cast<int32_t>(nowMs - deps_.axisAlignmentState->nextCollectionAllowedMs) < 0) {
        return;
    }
    if (deps_.axisAlignmentState->candidateStaged ||
        deps_.axisAlignmentState->blockedByExistingCandidate) {
        deps_.axisAlignmentState->pendingAction =
            MagAxisAlignmentDeferredAction::CheckCandidateSlot;
        return;
    }
    deps_.axisAlignmentCollector->observe(
        evidence.gyroSensorRadS, evidence.gyroTimestampUs, evidence.mag, true);
    static constexpr uint32_t kSolveRetryMs = 5000u;
    if (!deps_.axisAlignmentCollector->readyToSolve() ||
        deps_.axisAlignmentState->solvePending ||
        deps_.axisAlignmentState->stagePending) {
        return;
    }
    if (deps_.axisAlignmentState->lastSolveAttemptMs != 0u &&
        nowMs - deps_.axisAlignmentState->lastSolveAttemptMs < kSolveRetryMs) {
        return;
    }
    deps_.axisAlignmentState->solvePending = true;
    deps_.axisAlignmentState->pendingAction = MagAxisAlignmentDeferredAction::Solve;
#else
    (void)evidence;
#endif
}

bool MagRuntimeController::serviceDeferred() {
#if !TRACKER_ENABLE_CALIBRATION_CANDIDATES
    return false;
#else
    if (!axisAlignmentLearningEnabled_) return false;
    if (!deps_.axisAlignmentCollector || !deps_.axisAlignmentState || !deps_.config ||
        !deps_.configStore) {
        return false;
    }
    MagAxisAlignmentRuntimeState& state = *deps_.axisAlignmentState;
    const uint32_t nowMs = millis();
    static constexpr uint32_t kCandidateSlotRetryMs = 10000u;

    const bool evidencePending = axisEvidenceCount_ != 0u;
    const bool slotCheckDue =
        (state.candidateStaged || state.blockedByExistingCandidate) &&
        (state.lastStorageCheckMs == 0u ||
         nowMs - state.lastStorageCheckMs >= kCandidateSlotRetryMs);
    const bool stageRetryDue = state.stagePending &&
        (state.lastStorageCheckMs == 0u ||
         nowMs - state.lastStorageCheckMs >= kCandidateSlotRetryMs);
    const bool workPending = evidencePending || state.solvePending || stageRetryDue || slotCheckDue;
    if (!workPending) return false;

    // Evidence collection was moved out of the chronological mag callback to
    // protect raw IMU progress. It must obey the same measured FIFO/output
    // admission gate as solving and storage; otherwise the deferred path would
    // simply recreate the old pressure one app-loop later.
    MagDeferredServiceGate serviceGate;
    if (!deferredServiceAllowed(serviceGate)) {
        state.serviceDeferrals++;
        if (evidencePending) ++state.evidenceServiceDeferrals;
        state.lastDeferredFifoUnreadWords = serviceGate.fifoUnreadWords;
        if (serviceGate.fifoUnreadWords > state.maxDeferredFifoUnreadWords) {
            state.maxDeferredFifoUnreadWords = serviceGate.fifoUnreadWords;
        }
        state.lastDeferredRotationSlackMs = serviceGate.rotationDeadlineSlackMs;
        state.lastDeferredRejectFlags = serviceGate.rejectFlags;
        if (serviceGate.rejectFlags & MAG_DEFERRED_REJECT_SOFTWARE_FIFO_PENDING) {
            state.serviceDeferralSoftwareFifo++;
        }
        if (serviceGate.rejectFlags & MAG_DEFERRED_REJECT_HARDWARE_FIFO_STATUS) {
            state.serviceDeferralHardwareStatus++;
        }
        if (serviceGate.rejectFlags & MAG_DEFERRED_REJECT_HARDWARE_FIFO_BUSY) {
            state.serviceDeferralHardwareBusy++;
        }
        if (serviceGate.rejectFlags & MAG_DEFERRED_REJECT_OUTPUT_DEADLINE) {
            state.serviceDeferralOutputDeadline++;
        }
        return false;
    }
    state.lastDeferredFifoUnreadWords = serviceGate.fifoUnreadWords;
    if (serviceGate.fifoUnreadWords > state.maxDeferredFifoUnreadWords) {
        state.maxDeferredFifoUnreadWords = serviceGate.fifoUnreadWords;
    }
    state.lastDeferredRotationSlackMs = serviceGate.rotationDeadlineSlackMs;
    state.lastDeferredRejectFlags = MAG_DEFERRED_REJECT_NONE;

    // Process at most one queued observation per admitted service call. This
    // preserves order, keeps work bounded, and lets pose/FIFO deadlines
    // preempt the next evidence item.
    if (evidencePending) return serviceOneAxisAlignmentEvidence();

    if (slotCheckDue) {
        const uint32_t startUs = micros();
        bool exists = true;
        const bool ok = deps_.configStore->candidateExists(exists);
        const uint32_t elapsedUs = micros() - startUs;
        state.lastStorageUs = elapsedUs;
        if (elapsedUs > state.maxStorageUs) state.maxStorageUs = elapsedUs;
        state.storageServiceCalls++;
        state.lastStorageCheckMs = nowMs;
        if (ok && !exists) {
            state.candidateStaged = false;
            state.blockedByExistingCandidate = false;
            state.pendingAction = MagAxisAlignmentDeferredAction::None;
            state.confirmedIndependentSessions = 0u;
            state.sessionAgreementFailures = 0u;
            state.confirmedResult = MagAxisAlignmentResult{};
            state.activeAlignmentConfirmed = false;
            deps_.axisAlignmentCollector->reset();
        }
        return true;
    }

    if (state.solvePending) {
        state.solvePending = false;
        state.lastSolveAttemptMs = nowMs;
        state.pendingAction = MagAxisAlignmentDeferredAction::None;
        state.solveServiceCalls++;

        const Mat3* activeAlignment = deps_.config->data.magCal.axisAlignmentValid
            ? &deps_.config->data.magCal.magToImu
            : nullptr;
        MagAxisAlignmentResult result;
        const uint32_t startUs = micros();
        const bool solved = deps_.axisAlignmentCollector->solve(
            deps_.config->data.magCal.hardIron,
            deps_.config->data.magCal.softIron,
            result,
            activeAlignment);
        const uint32_t elapsedUs = micros() - startUs;
        state.lastSolveUs = elapsedUs;
        if (elapsedUs > state.maxSolveUs) state.maxSolveUs = elapsedUs;
        state.lastResult = result;
        state.lastSolveValid = solved;

        if (!solved) {
            if (deps_.axisAlignmentCollector->intervalCount() >=
                MagAxisAlignmentCollector::kMaxIntervals) {
                deps_.axisAlignmentCollector->reset();
            }
            return true;
        }

        static constexpr uint32_t kIndependentSessionSeparationMs = 15000u;
        static constexpr uint8_t kRequiredIndependentSessions = 3u;
        static constexpr float kSessionAgreementDeg = 1.25f;

        if (activeAlignment &&
            magAxisMatricesEquivalent(result.magToImu, *activeAlignment)) {
            // A fresh post-promotion solve is the magnetic probation proof.
            state.activeAlignmentConfirmed = true;
            state.lastIndependentSessionMs = nowMs;
            state.nextCollectionAllowedMs = nowMs + kIndependentSessionSeparationMs;
            deps_.axisAlignmentCollector->reset();
            return true;
        }

        if (activeAlignment && !result.improvesActive) {
            deps_.axisAlignmentCollector->reset();
            state.nextCollectionAllowedMs = nowMs + kIndependentSessionSeparationMs;
            return true;
        }
        if (result.qualityScore < 0.62f) {
            deps_.axisAlignmentCollector->reset();
            state.nextCollectionAllowedMs = nowMs + kIndependentSessionSeparationMs;
            return true;
        }

        // 0022 produces one internally train/validation-proven solve. 0023
        // requires three physically separated solve sessions before that result
        // is allowed to occupy the shared candidate slot. One long gesture is
        // therefore never counted as several confirmations.
        if (state.confirmedIndependentSessions == 0u) {
            state.confirmedResult = result;
            state.confirmedIndependentSessions = 1u;
        } else if (magAxisRotationDifferenceDeg(
                       state.confirmedResult.magToImu, result.magToImu) <=
                       kSessionAgreementDeg &&
                   magAxisMatricesEquivalent(
                       state.confirmedResult.coarseMagToImu,
                       result.coarseMagToImu,
                       0.05f)) {
            ++state.confirmedIndependentSessions;
            state.confirmedResult.qualityScore = std::min(
                state.confirmedResult.qualityScore, result.qualityScore);
            state.confirmedResult.score = std::max(
                state.confirmedResult.score, result.score);
            state.confirmedResult.validationScore = std::max(
                state.confirmedResult.validationScore, result.validationScore);
            state.confirmedResult.usedIntervals = static_cast<uint16_t>(
                std::min<uint32_t>(0xFFFFu,
                    static_cast<uint32_t>(state.confirmedResult.usedIntervals) +
                    result.usedIntervals));
            state.confirmedResult.independentWindows += result.independentWindows;
        } else {
            ++state.sessionAgreementFailures;
            state.confirmedResult = result;
            state.confirmedIndependentSessions = 1u;
        }
        state.lastIndependentSessionMs = nowMs;
        state.nextCollectionAllowedMs = nowMs + kIndependentSessionSeparationMs;
        deps_.axisAlignmentCollector->reset();

        if (state.confirmedIndependentSessions < kRequiredIndependentSessions) {
            return true;
        }
        state.lastResult = state.confirmedResult;
        state.stagePending = true;
        state.pendingAction = MagAxisAlignmentDeferredAction::Stage;
        state.lastStorageCheckMs = 0u;
        return true;
    }

    if (stageRetryDue) {
        const bool staged = stageAxisAlignmentCandidate(state.lastResult, nowMs);
        if (staged) {
            state.stagePending = false;
            state.pendingAction = MagAxisAlignmentDeferredAction::CheckCandidateSlot;
            deps_.axisAlignmentCollector->reset();
        } else if (state.blockedByExistingCandidate) {
            state.stagePending = false;
            state.pendingAction = MagAxisAlignmentDeferredAction::CheckCandidateSlot;
        }
        return true;
    }

    return false;
#endif
}

bool MagRuntimeController::stageAxisAlignmentCandidate(
    const MagAxisAlignmentResult& result,
    uint32_t nowMs) {
#if !TRACKER_ENABLE_CALIBRATION_CANDIDATES
    (void)result;
    (void)nowMs;
    return false;
#else
    if (!deps_.axisAlignmentState || !deps_.axisAlignmentCandidateWorkspace ||
        !deps_.config || !deps_.configStore || !result.valid ||
        result.qualityScore < 0.62f) {
        return false;
    }
    MagAxisAlignmentRuntimeState& state = *deps_.axisAlignmentState;
    state.stageAttempts++;

    const uint32_t storageStartUs = micros();
    bool candidateExists = true;
    const bool presenceRead = deps_.configStore->candidateExists(candidateExists);
    state.lastStorageCheckMs = nowMs;
    if (!presenceRead) {
        state.stageFailures++;
        const uint32_t elapsedUs = micros() - storageStartUs;
        state.lastStorageUs = elapsedUs;
        if (elapsedUs > state.maxStorageUs) state.maxStorageUs = elapsedUs;
        state.storageServiceCalls++;
        return false;
    }
    if (candidateExists) {
        state.blockedByExistingCandidate = true;
        const uint32_t elapsedUs = micros() - storageStartUs;
        state.lastStorageUs = elapsedUs;
        if (elapsedUs > state.maxStorageUs) state.maxStorageUs = elapsedUs;
        state.storageServiceCalls++;
        return false;
    }

    TrackerConfig& candidate = *deps_.axisAlignmentCandidateWorkspace;
    candidate = *deps_.config;
    candidate.data.magCal.magToImu = result.magToImu;
    candidate.data.magCal.axisAlignmentValid = true;
    candidate.sanitize();
    candidate.updateCrc();
    if (!candidate.data.magCal.axisAlignmentValid ||
        !magAxisMatricesEquivalent(candidate.data.magCal.magToImu,
                                   result.magToImu,
                                   0.05f)) {
        state.stageFailures++;
        state.stagePending = false;
        state.pendingAction = MagAxisAlignmentDeferredAction::None;
        deps_.axisAlignmentCollector->reset();
        const uint32_t elapsedUs = micros() - storageStartUs;
        state.lastStorageUs = elapsedUs;
        if (elapsedUs > state.maxStorageUs) state.maxStorageUs = elapsedUs;
        state.storageServiceCalls++;
        return false;
    }

    TrackerCalibrationCandidateMetadata metadata;
    metadata.provenance = TrackerCalibrationProvenance::Background;
    metadata.sampleCount = result.usedIntervals;
    metadata.independentWindowCount = result.independentWindows;
    metadata.quality = trackerCalibrationQualityFromConfig(candidate);
    metadata.quality.alignmentScore = result.qualityScore;
    metadata.quality.qualityFlags |=
        tracker_calibration_quality_flags::AUTONOMY_0022 |
        tracker_calibration_quality_flags::ALIGNMENT_MEASURED |
        tracker_calibration_quality_flags::SOURCE_MEASURED;
    trackerCalibrationQualityRecomputeOverall(candidate, metadata.quality);
    trackerCalibrationQualitySetProvenance(
        metadata.quality, TrackerCalibrationProvenance::Background);

    const bool staged = deps_.configStore->stageCandidate(candidate, metadata, nowMs);
    const uint32_t elapsedUs = micros() - storageStartUs;
    state.lastStorageUs = elapsedUs;
    if (elapsedUs > state.maxStorageUs) state.maxStorageUs = elapsedUs;
    state.storageServiceCalls++;
    if (!staged) {
        state.stageFailures++;
        return false;
    }

    state.candidateStaged = true;
    state.blockedByExistingCandidate = false;
    state.stageSuccesses++;
    state.lastStageMs = nowMs;
    return true;
#endif
}

void MagRuntimeController::updateAutoReference(uint32_t nowMs,
float gyroNormDps,
float accelTrust,
bool magTrustedForUse) {
    if (!deps_.headingAutoRef) return;
    if (!deps_.headingAutoRef->enabled) return;
    if (isRecoveryActive()) {
        deps_.headingAutoRef->resetCandidate();
        return;
    }

    if (deps_.headingRef && deps_.headingRef->valid) {
        deps_.headingAutoRef->done = true;
        return;
    }

    const MagYawCorrectionConfig yawCfg = yawConfig();

    uint32_t reject = 0;

    if (!yawCfg.enabled || !yawCfg.applyEnabled) reject |= 1u << 0;
    if (!deps_.lastHeading || !deps_.lastHeading->valid) reject |= 1u << 1;
    if (!magTrustedForUse) reject |= 1u << 2;
    if (!tracker::isFinite(gyroNormDps) || gyroNormDps > 1.5f) reject |= 1u << 3;
    if (!tracker::isFinite(accelTrust) || accelTrust < 0.90f) reject |= 1u << 4;

    const float hTrust = (deps_.lastHeading != nullptr)
        ? rampUp(deps_.lastHeading->horizontalNorm, yawCfg.horizontalNormBad, yawCfg.horizontalNormGood)
        : 0.0f;

    if (!tracker::isFinite(hTrust) || hTrust < 0.25f) reject |= 1u << 5;

    deps_.headingAutoRef->lastRejectFlags = reject;
    deps_.headingAutoRef->lastGyroNormDps = gyroNormDps;
    deps_.headingAutoRef->lastAccelTrust = accelTrust;
    deps_.headingAutoRef->lastHorizontalTrust = hTrust;

    if (reject != 0) {
        deps_.headingAutoRef->stableSinceMs = 0;
        return;
    }

    if (deps_.headingAutoRef->stableSinceMs == 0) {
        deps_.headingAutoRef->stableSinceMs = nowMs;
        return;
    }

    const uint32_t stableMs = nowMs - deps_.headingAutoRef->stableSinceMs;
    if (stableMs >= 3000) {
        // Auto-reference is a background stabilization event. Keep it silent so it
        // cannot interleave with SlimeVR Server serial setup replies.
        setHeadingReference("auto", false);
    }
}

bool MagRuntimeController::applyYawCorrectionToAhrs(const MagYawCorrectionOutput& yaw) {
    if (!deps_.ahrs) return false;
    if (!yaw.applyAllowed || yaw.correctionStepRad == 0.0f) return false;
    if (!tracker::isFinite(yaw.correctionStepRad)) return false;

    const Vec3 worldAxis = deps_.ahrs->config().worldUp.normalized();
    if (!worldAxis.isFinite() || worldAxis.normSq() < MATH_EPSILON) {
        return false;
    }

    const Vec3 correctionWorldRad = worldAxis * yaw.correctionStepRad;
    const Quat corrected = applyWorldCorrection(deps_.ahrs->quaternion(), correctionWorldRad).withPositiveW();

    if (!corrected.isFinite()) {
        return false;
    }

    deps_.ahrs->setQuaternion(corrected);
    return true;
}

#undef TRACKER_MAG_RUNTIME_NOINLINE

} // namespace tracker
