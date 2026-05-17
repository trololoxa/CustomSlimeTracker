#include "runtime/mag_runtime_controller.hpp"

#include <cmath>

#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "connection/lsm6dsv_sensorhub.hpp"
#include "runtime/output_runtime.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/imu_quality.hpp"
#include "sensor/mag_calibration.hpp"
#include "sensor/qmc6309.hpp"

namespace tracker {

void MagRuntimeController::begin(const MagRuntimeControllerDeps& deps) {
    deps_ = deps;
}

MagRuntimeConfig MagRuntimeController::runtimeConfig() const {
    MagRuntimeConfig c;
    if (!deps_.config) return c;

    const auto& magCal = deps_.config->data.magCal;
    c.enabled = magCal.driverEnabled;
    c.calibrationValid = magCal.calibrationValid;
    c.axisAlignmentValid = magCal.axisAlignmentValid;

    c.hardIron = magCal.hardIron;
    c.softIron = magCal.softIron;
    c.magToImu = magCal.magToImu;

    c.expectedFieldNorm = magCal.expectedFieldNorm;
    c.minTrustNorm = magCal.minTrustNorm;
    c.maxTrustNorm = magCal.maxTrustNorm;

    c.maxSampleAgeMs = 250;
    c.minUsableNorm = 1.0e-6f;
    return c;
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

void MagRuntimeController::resetOrientationState(const char* reason, uint64_t timestampUs, bool rebaseAhrsTimebase) {
    if (deps_.headingEstimator) deps_.headingEstimator->reset();
    if (deps_.lastHeading) *deps_.lastHeading = MagHeadingSample{};

    if (deps_.headingRef) deps_.headingRef->clear();
    if (deps_.headingAutoRef) {
        deps_.headingAutoRef->done = false;
        deps_.headingAutoRef->resetCandidate();
    }

    resetYawCorrectionRuntime();

    if (rebaseAhrsTimebase && timestampUs != 0 && deps_.ahrs) {
        deps_.ahrs->rebaseTimestamp(timestampUs);
    }

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

    const uint64_t fifoTs = deps_.fifo->stats().lastAssignedTimestampUs;
    const uint64_t keepTs = fifoTs != 0
        ? fifoTs
        : (deps_.fallbackTimestampUs ? *deps_.fallbackTimestampUs : 0);

    if (!enabled) {
        deps_.config->data.magCal.driverEnabled = false;
        deps_.config->updateCrc();
        deps_.hub->stopMaster();
        deps_.state->runtimeEnabled = false;
        deps_.state->fifoArmed = false;
        deps_.state->lastInitOk = true;
        reconfigureFifoForCurrentMagConfig(keepTs);
        resetRuntimeCounters();

        if (persist && deps_.configStore && !deps_.configStore->save(*deps_.config)) {
            stream().print("# ERR mag disable save failed: ");
            stream().println(deps_.configStore->lastErrorName());
            return false;
        }
        return true;
    }

    deps_.config->data.magCal.driverEnabled = true;
    deps_.config->updateCrc();

    if (!initSensorHub()) {
        deps_.state->lastInitOk = false;
        deps_.state->enableFailures++;
        return false;
    }

    if (!deps_.qmc->configureNormal100Hz()) {
        stream().print("# ERR QMC6309 init100 failed qmcErr=");
        stream().print(deps_.qmc->lastErrorName());
        stream().print(" hubErr=");
        stream().println(deps_.hub->lastErrorName());
        deps_.state->lastInitOk = false;
        deps_.state->enableFailures++;
        return false;
    }

    if (!reconfigureFifoForCurrentMagConfig(keepTs)) {
        deps_.state->lastInitOk = false;
        deps_.state->enableFailures++;
        return false;
    }

    if (!deps_.qmc->armHubFifoRead(Lsm6dsvSensorHub::ShubOdr::Hz60)) {
        stream().print("# ERR QMC6309 arm FIFO failed qmcErr=");
        stream().print(deps_.qmc->lastErrorName());
        stream().print(" hubErr=");
        stream().println(deps_.hub->lastErrorName());
        deps_.state->lastInitOk = false;
        deps_.state->enableFailures++;
        return false;
    }

    deps_.state->runtimeEnabled = true;
    deps_.state->fifoArmed = true;
    deps_.state->lastInitOk = true;
    deps_.state->lastEnableMs = millis();
    resetRuntimeCounters();

    if (persist && deps_.configStore && !deps_.configStore->save(*deps_.config)) {
        stream().print("# ERR mag enable save failed: ");
        stream().println(deps_.configStore->lastErrorName());
        return false;
    }

    stream().println("# OK QMC6309 -> LSM6DSV FIFO enabled: SLV0 0x01..0x06 @60Hz");
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

    if (verbose) {
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

    deps_.config->data.magYaw.applyEnabled = enabled;
    deps_.config->sanitize();
    deps_.config->updateCrc();

    resetYawCorrectionRuntime();

    if (persist && deps_.configStore && !deps_.configStore->save(*deps_.config)) {
        stream().print("# ERR mag yaw correction save failed: ");
        stream().println(deps_.configStore->lastErrorName());
        return false;
    }

    stream().print("# OK mag yaw correction apply=");
    stream().println(enabled ? "enabled" : "disabled");
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
        stream().println("# Need more samples and wider 3-axis rotation coverage");
        return false;
    }

    deps_.config->captureFromMagCalibrationResult(
        result,
        deps_.calibrationCollector->samples(),
        deps_.calibrationCollector->rejected(),
        deps_.calibrationCollector->saturated(),
        deps_.calibrationCollector->normMin(),
        deps_.calibrationCollector->normMean(),
        deps_.calibrationCollector->normMax(),
        millis()
    );

    if (persist && deps_.configStore && !deps_.configStore->save(*deps_.config)) {
        stream().print("# ERR mag calibration save failed: ");
        stream().println(deps_.configStore->lastErrorName());
        return false;
    }

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
    stream().print("# OK mag coverageScore=");
    stream().println(result.coverageScore, 6);

    return true;
}

void MagRuntimeController::processRawSample(const Lsm6dsvFifoReader::MagRawSample& mag) {
    if (!deps_.state || !deps_.processor || !deps_.headingEstimator ||
        !deps_.lastProcessed || !deps_.lastHeading || !deps_.lastYawCorrection ||
        !deps_.ahrs || !deps_.yawCorrection) {
        return;
    }

    deps_.state->samples++;
    deps_.state->queuePops++;
    deps_.state->lastSampleMs = millis();
    deps_.state->lastRaw = mag;
    deps_.state->lastNormRaw = magRawNorm(mag);

    if (deps_.calibrationCollector) {
        deps_.calibrationCollector->push(mag, deps_.state->lastNormRaw, millis());
    }

    MagProcessedSample processed;
    deps_.processor->process(mag, runtimeConfig(), millis(), processed);
    *deps_.lastProcessed = processed;

    MagHeadingSample heading;
    deps_.headingEstimator->update(
        *deps_.lastProcessed,
        deps_.ahrs->quaternionPositiveW(),
        headingConfig(),
        millis(),
        heading
    );
    *deps_.lastHeading = heading;

    const uint32_t nowMs = millis();
    const MagRuntimeConfig magCfg = runtimeConfig();
    const Ahrs6DofStats& ahrsStats = deps_.ahrs->stats();

    const float gyroNormDps = ahrsStats.lastGyroRadS.norm() * MATH_RAD_TO_DEG;
    const float accelTrust = ahrsStats.lastAccelGate.trust;

    const bool magTrustedForUse =
        MagRuntimeProcessor::trustedForUse(*deps_.lastProcessed, magCfg, nowMs);
    const uint32_t magRejectFlagsForUse =
        MagRuntimeProcessor::rejectFlagsForUse(*deps_.lastProcessed, magCfg, nowMs);

    updateAutoReference(nowMs, gyroNormDps, accelTrust, magTrustedForUse);

    MagYawCorrectionInput yawIn;
    yawIn.mag = *deps_.lastProcessed;
    yawIn.heading = *deps_.lastHeading;
    yawIn.referenceValid = deps_.headingRef && deps_.headingRef->valid;
    yawIn.referenceWorldYawRad = deps_.headingRef ? deps_.headingRef->worldYawRad : 0.0f;
    yawIn.magTrustedForUse = magTrustedForUse;
    yawIn.magRejectFlagsForUse = magRejectFlagsForUse;
    yawIn.gyroNormDps = gyroNormDps;
    yawIn.accelTrust = accelTrust;
    yawIn.nowMs = nowMs;

    MagYawCorrectionOutput yawOut;
    deps_.yawCorrection->update(yawIn, yawConfig(), yawOut);

    if (applyYawCorrectionToAhrs(yawOut)) {
        yawOut.applied = true;
        deps_.yawCorrection->markApplied(yawOut.correctionStepDeg);
    }

    *deps_.lastYawCorrection = yawOut;

    if (deps_.callbacks.emitMagFrame) {
        deps_.callbacks.emitMagFrame(*deps_.lastProcessed,
                                     *deps_.lastHeading,
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
                                                 yawOut,
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

    resetYawCorrectionRuntime();

    if (deps_.headingRef) deps_.headingRef->clear();
    if (deps_.headingAutoRef) deps_.headingAutoRef->resetAll();
}

bool MagRuntimeController::reconfigureFifoForCurrentMagConfig(uint64_t keepTimestampUs) {
    if (!deps_.fifo || !deps_.config) return false;

    Lsm6dsvFifoReader::Config fifoCfg = deps_.config->makeFifoConfig();
    fifoCfg.enableSensorHubSlave0 = deps_.config->data.magCal.driverEnabled;
    fifoCfg.sensorHubSlave0PeriodUs = deps_.config->data.magCal.driverEnabled
        ? deps_.magHubPeriodUs
        : 0.0f;

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
        setHeadingReference("auto", true);
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

} // namespace tracker
