#pragma once

// Serial command and status hooks used by the app composition layer.
// This file is included by app/tracker_app_hooks.hpp after shared app dependencies.

static void hookResetFifoRuntime(void* user) {
    (void)user;
    resetFifoRuntimeCounters();
    g_lastSampleTimestampUs = 0;
    enterTrackingRecovery(imu_quality_flags::FIFO_RECOVERY_REQUESTED, "manual_fifo_reset", lsmFifo.stats().lastAssignedTimestampUs);
}

static void hookResetAhrsRuntime(void* user) {
    (void)user;
    g_lastSampleTimestampUs = 0;
    resetOrientationDependentState("ahrs_or_config_reset", lsmFifo.stats().lastAssignedTimestampUs, false);
}

static RuntimeStatusReporterDeps makeRuntimeStatusReporterDeps() {
    RuntimeStatusReporterDeps deps;
    deps.config = &g_config;
    deps.configLoadedFromNvs = g_configLoadedFromNvs;
    deps.spiHz = lsmBus.spiHz();
    deps.runtimeSamples = g_runtimeSamples;
    deps.fifoIntCount = g_fifoIntCount;
    deps.fifoEvents = &g_fifoEvents;
    deps.fifo = &lsmFifo;
    deps.latestTempC = g_latestTempC;
    deps.imuCal = &g_imuCal;
    deps.gyroTempComp = &g_gyroTempComp;
    deps.runtimeBias = &g_runtimeBias;
    deps.quality = &g_quality;
    deps.ahrs = &g_ahrs6dof;
    deps.trackingState = &g_trackingState;
    deps.trackingStateName = trackingStateName();
    deps.lastOutputConfidence = g_lastOutputConfidence;
    deps.streamState = &g_streamState;
    deps.preparedOutput = &g_preparedOutput;
    deps.magState = &g_magState;
    deps.magProcessor = &g_magProcessor;
    deps.lastMagProcessed = &g_lastMagProcessed;
    deps.magHeading = &g_magHeading;
    deps.lastMagHeading = &g_lastMagHeading;
    deps.magHeadingRef = &g_magHeadingRef;
    deps.magHeadingAutoRef = &g_magHeadingAutoRef;
    deps.lastMagYawCorrection = &g_lastMagYawCorrection;
    return deps;
}

static void printRuntimeStatus(Stream& out, void* user) {
    (void)user;
    runtimeStatusPrint(out, makeRuntimeStatusReporterDeps());
}

static void printRuntimeHealth(Stream& out, void* user) {
    (void)user;
    runtimeStatusPrintHealth(out, makeRuntimeStatusReporterDeps());
}


static void printLogSummary(Stream& out, void* user) {
    (void)user;
    machineLogPrintSummary(out,
                           g_logState,
                           g_logCounters,
                           g_runtimeSamples,
                           g_trackingState.recoveryEnterCount(),
                           g_quality,
                           lsmFifo,
                           g_magProcessor,
                           g_magYawCorrection,
                           g_ahrs6dof,
                           g_lastMagProcessed,
                           g_lastMagYawCorrection,
                           g_runtimeBias);
}

static void emitMachineLogFrame(const Lsm6dsv::RawSample& raw,
                                const Lsm6dsv::Sample& calibrated,
                                const ImuQualityResult& quality) {
    machineLogEmitFrame(Serial,
                        g_logState,
                        g_logCounters,
                        g_lastBiasLogEmitUs,
                        MACHINE_BIAS_LOG_PERIOD_US,
                        raw,
                        calibrated,
                        quality,
                        g_ahrs6dof,
                        trackingStateName(),
                        g_trackingState.recoveryActive(),
                        g_gyroTempComp,
                        g_imuCal,
                        g_runtimeBias,
                        currentGyroBiasRadS(calibrated.temp_c) * MATH_RAD_TO_DEG,
                        gyroBiasRuntimeFlags(calibrated.temp_c));
}

static void emitMachineLogMagFrame(const MagProcessedSample& mag,
                                   const MagHeadingSample& heading,
                                   const MagYawCorrectionOutput& yaw,
                                   uint32_t rejectFlagsForUse,
                                   bool trustedForUse) {
    machineLogEmitMagFrame(Serial,
                           g_logState,
                           g_logCounters,
                           mag,
                           heading,
                           yaw,
                           rejectFlagsForUse,
                           trustedForUse);
}

static void setupStaticTestRunner() {
    StaticTestRunner::Dependencies deps;
    deps.activeTest = &g_staticTest;
    deps.lastCompletedTest = &g_lastCompletedStaticTest;
    deps.lastCompletedValid = &g_lastCompletedStaticTestValid;
    deps.lastCompletedFinishedMs = &g_lastCompletedStaticTestFinishedMs;
    deps.quality = &g_quality;
    deps.fifo = &lsmFifo;
    deps.perf = &g_perf;
    deps.config = &g_config;
    deps.ahrs = &g_ahrs6dof;
    deps.magProcessor = &g_magProcessor;
    deps.magHeading = &g_magHeading;
    deps.magYawCorrection = &g_magYawCorrection;
    deps.magHeadingRef = &g_magHeadingRef;
    deps.lastMagYawCorrection = &g_lastMagYawCorrection;
    deps.progressPeriodMs = HEARTBEAT_PERIOD_MS;
    g_staticTestRunner.begin(deps);
}

static bool startStaticTestHook(uint32_t durationMs, void* user) {
    (void)user;
    const float magErrorStartDeg =
        (g_magHeadingRef.valid && g_lastMagHeading.valid)
            ? magHeadingErrorToReferenceDeg(g_lastMagHeading)
            : 0.0f;
    return g_staticTestRunner.start(durationMs, Serial, magErrorStartDeg);
}

static bool stopStaticTestHook(void* user) {
    (void)user;
    return g_staticTestRunner.stop();
}

static void printStaticTestStatus(Stream& out, void* user) {
    (void)user;
    g_staticTestRunner.printStatus(out);
}

static bool fitGyroTempFromLastStaticHook(bool persist, Stream& out, void* user);

static TrackerCommandRuntimeObjects makeTrackerCommandRuntimeObjects() {
    TrackerCommandRuntimeObjects objects;
    objects.io = &Serial;
    objects.config = &g_config;
    objects.configStore = &g_configStore;
    objects.lsm = &lsm;
    objects.fifo = &lsmFifo;
    objects.sensorHub = &lsmHub;
    objects.mag = &qmc;
    objects.imuCal = &g_imuCal;
    objects.gyroTempComp = &g_gyroTempComp;
    objects.quality = &g_quality;
    objects.ahrs = &g_ahrs6dof;
    objects.calibrationIo = &g_calIo;
    objects.accelCalRunner = &g_accelCalRunner;
    objects.streamState = &g_streamState;
    objects.logState = &g_logState;
    return objects;
}

static TrackerCommandRuntimeHooks makeTrackerCommandRuntimeHooks() {
    TrackerCommandRuntimeHooks hooks;
    hooks.resetFifoRuntime = hookResetFifoRuntime;
    hooks.resetAhrsRuntime = hookResetAhrsRuntime;
    hooks.printRuntimeStatus = printRuntimeStatus;
    hooks.printRuntimeHealth = printRuntimeHealth;
    hooks.setSpiFrequency = setRuntimeSpiFrequency;
    hooks.emitLogHeader = emitMachineLogHeader;
    hooks.printLogSummary = printLogSummary;
    hooks.resetLogCounters = resetLogCountersHook;
    hooks.printRuntimeGyroBiasStatus = printRuntimeGyroBiasStatus;
    hooks.setRuntimeGyroBiasEnabled = setRuntimeGyroBiasEnabled;
    hooks.resetRuntimeGyroBiasEstimator = resetRuntimeGyroBiasEstimator;
    hooks.startStaticTest = startStaticTestHook;
    hooks.stopStaticTest = stopStaticTestHook;
    hooks.printStaticTestStatus = printStaticTestStatus;
    hooks.setMagRuntimeEnabled = setMagRuntimeEnabledHook;
    hooks.printMagRuntimeStatus = printMagRuntimeStatus;
    hooks.printMagProcessedStatus = printMagProcessedStatus;
    hooks.printMagHeadingStatus = printMagHeadingStatus;
    hooks.setMagHeadingReference = setMagHeadingReferenceHook;
    hooks.clearMagHeadingReference = clearMagHeadingReferenceHook;
    hooks.setMagHeadingAutoReferenceEnabled = setMagHeadingAutoReferenceEnabledHook;
    hooks.printMagYawCorrectionStatus = printMagYawCorrectionStatus;
    hooks.resetMagYawCorrection = resetMagYawCorrectionHook;
    hooks.setMagYawCorrectionApplyEnabled = setMagYawCorrectionApplyEnabledHook;
    hooks.startMagCalibration = startMagCalibrationHook;
    hooks.stopMagCalibration = stopMagCalibrationHook;
    hooks.resetMagCalibration = resetMagCalibrationHook;
    hooks.applyMagCalibration = applyMagCalibrationHook;
    hooks.printMagCalibrationStatus = printMagCalibrationStatus;
    hooks.fitGyroTempFromLastStatic = fitGyroTempFromLastStaticHook;
    return hooks;
}

static void setupCommandInterface() {
    setupStaticTestRunner();
    wireTrackerCommandContext(g_cmdCtx,
                              makeTrackerCommandRuntimeObjects(),
                              makeTrackerCommandRuntimeHooks());
    g_cli.begin(g_cmdCtx);
}
