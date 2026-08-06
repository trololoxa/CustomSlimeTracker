#pragma once

#include "runtime/orientation_runtime_reset.hpp"

// Serial command and status hooks used by the app composition layer.
// This file is included by app/tracker_app_hooks.hpp after shared app dependencies.

static void hookResetFifoRuntime(void* user) {
    (void)user;
    g_fifoRuntime.resetWork();
    resetFifoRuntimeCounters();
    g_lastSampleTimestampUs = 0;
}

static void hookResetAhrsRuntime(void* user) {
    (void)user;
    g_lastSampleTimestampUs = 0;
    g_lastQualityFlags = 0;

    OrientationRuntimeResetDeps deps;
    deps.ahrs = &g_ahrs6dof;
    deps.preparedOutput = &g_preparedOutput;
    deps.resetDependentState = resetOrientationDependentState;
    (void)resetOrientationRuntime(
        deps,
        "ahrs_or_config_reset",
        lsmFifo.stats().lastAssignedTimestampUs
    );
}

#if TRACKER_HAS_MOTION_LIGHT_SLEEP
static bool requestMotionLightSleepHook(void* user) {
    (void)user;
    return g_app.requestMotionLightSleep();
}
#endif

static bool serviceNonCliRuntimeHook(void* user);

#if TRACKER_ENABLE_DETAILED_RUNTIME_STATUS
static RuntimeStatusReporterDeps makeRuntimeStatusReporterDeps() {
    RuntimeStatusReporterDeps deps;
    deps.config = &g_config;
    deps.configLoadedFromNvs = g_configLoadedFromNvs;
    deps.spiHz = lsmBus.spiHz();
    deps.runtimeSamples = g_runtimeSamples;
    deps.fifoIntCount = g_fifoIntCount;
    deps.fifoEvents = &g_fifoEvents;
    deps.fifoRuntime = &g_fifoRuntime;
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
    deps.serviceNonCliRuntime = serviceNonCliRuntimeHook;
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

#endif

static void printLogSummary(Stream& out, void* user) {
    (void)user;
#if TRACKER_ENABLE_MACHINE_LOG
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
    g_machineLogDeferred.printStatus(out);
#else
    out.println("# machine log is not compiled in this profile");
#endif
}

static void emitMachineLogFrame(const Lsm6dsv::RawSample& raw,
                                const Lsm6dsv::Sample& calibrated,
                                const ImuQualityResult& quality,
                                const GyroTempCompRuntimeEval& tempEval,
                                const Vec3& currentGyroBiasRadS) {
#if TRACKER_ENABLE_MACHINE_LOG
    (void)g_machineLogDeferred.enqueueImu(
        raw,
        calibrated,
        quality,
        g_ahrs6dof,
        trackingStateName(),
        g_trackingState.recoveryActive(),
        g_gyroTempComp,
        g_imuCal,
        g_runtimeBias,
        currentGyroBiasRadS * MATH_RAD_TO_DEG,
        runtimeBiasGyroBiasRuntimeFlags(g_runtimeBias, tempEval));
#else
    (void)raw;
    (void)calibrated;
    (void)quality;
    (void)tempEval;
    (void)currentGyroBiasRadS;
#endif
}

static void emitMachineLogMagFrame(const MagProcessedSample& mag,
                                   const MagHeadingSample& heading,
                                   const MagFieldReliabilityOutput& reliability,
                                   const MagYawCorrectionOutput& yaw,
                                   uint32_t rejectFlagsForUse,
                                   bool trustedForUse) {
#if TRACKER_ENABLE_MACHINE_LOG
    (void)g_machineLogDeferred.enqueueMag(mag,
                                          heading,
                                          reliability,
                                          yaw,
                                          rejectFlagsForUse,
                                          trustedForUse);
#else
    (void)mag;
    (void)heading;
    (void)reliability;
    (void)yaw;
    (void)rejectFlagsForUse;
    (void)trustedForUse;
#endif
}

static void setupStaticTestRunner() {
#if TRACKER_ENABLE_STATIC_TEST
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
    deps.wifi = &g_wifiManager;
    deps.slimevr = &g_slimevrRuntime;
    deps.magProcessor = &g_magProcessor;
    deps.magHeading = &g_magHeading;
    deps.magYawCorrection = &g_magYawCorrection;
    deps.magHeadingRef = &g_magHeadingRef;
    deps.lastMagYawCorrection = &g_lastMagYawCorrection;
    deps.progressPeriodMs = HEARTBEAT_PERIOD_MS;
    g_staticTestRunner.begin(deps);
#endif
}

static void setupRuntimeTestRunner() {
#if TRACKER_ENABLE_RUNTIME_TEST
    RuntimeTestRunner::Dependencies deps;
    deps.perf = &g_perf;
    deps.fifo = &lsmFifo;
    deps.quality = &g_quality;
    deps.wifi = &g_wifiManager;
    deps.slimevr = &g_slimevrRuntime;
#if TRACKER_ENABLE_BATTERY_RUNTIME
    deps.battery = &g_batteryRuntime;
#endif
    deps.trackingState = &g_trackingState;
    deps.runtimeSamples = &g_runtimeSamples;
    deps.latestTempC = &g_latestTempC;
    deps.progressPeriodMs = HEARTBEAT_PERIOD_MS;
    g_runtimeTestRunner.begin(deps);
#endif
}

static bool startStaticTestHook(uint32_t durationMs, Stream& out, void* user) {
    (void)user;
#if TRACKER_ENABLE_STATIC_TEST
#if TRACKER_ENABLE_RUNTIME_TEST
    if (g_runtimeTestRunner.active()) return false;
#endif
    const float magErrorStartDeg =
        (g_magHeadingRef.valid && g_lastMagHeading.valid)
            ? magHeadingErrorToReferenceDeg(g_lastMagHeading)
            : 0.0f;
    return g_staticTestRunner.start(durationMs, out, magErrorStartDeg);
#else
    (void)durationMs;
    return false;
#endif
}

static bool stopStaticTestHook(Stream& out, bool force, void* user) {
    (void)user;
#if TRACKER_ENABLE_STATIC_TEST
    return g_staticTestRunner.stop(out, force);
#else
    (void)out;
    (void)force;
    return false;
#endif
}

static void printStaticTestStatus(Stream& out, void* user) {
    (void)user;
#if TRACKER_ENABLE_STATIC_TEST
    g_staticTestRunner.printStatus(out);
#else
    out.println("# static test is not compiled in this profile");
#endif
}

static bool printStaticTestReport(Stream& out, void* user) {
    (void)user;
#if TRACKER_ENABLE_STATIC_TEST
#if TRACKER_ENABLE_RUNTIME_TEST
    if (g_runtimeTestRunner.active()) {
        out.println("# ERR stop the active runtime test before printing a report");
        return false;
    }
#endif
    if (g_staticTestRunner.active()) {
        out.println("# ERR stop the active static test before printing a report");
        return false;
    }
    return g_staticTestRunner.printLastReport(out);
#else
    out.println("# static test is not compiled in this profile");
    return false;
#endif
}

static bool printStaticTestSummary(Stream& out, void* user) {
    (void)user;
#if TRACKER_ENABLE_STATIC_TEST
    if (g_staticTestRunner.active()) {
        out.println("# ERR stop the active static test before printing a summary");
        return false;
    }
    return g_staticTestRunner.printLastSummary(out);
#else
    out.println("# static test is not compiled in this profile");
    return false;
#endif
}

static bool startRuntimeTestHook(uint32_t durationMs, Stream& out, void* user) {
    (void)user;
#if TRACKER_ENABLE_RUNTIME_TEST
#if TRACKER_ENABLE_STATIC_TEST
    if (g_staticTestRunner.active()) return false;
#endif
    return g_runtimeTestRunner.start(durationMs, millis(), out);
#else
    (void)durationMs;
    return false;
#endif
}

static bool stopRuntimeTestHook(Stream& out, bool force, void* user) {
    (void)user;
#if TRACKER_ENABLE_RUNTIME_TEST
    return g_runtimeTestRunner.stop(out, force);
#else
    (void)out;
    (void)force;
    return false;
#endif
}

static void printRuntimeTestStatus(Stream& out, void* user) {
    (void)user;
#if TRACKER_ENABLE_RUNTIME_TEST
    g_runtimeTestRunner.printStatus(out, millis());
#else
    out.println("# runtime test is not compiled in this profile");
#endif
}

static bool printRuntimeTestReport(Stream& out, void* user) {
    (void)user;
#if TRACKER_ENABLE_RUNTIME_TEST
#if TRACKER_ENABLE_STATIC_TEST
    if (g_staticTestRunner.active()) {
        out.println("# ERR stop the active static test before printing a report");
        return false;
    }
#endif
    if (g_runtimeTestRunner.active()) {
        out.println("# ERR stop the active runtime test before printing a report");
        return false;
    }
    return g_runtimeTestRunner.printLastReport(out);
#else
    out.println("# runtime test is not compiled in this profile");
    return false;
#endif
}

static bool printRuntimeTestSummary(Stream& out, void* user) {
    (void)user;
#if TRACKER_ENABLE_RUNTIME_TEST
    if (g_runtimeTestRunner.active()) {
        out.println("# ERR stop the active runtime test before printing a summary");
        return false;
    }
    return g_runtimeTestRunner.printLastSummary(out);
#else
    out.println("# runtime test is not compiled in this profile");
    return false;
#endif
}

static bool closeCommandSessionHook(TrackerCommandOrigin origin,
                                    uint32_t sessionId,
                                    Stream& out,
                                    void* user) {
    (void)user;
    bool aborted = false;
#if TRACKER_ENABLE_MACHINE_LOG
    if (g_logState.enabled() && g_logState.output == &out &&
        g_logState.ownerOrigin == origin && g_logState.ownerSessionId == sessionId) {
        g_machineLogDeferred.reset(true);
        g_logState.release();
        ++g_logCounters.disconnectAbort;
        aborted = true;
    }
#endif
#if TRACKER_ENABLE_STATIC_TEST
    aborted = g_staticTestRunner.abortOutput(out) || aborted;
#endif
#if TRACKER_ENABLE_RUNTIME_TEST
    aborted = g_runtimeTestRunner.abortOutput(out) || aborted;
#endif
    return aborted;
}


#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
static bool setRemoteConsoleEnabledHook(bool enabled, void* user) {
    (void)user;
    g_wifiRemoteConsole.setEnabled(enabled);
    return true;
}

static void printRemoteConsoleStatusHook(Stream& out, void* user) {
    (void)user;
    g_wifiRemoteConsole.printStatus(out);
}
#endif

static void printConsoleOutputStatusHook(Stream& out, void* user) {
    (void)user;
    out.println("# CONSOLE OUTPUT STATUS");
#if TRACKER_HAS_SERIAL_CONSOLE
    printBoundedDuplexStreamStatus(out, "serial_output", g_serialConsoleStream.status());
    out.print("serial_output_drain_bytes="); out.println(TRACKER_SERIAL_OUTPUT_BYTES_PER_DRAIN);
    out.print("serial_output_drain_interval_ms="); out.println(TRACKER_SERIAL_OUTPUT_DRAIN_INTERVAL_MS);
    out.print("serial_output_current_backoff_ms="); out.println(g_serialOutputDrainBackoffMs);
    out.print("serial_output_stale_discard_ms="); out.println(TRACKER_SERIAL_OUTPUT_STALE_DISCARD_MS);
    out.print("serial_output_stale_discards="); out.println(g_serialOutputStaleDiscards);
#else
    out.println("serial_output_compiled=no");
#endif
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    const WifiRemoteConsoleStatus remote = g_wifiRemoteConsole.status();
    printBoundedDuplexStreamStatus(out, "remote_console_output", remote.output);
    out.print("remote_console_output_drain_bytes="); out.println(TRACKER_REMOTE_CONSOLE_OUTPUT_BYTES_PER_DRAIN);
    out.print("remote_console_output_drain_interval_ms="); out.println(TRACKER_REMOTE_CONSOLE_OUTPUT_DRAIN_INTERVAL_MS);
#else
    out.println("remote_console_output_compiled=no");
#endif
}

static void resetConsoleOutputStateHook(void* user) {
    (void)user;
#if TRACKER_HAS_SERIAL_CONSOLE
    g_serialConsoleStream.resetOutputState();
    g_lastSerialOutputDrainMs = 0u;
    g_serialOutputDrainBackoffMs = TRACKER_SERIAL_OUTPUT_DRAIN_INTERVAL_MS;
    g_serialOutputStallStartMs = 0u;
    g_serialOutputStaleDiscards = 0u;
#endif
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    g_wifiRemoteConsole.resetOutputState();
#endif
}

static bool fitGyroTempFromLastStaticHook(bool persist, Stream& out, void* user);
static bool fitGyroTempFromCaptureHook(const StaticRuntimeTest* capture, bool persist, Stream& out, void* user);
static bool fitGyroTempFromCaptureRamHook(const StaticRuntimeTest* capture, Stream& out, void* user);

static bool serviceNonCliRuntimeHook(void* user) {
    (void)user;
    g_app.serviceRuntimeForBlockingCommand();
    return true;
}

static TrackerCommandRuntimeObjects makeTrackerCommandRuntimeObjects() {
    TrackerCommandRuntimeObjects objects;
    objects.io = &appConsoleOutput();
    objects.config = &g_config;
    objects.configStore = &g_configStore;
    objects.networkConfig = &g_networkConfig;
    objects.networkConfigStore = &g_networkConfigStore;
    objects.networkConfigLoadedFromNvs = &g_networkConfigLoadedFromNvs;
    objects.wifiManager = &g_wifiManager;
    objects.slimevrRuntime = &g_slimevrRuntime;
    objects.health = &g_trackerHealth;
#if TRACKER_ENABLE_TAP_RUNTIME
    objects.tapRuntime = &g_tapRuntime;
#endif
#if TRACKER_ENABLE_STATUS_LED
    objects.statusLedRuntime = &g_statusLedRuntime;
#endif
#if TRACKER_ENABLE_BATTERY_RUNTIME
    objects.batteryRuntime = &g_batteryRuntime;
    objects.batteryAdcBatchSampler = &g_batteryAdcBatchSampler;
#endif
    objects.lsm = &lsm;
    objects.fifo = &lsmFifo;
    objects.sensorHub = &lsmHub;
    objects.mag = &qmc;
    objects.imuCal = &g_imuCal;
    objects.gyroTempComp = &g_gyroTempComp;
    objects.quality = &g_quality;
    objects.ahrs = &g_ahrs6dof;
    objects.runtimeBias = &g_runtimeBias;
#if TRACKER_HAS_RUNTIME_PROFILER
    objects.runtimeProfiler = &g_runtimeProfiler;
    objects.motionDiagnostics = &g_motionDiagnostics;
#endif
    objects.fifoRuntime = &g_fifoRuntime;
#if TRACKER_ENABLE_CALIBRATION_CANDIDATES
    objects.magAxisAlignmentState = &g_magAxisAlignmentState;
#endif
    objects.trackingState = &g_trackingState;
    objects.preparedOutput = &g_preparedOutput;
#if TRACKER_HAS_CALIBRATION_AUTONOMY
    objects.calibrationAutonomy = &g_calibrationAutonomy;
#endif
#if TRACKER_ENABLE_CALIBRATION_COMMANDS
    objects.calibrationIo = &g_calIo;
    objects.accelCalRunner = &g_accelCalRunner;
    objects.gyroTempCapture = &g_gyroTempCapture;
#endif
    objects.lastMagProcessed = &g_lastMagProcessed;
    objects.lastScaledSample = &g_lastScaledSample;
    objects.lastCalibratedSample = &g_lastCalibratedSample;
    objects.lastImuSampleSequence = &g_lastImuSampleSequence;
    objects.streamState = &g_streamState;
#if TRACKER_ENABLE_MACHINE_LOG
    objects.logState = &g_logState;
#endif
    return objects;
}

static TrackerCommandRuntimeHooks makeTrackerCommandRuntimeHooks() {
    TrackerCommandRuntimeHooks hooks;
    hooks.resetFifoRuntime = hookResetFifoRuntime;
    hooks.requestTrackingRecovery = hookRequestTrackingRecovery;
    hooks.resetAhrsRuntime = hookResetAhrsRuntime;
#if TRACKER_ENABLE_DETAILED_RUNTIME_STATUS
    hooks.printRuntimeStatus = printRuntimeStatus;
    hooks.printRuntimeHealth = printRuntimeHealth;
#endif
    hooks.setSpiFrequency = setRuntimeSpiFrequency;
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    hooks.setRemoteConsoleEnabled = setRemoteConsoleEnabledHook;
    hooks.printRemoteConsoleStatus = printRemoteConsoleStatusHook;
#endif
    hooks.printConsoleOutputStatus = printConsoleOutputStatusHook;
    hooks.resetConsoleOutputState = resetConsoleOutputStateHook;
#if TRACKER_ENABLE_MACHINE_LOG
    hooks.emitLogHeader = emitMachineLogHeader;
    hooks.printLogSummary = printLogSummary;
    hooks.resetLogCounters = resetLogCountersHook;
    hooks.resetLogPipeline = resetLogPipelineHook;
#endif
    hooks.printRuntimeGyroBiasStatus = printRuntimeGyroBiasStatus;
    hooks.setRuntimeGyroBiasEnabled = setRuntimeGyroBiasEnabled;
    hooks.resetRuntimeGyroBiasEstimator = resetRuntimeGyroBiasEstimator;
#if TRACKER_ENABLE_STATIC_TEST
    hooks.startStaticTest = startStaticTestHook;
    hooks.stopStaticTest = stopStaticTestHook;
    hooks.printStaticTestStatus = printStaticTestStatus;
    hooks.printStaticTestSummary = printStaticTestSummary;
    hooks.printStaticTestReport = printStaticTestReport;
#endif
#if TRACKER_ENABLE_RUNTIME_TEST
    hooks.startRuntimeTest = startRuntimeTestHook;
    hooks.stopRuntimeTest = stopRuntimeTestHook;
    hooks.printRuntimeTestStatus = printRuntimeTestStatus;
    hooks.printRuntimeTestSummary = printRuntimeTestSummary;
    hooks.printRuntimeTestReport = printRuntimeTestReport;
#endif
    hooks.closeCommandSession = closeCommandSessionHook;
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
#if TRACKER_ENABLE_STATIC_TEST
    hooks.fitGyroTempFromLastStatic = fitGyroTempFromLastStaticHook;
    hooks.fitGyroTempFromCapture = fitGyroTempFromCaptureHook;
#endif
#if TRACKER_HAS_GYRO_TEMP_FIT
    hooks.fitGyroTempFromCaptureRam = fitGyroTempFromCaptureRamHook;
#endif
    hooks.serviceNonCliRuntime = serviceNonCliRuntimeHook;
#if TRACKER_HAS_MOTION_LIGHT_SLEEP
    hooks.requestMotionLightSleep = requestMotionLightSleepHook;
#endif
    return hooks;
}

static void setupCommandInterface() {
#if TRACKER_ENABLE_STATIC_TEST
    setupStaticTestRunner();
#endif
#if TRACKER_ENABLE_RUNTIME_TEST
    setupRuntimeTestRunner();
#endif
#if TRACKER_ENABLE_SERIAL_CLI
#if TRACKER_ENABLE_MACHINE_LOG
    g_machineLogDeferred.begin(&g_logState,
                               &g_logCounters,
                               &g_lastBiasLogEmitUs,
                               MACHINE_BIAS_LOG_PERIOD_US);
#endif
    const TrackerCommandRuntimeObjects commandObjects = makeTrackerCommandRuntimeObjects();
    const TrackerCommandRuntimeHooks commandHooks = makeTrackerCommandRuntimeHooks();
    wireTrackerCommandContext(g_cmdCtx, commandObjects, commandHooks);
    g_cli.begin(g_cmdCtx);
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    TrackerSerialCommandContext remoteBaseCtx;
    wireTrackerCommandContext(remoteBaseCtx, commandObjects, commandHooks);
    g_wifiRemoteConsole.begin(remoteBaseCtx);
#endif
#endif
}
