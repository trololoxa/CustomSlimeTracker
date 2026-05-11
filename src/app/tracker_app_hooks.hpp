#pragma once

// App hooks: dependency builders and callback glue that connect command/runtime
// modules to the hardware and runtime contexts. This is the composition edge of
// the firmware; domain logic should live in runtime/, serial/, config/ or sensor/.

#include <Arduino.h>

#include "app/tracker_hardware_context.hpp"
#include "app/tracker_runtime_context.hpp"
#include "runtime/runtime_gyro_bias_controller.hpp"
#include "runtime/machine_log_runtime.hpp"
#include "runtime/imu_sample_pipeline.hpp"
#include "runtime/mag_status_reporter.hpp"
#include "runtime/runtime_status_reporter.hpp"
#include "runtime/gyro_temp_static_fit.hpp"
#include "app/tracker_command_wiring.hpp"
#include "app/tracker_bootstrap.hpp"

using namespace tracker;

static void recordFifoProcessTime(uint32_t dtUs) {
    g_perf.fifoProcessCalls++;
    g_perf.fifoProcessSumUs += dtUs;
    if (dtUs > g_perf.fifoProcessMaxUs) {
        g_perf.fifoProcessMaxUs = dtUs;
    }
}

static void resetLogCountersHook(void* user) {
    (void)user;
    machineLogResetCounters(g_logCounters, g_lastBiasLogEmitUs);
}

static void emitMachineLogHeader(Stream& out, void* user) {
    (void)user;
    machineLogEmitHeader(out, g_logState, g_config);
}

static void emitLogStateEvent(const char* state, const char* reason, uint64_t tUs, uint32_t flags, float confidence) {
    machineLogEmitStateEvent(Serial, g_logState, g_logCounters, state, reason, tUs, flags, confidence);
}

static void resetFifoRuntimeCounters() {
    noInterrupts();
    g_fifoIntCount = 0;
    g_fifoLastIrqUs = micros();
    interrupts();

    g_fifoEvents.reset();
    g_lastSampleTimestampUs = 0;
}

static bool consumeFifoInterruptEvent(uint32_t timeoutMs) {
    return g_fifoEvents.consume(timeoutMs, g_config.data.fifo.watermarkWords);
}

static bool waitFifoEventForCalibration(uint32_t timeoutMs, void* user) {
    (void)user;
    return consumeFifoInterruptEvent(timeoutMs);
}

static TrackerBootstrapDeps makeTrackerBootstrapDeps() {
    TrackerBootstrapDeps deps;
    deps.out = &Serial;
    deps.spi = &SPI;
    deps.lsmBus = &lsmBus;
    deps.lsm = &lsm;
    deps.fifo = &lsmFifo;
    deps.config = &g_config;
    deps.configStore = &g_configStore;
    deps.configLoadedFromNvs = &g_configLoadedFromNvs;
    deps.imuCal = &g_imuCal;
    deps.gyroTempComp = &g_gyroTempComp;
    deps.quality = &g_quality;
    deps.ahrs = &g_ahrs6dof;
    deps.streamState = &g_streamState;
    deps.calibrationIo = &g_calIo;
    deps.calibrationRawBuffer = g_fifoRaw;
    deps.calibrationRawBufferCapacity = FIFO_RAW_BUFFER_CAPACITY;
    deps.waitForCalibrationFifoEvent = waitFifoEventForCalibration;
    deps.waitForCalibrationFifoEventUser = nullptr;
    deps.latestTempC = g_latestTempC;
    deps.pins.sck = PIN_LSM_SCK;
    deps.pins.miso = PIN_LSM_MISO;
    deps.pins.mosi = PIN_LSM_MOSI;
    deps.pins.cs = PIN_LSM_CS;
    deps.pins.int1 = PIN_LSM_INT1;
    deps.magHubPeriodUs = MAG_HUB_PERIOD_US;
    return deps;
}

static bool setRuntimeSpiFrequency(uint32_t hz, void* user) {
    (void)user;
    return trackerBootstrapSetRuntimeSpiFrequency(g_config, lsmBus, hz);
}

static Vec3 currentGyroBiasRadS(float tempC) {
    return runtimeBiasCurrentGyroBiasRadS(g_runtimeBias, g_imuCal, g_gyroTempComp, tempC);
}

static uint32_t gyroBiasRuntimeFlags(float tempC) {
    return runtimeBiasGyroBiasRuntimeFlags(g_runtimeBias, g_gyroTempComp, tempC);
}

static void printRuntimeGyroBiasStatus(Stream& out, void* user) {
    (void)user;
    runtimeBiasPrintStatus(out, g_runtimeBias, g_imuCal, g_gyroTempComp, g_latestTempC);
}

static bool setRuntimeGyroBiasEnabled(bool enabled, void* user) {
    (void)user;
    runtimeBiasSetEnabled(g_runtimeBias, enabled);
    return true;
}

static void resetRuntimeGyroBiasEstimator(void* user) {
    (void)user;
    runtimeBiasReset(g_runtimeBias);
}

// ============================================================
// Config / init
// ============================================================

static void emitMachineLogMagFrame(const MagProcessedSample& mag,
                                   const MagHeadingSample& heading,
                                   const MagYawCorrectionOutput& yaw,
                                   uint32_t rejectFlagsForUse,
                                   bool trustedForUse);

static void magControllerResetFifoRuntimeCallback(void* user) {
    (void)user;
    resetFifoRuntimeCounters();
}

static bool magControllerRecoveryActiveCallback(void* user) {
    (void)user;
    return g_trackingState.recoveryActive();
}

static void magControllerEmitStateEventCallback(const char* state,
                                                const char* reason,
                                                uint64_t timestampUs,
                                                uint32_t flags,
                                                float confidence,
                                                void* user) {
    (void)user;
    emitLogStateEvent(state, reason, timestampUs, flags, confidence);
}

static void magControllerEmitMachineLogMagFrameCallback(const MagProcessedSample& mag,
                                                        const MagHeadingSample& heading,
                                                        const MagYawCorrectionOutput& yaw,
                                                        uint32_t rejectFlagsForUse,
                                                        bool trustedForUse,
                                                        void* user) {
    (void)user;
    emitMachineLogMagFrame(mag, heading, yaw, rejectFlagsForUse, trustedForUse);
}

static void magControllerRecordStaticMagYawSampleCallback(float magHeadingErrorDeg,
                                                          const MagHeadingSample& heading,
                                                          const MagYawCorrectionOutput& yaw,
                                                          void* user) {
    (void)user;
    g_staticTestRunner.recordMagYawSample(magHeadingErrorDeg, heading, yaw);
}

static MagRuntimeControllerDeps makeMagRuntimeControllerDeps() {
    MagRuntimeControllerCallbacks callbacks;
    callbacks.resetFifoRuntime = magControllerResetFifoRuntimeCallback;
    callbacks.emitStateEvent = magControllerEmitStateEventCallback;
    callbacks.emitMagFrame = magControllerEmitMachineLogMagFrameCallback;
    callbacks.recordStaticMagYawSample = magControllerRecordStaticMagYawSampleCallback;

    MagRuntimeControllerDeps deps;
    deps.out = &Serial;
    deps.config = &g_config;
    deps.configStore = &g_configStore;
    deps.hub = &lsmHub;
    deps.qmc = &qmc;
    deps.fifo = &lsmFifo;
    deps.quality = &g_quality;
    deps.ahrs = &g_ahrs6dof;
    deps.state = &g_magState;
    deps.processor = &g_magProcessor;
    deps.calibrationCollector = &g_magCalCollector;
    deps.headingEstimator = &g_magHeading;
    deps.headingRef = &g_magHeadingRef;
    deps.headingAutoRef = &g_magHeadingAutoRef;
    deps.yawCorrection = &g_magYawCorrection;
    deps.lastProcessed = &g_lastMagProcessed;
    deps.lastHeading = &g_lastMagHeading;
    deps.lastYawCorrection = &g_lastMagYawCorrection;
    deps.lastOutputConfidence = &g_lastOutputConfidence;
    deps.accelCalibrationReady = &g_imuCal.accelCalValid;
    deps.fallbackTimestampUs = &g_lastSampleTimestampUs;
    deps.recoveryActive = magControllerRecoveryActiveCallback;
    deps.magHubPeriodUs = MAG_HUB_PERIOD_US;
    deps.callbacks = callbacks;
    return deps;
}

static void setupMagRuntimeController() {
    g_magRuntime.begin(makeMagRuntimeControllerDeps());
}

static MagYawCorrectionConfig makeMagYawCorrectionConfig() {
    return g_magRuntime.yawConfig();
}

static MagRuntimeConfig makeMagRuntimeConfig() {
    return g_magRuntime.runtimeConfig();
}

static float magHeadingErrorToReferenceDeg(const MagHeadingSample& heading) {
    return g_magRuntime.headingErrorToReferenceDeg(heading);
}

static void resetOrientationDependentState(const char* reason, uint64_t timestampUs, bool rebaseAhrsTimebase) {
    g_magRuntime.resetOrientationState(reason, timestampUs, rebaseAhrsTimebase);
}

static void trackingResetOrientationCallback(const char* reason,
                                             uint64_t timestampUs,
                                             bool rebaseAhrsTimebase,
                                             void* user) {
    (void)user;
    resetOrientationDependentState(reason, timestampUs, rebaseAhrsTimebase);
}

static void trackingEmitStateEventCallback(const char* state,
                                           const char* reason,
                                           uint64_t timestampUs,
                                           uint32_t flags,
                                           float confidence,
                                           void* user) {
    (void)user;
    emitLogStateEvent(state, reason, timestampUs, flags, confidence);
}

static TrackingStateEventSink makeTrackingEventSink() {
    TrackingStateEventSink sink;
    sink.out = &Serial;
    sink.confidence = g_lastOutputConfidence;
    sink.resetOrientation = trackingResetOrientationCallback;
    sink.resetOrientationUser = nullptr;
    sink.emitStateEvent = trackingEmitStateEventCallback;
    sink.emitStateEventUser = nullptr;
    return sink;
}

static void enterTrackingRecovery(uint32_t reasonFlags, const char* reason, uint64_t timestampUs) {
    g_trackingState.enterRecovery(reasonFlags, reason, timestampUs, makeTrackingEventSink());
}

static void updateTrackingRecoveryState(const ImuQualityResult& quality) {
    g_trackingState.updateRecovery(quality, g_lastSampleTimestampUs, makeTrackingEventSink());
}

static const char* trackingStateName() {
    return g_trackingState.stateName(g_imuCal.accelCalValid,
                                     g_imuCal.gyroBiasValid,
                                     g_quality.recoveryRequested(),
                                     g_ahrs6dof.initialized(),
                                     g_magHeadingRef.valid,
                                     g_lastMagYawCorrection.applied);
}

static bool setMagRuntimeEnabledHook(bool enabled, bool persist, void* user) {
    (void)user;
    return g_magRuntime.setEnabled(enabled, persist);
}

static void processOneMagRawSample(const Lsm6dsvFifoReader::MagRawSample& mag) {
    g_magRuntime.processRawSample(mag);
}

static MagStatusReporterDeps makeMagStatusReporterDeps() {
    MagStatusReporterDeps deps;
    deps.config = &g_config;
    deps.qmc = &qmc;
    deps.hub = &lsmHub;
    deps.state = &g_magState;
    deps.processor = &g_magProcessor;
    deps.lastProcessed = &g_lastMagProcessed;
    deps.headingEstimator = &g_magHeading;
    deps.lastHeading = &g_lastMagHeading;
    deps.headingRef = &g_magHeadingRef;
    deps.headingAutoRef = &g_magHeadingAutoRef;
    deps.yawCorrection = &g_magYawCorrection;
    deps.lastYawCorrection = &g_lastMagYawCorrection;
    deps.calibrationCollector = &g_magCalCollector;
    deps.runtimeConfig = makeMagRuntimeConfig();
    deps.yawConfig = makeMagYawCorrectionConfig();
    return deps;
}

static void printMagRuntimeStatus(Stream& out, void* user) {
    (void)user;
    magStatusPrintRuntime(out, makeMagStatusReporterDeps());
}

static void printMagProcessedStatus(Stream& out, void* user) {
    (void)user;
    magStatusPrintProcessed(out, makeMagStatusReporterDeps());
}

static void printMagHeadingStatus(Stream& out, void* user) {
    (void)user;
    magStatusPrintHeading(out, makeMagStatusReporterDeps());
}

static void printMagYawCorrectionStatus(Stream& out, void* user) {
    (void)user;
    magStatusPrintYawCorrection(out, makeMagStatusReporterDeps());
}

static void resetMagYawCorrectionHook(void* user) {
    (void)user;
    g_magRuntime.resetYawCorrectionRuntime();
}

static bool setMagYawCorrectionApplyEnabledHook(bool enabled, bool persist, void* user) {
    (void)user;
    return g_magRuntime.setYawCorrectionApplyEnabled(enabled, persist);
}

static bool setMagHeadingReferenceHook(void* user) {
    (void)user;
    return g_magRuntime.setHeadingReference("manual", true);
}

static void clearMagHeadingReferenceHook(void* user) {
    (void)user;
    g_magRuntime.clearHeadingReference();
}

static bool setMagHeadingAutoReferenceEnabledHook(bool enabled, void* user) {
    (void)user;
    return g_magRuntime.setAutoReferenceEnabled(enabled);
}

static void printMagCalibrationStatus(Stream& out, void* user) {
    (void)user;
    magStatusPrintCalibration(out, makeMagStatusReporterDeps());
}

static bool startMagCalibrationHook(void* user) {
    (void)user;
    return g_magRuntime.startCalibration();
}

static void stopMagCalibrationHook(void* user) {
    (void)user;
    g_magRuntime.stopCalibration();
}

static void resetMagCalibrationHook(void* user) {
    (void)user;
    g_magRuntime.resetCalibration();
}

static bool applyMagCalibrationHook(bool persist, void* user) {
    (void)user;
    return g_magRuntime.applyCalibration(persist);
}

// ============================================================
// Command hooks
// ============================================================

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

// ============================================================
// Runtime processing
// ============================================================

static void maybeRecoverFifo(const ImuQualityResult& quality, const Lsm6dsv::RawSample& raw) {
    if (!quality.shouldRequestFifoRecovery) return;

    const uint32_t nowMs = millis();
    if (!g_trackingState.recoveryActive() || nowMs - g_lastRecoveryConsolePrintMs >= RECOVERY_CONSOLE_THROTTLE_MS) {
        g_lastRecoveryConsolePrintMs = nowMs;
        Serial.print("# WARN FIFO recovery requested quality_flags=0x");
        Serial.println(quality.flags, HEX);
    }

    const uint64_t ts = raw.t_us != 0 ? raw.t_us : lsmFifo.stats().lastAssignedTimestampUs;
    enterTrackingRecovery(quality.flags, "fifo_recovery", ts);
    lsmFifo.resetFifo();
    lsmFifo.resetTimestampReconstruction(ts);
    g_quality.clearRecoveryRequest();
    g_quality.syncFifoStats(lsmFifo.stats());
    resetFifoRuntimeCounters();
}

static GyroTempStaticFitDeps makeGyroTempStaticFitDeps() {
    GyroTempStaticFitDeps deps;
    deps.lastCompletedStaticTest = &g_lastCompletedStaticTest;
    deps.lastCompletedStaticTestValid = g_lastCompletedStaticTestValid;
    deps.gyroTempComp = &g_gyroTempComp;
    deps.imuCal = &g_imuCal;
    deps.runtimeBias = &g_runtimeBias;
    deps.config = &g_config;
    deps.configStore = &g_configStore;
    return deps;
}

static bool fitGyroTempFromLastStaticHook(bool persist, Stream& out, void* user) {
    (void)user;
    GyroTempStaticFitDeps deps = makeGyroTempStaticFitDeps();
    return fitGyroTempFromLastStatic(deps, persist, out);
}

static void pipelineEnterTrackingRecoveryCallback(uint32_t reasonFlags,
                                                  const char* reason,
                                                  uint64_t timestampUs,
                                                  void* user) {
    (void)user;
    enterTrackingRecovery(reasonFlags, reason, timestampUs);
}

static void pipelineUpdateTrackingRecoveryCallback(const ImuQualityResult& quality, void* user) {
    (void)user;
    updateTrackingRecoveryState(quality);
}

static void pipelineEmitMachineLogFrameCallback(const Lsm6dsv::RawSample& raw,
                                                const Lsm6dsv::Sample& calibrated,
                                                const ImuQualityResult& quality,
                                                void* user) {
    (void)user;
    emitMachineLogFrame(raw, calibrated, quality);
}

static void pipelineMaybeRecoverFifoCallback(const ImuQualityResult& quality,
                                             const Lsm6dsv::RawSample& raw,
                                             void* user) {
    (void)user;
    maybeRecoverFifo(quality, raw);
}

static ImuSamplePipelineDeps makeImuSamplePipelineDeps() {
    ImuSamplePipelineCallbacks callbacks;
    callbacks.enterTrackingRecovery = pipelineEnterTrackingRecoveryCallback;
    callbacks.updateTrackingRecoveryState = pipelineUpdateTrackingRecoveryCallback;
    callbacks.emitMachineLogFrame = pipelineEmitMachineLogFrameCallback;
    callbacks.maybeRecoverFifo = pipelineMaybeRecoverFifoCallback;
    callbacks.user = nullptr;

    ImuSamplePipelineDeps deps{
        lsm,
        lsmFifo,
        g_config,
        g_imuCal,
        g_gyroTempComp,
        g_quality,
        g_ahrs6dof,
        g_runtimeBias,
        g_trackingState,
        g_preparedOutput,
        g_streamState,
        g_logState,
        g_logCounters,
        g_staticTestRunner,
        g_perf,
        &g_calIo,
        Serial,
        g_runtimeSamples,
        g_lastSampleTimestampUs,
        g_latestTempC,
        g_lastOutputConfidence,
        callbacks
    };
    return deps;
}

static FifoRuntimeSampleResult processRuntimeRawSampleCallback(const Lsm6dsv::RawSample& raw,
                                                               bool checkFifoStatsDelta,
                                                               void* user) {
    (void)user;
    ImuSamplePipelineDeps deps = makeImuSamplePipelineDeps();
    return imuSamplePipelineProcessRaw(deps, raw, checkFifoStatsDelta);
}

static void processRuntimeMagSampleCallback(const Lsm6dsvFifoReader::MagRawSample& mag, void* user) {
    (void)user;
    processOneMagRawSample(mag);
}

static void recordRuntimeFifoProcessTime(uint32_t processUs, void* user) {
    (void)user;
    recordFifoProcessTime(processUs);
    g_staticTestRunner.recordFifoProcessTime(processUs);
}

static void appAttachFifoInterruptCallback() {
    attachInterrupt(digitalPinToInterrupt(PIN_LSM_INT1), onFifoInt1, RISING);
}

static bool appSetMagRuntimeEnabledCallback(bool enabled, bool persist) {
    return setMagRuntimeEnabledHook(enabled, persist, nullptr);
}

static TrackerAppDeps makeTrackerAppDeps() {
    TrackerAppDeps deps;

    deps.bootstrap = makeTrackerBootstrapDeps();

    deps.runtime.out = &Serial;
    deps.runtime.config = &g_config;
    deps.runtime.configLoadedFromNvs = &g_configLoadedFromNvs;
    deps.runtime.fifo = &lsmFifo;
    deps.runtime.quality = &g_quality;
    deps.runtime.ahrs = &g_ahrs6dof;
    deps.runtime.cli = &g_cli;
    deps.runtime.streamState = &g_streamState;
    deps.runtime.perf = &g_perf;
    deps.runtime.fifoEvents = &g_fifoEvents;
    deps.runtime.fifoRuntime = &g_fifoRuntime;
    deps.runtime.staticTestRunner = &g_staticTestRunner;
    deps.runtime.magState = &g_magState;
    deps.runtime.fifoIntCount = &g_fifoIntCount;
    deps.runtime.runtimeSamples = &g_runtimeSamples;
    deps.runtime.lastHeartbeatMs = &g_lastHeartbeatMs;
    deps.runtime.latestTempC = &g_latestTempC;

    deps.callbacks.setupMagRuntimeController = setupMagRuntimeController;
    deps.callbacks.setupCommandInterface = setupCommandInterface;
    deps.callbacks.resetFifoRuntimeCounters = resetFifoRuntimeCounters;
    deps.callbacks.attachFifoInterrupt = appAttachFifoInterruptCallback;
    deps.callbacks.resetOrientationState = resetOrientationDependentState;
    deps.callbacks.setMagRuntimeEnabled = appSetMagRuntimeEnabledCallback;
    deps.callbacks.processRawSample = processRuntimeRawSampleCallback;
    deps.callbacks.processMagSample = processRuntimeMagSampleCallback;
    deps.callbacks.recordFifoProcessTime = recordRuntimeFifoProcessTime;
    deps.callbacks.fifoCallbackUser = nullptr;

    deps.buffers.fifoRaw = g_fifoRaw;
    deps.buffers.fifoRawCapacity = FIFO_RAW_BUFFER_CAPACITY;
    deps.buffers.magRaw = g_magRaw;
    deps.buffers.magRawCapacity = MAG_RAW_BUFFER_CAPACITY;

    deps.pins.int1 = PIN_LSM_INT1;

    deps.timing.serialBaud = SERIAL_BAUD_DEFAULT;
    deps.timing.fifoMaxWordsPerDrainDefault = FIFO_MAX_WORDS_PER_DRAIN_DEFAULT;
    deps.timing.fifoMaxDrainRoundsPerEventDefault = MAX_DRAIN_ROUNDS_PER_EVENT_DEFAULT;
    deps.timing.fifoNonblockingStatusPollIntervalUs = FIFO_NONBLOCKING_STATUS_POLL_INTERVAL_US;

    return deps;
}
