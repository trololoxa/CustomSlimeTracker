#pragma once

#include "runtime/tracker_console_suppress.hpp"

// Magnetometer and tracking-state hooks used by the app composition layer.
// This file is included by app/tracker_app_hooks.hpp after shared app dependencies.

#if TRACKER_HAS_MACHINE_LOG
static void emitMachineLogMagFrame(const MagProcessedSample& mag,
                                   const MagHeadingSample& heading,
                                   const MagYawCorrectionOutput& yaw,
                                   uint32_t rejectFlagsForUse,
                                   bool trustedForUse);
#endif

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

#if TRACKER_HAS_MACHINE_LOG
static void magControllerEmitMachineLogMagFrameCallback(const MagProcessedSample& mag,
                                                        const MagHeadingSample& heading,
                                                        const MagYawCorrectionOutput& yaw,
                                                        uint32_t rejectFlagsForUse,
                                                        bool trustedForUse,
                                                        void* user) {
    (void)user;
    emitMachineLogMagFrame(mag, heading, yaw, rejectFlagsForUse, trustedForUse);
}
#endif

static void magControllerRecordStaticMagYawSampleCallback(float magHeadingErrorDeg,
                                                          const MagHeadingSample& heading,
                                                          const MagYawCorrectionOutput& yaw,
                                                          void* user) {
    (void)user;
#if TRACKER_ENABLE_STATIC_TEST
    g_staticTestRunner.recordMagYawSample(magHeadingErrorDeg, heading, yaw);
#else
    (void)magHeadingErrorDeg;
    (void)heading;
    (void)yaw;
#endif
}

static MagRuntimeControllerDeps makeMagRuntimeControllerDeps() {
    MagRuntimeControllerCallbacks callbacks;
    callbacks.resetFifoRuntime = magControllerResetFifoRuntimeCallback;
    callbacks.emitStateEvent = magControllerEmitStateEventCallback;
#if TRACKER_HAS_MACHINE_LOG
    callbacks.emitMagFrame = magControllerEmitMachineLogMagFrameCallback;
#endif
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
    sink.out = trackerConsoleTrackingMessagesSuppressed(millis()) ? nullptr : &Serial;
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

static TrackingStateInputs makeTrackingStateInputs() {
    TrackingStateInputs in;
    in.accelCalValid = g_imuCal.accelCalValid;
    in.gyroBiasValid = g_imuCal.gyroBiasValid;
    in.ahrsInitialized = g_ahrs6dof.initialized();
    in.qualityRecoveryRequested = g_quality.recoveryRequested();
    // Use the latest per-sample quality flags for ordinary DEGRADED_*
    // states.  lastRecoveryFlags() only contains recovery reasons and would
    // otherwise hide non-recovery accel/timing degradation from setup/status.
    in.qualityFlags = g_lastQualityFlags | g_quality.lastRecoveryFlags();
    in.magRuntimeEnabled = g_magState.runtimeEnabled;
    in.magSampleSeen = g_lastMagProcessed.seq != 0 || g_magProcessor.stats().processedSamples != 0;
    in.magTrusted = g_lastMagProcessed.trusted;
    in.magRejectFlags = g_lastMagProcessed.rejectFlags;
    in.magHeadingReferenceValid = g_magHeadingRef.valid;
    in.magYawControllerEnabled = g_config.data.magYaw.controllerEnabled;
    in.magYawApplied = g_lastMagYawCorrection.applied;
    in.magYawRejectFlags = g_lastMagYawCorrection.rejectFlags;
    return in;
}

static const char* trackingStateName() {
    return g_trackingState.stateName(makeTrackingStateInputs());
}

static bool setMagRuntimeEnabledHook(bool enabled, bool persist, void* user) {
    (void)user;
    return g_magRuntime.setEnabled(enabled, persist);
}

static void processOneMagRawSample(const Lsm6dsvFifoReader::MagRawSample& mag) {
    g_magRuntime.processRawSample(mag);
}

#if TRACKER_ENABLE_DETAILED_MAG_STATUS
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
#else
static void printMagRuntimeStatus(Stream& out, void* user) {
    (void)user;
    out.println("# detailed mag runtime status is not compiled in this profile");
    out.print("mag_config_enabled="); out.println(g_config.data.magCal.driverEnabled ? "yes" : "no");
    out.print("mag_runtime_enabled="); out.println(g_magState.runtimeEnabled ? "yes" : "no");
    out.print("mag_samples_processed="); out.println(g_magProcessor.stats().processedSamples);
    out.print("mag_last_trusted="); out.println(g_lastMagProcessed.trusted ? "yes" : "no");
}

static void printMagProcessedStatus(Stream& out, void* user) {
    (void)user;
    out.println("# compact mag processed status");
    out.print("seq="); out.println(g_lastMagProcessed.seq);
    out.print("trusted="); out.println(g_lastMagProcessed.trusted ? "yes" : "no");
    out.print("reject_flags=0x"); out.println(g_lastMagProcessed.rejectFlags, HEX);
}

static void printMagHeadingStatus(Stream& out, void* user) {
    (void)user;
    out.println("# compact mag heading status");
    out.print("heading_ref_valid="); out.println(g_magHeadingRef.valid ? "yes" : "no");
    out.print("auto_ref_enabled="); out.println(g_magHeadingAutoRef.enabled ? "yes" : "no");
}

static void printMagYawCorrectionStatus(Stream& out, void* user) {
    (void)user;
    out.println("# compact mag yaw status");
    out.print("controller_enabled="); out.println(g_config.data.magYaw.controllerEnabled ? "yes" : "no");
    out.print("apply_enabled="); out.println(g_config.data.magYaw.applyEnabled ? "yes" : "no");
    out.print("last_applied="); out.println(g_lastMagYawCorrection.applied ? "yes" : "no");
    out.print("last_reject_flags=0x"); out.println(g_lastMagYawCorrection.rejectFlags, HEX);
}

static void printMagCalibrationStatus(Stream& out, void* user) {
    (void)user;
    out.println("# compact mag calibration status");
    out.print("driver_enabled="); out.println(g_config.data.magCal.driverEnabled ? "yes" : "no");
    out.print("calibration_valid="); out.println(g_config.data.magCal.calibrationValid ? "yes" : "no");
    out.print("axis_alignment_valid="); out.println(g_config.data.magCal.axisAlignmentValid ? "yes" : "no");
    out.print("collected_samples="); out.println(g_magCalCollector.samples());
}
#endif

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

#if TRACKER_ENABLE_DETAILED_MAG_STATUS
static void printMagCalibrationStatus(Stream& out, void* user) {
    (void)user;
    magStatusPrintCalibration(out, makeMagStatusReporterDeps());
}
#endif

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
