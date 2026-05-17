#pragma once

// Runtime sample-processing hooks and final TrackerAppDeps wiring.
// This file is included by app/tracker_app_hooks.hpp after shared app dependencies.

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

static bool fitGyroTempFromCaptureHook(const StaticRuntimeTest* capture, bool persist, Stream& out, void* user) {
    (void)user;
    if (!capture) {
        out.println("# ERR gyro temp capture is not available");
        return false;
    }
    GyroTempStaticFitDeps deps = makeGyroTempStaticFitDeps();
    return fitGyroTempFromCompletedStaticTest(deps, *capture, persist, out);
}

static bool fitGyroTempFromCaptureRamHook(const StaticRuntimeTest* capture, Stream& out, void* user) {
    (void)user;
    if (!capture) {
        out.println("# ERR gyro temp capture is not available");
        return false;
    }
    GyroTempStaticFitDeps deps = makeGyroTempStaticFitDeps();
    return fitGyroTempFromCompletedStaticTestEx(
        deps,
        *capture,
        GyroTempStaticFitMode::ApplyRam,
        out
    );
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
        &g_gyroTempCapture,
        g_perf,
        &g_calIo,
        Serial,
        g_runtimeSamples,
        g_lastSampleTimestampUs,
        g_latestTempC,
        g_lastOutputConfidence,
        callbacks
    };
    deps.lastScaledSample = &g_lastScaledSample;
    deps.lastCalibratedSample = &g_lastCalibratedSample;
    deps.lastImuSampleSequence = &g_lastImuSampleSequence;
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


static TrackerWifiManagerConfig makeAppWifiManagerConfig() {
    TrackerWifiManagerConfig cfg;
    cfg.enabled = g_networkConfig.data.wifiEnabled;
    cfg.credentialsValid = g_networkConfig.data.credentialsValid;
    cfg.ssid = g_networkConfig.data.ssid;
    cfg.password = g_networkConfig.data.password;
    cfg.hostname = g_networkConfig.data.deviceName;
    cfg.connectTimeoutMs = 15000;
    cfg.reconnectBackoffMs = 5000;
    cfg.statusPollIntervalMs = 250;
    return cfg;
}

static bool slimevrMagSupportEnabledFromConfig() {
    return g_config.data.magCal.driverEnabled ||
           g_config.data.magCal.calibrationValid ||
           g_config.data.magCal.axisAlignmentValid ||
           g_config.data.magYaw.applyEnabled;
}

static bool slimevrAutostartEnabledFromConfig() {
    // SlimeVR UDP is now controlled by the network/slime runtime, not by the
    // local output/serial backend. If Wi-Fi is configured to start, the tracker
    // should also discover the server and emit RotationData automatically.
    return g_networkConfig.data.wifiEnabled &&
           g_networkConfig.data.credentialsValid;
}

static bool slimevrSetConfigFlagHook(uint8_t sensorId, uint16_t configType, bool enabled, void* user) {
    (void)sensorId;
    (void)user;
    if (configType != SLIMEVR_CONFIG_TYPE_MAGNETOMETER) {
        return false;
    }

    // Server-side mag toggles are runtime-only. They should not silently write
    // NVS, but they should go through the mag runtime controller so yaw state is
    // reset consistently.
    return setMagYawCorrectionApplyEnabledHook(enabled, false, nullptr);
}

static SlimeVROutputRuntimeConfig makeAppSlimeVRRuntimeConfig(bool enabled) {
    SlimeVROutputRuntimeConfig cfg;
    cfg.enabled = enabled;
    cfg.discoveryEnabled = g_networkConfig.data.discoveryEnabled;
    cfg.manualServerEnabled = g_networkConfig.data.manualServerEnabled;
    cfg.deviceName = g_networkConfig.data.deviceName;
    cfg.sensorId = g_networkConfig.data.sensorId;
    cfg.serverPort = g_networkConfig.data.serverPort;
    cfg.localPort = SLIMEVR_DISCOVERY_LOCAL_PORT;
    cfg.discoveryIntervalMs = 1000;
    cfg.rotationRateHz = g_config.data.output.outputRateHz;
    cfg.incomingPacketsPerUpdate = 4;
    cfg.magSupportEnabled = slimevrMagSupportEnabledFromConfig();
    cfg.magEnabled = cfg.magSupportEnabled && g_config.data.magYaw.applyEnabled;
    cfg.setConfigFlag = slimevrSetConfigFlagHook;
    cfg.setConfigFlagUser = nullptr;
    cfg.latestTemperatureValid = true;
    cfg.latestTemperatureC = g_latestTempC;
    cfg.hasCompletedRestCalibration = g_imuCal.gyroBiasValid;
    return cfg;
}

static bool copyPreparedOutputSnapshotForSlimeVR(TrackerPreparedOutputSnapshot& out, void* user) {
    (void)user;
    return g_preparedOutput.copy(out);
}

static void setupNetworkRuntime() {
    TrackerNetworkConfig loaded;
    if (g_networkConfigStore.load(loaded)) {
        g_networkConfig = loaded;
        g_networkConfigLoadedFromNvs = true;
    } else {
        g_networkConfig.resetDefaults();
        g_networkConfigLoadedFromNvs = false;
    }
    g_networkConfig.sanitize();

    g_wifiManager.begin(g_wifiStation);
    g_wifiManager.configure(makeAppWifiManagerConfig());
    g_slimevrRuntime.begin(g_udpTransport, g_wifiManager, copyPreparedOutputSnapshotForSlimeVR, nullptr);
    const bool slimeAutostart = slimevrAutostartEnabledFromConfig();
    g_slimevrRuntime.configure(makeAppSlimeVRRuntimeConfig(slimeAutostart));

    Serial.print("# network_config_loaded_from_nvs=");
    Serial.println(g_networkConfigLoadedFromNvs ? "yes" : "no");
    Serial.print("# wifi_enabled=");
    Serial.println(g_networkConfig.data.wifiEnabled ? "yes" : "no");
    if (g_networkConfig.data.wifiEnabled) {
        Serial.println("# wifi connection is non-blocking; use: net status");
    }
    if (slimeAutostart) {
        Serial.println("# slimevr_autostart=yes; use: slime status");
    }
}

static void updateNetworkRuntime() {
    const uint32_t nowMs = millis();
    g_wifiManager.update(nowMs);
    if (g_slimevrRuntime.enabled()) {
        g_slimevrRuntime.configure(makeAppSlimeVRRuntimeConfig(true));
    }
    g_slimevrRuntime.update(nowMs);
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
    deps.runtime.runtimeTestRunner = &g_runtimeTestRunner;
    deps.runtime.magState = &g_magState;
    deps.runtime.fifoIntCount = &g_fifoIntCount;
    deps.runtime.runtimeSamples = &g_runtimeSamples;
    deps.runtime.lastHeartbeatMs = &g_lastHeartbeatMs;
    deps.runtime.latestTempC = &g_latestTempC;

    deps.callbacks.setupMagRuntimeController = setupMagRuntimeController;
    deps.callbacks.setupCommandInterface = setupCommandInterface;
    deps.callbacks.setupNetworkRuntime = setupNetworkRuntime;
    deps.callbacks.updateNetworkRuntime = updateNetworkRuntime;
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
