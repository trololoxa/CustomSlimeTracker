#pragma once

#include "runtime/tracker_console_suppress.hpp"

// Runtime sample-processing hooks and final TrackerAppDeps wiring.
// This file is included by app/tracker_app_hooks.hpp after shared app dependencies.

static void maybeRecoverFifo(const ImuQualityResult& quality, const Lsm6dsv::RawSample& raw) {
    if (!quality.shouldRequestFifoRecovery) return;

    const uint32_t nowMs = millis();
    if (!trackerConsoleTrackingMessagesSuppressed(nowMs) &&
        (!g_trackingState.recoveryActive() || nowMs - g_lastRecoveryConsolePrintMs >= RECOVERY_CONSOLE_THROTTLE_MS)) {
        g_lastRecoveryConsolePrintMs = nowMs;
        Serial.print("# WARN FIFO recovery requested quality_flags=0x");
        Serial.println(quality.flags, HEX);
    }

    const uint64_t ts = raw.t_us != 0 ? raw.t_us : lsmFifo.stats().lastAssignedTimestampUs;
    enterTrackingRecovery(quality.flags, "fifo_recovery", ts);
    const bool resetOk = lsmFifo.resetFifo();
    lsmFifo.resetTimestampReconstruction(ts);
    // The sample that requested recovery already came from a broken pre-reset
    // stream. Drop prepared output so SlimeVR does not keep receiving a
    // sequence of stale/faulted quaternions while the FIFO restarts.
    g_preparedOutput.reset();
    // Preserve counters, but clear timestamp baselines and recovery latch.
    // Otherwise the next post-reset hardware timestamps can be compared
    // against the old pre-reset stream and AHRS may stay effectively frozen.
    g_quality.resetStreamRecoveryState();
    g_quality.syncFifoStats(lsmFifo.stats());
    resetFifoRuntimeCounters();

    if (!resetOk && !trackerConsoleTrackingMessagesSuppressed(millis())) {
        Serial.println("# ERR FIFO reset failed during recovery");
    }
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
        nullptr,
        callbacks
    };
    deps.lastScaledSample = &g_lastScaledSample;
    deps.lastCalibratedSample = &g_lastCalibratedSample;
    deps.lastQualityFlags = &g_lastQualityFlags;
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
#if TRACKER_ENABLE_STATIC_TEST
    g_staticTestRunner.recordFifoProcessTime(processUs);
#endif
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
    cfg.connectTimeoutMs = TRACKER_WIFI_CONNECT_TIMEOUT_MS;
    cfg.reconnectBackoffMs = TRACKER_WIFI_RECONNECT_BACKOFF_MS;
    cfg.statusPollIntervalMs = TRACKER_WIFI_STATUS_POLL_INTERVAL_MS;
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

#if TRACKER_ENABLE_BATTERY_RUNTIME
static const char* appBatteryAdcBackendName() {
#if defined(CONFIG_IDF_TARGET_ESP32C3) && (TRACKER_BATTERY_ADC_PIN >= 0) && (TRACKER_BATTERY_ADC_PIN <= 4)
    return "esp32c3_adc1_mv";
#else
    return "arduino_analog_mv";
#endif
}

static void appBatterySortSmall(uint16_t* values, uint8_t count) {
    for (uint8_t i = 1; i < count; ++i) {
        const uint16_t key = values[i];
        uint8_t j = i;
        while (j > 0 && values[j - 1] > key) {
            values[j] = values[j - 1];
            --j;
        }
        values[j] = key;
    }
}

static bool appBatteryReadOneMillivolts(uint16_t& outMillivolts) {
    const int raw = analogReadMilliVolts(TRACKER_BATTERY_ADC_PIN);
    if (raw < 0 || raw > TRACKER_BATTERY_ADC_MAX_MV) {
        return false;
    }
    outMillivolts = static_cast<uint16_t>(raw);
    return true;
}

static bool appBatteryReadMillivolts(uint16_t& outMillivolts, void* user) {
    (void)user;
#if TRACKER_ENABLE_BATTERY_RUNTIME
    constexpr uint8_t kMaxReads = 9;
    constexpr uint8_t kConfiguredReads =
        TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT < 1 ? 1 :
        (TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT > kMaxReads ? kMaxReads : TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT);

    uint16_t reads[kMaxReads] = {};
    uint8_t valid = 0;
    for (uint8_t i = 0; i < kConfiguredReads; ++i) {
        uint16_t mv = 0;
        if (appBatteryReadOneMillivolts(mv)) {
            reads[valid++] = mv;
        }
    }
    if (valid == 0) return false;

    appBatterySortSmall(reads, valid);

    // With the default three ADC1 reads, use the median. With larger override
    // values, trim one high/low tail and average the stable center.
    uint8_t begin = 0;
    uint8_t end = valid;
    if (valid >= 3) {
        begin = 1;
        end = valid - 1;
    }

    uint32_t sum = 0;
    uint8_t used = 0;
    for (uint8_t i = begin; i < end; ++i) {
        sum += reads[i];
        ++used;
    }
    if (used == 0) return false;
    outMillivolts = static_cast<uint16_t>((sum + used / 2) / used);
    return true;
#else
    outMillivolts = 0;
    return false;
#endif
}

static BatteryRuntimeConfig makeAppBatteryRuntimeConfig() {
    BatteryRuntimeConfig cfg;
    cfg.enabled = TRACKER_ENABLE_BATTERY_RUNTIME != 0;
    cfg.adcPin = TRACKER_BATTERY_ADC_PIN;
    cfg.rTopOhms = TRACKER_BATTERY_R_TOP_OHMS;
    cfg.rBottomOhms = TRACKER_BATTERY_R_BOTTOM_OHMS;
    cfg.voltageScale = TRACKER_BATTERY_VOLTAGE_SCALE;
    cfg.voltageOffset = TRACKER_BATTERY_VOLTAGE_OFFSET;
    cfg.emptyVoltage = TRACKER_BATTERY_EMPTY_VOLTAGE;
    cfg.fullVoltage = TRACKER_BATTERY_FULL_VOLTAGE;
    cfg.presentVoltageMin = TRACKER_BATTERY_PRESENT_MIN_VOLTAGE;
    cfg.emaAlpha = TRACKER_BATTERY_ADC_EMA_ALPHA;
    cfg.maxFilterStepVoltage = TRACKER_BATTERY_MAX_FILTER_STEP_V;
    cfg.sampleIntervalMs = TRACKER_BATTERY_ADC_SAMPLE_INTERVAL_MS;
    cfg.startupSamples = TRACKER_BATTERY_ADC_STARTUP_SAMPLES;
    return cfg;
}

static void setupBatteryRuntime() {
#if TRACKER_ENABLE_BATTERY_RUNTIME
    pinMode(TRACKER_BATTERY_ADC_PIN, INPUT);
#if defined(ARDUINO_ARCH_ESP32)
    analogSetPinAttenuation(TRACKER_BATTERY_ADC_PIN, ADC_11db);
#endif
#endif
    g_batteryRuntime.begin(appBatteryReadMillivolts, nullptr);
    const BatteryRuntimeConfig cfg = makeAppBatteryRuntimeConfig();
    g_batteryRuntime.configure(cfg);
    g_batteryRuntime.update(millis());

#if TRACKER_ENABLE_SERIAL_CONSOLE
    Serial.print("# battery_runtime_enabled=");
    Serial.print(cfg.enabled ? "yes" : "no");
    Serial.print(" pin=");
    Serial.print(cfg.adcPin);
    Serial.print(" divider=");
    Serial.print(cfg.rTopOhms, 0);
    Serial.print('/');
    Serial.print(cfg.rBottomOhms, 0);
    Serial.print(" backend=");
    Serial.println(appBatteryAdcBackendName());
#endif
}

static void updateBatteryRuntime() {
    g_batteryRuntime.update(millis());
}

#endif // TRACKER_ENABLE_BATTERY_RUNTIME

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
    cfg.discoveryIntervalMs = TRACKER_SLIMEVR_DISCOVERY_INTERVAL_MS;
    cfg.rotationRateHz = g_config.data.output.outputRateHz;
    if (cfg.rotationRateHz > TRACKER_SLIMEVR_OUTPUT_RATE_HZ_MAX) {
        cfg.rotationRateHz = TRACKER_SLIMEVR_OUTPUT_RATE_HZ_MAX;
    }
    cfg.incomingPacketsPerUpdate = TRACKER_SLIMEVR_INCOMING_PACKETS_PER_UPDATE;
    cfg.magSupportEnabled = slimevrMagSupportEnabledFromConfig();
    cfg.magEnabled = cfg.magSupportEnabled && g_config.data.magYaw.applyEnabled;
    cfg.setConfigFlag = slimevrSetConfigFlagHook;
    cfg.setConfigFlagUser = nullptr;
    cfg.latestTemperatureValid = true;
    cfg.latestTemperatureC = g_latestTempC;
#if TRACKER_ENABLE_BATTERY_RUNTIME
    cfg.batteryTelemetryEnabled = (TRACKER_SLIMEVR_ENABLE_BATTERY_TELEMETRY != 0);
    {
        float voltage = 0.0f;
        float percentage = 0.0f;
        cfg.latestBatteryValid = g_batteryRuntime.telemetry(voltage, percentage);
        cfg.latestBatteryVoltage = voltage;
        cfg.latestBatteryPercentage = percentage;
    }
#else
    cfg.batteryTelemetryEnabled = false;
    cfg.latestBatteryValid = false;
    cfg.latestBatteryVoltage = 0.0f;
    cfg.latestBatteryPercentage = 0.0f;
#endif
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

#if TRACKER_ENABLE_SERIAL_CONSOLE
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
#endif
}



#if TRACKER_ENABLE_STATUS_LED
static TrackerStatusLedMode deriveStatusLedMode() {
    if (g_statusLedSensorError) return TrackerStatusLedMode::SensorError;
#if !TRACKER_ENABLE_STATUS_LED
    return TrackerStatusLedMode::Disabled;
#else
    if (!g_slimevrRuntime.enabled()) {
        if (g_networkConfig.data.wifiEnabled && !g_networkConfig.data.credentialsValid) {
            return TrackerStatusLedMode::ConnectionError;
        }
        return TrackerStatusLedMode::Off;
    }

    const TrackerWifiManagerStatus wifi = g_wifiManager.status();
    if (wifi.desiredEnabled && !wifi.credentialsValid) {
        return TrackerStatusLedMode::ConnectionError;
    }
    if (wifi.state == TrackerWifiState::Backoff ||
        wifi.linkStatus == WifiLinkStatus::NoSsid ||
        wifi.linkStatus == WifiLinkStatus::ConnectFailed) {
        return TrackerStatusLedMode::ConnectionError;
    }

    switch (g_slimevrRuntime.state()) {
        case SlimeVROutputState::Disabled:
            return TrackerStatusLedMode::Off;
        case SlimeVROutputState::WaitingForWifi:
        case SlimeVROutputState::UdpStarting:
            return TrackerStatusLedMode::WifiConnecting;
        case SlimeVROutputState::Discovering:
            return TrackerStatusLedMode::ServerDiscovering;
        case SlimeVROutputState::ServerFound:
            return TrackerStatusLedMode::Normal;
        case SlimeVROutputState::Error:
            return TrackerStatusLedMode::ConnectionError;
    }
    return TrackerStatusLedMode::ConnectionError;
#endif
}

static void setupStatusLedRuntime() {
    StatusLedRuntimeConfig cfg;
    cfg.enabled = TRACKER_ENABLE_STATUS_LED != 0;
    cfg.pin = TRACKER_STATUS_LED_PIN;
    cfg.activeLow = TRACKER_STATUS_LED_ACTIVE_LOW != 0;
    cfg.updateIntervalMs = TRACKER_STATUS_LED_UPDATE_INTERVAL_MS;
    cfg.normalBlinkPeriodMs = TRACKER_STATUS_LED_NORMAL_BLINK_PERIOD_MS;
    cfg.normalBlinkOnMs = TRACKER_STATUS_LED_NORMAL_BLINK_ON_MS;
    cfg.shortBlinkOnMs = TRACKER_STATUS_LED_SHORT_BLINK_ON_MS;
    cfg.shortBlinkOffMs = TRACKER_STATUS_LED_SHORT_BLINK_OFF_MS;
    cfg.longBlinkOnMs = TRACKER_STATUS_LED_LONG_BLINK_ON_MS;
    cfg.longBlinkOffMs = TRACKER_STATUS_LED_LONG_BLINK_OFF_MS;
    cfg.errorBlinkPeriodMs = TRACKER_STATUS_LED_ERROR_BLINK_PERIOD_MS;
    cfg.identifyBlinkOnMs = TRACKER_STATUS_LED_IDENTIFY_BLINK_ON_MS;
    cfg.identifyBlinkOffMs = TRACKER_STATUS_LED_IDENTIFY_BLINK_OFF_MS;

    g_statusLedRuntime.begin(g_statusLedSink, cfg);
    g_statusLedRuntime.setMode(TrackerStatusLedMode::Boot, millis());
    g_statusLedRuntime.update(millis());

#if TRACKER_ENABLE_SERIAL_CONSOLE
    Serial.print("# status_led_enabled=");
    Serial.print(cfg.enabled ? "yes" : "no");
    Serial.print(" pin=");
    Serial.print(static_cast<int>(cfg.pin));
    Serial.print(" active_low=");
    Serial.println(cfg.activeLow ? "yes" : "no");
#endif
}

static void updateStatusLedRuntime() {
    const uint32_t nowMs = millis();
    g_statusLedRuntime.setMode(deriveStatusLedMode(), nowMs);
    g_statusLedRuntime.update(nowMs);
}

static void setStatusLedSensorError() {
    g_statusLedSensorError = true;
    const uint32_t nowMs = millis();
    g_statusLedRuntime.setMode(TrackerStatusLedMode::SensorError, nowMs);
    g_statusLedRuntime.update(nowMs);
}

#endif // TRACKER_ENABLE_STATUS_LED

#if TRACKER_ENABLE_TAP_RUNTIME
static void setupTapRuntime() {
    g_tapRuntime.begin(lsm, g_slimevrRuntime);

    TapRuntimeConfig cfg;
    cfg.enabled = TRACKER_ENABLE_TAP_RUNTIME != 0;
    cfg.hardwareDoubleTap = TRACKER_TAP_HARDWARE_DOUBLE_TAP != 0;
    cfg.slidingWindow = TRACKER_TAP_SLIDING_WINDOW != 0;
    cfg.sensorId = g_networkConfig.data.sensorId;
    cfg.minCount = TRACKER_TAP_MIN_COUNT;
    cfg.maxCount = TRACKER_TAP_MAX_COUNT;
    cfg.pollIntervalMs = TRACKER_TAP_POLL_INTERVAL_MS;
    cfg.aggregationWindowMs = TRACKER_TAP_AGGREGATION_WINDOW_MS;
    cfg.duplicateSuppressMs = TRACKER_TAP_DUPLICATE_SUPPRESS_MS;
    cfg.postSendLockoutMs = TRACKER_TAP_POST_SEND_LOCKOUT_MS;
    cfg.registerVerifyIntervalMs = TRACKER_TAP_REGISTER_VERIFY_INTERVAL_MS;
    cfg.threshold = TRACKER_LSM6DSV_TAP_THRESHOLD;
    cfg.shock = TRACKER_LSM6DSV_TAP_SHOCK;
    cfg.quiet = TRACKER_LSM6DSV_TAP_QUIET;
    cfg.duration = TRACKER_LSM6DSV_TAP_DURATION;

    const bool ok = g_tapRuntime.configure(cfg);
#if TRACKER_ENABLE_SERIAL_CONSOLE
    Serial.print("# tap_runtime_enabled=");
    Serial.print(cfg.enabled ? "yes" : "no");
    Serial.print(" hardware=");
    Serial.println(ok ? "ok" : "fail");
#endif
}

static void updateTapRuntime() {
    g_tapRuntime.update(millis());
}

#endif // TRACKER_ENABLE_TAP_RUNTIME

static void updateNetworkRuntime() {
    const uint32_t nowMs = millis();
#if TRACKER_NETWORK_RUNTIME_UPDATE_INTERVAL_MS > 0
    static bool s_networkRuntimeUpdateValid = false;
    static uint32_t s_lastNetworkRuntimeUpdateMs = 0;
    if (s_networkRuntimeUpdateValid &&
        static_cast<uint32_t>(nowMs - s_lastNetworkRuntimeUpdateMs) < TRACKER_NETWORK_RUNTIME_UPDATE_INTERVAL_MS) {
        return;
    }
    s_networkRuntimeUpdateValid = true;
    s_lastNetworkRuntimeUpdateMs = nowMs;
#endif

    g_wifiManager.update(nowMs);
    if (g_slimevrRuntime.enabled()) {
#if TRACKER_SLIMEVR_RUNTIME_CONFIG_REFRESH_MS == 0
        g_slimevrRuntime.configure(makeAppSlimeVRRuntimeConfig(true));
#else
        static bool s_slimeRuntimeConfigRefreshValid = false;
        static uint32_t s_lastSlimeRuntimeConfigRefreshMs = 0;
        if (!s_slimeRuntimeConfigRefreshValid ||
            static_cast<uint32_t>(nowMs - s_lastSlimeRuntimeConfigRefreshMs) >= TRACKER_SLIMEVR_RUNTIME_CONFIG_REFRESH_MS) {
            s_slimeRuntimeConfigRefreshValid = true;
            s_lastSlimeRuntimeConfigRefreshMs = nowMs;
            g_slimevrRuntime.configure(makeAppSlimeVRRuntimeConfig(true));
        }
#endif
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
#if TRACKER_ENABLE_SERIAL_CLI
    deps.runtime.cli = &g_cli;
#endif
    deps.runtime.streamState = &g_streamState;
    deps.runtime.perf = &g_perf;
    deps.runtime.fifoEvents = &g_fifoEvents;
    deps.runtime.fifoRuntime = &g_fifoRuntime;
#if TRACKER_ENABLE_STATIC_TEST || TRACKER_ENABLE_BOOT_HEARTBEAT
    deps.runtime.staticTestRunner = &g_staticTestRunner;
#endif
#if TRACKER_ENABLE_RUNTIME_TEST || TRACKER_ENABLE_BOOT_HEARTBEAT
    deps.runtime.runtimeTestRunner = &g_runtimeTestRunner;
#endif
    deps.runtime.magState = &g_magState;
    deps.runtime.fifoIntCount = &g_fifoIntCount;
    deps.runtime.runtimeSamples = &g_runtimeSamples;
    deps.runtime.lastHeartbeatMs = &g_lastHeartbeatMs;
    deps.runtime.latestTempC = &g_latestTempC;

    deps.callbacks.setupMagRuntimeController = setupMagRuntimeController;
#if TRACKER_ENABLE_SERIAL_CLI || TRACKER_ENABLE_STATIC_TEST || TRACKER_ENABLE_RUNTIME_TEST
    deps.callbacks.setupCommandInterface = setupCommandInterface;
#endif
    deps.callbacks.setupNetworkRuntime = setupNetworkRuntime;
    deps.callbacks.updateNetworkRuntime = updateNetworkRuntime;
#if TRACKER_ENABLE_TAP_RUNTIME
    deps.callbacks.setupTapRuntime = setupTapRuntime;
    deps.callbacks.updateTapRuntime = updateTapRuntime;
#endif
#if TRACKER_ENABLE_STATUS_LED
    deps.callbacks.setupStatusLedRuntime = setupStatusLedRuntime;
    deps.callbacks.updateStatusLedRuntime = updateStatusLedRuntime;
    deps.callbacks.setStatusLedSensorError = setStatusLedSensorError;
#endif
#if TRACKER_ENABLE_BATTERY_RUNTIME
    deps.callbacks.setupBatteryRuntime = setupBatteryRuntime;
    deps.callbacks.updateBatteryRuntime = updateBatteryRuntime;
#endif
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
