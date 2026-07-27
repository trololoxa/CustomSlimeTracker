#pragma once

#include <cstdio>
#include <cstring>

#include "runtime/tracker_console_suppress.hpp"

// Runtime sample-processing hooks and final TrackerAppDeps wiring.
// This file is included by app/tracker_app_hooks.hpp after shared app dependencies.

static void maybeRecoverFifo(const ImuQualityResult& quality, const Lsm6dsv::RawSample& raw) {
    if (!quality.shouldRequestFifoRecovery) return;

    const uint32_t nowMs = millis();
    const bool consoleSuppressed = trackerConsoleTrackingMessagesSuppressed(nowMs);
    if (!consoleSuppressed &&
        (!g_trackingState.recoveryActive() || nowMs - g_lastRecoveryConsolePrintMs >= RECOVERY_CONSOLE_THROTTLE_MS)) {
        g_lastRecoveryConsolePrintMs = nowMs;
        appConsoleOutput().print("# WARN FIFO recovery requested quality_flags=0x");
        appConsoleOutput().println(quality.flags, HEX);
    }

    const uint64_t ts = raw.t_us != 0 ? raw.t_us : lsmFifo.stats().lastAssignedTimestampUs;
    const bool useSoftRecovery = trackingFifoLossCanUseSoftRecovery(quality);
    enterTrackingRecovery(quality.flags,
                          useSoftRecovery ? "fifo_recovery_soft" : "fifo_recovery",
                          ts);
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
        appConsoleOutput().println("# ERR FIFO reset failed during recovery");
    }
}

#if TRACKER_HAS_GYRO_TEMP_FIT
static void gyroTempFitModelAppliedHook(bool biasValidityChanged, void* user) {
    (void)user;
#if TRACKER_HAS_CALIBRATION_UI
    g_gyroTempCapture.reset();
#endif
    runtimeBiasReset(g_runtimeBias);
#if TRACKER_HAS_SERIAL_CLI
    hookResetAhrsRuntime(nullptr);
#else
    g_ahrs6dof.reset();
    g_preparedOutput.reset();
#endif
    if (biasValidityChanged) g_slimevrRuntime.requestSensorInfoRefresh();
}

static GyroTempStaticFitDeps makeGyroTempStaticFitDeps() {
    GyroTempStaticFitDeps deps;
#if TRACKER_HAS_STATIC_TEST_STATE
    deps.lastCompletedStaticTest = &g_lastCompletedStaticTest;
    deps.lastCompletedStaticTestValid = g_lastCompletedStaticTestValid;
#endif
    deps.gyroTempComp = &g_gyroTempComp;
    deps.imuCal = &g_imuCal;
    deps.runtimeBias = &g_runtimeBias;
    deps.config = &g_config;
    deps.configStore = &g_configStore;
    deps.onModelApplied = gyroTempFitModelAppliedHook;
    deps.onModelAppliedUser = nullptr;
    return deps;
}

static bool fitGyroTempFromLastStaticHook(bool persist, Stream& out, void* user) {
    (void)user;
#if TRACKER_HAS_STATIC_TEST_STATE
    GyroTempStaticFitDeps deps = makeGyroTempStaticFitDeps();
    return fitGyroTempFromLastStatic(deps, persist, out);
#else
    (void)persist;
    out.println("# gyro temperature static-test history is not compiled in this profile");
    return false;
#endif
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
#else
static bool fitGyroTempFromLastStaticHook(bool persist, Stream& out, void* user) {
    (void)persist;
    (void)user;
    out.println("# gyro temperature fit is not compiled in this profile");
    return false;
}

static bool fitGyroTempFromCaptureHook(const StaticRuntimeTest* capture, bool persist, Stream& out, void* user) {
    (void)capture;
    (void)persist;
    (void)user;
    out.println("# gyro temperature fit is not compiled in this profile");
    return false;
}

static bool fitGyroTempFromCaptureRamHook(const StaticRuntimeTest* capture, Stream& out, void* user) {
    (void)capture;
    (void)user;
    out.println("# gyro temperature fit is not compiled in this profile");
    return false;
}
#endif

static void pipelineEnterTrackingRecoveryCallback(uint32_t reasonFlags,
                                                  const char* reason,
                                                  uint64_t timestampUs,
                                                  void* user) {
    (void)user;
    enterTrackingRecovery(reasonFlags, reason, timestampUs);
}

static void pipelineUpdateTrackingRecoveryCallback(const ImuQualityResult& quality,
                                                   const Vec3& gyroRadS,
                                                   const Vec3& accelG,
                                                   uint64_t timestampUs,
                                                   bool ahrsIntegrated,
                                                   void* user) {
    (void)user;
    updateTrackingRecoveryState(quality, gyroRadS, accelG, timestampUs, ahrsIntegrated);
}

#if TRACKER_HAS_MACHINE_LOG
static void pipelineEmitMachineLogFrameCallback(const Lsm6dsv::RawSample& raw,
                                                const Lsm6dsv::Sample& calibrated,
                                                const ImuQualityResult& quality,
                                                void* user) {
    (void)user;
    emitMachineLogFrame(raw, calibrated, quality);
}
#endif

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
#if TRACKER_HAS_MACHINE_LOG
    callbacks.emitMachineLogFrame = pipelineEmitMachineLogFrameCallback;
#endif
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
#if TRACKER_HAS_SERIAL_STREAM
        &g_streamState,
#else
        nullptr,
#endif
#if TRACKER_HAS_MACHINE_LOG
        &g_logState,
        &g_logCounters,
#else
        nullptr,
        nullptr,
#endif
#if TRACKER_HAS_STATIC_TEST
        &g_staticTestRunner,
#else
        nullptr,
#endif
#if TRACKER_HAS_CALIBRATION_UI
        &g_gyroTempCapture,
#else
        nullptr,
#endif
#if TRACKER_HAS_CALIBRATION_AUTONOMY
        &g_calibrationAutonomy,
#endif
        g_perf,
#if TRACKER_HAS_RUNTIME_PROFILER
        &g_motionDiagnostics,
#endif
#if TRACKER_HAS_CALIBRATION_UI
        &g_calIo,
#else
        nullptr,
#endif
        appConsoleOutput(),
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

static ImuSamplePipelineDeps& runtimeImuSamplePipelineDeps() {
    // All referenced runtime objects have static lifetime. Reusing this wiring
    // avoids rebuilding a large aggregate of references/pointers at 960 Hz.
    static ImuSamplePipelineDeps deps = makeImuSamplePipelineDeps();
    return deps;
}

static FifoRuntimeSampleResult processRuntimeRawSampleCallback(const Lsm6dsv::RawSample& raw,
                                                               bool checkFifoStatsDelta,
                                                               void* user) {
    (void)user;
    return imuSamplePipelineProcessRaw(runtimeImuSamplePipelineDeps(), raw, checkFifoStatsDelta);
}

static void processRuntimeMagSampleCallback(const Lsm6dsvFifoReader::MagRawSample& mag, void* user) {
    (void)user;
    processOneMagRawSample(mag);
}

static void recordRuntimeFifoProcessTime(uint32_t processUs, void* user) {
    (void)user;
    recordFifoProcessTime(processUs);
#if TRACKER_HAS_STATIC_TEST
    g_staticTestRunner.recordFifoProcessTime(processUs);
#endif
}

static void appAttachFifoInterruptCallback() {
    attachInterrupt(digitalPinToInterrupt(PIN_LSM_INT1), onFifoInt1, RISING);
}

static void appDetachFifoInterruptCallback() {
    detachInterrupt(digitalPinToInterrupt(PIN_LSM_INT1));
}

static bool appSetMagRuntimeEnabledCallback(bool enabled, bool persist) {
    return setMagRuntimeEnabledHook(enabled, persist, nullptr);
}

static bool appStartMagRuntimeFromPreconfiguredFifoCallback() {
    return g_magRuntime.startFromPreconfiguredFifo();
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
    constexpr uint8_t kMaxReads = 128;
    constexpr uint8_t kConfiguredReads =
        TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT < 1 ? 1 :
        (TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT > kMaxReads ? kMaxReads : TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT);
    constexpr uint8_t kDiscardReadsRaw =
        TRACKER_BATTERY_ADC_DISCARD_COUNT > 8 ? 8 : TRACKER_BATTERY_ADC_DISCARD_COUNT;
    constexpr uint8_t kDiscardReads = kConfiguredReads <= 1 ? 0 :
        (kDiscardReadsRaw >= kConfiguredReads ? static_cast<uint8_t>(kConfiguredReads - 1) : kDiscardReadsRaw);

    // High-value divider without a hardware capacitor: throw away the first
    // few conversions after the sparse wake-up read, then use a trimmed mean
    // of the remaining burst. This keeps the long-term EMA from chasing SAR
    // settling noise or one-off RF/USB spikes.
    for (uint8_t i = 0; i < kDiscardReads; ++i) {
        uint16_t ignored = 0;
        (void)appBatteryReadOneMillivolts(ignored);
    }

    uint16_t reads[kMaxReads] = {};
    uint8_t valid = 0;
    const uint8_t readsToKeep = static_cast<uint8_t>(kConfiguredReads - kDiscardReads);
    for (uint8_t i = 0; i < readsToKeep; ++i) {
        uint16_t mv = 0;
        if (appBatteryReadOneMillivolts(mv)) {
            reads[valid++] = mv;
        }
    }
    if (valid == 0) return false;

    appBatterySortSmall(reads, valid);

    uint8_t trim = 0;
    if (valid >= 16) {
        trim = valid / 8; // discard roughly 12.5% from each tail
    } else if (valid >= 5) {
        trim = 1;
    }

    uint8_t begin = trim;
    uint8_t end = static_cast<uint8_t>(valid - trim);
    if (begin >= end) {
        begin = 0;
        end = valid;
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
    cfg.presentVoltageMax = TRACKER_BATTERY_PRESENT_MAX_VOLTAGE;
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
    appConsoleOutput().print("# battery_runtime_enabled=");
    appConsoleOutput().print(cfg.enabled ? "yes" : "no");
    appConsoleOutput().print(" pin=");
    appConsoleOutput().print(cfg.adcPin);
    appConsoleOutput().print(" divider=");
    appConsoleOutput().print(cfg.rTopOhms, 0);
    appConsoleOutput().print('/');
    appConsoleOutput().print(cfg.rBottomOhms, 0);
    appConsoleOutput().print(" backend=");
    appConsoleOutput().println(appBatteryAdcBackendName());
#endif
}

static bool updateBatteryRuntime() {
    return g_batteryRuntime.update(millis());
}

#endif // TRACKER_ENABLE_BATTERY_RUNTIME

static bool slimevrSetConfigFlagHook(uint8_t sensorId, uint16_t configType, bool enabled, void* user) {
    (void)sensorId;
    (void)user;
    if (configType != SLIMEVR_CONFIG_TYPE_MAGNETOMETER) {
        return false;
    }

    // Packet 25 is a persistent configuration transaction. The mag runtime
    // controller saves the candidate first and rolls back runtime/config on any
    // failure; packet 24 is emitted only after this hook returns success.
    return setMagYawCorrectionApplyEnabledHook(enabled, true, nullptr);
}

static SlimeVROutputRuntimeConfig makeAppSlimeVRRuntimeConfig(bool enabled) {
    SlimeVROutputRuntimeConfig cfg;
    cfg.enabled = enabled;
    cfg.discoveryEnabled = g_networkConfig.data.discoveryEnabled;
    cfg.manualServerEnabled = g_networkConfig.data.manualServerEnabled;
    cfg.manualServerHost = g_networkConfig.data.serverHost;
    cfg.deviceName = g_networkConfig.data.deviceName;
    cfg.sensorId = g_networkConfig.data.sensorId;
    cfg.serverPort = g_networkConfig.data.serverPort;
    cfg.localPort = SLIMEVR_DISCOVERY_LOCAL_PORT;
    cfg.discoveryIntervalMs = TRACKER_SLIMEVR_DISCOVERY_INTERVAL_MS;
#if TRACKER_SLIMEVR_FORCE_ROTATION_RATE_HZ > 0
    cfg.rotationRateHz = TRACKER_SLIMEVR_FORCE_ROTATION_RATE_HZ;
#else
    cfg.rotationRateHz = g_config.data.output.outputRateHz;
#endif
    if (cfg.rotationRateHz > TRACKER_SLIMEVR_OUTPUT_RATE_HZ_MAX) {
        cfg.rotationRateHz = TRACKER_SLIMEVR_OUTPUT_RATE_HZ_MAX;
    }
    if (cfg.rotationRateHz == 0u) {
        cfg.rotationRateHz = cfg::OUTPUT_RATE_HZ;
    }
    cfg.incomingPacketsPerUpdate = TRACKER_SLIMEVR_INCOMING_PACKETS_PER_UPDATE;
    cfg.signalTelemetryEnabled = (TRACKER_SLIMEVR_ENABLE_SIGNAL_TELEMETRY != 0);
    cfg.temperatureTelemetryEnabled = (TRACKER_SLIMEVR_ENABLE_TEMPERATURE_TELEMETRY != 0);
    cfg.telemetryIntervalMs = TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS;
    cfg.signalTelemetryIntervalMs = TRACKER_SLIMEVR_SIGNAL_TELEMETRY_INTERVAL_MS;
    cfg.temperatureTelemetryIntervalMs = TRACKER_SLIMEVR_TEMPERATURE_TELEMETRY_INTERVAL_MS;
    cfg.batteryTelemetryIntervalMs = TRACKER_SLIMEVR_BATTERY_TELEMETRY_INTERVAL_MS;
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


struct AppSlimeVRRuntimeStaticConfigCache {
    bool valid = false;
    bool enabled = false;
    bool discoveryEnabled = true;
    bool manualServerEnabled = false;
    char manualServerHost[64] = {};
    char deviceName[32] = {};
    uint8_t sensorId = 0;
    uint16_t serverPort = 0;
    uint16_t localPort = 0;
    uint32_t discoveryIntervalMs = 0;
    uint16_t rotationRateHz = 0;
    uint8_t incomingPacketsPerUpdate = 0;
    bool magSupportEnabled = false;
    bool magEnabled = false;
    bool signalTelemetryEnabled = false;
    bool temperatureTelemetryEnabled = false;
    bool batteryTelemetryEnabled = false;
    uint32_t telemetryIntervalMs = 0;
    uint32_t signalTelemetryIntervalMs = 0;
    uint32_t temperatureTelemetryIntervalMs = 0;
    uint32_t batteryTelemetryIntervalMs = 0;
};

static AppSlimeVRRuntimeStaticConfigCache g_slimeRuntimeStaticConfigCache;

static void copyAppSlimeVRDeviceName(char* dst, size_t dstSize, const char* src) {
    if (dst == nullptr || dstSize == 0u) return;
    const char* value = (src != nullptr && src[0] != '\0') ? src : "c3-6dsv-tracker";
    std::strncpy(dst, value, dstSize - 1u);
    dst[dstSize - 1u] = '\0';
}

static bool appSlimeVRRuntimeStaticConfigMatches(const SlimeVROutputRuntimeConfig& cfg) {
    if (!g_slimeRuntimeStaticConfigCache.valid) return false;
    char deviceName[sizeof(g_slimeRuntimeStaticConfigCache.deviceName)] = {};
    copyAppSlimeVRDeviceName(deviceName, sizeof(deviceName), cfg.deviceName);

    return g_slimeRuntimeStaticConfigCache.enabled == cfg.enabled &&
           g_slimeRuntimeStaticConfigCache.discoveryEnabled == cfg.discoveryEnabled &&
           g_slimeRuntimeStaticConfigCache.manualServerEnabled == cfg.manualServerEnabled &&
           std::strncmp(g_slimeRuntimeStaticConfigCache.manualServerHost,
                        cfg.manualServerHost ? cfg.manualServerHost : "",
                        sizeof(g_slimeRuntimeStaticConfigCache.manualServerHost)) == 0 &&
           std::strncmp(g_slimeRuntimeStaticConfigCache.deviceName, deviceName, sizeof(g_slimeRuntimeStaticConfigCache.deviceName)) == 0 &&
           g_slimeRuntimeStaticConfigCache.sensorId == cfg.sensorId &&
           g_slimeRuntimeStaticConfigCache.serverPort == cfg.serverPort &&
           g_slimeRuntimeStaticConfigCache.localPort == cfg.localPort &&
           g_slimeRuntimeStaticConfigCache.discoveryIntervalMs == cfg.discoveryIntervalMs &&
           g_slimeRuntimeStaticConfigCache.rotationRateHz == cfg.rotationRateHz &&
           g_slimeRuntimeStaticConfigCache.incomingPacketsPerUpdate == cfg.incomingPacketsPerUpdate &&
           g_slimeRuntimeStaticConfigCache.magSupportEnabled == cfg.magSupportEnabled &&
           g_slimeRuntimeStaticConfigCache.magEnabled == cfg.magEnabled &&
           g_slimeRuntimeStaticConfigCache.signalTelemetryEnabled == cfg.signalTelemetryEnabled &&
           g_slimeRuntimeStaticConfigCache.temperatureTelemetryEnabled == cfg.temperatureTelemetryEnabled &&
           g_slimeRuntimeStaticConfigCache.batteryTelemetryEnabled == cfg.batteryTelemetryEnabled &&
           g_slimeRuntimeStaticConfigCache.telemetryIntervalMs == cfg.telemetryIntervalMs &&
           g_slimeRuntimeStaticConfigCache.signalTelemetryIntervalMs == cfg.signalTelemetryIntervalMs &&
           g_slimeRuntimeStaticConfigCache.temperatureTelemetryIntervalMs == cfg.temperatureTelemetryIntervalMs &&
           g_slimeRuntimeStaticConfigCache.batteryTelemetryIntervalMs == cfg.batteryTelemetryIntervalMs;
}

static void appSlimeVRRuntimeStaticConfigCapture(const SlimeVROutputRuntimeConfig& cfg) {
    g_slimeRuntimeStaticConfigCache.valid = true;
    g_slimeRuntimeStaticConfigCache.enabled = cfg.enabled;
    g_slimeRuntimeStaticConfigCache.discoveryEnabled = cfg.discoveryEnabled;
    g_slimeRuntimeStaticConfigCache.manualServerEnabled = cfg.manualServerEnabled;
    std::strncpy(g_slimeRuntimeStaticConfigCache.manualServerHost,
                 cfg.manualServerHost ? cfg.manualServerHost : "",
                 sizeof(g_slimeRuntimeStaticConfigCache.manualServerHost) - 1u);
    g_slimeRuntimeStaticConfigCache.manualServerHost[
        sizeof(g_slimeRuntimeStaticConfigCache.manualServerHost) - 1u] = '\0';
    copyAppSlimeVRDeviceName(g_slimeRuntimeStaticConfigCache.deviceName,
                             sizeof(g_slimeRuntimeStaticConfigCache.deviceName),
                             cfg.deviceName);
    g_slimeRuntimeStaticConfigCache.sensorId = cfg.sensorId;
    g_slimeRuntimeStaticConfigCache.serverPort = cfg.serverPort;
    g_slimeRuntimeStaticConfigCache.localPort = cfg.localPort;
    g_slimeRuntimeStaticConfigCache.discoveryIntervalMs = cfg.discoveryIntervalMs;
    g_slimeRuntimeStaticConfigCache.rotationRateHz = cfg.rotationRateHz;
    g_slimeRuntimeStaticConfigCache.incomingPacketsPerUpdate = cfg.incomingPacketsPerUpdate;
    g_slimeRuntimeStaticConfigCache.magSupportEnabled = cfg.magSupportEnabled;
    g_slimeRuntimeStaticConfigCache.magEnabled = cfg.magEnabled;
    g_slimeRuntimeStaticConfigCache.signalTelemetryEnabled = cfg.signalTelemetryEnabled;
    g_slimeRuntimeStaticConfigCache.temperatureTelemetryEnabled = cfg.temperatureTelemetryEnabled;
    g_slimeRuntimeStaticConfigCache.batteryTelemetryEnabled = cfg.batteryTelemetryEnabled;
    g_slimeRuntimeStaticConfigCache.telemetryIntervalMs = cfg.telemetryIntervalMs;
    g_slimeRuntimeStaticConfigCache.signalTelemetryIntervalMs = cfg.signalTelemetryIntervalMs;
    g_slimeRuntimeStaticConfigCache.temperatureTelemetryIntervalMs = cfg.temperatureTelemetryIntervalMs;
    g_slimeRuntimeStaticConfigCache.batteryTelemetryIntervalMs = cfg.batteryTelemetryIntervalMs;
}

static void updateAppSlimeVRRuntimeLiveState(uint32_t nowMs, bool force = false) {
#if TRACKER_SLIMEVR_LIVE_STATE_REFRESH_MS > 0
    static bool s_liveStateRefreshValid = false;
    static uint32_t s_lastLiveStateRefreshMs = 0;
    if (!force && s_liveStateRefreshValid &&
        static_cast<uint32_t>(nowMs - s_lastLiveStateRefreshMs) < TRACKER_SLIMEVR_LIVE_STATE_REFRESH_MS) {
        return;
    }
    s_liveStateRefreshValid = true;
    s_lastLiveStateRefreshMs = nowMs;
#else
    (void)nowMs;
#endif

    bool batteryValid = false;
    float batteryVoltage = 0.0f;
    float batteryPercentage = 0.0f;
#if TRACKER_ENABLE_BATTERY_RUNTIME
    batteryValid = g_batteryRuntime.telemetry(batteryVoltage, batteryPercentage);
#endif
    g_slimevrRuntime.updateLiveState(
        true,
        g_latestTempC,
        batteryValid,
        batteryVoltage,
        batteryPercentage,
        g_imuCal.gyroBiasValid
    );
}

static void refreshAppSlimeVRRuntimeStaticConfig(uint32_t nowMs, bool force, bool enabled) {
#if TRACKER_SLIMEVR_RUNTIME_CONFIG_REFRESH_MS > 0
    static bool s_slimeRuntimeConfigRefreshValid = false;
    static uint32_t s_lastSlimeRuntimeConfigRefreshMs = 0;
    if (!force && s_slimeRuntimeConfigRefreshValid &&
        static_cast<uint32_t>(nowMs - s_lastSlimeRuntimeConfigRefreshMs) < TRACKER_SLIMEVR_RUNTIME_CONFIG_REFRESH_MS) {
        return;
    }
    s_slimeRuntimeConfigRefreshValid = true;
    s_lastSlimeRuntimeConfigRefreshMs = nowMs;
#else
    (void)nowMs;
#endif

    SlimeVROutputRuntimeConfig cfg = makeAppSlimeVRRuntimeConfig(enabled);
    if (force || !appSlimeVRRuntimeStaticConfigMatches(cfg)) {
        g_slimevrRuntime.configure(cfg);
        appSlimeVRRuntimeStaticConfigCapture(cfg);
    }
}

static bool copyPreparedOutputSnapshotForSlimeVR(TrackerPreparedOutputSnapshot& out, void* user) {
    (void)user;
    return g_preparedOutput.copy(out);
}

static void publishTrackerHealthState(const TrackerHealthSnapshot& health) {
    g_slimevrRuntime.setTrackerHealth(health);
}

static void startNetworkRuntimeFromCurrentConfig(bool printStartup) {
    g_networkConfig.sanitize();

    g_wifiManager.begin(g_wifiStation);
    g_wifiManager.configure(makeAppWifiManagerConfig());
    g_slimevrRuntime.begin(g_udpTransport, g_wifiManager, copyPreparedOutputSnapshotForSlimeVR, nullptr);
    g_slimevrRuntime.setTrackerHealth(g_trackerHealth.snapshot());
    const bool slimeAutostart = slimevrAutostartEnabledFromConfig();
    const uint32_t nowMs = millis();
    refreshAppSlimeVRRuntimeStaticConfig(nowMs, true, slimeAutostart);
    updateAppSlimeVRRuntimeLiveState(nowMs, true);

#if TRACKER_ENABLE_SERIAL_CONSOLE
    if (printStartup) {
        appConsoleOutput().print("# network_config_loaded_from_nvs=");
        appConsoleOutput().println(g_networkConfigLoadedFromNvs ? "yes" : "no");
        appConsoleOutput().print("# wifi_enabled=");
        appConsoleOutput().println(g_networkConfig.data.wifiEnabled ? "yes" : "no");
        if (g_networkConfig.data.wifiEnabled) {
            appConsoleOutput().println("# wifi connection is non-blocking; use: net status");
        }
        if (slimeAutostart) {
            appConsoleOutput().println("# slimevr_autostart=yes; use: slime status");
        }
    }
#else
    (void)printStartup;
#endif
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
    startNetworkRuntimeFromCurrentConfig(true);
}

#if TRACKER_HAS_MOTION_LIGHT_SLEEP
static void resumeNetworkRuntime() {
    // A motion wake is a transient platform event, not a configuration load:
    // retain unsaved `net` / `slime` runtime choices and existing counters.
    startNetworkRuntimeFromCurrentConfig(false);
}
#endif



#if TRACKER_ENABLE_STATUS_LED
static TrackerStatusLedMode deriveStatusLedMode() {
    if (g_trackerHealth.fatalActive()) return TrackerStatusLedMode::SensorError;
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
    appConsoleOutput().print("# status_led_enabled=");
    appConsoleOutput().print(cfg.enabled ? "yes" : "no");
    appConsoleOutput().print(" pin=");
    appConsoleOutput().print(static_cast<int>(cfg.pin));
    appConsoleOutput().print(" active_low=");
    appConsoleOutput().println(cfg.activeLow ? "yes" : "no");
#endif
}

static bool updateStatusLedRuntime() {
    const uint32_t nowMs = millis();
    g_statusLedRuntime.setMode(deriveStatusLedMode(), nowMs);
    return g_statusLedRuntime.update(nowMs);
}

static void setStatusLedSensorError() {
    const uint32_t nowMs = millis();
    g_statusLedRuntime.setMode(TrackerStatusLedMode::SensorError, nowMs);
    g_statusLedRuntime.update(nowMs);
}

#endif // TRACKER_ENABLE_STATUS_LED

#if TRACKER_ENABLE_TAP_RUNTIME
#if TRACKER_ENABLE_TAP_DIAGNOSTICS
static const char* tapDiagnosticKindName(TapDiagnosticKind kind) {
    switch (kind) {
        case TapDiagnosticKind::HardwareConfigured: return "hardware_configured";
        case TapDiagnosticKind::HardwareDisabled: return "hardware_disabled";
        case TapDiagnosticKind::HardwareConfigureFailed: return "hardware_config_failed";
        case TapDiagnosticKind::RegisterVerifyFailed: return "register_verify_failed";
        case TapDiagnosticKind::SourceReadFailed: return "tap_src_read_failed";
        case TapDiagnosticKind::SourceObserved: return "tap_src";
        case TapDiagnosticKind::PhysicalTap: return "physical_tap";
        case TapDiagnosticKind::AccumulatorQueued: return "queued";
        case TapDiagnosticKind::SuppressedDuplicate: return "suppressed_duplicate";
        case TapDiagnosticKind::SuppressedLockout: return "suppressed_lockout";
        case TapDiagnosticKind::SuppressedBelowMin: return "suppressed_below_min";
        case TapDiagnosticKind::PacketReady: return "packet_ready";
        case TapDiagnosticKind::SlimeVrSent: return "slimevr_sent";
        case TapDiagnosticKind::SlimeVrNoServer: return "slimevr_no_server";
        case TapDiagnosticKind::SlimeVrSendFailed: return "slimevr_send_failed";
    }
    return "unknown";
}

static void emitTapDiagnosticLine(const char* line) {
#if TRACKER_ENABLE_SERIAL_CONSOLE
    appConsoleOutput().println(line);
#endif
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    (void)g_wifiRemoteConsole.writeDiagnosticLine(line);
#endif
}

static void onTapDiagnostic(const TapDiagnosticEvent& event, void*) {
    char line[240] = {};
    const char* const kind = tapDiagnosticKindName(event.kind);
    if (event.kind == TapDiagnosticKind::SourceObserved ||
        event.kind == TapDiagnosticKind::PhysicalTap ||
        event.kind == TapDiagnosticKind::AccumulatorQueued ||
        event.kind == TapDiagnosticKind::SuppressedDuplicate ||
        event.kind == TapDiagnosticKind::SuppressedLockout) {
        std::snprintf(line,
                      sizeof(line),
                      "# TAP_LOG ms=%lu event=%s raw=0x%02X tap=%u single=%u double=%u sign=%u x=%u y=%u z=%u physical=%u pending=%u manual=%u",
                      static_cast<unsigned long>(event.atMs),
                      kind,
                      static_cast<unsigned int>(event.rawSource),
                      event.tapDetected ? 1u : 0u,
                      event.singleTap ? 1u : 0u,
                      event.doubleTap ? 1u : 0u,
                      event.negative ? 1u : 0u,
                      event.x ? 1u : 0u,
                      event.y ? 1u : 0u,
                      event.z ? 1u : 0u,
                      static_cast<unsigned int>(event.physicalCount),
                      static_cast<unsigned int>(event.pendingCount),
                      event.manual ? 1u : 0u);
    } else {
        std::snprintf(line,
                      sizeof(line),
                      "# TAP_LOG ms=%lu event=%s value=%u pending=%u physical=%u manual=%u",
                      static_cast<unsigned long>(event.atMs),
                      kind,
                      static_cast<unsigned int>(event.packetValue),
                      static_cast<unsigned int>(event.pendingCount),
                      static_cast<unsigned int>(event.physicalCount),
                      event.manual ? 1u : 0u);
    }
    emitTapDiagnosticLine(line);
}
#endif // TRACKER_ENABLE_TAP_DIAGNOSTICS

static void setupTapRuntime() {
#if TRACKER_ENABLE_TAP_DIAGNOSTICS
    g_tapRuntime.setDiagnosticSink(onTapDiagnostic, nullptr);
#endif
    g_tapRuntime.begin(lsm, g_slimevrRuntime);

    TapRuntimeConfig cfg;
    cfg.enabled = TRACKER_ENABLE_TAP_RUNTIME != 0;
    cfg.hardwareDoubleTap = TRACKER_TAP_HARDWARE_DOUBLE_TAP != 0;
    cfg.slidingWindow = TRACKER_TAP_SLIDING_WINDOW != 0;
    cfg.sensorId = g_networkConfig.data.sensorId;
    cfg.physicalTapUserAction = g_networkConfig.tapUserAction();
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
    appConsoleOutput().print("# tap_runtime_enabled=");
    appConsoleOutput().print(cfg.enabled ? "yes" : "no");
    appConsoleOutput().print(" hardware=");
    appConsoleOutput().println(ok ? "ok" : "fail");
#endif
}

static bool updateTapRuntime() {
    return g_tapRuntime.update(millis());
}

#endif // TRACKER_ENABLE_TAP_RUNTIME


#if TRACKER_HAS_SERIAL_CONSOLE
static bool updateSerialConsoleRuntime() {
    if (!g_serialConsoleStream.hasPending()) {
        g_serialOutputDrainBackoffMs = TRACKER_SERIAL_OUTPUT_DRAIN_INTERVAL_MS;
        g_serialOutputStallStartMs = 0u;
        return false;
    }

    const uint32_t nowMs = millis();
    if (g_lastSerialOutputDrainMs != 0u &&
        static_cast<uint32_t>(nowMs - g_lastSerialOutputDrainMs) < g_serialOutputDrainBackoffMs) {
        return false;
    }
    g_lastSerialOutputDrainMs = nowMs;

    const size_t drained = g_serialConsoleStream.drain(TRACKER_SERIAL_OUTPUT_BYTES_PER_DRAIN);
    if (drained > 0u) {
        g_serialOutputDrainBackoffMs = TRACKER_SERIAL_OUTPUT_DRAIN_INTERVAL_MS;
        g_serialOutputStallStartMs = 0u;
        return true;
    }

    if (g_serialOutputStallStartMs == 0u) g_serialOutputStallStartMs = nowMs;
    if (TRACKER_SERIAL_OUTPUT_STALE_DISCARD_MS > 0u &&
        static_cast<uint32_t>(nowMs - g_serialOutputStallStartMs) >= TRACKER_SERIAL_OUTPUT_STALE_DISCARD_MS) {
        (void)g_serialConsoleStream.discardPending();
        ++g_serialOutputStaleDiscards;
        g_serialOutputDrainBackoffMs = TRACKER_SERIAL_OUTPUT_DRAIN_INTERVAL_MS;
        g_serialOutputStallStartMs = 0u;
        return true;
    }

    uint32_t nextBackoff = g_serialOutputDrainBackoffMs * 2u;
    if (nextBackoff < g_serialOutputDrainBackoffMs ||
        nextBackoff > TRACKER_SERIAL_OUTPUT_STALL_BACKOFF_MAX_MS) {
        nextBackoff = TRACKER_SERIAL_OUTPUT_STALL_BACKOFF_MAX_MS;
    }
    if (nextBackoff < TRACKER_SERIAL_OUTPUT_DRAIN_INTERVAL_MS) {
        nextBackoff = TRACKER_SERIAL_OUTPUT_DRAIN_INTERVAL_MS;
    }
    g_serialOutputDrainBackoffMs = nextBackoff;
    return false;
}
#endif

#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
static bool updateRemoteConsoleRuntime() {
    return g_wifiRemoteConsole.update(
        g_wifiManager.connected(),
        millis(),
        TRACKER_REMOTE_CONSOLE_BYTES_PER_LOOP,
        TRACKER_REMOTE_CONSOLE_OUTPUT_BYTES_PER_DRAIN
    );
}
#endif

static bool updateNetworkRuntime() {
    const uint32_t nowMs = millis();
#if TRACKER_NETWORK_RUNTIME_UPDATE_INTERVAL_MS > 0
    static bool s_networkRuntimeUpdateValid = false;
    static uint32_t s_lastNetworkRuntimeUpdateMs = 0;
    if (s_networkRuntimeUpdateValid &&
        static_cast<uint32_t>(nowMs - s_lastNetworkRuntimeUpdateMs) < TRACKER_NETWORK_RUNTIME_UPDATE_INTERVAL_MS) {
        return false;
    }
    s_networkRuntimeUpdateValid = true;
    s_lastNetworkRuntimeUpdateMs = nowMs;
#endif

    g_wifiManager.update(nowMs);
    updateAppSlimeVRRuntimeLiveState(nowMs);
    if (g_slimevrRuntime.enabled()) {
        refreshAppSlimeVRRuntimeStaticConfig(nowMs, false, true);
    }
    return g_slimevrRuntime.update(nowMs);
}


#if TRACKER_ENABLE_MOTION_LIGHT_SLEEP
static bool serverFoundForMotionLightSleep() {
    return g_slimevrRuntime.serverFound();
}

static void prepareMotionLightSleepRuntime() {
    // Do not call g_magRuntime.setEnabled(false, ...): that would mutate the
    // user's persistent mag configuration. This is a transient power state.
    (void)qmc.suspend();
    (void)lsmHub.stopMaster();
    g_magState.runtimeEnabled = false;
    g_magState.fifoArmed = false;
    g_preparedOutput.reset();

#if TRACKER_ENABLE_STATUS_LED
    g_statusLedRuntime.setMode(TrackerStatusLedMode::Off, millis());
    g_statusLedRuntime.update(millis());
#endif

#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    g_wifiRemoteConsole.suspend();
#endif
    g_slimevrRuntime.stop();
    g_wifiManager.suspend();
    g_wifiStation.radioOff();
}
#endif


#if TRACKER_HAS_CALIBRATION_AUTONOMY
static void calibrationAutonomyApplyConfigCallback(const TrackerConfig& promoted,
                                                     void* user) {
    (void)user;
    trackerApplyCalibrationCandidateToConfig(g_config, promoted);
    g_config.applyToImuCalibration(g_imuCal);
    g_config.applyToGyroTempComp(g_gyroTempComp);
    runtimeBiasReset(g_runtimeBias);
#if TRACKER_HAS_CALIBRATION_UI
    g_accelCalRunner.reset();
    g_gyroTempCapture.reset();
#endif
    g_lastSampleTimestampUs = 0;
    g_lastQualityFlags = 0;

    OrientationRuntimeResetDeps resetDeps;
    resetDeps.ahrs = &g_ahrs6dof;
    resetDeps.preparedOutput = &g_preparedOutput;
    resetDeps.resetDependentState = resetOrientationDependentState;
    (void)resetOrientationRuntime(
        resetDeps,
        "calibration_autonomy_apply",
        lsmFifo.stats().lastAssignedTimestampUs
    );
    g_slimevrRuntime.requestSensorInfoRefresh();
}

static void calibrationAutonomySet0022EnabledCallback(bool enabled, void* user) {
    (void)user;
    g_magRuntime.setAxisAlignmentLearningEnabled(enabled);
}

static void calibrationAutonomyReset0022EvidenceCallback(void* user) {
    (void)user;
    g_magRuntime.resetAxisAlignmentCandidate();
}

static bool calibrationAutonomyManualTransactionActiveCallback(void* user) {
    (void)user;
    if (g_magCalCollector.active()) return true;
#if TRACKER_HAS_CALIBRATION_UI
    if (g_gyroTempCapture.active()) return true;
#endif
    return false;
}

static CalibrationAutonomyDeps makeCalibrationAutonomyDeps() {
    CalibrationAutonomyDeps deps;
    deps.out = &appConsoleOutput();
    deps.config = &g_config;
    deps.configStore = &g_configStore;
    deps.autonomyStore = &g_calibrationAutonomyStore;
    deps.imuCal = &g_imuCal;
    deps.gyroTempComp = &g_gyroTempComp;
    deps.runtimeBias = &g_runtimeBias;
    deps.quality = &g_quality;
    deps.slimevr = &g_slimevrRuntime;
#if TRACKER_ENABLE_CALIBRATION_CANDIDATES
    deps.magAxisState = &g_magAxisAlignmentState;
#endif
    deps.magReliability = &g_lastMagFieldReliability;
    deps.callbacks.evaluateRealtimeGate =
        magControllerEvaluateDeferredServiceGateCallback;
    deps.callbacks.applyCalibrationConfig =
        calibrationAutonomyApplyConfigCallback;
    deps.callbacks.setWave0022RuntimeEnabled =
        calibrationAutonomySet0022EnabledCallback;
    deps.callbacks.resetWave0022Evidence =
        calibrationAutonomyReset0022EvidenceCallback;
    deps.callbacks.manualCalibrationTransactionActive =
        calibrationAutonomyManualTransactionActiveCallback;
    return deps;
}

static void setupCalibrationAutonomy() {
    g_calibrationAutonomy.begin(makeCalibrationAutonomyDeps(), millis());
}

static bool updateCalibrationAutonomyRuntime() {
    if (!g_calibrationAutonomy.deferredServiceRequired()) return false;
    return g_calibrationAutonomy.service(millis());
}

static bool calibrationBlocksMotionLightSleep() {
    return g_calibrationAutonomy.blocksMotionLightSleep();
}
#endif

static TrackerAppDeps makeTrackerAppDeps() {
    TrackerAppDeps deps;

    deps.bootstrap = makeTrackerBootstrapDeps();

    deps.runtime.out = &appConsoleOutput();
    deps.runtime.config = &g_config;
    deps.runtime.configLoadedFromNvs = &g_configLoadedFromNvs;
    deps.runtime.fifo = &lsmFifo;
    deps.runtime.quality = &g_quality;
    deps.runtime.ahrs = &g_ahrs6dof;
#if TRACKER_ENABLE_SERIAL_CLI
    deps.runtime.cli = &g_cli;
#endif
#if TRACKER_HAS_SERIAL_STREAM_STATE
    deps.runtime.streamState = &g_streamState;
#endif
    deps.runtime.perf = &g_perf;
#if TRACKER_HAS_RUNTIME_PROFILER
    deps.runtime.runtimeProfiler = &g_runtimeProfiler;
    deps.runtime.motionDiagnostics = &g_motionDiagnostics;
#endif
    deps.runtime.fifoEvents = &g_fifoEvents;
    deps.runtime.fifoRuntime = &g_fifoRuntime;
#if TRACKER_HAS_STATIC_TEST_STATE
    deps.runtime.staticTestRunner = &g_staticTestRunner;
#endif
#if TRACKER_HAS_RUNTIME_TEST_STATE
    deps.runtime.runtimeTestRunner = &g_runtimeTestRunner;
#endif
    deps.runtime.magState = &g_magState;
    deps.runtime.fifoIntCount = &g_fifoIntCount;
    deps.runtime.runtimeSamples = &g_runtimeSamples;
    deps.runtime.lastHeartbeatMs = &g_lastHeartbeatMs;
    deps.runtime.latestTempC = &g_latestTempC;
    deps.runtime.health = &g_trackerHealth;

    deps.callbacks.setupMagRuntimeController = setupMagRuntimeController;
#if TRACKER_HAS_SERIAL_CLI
    deps.callbacks.setupCommandInterface = setupCommandInterface;
#endif
    deps.callbacks.setupNetworkRuntime = setupNetworkRuntime;
#if TRACKER_HAS_CALIBRATION_AUTONOMY
    deps.callbacks.setupCalibrationAutonomy = setupCalibrationAutonomy;
#endif
#if TRACKER_HAS_MOTION_LIGHT_SLEEP
    deps.callbacks.resumeNetworkRuntime = resumeNetworkRuntime;
#endif
    deps.callbacks.publishHealthState = publishTrackerHealthState;
    deps.callbacks.updateNetworkRuntime = updateNetworkRuntime;
    deps.callbacks.updateMagDeferredRuntime = updateMagDeferredRuntime;
#if TRACKER_HAS_CALIBRATION_AUTONOMY
    deps.callbacks.updateCalibrationAutonomyRuntime = updateCalibrationAutonomyRuntime;
#endif
#if TRACKER_HAS_SERIAL_CONSOLE
    deps.callbacks.updateSerialConsoleRuntime = updateSerialConsoleRuntime;
#endif
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    deps.callbacks.updateRemoteConsoleRuntime = updateRemoteConsoleRuntime;
#endif
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
    deps.callbacks.detachFifoInterrupt = appDetachFifoInterruptCallback;
#if TRACKER_ENABLE_MOTION_LIGHT_SLEEP
    deps.callbacks.serverFoundForMotionSleep = serverFoundForMotionLightSleep;
#if TRACKER_HAS_CALIBRATION_AUTONOMY
    deps.callbacks.calibrationBlocksMotionSleep = calibrationBlocksMotionLightSleep;
#endif
    deps.callbacks.prepareMotionLightSleepRuntime = prepareMotionLightSleepRuntime;
#endif
    deps.callbacks.resetOrientationState = resetOrientationDependentState;
    deps.callbacks.setMagRuntimeEnabled = appSetMagRuntimeEnabledCallback;
    deps.callbacks.startMagRuntimeFromPreconfiguredFifo = appStartMagRuntimeFromPreconfiguredFifoCallback;
    deps.callbacks.processRawSample = processRuntimeRawSampleCallback;
    deps.callbacks.processMagSample = processRuntimeMagSampleCallback;
    deps.callbacks.recordFifoProcessTime = recordRuntimeFifoProcessTime;
    deps.callbacks.fifoCallbackUser = nullptr;

    deps.buffers.fifoRaw = g_fifoRaw;
    deps.buffers.fifoRawCapacity = FIFO_RAW_BUFFER_CAPACITY;
    deps.buffers.magRaw = g_magRaw;
    deps.buffers.magRawCapacity = MAG_RAW_BUFFER_CAPACITY;
    deps.buffers.fifoRuntimeRawQueue = g_fifoRuntimeRawQueue;
    deps.buffers.fifoRuntimeRawQueueFlags = g_fifoRuntimeRawQueueFlags;
    deps.buffers.fifoRuntimeRawQueueCapacity = FIFO_RUNTIME_RAW_QUEUE_CAPACITY;
    deps.buffers.fifoRuntimeMagQueue = g_fifoRuntimeMagQueue;
    deps.buffers.fifoRuntimeMagQueueCapacity = FIFO_RUNTIME_MAG_QUEUE_CAPACITY;

    deps.pins.int1 = PIN_LSM_INT1;

    deps.timing.serialBaud = SERIAL_BAUD_DEFAULT;
    deps.timing.fifoMaxWordsPerDrainDefault = FIFO_MAX_WORDS_PER_DRAIN_DEFAULT;
    deps.timing.fifoMaxDrainRoundsPerEventDefault = MAX_DRAIN_ROUNDS_PER_EVENT_DEFAULT;
    deps.timing.fifoNonblockingStatusPollIntervalUs = FIFO_NONBLOCKING_STATUS_POLL_INTERVAL_US;

    return deps;
}
