#pragma once

// Common app hooks: lightweight counters, logging glue, FIFO wait glue, bootstrap deps, and runtime bias callbacks.
// This file is included by app/tracker_app_hooks.hpp after shared app dependencies.

static Stream& appConsoleOutput() {
#if TRACKER_HAS_SERIAL_CONSOLE
    return g_serialConsoleStream;
#else
    return Serial;
#endif
}

static void recordFifoProcessTime(uint32_t dtUs) {
#if TRACKER_HAS_HOTPATH_PERF
    g_perf.fifoProcessCalls++;
    g_perf.fifoProcessSumUs += dtUs;
    if (dtUs > g_perf.fifoProcessMaxUs) {
        g_perf.fifoProcessMaxUs = dtUs;
    }
#else
    (void)dtUs;
#endif
}

static void resetLogCountersHook(void* user) {
    (void)user;
#if TRACKER_ENABLE_MACHINE_LOG
    machineLogResetCounters(g_logCounters, g_lastBiasLogEmitUs);
#endif
}

static void resetLogPipelineHook(bool countPendingAsDropped, void* user) {
    (void)user;
#if TRACKER_ENABLE_MACHINE_LOG
    g_machineLogDeferred.reset(countPendingAsDropped);
#else
    (void)countPendingAsDropped;
#endif
}

static void emitMachineLogHeader(Stream& out, void* user) {
    (void)user;
#if TRACKER_ENABLE_MACHINE_LOG
    machineLogEmitHeader(out, g_logState, g_config);
#else
    (void)out;
#endif
}

static void emitLogStateEvent(const char* state, const char* reason, uint64_t tUs, uint32_t flags, float confidence) {
#if TRACKER_ENABLE_MACHINE_LOG
    (void)g_machineLogDeferred.enqueueState(state, reason, tUs, flags, confidence);
#else
    (void)state;
    (void)reason;
    (void)tUs;
    (void)flags;
    (void)confidence;
#endif
}

static void resetFifoRuntimeCounters() {
    noInterrupts();
    g_fifoIntCount = 0;
    g_fifoLastIrqUs = micros();
    interrupts();

    g_fifoEvents.reset();
    g_lastSampleTimestampUs = 0;
    g_lastQualityFlags = 0;
}

static bool consumeFifoInterruptEvent(uint32_t timeoutMs) {
    return g_fifoEvents.consume(timeoutMs, g_config.data.fifo.watermarkWords);
}

#if TRACKER_ENABLE_CALIBRATION_COMMANDS
static bool waitFifoEventForCalibration(uint32_t timeoutMs, void* user) {
    (void)user;
    return consumeFifoInterruptEvent(timeoutMs);
}

#endif
static TrackerBootstrapDeps makeTrackerBootstrapDeps() {
    TrackerBootstrapDeps deps;
    deps.out = &appConsoleOutput();
    deps.spi = &SPI;
    deps.lsmBus = &lsmBus;
    deps.lsm = &lsm;
    deps.fifo = &lsmFifo;
    deps.config = &g_config;
    deps.configStore = &g_configStore;
    deps.configLoadedFromNvs = &g_configLoadedFromNvs;
    deps.imuCal = &g_imuCal;
    deps.gyroTempComp = &g_gyroTempComp;
    deps.runtimeBias = &g_runtimeBias;
    deps.quality = &g_quality;
    deps.ahrs = &g_ahrs6dof;
#if TRACKER_HAS_SERIAL_STREAM_STATE
    deps.streamState = &g_streamState;
#endif
#if TRACKER_ENABLE_CALIBRATION_COMMANDS
    deps.calibrationIo = &g_calIo;
    deps.calibrationRawBuffer = g_fifoRaw;
    deps.calibrationRawBufferCapacity = FIFO_RAW_BUFFER_CAPACITY;
    deps.waitForCalibrationFifoEvent = waitFifoEventForCalibration;
    deps.waitForCalibrationFifoEventUser = nullptr;
#endif
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
