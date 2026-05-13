#pragma once

// Common app hooks: lightweight counters, logging glue, FIFO wait glue, bootstrap deps, and runtime bias callbacks.
// This file is included by app/tracker_app_hooks.hpp after shared app dependencies.

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
