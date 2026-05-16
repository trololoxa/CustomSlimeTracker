#include "app/tracker_app.hpp"

#include "runtime/output_runtime.hpp"

namespace tracker {

void TrackerApp::begin(const TrackerAppDeps& deps) {
    deps_ = deps;
}

void TrackerApp::setup() {
    if (!ready()) return;

    Serial.begin(deps_.timing.serialBaud);
    sleep(2);
    deps_.runtime.perf->reset(millis());
    delay(deps_.timing.startupDelayMs);

    Stream& out = *deps_.runtime.out;
    out.println();
    out.println("==============================================================================");
    out.println("ESP32-C3 + LSM6DSV COMMAND TRACKER FIRMWARE");
    out.println("==============================================================================");

    trackerBootstrapLoadConfigAndApplyRuntime(deps_.bootstrap);

    out.print("# config_loaded_from_nvs=");
    out.println(*deps_.runtime.configLoadedFromNvs ? "yes" : "no");
    out.print("# config_valid=");
    out.println(deps_.runtime.config->validate() ? "yes" : "no");

    pinMode(deps_.pins.int1, INPUT);

    if (!trackerBootstrapInitLsm(deps_.bootstrap)) {
        fatal("# fatal: LSM init failed");
    }

    if (!trackerBootstrapInitFifo(deps_.bootstrap)) {
        fatal("# fatal: FIFO init failed");
    }

    deps_.runtime.fifoEvents->begin(
        deps_.runtime.fifoIntCount,
        deps_.runtime.fifo,
        deps_.runtime.perf,
        deps_.timing.fifoNonblockingStatusPollIntervalUs
    );

    deps_.runtime.fifoRuntime->begin(
        deps_.runtime.fifoEvents,
        deps_.runtime.fifo,
        deps_.buffers.fifoRaw,
        deps_.buffers.fifoRawCapacity,
        deps_.buffers.magRaw,
        deps_.buffers.magRawCapacity,
        deps_.callbacks.processRawSample,
        deps_.callbacks.processMagSample,
        deps_.callbacks.recordFifoProcessTime,
        deps_.callbacks.fifoCallbackUser
    );

    trackerBootstrapSetupCalibrationIo(deps_.bootstrap);
    call(deps_.callbacks.setupMagRuntimeController);
    call(deps_.callbacks.setupNetworkRuntime);
    call(deps_.callbacks.setupCommandInterface);
    call(deps_.callbacks.resetFifoRuntimeCounters);
    call(deps_.callbacks.attachFifoInterrupt);

    deps_.runtime.fifo->resetFifo();
    deps_.runtime.fifo->resetTimestampReconstruction(0);
    deps_.runtime.quality->reset();
    deps_.runtime.quality->syncFifoStats(deps_.runtime.fifo->stats());
    deps_.runtime.ahrs->reset();
    if (deps_.callbacks.resetOrientationState != nullptr) {
        deps_.callbacks.resetOrientationState("startup", 0, false);
    }

    startMagFromConfig(out);

    out.println("# OK INT1 attached: FIFO_WTM/FIFO_OVR/FIFO_FULL, RISING");
    out.println("# Type: help");
    out.println("==============================================================================");
}

void TrackerApp::loop() {
    if (!ready()) return;

    RuntimeLoopTimingSample timing;
    const uint32_t loopStartUs = micros();
    uint32_t sectionStartUs = 0;

#if TRACKER_ENABLE_SERIAL_CLI
    sectionStartUs = micros();
    deps_.runtime.cli->poll(TRACKER_CLI_BYTES_PER_LOOP);
    timing.cliUs += micros() - sectionStartUs;
#endif

    sectionStartUs = micros();
    processFifoRuntime();
    timing.fifoUs = micros() - sectionStartUs;

    sectionStartUs = micros();
    call(deps_.callbacks.updateNetworkRuntime);
    timing.networkUs = micros() - sectionStartUs;

#if TRACKER_ENABLE_SERIAL_CLI
    sectionStartUs = micros();
    deps_.runtime.cli->poll(TRACKER_CLI_BYTES_PER_LOOP);
    timing.cliUs += micros() - sectionStartUs;
#endif

    sectionStartUs = micros();
    maybePrintBootHeartbeat(
        *deps_.runtime.out,
        *deps_.runtime.streamState,
        deps_.runtime.staticTestRunner->active() || deps_.runtime.runtimeTestRunner->active(),
        millis(),
        *deps_.runtime.lastHeartbeatMs,
        *deps_.runtime.runtimeSamples,
        *deps_.runtime.fifoIntCount,
        *deps_.runtime.latestTempC,
        deps_.runtime.magState->samples
    );
    timing.heartbeatUs = micros() - sectionStartUs;
    timing.loopUs = micros() - loopStartUs;

    deps_.runtime.runtimeTestRunner->recordLoopTiming(timing);
    deps_.runtime.runtimeTestRunner->update(millis(), *deps_.runtime.out);
}

bool TrackerApp::ready() const {
    return deps_.runtime.out != nullptr &&
           deps_.runtime.config != nullptr &&
           deps_.runtime.configLoadedFromNvs != nullptr &&
           deps_.runtime.fifo != nullptr &&
           deps_.runtime.quality != nullptr &&
           deps_.runtime.ahrs != nullptr &&
           deps_.runtime.cli != nullptr &&
           deps_.runtime.streamState != nullptr &&
           deps_.runtime.perf != nullptr &&
           deps_.runtime.fifoEvents != nullptr &&
           deps_.runtime.fifoRuntime != nullptr &&
           deps_.runtime.staticTestRunner != nullptr &&
           deps_.runtime.magState != nullptr &&
           deps_.runtime.fifoIntCount != nullptr &&
           deps_.runtime.runtimeSamples != nullptr &&
           deps_.runtime.lastHeartbeatMs != nullptr &&
           deps_.runtime.latestTempC != nullptr &&
           deps_.buffers.fifoRaw != nullptr &&
           deps_.buffers.fifoRawCapacity > 0u &&
           deps_.buffers.magRaw != nullptr &&
           deps_.buffers.magRawCapacity > 0u &&
           deps_.callbacks.processRawSample != nullptr &&
           deps_.callbacks.processMagSample != nullptr &&
           deps_.callbacks.recordFifoProcessTime != nullptr;
}

void TrackerApp::fatal(const char* message) {
    if (deps_.runtime.out != nullptr) deps_.runtime.out->println(message);
    while (true) delay(1000);
}

void TrackerApp::call(void (*callback)()) {
    if (callback != nullptr) callback();
}

void TrackerApp::serviceRuntimeForBlockingCommand() {
    if (!ready()) return;

    processFifoRuntime();
    call(deps_.callbacks.updateNetworkRuntime);
    deps_.runtime.runtimeTestRunner->update(millis(), *deps_.runtime.out);
}

void TrackerApp::processFifoRuntime() {
    const TrackerConfig& config = *deps_.runtime.config;
    const uint16_t watermarkWords = config.data.fifo.watermarkWords;
    const uint16_t maxWords = config.data.fifo.maxWordsPerDrain > 0
        ? config.data.fifo.maxWordsPerDrain
        : deps_.timing.fifoMaxWordsPerDrainDefault;
    const uint8_t maxRounds = config.data.fifo.maxDrainRoundsPerEvent > 0
        ? config.data.fifo.maxDrainRoundsPerEvent
        : deps_.timing.fifoMaxDrainRoundsPerEventDefault;

    deps_.runtime.fifoRuntime->process(watermarkWords, maxWords, maxRounds, *deps_.runtime.out);
}

void TrackerApp::startMagFromConfig(Stream& out) {
    TrackerConfig& config = *deps_.runtime.config;
    if (!config.data.magCal.driverEnabled) return;

    out.println("# mag enabled in config; starting QMC6309 FIFO stream");
    const bool ok = deps_.callbacks.setMagRuntimeEnabled != nullptr &&
                    deps_.callbacks.setMagRuntimeEnabled(true, false);
    if (!ok) {
        out.println("# WARN mag startup failed; continuing 6DoF without mag");
        config.data.magCal.driverEnabled = false;
        config.updateCrc();
    }
}

} // namespace tracker
