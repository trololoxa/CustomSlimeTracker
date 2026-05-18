#pragma once

#include <Arduino.h>

#include "defines.h"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "config/tracker_config_runtime.hpp"
#include "runtime/fifo_runtime_processor.hpp"
#include "runtime/static_test_runner.hpp"
#include "runtime/runtime_test_runner.hpp"
#include "runtime/tracker_runtime_types.hpp"
#include "runtime/mag_runtime_state.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/imu_quality.hpp"
#include "serial/tracker_serial_commands.hpp"
#include "app/tracker_bootstrap.hpp"

namespace tracker {

struct TrackerAppPins {
    int int1 = -1;
};

struct TrackerAppBuffers {
    Lsm6dsv::RawSample* fifoRaw = nullptr;
    size_t fifoRawCapacity = 0;
    Lsm6dsvFifoReader::MagRawSample* magRaw = nullptr;
    size_t magRawCapacity = 0;
};

struct TrackerAppTimingConfig {
    uint32_t serialBaud = cfg::SERIAL_BAUD;
    uint32_t startupDelayMs = 300;
    uint16_t fifoMaxWordsPerDrainDefault = cfg::FIFO_MAX_WORDS_PER_DRAIN;
    uint8_t fifoMaxDrainRoundsPerEventDefault = cfg::FIFO_MAX_DRAIN_ROUNDS_PER_EVENT;
    uint32_t fifoNonblockingStatusPollIntervalUs = cfg::FIFO_NONBLOCKING_STATUS_POLL_INTERVAL_US;
};

struct TrackerAppRuntimeObjects {
    Stream* out = nullptr;
    TrackerConfig* config = nullptr;
    bool* configLoadedFromNvs = nullptr;
    Lsm6dsvFifoReader* fifo = nullptr;
    ImuQualityMonitor* quality = nullptr;
    Ahrs6Dof* ahrs = nullptr;
    TrackerSerialCommandInterface<>* cli = nullptr;
    TrackerSerialStreamState* streamState = nullptr;
    TrackerPerfCounters* perf = nullptr;
    FifoInterruptEventSource* fifoEvents = nullptr;
    FifoRuntimeProcessor* fifoRuntime = nullptr;
    StaticTestRunner* staticTestRunner = nullptr;
    RuntimeTestRunner* runtimeTestRunner = nullptr;
    MagRuntimeState* magState = nullptr;
    volatile uint32_t* fifoIntCount = nullptr;
    uint32_t* runtimeSamples = nullptr;
    uint32_t* lastHeartbeatMs = nullptr;
    float* latestTempC = nullptr;
};

struct TrackerAppCallbacks {
    void (*setupMagRuntimeController)() = nullptr;
    void (*setupCommandInterface)() = nullptr;
    void (*setupNetworkRuntime)() = nullptr;
    void (*updateNetworkRuntime)() = nullptr;
    void (*setupTapRuntime)() = nullptr;
    void (*updateTapRuntime)() = nullptr;
    void (*setupStatusLedRuntime)() = nullptr;
    void (*updateStatusLedRuntime)() = nullptr;
    void (*setupBatteryRuntime)() = nullptr;
    void (*updateBatteryRuntime)() = nullptr;
    void (*setStatusLedSensorError)() = nullptr;
    void (*resetFifoRuntimeCounters)() = nullptr;
    void (*attachFifoInterrupt)() = nullptr;
    void (*resetOrientationState)(const char* reason, uint64_t timestampUs, bool rebaseAhrsTimebase) = nullptr;
    bool (*setMagRuntimeEnabled)(bool enabled, bool persist) = nullptr;
    FifoRuntimeSampleCallback processRawSample = nullptr;
    FifoRuntimeMagCallback processMagSample = nullptr;
    FifoRuntimeRecordTimeCallback recordFifoProcessTime = nullptr;
    void* fifoCallbackUser = nullptr;
};

struct TrackerAppDeps {
    TrackerBootstrapDeps bootstrap;
    TrackerAppRuntimeObjects runtime;
    TrackerAppCallbacks callbacks;
    TrackerAppBuffers buffers;
    TrackerAppPins pins;
    TrackerAppTimingConfig timing;
};

class TrackerApp {
public:
    void begin(const TrackerAppDeps& deps);
    void setup();
    void loop();

    // Services the non-CLI runtime while a blocking command-driven calibration
    // flow is active. This keeps FIFO, mag runtime, Wi-Fi and SlimeVR alive
    // without recursively polling the serial parser.
    void serviceRuntimeForBlockingCommand();

private:
    bool ready() const;
    void fatal(const char* message);
    void call(void (*callback)());
    void processFifoRuntime();
    void startMagFromConfig(Stream& out);

    TrackerAppDeps deps_;
};

} // namespace tracker
