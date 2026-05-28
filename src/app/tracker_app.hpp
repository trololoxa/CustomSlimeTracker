#pragma once

#include <Arduino.h>

#include "defines.h"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "config/tracker_config_runtime.hpp"
#include "runtime/fifo_runtime_processor.hpp"
#if TRACKER_HAS_STATIC_TEST_STATE
#include "runtime/static_test_runner.hpp"
#endif
#if TRACKER_HAS_RUNTIME_TEST_STATE
#include "runtime/runtime_test_runner.hpp"
#endif
#include "runtime/tracker_runtime_types.hpp"
#include "runtime/tracker_health_state.hpp"
#include "runtime/mag_runtime_state.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/imu_quality.hpp"
#if TRACKER_HAS_SERIAL_STREAM_STATE || TRACKER_HAS_SERIAL_CLI
#include "serial/tracker_serial_context.hpp"
#endif
#if TRACKER_HAS_SERIAL_CLI
#include "serial/tracker_serial_commands.hpp"
#endif
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
    uint32_t startupDelayMs = TRACKER_STARTUP_DELAY_MS;
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
#if TRACKER_HAS_SERIAL_CLI
    TrackerSerialCommandInterface<>* cli = nullptr;
#endif
#if TRACKER_HAS_SERIAL_STREAM_STATE
    TrackerSerialStreamState* streamState = nullptr;
#endif
    TrackerPerfCounters* perf = nullptr;
    FifoInterruptEventSource* fifoEvents = nullptr;
    FifoRuntimeProcessor* fifoRuntime = nullptr;
#if TRACKER_HAS_STATIC_TEST_STATE
    StaticTestRunner* staticTestRunner = nullptr;
#endif
#if TRACKER_HAS_RUNTIME_TEST_STATE
    RuntimeTestRunner* runtimeTestRunner = nullptr;
#endif
    MagRuntimeState* magState = nullptr;
    volatile uint32_t* fifoIntCount = nullptr;
    uint32_t* runtimeSamples = nullptr;
    uint32_t* lastHeartbeatMs = nullptr;
    float* latestTempC = nullptr;
    TrackerHealthState* health = nullptr;
};

struct TrackerAppCallbacks {
    void (*setupMagRuntimeController)() = nullptr;
    void (*setupCommandInterface)() = nullptr;
    void (*setupNetworkRuntime)() = nullptr;
    bool (*updateNetworkRuntime)() = nullptr;
    bool (*updateRemoteConsoleRuntime)() = nullptr;
    void (*setupTapRuntime)() = nullptr;
    bool (*updateTapRuntime)() = nullptr;
    void (*setupStatusLedRuntime)() = nullptr;
    bool (*updateStatusLedRuntime)() = nullptr;
    void (*setupBatteryRuntime)() = nullptr;
    bool (*updateBatteryRuntime)() = nullptr;
    void (*setStatusLedSensorError)() = nullptr;
    void (*publishHealthState)(const TrackerHealthSnapshot& health) = nullptr;
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
    bool initLsmWithRetries();
    bool initFifoWithRetries();
    void beginSensorStartupRecovery(TrackerHealthFaultCode code, const char* message);
    bool updateSensorStartupRecovery(uint32_t nowMs);
    void finishSensorStartupRecoverySuccess();
    void setupSensorRuntime();
    void enterFatalDegraded(TrackerHealthFaultCode code, const char* message);
    void publishHealthState();
    void call(void (*callback)());
    bool processFifoRuntime();
    bool maybeIdleYield(bool anyWork);
    static bool callBool(bool (*callback)());
    void startMagFromConfig(Stream& out);

    TrackerAppDeps deps_;
    bool sensorRuntimeReady_ = false;
    bool sensorStartupRecoveryActive_ = false;
    bool sensorStartupHardFailed_ = false;
    TrackerHealthFaultCode pendingSensorFaultCode_ = TrackerHealthFaultCode::None;
    uint32_t sensorStartupRecoveryStartedMs_ = 0;
    uint32_t nextSensorStartupRecoveryMs_ = 0;
    uint16_t sensorStartupRecoveryAttempts_ = 0;
    char pendingSensorFaultMessage_[64] = {};
};

} // namespace tracker
