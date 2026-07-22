#include "app/tracker_app.hpp"

#include "runtime/output_runtime.hpp"

#include <cstring>

#if TRACKER_ENABLE_MOTION_LIGHT_SLEEP
#include <driver/gpio.h>
#include <esp_sleep.h>
#endif

namespace tracker {

namespace {

constexpr uint8_t SENSOR_INIT_ATTEMPTS = 5;
constexpr uint32_t SENSOR_INIT_RETRY_DELAY_MS = 80;
constexpr uint8_t FIFO_INIT_ATTEMPTS = 3;
constexpr uint32_t FIFO_INIT_RETRY_DELAY_MS = 50;
constexpr uint32_t SENSOR_STARTUP_RECOVERY_INTERVAL_MS = 5000;
constexpr uint32_t SENSOR_STARTUP_HARD_FAIL_DELAY_MS = 60000;
constexpr uint32_t SENSOR_STARTUP_HARD_FAIL_RECOVERY_INTERVAL_MS = 30000;

} // namespace

void TrackerApp::begin(const TrackerAppDeps& deps) {
    deps_ = deps;
}

void TrackerApp::setup() {
#if TRACKER_HAS_SERIAL_CONSOLE
    Serial.begin(deps_.timing.serialBaud);
#endif

    if (!ready()) {
#if TRACKER_HAS_SERIAL_CONSOLE
        delay(50);
        Serial.println();
        Serial.println("# FATAL tracker_app_ready=no");
        Serial.println("# FATAL tracker app dependencies are incomplete; firmware will idle instead of reset-looping");
#endif
        return;
    }

    sensorRuntimeReady_ = false;
#if TRACKER_HAS_MOTION_LIGHT_SLEEP
    motionLightSleep_.setServerAbsenceTimeoutMs(TRACKER_MOTION_LIGHT_SLEEP_SERVER_ABSENCE_MS);
    motionLightSleepManualRequested_ = false;
#endif
    deps_.runtime.health->reset();

#if TRACKER_ENABLE_BOOT_DELAY
    delay(TRACKER_BOOT_SERIAL_SETTLE_DELAY_MS);
#endif

    deps_.runtime.perf->reset(millis());
#if TRACKER_HAS_RUNTIME_PROFILER
    if (deps_.runtime.runtimeProfiler != nullptr) {
        deps_.runtime.runtimeProfiler->begin(millis());
    }
    if (deps_.runtime.motionDiagnostics != nullptr) {
        deps_.runtime.motionDiagnostics->begin(millis());
    }
#endif

#if TRACKER_HAS_SERIAL_CONSOLE
    delay(deps_.timing.startupDelayMs);
#endif

    Stream& out = *deps_.runtime.out;
#if TRACKER_ENABLE_BOOT_BANNER
    out.println();
    out.println("==============================================================================");
    out.println("ESP32-C3 + LSM6DSV COMMAND TRACKER FIRMWARE");
    out.print("# build_profile=");
    out.print(trackerBuildProfileName());
    out.print(" cli_level=");
    out.println(trackerCliLevelName());
    out.println("==============================================================================");
#endif

    trackerBootstrapLoadConfigAndApplyRuntime(deps_.bootstrap);

#if TRACKER_HAS_SERIAL_CONSOLE
    out.print("# config_loaded_from_nvs=");
    out.println(*deps_.runtime.configLoadedFromNvs ? "yes" : "no");
    out.print("# config_valid=");
    out.println(deps_.runtime.config->validate() ? "yes" : "no");
#endif

    call(deps_.callbacks.setupStatusLedRuntime);
    call(deps_.callbacks.setupBatteryRuntime);

    pinMode(deps_.pins.int1, INPUT);

    if (!initLsmWithRetries()) {
        beginSensorStartupRecovery(TrackerHealthFaultCode::LsmInitFailed,
                                   "LSM6DSV init failed after retries");
    } else if (!initFifoWithRetries()) {
        beginSensorStartupRecovery(TrackerHealthFaultCode::FifoInitFailed,
                                   "FIFO init failed after retries");
    } else {
        setupSensorRuntime();
        sensorRuntimeReady_ = true;
    }

    call(deps_.callbacks.setupNetworkRuntime);
    publishHealthState();

    if (sensorRuntimeReady_) {
        call(deps_.callbacks.setupTapRuntime);
    }
    call(deps_.callbacks.setupCommandInterface);

#if TRACKER_HAS_SERIAL_CONSOLE
    if (sensorRuntimeReady_) {
        out.println("# OK INT1 attached: FIFO_WTM/FIFO_OVR/FIFO_FULL, RISING");
    } else if (sensorStartupRecoveryActive_) {
        out.print("# WARN sensor_startup_recovery=pending code=");
        out.print(trackerHealthFaultCodeName(pendingSensorFaultCode_));
        out.print(" message=");
        out.println(pendingSensorFaultMessage_);
    } else {
        out.print("# WARN tracker_degraded_no_imu=yes code=");
        out.print(trackerHealthFaultCodeName(deps_.runtime.health->faultCode()));
        out.print(" message=");
        out.println(deps_.runtime.health->message());
    }
#if TRACKER_HAS_SERIAL_CLI
    out.println("# Type: help");
#endif
    out.println("==============================================================================");
#endif
}

void TrackerApp::loop() {
    if (!ready()) {
        delay(10);
        return;
    }

#if TRACKER_ENABLE_LOOP_TIMING
    RuntimeLoopTimingSample timing;
    const uint32_t loopStartUs = micros();
    uint32_t sectionStartUs = 0;
#endif

#if TRACKER_HAS_RUNTIME_PROFILER
    const uint32_t profilerNowMs = millis();
    RuntimeProfiler* profiler = deps_.runtime.runtimeProfiler;
    const bool profilerActive = profiler != nullptr && profiler->enabled();
    const uint32_t profilerLoopStartUs = profilerActive ? micros() : 0;
    uint32_t profilerSectionStartUs = 0;
#endif

    bool remoteConsoleWorked = false;
    bool serialConsoleWorked = false;
#if TRACKER_ENABLE_LOOP_TIMING
    sectionStartUs = micros();
#endif
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) profilerSectionStartUs = micros();
#endif
    const bool sensorRecoveryWorked = updateSensorStartupRecovery(millis());
    const bool fifoWorked = sensorRuntimeReady_ ? processFifoRuntime() : false;
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) {
        profiler->record(RuntimeProfiler::Section::Fifo, micros() - profilerSectionStartUs, sensorRecoveryWorked || fifoWorked, profilerNowMs);
    }
#endif
#if TRACKER_ENABLE_LOOP_TIMING
    timing.fifoUs = micros() - sectionStartUs;
    timing.fifoWorked = fifoWorked;
#endif

#if TRACKER_ENABLE_LOOP_TIMING
    sectionStartUs = micros();
#endif
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) profilerSectionStartUs = micros();
#endif
    const bool batteryWorked = callBool(deps_.callbacks.updateBatteryRuntime);
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) {
        profiler->record(RuntimeProfiler::Section::Battery, micros() - profilerSectionStartUs, batteryWorked, profilerNowMs);
        profilerSectionStartUs = micros();
    }
#endif
    const bool networkWorked = callBool(deps_.callbacks.updateNetworkRuntime);
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) {
        profiler->record(RuntimeProfiler::Section::Network, micros() - profilerSectionStartUs, networkWorked, profilerNowMs);
        profilerSectionStartUs = micros();
    }
#endif
    const bool tapWorked = sensorRuntimeReady_ ? callBool(deps_.callbacks.updateTapRuntime) : false;
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) {
        profiler->record(RuntimeProfiler::Section::Tap, micros() - profilerSectionStartUs, tapWorked, profilerNowMs);
        profilerSectionStartUs = micros();
    }
#endif
    const bool ledWorked = callBool(deps_.callbacks.updateStatusLedRuntime);
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) {
        profiler->record(RuntimeProfiler::Section::Led, micros() - profilerSectionStartUs, ledWorked, profilerNowMs);
    }
#endif
#if TRACKER_ENABLE_LOOP_TIMING
    timing.networkUs = micros() - sectionStartUs;
    timing.batteryWorked = batteryWorked;
    timing.networkWorked = networkWorked;
    timing.tapWorked = tapWorked;
    timing.ledWorked = ledWorked;
#endif

    // Tracking delivery has priority over all diagnostic I/O. CLI command
    // dispatch, telnet socket work and output drains run only after FIFO/AHRS
    // and SlimeVR UDP have been serviced for this loop.
#if TRACKER_ENABLE_LOOP_TIMING
    sectionStartUs = micros();
#endif
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) profilerSectionStartUs = micros();
#endif
#if TRACKER_HAS_SERIAL_CLI
    deps_.runtime.cli->poll(TRACKER_CLI_BYTES_PER_LOOP);
#endif
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) {
        profiler->record(RuntimeProfiler::Section::Cli, micros() - profilerSectionStartUs, false, profilerNowMs);
        profilerSectionStartUs = micros();
    }
#endif
    remoteConsoleWorked = callBool(deps_.callbacks.updateRemoteConsoleRuntime);
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) {
        profiler->record(RuntimeProfiler::Section::RemoteConsole, micros() - profilerSectionStartUs, remoteConsoleWorked, profilerNowMs);
        profilerSectionStartUs = micros();
    }
#endif
    serialConsoleWorked = callBool(deps_.callbacks.updateSerialConsoleRuntime);
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) {
        profiler->record(RuntimeProfiler::Section::Cli, micros() - profilerSectionStartUs, serialConsoleWorked, profilerNowMs);
    }
#endif
#if TRACKER_ENABLE_LOOP_TIMING
    timing.cliUs += micros() - sectionStartUs;
#endif

#if TRACKER_ENABLE_MOTION_LIGHT_SLEEP
    // Light sleep is deliberately entered after the network state machine has
    // observed the current server state, but before the next ordinary loop
    // section. The call blocks until physical motion wakes the tracker.
    if (maybeEnterMotionLightSleep()) {
        return;
    }
#endif

#if TRACKER_HAS_SERIAL_CLI && TRACKER_CLI_SECOND_POLL_ENABLED
#if TRACKER_ENABLE_LOOP_TIMING
    sectionStartUs = micros();
#endif
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) profilerSectionStartUs = micros();
#endif
    deps_.runtime.cli->poll(TRACKER_CLI_BYTES_PER_LOOP);
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) {
        profiler->record(RuntimeProfiler::Section::Cli, micros() - profilerSectionStartUs, false, profilerNowMs);
    }
#endif
#if TRACKER_ENABLE_LOOP_TIMING
    timing.cliUs += micros() - sectionStartUs;
#endif
#endif

    bool heartbeatPrinted = false;
#if TRACKER_HAS_BOOT_HEARTBEAT
#if TRACKER_ENABLE_LOOP_TIMING
    sectionStartUs = micros();
#endif
    heartbeatPrinted = maybePrintBootHeartbeat(
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
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) {
        profiler->record(RuntimeProfiler::Section::Heartbeat, micros() - sectionStartUs, heartbeatPrinted, profilerNowMs);
    }
#endif
#if TRACKER_ENABLE_LOOP_TIMING
    timing.heartbeatUs = micros() - sectionStartUs;
    timing.heartbeatWorked = heartbeatPrinted;
#endif
#endif

    const bool anyWork = sensorRecoveryWorked ||
                         fifoWorked ||
                         serialConsoleWorked ||
                         batteryWorked ||
                         networkWorked ||
                         tapWorked ||
                         ledWorked ||
                         remoteConsoleWorked ||
                         heartbeatPrinted;
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) profilerSectionStartUs = micros();
#endif
    const bool idleYielded = maybeIdleYield(anyWork);
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) {
        profiler->record(RuntimeProfiler::Section::IdleYield, micros() - profilerSectionStartUs, idleYielded, profilerNowMs);
    }
#endif

#if TRACKER_ENABLE_LOOP_TIMING
    timing.loopUs = micros() - loopStartUs;
    timing.anyWork = anyWork;
    timing.idleYielded = idleYielded;
#else
    (void)idleYielded;
#endif
#if TRACKER_HAS_RUNTIME_PROFILER
    if (profilerActive) {
        profiler->record(RuntimeProfiler::Section::Loop, micros() - profilerLoopStartUs, anyWork, profilerNowMs);
    }
#endif

#if TRACKER_HAS_RUNTIME_TEST
#if TRACKER_ENABLE_LOOP_TIMING
    deps_.runtime.runtimeTestRunner->recordLoopTiming(timing);
#endif
    deps_.runtime.runtimeTestRunner->update(millis(), *deps_.runtime.out);
#endif
}

bool TrackerApp::ready() const {
    return deps_.runtime.out != nullptr &&
           deps_.runtime.config != nullptr &&
           deps_.runtime.configLoadedFromNvs != nullptr &&
           deps_.runtime.fifo != nullptr &&
           deps_.runtime.quality != nullptr &&
           deps_.runtime.ahrs != nullptr &&
#if TRACKER_HAS_SERIAL_CLI
           deps_.runtime.cli != nullptr &&
#endif
#if TRACKER_HAS_SERIAL_STREAM_STATE
           deps_.runtime.streamState != nullptr &&
#endif
           deps_.runtime.perf != nullptr &&
#if TRACKER_HAS_RUNTIME_PROFILER
           // Runtime profiler/motion diagnostics are optional diagnostic sinks.
           // They must never make the tracker unbootable if profile/source-filter
           // wiring drifts; the loop and commands already null-check them.
#endif
           deps_.runtime.fifoEvents != nullptr &&
           deps_.runtime.fifoRuntime != nullptr &&
#if TRACKER_HAS_STATIC_TEST_STATE
           deps_.runtime.staticTestRunner != nullptr &&
#endif
#if TRACKER_HAS_RUNTIME_TEST_STATE
           deps_.runtime.runtimeTestRunner != nullptr &&
#endif
           deps_.runtime.magState != nullptr &&
           deps_.runtime.fifoIntCount != nullptr &&
           deps_.runtime.runtimeSamples != nullptr &&
           deps_.runtime.lastHeartbeatMs != nullptr &&
           deps_.runtime.latestTempC != nullptr &&
           deps_.runtime.health != nullptr &&
           deps_.buffers.fifoRaw != nullptr &&
           deps_.buffers.fifoRawCapacity > 0u &&
           deps_.buffers.magRaw != nullptr &&
           deps_.buffers.magRawCapacity > 0u &&
           deps_.buffers.fifoRuntimeRawQueue != nullptr &&
           deps_.buffers.fifoRuntimeRawQueueFlags != nullptr &&
           deps_.buffers.fifoRuntimeRawQueueCapacity > 0u &&
           deps_.buffers.fifoRuntimeMagQueue != nullptr &&
           deps_.buffers.fifoRuntimeMagQueueCapacity > 0u &&
           deps_.callbacks.processRawSample != nullptr &&
           deps_.callbacks.processMagSample != nullptr &&
           deps_.callbacks.recordFifoProcessTime != nullptr;
}

bool TrackerApp::initLsmWithRetries() {
    for (uint8_t attempt = 1; attempt <= SENSOR_INIT_ATTEMPTS; ++attempt) {
        if (trackerBootstrapInitLsm(deps_.bootstrap)) return true;
#if TRACKER_HAS_SERIAL_CONSOLE
        if (attempt < SENSOR_INIT_ATTEMPTS && deps_.runtime.out) {
            deps_.runtime.out->print("# WARN LSM init retry ");
            deps_.runtime.out->print(attempt);
            deps_.runtime.out->print('/');
            deps_.runtime.out->println(SENSOR_INIT_ATTEMPTS);
        }
#endif
        if (attempt < SENSOR_INIT_ATTEMPTS) delay(SENSOR_INIT_RETRY_DELAY_MS);
    }
    return false;
}

bool TrackerApp::initFifoWithRetries() {
    for (uint8_t attempt = 1; attempt <= FIFO_INIT_ATTEMPTS; ++attempt) {
        if (trackerBootstrapInitFifo(deps_.bootstrap)) return true;
#if TRACKER_HAS_SERIAL_CONSOLE
        if (attempt < FIFO_INIT_ATTEMPTS && deps_.runtime.out) {
            deps_.runtime.out->print("# WARN FIFO init retry ");
            deps_.runtime.out->print(attempt);
            deps_.runtime.out->print('/');
            deps_.runtime.out->println(FIFO_INIT_ATTEMPTS);
        }
#endif
        if (attempt < FIFO_INIT_ATTEMPTS) delay(FIFO_INIT_RETRY_DELAY_MS);
    }
    return false;
}

void TrackerApp::beginSensorStartupRecovery(TrackerHealthFaultCode code, const char* message) {
    sensorStartupRecoveryActive_ = true;
    sensorStartupHardFailed_ = false;
    pendingSensorFaultCode_ = code;
    const char* safe = message ? message : "sensor startup failed";
    std::strncpy(pendingSensorFaultMessage_, safe, sizeof(pendingSensorFaultMessage_) - 1u);
    pendingSensorFaultMessage_[sizeof(pendingSensorFaultMessage_) - 1u] = '\0';
    sensorStartupRecoveryStartedMs_ = millis();
    // Initial blocking retries have just failed. Do not immediately publish a
    // fatal SlimeVR state: keep CLI/Wi-Fi alive and prove the sensor stays dead
    // across a longer non-blocking recovery window first.
    nextSensorStartupRecoveryMs_ = sensorStartupRecoveryStartedMs_ + SENSOR_STARTUP_RECOVERY_INTERVAL_MS;
    sensorStartupRecoveryAttempts_ = 0;

#if TRACKER_HAS_SERIAL_CONSOLE
    if (deps_.runtime.out != nullptr) {
        deps_.runtime.out->print("# WARN sensor_startup_recovery=pending code=");
        deps_.runtime.out->print(trackerHealthFaultCodeName(code));
        deps_.runtime.out->print(" hard_fail_after_ms=");
        deps_.runtime.out->println(SENSOR_STARTUP_HARD_FAIL_DELAY_MS);
    }
#endif
    call(deps_.callbacks.setStatusLedSensorError);
}

bool TrackerApp::updateSensorStartupRecovery(uint32_t nowMs) {
    if (sensorRuntimeReady_ || !sensorStartupRecoveryActive_) return false;
    if (static_cast<int32_t>(nowMs - nextSensorStartupRecoveryMs_) < 0) return false;

    ++sensorStartupRecoveryAttempts_;

    bool ok = false;
    if (pendingSensorFaultCode_ == TrackerHealthFaultCode::LsmInitFailed) {
        ok = trackerBootstrapInitLsm(deps_.bootstrap) && trackerBootstrapInitFifo(deps_.bootstrap);
    } else if (pendingSensorFaultCode_ == TrackerHealthFaultCode::FifoInitFailed) {
        ok = trackerBootstrapInitFifo(deps_.bootstrap);
        if (!ok) {
            // A FIFO configure failure after a nominally successful LSM init can
            // still be an SPI/IMU transient. Re-probe the LSM before the next
            // FIFO attempt so startup recovery can heal from a late sensor reset.
            (void)trackerBootstrapInitLsm(deps_.bootstrap);
        }
    }

    if (ok) {
        finishSensorStartupRecoverySuccess();
        return true;
    }

    const uint32_t elapsedMs = nowMs - sensorStartupRecoveryStartedMs_;
    if (!sensorStartupHardFailed_ && elapsedMs >= SENSOR_STARTUP_HARD_FAIL_DELAY_MS) {
        sensorStartupHardFailed_ = true;
        enterFatalDegraded(pendingSensorFaultCode_, pendingSensorFaultMessage_);
    }

    const uint32_t interval = sensorStartupHardFailed_
        ? SENSOR_STARTUP_HARD_FAIL_RECOVERY_INTERVAL_MS
        : SENSOR_STARTUP_RECOVERY_INTERVAL_MS;
    nextSensorStartupRecoveryMs_ = nowMs + interval;
    return true;
}

void TrackerApp::finishSensorStartupRecoverySuccess() {
    setupSensorRuntime();
    sensorRuntimeReady_ = true;
    sensorStartupRecoveryActive_ = false;
    sensorStartupHardFailed_ = false;
    pendingSensorFaultCode_ = TrackerHealthFaultCode::None;
    pendingSensorFaultMessage_[0] = '\0';
    if (deps_.runtime.health != nullptr) {
        deps_.runtime.health->reset();
    }
    publishHealthState();
    call(deps_.callbacks.setupTapRuntime);
#if TRACKER_HAS_SERIAL_CONSOLE
    if (deps_.runtime.out != nullptr) {
        deps_.runtime.out->print("# OK sensor startup recovered attempts=");
        deps_.runtime.out->println(sensorStartupRecoveryAttempts_);
    }
#endif
}

void TrackerApp::setupSensorRuntime() {
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
        deps_.buffers.fifoRuntimeRawQueue,
        deps_.buffers.fifoRuntimeRawQueueFlags,
        deps_.buffers.fifoRuntimeRawQueueCapacity,
        deps_.buffers.fifoRuntimeMagQueue,
        deps_.buffers.fifoRuntimeMagQueueCapacity,
        deps_.callbacks.processRawSample,
        deps_.callbacks.processMagSample,
        deps_.callbacks.recordFifoProcessTime,
        deps_.callbacks.fifoCallbackUser
    );

    trackerBootstrapSetupCalibrationIo(deps_.bootstrap);
    call(deps_.callbacks.setupMagRuntimeController);
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

    startMagFromConfig(*deps_.runtime.out);
}

void TrackerApp::enterFatalDegraded(TrackerHealthFaultCode code, const char* message) {
    deps_.runtime.health->enterDegradedNoImu(code, message);
    if (deps_.runtime.out != nullptr) {
        deps_.runtime.out->print("# fatal_degraded: ");
        deps_.runtime.out->println(message ? message : "");
    }
    call(deps_.callbacks.setStatusLedSensorError);
    publishHealthState();
}

void TrackerApp::publishHealthState() {
    if (deps_.callbacks.publishHealthState != nullptr && deps_.runtime.health != nullptr) {
        deps_.callbacks.publishHealthState(deps_.runtime.health->snapshot());
    }
}

void TrackerApp::call(void (*callback)()) {
    if (callback != nullptr) callback();
}

bool TrackerApp::callBool(bool (*callback)()) {
    return callback != nullptr && callback();
}




#if TRACKER_HAS_MOTION_LIGHT_SLEEP
bool TrackerApp::motionLightSleepBlocked() const {
    if (!sensorRuntimeReady_ || sensorStartupRecoveryActive_) return true;
#if TRACKER_HAS_STATIC_TEST_STATE
    if (deps_.runtime.staticTestRunner != nullptr && deps_.runtime.staticTestRunner->active()) return true;
#endif
#if TRACKER_HAS_RUNTIME_TEST_STATE
    if (deps_.runtime.runtimeTestRunner != nullptr && deps_.runtime.runtimeTestRunner->active()) return true;
#endif
    // These hooks own the non-IMU resources that must be quiesced before the
    // shared INT1 line is repurposed as a level wake source.
    return deps_.bootstrap.lsm == nullptr ||
           deps_.pins.int1 < 0 ||
           deps_.callbacks.prepareMotionLightSleepRuntime == nullptr;
}

bool TrackerApp::requestMotionLightSleep() {
    if (motionLightSleepBlocked()) return false;
    motionLightSleepManualRequested_ = true;
    return true;
}

bool TrackerApp::maybeEnterMotionLightSleep() {
    if (motionLightSleepBlocked()) {
        motionLightSleep_.reset();
        motionLightSleepManualRequested_ = false;
        return false;
    }

    // A serial command only queues the request. This method is reached after
    // cli->poll() has returned, so ending/restarting Serial cannot corrupt the
    // active command parser frame.
    if (motionLightSleepManualRequested_) {
        motionLightSleepManualRequested_ = false;
        motionLightSleep_.noteSleepAttempted();
        return enterMotionLightSleep();
    }

    const bool serverFound = deps_.callbacks.serverFoundForMotionSleep != nullptr &&
                             deps_.callbacks.serverFoundForMotionSleep();
    if (!motionLightSleep_.shouldEnter(serverFound, millis())) {
        return false;
    }

    // Always start a fresh timeout after a wake attempt, including an instant
    // wake caused by motion that happened while the IMU was being armed.
    motionLightSleep_.noteSleepAttempted();
    return enterMotionLightSleep();
}

bool TrackerApp::enterMotionLightSleep() {
    if (deps_.bootstrap.lsm == nullptr || deps_.pins.int1 < 0) {
        return false;
    }

#if TRACKER_HAS_SERIAL_CONSOLE
    if (deps_.runtime.out != nullptr) {
        deps_.runtime.out->println("# motion_light_sleep=enter");
        deps_.runtime.out->flush();
    }
#endif

    // The motion wake source owns the shared INT1 line while asleep. Stop the
    // FIFO ISR first, then stop subsystems that can issue sensor-hub/Wi-Fi I/O.
    call(deps_.callbacks.detachFifoInterrupt);
    call(deps_.callbacks.prepareMotionLightSleepRuntime);
    sensorRuntimeReady_ = false;

    Lsm6dsv::MotionWakeConfig wake;
    wake.enabled = true;
    wake.routeToInt1 = true;
    wake.latchedInterrupt = true;
    wake.maskDuringAccelSettling = true;
#if TRACKER_MOTION_LIGHT_SLEEP_ACCEL_ODR == 30
    wake.accelOdr = Lsm6dsv::Odr::Hz30;
#else
    wake.accelOdr = Lsm6dsv::Odr::Hz60;
#endif
    wake.accelFs = Lsm6dsv::AccelFs::G2;
    wake.accelMode = Lsm6dsv::AccelMode::LowPower1;
    wake.threshold = TRACKER_MOTION_LIGHT_SLEEP_WAKE_THRESHOLD;
    wake.duration = TRACKER_MOTION_LIGHT_SLEEP_WAKE_DURATION;

    const bool imuReady = deps_.bootstrap.lsm->configureMotionWake(wake);
    if (!imuReady) {
#if TRACKER_HAS_SERIAL_CONSOLE
        if (deps_.runtime.out != nullptr) {
            deps_.runtime.out->println("# ERR motion_light_sleep=arm_failed");
        }
#endif
        resumeFromMotionLightSleep();
        return true;
    }

    // Read once after arm to clear an old latched source before level wake is
    // enabled. A new motion in the small race window simply causes an instant,
    // safe wake and full normal-path reinitialization.
    Lsm6dsv::MotionWakeSource source;
    (void)deps_.bootstrap.lsm->readMotionWakeSource(source);

    const gpio_num_t wakePin = static_cast<gpio_num_t>(deps_.pins.int1);
    pinMode(deps_.pins.int1, INPUT);
    const esp_err_t gpioResult = gpio_wakeup_enable(wakePin, GPIO_INTR_HIGH_LEVEL);
    const esp_err_t sleepResult = gpioResult == ESP_OK ? esp_sleep_enable_gpio_wakeup() : gpioResult;
    if (sleepResult != ESP_OK) {
#if TRACKER_HAS_SERIAL_CONSOLE
        if (deps_.runtime.out != nullptr) {
            deps_.runtime.out->print("# ERR motion_light_sleep=gpio_wake_failed err=");
            deps_.runtime.out->println(static_cast<int>(sleepResult));
        }
#endif
        resumeFromMotionLightSleep();
        return true;
    }

#if TRACKER_HAS_SERIAL_CONSOLE
    Serial.flush();
    Serial.end();
#endif

    (void)esp_light_sleep_start();
    // esp_sleep_enable_gpio_wakeup() has no matching global disable API on
    // the Arduino-ESP32 / ESP-IDF version used by this project. Disabling
    // the only armed pin is sufficient: later light-sleep entries have no
    // GPIO wake source until gpio_wakeup_enable() is called again.
    (void)gpio_wakeup_disable(wakePin);

    resumeFromMotionLightSleep();
    return true;
}

void TrackerApp::resumeFromMotionLightSleep() {
#if TRACKER_HAS_SERIAL_CONSOLE
    Serial.begin(deps_.timing.serialBaud);
#endif

    // Clear the latched event before reinitializing. trackerBootstrapInitLsm()
    // also performs the full reset/configure sequence, which removes the
    // motion route and restores the normal high-rate FIFO configuration.
    if (deps_.bootstrap.lsm != nullptr) {
        Lsm6dsv::MotionWakeSource source;
        (void)deps_.bootstrap.lsm->readMotionWakeSource(source);
        Lsm6dsv::MotionWakeConfig disableWake;
        disableWake.enabled = false;
        (void)deps_.bootstrap.lsm->configureMotionWake(disableWake);
    }

    sensorRuntimeReady_ = false;
    sensorStartupRecoveryActive_ = false;
    sensorStartupHardFailed_ = false;
    pendingSensorFaultCode_ = TrackerHealthFaultCode::None;
    pendingSensorFaultMessage_[0] = '\0';

    if (initLsmWithRetries() && initFifoWithRetries()) {
        setupSensorRuntime();
        sensorRuntimeReady_ = true;
    } else {
        beginSensorStartupRecovery(TrackerHealthFaultCode::LsmInitFailed,
                                   "LSM6DSV resume init failed after motion wake");
    }

    call(deps_.callbacks.setupBatteryRuntime);
    call(deps_.callbacks.setupStatusLedRuntime);
    // Preserve unsaved, runtime-applied network settings and connection
    // counters. The first boot uses setupNetworkRuntime(); a wake uses the
    // dedicated resume hook instead of reloading NVS.
    if (deps_.callbacks.resumeNetworkRuntime != nullptr) {
        call(deps_.callbacks.resumeNetworkRuntime);
    } else {
        call(deps_.callbacks.setupNetworkRuntime);
    }
    if (sensorRuntimeReady_) {
        call(deps_.callbacks.setupTapRuntime);
    }
    call(deps_.callbacks.setupCommandInterface);
    publishHealthState();

#if TRACKER_HAS_SERIAL_CONSOLE
    if (deps_.runtime.out != nullptr) {
        deps_.runtime.out->println(sensorRuntimeReady_
            ? "# motion_light_sleep=wake_resume_ok"
            : "# motion_light_sleep=wake_resume_sensor_recovery");
    }
#endif
}
#endif // TRACKER_HAS_MOTION_LIGHT_SLEEP

bool TrackerApp::maybeIdleYield(bool anyWork) {
#if !TRACKER_ENABLE_IDLE_YIELD
    (void)anyWork;
    return false;
#else
    static uint32_t idleLoopStreak = 0;
    if (anyWork) {
        idleLoopStreak = 0;
        return false;
    }

    ++idleLoopStreak;
    if ((idleLoopStreak % TRACKER_IDLE_YIELD_EVERY_N_IDLE_LOOPS) != 0u) {
        return false;
    }

#if TRACKER_IDLE_YIELD_MODE == TRACKER_IDLE_YIELD_MODE_DELAY0
    delay(0);
#elif TRACKER_IDLE_YIELD_MODE == TRACKER_IDLE_YIELD_MODE_DELAY1
    delay(1);
#else
#error "Unsupported TRACKER_IDLE_YIELD_MODE"
#endif
    return true;
#endif
}

void TrackerApp::serviceRuntimeForBlockingCommand() {
    if (!ready()) return;

    if (!sensorRuntimeReady_) {
        (void)updateSensorStartupRecovery(millis());
    }
    if (sensorRuntimeReady_) {
        (void)processFifoRuntime();
    }
    (void)callBool(deps_.callbacks.updateSerialConsoleRuntime);
    (void)callBool(deps_.callbacks.updateBatteryRuntime);
    (void)callBool(deps_.callbacks.updateNetworkRuntime);
    if (sensorRuntimeReady_) {
        (void)callBool(deps_.callbacks.updateTapRuntime);
    }
    (void)callBool(deps_.callbacks.updateStatusLedRuntime);
#if TRACKER_HAS_RUNTIME_TEST
    deps_.runtime.runtimeTestRunner->update(millis(), *deps_.runtime.out);
#endif
}

bool TrackerApp::processFifoRuntime() {
    const TrackerConfig& config = *deps_.runtime.config;
    const uint16_t watermarkWords = config.data.fifo.watermarkWords;
    const uint16_t maxWords = config.data.fifo.maxWordsPerDrain > 0
        ? config.data.fifo.maxWordsPerDrain
        : deps_.timing.fifoMaxWordsPerDrainDefault;
    const uint8_t maxRounds = config.data.fifo.maxDrainRoundsPerEvent > 0
        ? config.data.fifo.maxDrainRoundsPerEvent
        : deps_.timing.fifoMaxDrainRoundsPerEventDefault;

    const uint32_t startUs = micros();
    bool worked = false;
    do {
        const bool sliceWorked = deps_.runtime.fifoRuntime->process(
            watermarkWords, maxWords, maxRounds, *deps_.runtime.out);
        if (!sliceWorked) break;
        worked = true;

        if (!deps_.runtime.fifoRuntime->hasPendingWork()) break;

        // Keep UDP/ping-pong cadence alive while catching up, but do not run
        // CLI, battery, LED or tap work inside the FIFO loop.
        (void)callBool(deps_.callbacks.updateNetworkRuntime);

        const uint32_t elapsedUs = micros() - startUs;
        const uint32_t budgetUs = deps_.runtime.fifoRuntime->urgent()
            ? cfg::FIFO_RUNTIME_URGENT_BUDGET_US
            : cfg::FIFO_RUNTIME_APP_BUDGET_US;
        if (elapsedUs >= budgetUs) break;
    } while (deps_.runtime.fifoRuntime->hasPendingWork());

    return worked;
}

void TrackerApp::startMagFromConfig(Stream& out) {
    TrackerConfig& config = *deps_.runtime.config;
    if (!config.data.magCal.driverEnabled) return;

#if TRACKER_HAS_SERIAL_CONSOLE
    out.println("# mag enabled in config; starting QMC6309 FIFO stream");
#endif
    const bool ok = deps_.callbacks.setMagRuntimeEnabled != nullptr &&
                    deps_.callbacks.setMagRuntimeEnabled(true, false);
    if (!ok) {
#if TRACKER_HAS_SERIAL_CONSOLE
        out.println("# WARN mag startup failed; continuing 6DoF without mag");
#endif
        config.data.magCal.driverEnabled = false;
        config.updateCrc();
    }
}

} // namespace tracker
