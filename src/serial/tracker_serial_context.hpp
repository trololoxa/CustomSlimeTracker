#pragma once

#include <Arduino.h>
#include <cstdint>
#include "serial/tracker_serial_parse.hpp"
#include "serial/tracker_serial_print.hpp"

namespace tracker {

class TrackerConfig;
class TrackerConfigStore;
class TrackerNetworkConfig;
class TrackerNetworkConfigStore;
class TrackerWifiManager;
class SlimeVROutputRuntime;
class Lsm6dsv;
class Lsm6dsvFifoReader;
class Lsm6dsvSensorHub;
class Qmc6309;
struct ImuCalibration;
class GyroTempCompensator;
class ImuQualityMonitor;
class Ahrs6Dof;
struct FifoCalibrationIo;
class FifoAccel6PosCalibrationRunner;
class GyroTempCalibrationCapture;
struct StaticRuntimeTest;
struct MagProcessedSample;

// ============================================================
// Lightweight serial command protocol
// ============================================================
// Design goals:
//   - no heap allocation
//   - no String
//   - non-blocking poll(), optionally limited by bytes per loop
//   - nearly zero CPU load when Serial has no bytes
//   - text commands for development/config/calibration
//   - local stream/log output and SlimeVR UDP output stay separate domains
//
// Usage in main.cpp:
//   static TrackerSerialCommandInterface<> g_cli;
//   static TrackerSerialCommandContext g_cmdCtx;
//
//   void setup() {
//       g_cmdCtx.io = &Serial;
//       g_cmdCtx.config = &g_config;
//       ... fill pointers ...
//       g_cli.begin(g_cmdCtx);
//   }
//
//   void loop() {
//       g_cli.poll(32);
//       // normal tracker loop
//   }
// ============================================================

enum class TrackerStreamMode : uint8_t {
    Off,
    Raw,
    Scaled,
    Quat,
    Debug,
    Heartbeat
};

struct TrackerSerialStreamState {
    TrackerStreamMode mode = TrackerStreamMode::Off;
    uint16_t rateHz = 100;
    uint32_t lastEmitUs = 0;

    uint32_t periodUs() const {
        const uint16_t hz = rateHz == 0 ? 1 : rateHz;
        return 1000000UL / hz;
    }
};

enum class TrackerLogMode : uint8_t {
    Off,
    Basic,
    Full
};

struct TrackerSerialLogState {
    TrackerLogMode mode = TrackerLogMode::Off;
    uint16_t rateHz = 20;
    uint32_t lastEmitUs = 0;
    uint32_t lastMagEmitUs = 0;
    uint32_t sequence = 0;

    bool enabled() const {
        return mode != TrackerLogMode::Off;
    }

    uint32_t periodUs() const {
        const uint16_t hz = rateHz == 0 ? 1 : rateHz;
        return 1000000UL / hz;
    }
};

struct TrackerSerialCommandContext {
    Stream* io = nullptr;

    TrackerConfig* config = nullptr;
    TrackerConfigStore* configStore = nullptr;

    TrackerNetworkConfig* networkConfig = nullptr;
    TrackerNetworkConfigStore* networkConfigStore = nullptr;
    bool* networkConfigLoadedFromNvs = nullptr;
    TrackerWifiManager* wifiManager = nullptr;
    SlimeVROutputRuntime* slimevrRuntime = nullptr;

    Lsm6dsv* lsm = nullptr;
    Lsm6dsvFifoReader* fifo = nullptr;
    Lsm6dsvSensorHub* sensorHub = nullptr;
    Qmc6309* mag = nullptr;
    ImuCalibration* imuCal = nullptr;
    GyroTempCompensator* gyroTempComp = nullptr;
    ImuQualityMonitor* quality = nullptr;
    Ahrs6Dof* ahrs = nullptr;

    FifoCalibrationIo* calibrationIo = nullptr;
    FifoAccel6PosCalibrationRunner* accelCalRunner = nullptr;
    GyroTempCalibrationCapture* gyroTempCapture = nullptr;

    TrackerSerialStreamState* streamState = nullptr;
    TrackerSerialLogState* logState = nullptr;

    // Optional hooks supplied by main.cpp.
    void (*resetFifoRuntime)(void* user) = nullptr;
    void* resetFifoRuntimeUser = nullptr;

    void (*resetAhrsRuntime)(void* user) = nullptr;
    void* resetAhrsRuntimeUser = nullptr;

    void (*printRuntimeStatus)(Stream& out, void* user) = nullptr;
    void* printRuntimeStatusUser = nullptr;

    void (*printRuntimeHealth)(Stream& out, void* user) = nullptr;
    void* printRuntimeHealthUser = nullptr;

    bool (*setSpiFrequency)(uint32_t hz, void* user) = nullptr;
    void* setSpiFrequencyUser = nullptr;

    void (*emitLogHeader)(Stream& out, void* user) = nullptr;
    void* emitLogHeaderUser = nullptr;

    void (*printLogSummary)(Stream& out, void* user) = nullptr;
    void* printLogSummaryUser = nullptr;

    void (*resetLogCounters)(void* user) = nullptr;
    void* resetLogCountersUser = nullptr;

    void (*printRuntimeGyroBiasStatus)(Stream& out, void* user) = nullptr;
    void* printRuntimeGyroBiasStatusUser = nullptr;

    bool (*setRuntimeGyroBiasEnabled)(bool enabled, void* user) = nullptr;
    void* setRuntimeGyroBiasEnabledUser = nullptr;

    void (*resetRuntimeGyroBiasEstimator)(void* user) = nullptr;
    void* resetRuntimeGyroBiasEstimatorUser = nullptr;

    bool (*startStaticTest)(uint32_t durationMs, void* user) = nullptr;
    void* startStaticTestUser = nullptr;

    bool (*stopStaticTest)(void* user) = nullptr;
    void* stopStaticTestUser = nullptr;

    void (*printStaticTestStatus)(Stream& out, void* user) = nullptr;
    void* printStaticTestStatusUser = nullptr;

    bool (*startRuntimeTest)(uint32_t durationMs, void* user) = nullptr;
    void* startRuntimeTestUser = nullptr;

    bool (*stopRuntimeTest)(void* user) = nullptr;
    void* stopRuntimeTestUser = nullptr;

    void (*printRuntimeTestStatus)(Stream& out, void* user) = nullptr;
    void* printRuntimeTestStatusUser = nullptr;

    bool (*setMagRuntimeEnabled)(bool enabled, bool persist, void* user) = nullptr;
    void* setMagRuntimeEnabledUser = nullptr;

    void (*printMagRuntimeStatus)(Stream& out, void* user) = nullptr;
    void* printMagRuntimeStatusUser = nullptr;

    void (*printMagProcessedStatus)(Stream& out, void* user) = nullptr;
    void* printMagProcessedStatusUser = nullptr;

    void (*printMagHeadingStatus)(Stream& out, void* user) = nullptr;
    void* printMagHeadingStatusUser = nullptr;

    bool (*setMagHeadingReference)(void* user) = nullptr;
    void* setMagHeadingReferenceUser = nullptr;

    void (*clearMagHeadingReference)(void* user) = nullptr;
    void* clearMagHeadingReferenceUser = nullptr;

    bool (*setMagHeadingAutoReferenceEnabled)(bool enabled, void* user) = nullptr;
    void* setMagHeadingAutoReferenceEnabledUser = nullptr;

    void (*printMagYawCorrectionStatus)(Stream& out, void* user) = nullptr;
    void* printMagYawCorrectionStatusUser = nullptr;

    void (*resetMagYawCorrection)(void* user) = nullptr;
    void* resetMagYawCorrectionUser = nullptr;

    bool (*setMagYawCorrectionApplyEnabled)(bool enabled, bool persist, void* user) = nullptr;
    void* setMagYawCorrectionApplyEnabledUser = nullptr;

    bool (*startMagCalibration)(void* user) = nullptr;
    void* startMagCalibrationUser = nullptr;

    void (*stopMagCalibration)(void* user) = nullptr;
    void* stopMagCalibrationUser = nullptr;

    void (*resetMagCalibration)(void* user) = nullptr;
    void* resetMagCalibrationUser = nullptr;

    bool (*applyMagCalibration)(bool persist, void* user) = nullptr;
    void* applyMagCalibrationUser = nullptr;

    void (*printMagCalibrationStatus)(Stream& out, void* user) = nullptr;
    void* printMagCalibrationStatusUser = nullptr;

    bool (*fitGyroTempFromLastStatic)(bool persist, Stream& out, void* user) = nullptr;
    void* fitGyroTempFromLastStaticUser = nullptr;

    bool (*fitGyroTempFromCapture)(const StaticRuntimeTest* capture, bool persist, Stream& out, void* user) = nullptr;
    void* fitGyroTempFromCaptureUser = nullptr;

    const MagProcessedSample* lastMagProcessed = nullptr;

    // Used by blocking guided setup/calibration flows. The hook must service
    // FIFO, magnetometer runtime and network/slime runtime without polling the
    // serial command parser recursively. It is intentionally optional so unit
    // tests and host-only command dispatch can still build.
    bool (*serviceCalibrationRuntime)(void* user) = nullptr;
    void* serviceCalibrationRuntimeUser = nullptr;
};


} // namespace tracker
