#pragma once

#include <Arduino.h>
#include <cstdint>

// This context has feature-gated fields. It must include the canonical feature
// contract itself; otherwise translation units that include it indirectly can
// compile a different struct layout (and silently drop optional callbacks).
#include "defines.h"
#include "connection/lsm6dsv_driver.hpp"
#include "core/slimevr_motion_policy.hpp"
#include "serial/tracker_command_origin.hpp"
#include "serial/tracker_serial_parse.hpp"
#include "serial/tracker_serial_print.hpp"

namespace tracker {

class TrackerConfig;
class TrackerConfigStore;
class TrackerNetworkConfig;
class TrackerNetworkConfigStore;
class TrackerWifiManager;
class SlimeVROutputRuntime;
class TapRuntimeController;
class StatusLedRuntime;
class BatteryRuntime;
class BatteryAdcBatchSampler;
class TrackerHealthState;
class RuntimeProfiler;
class RuntimeMotionDiagnostics;
class FifoRuntimeProcessor;
class TrackingStateController;
class PreparedOutputRuntime;
class CalibrationAutonomyController;
class Lsm6dsv;
class Lsm6dsvFifoReader;
class Lsm6dsvSensorHub;
class Qmc6309;
struct ImuCalibration;
class GyroTempCompensator;
class ImuQualityMonitor;
class Ahrs6Dof;
struct RuntimeGyroBiasEstimator;
struct FifoCalibrationIo;
class FifoAccel6PosCalibrationRunner;
class GyroTempCalibrationCapture;
struct StaticRuntimeTest;
struct MagProcessedSample;
struct MagAxisAlignmentRuntimeState;
struct TrackerSerialCommandContext;

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
    uint32_t lastNetworkEmitUs = 0;
    uint32_t sequence = 0;
    bool finishing = false;
    Stream* output = nullptr;
    TrackerCommandOrigin ownerOrigin = TrackerCommandOrigin::UsbSerial;
    uint32_t ownerSessionId = 0;

    bool enabled() const {
        return mode != TrackerLogMode::Off;
    }

    bool accepting() const {
        return enabled() && !finishing;
    }

    uint32_t periodUs() const {
        const uint16_t hz = rateHz == 0 ? 1 : rateHz;
        return 1000000UL / hz;
    }

    bool ownedBy(const TrackerSerialCommandContext& context) const;

    void bind(Stream& sink, TrackerCommandOrigin origin, uint32_t sessionId) {
        output = &sink;
        ownerOrigin = origin;
        ownerSessionId = sessionId;
        finishing = false;
    }

    void release() {
        mode = TrackerLogMode::Off;
        lastEmitUs = 0;
        lastMagEmitUs = 0;
        lastNetworkEmitUs = 0;
        finishing = false;
        output = nullptr;
        ownerOrigin = TrackerCommandOrigin::UsbSerial;
        ownerSessionId = 0;
    }
};

struct TrackerSerialCommandContext {
    Stream* io = nullptr;
    TrackerCommandOrigin origin = TrackerCommandOrigin::UsbSerial;
    uint32_t sessionId = 0;

    TrackerConfig* config = nullptr;
    TrackerConfigStore* configStore = nullptr;

    TrackerNetworkConfig* networkConfig = nullptr;
    TrackerNetworkConfigStore* networkConfigStore = nullptr;
    bool* networkConfigLoadedFromNvs = nullptr;
    TrackerWifiManager* wifiManager = nullptr;
    SlimeVROutputRuntime* slimevrRuntime = nullptr;
    TapRuntimeController* tapRuntime = nullptr;
    StatusLedRuntime* statusLedRuntime = nullptr;
    BatteryRuntime* batteryRuntime = nullptr;
    BatteryAdcBatchSampler* batteryAdcBatchSampler = nullptr;
    TrackerHealthState* health = nullptr;

    Lsm6dsv* lsm = nullptr;
    Lsm6dsvFifoReader* fifo = nullptr;
    Lsm6dsvSensorHub* sensorHub = nullptr;
    Qmc6309* mag = nullptr;
    ImuCalibration* imuCal = nullptr;
    GyroTempCompensator* gyroTempComp = nullptr;
    ImuQualityMonitor* quality = nullptr;
    Ahrs6Dof* ahrs = nullptr;
    RuntimeGyroBiasEstimator* runtimeBias = nullptr;

    FifoCalibrationIo* calibrationIo = nullptr;
    FifoAccel6PosCalibrationRunner* accelCalRunner = nullptr;
    GyroTempCalibrationCapture* gyroTempCapture = nullptr;

    TrackerSerialStreamState* streamState = nullptr;
    TrackerSerialLogState* logState = nullptr;
    RuntimeProfiler* runtimeProfiler = nullptr;
    RuntimeMotionDiagnostics* motionDiagnostics = nullptr;
    FifoRuntimeProcessor* fifoRuntime = nullptr;
    const MagAxisAlignmentRuntimeState* magAxisAlignmentState = nullptr;
    TrackingStateController* trackingState = nullptr;
    PreparedOutputRuntime* preparedOutput = nullptr;
#if TRACKER_HAS_CALIBRATION_AUTONOMY
    CalibrationAutonomyController* calibrationAutonomy = nullptr;
#endif

    // Optional hooks supplied by main.cpp.
    // Clears software FIFO queues/counters only. Recovery is requested
    // explicitly through requestTrackingRecovery so diagnostics can identify
    // manual, blocking-operation and reconfiguration causes correctly.
    void (*resetFifoRuntime)(void* user) = nullptr;
    void* resetFifoRuntimeUser = nullptr;

    void (*requestTrackingRecovery)(uint32_t reasonFlags,
                                    const char* reason,
                                    uint64_t timestampUs,
                                    void* user) = nullptr;
    void* requestTrackingRecoveryUser = nullptr;

    void (*resetAhrsRuntime)(void* user) = nullptr;
    void* resetAhrsRuntimeUser = nullptr;

    void (*printRuntimeStatus)(Stream& out, void* user) = nullptr;
    void* printRuntimeStatusUser = nullptr;

    void (*printRuntimeHealth)(Stream& out, void* user) = nullptr;
    void* printRuntimeHealthUser = nullptr;

    bool (*setSpiFrequency)(uint32_t hz, void* user) = nullptr;
    void* setSpiFrequencyUser = nullptr;

    bool (*setRemoteConsoleEnabled)(bool enabled, void* user) = nullptr;
    void* setRemoteConsoleEnabledUser = nullptr;

    void (*printRemoteConsoleStatus)(Stream& out, void* user) = nullptr;
    void* printRemoteConsoleStatusUser = nullptr;

    void (*printConsoleOutputStatus)(Stream& out, void* user) = nullptr;
    void* printConsoleOutputStatusUser = nullptr;
    void (*resetConsoleOutputState)(void* user) = nullptr;
    void* resetConsoleOutputStateUser = nullptr;

    void (*emitLogHeader)(Stream& out, void* user) = nullptr;
    void* emitLogHeaderUser = nullptr;

    void (*printLogSummary)(Stream& out, void* user) = nullptr;
    void* printLogSummaryUser = nullptr;

    void (*resetLogCounters)(void* user) = nullptr;
    void* resetLogCountersUser = nullptr;

    void (*resetLogPipeline)(bool countPendingAsDropped, void* user) = nullptr;
    void* resetLogPipelineUser = nullptr;

    void (*printRuntimeGyroBiasStatus)(Stream& out, void* user) = nullptr;
    void* printRuntimeGyroBiasStatusUser = nullptr;

    bool (*setRuntimeGyroBiasEnabled)(bool enabled, void* user) = nullptr;
    void* setRuntimeGyroBiasEnabledUser = nullptr;

    bool (*setSlimeVRMotionPacketPolicy)(SlimeVRMotionPacketPolicy policy, void* user) = nullptr;
    void* setSlimeVRMotionPacketPolicyUser = nullptr;

    void (*resetRuntimeGyroBiasEstimator)(void* user) = nullptr;
    void* resetRuntimeGyroBiasEstimatorUser = nullptr;

    bool (*startStaticTest)(uint32_t durationMs, Stream& out, void* user) = nullptr;
    void* startStaticTestUser = nullptr;

    bool (*stopStaticTest)(Stream& out, bool force, void* user) = nullptr;
    void* stopStaticTestUser = nullptr;

    void (*printStaticTestStatus)(Stream& out, void* user) = nullptr;
    void* printStaticTestStatusUser = nullptr;
    bool (*printStaticTestSummary)(Stream& out, void* user) = nullptr;
    void* printStaticTestSummaryUser = nullptr;
    bool (*printStaticTestReport)(Stream& out, void* user) = nullptr;
    void* printStaticTestReportUser = nullptr;

    bool (*startRuntimeTest)(uint32_t durationMs, Stream& out, void* user) = nullptr;
    void* startRuntimeTestUser = nullptr;

    bool (*stopRuntimeTest)(Stream& out, bool force, void* user) = nullptr;
    void* stopRuntimeTestUser = nullptr;

    void (*printRuntimeTestStatus)(Stream& out, void* user) = nullptr;
    void* printRuntimeTestStatusUser = nullptr;
    bool (*printRuntimeTestSummary)(Stream& out, void* user) = nullptr;
    void* printRuntimeTestSummaryUser = nullptr;
    bool (*printRuntimeTestReport)(Stream& out, void* user) = nullptr;
    void* printRuntimeTestReportUser = nullptr;

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

    // Apply a temperature fit to RAM without saving it. Used by transactional
    // guided setup so NVS is only changed by the final commit.
    bool (*fitGyroTempFromCaptureRam)(const StaticRuntimeTest* capture, Stream& out, void* user) = nullptr;
    void* fitGyroTempFromCaptureRamUser = nullptr;

    const MagProcessedSample* lastMagProcessed = nullptr;
    const Lsm6dsv::Sample* lastScaledSample = nullptr;
    const Lsm6dsv::Sample* lastCalibratedSample = nullptr;
    const uint32_t* lastImuSampleSequence = nullptr;

    // Services FIFO, magnetometer and network/slime runtime without polling
    // the serial parser recursively. Blocking setup flows and long diagnostic
    // reports use it to avoid starving the IMU while command output is written.
    // It remains optional for host-only command dispatch and unit tests.
    bool (*serviceNonCliRuntime)(void* user) = nullptr;
    void* serviceNonCliRuntimeUser = nullptr;

    // TCP remote-console output is intentionally buffered and normally drained
    // by the outer application loop. Blocking setup/calibration commands own
    // that loop, so they need a bounded explicit flush path to keep prompts
    // visible before waiting for input.
    bool commandOutputNeedsExplicitFlush = false;
    uint32_t lastCommandOutputFlushMs = 0u;

    // Called before a transport-owned Stream is detached. Session-bound log
    // and test producers must release the sink here; retaining it would turn a
    // normal TCP disconnect into a use-after-detach or an unintended USB
    // fallback.
    bool (*closeCommandSession)(TrackerCommandOrigin origin,
                                uint32_t sessionId,
                                Stream& out,
                                void* user) = nullptr;
    void* closeCommandSessionUser = nullptr;

#if TRACKER_HAS_MOTION_LIGHT_SLEEP
    // Queues a deferred platform sleep transition. It must not enter sleep
    // synchronously from the CLI parser callback.
    bool (*requestMotionLightSleep)(void* user) = nullptr;
    void* requestMotionLightSleepUser = nullptr;
#endif
};

inline bool TrackerSerialLogState::ownedBy(const TrackerSerialCommandContext& context) const {
    return output == context.io && ownerOrigin == context.origin &&
           ownerSessionId == context.sessionId;
}


} // namespace tracker
