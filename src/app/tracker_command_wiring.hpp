#pragma once

#include <Arduino.h>

#include "serial/tracker_serial_context.hpp"
#include "runtime/tracker_health_state.hpp"

namespace tracker {

struct TrackerCommandRuntimeObjects {
    Stream* io = nullptr;

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
    RuntimeProfiler* runtimeProfiler = nullptr;
    RuntimeMotionDiagnostics* motionDiagnostics = nullptr;
    FifoRuntimeProcessor* fifoRuntime = nullptr;
    TrackingStateController* trackingState = nullptr;
    PreparedOutputRuntime* preparedOutput = nullptr;
#if TRACKER_HAS_CALIBRATION_AUTONOMY
    CalibrationAutonomyController* calibrationAutonomy = nullptr;
#endif

    FifoCalibrationIo* calibrationIo = nullptr;
    FifoAccel6PosCalibrationRunner* accelCalRunner = nullptr;
    GyroTempCalibrationCapture* gyroTempCapture = nullptr;
    const MagProcessedSample* lastMagProcessed = nullptr;
    const Lsm6dsv::Sample* lastScaledSample = nullptr;
    const Lsm6dsv::Sample* lastCalibratedSample = nullptr;
    const uint32_t* lastImuSampleSequence = nullptr;

    TrackerSerialStreamState* streamState = nullptr;
    TrackerSerialLogState* logState = nullptr;
};

struct TrackerCommandRuntimeHooks {
    decltype(TrackerSerialCommandContext::resetFifoRuntime) resetFifoRuntime = nullptr;
    void* resetFifoRuntimeUser = nullptr;

    decltype(TrackerSerialCommandContext::requestTrackingRecovery) requestTrackingRecovery = nullptr;
    void* requestTrackingRecoveryUser = nullptr;

    decltype(TrackerSerialCommandContext::resetAhrsRuntime) resetAhrsRuntime = nullptr;
    void* resetAhrsRuntimeUser = nullptr;

    decltype(TrackerSerialCommandContext::printRuntimeStatus) printRuntimeStatus = nullptr;
    void* printRuntimeStatusUser = nullptr;

    decltype(TrackerSerialCommandContext::printRuntimeHealth) printRuntimeHealth = nullptr;
    void* printRuntimeHealthUser = nullptr;

    decltype(TrackerSerialCommandContext::setSpiFrequency) setSpiFrequency = nullptr;
    void* setSpiFrequencyUser = nullptr;

    decltype(TrackerSerialCommandContext::setRemoteConsoleEnabled) setRemoteConsoleEnabled = nullptr;
    void* setRemoteConsoleEnabledUser = nullptr;

    decltype(TrackerSerialCommandContext::printRemoteConsoleStatus) printRemoteConsoleStatus = nullptr;
    void* printRemoteConsoleStatusUser = nullptr;

    decltype(TrackerSerialCommandContext::printConsoleOutputStatus) printConsoleOutputStatus = nullptr;
    void* printConsoleOutputStatusUser = nullptr;
    decltype(TrackerSerialCommandContext::resetConsoleOutputState) resetConsoleOutputState = nullptr;
    void* resetConsoleOutputStateUser = nullptr;

    decltype(TrackerSerialCommandContext::emitLogHeader) emitLogHeader = nullptr;
    void* emitLogHeaderUser = nullptr;

    decltype(TrackerSerialCommandContext::printLogSummary) printLogSummary = nullptr;
    void* printLogSummaryUser = nullptr;

    decltype(TrackerSerialCommandContext::resetLogCounters) resetLogCounters = nullptr;
    void* resetLogCountersUser = nullptr;

    decltype(TrackerSerialCommandContext::printRuntimeGyroBiasStatus) printRuntimeGyroBiasStatus = nullptr;
    void* printRuntimeGyroBiasStatusUser = nullptr;

    decltype(TrackerSerialCommandContext::setRuntimeGyroBiasEnabled) setRuntimeGyroBiasEnabled = nullptr;
    void* setRuntimeGyroBiasEnabledUser = nullptr;

    decltype(TrackerSerialCommandContext::resetRuntimeGyroBiasEstimator) resetRuntimeGyroBiasEstimator = nullptr;
    void* resetRuntimeGyroBiasEstimatorUser = nullptr;

    decltype(TrackerSerialCommandContext::startStaticTest) startStaticTest = nullptr;
    void* startStaticTestUser = nullptr;

    decltype(TrackerSerialCommandContext::stopStaticTest) stopStaticTest = nullptr;
    void* stopStaticTestUser = nullptr;

    decltype(TrackerSerialCommandContext::printStaticTestStatus) printStaticTestStatus = nullptr;
    void* printStaticTestStatusUser = nullptr;

    decltype(TrackerSerialCommandContext::startRuntimeTest) startRuntimeTest = nullptr;
    void* startRuntimeTestUser = nullptr;

    decltype(TrackerSerialCommandContext::stopRuntimeTest) stopRuntimeTest = nullptr;
    void* stopRuntimeTestUser = nullptr;

    decltype(TrackerSerialCommandContext::printRuntimeTestStatus) printRuntimeTestStatus = nullptr;
    void* printRuntimeTestStatusUser = nullptr;

    decltype(TrackerSerialCommandContext::setMagRuntimeEnabled) setMagRuntimeEnabled = nullptr;
    void* setMagRuntimeEnabledUser = nullptr;

    decltype(TrackerSerialCommandContext::printMagRuntimeStatus) printMagRuntimeStatus = nullptr;
    void* printMagRuntimeStatusUser = nullptr;

    decltype(TrackerSerialCommandContext::printMagProcessedStatus) printMagProcessedStatus = nullptr;
    void* printMagProcessedStatusUser = nullptr;

    decltype(TrackerSerialCommandContext::printMagHeadingStatus) printMagHeadingStatus = nullptr;
    void* printMagHeadingStatusUser = nullptr;

    decltype(TrackerSerialCommandContext::setMagHeadingReference) setMagHeadingReference = nullptr;
    void* setMagHeadingReferenceUser = nullptr;

    decltype(TrackerSerialCommandContext::clearMagHeadingReference) clearMagHeadingReference = nullptr;
    void* clearMagHeadingReferenceUser = nullptr;

    decltype(TrackerSerialCommandContext::setMagHeadingAutoReferenceEnabled) setMagHeadingAutoReferenceEnabled = nullptr;
    void* setMagHeadingAutoReferenceEnabledUser = nullptr;

    decltype(TrackerSerialCommandContext::printMagYawCorrectionStatus) printMagYawCorrectionStatus = nullptr;
    void* printMagYawCorrectionStatusUser = nullptr;

    decltype(TrackerSerialCommandContext::resetMagYawCorrection) resetMagYawCorrection = nullptr;
    void* resetMagYawCorrectionUser = nullptr;

    decltype(TrackerSerialCommandContext::setMagYawCorrectionApplyEnabled) setMagYawCorrectionApplyEnabled = nullptr;
    void* setMagYawCorrectionApplyEnabledUser = nullptr;

    decltype(TrackerSerialCommandContext::startMagCalibration) startMagCalibration = nullptr;
    void* startMagCalibrationUser = nullptr;

    decltype(TrackerSerialCommandContext::stopMagCalibration) stopMagCalibration = nullptr;
    void* stopMagCalibrationUser = nullptr;

    decltype(TrackerSerialCommandContext::resetMagCalibration) resetMagCalibration = nullptr;
    void* resetMagCalibrationUser = nullptr;

    decltype(TrackerSerialCommandContext::applyMagCalibration) applyMagCalibration = nullptr;
    void* applyMagCalibrationUser = nullptr;

    decltype(TrackerSerialCommandContext::printMagCalibrationStatus) printMagCalibrationStatus = nullptr;
    void* printMagCalibrationStatusUser = nullptr;

    decltype(TrackerSerialCommandContext::fitGyroTempFromLastStatic) fitGyroTempFromLastStatic = nullptr;
    void* fitGyroTempFromLastStaticUser = nullptr;

    decltype(TrackerSerialCommandContext::fitGyroTempFromCapture) fitGyroTempFromCapture = nullptr;
    void* fitGyroTempFromCaptureUser = nullptr;

    decltype(TrackerSerialCommandContext::fitGyroTempFromCaptureRam) fitGyroTempFromCaptureRam = nullptr;
    void* fitGyroTempFromCaptureRamUser = nullptr;

    decltype(TrackerSerialCommandContext::serviceNonCliRuntime) serviceNonCliRuntime = nullptr;
    void* serviceNonCliRuntimeUser = nullptr;

#if TRACKER_HAS_MOTION_LIGHT_SLEEP
    decltype(TrackerSerialCommandContext::requestMotionLightSleep) requestMotionLightSleep = nullptr;
    void* requestMotionLightSleepUser = nullptr;
#endif
};

void wireTrackerCommandContext(TrackerSerialCommandContext& ctx,
                               const TrackerCommandRuntimeObjects& objects,
                               const TrackerCommandRuntimeHooks& hooks);

} // namespace tracker
