#pragma once

#include <Arduino.h>

#include "serial/tracker_serial_context.hpp"

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

    TrackerSerialStreamState* streamState = nullptr;
    TrackerSerialLogState* logState = nullptr;
};

struct TrackerCommandRuntimeHooks {
    decltype(TrackerSerialCommandContext::resetFifoRuntime) resetFifoRuntime = nullptr;
    void* resetFifoRuntimeUser = nullptr;

    decltype(TrackerSerialCommandContext::resetAhrsRuntime) resetAhrsRuntime = nullptr;
    void* resetAhrsRuntimeUser = nullptr;

    decltype(TrackerSerialCommandContext::printRuntimeStatus) printRuntimeStatus = nullptr;
    void* printRuntimeStatusUser = nullptr;

    decltype(TrackerSerialCommandContext::printRuntimeHealth) printRuntimeHealth = nullptr;
    void* printRuntimeHealthUser = nullptr;

    decltype(TrackerSerialCommandContext::setSpiFrequency) setSpiFrequency = nullptr;
    void* setSpiFrequencyUser = nullptr;

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
};

void wireTrackerCommandContext(TrackerSerialCommandContext& ctx,
                               const TrackerCommandRuntimeObjects& objects,
                               const TrackerCommandRuntimeHooks& hooks);

} // namespace tracker
