#pragma once

#include <Arduino.h>

#include "serial/tracker_serial_commands.hpp"

namespace tracker {

struct TrackerCommandRuntimeObjects {
    Stream* io = nullptr;

    TrackerConfig* config = nullptr;
    TrackerConfigStore* configStore = nullptr;

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

inline void wireTrackerCommandContext(TrackerSerialCommandContext& ctx,
                                      const TrackerCommandRuntimeObjects& objects,
                                      const TrackerCommandRuntimeHooks& hooks) {
    ctx = TrackerSerialCommandContext{};

    ctx.io = objects.io;
    ctx.config = objects.config;
    ctx.configStore = objects.configStore;
    ctx.lsm = objects.lsm;
    ctx.fifo = objects.fifo;
    ctx.sensorHub = objects.sensorHub;
    ctx.mag = objects.mag;
    ctx.imuCal = objects.imuCal;
    ctx.gyroTempComp = objects.gyroTempComp;
    ctx.quality = objects.quality;
    ctx.ahrs = objects.ahrs;
    ctx.calibrationIo = objects.calibrationIo;
    ctx.accelCalRunner = objects.accelCalRunner;
    ctx.streamState = objects.streamState;
    ctx.logState = objects.logState;

    ctx.resetFifoRuntime = hooks.resetFifoRuntime;
    ctx.resetFifoRuntimeUser = hooks.resetFifoRuntimeUser;
    ctx.resetAhrsRuntime = hooks.resetAhrsRuntime;
    ctx.resetAhrsRuntimeUser = hooks.resetAhrsRuntimeUser;
    ctx.printRuntimeStatus = hooks.printRuntimeStatus;
    ctx.printRuntimeStatusUser = hooks.printRuntimeStatusUser;
    ctx.printRuntimeHealth = hooks.printRuntimeHealth;
    ctx.printRuntimeHealthUser = hooks.printRuntimeHealthUser;
    ctx.setSpiFrequency = hooks.setSpiFrequency;
    ctx.setSpiFrequencyUser = hooks.setSpiFrequencyUser;
    ctx.emitLogHeader = hooks.emitLogHeader;
    ctx.emitLogHeaderUser = hooks.emitLogHeaderUser;
    ctx.printLogSummary = hooks.printLogSummary;
    ctx.printLogSummaryUser = hooks.printLogSummaryUser;
    ctx.resetLogCounters = hooks.resetLogCounters;
    ctx.resetLogCountersUser = hooks.resetLogCountersUser;
    ctx.printRuntimeGyroBiasStatus = hooks.printRuntimeGyroBiasStatus;
    ctx.printRuntimeGyroBiasStatusUser = hooks.printRuntimeGyroBiasStatusUser;
    ctx.setRuntimeGyroBiasEnabled = hooks.setRuntimeGyroBiasEnabled;
    ctx.setRuntimeGyroBiasEnabledUser = hooks.setRuntimeGyroBiasEnabledUser;
    ctx.resetRuntimeGyroBiasEstimator = hooks.resetRuntimeGyroBiasEstimator;
    ctx.resetRuntimeGyroBiasEstimatorUser = hooks.resetRuntimeGyroBiasEstimatorUser;
    ctx.startStaticTest = hooks.startStaticTest;
    ctx.startStaticTestUser = hooks.startStaticTestUser;
    ctx.stopStaticTest = hooks.stopStaticTest;
    ctx.stopStaticTestUser = hooks.stopStaticTestUser;
    ctx.printStaticTestStatus = hooks.printStaticTestStatus;
    ctx.printStaticTestStatusUser = hooks.printStaticTestStatusUser;
    ctx.setMagRuntimeEnabled = hooks.setMagRuntimeEnabled;
    ctx.setMagRuntimeEnabledUser = hooks.setMagRuntimeEnabledUser;
    ctx.printMagRuntimeStatus = hooks.printMagRuntimeStatus;
    ctx.printMagRuntimeStatusUser = hooks.printMagRuntimeStatusUser;
    ctx.printMagProcessedStatus = hooks.printMagProcessedStatus;
    ctx.printMagProcessedStatusUser = hooks.printMagProcessedStatusUser;
    ctx.printMagHeadingStatus = hooks.printMagHeadingStatus;
    ctx.printMagHeadingStatusUser = hooks.printMagHeadingStatusUser;
    ctx.setMagHeadingReference = hooks.setMagHeadingReference;
    ctx.setMagHeadingReferenceUser = hooks.setMagHeadingReferenceUser;
    ctx.clearMagHeadingReference = hooks.clearMagHeadingReference;
    ctx.clearMagHeadingReferenceUser = hooks.clearMagHeadingReferenceUser;
    ctx.setMagHeadingAutoReferenceEnabled = hooks.setMagHeadingAutoReferenceEnabled;
    ctx.setMagHeadingAutoReferenceEnabledUser = hooks.setMagHeadingAutoReferenceEnabledUser;
    ctx.printMagYawCorrectionStatus = hooks.printMagYawCorrectionStatus;
    ctx.printMagYawCorrectionStatusUser = hooks.printMagYawCorrectionStatusUser;
    ctx.resetMagYawCorrection = hooks.resetMagYawCorrection;
    ctx.resetMagYawCorrectionUser = hooks.resetMagYawCorrectionUser;
    ctx.setMagYawCorrectionApplyEnabled = hooks.setMagYawCorrectionApplyEnabled;
    ctx.setMagYawCorrectionApplyEnabledUser = hooks.setMagYawCorrectionApplyEnabledUser;
    ctx.startMagCalibration = hooks.startMagCalibration;
    ctx.startMagCalibrationUser = hooks.startMagCalibrationUser;
    ctx.stopMagCalibration = hooks.stopMagCalibration;
    ctx.stopMagCalibrationUser = hooks.stopMagCalibrationUser;
    ctx.resetMagCalibration = hooks.resetMagCalibration;
    ctx.resetMagCalibrationUser = hooks.resetMagCalibrationUser;
    ctx.applyMagCalibration = hooks.applyMagCalibration;
    ctx.applyMagCalibrationUser = hooks.applyMagCalibrationUser;
    ctx.printMagCalibrationStatus = hooks.printMagCalibrationStatus;
    ctx.printMagCalibrationStatusUser = hooks.printMagCalibrationStatusUser;
    ctx.fitGyroTempFromLastStatic = hooks.fitGyroTempFromLastStatic;
    ctx.fitGyroTempFromLastStaticUser = hooks.fitGyroTempFromLastStaticUser;
}

} // namespace tracker
