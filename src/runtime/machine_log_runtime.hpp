#pragma once

#include <Arduino.h>
#include <cstdint>

#include "defines.h"
#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/calibration.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"
#include "sensor/mag_heading.hpp"
#include "sensor/mag_field_reliability.hpp"
#include "sensor/mag_runtime.hpp"
#include "sensor/mag_yaw_correction.hpp"
#include "runtime/runtime_bias_types.hpp"
#include "runtime/tracker_runtime_types.hpp"
#include "config/tracker_config_runtime.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

void machineLogPrintU64Dec(Stream& out, uint64_t v);
const char* machineLogModeName(TrackerLogMode mode);
bool machineLogDue(TrackerSerialLogState& state, uint32_t nowUs);
bool machineLogMagDue(TrackerSerialLogState& state, uint32_t nowUs);
bool machineLogBiasDue(TrackerSerialLogState& state,
                       uint32_t& lastBiasEmitUs,
                       uint32_t biasPeriodUs,
                       uint32_t nowUs);
void machineLogResetCounters(MachineLogCounters& counters, uint32_t& lastBiasEmitUs);
void machineLogEmitHeader(Stream& out,
                          const TrackerSerialLogState& state,
                          const TrackerConfig& config);
void machineLogEmitStateEvent(Stream& out,
                              TrackerSerialLogState& state,
                              MachineLogCounters& counters,
                              const char* eventState,
                              const char* reason,
                              uint64_t tUs,
                              uint32_t flags,
                              float confidence);
void machineLogPrintSummary(Stream& out,
                            const TrackerSerialLogState& state,
                            const MachineLogCounters& counters,
                            uint32_t runtimeSamples,
                            uint32_t trackingRecoveryEnterCount,
                            const ImuQualityMonitor& quality,
                            const Lsm6dsvFifoReader& fifo,
                            const MagRuntimeProcessor& magProcessor,
                            const MagYawCorrectionController& magYawCorrection,
                            const Ahrs6Dof& ahrs,
                            const MagProcessedSample& lastMagProcessed,
                            const MagYawCorrectionOutput& lastMagYawCorrection,
                            const RuntimeGyroBiasEstimator& runtimeBias);
const char* machineLogBiasSource(const RuntimeGyroBiasEstimator& runtimeBias,
                                 const ImuCalibration& imuCal,
                                 const GyroTempCompensator& gyroTempComp);
void machineLogEmitFrame(Stream& out,
                         TrackerSerialLogState& state,
                         MachineLogCounters& counters,
                         uint32_t& lastBiasEmitUs,
                         uint32_t biasPeriodUs,
                         const Lsm6dsv::RawSample& raw,
                         const Lsm6dsv::Sample& calibrated,
                         const ImuQualityResult& quality,
                         const Ahrs6Dof& ahrs,
                         const char* trackingState,
                         bool trackingRecovering,
                         const GyroTempCompensator& gyroTempComp,
                         const ImuCalibration& imuCal,
                         const RuntimeGyroBiasEstimator& runtimeBias,
                         const Vec3& currentBiasDps,
                         uint32_t gyroBiasFlags);
void machineLogEmitMagFrame(Stream& out,
                            TrackerSerialLogState& state,
                            MachineLogCounters& counters,
                            const MagProcessedSample& mag,
                            const MagHeadingSample& heading,
                            const MagFieldReliabilityOutput& reliability,
                            const MagYawCorrectionOutput& yaw,
                            uint32_t rejectFlagsForUse,
                            bool trustedForUse);

} // namespace tracker
