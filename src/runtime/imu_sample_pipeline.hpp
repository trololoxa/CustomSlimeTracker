#pragma once

#include <Arduino.h>
#include <cstdint>

#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "config/tracker_config_runtime.hpp"
#include "runtime/fifo_runtime_processor.hpp"
#include "runtime/output_runtime.hpp"
#include "runtime/runtime_bias_types.hpp"
#include "runtime/runtime_gyro_bias_controller.hpp"
#include "runtime/static_test_runner.hpp"
#include "runtime/tracker_runtime_types.hpp"
#include "runtime/tracking_state_controller.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/calibration.hpp"
#include "sensor/fifo_calibrations.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

using ImuPipelineRecoveryCallback = void (*)(uint32_t reasonFlags,
                                             const char* reason,
                                             uint64_t timestampUs,
                                             void* user);
using ImuPipelineQualityCallback = void (*)(const ImuQualityResult& quality, void* user);
using ImuPipelineMachineLogCallback = void (*)(const Lsm6dsv::RawSample& raw,
                                               const Lsm6dsv::Sample& calibrated,
                                               const ImuQualityResult& quality,
                                               void* user);
using ImuPipelineFifoRecoveryCallback = void (*)(const ImuQualityResult& quality,
                                                 const Lsm6dsv::RawSample& raw,
                                                 void* user);

struct ImuSamplePipelineCallbacks {
    ImuPipelineRecoveryCallback enterTrackingRecovery = nullptr;
    ImuPipelineQualityCallback updateTrackingRecoveryState = nullptr;
    ImuPipelineMachineLogCallback emitMachineLogFrame = nullptr;
    ImuPipelineFifoRecoveryCallback maybeRecoverFifo = nullptr;
    void* user = nullptr;
};

struct ImuSamplePipelineDeps {
    Lsm6dsv& lsm;
    Lsm6dsvFifoReader& fifo;
    TrackerConfig& config;
    ImuCalibration& imuCal;
    GyroTempCompensator& gyroTempComp;
    ImuQualityMonitor& qualityMonitor;
    Ahrs6Dof& ahrs;
    RuntimeGyroBiasEstimator& runtimeBias;
    TrackingStateController& trackingState;
    PreparedOutputRuntime& preparedOutput;
    TrackerSerialStreamState& streamState;
    TrackerSerialLogState& logState;
    MachineLogCounters& logCounters;
    StaticTestRunner& staticTestRunner;
    TrackerPerfCounters& perf;
    FifoCalibrationIo* calibrationIo;
    Stream& out;
    uint32_t& runtimeSamples;
    uint64_t& lastSampleTimestampUs;
    float& latestTempC;
    float& lastOutputConfidence;
    ImuSamplePipelineCallbacks callbacks;
};

void imuPipelineUpdateLatestTemperature(ImuSamplePipelineDeps& deps);
Vec3 imuPipelineCurrentGyroBiasRadS(const ImuSamplePipelineDeps& deps, float tempC);
Lsm6dsv::Sample imuPipelineMakeCalibratedSample(const ImuSamplePipelineDeps& deps,
                                                const Lsm6dsv::Sample& scaled);
void imuPipelineRecordSampleProcessTime(ImuSamplePipelineDeps& deps, uint32_t dtUs);
void imuPipelineUpdateRuntimeGyroBiasEstimator(ImuSamplePipelineDeps& deps,
                                               const Lsm6dsv::Sample& scaled,
                                               const Lsm6dsv::Sample& calibrated,
                                               const ImuQualityResult& quality,
                                               uint64_t timestampUs);
void imuPipelineEmitPerSampleOutputs(ImuSamplePipelineDeps& deps,
                                     const Lsm6dsv::RawSample& raw,
                                     const Lsm6dsv::Sample& scaled,
                                     const Lsm6dsv::Sample& calibrated,
                                     const ImuQualityResult& quality);
FifoRuntimeSampleResult imuSamplePipelineProcessRaw(ImuSamplePipelineDeps& deps,
                                                    const Lsm6dsv::RawSample& raw,
                                                    bool checkFifoStatsDelta);

} // namespace tracker
