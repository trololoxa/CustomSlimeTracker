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
#if TRACKER_HAS_RUNTIME_PROFILER
#include "runtime/runtime_motion_diagnostics.hpp"
#endif
#include "runtime/static_test_runner.hpp"
#include "runtime/gyro_temp_calibration_capture.hpp"
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
using ImuPipelineQualityCallback = void (*)(const ImuQualityResult& quality,
                                               const Vec3& gyroRadS,
                                               const Vec3& accelG,
                                               uint64_t timestampUs,
                                               bool ahrsIntegrated,
                                               void* user);
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
    TrackerSerialStreamState* streamState = nullptr;
    TrackerSerialLogState* logState = nullptr;
    MachineLogCounters* logCounters = nullptr;
    StaticTestRunner* staticTestRunner = nullptr;
    GyroTempCalibrationCapture* gyroTempCapture = nullptr;
    TrackerPerfCounters& perf;
#if TRACKER_HAS_RUNTIME_PROFILER
    RuntimeMotionDiagnostics* motionDiagnostics = nullptr;
#endif
    FifoCalibrationIo* calibrationIo;
    Stream& out;
    uint32_t& runtimeSamples;
    uint64_t& lastSampleTimestampUs;
    float& latestTempC;
    float& lastOutputConfidence;
    uint32_t* lastQualityFlags = nullptr;
    ImuSamplePipelineCallbacks callbacks;

    // Optional latest-sample mirrors for blocking guided calibration flows.
    // They let setup calibration observe normal runtime samples without
    // recursively draining FIFO or bypassing the production sample pipeline.
    Lsm6dsv::Sample* lastScaledSample = nullptr;
    Lsm6dsv::Sample* lastCalibratedSample = nullptr;
    uint32_t* lastImuSampleSequence = nullptr;
};

void imuPipelineUpdateLatestTemperature(ImuSamplePipelineDeps& deps);
Vec3 imuPipelineCurrentGyroBiasRadS(const ImuSamplePipelineDeps& deps, float tempC);
Lsm6dsv::Sample imuPipelineMakeSensorFrameCalibratedSample(const ImuSamplePipelineDeps& deps,
                                                           const Lsm6dsv::Sample& scaled);
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
