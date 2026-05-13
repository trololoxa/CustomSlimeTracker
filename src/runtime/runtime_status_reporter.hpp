#pragma once

#include <Arduino.h>
#include <cstdint>

#include "config/tracker_config_runtime.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "core/math.hpp"
#include "runtime/fifo_runtime_processor.hpp"
#include "runtime/mag_runtime_state.hpp"
#include "runtime/output_runtime.hpp"
#include "runtime/runtime_bias_types.hpp"
#include "runtime/tracking_state_controller.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/calibration.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"
#include "sensor/mag_heading.hpp"
#include "sensor/mag_runtime.hpp"
#include "sensor/mag_yaw_correction.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

struct RuntimeStatusReporterDeps {
    const TrackerConfig* config = nullptr;
    bool configLoadedFromNvs = false;
    uint32_t spiHz = 0;

    uint32_t runtimeSamples = 0;
    uint32_t fifoIntCount = 0;
    const FifoInterruptEventSource* fifoEvents = nullptr;
    const Lsm6dsvFifoReader* fifo = nullptr;

    float latestTempC = 0.0f;
    const ImuCalibration* imuCal = nullptr;
    const GyroTempCompensator* gyroTempComp = nullptr;
    const RuntimeGyroBiasEstimator* runtimeBias = nullptr;
    const ImuQualityMonitor* quality = nullptr;
    const Ahrs6Dof* ahrs = nullptr;
    const TrackingStateController* trackingState = nullptr;
    const char* trackingStateName = "UNKNOWN";
    float lastOutputConfidence = 0.0f;

    const TrackerSerialStreamState* streamState = nullptr;
    PreparedOutputRuntime* preparedOutput = nullptr;

    const MagRuntimeState* magState = nullptr;
    const MagRuntimeProcessor* magProcessor = nullptr;
    const MagProcessedSample* lastMagProcessed = nullptr;
    const MagHeadingEstimator* magHeading = nullptr;
    const MagHeadingSample* lastMagHeading = nullptr;
    const MagHeadingReferenceState* magHeadingRef = nullptr;
    const MagHeadingAutoReferenceState* magHeadingAutoRef = nullptr;
    const MagYawCorrectionOutput* lastMagYawCorrection = nullptr;
};

const char* runtimeStatusStreamModeName(TrackerStreamMode mode);
float runtimeStatusMagHeadingErrorDeg(const MagHeadingReferenceState& ref,
                                      const MagHeadingSample& heading);
void runtimeStatusPrint(Stream& out, const RuntimeStatusReporterDeps& deps);
void runtimeStatusPrintHealth(Stream& out, const RuntimeStatusReporterDeps& deps);

} // namespace tracker
