#pragma once

#include <Arduino.h>

#include "core/math.hpp"
#include "connection/lsm6dsv_sensorhub.hpp"
#include "sensor/qmc6309.hpp"
#include "sensor/mag_calibration.hpp"
#include "runtime/mag_calibration_fit_quality_reporter.hpp"
#include "sensor/mag_runtime.hpp"
#include "sensor/mag_axis_alignment.hpp"
#include "sensor/mag_field_reliability.hpp"
#include "sensor/mag_heading.hpp"
#include "sensor/mag_yaw_correction.hpp"
#include "runtime/mag_runtime_state.hpp"
#include "runtime/output_runtime.hpp"
#include "config/tracker_config_runtime.hpp"

namespace tracker {

struct MagStatusReporterDeps {
    TrackerConfig* config = nullptr;
    Qmc6309* qmc = nullptr;
    Lsm6dsvSensorHub* hub = nullptr;

    MagRuntimeState* state = nullptr;
    MagRuntimeProcessor* processor = nullptr;
    MagProcessedSample* lastProcessed = nullptr;

    MagHeadingEstimator* headingEstimator = nullptr;
    MagHeadingSample* lastHeading = nullptr;
    MagFieldReliabilityMonitor* fieldReliability = nullptr;
    MagFieldReliabilityOutput* lastFieldReliability = nullptr;
    MagAxisAlignmentCollector* axisAlignmentCollector = nullptr;
    MagAxisAlignmentRuntimeState* axisAlignmentState = nullptr;
    MagHeadingReferenceState* headingRef = nullptr;
    MagHeadingAutoReferenceState* headingAutoRef = nullptr;

    MagYawCorrectionController* yawCorrection = nullptr;
    MagYawCorrectionOutput* lastYawCorrection = nullptr;

    MagCalibrationCollector* calibrationCollector = nullptr;

    MagRuntimeConfig runtimeConfig;
    MagYawCorrectionConfig yawConfig;
    MagFieldReliabilityConfig fieldReliabilityConfig;
};

float magStatusHeadingErrorToReferenceRad(const MagHeadingReferenceState& ref,
                                          const MagHeadingSample& heading);

void magStatusPrintRuntime(Stream& out, const MagStatusReporterDeps& deps);
void magStatusPrintProcessed(Stream& out, const MagStatusReporterDeps& deps);
void magStatusPrintHeading(Stream& out, const MagStatusReporterDeps& deps);
void magStatusPrintYawCorrection(Stream& out, const MagStatusReporterDeps& deps);
void magStatusPrintCalibration(Stream& out, const MagStatusReporterDeps& deps);

} // namespace tracker
