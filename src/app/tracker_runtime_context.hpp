#pragma once

// Runtime context: firmware-wide runtime objects and state counters. This file
// owns logical tracker state, but does not own board transports/ISR objects.

#include <Arduino.h>

#include "core/math.hpp"
#include "sensor/calibration.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"
#include "sensor/fifo_calibrations.hpp"
#include "sensor/mag_calibration.hpp"
#include "sensor/mag_runtime.hpp"
#include "sensor/mag_heading.hpp"
#include "sensor/mag_yaw_correction.hpp"
#include "runtime/static_test_runner.hpp"
#include "runtime/tracker_runtime_types.hpp"
#include "runtime/fifo_runtime_processor.hpp"
#include "runtime/runtime_bias_types.hpp"
#include "runtime/mag_runtime_state.hpp"
#include "runtime/machine_log_runtime.hpp"
#include "runtime/tracking_state_controller.hpp"
#include "runtime/output_runtime.hpp"
#include "runtime/mag_runtime_controller.hpp"
#include "app/tracker_app.hpp"
#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "config/tracker_network_config.hpp"
#include "network/wifi_manager.hpp"
#include "network/esp32_wifi_station.hpp"
#include "serial/tracker_serial_commands.hpp"

using namespace tracker;

static TrackerConfig g_config;
static TrackerConfigStore g_configStore;

static TrackerNetworkConfig g_networkConfig;
static TrackerNetworkConfigStore g_networkConfigStore;
static bool g_networkConfigLoadedFromNvs = false;
static Esp32WifiStationAdapter g_wifiStation;
static TrackerWifiManager g_wifiManager;

static ImuCalibration g_imuCal;
static GyroTempCompensator g_gyroTempComp;
static ImuQualityMonitor g_quality;
static Ahrs6Dof g_ahrs6dof;

static TrackerSerialStreamState g_streamState;
static TrackerSerialLogState g_logState;
static TrackerSerialCommandContext g_cmdCtx;
static TrackerSerialCommandInterface<> g_cli;
static TrackerApp g_app;

static FifoCalibrationIo g_calIo;
static FifoAccel6PosCalibrationRunner g_accelCalRunner;

static uint64_t g_lastSampleTimestampUs = 0;
static uint32_t g_runtimeSamples = 0;
static float g_latestTempC = 25.0f;
static bool g_configLoadedFromNvs = false;
static uint32_t g_lastHeartbeatMs = 0;
static float g_lastOutputConfidence = 0.0f;

static TrackerPerfCounters g_perf;
static FifoInterruptEventSource g_fifoEvents;
static FifoRuntimeProcessor g_fifoRuntime;

static PreparedOutputRuntime g_preparedOutput;

static TrackingStateController g_trackingState;

static MagRuntimeState g_magState;
static MagCalibrationCollector g_magCalCollector;
static MagRuntimeProcessor g_magProcessor;
static MagProcessedSample g_lastMagProcessed;

static MagHeadingEstimator g_magHeading;
static MagHeadingSample g_lastMagHeading;

static MagYawCorrectionController g_magYawCorrection;
static MagRuntimeController g_magRuntime;
static MagYawCorrectionOutput g_lastMagYawCorrection;

static MagHeadingReferenceState g_magHeadingRef;
static MagHeadingAutoReferenceState g_magHeadingAutoRef;

static StaticRuntimeTest g_staticTest;
static StaticRuntimeTest g_lastCompletedStaticTest;
static StaticTestRunner g_staticTestRunner;
static bool g_lastCompletedStaticTestValid = false;
static uint32_t g_lastCompletedStaticTestFinishedMs = 0;

static RuntimeGyroBiasEstimator g_runtimeBias;

static MachineLogCounters g_logCounters;
static uint32_t g_lastBiasLogEmitUs = 0;
static uint32_t g_lastRecoveryConsolePrintMs = 0;
static constexpr uint32_t MACHINE_BIAS_LOG_PERIOD_US = 1000000UL; // 1 Hz: enough for temp/bias tracking and safer for FIFO while logging.
static constexpr uint32_t RECOVERY_CONSOLE_THROTTLE_MS = 1000UL;
