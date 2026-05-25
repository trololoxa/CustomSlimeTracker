#pragma once

// Runtime context: firmware-wide runtime objects and state counters. This file
// owns logical tracker state, but does not own board transports/ISR objects.

#include <Arduino.h>

#include "defines.h"
#include "core/math.hpp"
#include "sensor/calibration.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"
#if TRACKER_HAS_CALIBRATION_UI
#include "sensor/fifo_calibrations.hpp"
#endif
#include "sensor/mag_calibration.hpp"
#include "sensor/mag_runtime.hpp"
#include "sensor/mag_heading.hpp"
#include "sensor/mag_yaw_correction.hpp"
#if TRACKER_HAS_STATIC_TEST_STATE
#include "runtime/static_test_runner.hpp"
#endif
#if TRACKER_HAS_CALIBRATION_UI
#include "runtime/gyro_temp_calibration_capture.hpp"
#endif
#if TRACKER_HAS_RUNTIME_TEST_STATE
#include "runtime/runtime_test_runner.hpp"
#endif
#include "runtime/tracker_runtime_types.hpp"
#include "runtime/fifo_runtime_processor.hpp"
#include "runtime/runtime_bias_types.hpp"
#include "runtime/mag_runtime_state.hpp"
#if TRACKER_HAS_MACHINE_LOG
#include "runtime/machine_log_runtime.hpp"
#endif
#include "runtime/tracking_state_controller.hpp"
#include "runtime/output_runtime.hpp"
#include "runtime/mag_runtime_controller.hpp"
#include "app/tracker_app.hpp"
#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "config/tracker_network_config.hpp"
#include "network/wifi_manager.hpp"
#include "network/esp32_wifi_station.hpp"
#include "network/esp32_udp_transport.hpp"
#if TRACKER_HAS_WIFI_REMOTE_CONSOLE
#include "network/wifi_remote_console.hpp"
#endif
#include "runtime/slimevr_output_runtime.hpp"
#if TRACKER_HAS_TAP_RUNTIME
#include "runtime/tap_runtime_controller.hpp"
#endif
#if TRACKER_HAS_STATUS_LED
#include "runtime/status_led_runtime.hpp"
#endif
#if TRACKER_HAS_BATTERY_RUNTIME
#include "runtime/battery_runtime.hpp"
#endif
#if TRACKER_HAS_SERIAL_STREAM_STATE || TRACKER_HAS_SERIAL_CLI || TRACKER_HAS_MACHINE_LOG
#include "serial/tracker_serial_context.hpp"
#endif
#if TRACKER_HAS_SERIAL_CLI
#include "serial/tracker_serial_commands.hpp"
#endif

using namespace tracker;

static TrackerConfig g_config;
static TrackerConfigStore g_configStore;

static TrackerNetworkConfig g_networkConfig;
static TrackerNetworkConfigStore g_networkConfigStore;
static bool g_networkConfigLoadedFromNvs = false;
static Esp32WifiStationAdapter g_wifiStation;
static TrackerWifiManager g_wifiManager;
static Esp32UdpTransport g_udpTransport;
#if TRACKER_HAS_WIFI_REMOTE_CONSOLE
static WifiRemoteConsoleRuntime g_wifiRemoteConsole;
#endif
static SlimeVROutputRuntime g_slimevrRuntime;
#if TRACKER_HAS_TAP_RUNTIME
static TapRuntimeController g_tapRuntime;
#endif
#if TRACKER_HAS_STATUS_LED
static GpioStatusLedSink g_statusLedSink;
static StatusLedRuntime g_statusLedRuntime;
static bool g_statusLedSensorError = false;
#endif
#if TRACKER_HAS_BATTERY_RUNTIME
static BatteryRuntime g_batteryRuntime;
#endif

static ImuCalibration g_imuCal;
static GyroTempCompensator g_gyroTempComp;
static ImuQualityMonitor g_quality;
static Ahrs6Dof g_ahrs6dof;

#if TRACKER_HAS_SERIAL_STREAM_STATE
static TrackerSerialStreamState g_streamState;
#endif
#if TRACKER_HAS_MACHINE_LOG
static TrackerSerialLogState g_logState;
#endif
#if TRACKER_HAS_SERIAL_CLI
static TrackerSerialCommandContext g_cmdCtx;
static TrackerSerialCommandInterface<> g_cli;
#endif
static TrackerApp g_app;

#if TRACKER_HAS_CALIBRATION_UI
static FifoCalibrationIo g_calIo;
static FifoAccel6PosCalibrationRunner g_accelCalRunner;
#endif

static uint64_t g_lastSampleTimestampUs = 0;
static uint32_t g_runtimeSamples = 0;
static float g_latestTempC = 25.0f;
static bool g_configLoadedFromNvs = false;
static uint32_t g_lastHeartbeatMs = 0;
static float g_lastOutputConfidence = 0.0f;
static uint32_t g_lastQualityFlags = 0;

static TrackerPerfCounters g_perf;
static FifoInterruptEventSource g_fifoEvents;
static FifoRuntimeProcessor g_fifoRuntime;

static PreparedOutputRuntime g_preparedOutput;

static TrackingStateController g_trackingState;

static MagRuntimeState g_magState;
static MagCalibrationCollector g_magCalCollector;
static MagRuntimeProcessor g_magProcessor;
static MagProcessedSample g_lastMagProcessed;
static Lsm6dsv::Sample g_lastScaledSample;
static Lsm6dsv::Sample g_lastCalibratedSample;
static uint32_t g_lastImuSampleSequence = 0;

static MagHeadingEstimator g_magHeading;
static MagHeadingSample g_lastMagHeading;

static MagYawCorrectionController g_magYawCorrection;
static MagRuntimeController g_magRuntime;
static MagYawCorrectionOutput g_lastMagYawCorrection;

static MagHeadingReferenceState g_magHeadingRef;
static MagHeadingAutoReferenceState g_magHeadingAutoRef;

#if TRACKER_HAS_STATIC_TEST_STATE
static StaticRuntimeTest g_staticTest;
static StaticRuntimeTest g_lastCompletedStaticTest;
static StaticTestRunner g_staticTestRunner;
static bool g_lastCompletedStaticTestValid = false;
static uint32_t g_lastCompletedStaticTestFinishedMs = 0;
#endif
#if TRACKER_HAS_CALIBRATION_UI
static GyroTempCalibrationCapture g_gyroTempCapture;
#endif
#if TRACKER_HAS_RUNTIME_TEST_STATE
static RuntimeTestRunner g_runtimeTestRunner;
#endif

static RuntimeGyroBiasEstimator g_runtimeBias;

#if TRACKER_HAS_MACHINE_LOG
static MachineLogCounters g_logCounters;
static uint32_t g_lastBiasLogEmitUs = 0;
static constexpr uint32_t MACHINE_BIAS_LOG_PERIOD_US = 1000000UL; // 1 Hz: enough for temp/bias tracking and safer for FIFO while logging.
#endif
static uint32_t g_lastRecoveryConsolePrintMs = 0;
static constexpr uint32_t RECOVERY_CONSOLE_THROTTLE_MS = 1000UL;
