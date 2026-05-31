#pragma once

#include "build_config/build_profiles.hpp"
#include "build_config/feature_flags.hpp"

// Network / SlimeVR transport defaults.
// TRACKER_WIFI_TX_POWER intentionally references the ESP32 Arduino WiFi enum
// token, but is kept as a macro so board profiles can override it from
// platformio build_flags or by editing build_config/network_tuning.hpp. It is
// only consumed by the ESP32 Wi-Fi adapter implementation after including WiFi.h.
#ifndef TRACKER_WIFI_APPLY_TX_POWER_AFTER_BEGIN
#define TRACKER_WIFI_APPLY_TX_POWER_AFTER_BEGIN 1
#endif

#ifndef TRACKER_WIFI_TX_POWER
#define TRACKER_WIFI_TX_POWER WIFI_POWER_8_5dBm
#endif



// Wi-Fi power save policy for wave-5 A/B tests. Keep the default at NONE so
// existing stable behavior is preserved until runtime tests approve modem sleep.
#define TRACKER_WIFI_POWER_SAVE_NONE 0
#define TRACKER_WIFI_POWER_SAVE_MIN_MODEM 1
#define TRACKER_WIFI_POWER_SAVE_MAX_MODEM 2

#ifndef TRACKER_WIFI_POWER_SAVE_MODE
#define TRACKER_WIFI_POWER_SAVE_MODE TRACKER_WIFI_POWER_SAVE_NONE
#endif

#ifndef TRACKER_WIFI_RESTORE_POWER_SAVE_AFTER_SCAN
#define TRACKER_WIFI_RESTORE_POWER_SAVE_AFTER_SCAN 1
#endif

#ifndef TRACKER_WIFI_CONNECT_TIMEOUT_MS
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_WIFI_CONNECT_TIMEOUT_MS 20000UL
  #else
    #define TRACKER_WIFI_CONNECT_TIMEOUT_MS 15000UL
  #endif
#endif

#ifndef TRACKER_WIFI_RECONNECT_BACKOFF_MS
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_WIFI_RECONNECT_BACKOFF_MS 10000UL
  #else
    #define TRACKER_WIFI_RECONNECT_BACKOFF_MS 5000UL
  #endif
#endif

#ifndef TRACKER_WIFI_STATUS_POLL_INTERVAL_MS
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_WIFI_STATUS_POLL_INTERVAL_MS 500UL
  #else
    #define TRACKER_WIFI_STATUS_POLL_INTERVAL_MS 250UL
  #endif
#endif


// TCP remote CLI console. It reuses the serial CLI parser over Wi-Fi for
// cable-free setup/calibration. Debug and Production enable the feature by
// default through feature_flags.hpp; Slim keeps it compiled out. Runtime
// command `remote off` stops the TCP server/client for the current boot.
#ifndef TRACKER_REMOTE_CONSOLE_PORT
#define TRACKER_REMOTE_CONSOLE_PORT 7777
#endif

#ifndef TRACKER_REMOTE_CONSOLE_BYTES_PER_LOOP
  #if TRACKER_BUILD_IS_DEBUG
    #define TRACKER_REMOTE_CONSOLE_BYTES_PER_LOOP 32
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_REMOTE_CONSOLE_BYTES_PER_LOOP 16
  #else
    #define TRACKER_REMOTE_CONSOLE_BYTES_PER_LOOP 0
  #endif
#endif

// Network/SlimeVR update budget. This does not affect IMU/FIFO/AHRS cadence;
// it only avoids spinning Wi-Fi/UDP state machines on every high-rate loop.
// The values remain well below RotationData periods (100 Hz = 10 ms, Slim
// default 50 Hz = 20 ms) and keep ping/config handling latency small.
#ifndef TRACKER_NETWORK_RUNTIME_UPDATE_INTERVAL_MS
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_NETWORK_RUNTIME_UPDATE_INTERVAL_MS 5UL
  #else
    #define TRACKER_NETWORK_RUNTIME_UPDATE_INTERVAL_MS 2UL
  #endif
#endif


// Slow SlimeVR service cadence. RotationData is checked on the normal network
// runtime cadence, while incoming packets, heartbeat, SensorInfo, telemetry,
// discovery and server-silence checks can run less often. This reduces UDP
// service-path churn without touching FIFO/AHRS cadence.
#ifndef TRACKER_SLIMEVR_SERVICE_UPDATE_INTERVAL_MS
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SLIMEVR_SERVICE_UPDATE_INTERVAL_MS 20UL
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_SLIMEVR_SERVICE_UPDATE_INTERVAL_MS 10UL
  #else
    #define TRACKER_SLIMEVR_SERVICE_UPDATE_INTERVAL_MS 5UL
  #endif
#endif

#ifndef TRACKER_SLIMEVR_DISCOVERY_INTERVAL_MS
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SLIMEVR_DISCOVERY_INTERVAL_MS 2000UL
  #else
    #define TRACKER_SLIMEVR_DISCOVERY_INTERVAL_MS 1000UL
  #endif
#endif

#ifndef TRACKER_SLIMEVR_INCOMING_PACKETS_PER_UPDATE
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SLIMEVR_INCOMING_PACKETS_PER_UPDATE 1
  #else
    #define TRACKER_SLIMEVR_INCOMING_PACKETS_PER_UPDATE 4
  #endif
#endif

#ifndef TRACKER_SLIMEVR_OUTPUT_RATE_HZ_MAX
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SLIMEVR_OUTPUT_RATE_HZ_MAX 125
  #else
    #define TRACKER_SLIMEVR_OUTPUT_RATE_HZ_MAX 1000
  #endif
#endif

// Build-enforced RotationData target. Slim has no CLI, so it cannot rely on
// `slime rate` after flashing; force 125 TPS at runtime even if an older NVS
// config contains 50/100 Hz from a previous profile. Other profiles keep the
// NVS/user-configured rate.
#ifndef TRACKER_SLIMEVR_FORCE_ROTATION_RATE_HZ
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SLIMEVR_FORCE_ROTATION_RATE_HZ 125
  #else
    #define TRACKER_SLIMEVR_FORCE_ROTATION_RATE_HZ 0
  #endif
#endif

#ifndef TRACKER_SLIMEVR_RUNTIME_CONFIG_REFRESH_MS
  #if TRACKER_BUILD_IS_DEBUG
    #define TRACKER_SLIMEVR_RUNTIME_CONFIG_REFRESH_MS 0UL
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_SLIMEVR_RUNTIME_CONFIG_REFRESH_MS 250UL
  #else
    #define TRACKER_SLIMEVR_RUNTIME_CONFIG_REFRESH_MS 1000UL
  #endif
#endif


// Live telemetry state is cheaper than a full runtime configure(), but it still
// reads battery runtime state and may request SensorInfo refreshes when rest
// calibration changes. Keep Debug immediate, and throttle Product/Slim.
#ifndef TRACKER_SLIMEVR_LIVE_STATE_REFRESH_MS
  #if TRACKER_BUILD_IS_DEBUG
    #define TRACKER_SLIMEVR_LIVE_STATE_REFRESH_MS 0UL
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_SLIMEVR_LIVE_STATE_REFRESH_MS 1000UL
  #else
    #define TRACKER_SLIMEVR_LIVE_STATE_REFRESH_MS 5000UL
  #endif
#endif

#ifndef TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS
  #if TRACKER_BUILD_IS_DEBUG
    #define TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS 5000UL
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS 15000UL
  #else
    #define TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS 15000UL
  #endif
#endif


#ifndef TRACKER_SLIMEVR_SIGNAL_TELEMETRY_INTERVAL_MS
  #if TRACKER_BUILD_IS_DEBUG
    #define TRACKER_SLIMEVR_SIGNAL_TELEMETRY_INTERVAL_MS 5000UL
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_SLIMEVR_SIGNAL_TELEMETRY_INTERVAL_MS 15000UL
  #else
    #define TRACKER_SLIMEVR_SIGNAL_TELEMETRY_INTERVAL_MS 15000UL
  #endif
#endif

#ifndef TRACKER_SLIMEVR_TEMPERATURE_TELEMETRY_INTERVAL_MS
  #if TRACKER_BUILD_IS_DEBUG
    #define TRACKER_SLIMEVR_TEMPERATURE_TELEMETRY_INTERVAL_MS 5000UL
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_SLIMEVR_TEMPERATURE_TELEMETRY_INTERVAL_MS 15000UL
  #else
    #define TRACKER_SLIMEVR_TEMPERATURE_TELEMETRY_INTERVAL_MS 15000UL
  #endif
#endif

#ifndef TRACKER_SLIMEVR_BATTERY_TELEMETRY_INTERVAL_MS
  #if TRACKER_BUILD_IS_DEBUG
    #define TRACKER_SLIMEVR_BATTERY_TELEMETRY_INTERVAL_MS 5000UL
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_SLIMEVR_BATTERY_TELEMETRY_INTERVAL_MS 30000UL
  #else
    #define TRACKER_SLIMEVR_BATTERY_TELEMETRY_INTERVAL_MS 30000UL
  #endif
#endif

#ifndef TRACKER_SLIMEVR_ENABLE_SIGNAL_TELEMETRY
#define TRACKER_SLIMEVR_ENABLE_SIGNAL_TELEMETRY 1
#endif

#ifndef TRACKER_SLIMEVR_ENABLE_TEMPERATURE_TELEMETRY
#define TRACKER_SLIMEVR_ENABLE_TEMPERATURE_TELEMETRY 1
#endif

#ifndef TRACKER_SLIMEVR_ENABLE_BATTERY_TELEMETRY
#define TRACKER_SLIMEVR_ENABLE_BATTERY_TELEMETRY 1
#endif

#ifndef TRACKER_SLIMEVR_SERVER_SILENCE_TIMEOUT_MS
#define TRACKER_SLIMEVR_SERVER_SILENCE_TIMEOUT_MS 15000UL
#endif

#ifndef TRACKER_SLIMEVR_SEND_FAILURE_REOPEN_THRESHOLD
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SLIMEVR_SEND_FAILURE_REOPEN_THRESHOLD 12UL
  #else
    #define TRACKER_SLIMEVR_SEND_FAILURE_REOPEN_THRESHOLD 20UL
  #endif
#endif
