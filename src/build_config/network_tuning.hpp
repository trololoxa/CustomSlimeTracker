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

#ifndef TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS
  #if TRACKER_BUILD_IS_DEBUG
    #define TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS 5000UL
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS 15000UL
  #else
    #define TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS 0UL
  #endif
#endif

#ifndef TRACKER_SLIMEVR_ENABLE_SIGNAL_TELEMETRY
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SLIMEVR_ENABLE_SIGNAL_TELEMETRY 0
  #else
    #define TRACKER_SLIMEVR_ENABLE_SIGNAL_TELEMETRY 1
  #endif
#endif

#ifndef TRACKER_SLIMEVR_ENABLE_TEMPERATURE_TELEMETRY
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SLIMEVR_ENABLE_TEMPERATURE_TELEMETRY 0
  #else
    #define TRACKER_SLIMEVR_ENABLE_TEMPERATURE_TELEMETRY 1
  #endif
#endif

#ifndef TRACKER_SLIMEVR_ENABLE_BATTERY_TELEMETRY
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SLIMEVR_ENABLE_BATTERY_TELEMETRY 0
  #else
    #define TRACKER_SLIMEVR_ENABLE_BATTERY_TELEMETRY 1
  #endif
#endif

#ifndef TRACKER_SLIMEVR_SERVER_SILENCE_TIMEOUT_MS
#define TRACKER_SLIMEVR_SERVER_SILENCE_TIMEOUT_MS 15000UL
#endif

#ifndef TRACKER_SLIMEVR_SEND_FAILURE_REOPEN_THRESHOLD
#define TRACKER_SLIMEVR_SEND_FAILURE_REOPEN_THRESHOLD 5UL
#endif
