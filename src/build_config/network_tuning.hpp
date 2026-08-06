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
// cable-free capture/diagnostics. Debug and the explicit ProductionDiag
// environment enable it; normal Production and Slim compile it out.
#ifndef TRACKER_REMOTE_CONSOLE_PORT
#define TRACKER_REMOTE_CONSOLE_PORT 7777
#endif

#ifndef TRACKER_REMOTE_CONSOLE_BYTES_PER_LOOP
  #if TRACKER_BUILD_IS_DEBUG
    #define TRACKER_REMOTE_CONSOLE_BYTES_PER_LOOP 32
  #elif TRACKER_BUILD_IS_PRODUCTION_DIAG
    #define TRACKER_REMOTE_CONSOLE_BYTES_PER_LOOP 32
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_REMOTE_CONSOLE_BYTES_PER_LOOP 16
  #else
    #define TRACKER_REMOTE_CONSOLE_BYTES_PER_LOOP 0
  #endif
#endif

// Telnet carries the long diagnostic reports, so its queue is intentionally
// sized for complete perf/motion/slime bursts. The tracker currently has ample
// RAM headroom; reduce this compile-time value first if a future subsystem needs
// memory, rather than increasing the drain rate and competing with RotationData.
#ifndef TRACKER_REMOTE_CONSOLE_OUTPUT_QUEUE_BYTES
  #if TRACKER_BUILD_IS_DEBUG || TRACKER_BUILD_IS_PRODUCTION_DIAG
    #define TRACKER_REMOTE_CONSOLE_OUTPUT_QUEUE_BYTES 8192
  #else
    #define TRACKER_REMOTE_CONSOLE_OUTPUT_QUEUE_BYTES 1
  #endif
#endif

#ifndef TRACKER_REMOTE_CONSOLE_OUTPUT_RECORD_BYTES
  #if TRACKER_BUILD_IS_DEBUG || TRACKER_BUILD_IS_PRODUCTION_DIAG
    #define TRACKER_REMOTE_CONSOLE_OUTPUT_RECORD_BYTES 768
  #else
    #define TRACKER_REMOTE_CONSOLE_OUTPUT_RECORD_BYTES 1
  #endif
#endif

#ifndef TRACKER_REMOTE_CONSOLE_OUTPUT_BYTES_PER_DRAIN
  #if TRACKER_BUILD_IS_DEBUG || TRACKER_BUILD_IS_PRODUCTION_DIAG
    #define TRACKER_REMOTE_CONSOLE_OUTPUT_BYTES_PER_DRAIN 128
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_REMOTE_CONSOLE_OUTPUT_BYTES_PER_DRAIN 64
  #else
    #define TRACKER_REMOTE_CONSOLE_OUTPUT_BYTES_PER_DRAIN 0
  #endif
#endif

#ifndef TRACKER_REMOTE_CONSOLE_OUTPUT_DRAIN_INTERVAL_MS
  #if TRACKER_BUILD_IS_DEBUG || TRACKER_BUILD_IS_PRODUCTION_DIAG
    #define TRACKER_REMOTE_CONSOLE_OUTPUT_DRAIN_INTERVAL_MS 4UL
  #elif TRACKER_BUILD_IS_PRODUCTION_FAMILY
    #define TRACKER_REMOTE_CONSOLE_OUTPUT_DRAIN_INTERVAL_MS 5UL
  #else
    #define TRACKER_REMOTE_CONSOLE_OUTPUT_DRAIN_INTERVAL_MS 0UL
  #endif
#endif


#ifndef TRACKER_REMOTE_CONSOLE_ACCEPT_POLL_INTERVAL_MS
// The listening socket is diagnostic-only. Polling it at 20 Hz keeps connect
// latency bounded without charging server.available() to every tracker loop.
#define TRACKER_REMOTE_CONSOLE_ACCEPT_POLL_INTERVAL_MS 50UL
#endif

#ifndef TRACKER_REMOTE_CONSOLE_SESSION_LEASE_MS
// The host capture tool sends Telnet NOP keepalives every five seconds. If no
// input can be consumed for this complete window, the capture is already
// unsuitable as release evidence; release its log/test ownership and sleep
// blocker instead of relying on the much longer TCP half-open timeout.
#define TRACKER_REMOTE_CONSOLE_SESSION_LEASE_MS 30000UL
#endif

// Network/SlimeVR update budget. This does not affect IMU/FIFO/AHRS cadence;
// it only avoids spinning Wi-Fi/UDP state machines on every high-rate loop.
// The values remain well below RotationData periods (100 Hz = 10 ms, Slim
// default 50 Hz = 20 ms) and keep ping/config handling latency small.
#ifndef TRACKER_WIFI_DIAGNOSTIC_INFO_REFRESH_MS
// Link state is still checked at TrackerWifiManager cadence. RSSI, power-save,
// TX power and MAC are diagnostic/telemetry fields and need not call ESP-IDF
// getters four times per second.
#define TRACKER_WIFI_DIAGNOSTIC_INFO_REFRESH_MS 1000UL
#endif

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
  #else
    #define TRACKER_SLIMEVR_SERVICE_UPDATE_INTERVAL_MS 10UL
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
  #if !TRACKER_BUILD_IS_SLIM
    #define TRACKER_SLIMEVR_RUNTIME_CONFIG_REFRESH_MS 250UL
  #else
    #define TRACKER_SLIMEVR_RUNTIME_CONFIG_REFRESH_MS 1000UL
  #endif
#endif


// Live telemetry state is cheaper than a full runtime configure(), but it still
// reads battery runtime state and may request SensorInfo refreshes when rest
// calibration changes. Debug uses the same hot-path cadence as ProductionDiag;
// explicit commands still apply immediately.
#ifndef TRACKER_SLIMEVR_LIVE_STATE_REFRESH_MS
  #if !TRACKER_BUILD_IS_SLIM
    #define TRACKER_SLIMEVR_LIVE_STATE_REFRESH_MS 1000UL
  #else
    #define TRACKER_SLIMEVR_LIVE_STATE_REFRESH_MS 5000UL
  #endif
#endif

#ifndef TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS
#define TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS 15000UL
#endif


#ifndef TRACKER_SLIMEVR_SIGNAL_TELEMETRY_INTERVAL_MS
#define TRACKER_SLIMEVR_SIGNAL_TELEMETRY_INTERVAL_MS 15000UL
#endif

#ifndef TRACKER_SLIMEVR_TEMPERATURE_TELEMETRY_INTERVAL_MS
#define TRACKER_SLIMEVR_TEMPERATURE_TELEMETRY_INTERVAL_MS 15000UL
#endif

#ifndef TRACKER_SLIMEVR_BATTERY_TELEMETRY_INTERVAL_MS
#define TRACKER_SLIMEVR_BATTERY_TELEMETRY_INTERVAL_MS 30000UL
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

#ifndef TRACKER_SLIMEVR_USE_COMPACT_MOTION_PACKET
// Experimental packet 23 override. It is not negotiated by the legacy
// FeatureFlags bit used for packet-100 bundles, and some step-mounting beta
// branches contain the packet class but omit its parser mapping. Keep it off
// by default; enable only for a server build explicitly verified to parse 23.
#define TRACKER_SLIMEVR_USE_COMPACT_MOTION_PACKET 0
#endif

#ifndef TRACKER_SLIMEVR_ENABLE_BUNDLE_NEGOTIATION
// Request server FeatureFlags and use packet 100 only after bit 0 confirms
// PROTOCOL_BUNDLE_SUPPORT. The bundle contains only packet 17 + packet 4.
#define TRACKER_SLIMEVR_ENABLE_BUNDLE_NEGOTIATION 1
#endif

#ifndef TRACKER_SLIMEVR_FEATURE_FLAGS_REQUEST_INTERVAL_MS
#define TRACKER_SLIMEVR_FEATURE_FLAGS_REQUEST_INTERVAL_MS 500UL
#endif

#ifndef TRACKER_SLIMEVR_FEATURE_FLAGS_REQUEST_ATTEMPTS
#define TRACKER_SLIMEVR_FEATURE_FLAGS_REQUEST_ATTEMPTS 15U
#endif

#ifndef TRACKER_SLIMEVR_FALLBACK_ACCEL_RATE_HZ
// Old servers without packet-100 support receive rotation at the configured
// pose rate and coherent packet-4 acceleration at this reduced rate. Step
// mounting integrates real callback timestamps and does not require 100 Hz.
#define TRACKER_SLIMEVR_FALLBACK_ACCEL_RATE_HZ 50U
#endif

// ESP-IDF documents that UDP sendto() can fail with ENOMEM while Wi-Fi TX
// buffers are temporarily full. Treat ENOMEM/ENOBUFS/EAGAIN as pressure, not
// immediate socket corruption: skip stale motion for one or more output slots,
// then recover only when no successful motion datagram has passed for a bounded
// interval. This prevents a multi-tracker contention burst from becoming a
// rebind/discovery storm.
#ifndef TRACKER_SLIMEVR_TX_BACKOFF_INITIAL_MS
#define TRACKER_SLIMEVR_TX_BACKOFF_INITIAL_MS 10UL
#endif

#ifndef TRACKER_SLIMEVR_TX_BACKOFF_MAX_MS
#define TRACKER_SLIMEVR_TX_BACKOFF_MAX_MS 80UL
#endif

#ifndef TRACKER_SLIMEVR_TX_OTHER_REBIND_CONSECUTIVE_FAILURES
#define TRACKER_SLIMEVR_TX_OTHER_REBIND_CONSECUTIVE_FAILURES 4U
#endif

#ifndef TRACKER_SLIMEVR_TX_FAILURE_WINDOW_ATTEMPTS
#define TRACKER_SLIMEVR_TX_FAILURE_WINDOW_ATTEMPTS 32U
#endif

#ifndef TRACKER_SLIMEVR_TX_FAILURE_WINDOW_FAILURES
#define TRACKER_SLIMEVR_TX_FAILURE_WINDOW_FAILURES 8U
#endif

#ifndef TRACKER_SLIMEVR_TX_RECENT_RX_MS
#define TRACKER_SLIMEVR_TX_RECENT_RX_MS 2000UL
#endif

#ifndef TRACKER_SLIMEVR_TX_PRESSURE_REBIND_STALL_MS
// A local socket rebind is justified only when no motion datagram succeeds for
// this long. Intermittent ENOMEM with continuing successful motion remains a
// transient pressure episode and never churns the socket.
#define TRACKER_SLIMEVR_TX_PRESSURE_REBIND_STALL_MS 500UL
#endif

#ifndef TRACKER_SLIMEVR_TX_POST_REBIND_REOPEN_STALL_MS
// After a successful local rebind, escalate to full discovery only when motion
// still cannot pass for this additional bounded interval.
#define TRACKER_SLIMEVR_TX_POST_REBIND_REOPEN_STALL_MS 1000UL
#endif

#ifndef TRACKER_SLIMEVR_TX_PRESSURE_STABLE_RESET_MS
// Close a pressure episode only after a genuinely stable motion-success period;
// one isolated success does not erase the episode history.
#define TRACKER_SLIMEVR_TX_PRESSURE_STABLE_RESET_MS 1000UL
#endif

#ifndef TRACKER_SLIMEVR_TX_PRESSURE_ACTIVE_ATTEMPT_MS
// Recovery requires evidence that fresh motion is still being attempted and
// failing. A paused/duplicate snapshot source must not be mistaken for a dead
// socket merely because no successful motion was observed recently.
#define TRACKER_SLIMEVR_TX_PRESSURE_ACTIVE_ATTEMPT_MS 250UL
#endif

#ifndef TRACKER_SLIMEVR_TX_REBIND_COOLDOWN_MS
#define TRACKER_SLIMEVR_TX_REBIND_COOLDOWN_MS 5000UL
#endif

#ifndef TRACKER_SLIMEVR_TX_FULL_REOPEN_COOLDOWN_MS
#define TRACKER_SLIMEVR_TX_FULL_REOPEN_COOLDOWN_MS 10000UL
#endif

#ifndef TRACKER_SLIMEVR_TX_REBIND_SEND_GRACE_MS
#define TRACKER_SLIMEVR_TX_REBIND_SEND_GRACE_MS 20UL
#endif
