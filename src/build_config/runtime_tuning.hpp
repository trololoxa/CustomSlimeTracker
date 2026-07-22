#pragma once

#include <cstdint>

#include "build_config/build_profiles.hpp"
#include "build_config/feature_flags.hpp"

// Serial / CLI runtime tuning.
#ifndef TRACKER_CLI_BYTES_PER_LOOP
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_CLI_BYTES_PER_LOOP 0
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_CLI_BYTES_PER_LOOP 16
  #else
    #define TRACKER_CLI_BYTES_PER_LOOP 32
  #endif
#endif

#ifndef TRACKER_STARTUP_DELAY_MS
#define TRACKER_STARTUP_DELAY_MS 300UL
#endif

#ifndef TRACKER_CLI_SECOND_POLL_ENABLED
  #if TRACKER_BUILD_IS_DEBUG
    #define TRACKER_CLI_SECOND_POLL_ENABLED 1
  #else
    #define TRACKER_CLI_SECOND_POLL_ENABLED 0
  #endif
#endif

// Bounded asynchronous USB console output. Serial is primarily used by the
// SlimeVR setup compatibility commands, so it keeps a smaller queue than the
// telnet diagnostic console. Both the queue and per-record staging capacities
// are compile-time knobs and may be reduced if a future feature needs RAM.
#ifndef TRACKER_SERIAL_OUTPUT_QUEUE_BYTES
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SERIAL_OUTPUT_QUEUE_BYTES 1
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_SERIAL_OUTPUT_QUEUE_BYTES 1536
  #else
    #define TRACKER_SERIAL_OUTPUT_QUEUE_BYTES 2048
  #endif
#endif

#ifndef TRACKER_SERIAL_OUTPUT_RECORD_BYTES
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SERIAL_OUTPUT_RECORD_BYTES 1
  #else
    #define TRACKER_SERIAL_OUTPUT_RECORD_BYTES 512
  #endif
#endif

#ifndef TRACKER_SERIAL_OUTPUT_BYTES_PER_DRAIN
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SERIAL_OUTPUT_BYTES_PER_DRAIN 0
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_SERIAL_OUTPUT_BYTES_PER_DRAIN 48
  #else
    #define TRACKER_SERIAL_OUTPUT_BYTES_PER_DRAIN 64
  #endif
#endif

#ifndef TRACKER_SERIAL_OUTPUT_DRAIN_INTERVAL_MS
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SERIAL_OUTPUT_DRAIN_INTERVAL_MS 0UL
  #elif TRACKER_BUILD_IS_PRODUCTION
    #define TRACKER_SERIAL_OUTPUT_DRAIN_INTERVAL_MS 4UL
  #else
    #define TRACKER_SERIAL_OUTPUT_DRAIN_INTERVAL_MS 2UL
  #endif
#endif

#ifndef TRACKER_SERIAL_OUTPUT_STALL_BACKOFF_MAX_MS
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SERIAL_OUTPUT_STALL_BACKOFF_MAX_MS 0UL
  #else
    #define TRACKER_SERIAL_OUTPUT_STALL_BACKOFF_MAX_MS 250UL
  #endif
#endif

#ifndef TRACKER_SERIAL_OUTPUT_STALE_DISCARD_MS
  #if TRACKER_BUILD_IS_SLIM
    #define TRACKER_SERIAL_OUTPUT_STALE_DISCARD_MS 0UL
  #else
    #define TRACKER_SERIAL_OUTPUT_STALE_DISCARD_MS 5000UL
  #endif
#endif

#ifndef TRACKER_BOOT_SERIAL_SETTLE_DELAY_MS
#define TRACKER_BOOT_SERIAL_SETTLE_DELAY_MS 2000UL
#endif

// Optional idle/yield experiment. Disabled by default because it can affect
// latency and FIFO freshness. Enable only for A/B tests with runtime reports.
#ifndef TRACKER_IDLE_YIELD_MODE_NONE
#define TRACKER_IDLE_YIELD_MODE_NONE 0
#endif

#ifndef TRACKER_IDLE_YIELD_MODE_DELAY0
#define TRACKER_IDLE_YIELD_MODE_DELAY0 1
#endif

#ifndef TRACKER_IDLE_YIELD_MODE_DELAY1
#define TRACKER_IDLE_YIELD_MODE_DELAY1 2
#endif

#ifndef TRACKER_ENABLE_IDLE_YIELD
#define TRACKER_ENABLE_IDLE_YIELD 0
#endif

#ifndef TRACKER_IDLE_YIELD_MODE
#define TRACKER_IDLE_YIELD_MODE TRACKER_IDLE_YIELD_MODE_DELAY0
#endif

#ifndef TRACKER_IDLE_YIELD_EVERY_N_IDLE_LOOPS
#define TRACKER_IDLE_YIELD_EVERY_N_IDLE_LOOPS 16UL
#endif

// Optional motion-triggered light sleep. This is intentionally disabled by
// default: it takes exclusive ownership of the shared INT1/FIFO pin while
// sleeping. Enable with TRACKER_ENABLE_MOTION_LIGHT_SLEEP=1 only on ESP32
// builds where PIN_LSM_INT1 is connected directly to LSM6DSV INT1.
#ifndef TRACKER_MOTION_LIGHT_SLEEP_SERVER_ABSENCE_MS
#define TRACKER_MOTION_LIGHT_SLEEP_SERVER_ABSENCE_MS 60000UL
#endif

// LSM6DSV WAKE_UP_THS register code. The driver selects the 62.5 mg/code
// resolution; 12 is about 750 mg and avoids waking from small desk noise.
#ifndef TRACKER_MOTION_LIGHT_SLEEP_WAKE_THRESHOLD
#define TRACKER_MOTION_LIGHT_SLEEP_WAKE_THRESHOLD 12u
#endif

// LSM6DSV WAKE_UP_DUR low two bits. Zero gives the shortest qualified event;
// raise it only after measuring false wakes on the actual tracker enclosure.
#ifndef TRACKER_MOTION_LIGHT_SLEEP_WAKE_DURATION
#define TRACKER_MOTION_LIGHT_SLEEP_WAKE_DURATION 0u
#endif

#ifndef TRACKER_MOTION_LIGHT_SLEEP_ACCEL_ODR
#define TRACKER_MOTION_LIGHT_SLEEP_ACCEL_ODR 60
#endif

#ifndef TRACKER_SERIAL_COMMAND_RECOVERY_SUPPRESS_MS
#define TRACKER_SERIAL_COMMAND_RECOVERY_SUPPRESS_MS 3000UL
#endif

// Live runtime section profiler. This is independent from the long runtime
// tests: enable it from serial/telnet with `perf on`, then inspect timing,
// temperature/system, Wi-Fi and SlimeVR counters with `perf status`.
#ifndef TRACKER_ENABLE_RUNTIME_PROFILER_DEFAULT_ON
#define TRACKER_ENABLE_RUNTIME_PROFILER_DEFAULT_ON 0
#endif

#ifndef TRACKER_RUNTIME_PROFILER_SLOW_LOOP_US
#define TRACKER_RUNTIME_PROFILER_SLOW_LOOP_US 5000UL
#endif
#ifndef TRACKER_RUNTIME_PROFILER_SLOW_CLI_US
#define TRACKER_RUNTIME_PROFILER_SLOW_CLI_US 1000UL
#endif
#ifndef TRACKER_RUNTIME_PROFILER_SLOW_REMOTE_US
#define TRACKER_RUNTIME_PROFILER_SLOW_REMOTE_US 2000UL
#endif
#ifndef TRACKER_RUNTIME_PROFILER_SLOW_FIFO_US
#define TRACKER_RUNTIME_PROFILER_SLOW_FIFO_US 20000UL
#endif
#ifndef TRACKER_RUNTIME_PROFILER_SLOW_BATTERY_US
#define TRACKER_RUNTIME_PROFILER_SLOW_BATTERY_US 5000UL
#endif
#ifndef TRACKER_RUNTIME_PROFILER_SLOW_NETWORK_US
#define TRACKER_RUNTIME_PROFILER_SLOW_NETWORK_US 2000UL
#endif
#ifndef TRACKER_RUNTIME_PROFILER_SLOW_TAP_US
#define TRACKER_RUNTIME_PROFILER_SLOW_TAP_US 1000UL
#endif
#ifndef TRACKER_RUNTIME_PROFILER_SLOW_LED_US
#define TRACKER_RUNTIME_PROFILER_SLOW_LED_US 1000UL
#endif
#ifndef TRACKER_RUNTIME_PROFILER_SLOW_HEARTBEAT_US
#define TRACKER_RUNTIME_PROFILER_SLOW_HEARTBEAT_US 2000UL
#endif
#ifndef TRACKER_RUNTIME_PROFILER_SLOW_IDLE_YIELD_US
#define TRACKER_RUNTIME_PROFILER_SLOW_IDLE_YIELD_US 2000UL
#endif

// Status LED runtime tuning.
#ifndef TRACKER_STATUS_LED_PIN
#define TRACKER_STATUS_LED_PIN 8
#endif

#ifndef TRACKER_STATUS_LED_ACTIVE_LOW
#define TRACKER_STATUS_LED_ACTIVE_LOW 1
#endif

#ifndef TRACKER_STATUS_LED_UPDATE_INTERVAL_MS
#define TRACKER_STATUS_LED_UPDATE_INTERVAL_MS 20
#endif

#ifndef TRACKER_STATUS_LED_NORMAL_BLINK_PERIOD_MS
#define TRACKER_STATUS_LED_NORMAL_BLINK_PERIOD_MS 10000UL
#endif

#ifndef TRACKER_STATUS_LED_NORMAL_BLINK_ON_MS
#define TRACKER_STATUS_LED_NORMAL_BLINK_ON_MS 30
#endif

#ifndef TRACKER_STATUS_LED_SHORT_BLINK_ON_MS
#define TRACKER_STATUS_LED_SHORT_BLINK_ON_MS 80
#endif

#ifndef TRACKER_STATUS_LED_SHORT_BLINK_OFF_MS
#define TRACKER_STATUS_LED_SHORT_BLINK_OFF_MS 160
#endif

#ifndef TRACKER_STATUS_LED_LONG_BLINK_ON_MS
#define TRACKER_STATUS_LED_LONG_BLINK_ON_MS 450
#endif

#ifndef TRACKER_STATUS_LED_LONG_BLINK_OFF_MS
#define TRACKER_STATUS_LED_LONG_BLINK_OFF_MS 250
#endif

#ifndef TRACKER_STATUS_LED_ERROR_BLINK_PERIOD_MS
#define TRACKER_STATUS_LED_ERROR_BLINK_PERIOD_MS 5000UL
#endif

#ifndef TRACKER_STATUS_LED_IDENTIFY_BLINK_ON_MS
#define TRACKER_STATUS_LED_IDENTIFY_BLINK_ON_MS 100
#endif

#ifndef TRACKER_STATUS_LED_IDENTIFY_BLINK_OFF_MS
#define TRACKER_STATUS_LED_IDENTIFY_BLINK_OFF_MS 100
#endif

#ifndef TRACKER_STATUS_LED_IDENTIFY_DEFAULT_MS
#define TRACKER_STATUS_LED_IDENTIFY_DEFAULT_MS 5000UL
#endif

// Tap runtime tuning.
// SlimeVR packet 13 accepts an explicit tap count byte.  Keep the transport
// range independent from the physical-gesture filter so CLI tests can send
// a single tap while runtime gesture recognition still defaults to double tap.
#ifndef TRACKER_TAP_PACKET_MIN_VALUE
#define TRACKER_TAP_PACKET_MIN_VALUE 1
#endif

#ifndef TRACKER_TAP_MIN_COUNT
#define TRACKER_TAP_MIN_COUNT 2
#endif

#ifndef TRACKER_TAP_MAX_COUNT
#define TRACKER_TAP_MAX_COUNT 10
#endif

#ifndef TRACKER_TAP_AGGREGATION_WINDOW_MS
#define TRACKER_TAP_AGGREGATION_WINDOW_MS 350
#endif

#ifndef TRACKER_TAP_SLIDING_WINDOW
#define TRACKER_TAP_SLIDING_WINDOW 1
#endif

#ifndef TRACKER_TAP_POLL_INTERVAL_MS
#define TRACKER_TAP_POLL_INTERVAL_MS 5
#endif

#ifndef TRACKER_TAP_DUPLICATE_SUPPRESS_MS
#define TRACKER_TAP_DUPLICATE_SUPPRESS_MS 35
#endif

#ifndef TRACKER_TAP_POST_SEND_LOCKOUT_MS
#define TRACKER_TAP_POST_SEND_LOCKOUT_MS 150
#endif

#ifndef TRACKER_TAP_HARDWARE_DOUBLE_TAP
#define TRACKER_TAP_HARDWARE_DOUBLE_TAP 0
#endif

#ifndef TRACKER_TAP_REGISTER_VERIFY_INTERVAL_MS
#define TRACKER_TAP_REGISTER_VERIFY_INTERVAL_MS 5000
#endif

#ifndef TRACKER_LSM6DSV_TAP_THRESHOLD
// LSM6DSV single-tap app-note examples use threshold 2 at +/-8 g (about
// 500 mg).  Runtime still requires TRACKER_TAP_MIN_COUNT taps before sending,
// so this improves physical detection without making one accidental tap fire.
#define TRACKER_LSM6DSV_TAP_THRESHOLD 2
#endif

#ifndef TRACKER_LSM6DSV_TAP_SHOCK
#define TRACKER_LSM6DSV_TAP_SHOCK 2
#endif

#ifndef TRACKER_LSM6DSV_TAP_QUIET
#define TRACKER_LSM6DSV_TAP_QUIET 2
#endif

#ifndef TRACKER_LSM6DSV_TAP_DURATION
#define TRACKER_LSM6DSV_TAP_DURATION 7
#endif

// Battery ADC runtime tuning.
#ifndef TRACKER_BATTERY_ADC_PIN
#define TRACKER_BATTERY_ADC_PIN 4
#endif

#ifndef TRACKER_BATTERY_R_TOP_OHMS
#define TRACKER_BATTERY_R_TOP_OHMS 180000.0f
#endif

#ifndef TRACKER_BATTERY_R_BOTTOM_OHMS
#define TRACKER_BATTERY_R_BOTTOM_OHMS 180000.0f
#endif

#ifndef TRACKER_BATTERY_ADC_SAMPLE_INTERVAL_MS
  #if TRACKER_BUILD_IS_DEBUG
    #define TRACKER_BATTERY_ADC_SAMPLE_INTERVAL_MS 10000UL
  #else
    #define TRACKER_BATTERY_ADC_SAMPLE_INTERVAL_MS 30000UL
  #endif
#endif

#ifndef TRACKER_BATTERY_ADC_STARTUP_SAMPLES
#define TRACKER_BATTERY_ADC_STARTUP_SAMPLES 2
#endif

// Battery ADC noise filtering. The default RC1 divider is 180 kOhm /
// 180 kOhm and has no hardware capacitor, so the ADC input is intentionally
// sampled sparsely but in a wider burst. The first few conversions are thrown
// away to let the SAR sampling path settle, then the sorted burst is trimmed
// before the runtime EMA filter is applied.
#ifndef TRACKER_BATTERY_ADC_EMA_ALPHA
#define TRACKER_BATTERY_ADC_EMA_ALPHA 0.12f
#endif

#ifndef TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT
#define TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT 64
#endif

#ifndef TRACKER_BATTERY_ADC_DISCARD_COUNT
#define TRACKER_BATTERY_ADC_DISCARD_COUNT 4
#endif

#ifndef TRACKER_BATTERY_ADC_MAX_MV
#define TRACKER_BATTERY_ADC_MAX_MV 2500
#endif

#ifndef TRACKER_BATTERY_MAX_FILTER_STEP_V
#define TRACKER_BATTERY_MAX_FILTER_STEP_V 0.30f
#endif

#ifndef TRACKER_BATTERY_VOLTAGE_SCALE
#define TRACKER_BATTERY_VOLTAGE_SCALE 1.0f
#endif

#ifndef TRACKER_BATTERY_VOLTAGE_OFFSET
#define TRACKER_BATTERY_VOLTAGE_OFFSET 0.0f
#endif

#ifndef TRACKER_BATTERY_EMPTY_VOLTAGE
#define TRACKER_BATTERY_EMPTY_VOLTAGE 3.30f
#endif

#ifndef TRACKER_BATTERY_FULL_VOLTAGE
#define TRACKER_BATTERY_FULL_VOLTAGE 4.20f
#endif

#ifndef TRACKER_BATTERY_PRESENT_MIN_VOLTAGE
#define TRACKER_BATTERY_PRESENT_MIN_VOLTAGE 1.00f
#endif

// Plausible maximum for a single Li-ion/LiPo cell at BAT+. Values above this
// are treated as ADC/glitch/out-of-range samples and do not update telemetry.
#ifndef TRACKER_BATTERY_PRESENT_MAX_VOLTAGE
#define TRACKER_BATTERY_PRESENT_MAX_VOLTAGE 4.35f
#endif
