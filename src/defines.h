#pragma once

#include <cstdint>
#include <cstddef>

// ============================================================
// Runtime feature gates
// ============================================================
// Edit these project-local defines for production/WiFi builds.
// Keep them as preprocessor macros: runtime code uses them in #if blocks, so
// constexpr values would not disable code at compile time.
//
// Suggested WiFi/production profile:
//   TRACKER_ENABLE_SERIAL_STREAM = 0
//   TRACKER_ENABLE_MACHINE_LOG = 0
//   TRACKER_ENABLE_STATIC_TEST = 0
//   TRACKER_ENABLE_BOOT_HEARTBEAT = 0
//   TRACKER_CLI_BYTES_PER_LOOP = 16

#ifndef TRACKER_ENABLE_SERIAL_CLI
#define TRACKER_ENABLE_SERIAL_CLI 1
#endif

#ifndef TRACKER_CLI_BYTES_PER_LOOP
#define TRACKER_CLI_BYTES_PER_LOOP 32
#endif

#ifndef TRACKER_ENABLE_SERIAL_STREAM
#define TRACKER_ENABLE_SERIAL_STREAM 1
#endif

#ifndef TRACKER_ENABLE_MACHINE_LOG
#define TRACKER_ENABLE_MACHINE_LOG 1
#endif

#ifndef TRACKER_ENABLE_STATIC_TEST
#define TRACKER_ENABLE_STATIC_TEST 1
#endif

#ifndef TRACKER_ENABLE_BOOT_HEARTBEAT
#define TRACKER_ENABLE_BOOT_HEARTBEAT 1
#endif

#ifndef TRACKER_ENABLE_PREPARED_OUTPUT_SNAPSHOT
#define TRACKER_ENABLE_PREPARED_OUTPUT_SNAPSHOT 1
#endif

// ============================================================
// Network / SlimeVR transport defaults
// ============================================================
// TRACKER_WIFI_TX_POWER intentionally references the ESP32 Arduino WiFi enum
// token, but is kept as a macro so board profiles can override it from
// platformio build_flags or by editing defines.h. It is only consumed by the
// ESP32 Wi-Fi adapter implementation after including WiFi.h.
#ifndef TRACKER_WIFI_APPLY_TX_POWER_AFTER_BEGIN
#define TRACKER_WIFI_APPLY_TX_POWER_AFTER_BEGIN 1
#endif

#ifndef TRACKER_WIFI_TX_POWER
#define TRACKER_WIFI_TX_POWER WIFI_POWER_8_5dBm
#endif

#ifndef TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS
#define TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS 5000UL
#endif

#ifndef TRACKER_SLIMEVR_ENABLE_SIGNAL_TELEMETRY
#define TRACKER_SLIMEVR_ENABLE_SIGNAL_TELEMETRY 1
#endif

#ifndef TRACKER_SLIMEVR_ENABLE_TEMPERATURE_TELEMETRY
#define TRACKER_SLIMEVR_ENABLE_TEMPERATURE_TELEMETRY 1
#endif

#ifndef TRACKER_SLIMEVR_SERVER_SILENCE_TIMEOUT_MS
#define TRACKER_SLIMEVR_SERVER_SILENCE_TIMEOUT_MS 15000UL
#endif

#ifndef TRACKER_SLIMEVR_SEND_FAILURE_REOPEN_THRESHOLD
#define TRACKER_SLIMEVR_SEND_FAILURE_REOPEN_THRESHOLD 5UL
#endif

#ifndef TRACKER_ENABLE_RUNTIME_TEST
#define TRACKER_ENABLE_RUNTIME_TEST 1
#endif


#ifndef TRACKER_ENABLE_STATUS_LED
#define TRACKER_ENABLE_STATUS_LED 1
#endif

// ESP32-C3 SuperMini boards commonly expose the user/on-board blue LED on
// GPIO8. Most standard blue-LED variants drive it active-low; override these
// macros for clone boards or ESP32-C3 SuperMini Plus RGB/WS2812 variants.
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

#ifndef TRACKER_ENABLE_TAP_RUNTIME
#define TRACKER_ENABLE_TAP_RUNTIME 1
#endif

// LSM6DSV tap recognition emits one physical tap event. Firmware aggregates
// physical taps into a SlimeVR Tap packet value so the server can distinguish
// 2..10 tap gestures. Keep hardware double-tap off by default; otherwise the
// sensor can merge two physical taps before the firmware counter sees them.
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

// Backward-compatible no-op/alias for old build flags. The accumulator min
// count controls whether single taps are sent; post-send lockout replaces the
// old cooldown path.
#ifndef TRACKER_TAP_SEND_SINGLE
#define TRACKER_TAP_SEND_SINGLE 0
#endif

#ifndef TRACKER_TAP_COOLDOWN_MS
#define TRACKER_TAP_COOLDOWN_MS TRACKER_TAP_POST_SEND_LOCKOUT_MS
#endif

#ifndef TRACKER_LSM6DSV_TAP_THRESHOLD
#define TRACKER_LSM6DSV_TAP_THRESHOLD 3
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

namespace tracker::cfg {

// ============================================================
// Board / transport defaults
// ============================================================

static constexpr int PIN_LSM_SCK  = 3;
static constexpr int PIN_LSM_MISO = 0;
static constexpr int PIN_LSM_MOSI = 2;
static constexpr int PIN_LSM_CS   = 1;
static constexpr int PIN_LSM_INT1 = 10;

static constexpr uint32_t SERIAL_BAUD = 921600UL;
static constexpr uint32_t SPI_HZ = 4000000UL;
static constexpr uint32_t LEGACY_SPI_HZ = 1000000UL;
static constexpr uint32_t MIN_SPI_HZ = 100000UL;
static constexpr uint32_t MAX_SPI_HZ = 10000000UL;

// SPI_MODE0 is 0 in Arduino SPI. Keep the header independent from SPI.h so it
// can be included by config/test code without pulling in the transport layer.
static constexpr uint8_t SPI_MODE = 0;

// ============================================================
// FIFO / runtime loop defaults
// ============================================================

static constexpr uint8_t LEGACY_FIFO_WATERMARK_WORDS = 48;
// SlimeVR UDP output is paced from prepared quaternion snapshots. With the
// original 48-word FIFO watermark, snapshots were only refreshed at about
// 23 Hz because samples arrived in large FIFO batches. A 12-word watermark
// keeps FIFO/AHRS latency low enough for a real ~100 Hz RotationData stream
// while the runtime test still shows no drops/recovery on the target board.
static constexpr uint8_t FIFO_WATERMARK_WORDS = 12;
static constexpr uint16_t FIFO_MAX_WORDS_PER_DRAIN = 384;
static constexpr uint8_t FIFO_MAX_DRAIN_ROUNDS_PER_EVENT = 6;
static constexpr uint8_t FIFO_MAX_WAITING_SAMPLES_BEFORE_FALLBACK = 32;

static constexpr size_t FIFO_RAW_BUFFER_CAPACITY = 160;
static constexpr size_t MAG_RAW_BUFFER_CAPACITY = 48;

static constexpr float MAG_HUB_ODR_HZ = 60.0f;
static constexpr float MAG_HUB_PERIOD_US = 1000000.0f / MAG_HUB_ODR_HZ;

static constexpr uint32_t FIFO_WAIT_TIMEOUT_MS = 1000UL;
static constexpr uint32_t FIFO_NONBLOCKING_STATUS_POLL_INTERVAL_US = 2000UL;
static constexpr uint32_t HEARTBEAT_PERIOD_MS = 30000UL;

// ============================================================
// Output defaults
// ============================================================

static constexpr uint16_t OUTPUT_RATE_HZ = 100;
static constexpr uint16_t OUTPUT_RATE_HZ_MAX = 1000;

// ============================================================
// Device calibration policy
// ============================================================

static constexpr bool USE_ACCEL_6POS_CAL = true;

// Intentionally no device-specific accel calibration defaults here.
// A new board must start with accelCal.valid=false unless a unit-specific
// calibration is loaded from NVS or imported explicitly by a setup/cal command.
static constexpr bool HAS_DEVICE_ACCEL_FACTORY_CAL = false;

} // namespace tracker::cfg
