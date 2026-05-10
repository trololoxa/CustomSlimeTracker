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

static constexpr uint8_t FIFO_WATERMARK_WORDS = 48;
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
