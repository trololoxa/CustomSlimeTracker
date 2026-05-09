#pragma once

#include <cstdint>

// ============================================================
// Runtime feature gates
// ============================================================
// Edit these project-local defines for production/WiFi builds.
// Keep them as preprocessor macros: main.cpp uses them in #if blocks, so
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

static constexpr int PIN_LSM_SCK  = 3;
static constexpr int PIN_LSM_MISO = 0;
static constexpr int PIN_LSM_MOSI = 2;
static constexpr int PIN_LSM_CS   = 1;
static constexpr int PIN_LSM_INT1 = 10;

static constexpr uint32_t SERIAL_BAUD = 921600;
static constexpr uint32_t SPI_HZ = 1000000;

static constexpr uint8_t FIFO_WATERMARK_WORDS = 48;
static constexpr uint16_t FIFO_MAX_WORDS_PER_DRAIN = 384;

static constexpr bool USE_ACCEL_6POS_CAL = true;

// Intentionally no device-specific accel calibration defaults here.
// A new board must start with accelCal.valid=false unless a unit-specific
// calibration is loaded from NVS or imported explicitly by a setup/cal command.
static constexpr bool HAS_DEVICE_ACCEL_FACTORY_CAL = false;

} // namespace tracker::cfg
