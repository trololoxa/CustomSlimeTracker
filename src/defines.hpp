#pragma once

#include <cstdint>

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
