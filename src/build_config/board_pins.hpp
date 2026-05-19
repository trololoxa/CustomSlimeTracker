#pragma once

#include <cstdint>
#include <cstddef>

namespace tracker::cfg {

// Board / transport defaults.
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

// SPI_MODE0 is 0 in Arduino SPI. Keep this header independent from SPI.h so it
// can be included by config/test code without pulling in the transport layer.
static constexpr uint8_t SPI_MODE = 0;

} // namespace tracker::cfg
