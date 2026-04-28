#pragma once

#include <cstdint>
#include "core/math.hpp"

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

inline Vec3 accelBiasG() {
    return Vec3(0.00214949f, 0.00605807f, 0.00125885f);
}

inline Mat3 accelScale() {
    return Mat3::diagonal(1.00130630f, 1.00026011f, 1.00308013f);
}

} // namespace tracker::cfg