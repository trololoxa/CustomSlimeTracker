#pragma once

#include <cstdint>
#include <cstddef>

namespace tracker::cfg {

// FIFO / runtime loop defaults.
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

// Output defaults.
static constexpr uint16_t OUTPUT_RATE_HZ = 100;
static constexpr uint16_t OUTPUT_RATE_HZ_MAX = 1000;

// Device calibration policy.
static constexpr bool USE_ACCEL_6POS_CAL = true;

// Intentionally no device-specific accel calibration defaults here. A new board
// must start with accelCal.valid=false unless a unit-specific calibration is
// loaded from NVS or imported explicitly by a setup/cal command.
static constexpr bool HAS_DEVICE_ACCEL_FACTORY_CAL = false;

} // namespace tracker::cfg
