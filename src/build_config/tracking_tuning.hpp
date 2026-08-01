#pragma once

#include <cstdint>
#include <cstddef>

#include "build_config/build_profiles.hpp"

namespace tracker::cfg {

// FIFO / runtime loop defaults.
static constexpr uint8_t LEGACY_FIFO_WATERMARK_WORDS = 48;
static constexpr uint8_t PREVIOUS_FIFO_WATERMARK_WORDS = 12;
// A modestly larger watermark amortizes status/SPI transaction overhead while
// keeping a full batch below one 100 Hz output interval at the 960 Hz IMU ODR.
// The RAM queue and work-conserving consumer absorb larger transient bursts.
#if TRACKER_BUILD_IS_SLIM
static constexpr uint8_t FIFO_WATERMARK_WORDS = 9;
#else
static constexpr uint8_t FIFO_WATERMARK_WORDS = 18;
#endif
static constexpr uint16_t FIFO_MAX_WORDS_PER_DRAIN = 384;
static constexpr uint8_t FIFO_MAX_DRAIN_ROUNDS_PER_EVENT = 6;
// Runtime FIFO work is deliberately sliced so the app loop can service the
// 100 Hz UDP scheduler between IMU batches. SPI drain and raw/mag callbacks
// share one absolute slice budget; a small emergency slice guarantees bounded
// hardware progress near a pose deadline without allowing multi-millisecond
// drain bursts to bypass the scheduler.
// Re-check the absolute budget after a small coherent micro-batch. Twelve
// callbacks allowed a diagnostic/heavy sample path to overshoot a 3.5 ms
// slice by more than 10 ms before the scheduler could regain control.
static constexpr uint8_t FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE = 4;
static constexpr uint8_t FIFO_RUNTIME_MAX_RAW_CALLBACKS_PER_SLICE = 64;
// At 960 Hz raw IMU and 60 Hz sensor-hub ODR, one 64-sample raw slice
// spans about four magnetometer periods. Keep enough bounded budget to
// dispatch every due mag sample chronologically, with headroom for one
// delayed hardware burst, without letting mag work consume raw progress.
static constexpr uint8_t FIFO_RUNTIME_MAX_MAG_CALLBACKS_PER_SLICE = 8;
static constexpr uint32_t FIFO_RUNTIME_SLICE_BUDGET_US = 3500;
static constexpr uint32_t FIFO_RUNTIME_APP_BUDGET_US = 9000;
#if TRACKER_BUILD_IS_SLIM
static constexpr uint32_t FIFO_RUNTIME_URGENT_BUDGET_US = 18000;
static constexpr size_t FIFO_RUNTIME_RAW_QUEUE_CAPACITY = 256;
#else
// Production keeps extra RAM headroom for rare Wi-Fi/console stalls. A full
// 512-sample queue is roughly half a second at the configured IMU ODR and is
// preferable to losing orientation continuity after a single scheduling spike.
static constexpr uint32_t FIFO_RUNTIME_URGENT_BUDGET_US = 24000;
static constexpr size_t FIFO_RUNTIME_RAW_QUEUE_CAPACITY = 512;
#endif
static constexpr size_t FIFO_RUNTIME_MAG_QUEUE_CAPACITY = 64;
static constexpr size_t FIFO_RUNTIME_RAW_QUEUE_HIGH_WATER =
    (FIFO_RUNTIME_RAW_QUEUE_CAPACITY * 3u) / 8u;
// Count alone reacts too late on a 512-slot queue. Forty milliseconds of
// sensor-time span is already visible latency and must enter urgent catch-up
// even when the queue contains far fewer than the legacy depth threshold.
static constexpr uint32_t FIFO_RUNTIME_URGENT_SPAN_US = 40000u;
static_assert(FIFO_RUNTIME_RAW_QUEUE_HIGH_WATER < FIFO_RUNTIME_RAW_QUEUE_CAPACITY,
              "FIFO urgent threshold must leave queue headroom");

// Gyro prediction remains at the full IMU ODR. Gravity correction and prepared
// motion snapshots run at a still-high sub-rate to remove redundant trigonometry
// and quaternion/world-vector work without reducing observable tracking bandwidth.
static constexpr uint8_t AHRS_ACCEL_CORRECTION_DIVISOR = 4;
static constexpr uint32_t PREPARED_OUTPUT_MIN_INTERVAL_US = 4000;
static constexpr uint8_t FIFO_MAX_WAITING_SAMPLES_BEFORE_FALLBACK = 32;

static constexpr size_t FIFO_RAW_BUFFER_CAPACITY = 160;
static constexpr size_t MAG_RAW_BUFFER_CAPACITY = 48;

static constexpr float MAG_HUB_ODR_HZ = 60.0f;
static constexpr float MAG_HUB_PERIOD_US = 1000000.0f / MAG_HUB_ODR_HZ;

static constexpr uint32_t FIFO_WAIT_TIMEOUT_MS = 1000UL;
static constexpr uint32_t FIFO_NONBLOCKING_STATUS_POLL_INTERVAL_US = 2000UL;
static constexpr uint32_t HEARTBEAT_PERIOD_MS = 30000UL;

// Output defaults.
#if TRACKER_BUILD_IS_SLIM
static constexpr uint16_t OUTPUT_RATE_HZ = 125;
#else
static constexpr uint16_t OUTPUT_RATE_HZ = 100;
#endif
#if TRACKER_BUILD_IS_SLIM
static constexpr uint16_t OUTPUT_RATE_HZ_MAX = 125;
#else
static constexpr uint16_t OUTPUT_RATE_HZ_MAX = 1000;
#endif

// Device calibration policy.
static constexpr bool USE_ACCEL_6POS_CAL = true;

// Intentionally no device-specific accel calibration defaults here. A new board
// must start with accelCal.valid=false unless a unit-specific calibration is
// loaded from NVS or imported explicitly by a setup/cal command.
static constexpr bool HAS_DEVICE_ACCEL_FACTORY_CAL = false;

} // namespace tracker::cfg
