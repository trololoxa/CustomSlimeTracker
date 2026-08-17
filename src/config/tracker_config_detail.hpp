#pragma once

#include <cstddef>
#include <cstdint>
#include <cmath>

#include "defines.h"
#include "core/math.hpp"

namespace tracker {

namespace tracker_config_detail {

static constexpr uint32_t CONFIG_MAGIC = 0x54364453UL; // 'T6DS' little-endian-ish
static constexpr uint16_t CONFIG_VERSION = 2;
static constexpr const char* NVS_NAMESPACE = "tracker";
static constexpr const char* NVS_KEY_CONFIG = "cfg";

static constexpr uint16_t SCHEMA_HARDWARE_VERSION = 1;
static constexpr uint16_t SCHEMA_IMU_VERSION = 1;
static constexpr uint16_t SCHEMA_FIFO_VERSION = 1;
static constexpr uint16_t SCHEMA_AHRS_VERSION = 2;
static constexpr uint16_t SCHEMA_GYRO_CAL_VERSION = 2;
static constexpr uint16_t SCHEMA_ACCEL_CAL_VERSION = 2;
static constexpr uint16_t SCHEMA_MAG_CAL_VERSION = 2;
static constexpr uint16_t SCHEMA_MAG_YAW_VERSION = 2;
static constexpr uint16_t SCHEMA_QUALITY_VERSION = 1;
static constexpr uint16_t SCHEMA_OUTPUT_VERSION = 2;
static constexpr uint16_t SCHEMA_FRAME_VERSION = 1;

// TrackerOutputConfig::packetFormat was a deprecated byte that sanitization
// previously forced to zero. Reuse it without changing the 756-byte blob. The
// high marker bit distinguishes the new persisted SlimeVR motion-mode meaning
// from every historical packetFormat value, which migrates to quaternion-only.
static constexpr uint8_t OUTPUT_PACKET_MODE_MARKER = 0x80u;
static constexpr uint8_t OUTPUT_PACKET_MODE_VALUE_MASK = 0x03u;
static constexpr uint8_t OUTPUT_PACKET_MODE_ALLOWED_MASK =
    OUTPUT_PACKET_MODE_MARKER | OUTPUT_PACKET_MODE_VALUE_MASK;

// Reuse TrackerAhrsRuntimeConfigPersisted::reserved without changing the
// persistent config blob size. Bit 0 means the production runtime should start
// the cautious stationary gyro-bias estimator once base gyro/temp/accel
// calibration is valid. The estimator trim itself is never persisted.
static constexpr uint8_t AHRS_RUNTIME_FLAG_RUNTIME_BIAS_ENABLED = 0x01;

static constexpr uint32_t DEFAULT_SPI_HZ = cfg::SPI_HZ;
static constexpr uint32_t LEGACY_SPI_HZ = cfg::LEGACY_SPI_HZ;
static constexpr uint32_t PREVIOUS_SPI_HZ = cfg::PREVIOUS_SPI_HZ;
static constexpr uint32_t SPI_FALLBACK_HZ = cfg::SPI_FALLBACK_HZ;
static constexpr uint32_t MIN_SPI_HZ = cfg::MIN_SPI_HZ;
static constexpr uint32_t MAX_SPI_HZ = cfg::MAX_SPI_HZ;

inline uint32_t fnv1a32(const uint8_t* data, size_t len) {
    uint32_t h = 2166136261UL;
    for (size_t i = 0; i < len; ++i) {
        h ^= static_cast<uint32_t>(data[i]);
        h *= 16777619UL;
    }
    return h;
}

inline bool finiteFloat(float x) {
    return std::isfinite(x);
}

inline bool finiteVec3(const Vec3& v) {
    return finiteFloat(v.x) && finiteFloat(v.y) && finiteFloat(v.z);
}

inline bool finiteQuat(const Quat& q) {
    return finiteFloat(q.w) && finiteFloat(q.x) && finiteFloat(q.y) && finiteFloat(q.z);
}

inline bool finiteMat3(const Mat3& m) {
    for (uint8_t r = 0; r < 3; ++r) {
        for (uint8_t c = 0; c < 3; ++c) {
            if (!finiteFloat(m.m[r][c])) return false;
        }
    }
    return true;
}

inline float clampFloat(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

} // namespace tracker_config_detail

} // namespace tracker
