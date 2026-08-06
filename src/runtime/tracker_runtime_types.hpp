#pragma once

#include <cstdint>

#include "core/math.hpp"

namespace tracker {

struct TrackerPerfCounters {
    uint32_t fifoProcessCalls = 0;
    uint32_t fifoProcessMaxUs = 0;
    uint64_t fifoProcessSumUs = 0;

    uint32_t sampleProcessCalls = 0;
    uint32_t sampleProcessMaxUs = 0;
    uint64_t sampleProcessSumUs = 0;

    uint32_t fifoEmptyPolls = 0;
    uint32_t fifoIrqEvents = 0;
    uint32_t fifoFallbackStatusPolls = 0;
    uint32_t fifoFallbackEvents = 0;

    uint32_t resetMs = 0;

    void reset(uint32_t nowMs) {
        fifoProcessCalls = 0;
        fifoProcessMaxUs = 0;
        fifoProcessSumUs = 0;
        sampleProcessCalls = 0;
        sampleProcessMaxUs = 0;
        sampleProcessSumUs = 0;
        fifoEmptyPolls = 0;
        fifoIrqEvents = 0;
        fifoFallbackStatusPolls = 0;
        fifoFallbackEvents = 0;
        resetMs = nowMs;
    }
};


struct MachineLogCounters {
    uint32_t q = 0;
    uint32_t cal = 0;
    uint32_t fifo = 0;
    uint32_t mag = 0;
    uint32_t yaw = 0;
    uint32_t state = 0;
    uint32_t bias = 0;
    uint32_t biasUpdate = 0;
    uint32_t network = 0;
    uint32_t backpressureDrop = 0;
    uint32_t producerQueueDrop = 0;
    uint32_t serviceDeferral = 0;
    uint32_t shutdownDrop = 0;
    uint32_t disconnectAbort = 0;
};

namespace prepared_output_motion_flags {
static constexpr uint8_t NONE = 0u;
static constexpr uint8_t CONFIGURATION_NOT_READY = 1u << 0;
static constexpr uint8_t ACCEL_COMPONENT_MISSING = 1u << 1;
static constexpr uint8_t PAIR_COHERENCY_DEGRADED = 1u << 2;
static constexpr uint8_t ACCEL_SATURATED = 1u << 3;
static constexpr uint8_t NON_FINITE = 1u << 4;
}

// Latest timestamp-coherent motion snapshot for non-blocking output
// transports. The tracking path owns writes; an output scheduler can copy
// orientation and acceleration from one accepted IMU sample without touching
// AHRS/FIFO or recomputing frame transforms.
struct TrackerPreparedOutputSnapshot {
    bool valid = false;
    bool linearAccelerationValid = false;
    uint8_t linearAccelerationInvalidFlags = prepared_output_motion_flags::NONE;
    uint32_t sequence = 0;
    uint32_t runtimeSample = 0;
    uint32_t ahrsUpdateCount = 0;
    uint64_t timestampUs = 0;
    // MCU-clock publication time. Unlike timestampUs, this shares the same
    // micros() epoch as the network scheduler and is safe for age metrics.
    uint32_t publishedAtMcuUs = 0;
    // Exact wait in the application raw-sample queue before this sample was
    // processed. This intentionally excludes unknown hardware-FIFO residence.
    uint32_t softwareQueueAgeUs = 0;
    // Hamilton q_world_from_device: rotates device-frame vectors into the
    // AHRS world frame. The SlimeVR protocol adapter preserves this local basis.
    Quat q = Quat::identity();
    // Gravity-removed acceleration in the same device frame as q, in g.
    Vec3 linearAccelerationDeviceG = Vec3::zero();
    uint32_t qualityFlags = 0;
    float confidence = 0.0f;
};

} // namespace tracker
