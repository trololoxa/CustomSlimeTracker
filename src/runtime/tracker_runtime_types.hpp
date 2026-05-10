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
};

// Latest orientation snapshot for future non-blocking output transports
// (WiFi/UDP/SlimeVR). The tracking path owns writes; an output scheduler can
// copy this snapshot and send it at its own rate without touching AHRS/FIFO.
struct TrackerPreparedOutputSnapshot {
    bool valid = false;
    uint32_t sequence = 0;
    uint32_t runtimeSample = 0;
    uint32_t ahrsUpdateCount = 0;
    uint64_t timestampUs = 0;
    Quat q = Quat::identity();
    uint32_t qualityFlags = 0;
    float confidence = 0.0f;
};

} // namespace tracker
