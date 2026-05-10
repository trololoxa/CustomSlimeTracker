#pragma once

#include <cstdint>

#include "connection/lsm6dsv_fifo.hpp"
#include "core/math.hpp"

namespace tracker {

struct MagRuntimeState {
    bool hubInitialized = false;
    bool runtimeEnabled = false;
    bool fifoArmed = false;
    bool lastInitOk = false;
    uint32_t samples = 0;
    uint32_t queuePops = 0;
    uint32_t lastSampleMs = 0;
    uint32_t lastEnableMs = 0;
    uint32_t enableFailures = 0;
    uint32_t nacksSeen = 0;
    Lsm6dsvFifoReader::MagRawSample lastRaw;
    float lastNormRaw = 0.0f;
};

struct MagHeadingReferenceState {
    bool valid = false;
    float worldYawRad = 0.0f;
    uint32_t setMs = 0;
    uint32_t magSeq = 0;
    uint64_t magTimestampUs = 0;

    void clear() {
        valid = false;
        worldYawRad = 0.0f;
        setMs = 0;
        magSeq = 0;
        magTimestampUs = 0;
    }

    float worldYawDeg() const {
        return worldYawRad * MATH_RAD_TO_DEG;
    }
};

struct MagHeadingAutoReferenceState {
    bool enabled = true;
    bool done = false;

    uint32_t stableSinceMs = 0;
    uint32_t setCount = 0;
    uint32_t lastSetMs = 0;
    uint32_t lastRejectFlags = 0;

    float lastGyroNormDps = 0.0f;
    float lastAccelTrust = 0.0f;
    float lastHorizontalTrust = 0.0f;

    void resetCandidate() {
        stableSinceMs = 0;
        lastRejectFlags = 0;
        lastGyroNormDps = 0.0f;
        lastAccelTrust = 0.0f;
        lastHorizontalTrust = 0.0f;
    }

    void resetAll() {
        enabled = true;
        done = false;
        stableSinceMs = 0;
        setCount = 0;
        lastSetMs = 0;
        lastRejectFlags = 0;
        lastGyroNormDps = 0.0f;
        lastAccelTrust = 0.0f;
        lastHorizontalTrust = 0.0f;
    }
};

} // namespace tracker
