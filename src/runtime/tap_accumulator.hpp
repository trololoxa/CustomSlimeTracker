#pragma once

#include <cstdint>

#include "defines.h"

namespace tracker {

struct TapAccumulatorConfig {
    uint8_t minCount = TRACKER_TAP_MIN_COUNT;
    uint8_t maxCount = TRACKER_TAP_MAX_COUNT;
    uint16_t aggregationWindowMs = TRACKER_TAP_AGGREGATION_WINDOW_MS;
    bool slidingWindow = TRACKER_TAP_SLIDING_WINDOW != 0;
    uint16_t duplicateSuppressMs = TRACKER_TAP_DUPLICATE_SUPPRESS_MS;
    uint16_t postSendLockoutMs = TRACKER_TAP_POST_SEND_LOCKOUT_MS;
};

struct TapAccumulatorAction {
    bool send = false;
    uint8_t value = 0;
};

struct TapAccumulatorStatus {
    uint8_t pendingCount = 0;
    uint8_t lastOutputValue = 0;
    uint32_t pendingFirstMs = 0;
    uint32_t pendingLastMs = 0;
    uint32_t flushDeadlineMs = 0;
    uint32_t lastPhysicalTapMs = 0;
    uint32_t postSendLockoutUntilMs = 0;
    uint32_t lastOutputMs = 0;

    uint32_t physicalTapEvents = 0;
    uint32_t physicalTapCount = 0;
    uint32_t windowsStarted = 0;
    uint32_t windowsFlushed = 0;
    uint32_t suppressedBelowMin = 0;
    uint32_t suppressedDuplicate = 0;
    uint32_t suppressedLockout = 0;
    uint32_t clampedOverflow = 0;
};

class TapAccumulator {
public:
    void configure(const TapAccumulatorConfig& config);
    void reset();
    void resetCounters();

    TapAccumulatorAction update(uint32_t nowMs);
    TapAccumulatorAction recordPhysicalTaps(uint8_t count, uint32_t nowMs);
    TapAccumulatorAction flush(uint32_t nowMs);

    TapAccumulatorStatus status() const { return status_; }
    TapAccumulatorConfig config() const { return config_; }
    bool pending() const { return status_.pendingCount != 0; }

private:
    static bool timeReached(uint32_t nowMs, uint32_t deadlineMs);
    uint8_t normalizedMinCount() const;
    uint8_t normalizedMaxCount() const;
    void beginWindow(uint32_t nowMs);
    void clearWindow();
    void startPostSendLockout(uint32_t nowMs);

    TapAccumulatorConfig config_;
    TapAccumulatorStatus status_;
};

} // namespace tracker
