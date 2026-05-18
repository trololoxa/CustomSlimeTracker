#include "runtime/tap_accumulator.hpp"

namespace tracker {

bool TapAccumulator::timeReached(uint32_t nowMs, uint32_t deadlineMs) {
    return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

uint8_t TapAccumulator::normalizedMinCount() const {
    const uint8_t maxCount = normalizedMaxCount();
    if (config_.minCount == 0) return 1;
    return config_.minCount > maxCount ? maxCount : config_.minCount;
}

uint8_t TapAccumulator::normalizedMaxCount() const {
    if (config_.maxCount == 0) return 1;
    return config_.maxCount;
}

void TapAccumulator::configure(const TapAccumulatorConfig& config) {
    config_ = config;
    const uint8_t maxCount = normalizedMaxCount();
    if (config_.minCount == 0) config_.minCount = 1;
    if (config_.minCount > maxCount) config_.minCount = maxCount;
    if (config_.aggregationWindowMs == 0) config_.aggregationWindowMs = 1;
    if (status_.pendingCount > maxCount) status_.pendingCount = maxCount;
}

void TapAccumulator::reset() {
    status_ = TapAccumulatorStatus{};
}

void TapAccumulator::resetCounters() {
    const uint8_t pendingCount = status_.pendingCount;
    const uint32_t pendingFirstMs = status_.pendingFirstMs;
    const uint32_t pendingLastMs = status_.pendingLastMs;
    const uint32_t flushDeadlineMs = status_.flushDeadlineMs;
    const uint32_t lastPhysicalTapMs = status_.lastPhysicalTapMs;
    const uint32_t postSendLockoutUntilMs = status_.postSendLockoutUntilMs;
    status_ = TapAccumulatorStatus{};
    status_.pendingCount = pendingCount;
    status_.pendingFirstMs = pendingFirstMs;
    status_.pendingLastMs = pendingLastMs;
    status_.flushDeadlineMs = flushDeadlineMs;
    status_.lastPhysicalTapMs = lastPhysicalTapMs;
    status_.postSendLockoutUntilMs = postSendLockoutUntilMs;
}

TapAccumulatorAction TapAccumulator::update(uint32_t nowMs) {
    if (!pending()) return {};
    if (!timeReached(nowMs, status_.flushDeadlineMs)) return {};
    return flush(nowMs);
}

TapAccumulatorAction TapAccumulator::recordPhysicalTaps(uint8_t count, uint32_t nowMs) {
    if (count == 0) return {};

    ++status_.physicalTapEvents;
    status_.physicalTapCount += count;

    if (status_.postSendLockoutUntilMs != 0 && !timeReached(nowMs, status_.postSendLockoutUntilMs)) {
        ++status_.suppressedLockout;
        return {};
    }

    if (status_.lastPhysicalTapMs != 0 && config_.duplicateSuppressMs > 0) {
        const uint32_t sinceLast = nowMs - status_.lastPhysicalTapMs;
        if (sinceLast < config_.duplicateSuppressMs) {
            ++status_.suppressedDuplicate;
            return {};
        }
    }
    status_.lastPhysicalTapMs = nowMs;

    if (!pending()) {
        beginWindow(nowMs);
    }

    const uint8_t maxCount = normalizedMaxCount();
    const uint16_t requested = static_cast<uint16_t>(status_.pendingCount) + count;
    if (requested > maxCount) {
        status_.clampedOverflow += static_cast<uint32_t>(requested - maxCount);
        status_.pendingCount = maxCount;
    } else {
        status_.pendingCount = static_cast<uint8_t>(requested);
    }

    status_.pendingLastMs = nowMs;
    if (config_.slidingWindow) {
        status_.flushDeadlineMs = nowMs + config_.aggregationWindowMs;
    }

    if (status_.pendingCount >= maxCount) {
        return flush(nowMs);
    }
    return {};
}

TapAccumulatorAction TapAccumulator::flush(uint32_t nowMs) {
    if (!pending()) return {};

    TapAccumulatorAction action;
    const uint8_t count = status_.pendingCount;
    clearWindow();
    ++status_.windowsFlushed;

    if (count < normalizedMinCount()) {
        ++status_.suppressedBelowMin;
        return action;
    }

    action.send = true;
    action.value = count;
    status_.lastOutputValue = count;
    status_.lastOutputMs = nowMs;
    startPostSendLockout(nowMs);
    return action;
}

void TapAccumulator::beginWindow(uint32_t nowMs) {
    status_.pendingCount = 0;
    status_.pendingFirstMs = nowMs;
    status_.pendingLastMs = nowMs;
    status_.flushDeadlineMs = nowMs + config_.aggregationWindowMs;
    ++status_.windowsStarted;
}

void TapAccumulator::clearWindow() {
    status_.pendingCount = 0;
    status_.pendingFirstMs = 0;
    status_.pendingLastMs = 0;
    status_.flushDeadlineMs = 0;
}

void TapAccumulator::startPostSendLockout(uint32_t nowMs) {
    status_.postSendLockoutUntilMs = config_.postSendLockoutMs == 0 ? 0 : nowMs + config_.postSendLockoutMs;
}

} // namespace tracker
