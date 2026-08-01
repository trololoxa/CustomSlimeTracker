#pragma once

#include <cstdint>

#include "build_config/runtime_tuning.hpp"

namespace tracker {

enum class TrackingOptionalServiceClass : uint8_t {
    Short = 0,
    Console,
    Background
};

struct TrackingSlackAdmissionInput {
    bool softwareQueuePending = false;
    bool fifoUrgent = false;
    bool rotationSlackKnown = false;
    uint32_t rotationSlackUs = 0u;
};

inline uint32_t trackingOptionalServiceMinSlackUs(
    TrackingOptionalServiceClass serviceClass) {
    switch (serviceClass) {
        case TrackingOptionalServiceClass::Short:
            return TRACKER_OPTIONAL_SHORT_SERVICE_MIN_SLACK_US;
        case TrackingOptionalServiceClass::Console:
            return TRACKER_OPTIONAL_CONSOLE_SERVICE_MIN_SLACK_US;
        case TrackingOptionalServiceClass::Background:
            return TRACKER_OPTIONAL_BACKGROUND_SERVICE_MIN_SLACK_US;
    }
    return TRACKER_OPTIONAL_BACKGROUND_SERVICE_MIN_SLACK_US;
}

inline bool trackingOptionalRuntimeAdmitted(
    const TrackingSlackAdmissionInput& input,
    TrackingOptionalServiceClass serviceClass) {
    if (input.fifoUrgent || input.softwareQueuePending) {
        return false;
    }
    if (!input.rotationSlackKnown) {
        return true;
    }
    return input.rotationSlackUs >=
        trackingOptionalServiceMinSlackUs(serviceClass);
}

inline bool trackingBackgroundRuntimeAdmitted(
    const TrackingSlackAdmissionInput& input,
    bool backgroundSlotConsumed) {
    return !backgroundSlotConsumed && trackingOptionalRuntimeAdmitted(
        input, TrackingOptionalServiceClass::Background);
}

} // namespace tracker
