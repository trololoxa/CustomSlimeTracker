#pragma once

#include <cstdint>

namespace tracker {

// Pure policy state for the optional motion-light-sleep feature. Hardware is
// deliberately kept out of this class so the timeout semantics can be tested
// natively, including uint32_t millis() wrap-around.
class MotionLightSleepController {
public:
    explicit MotionLightSleepController(uint32_t serverAbsenceTimeoutMs = 60000UL)
        : serverAbsenceTimeoutMs_(serverAbsenceTimeoutMs) {}

    void setServerAbsenceTimeoutMs(uint32_t timeoutMs) {
        serverAbsenceTimeoutMs_ = timeoutMs;
        reset();
    }

    uint32_t serverAbsenceTimeoutMs() const { return serverAbsenceTimeoutMs_; }

    // Returns true exactly when the server has been continuously absent for
    // the configured timeout. Call noteSleepAttempted() before the blocking
    // light-sleep call so a failed/instant wake begins a fresh absence window.
    bool shouldEnter(bool serverFound, uint32_t nowMs);

    void noteSleepAttempted() { reset(); }

    void reset();

    bool serverAbsenceActive() const { return serverAbsenceActive_; }
    uint32_t serverAbsentSinceMs() const { return serverAbsentSinceMs_; }

private:
    uint32_t serverAbsenceTimeoutMs_ = 60000UL;
    uint32_t serverAbsentSinceMs_ = 0;
    bool serverAbsenceActive_ = false;
};

} // namespace tracker
