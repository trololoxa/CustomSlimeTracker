#include "runtime/motion_light_sleep_controller.hpp"

namespace tracker {

bool MotionLightSleepController::shouldEnter(bool serverFound, uint32_t nowMs) {
    if (serverFound || serverAbsenceTimeoutMs_ == 0u) {
        reset();
        return false;
    }

    if (!serverAbsenceActive_) {
        serverAbsenceActive_ = true;
        serverAbsentSinceMs_ = nowMs;
        return false;
    }

    return static_cast<uint32_t>(nowMs - serverAbsentSinceMs_) >= serverAbsenceTimeoutMs_;
}

void MotionLightSleepController::reset() {
    serverAbsentSinceMs_ = 0;
    serverAbsenceActive_ = false;
}

} // namespace tracker
