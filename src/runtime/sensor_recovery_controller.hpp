#pragma once

#include <cstdint>

#include "runtime/tracker_health_state.hpp"

namespace tracker {

enum class SensorRecoveryState : uint8_t {
    Idle = 0,
    ResetPending,
    ReinitPending,
    Backoff,
    ExhaustedBackoff,
    AwaitingProgress,
};

enum class SensorRecoveryAction : uint8_t {
    None = 0,
    TransactionalFifoReset,
    FullSensorReinit,
};

inline bool sensorRecoveryReinitializesDevice(SensorRecoveryAction action) {
    return action == SensorRecoveryAction::FullSensorReinit;
}

inline const char* sensorRecoveryStateName(SensorRecoveryState state) {
    switch (state) {
        case SensorRecoveryState::Idle: return "idle";
        case SensorRecoveryState::ResetPending: return "reset_pending";
        case SensorRecoveryState::ReinitPending: return "reinit_pending";
        case SensorRecoveryState::Backoff: return "backoff";
        case SensorRecoveryState::ExhaustedBackoff: return "exhausted_backoff";
        case SensorRecoveryState::AwaitingProgress: return "awaiting_progress";
    }
    return "unknown";
}

struct SensorRecoverySnapshot {
    SensorRecoveryState state = SensorRecoveryState::Idle;
    TrackerHealthFaultCode faultCode = TrackerHealthFaultCode::None;
    uint32_t reasonFlags = 0u;
    uint32_t requestCount = 0u;
    uint32_t successCount = 0u;
    uint32_t failureCount = 0u;
    uint16_t attemptsInEpisode = 0u;
    uint32_t nextAttemptMs = 0u;
    bool exhausted = false;
};

// Owns retry order and deadlines only. Hardware calls remain in TrackerApp so
// this state machine is deterministic and host-testable.
class SensorRecoveryController {
public:
    bool request(TrackerHealthFaultCode code, uint32_t reasonFlags, uint32_t nowMs) {
        ++requestCount_;
        reasonFlags_ |= reasonFlags;
        if (active()) {
            if (awaitingProgress()) failProgress(nowMs);
            // Preserve the earliest/root fault for the current episode. A later
            // failure is still represented by the typed health transition.
            return false;
        }
        faultCode_ = code;
        attemptsInEpisode_ = 0u;
        nextAttemptMs_ = nowMs;
        exhausted_ = false;
        state_ = SensorRecoveryState::ResetPending;
        return true;
    }

    SensorRecoveryAction poll(uint32_t nowMs) {
        if (!active() || !deadlineReached(nowMs, nextAttemptMs_)) {
            return SensorRecoveryAction::None;
        }
        if (awaitingProgress()) {
            failProgress(nowMs);
            return SensorRecoveryAction::None;
        }
        if (state_ == SensorRecoveryState::ResetPending ||
            (state_ == SensorRecoveryState::Backoff && attemptsInEpisode_ < kFifoResetAttempts)) {
            state_ = SensorRecoveryState::ResetPending;
            return SensorRecoveryAction::TransactionalFifoReset;
        }
        state_ = exhausted_ ? SensorRecoveryState::ExhaustedBackoff
                            : SensorRecoveryState::ReinitPending;
        return SensorRecoveryAction::FullSensorReinit;
    }

    void completeAttempt(SensorRecoveryAction action, bool success, uint32_t nowMs,
                         uint32_t progressTimeoutMs = 2000u) {
        if (action == SensorRecoveryAction::None || !active()) return;
        if (success) {
            // Register read-back is preparation, not proof of a live source.
            state_ = SensorRecoveryState::AwaitingProgress;
            nextAttemptMs_ = nowMs + ((progressTimeoutMs >= 1u &&
                progressTimeoutMs <= 10000u) ? progressTimeoutMs : 2000u);
            return;
        }

        recordFailure(nowMs);
    }

    bool active() const { return state_ != SensorRecoveryState::Idle; }
    bool awaitingProgress() const { return state_ == SensorRecoveryState::AwaitingProgress; }
    bool blocksSampling() const { return active() && !awaitingProgress(); }
    void failProgress(uint32_t nowMs) {
        if (awaitingProgress()) {
            recordFailure(nowMs);
        }
    }
    bool confirmProgress(uint32_t nowMs) {
        if (!awaitingProgress() || deadlineReached(nowMs, nextAttemptMs_)) return false;
        ++successCount_;
        state_ = SensorRecoveryState::Idle;
        faultCode_ = TrackerHealthFaultCode::None;
        reasonFlags_ = 0u;
        attemptsInEpisode_ = 0u;
        nextAttemptMs_ = 0u;
        exhausted_ = false;
        return true;
    }
    bool exhausted() const { return exhausted_; }
    TrackerHealthFaultCode faultCode() const { return faultCode_; }
    uint32_t reasonFlags() const { return reasonFlags_; }

    SensorRecoverySnapshot snapshot() const {
        SensorRecoverySnapshot out;
        out.state = state_;
        out.faultCode = faultCode_;
        out.reasonFlags = reasonFlags_;
        out.requestCount = requestCount_;
        out.successCount = successCount_;
        out.failureCount = failureCount_;
        out.attemptsInEpisode = attemptsInEpisode_;
        out.nextAttemptMs = nextAttemptMs_;
        out.exhausted = exhausted_;
        return out;
    }

private:
    void recordFailure(uint32_t nowMs) {
        ++failureCount_;
        if (attemptsInEpisode_ != 0xffffu) ++attemptsInEpisode_;
        if (attemptsInEpisode_ >= kAttemptsBeforeExhausted) {
            exhausted_ = true;
            state_ = SensorRecoveryState::ExhaustedBackoff;
            nextAttemptMs_ = nowMs + kExhaustedRetryMs;
            return;
        }

        state_ = SensorRecoveryState::Backoff;
        nextAttemptMs_ = nowMs + retryDelayMs(attemptsInEpisode_);
    }

    static constexpr uint16_t kFifoResetAttempts = 2u;
    static constexpr uint16_t kAttemptsBeforeExhausted = 5u;
    static constexpr uint32_t kExhaustedRetryMs = 30000u;

    static bool deadlineReached(uint32_t nowMs, uint32_t deadlineMs) {
        return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
    }

    static uint32_t retryDelayMs(uint16_t attempt) {
        switch (attempt) {
            case 0u: return 0u;
            case 1u: return 50u;
            case 2u: return 250u;
            case 3u: return 1000u;
            default: return 5000u;
        }
    }

    SensorRecoveryState state_ = SensorRecoveryState::Idle;
    TrackerHealthFaultCode faultCode_ = TrackerHealthFaultCode::None;
    uint32_t reasonFlags_ = 0u;
    uint32_t requestCount_ = 0u;
    uint32_t successCount_ = 0u;
    uint32_t failureCount_ = 0u;
    uint16_t attemptsInEpisode_ = 0u;
    uint32_t nextAttemptMs_ = 0u;
    bool exhausted_ = false;
};

} // namespace tracker
