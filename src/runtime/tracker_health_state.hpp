#pragma once

#include <cstdint>
#include <cstring>

namespace tracker {

// Firmware-wide health/fault state shared by app, CLI and SlimeVR output.
// Keep this header-only and POD-like so it is cheap to copy into the network
// runtime only when a state transition happens.
enum class TrackerHealthFaultCode : uint8_t {
    None = 0,
    LsmInitFailed = 1,
    FifoInitFailed = 2,
    ImuNoProgress = 3,
    FifoDrainFailed = 4,
    FifoResetFailed = 5,
    FifoDiscontinuity = 6,
    SpiPlausibilityFailed = 7,
    RecoveryExhausted = 8,
    // Reserved for the magnetic liveness/rearm owner introduced by 0030.
    MagNoProgress = 9,
    MagRearmFailed = 10,
};

struct TrackerHealthSnapshot {
    bool fatalActive = false;
    bool degradedNoImu = false;
    bool recoveryActive = false;
    bool safeMode = false;
    TrackerHealthFaultCode faultCode = TrackerHealthFaultCode::None;
    TrackerHealthFaultCode lastFaultCode = TrackerHealthFaultCode::None;
    uint32_t revision = 0;
    uint32_t faultCount = 0;
    uint32_t recoveryCount = 0;
    uint32_t recoverySuccessCount = 0;
    char message[64] = {};
};

inline const char* trackerHealthFaultCodeName(TrackerHealthFaultCode code) {
    switch (code) {
        case TrackerHealthFaultCode::None: return "none";
        case TrackerHealthFaultCode::LsmInitFailed: return "lsm_init_failed";
        case TrackerHealthFaultCode::FifoInitFailed: return "fifo_init_failed";
        case TrackerHealthFaultCode::ImuNoProgress: return "imu_no_progress";
        case TrackerHealthFaultCode::FifoDrainFailed: return "fifo_drain_failed";
        case TrackerHealthFaultCode::FifoResetFailed: return "fifo_reset_failed";
        case TrackerHealthFaultCode::FifoDiscontinuity: return "fifo_discontinuity";
        case TrackerHealthFaultCode::SpiPlausibilityFailed: return "spi_plausibility_failed";
        case TrackerHealthFaultCode::RecoveryExhausted: return "recovery_exhausted";
        case TrackerHealthFaultCode::MagNoProgress: return "mag_no_progress";
        case TrackerHealthFaultCode::MagRearmFailed: return "mag_rearm_failed";
    }
    return "unknown";
}

class TrackerHealthState {
public:
    void reset() {
        fatalActive_ = false;
        degradedNoImu_ = false;
        recoveryActive_ = false;
        faultCode_ = TrackerHealthFaultCode::None;
        message_[0] = '\0';
        ++revision_;
    }

    void setSafeMode(bool enabled) {
        if (safeMode_ == enabled) return;
        safeMode_ = enabled;
        ++revision_;
    }

    void beginRecovery(TrackerHealthFaultCode code, const char* message) {
        recoveryActive_ = true;
        fatalActive_ = false;
        degradedNoImu_ = true;
        faultCode_ = code;
        lastFaultCode_ = code;
        copyMessage(message);
        ++faultCount_;
        ++recoveryCount_;
        ++revision_;
    }

    void finishRecovery() {
        recoveryActive_ = false;
        fatalActive_ = false;
        degradedNoImu_ = false;
        faultCode_ = TrackerHealthFaultCode::None;
        message_[0] = '\0';
        ++recoverySuccessCount_;
        ++revision_;
    }

    void reportRecoveryFault(TrackerHealthFaultCode code, const char* message) {
        recoveryActive_ = true;
        degradedNoImu_ = true;
        faultCode_ = code;
        lastFaultCode_ = code;
        copyMessage(message);
        ++faultCount_;
        ++revision_;
    }

    void enterDegradedNoImu(TrackerHealthFaultCode code, const char* message) {
        fatalActive_ = true;
        degradedNoImu_ = true;
        recoveryActive_ = false;
        faultCode_ = code;
        lastFaultCode_ = code;
        copyMessage(message);
        ++faultCount_;
        ++revision_;
    }

    void enterRecoveryExhausted(TrackerHealthFaultCode cause, const char* message) {
        fatalActive_ = true;
        degradedNoImu_ = true;
        recoveryActive_ = true;
        faultCode_ = TrackerHealthFaultCode::RecoveryExhausted;
        lastFaultCode_ = cause;
        copyMessage(message);
        ++faultCount_;
        ++revision_;
    }

    TrackerHealthSnapshot snapshot() const {
        TrackerHealthSnapshot out;
        out.fatalActive = fatalActive_;
        out.degradedNoImu = degradedNoImu_;
        out.recoveryActive = recoveryActive_;
        out.safeMode = safeMode_;
        out.faultCode = faultCode_;
        out.lastFaultCode = lastFaultCode_;
        out.revision = revision_;
        out.faultCount = faultCount_;
        out.recoveryCount = recoveryCount_;
        out.recoverySuccessCount = recoverySuccessCount_;
        std::memcpy(out.message, message_, sizeof(out.message));
        return out;
    }

    bool fatalActive() const { return fatalActive_; }
    bool degradedNoImu() const { return degradedNoImu_; }
    bool recoveryActive() const { return recoveryActive_; }
    bool safeMode() const { return safeMode_; }
    TrackerHealthFaultCode faultCode() const { return faultCode_; }
    uint32_t revision() const { return revision_; }
    const char* message() const { return message_; }

private:
    void copyMessage(const char* message) {
        const char* safe = message ? message : "";
        std::strncpy(message_, safe, sizeof(message_) - 1u);
        message_[sizeof(message_) - 1u] = '\0';
    }

    bool fatalActive_ = false;
    bool degradedNoImu_ = false;
    bool recoveryActive_ = false;
    bool safeMode_ = false;
    TrackerHealthFaultCode faultCode_ = TrackerHealthFaultCode::None;
    TrackerHealthFaultCode lastFaultCode_ = TrackerHealthFaultCode::None;
    uint32_t revision_ = 0;
    uint32_t faultCount_ = 0;
    uint32_t recoveryCount_ = 0;
    uint32_t recoverySuccessCount_ = 0;
    char message_[64] = {};
};

} // namespace tracker
