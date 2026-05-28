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
};

struct TrackerHealthSnapshot {
    bool fatalActive = false;
    bool degradedNoImu = false;
    TrackerHealthFaultCode faultCode = TrackerHealthFaultCode::None;
    uint32_t revision = 0;
    char message[64] = {};
};

inline const char* trackerHealthFaultCodeName(TrackerHealthFaultCode code) {
    switch (code) {
        case TrackerHealthFaultCode::None: return "none";
        case TrackerHealthFaultCode::LsmInitFailed: return "lsm_init_failed";
        case TrackerHealthFaultCode::FifoInitFailed: return "fifo_init_failed";
    }
    return "unknown";
}

class TrackerHealthState {
public:
    void reset() {
        fatalActive_ = false;
        degradedNoImu_ = false;
        faultCode_ = TrackerHealthFaultCode::None;
        message_[0] = '\0';
        ++revision_;
    }

    void enterDegradedNoImu(TrackerHealthFaultCode code, const char* message) {
        fatalActive_ = true;
        degradedNoImu_ = true;
        faultCode_ = code;
        copyMessage(message);
        ++revision_;
    }

    TrackerHealthSnapshot snapshot() const {
        TrackerHealthSnapshot out;
        out.fatalActive = fatalActive_;
        out.degradedNoImu = degradedNoImu_;
        out.faultCode = faultCode_;
        out.revision = revision_;
        std::memcpy(out.message, message_, sizeof(out.message));
        return out;
    }

    bool fatalActive() const { return fatalActive_; }
    bool degradedNoImu() const { return degradedNoImu_; }
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
    TrackerHealthFaultCode faultCode_ = TrackerHealthFaultCode::None;
    uint32_t revision_ = 0;
    char message_[64] = {};
};

} // namespace tracker
