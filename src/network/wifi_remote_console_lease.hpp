#pragma once

#include <cstdint>

namespace tracker {

// Wrap-safe, allocation-free application lease used to bound half-open TCP
// diagnostic sessions independently of the underlying lwIP timeout policy.
class WifiRemoteConsoleSessionLease {
public:
    void begin(uint32_t nowMs) {
        active_ = true;
        lastActivityMs_ = nowMs;
    }

    void noteActivity(uint32_t nowMs) {
        if (active_) lastActivityMs_ = nowMs;
    }

    void reset() {
        active_ = false;
        lastActivityMs_ = 0u;
    }

    bool active() const { return active_; }
    uint32_t ageMs(uint32_t nowMs) const {
        return active_ ? static_cast<uint32_t>(nowMs - lastActivityMs_) : 0u;
    }
    bool expired(uint32_t nowMs, uint32_t timeoutMs) const {
        return active_ && timeoutMs != 0u && ageMs(nowMs) >= timeoutMs;
    }

private:
    bool active_ = false;
    uint32_t lastActivityMs_ = 0u;
};

} // namespace tracker
