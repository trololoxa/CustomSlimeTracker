#pragma once

#include <cstdint>

namespace tracker {

enum class SensorProgressSuppressReason : uint8_t {
    Boot = 1u << 0,
    IntentionalSleep = 1u << 1,
    BlockingScan = 1u << 2,
    SensorReinit = 1u << 3,
    CalibrationCapture = 1u << 4,
};

enum class SensorProgressFault : uint8_t {
    None = 0,
    NoIrqOrDrain,
    NoAcceptedGyro,
    NoOrientationPublication,
};

struct SensorProgressSnapshot {
    bool configured = false;
    uint8_t suppressMask = 0u;
    uint32_t timeoutUs = 0u;
    uint32_t armedAtUs = 0u;
    uint32_t lastIrqAtUs = 0u;
    uint32_t lastDrainAtUs = 0u;
    uint32_t lastAcceptedGyroAtUs = 0u;
    uint32_t lastOrientationAtUs = 0u;
    uint32_t faultCount = 0u;
    SensorProgressFault lastFault = SensorProgressFault::None;
    SensorProgressFault lastTriggeredFault = SensorProgressFault::None;
    uint32_t lastFaultAtUs = 0u;
};

// Orientation liveness is meaningful only after the producer is contractually
// allowed to publish. Strict tracking recovery intentionally suppresses pose
// until a short monotonic-gyro proof completes; IRQ/drain and accepted-gyro
// progress remain enforced during that blackout.
inline bool sensorProgressOrientationExpected(bool ahrsInitialized,
                                              bool trackingRecoveryActive,
                                              bool degradedGyroOutputAllowed) {
    return ahrsInitialized &&
           (!trackingRecoveryActive || degradedGyroOutputAllowed);
}

// Fixed-size, wrap-safe liveness monitor. IRQ/drain producers publish local
// MCU-clock observations; per-sample producers publish timestamp-free sequence
// edges. All timeout decisions run from the app service path rather than the
// IMU ISR or the per-sample correction path.
class SensorProgressWatchdog {
public:
    void configure(uint32_t nowUs, float samplePeriodUs, uint16_t watermarkWords) {
        uint32_t periodUs = 1000u;
        if (samplePeriodUs >= 1.0f && samplePeriodUs <= 1000000.0f) {
            periodUs = static_cast<uint32_t>(samplePeriodUs + 0.5f);
        }
        const uint32_t words = watermarkWords == 0u ? 1u : watermarkWords;
        uint64_t derived = static_cast<uint64_t>(periodUs) * words * 16u;
        if (derived < kMinimumTimeoutUs) derived = kMinimumTimeoutUs;
        if (derived > kMaximumTimeoutUs) derived = kMaximumTimeoutUs;
        timeoutUs_ = static_cast<uint32_t>(derived);
        configured_ = true;
        resetEpoch(nowUs);
    }

    void resetEpoch(uint32_t nowUs) {
        armedAtUs_ = nowUs;
        lastIrqAtUs_ = 0u;
        lastDrainAtUs_ = 0u;
        lastAcceptedGyroAtUs_ = 0u;
        lastOrientationAtUs_ = 0u;
        observedAcceptedGyroProgress_ = acceptedGyroProgress_;
        observedOrientationProgress_ = orientationProgress_;
        lastFault_ = SensorProgressFault::None;
    }

    void noteIrq(uint32_t nowUs) { lastIrqAtUs_ = nonzero(nowUs); }
    void noteDrain(uint32_t nowUs) { lastDrainAtUs_ = nonzero(nowUs); }

    // Producer timestamps belong to the IMU hardware timebase and must never
    // be compared with micros(). Publish only a cheap sequence edge here; the
    // bounded app watchdog service timestamps that edge in its own clock
    // domain. This retains zero clock reads in the 960 Hz sample path.
    void noteAcceptedGyro() { ++acceptedGyroProgress_; }
    void noteOrientationPublication() { ++orientationProgress_; }

    void setSuppressed(SensorProgressSuppressReason reason, bool suppressed, uint32_t nowUs) {
        const uint8_t bit = static_cast<uint8_t>(reason);
        const uint8_t before = suppressMask_;
        if (suppressed) suppressMask_ |= bit;
        else suppressMask_ &= static_cast<uint8_t>(~bit);
        if (before != 0u && suppressMask_ == 0u) {
            // A blocking service or reinit is an intentional discontinuity.
            // Give the new epoch a complete grace window instead of reporting
            // an immediate fault based on pre-suppression timestamps.
            resetEpoch(nowUs);
        }
    }

    SensorProgressFault evaluate(uint32_t nowUs, bool orientationExpected) {
        if (!configured_ || suppressMask_ != 0u) return SensorProgressFault::None;

        observeProducerProgress(nowUs);

        const uint32_t irqOrDrainAt = newer(lastIrqAtUs_, lastDrainAtUs_);
        SensorProgressFault fault = SensorProgressFault::None;
        if (expired(nowUs, irqOrDrainAt)) {
            fault = SensorProgressFault::NoIrqOrDrain;
        } else if (expired(nowUs, lastAcceptedGyroAtUs_)) {
            fault = SensorProgressFault::NoAcceptedGyro;
        } else if (orientationExpected && expired(nowUs, lastOrientationAtUs_)) {
            fault = SensorProgressFault::NoOrientationPublication;
        }

        if (fault != SensorProgressFault::None && fault != lastFault_) {
            ++faultCount_;
            lastTriggeredFault_ = fault;
            lastFaultAtUs_ = nowUs;
        }
        lastFault_ = fault;
        return fault;
    }

    uint32_t timeoutUs() const { return timeoutUs_; }
    uint8_t suppressMask() const { return suppressMask_; }

    SensorProgressSnapshot snapshot() const {
        SensorProgressSnapshot out;
        out.configured = configured_;
        out.suppressMask = suppressMask_;
        out.timeoutUs = timeoutUs_;
        out.armedAtUs = armedAtUs_;
        out.lastIrqAtUs = lastIrqAtUs_;
        out.lastDrainAtUs = lastDrainAtUs_;
        out.lastAcceptedGyroAtUs = lastAcceptedGyroAtUs_;
        out.lastOrientationAtUs = lastOrientationAtUs_;
        out.faultCount = faultCount_;
        out.lastFault = lastFault_;
        out.lastTriggeredFault = lastTriggeredFault_;
        out.lastFaultAtUs = lastFaultAtUs_;
        return out;
    }

private:
    static constexpr uint32_t kMinimumTimeoutUs = 500000u;
    static constexpr uint32_t kMaximumTimeoutUs = 2000000u;

    static uint32_t nonzero(uint32_t value) { return value == 0u ? 1u : value; }

    static uint32_t newer(uint32_t a, uint32_t b) {
        if (a == 0u) return b;
        if (b == 0u) return a;
        return static_cast<int32_t>(a - b) >= 0 ? a : b;
    }

    bool expired(uint32_t nowUs, uint32_t timestampUs) const {
        const uint32_t base = timestampUs == 0u ? armedAtUs_ : timestampUs;
        return static_cast<uint32_t>(nowUs - base) >= timeoutUs_;
    }

    void observeProducerProgress(uint32_t nowUs) {
        if (acceptedGyroProgress_ != observedAcceptedGyroProgress_) {
            observedAcceptedGyroProgress_ = acceptedGyroProgress_;
            lastAcceptedGyroAtUs_ = nonzero(nowUs);
        }
        if (orientationProgress_ != observedOrientationProgress_) {
            observedOrientationProgress_ = orientationProgress_;
            lastOrientationAtUs_ = nonzero(nowUs);
        }
    }

    bool configured_ = false;
    uint8_t suppressMask_ = 0u;
    uint32_t timeoutUs_ = kMaximumTimeoutUs;
    uint32_t armedAtUs_ = 0u;
    uint32_t lastIrqAtUs_ = 0u;
    uint32_t lastDrainAtUs_ = 0u;
    uint32_t lastAcceptedGyroAtUs_ = 0u;
    uint32_t lastOrientationAtUs_ = 0u;
    uint32_t acceptedGyroProgress_ = 0u;
    uint32_t orientationProgress_ = 0u;
    uint32_t observedAcceptedGyroProgress_ = 0u;
    uint32_t observedOrientationProgress_ = 0u;
    uint32_t faultCount_ = 0u;
    SensorProgressFault lastFault_ = SensorProgressFault::None;
    SensorProgressFault lastTriggeredFault_ = SensorProgressFault::None;
    uint32_t lastFaultAtUs_ = 0u;
};

inline const char* sensorProgressFaultName(SensorProgressFault fault) {
    switch (fault) {
        case SensorProgressFault::None: return "none";
        case SensorProgressFault::NoIrqOrDrain: return "no_irq_or_drain";
        case SensorProgressFault::NoAcceptedGyro: return "no_accepted_gyro";
        case SensorProgressFault::NoOrientationPublication: return "no_orientation_publication";
    }
    return "unknown";
}

} // namespace tracker
