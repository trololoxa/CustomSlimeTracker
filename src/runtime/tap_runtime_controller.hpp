#pragma once

#include <cstdint>

#include "defines.h"
#include "connection/lsm6dsv_driver.hpp"
#include "runtime/tap_accumulator.hpp"

namespace tracker {

class SlimeVROutputRuntime;

constexpr uint8_t TRACKER_TAP_VALUE_MIN = TRACKER_TAP_PACKET_MIN_VALUE;
constexpr uint8_t TRACKER_TAP_VALUE_MAX = TRACKER_TAP_MAX_COUNT;

struct TapRuntimeConfig {
    bool enabled = TRACKER_ENABLE_TAP_RUNTIME != 0;
    bool hardwareDoubleTap = TRACKER_TAP_HARDWARE_DOUBLE_TAP != 0;
    bool slidingWindow = TRACKER_TAP_SLIDING_WINDOW != 0;

    uint8_t sensorId = 0;
    uint8_t minCount = TRACKER_TAP_MIN_COUNT;
    uint8_t maxCount = TRACKER_TAP_MAX_COUNT;
    uint16_t pollIntervalMs = TRACKER_TAP_POLL_INTERVAL_MS;
    uint16_t aggregationWindowMs = TRACKER_TAP_AGGREGATION_WINDOW_MS;
    uint16_t duplicateSuppressMs = TRACKER_TAP_DUPLICATE_SUPPRESS_MS;
    uint16_t postSendLockoutMs = TRACKER_TAP_POST_SEND_LOCKOUT_MS;
    uint16_t registerVerifyIntervalMs = TRACKER_TAP_REGISTER_VERIFY_INTERVAL_MS;

    uint8_t threshold = TRACKER_LSM6DSV_TAP_THRESHOLD;
    uint8_t shock = TRACKER_LSM6DSV_TAP_SHOCK;
    uint8_t quiet = TRACKER_LSM6DSV_TAP_QUIET;
    uint8_t duration = TRACKER_LSM6DSV_TAP_DURATION;
};

struct TapRuntimeStatus {
    bool enabled = false;
    bool hardwareConfigured = false;
    bool lastReadOk = false;
    uint8_t lastRawSource = 0;
    uint8_t lastPhysicalCount = 0;
    uint8_t lastValue = 0;
    bool lastSentOk = false;

    uint32_t singleDetected = 0;
    uint32_t doubleDetected = 0;
    uint32_t tapDetectedNoType = 0;
    uint32_t sent = 0;
    uint32_t sendFailures = 0;
    uint32_t noServer = 0;
    uint32_t readFailures = 0;
    uint32_t configureFailures = 0;
    uint32_t lastEventMs = 0;
    uint32_t lastSentMs = 0;

    uint8_t pendingCount = 0;
    uint32_t pendingFirstMs = 0;
    uint32_t pendingLastMs = 0;
    uint32_t flushDeadlineMs = 0;
    uint32_t lastPhysicalTapMs = 0;
    uint32_t postSendLockoutUntilMs = 0;
    uint32_t physicalTapEvents = 0;
    uint32_t physicalTapCount = 0;
    uint32_t windowsStarted = 0;
    uint32_t windowsFlushed = 0;
    uint32_t suppressedBelowMin = 0;
    uint32_t suppressedDuplicate = 0;
    uint32_t suppressedLockout = 0;
    uint32_t clampedOverflow = 0;

    bool registerVerifyOk = false;
    bool lastRegisterReadOk = false;
    uint32_t registerVerifyFailures = 0;
    uint32_t registerMismatchCount = 0;
    uint32_t lastRegisterVerifyMs = 0;
    Lsm6dsv::TapRegisterVerification lastRegisterVerification;
};

class TapRuntimeController {
public:
    void begin(Lsm6dsv& lsm, SlimeVROutputRuntime& slimevr);
    bool configure(const TapRuntimeConfig& config);
    bool setEnabled(bool enabled);
    bool enabled() const { return config_.enabled; }
    void resetCounters();
    bool update(uint32_t nowMs);
    bool sendManualTap(uint8_t value, uint32_t nowMs);
    bool injectPhysicalTaps(uint8_t count, uint32_t nowMs);
    TapRuntimeStatus status() const;

private:
    bool configureHardware();
    bool verifyHardware(uint32_t nowMs, bool force);
    void maybeVerifyHardware(uint32_t nowMs);
    bool flushAccumulator(uint32_t nowMs);
    bool handleSource(const Lsm6dsv::TapSource& source, uint32_t nowMs, bool manual);
    bool handleAccumulatorAction(const TapAccumulatorAction& action, uint32_t nowMs);
    bool sendTap(uint8_t value, uint32_t nowMs);
    Lsm6dsv::TapConfig makeHardwareConfig() const;
    TapAccumulatorConfig makeAccumulatorConfig() const;
    static uint8_t normalizeTapValue(uint8_t value);

    Lsm6dsv* lsm_ = nullptr;
    SlimeVROutputRuntime* slimevr_ = nullptr;
    TapRuntimeConfig config_;
    TapRuntimeStatus status_;
    TapAccumulator accumulator_;
    uint32_t nextPollMs_ = 0;
    uint32_t nextRegisterVerifyMs_ = 0;
};

} // namespace tracker
