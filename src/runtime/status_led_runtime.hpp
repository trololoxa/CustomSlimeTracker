#pragma once

#include <Arduino.h>
#include <cstdint>

#include "defines.h"

namespace tracker {

enum class TrackerStatusLedMode : uint8_t {
    Disabled,
    Off,
    Boot,
    Normal,
    WifiConnecting,
    ServerDiscovering,
    ConnectionError,
    SensorError,
    HardwareError,
    Identify,
    ManualOn,
    ManualOff,
};

const char* trackerStatusLedModeName(TrackerStatusLedMode mode);
bool trackerStatusLedModeFromName(const char* name, TrackerStatusLedMode& out);

struct StatusLedPattern {
    bool forced = false;
    bool forcedOn = false;
    uint8_t blinkCount = 0;
    uint16_t onMs = 0;
    uint16_t offMs = 0;
    uint32_t periodMs = 0;
};

struct StatusLedRuntimeConfig {
    bool enabled = TRACKER_ENABLE_STATUS_LED != 0;
    int8_t pin = TRACKER_STATUS_LED_PIN;
    bool activeLow = TRACKER_STATUS_LED_ACTIVE_LOW != 0;
    uint16_t updateIntervalMs = TRACKER_STATUS_LED_UPDATE_INTERVAL_MS;
    uint32_t normalBlinkPeriodMs = TRACKER_STATUS_LED_NORMAL_BLINK_PERIOD_MS;
    uint16_t normalBlinkOnMs = TRACKER_STATUS_LED_NORMAL_BLINK_ON_MS;
    uint16_t shortBlinkOnMs = TRACKER_STATUS_LED_SHORT_BLINK_ON_MS;
    uint16_t shortBlinkOffMs = TRACKER_STATUS_LED_SHORT_BLINK_OFF_MS;
    uint16_t longBlinkOnMs = TRACKER_STATUS_LED_LONG_BLINK_ON_MS;
    uint16_t longBlinkOffMs = TRACKER_STATUS_LED_LONG_BLINK_OFF_MS;
    uint32_t errorBlinkPeriodMs = TRACKER_STATUS_LED_ERROR_BLINK_PERIOD_MS;
    uint16_t identifyBlinkOnMs = TRACKER_STATUS_LED_IDENTIFY_BLINK_ON_MS;
    uint16_t identifyBlinkOffMs = TRACKER_STATUS_LED_IDENTIFY_BLINK_OFF_MS;
};

struct StatusLedRuntimeStatus {
    bool configured = false;
    bool enabled = false;
    int8_t pin = -1;
    bool activeLow = true;
    bool outputOn = false;
    bool manualOverride = false;
    TrackerStatusLedMode requestedMode = TrackerStatusLedMode::Disabled;
    TrackerStatusLedMode effectiveMode = TrackerStatusLedMode::Disabled;
    uint32_t lastUpdateMs = 0;
    uint32_t modeSinceMs = 0;
    uint32_t identifyUntilMs = 0;
    uint32_t writes = 0;
};

class IStatusLedSink {
public:
    virtual ~IStatusLedSink() = default;
    virtual void begin(int8_t pin, bool activeLow) = 0;
    virtual void write(bool on) = 0;
};

class GpioStatusLedSink final : public IStatusLedSink {
public:
    void begin(int8_t pin, bool activeLow) override;
    void write(bool on) override;

    int8_t pin() const { return pin_; }
    bool activeLow() const { return activeLow_; }
    bool lastOn() const { return lastOn_; }

private:
    int8_t pin_ = -1;
    bool activeLow_ = true;
    bool lastOn_ = false;
};

class StatusLedRuntime {
public:
    void begin(IStatusLedSink& sink, const StatusLedRuntimeConfig& config = StatusLedRuntimeConfig{});
    void configure(const StatusLedRuntimeConfig& config);
    bool update(uint32_t nowMs);
    void setMode(TrackerStatusLedMode mode, uint32_t nowMs);
    void setManualOverride(TrackerStatusLedMode mode, uint32_t nowMs);
    void clearManualOverride(uint32_t nowMs);
    void identify(uint32_t nowMs, uint32_t durationMs);
    void resetCounters();

    TrackerStatusLedMode requestedMode() const { return requestedMode_; }
    TrackerStatusLedMode effectiveMode(uint32_t nowMs) const;
    StatusLedRuntimeStatus status() const;

    static StatusLedPattern patternFor(TrackerStatusLedMode mode, const StatusLedRuntimeConfig& config);
    static bool patternOutputAt(const StatusLedPattern& pattern, uint32_t elapsedMs);

private:
    void writeOutput(bool on);

    IStatusLedSink* sink_ = nullptr;
    StatusLedRuntimeConfig config_;
    bool configured_ = false;
    bool outputOn_ = false;
    bool manualOverride_ = false;
    TrackerStatusLedMode requestedMode_ = TrackerStatusLedMode::Disabled;
    TrackerStatusLedMode manualMode_ = TrackerStatusLedMode::ManualOff;
    uint32_t lastUpdateMs_ = 0;
    uint32_t modeSinceMs_ = 0;
    uint32_t identifyUntilMs_ = 0;
    uint32_t writes_ = 0;
};

} // namespace tracker
