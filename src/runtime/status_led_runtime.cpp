#include "runtime/status_led_runtime.hpp"

#include "serial/tracker_serial_parse.hpp"

#ifndef OUTPUT
#define OUTPUT 1
#endif

namespace tracker {

const char* trackerStatusLedModeName(TrackerStatusLedMode mode) {
    switch (mode) {
        case TrackerStatusLedMode::Disabled: return "disabled";
        case TrackerStatusLedMode::Off: return "off";
        case TrackerStatusLedMode::Boot: return "boot";
        case TrackerStatusLedMode::Normal: return "normal";
        case TrackerStatusLedMode::WifiConnecting: return "wifi_connecting";
        case TrackerStatusLedMode::ServerDiscovering: return "server_discovering";
        case TrackerStatusLedMode::ConnectionError: return "connection_error";
        case TrackerStatusLedMode::SensorError: return "sensor_error";
        case TrackerStatusLedMode::HardwareError: return "hardware_error";
        case TrackerStatusLedMode::Identify: return "identify";
        case TrackerStatusLedMode::ManualOn: return "manual_on";
        case TrackerStatusLedMode::ManualOff: return "manual_off";
    }
    return "unknown";
}

bool trackerStatusLedModeFromName(const char* name, TrackerStatusLedMode& out) {
    if (!name) return false;
    if (tracker_serial_detail::eqIgnoreCase(name, "disabled")) { out = TrackerStatusLedMode::Disabled; return true; }
    if (tracker_serial_detail::eqIgnoreCase(name, "off")) { out = TrackerStatusLedMode::Off; return true; }
    if (tracker_serial_detail::eqIgnoreCase(name, "boot")) { out = TrackerStatusLedMode::Boot; return true; }
    if (tracker_serial_detail::eqIgnoreCase(name, "normal") ||
        tracker_serial_detail::eqIgnoreCase(name, "connected")) { out = TrackerStatusLedMode::Normal; return true; }
    if (tracker_serial_detail::eqIgnoreCase(name, "wifi") ||
        tracker_serial_detail::eqIgnoreCase(name, "wifi_connecting") ||
        tracker_serial_detail::eqIgnoreCase(name, "connecting")) { out = TrackerStatusLedMode::WifiConnecting; return true; }
    if (tracker_serial_detail::eqIgnoreCase(name, "server") ||
        tracker_serial_detail::eqIgnoreCase(name, "server_discovering") ||
        tracker_serial_detail::eqIgnoreCase(name, "discovering")) { out = TrackerStatusLedMode::ServerDiscovering; return true; }
    if (tracker_serial_detail::eqIgnoreCase(name, "connection_error") ||
        tracker_serial_detail::eqIgnoreCase(name, "conn_error") ||
        tracker_serial_detail::eqIgnoreCase(name, "error")) { out = TrackerStatusLedMode::ConnectionError; return true; }
    if (tracker_serial_detail::eqIgnoreCase(name, "sensor_error") ||
        tracker_serial_detail::eqIgnoreCase(name, "sensor")) { out = TrackerStatusLedMode::SensorError; return true; }
    if (tracker_serial_detail::eqIgnoreCase(name, "hardware_error") ||
        tracker_serial_detail::eqIgnoreCase(name, "hardware")) { out = TrackerStatusLedMode::HardwareError; return true; }
    if (tracker_serial_detail::eqIgnoreCase(name, "identify") ||
        tracker_serial_detail::eqIgnoreCase(name, "id")) { out = TrackerStatusLedMode::Identify; return true; }
    if (tracker_serial_detail::eqIgnoreCase(name, "on") ||
        tracker_serial_detail::eqIgnoreCase(name, "manual_on")) { out = TrackerStatusLedMode::ManualOn; return true; }
    if (tracker_serial_detail::eqIgnoreCase(name, "manual_off")) { out = TrackerStatusLedMode::ManualOff; return true; }
    return false;
}

void GpioStatusLedSink::begin(int8_t pin, bool activeLow) {
    pin_ = pin;
    activeLow_ = activeLow;
    lastOn_ = false;
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    if (pin_ >= 0) {
        pinMode(pin_, OUTPUT);
        digitalWrite(pin_, activeLow_ ? HIGH : LOW);
    }
#else
    (void)OUTPUT;
#endif
}

void GpioStatusLedSink::write(bool on) {
    lastOn_ = on;
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    if (pin_ >= 0) {
        const int level = activeLow_ ? (on ? LOW : HIGH) : (on ? HIGH : LOW);
        digitalWrite(pin_, level);
    }
#endif
}

void StatusLedRuntime::begin(IStatusLedSink& sink, const StatusLedRuntimeConfig& config) {
    sink_ = &sink;
    configure(config);
}

void StatusLedRuntime::configure(const StatusLedRuntimeConfig& config) {
    config_ = config;
    if (config_.updateIntervalMs == 0) config_.updateIntervalMs = 1;
    if (config_.normalBlinkPeriodMs == 0) config_.normalBlinkPeriodMs = 10000UL;
    if (config_.normalBlinkOnMs == 0) config_.normalBlinkOnMs = 30;
    if (config_.shortBlinkOnMs == 0) config_.shortBlinkOnMs = 80;
    if (config_.shortBlinkOffMs == 0) config_.shortBlinkOffMs = 160;
    if (config_.longBlinkOnMs == 0) config_.longBlinkOnMs = 450;
    if (config_.longBlinkOffMs == 0) config_.longBlinkOffMs = 250;
    if (config_.errorBlinkPeriodMs == 0) config_.errorBlinkPeriodMs = 5000UL;
    if (config_.identifyBlinkOnMs == 0) config_.identifyBlinkOnMs = 100;
    if (config_.identifyBlinkOffMs == 0) config_.identifyBlinkOffMs = 100;

    configured_ = sink_ != nullptr && config_.enabled && config_.pin >= 0;
    if (sink_) sink_->begin(config_.pin, config_.activeLow);
    if (!configured_) {
        writeOutput(false);
    }
}

void StatusLedRuntime::setMode(TrackerStatusLedMode mode, uint32_t nowMs) {
    if (requestedMode_ == mode) return;
    requestedMode_ = mode;
    modeSinceMs_ = nowMs;
}

void StatusLedRuntime::setManualOverride(TrackerStatusLedMode mode, uint32_t nowMs) {
    if (mode != TrackerStatusLedMode::ManualOn &&
        mode != TrackerStatusLedMode::ManualOff &&
        mode != TrackerStatusLedMode::Off &&
        mode != TrackerStatusLedMode::Disabled) {
        manualMode_ = mode;
    } else if (mode == TrackerStatusLedMode::ManualOn) {
        manualMode_ = TrackerStatusLedMode::ManualOn;
    } else {
        manualMode_ = TrackerStatusLedMode::ManualOff;
    }
    manualOverride_ = true;
    modeSinceMs_ = nowMs;
}

void StatusLedRuntime::clearManualOverride(uint32_t nowMs) {
    if (!manualOverride_) return;
    manualOverride_ = false;
    modeSinceMs_ = nowMs;
}

void StatusLedRuntime::identify(uint32_t nowMs, uint32_t durationMs) {
    if (durationMs == 0) durationMs = TRACKER_STATUS_LED_IDENTIFY_DEFAULT_MS;
    identifyUntilMs_ = nowMs + durationMs;
    modeSinceMs_ = nowMs;
}

TrackerStatusLedMode StatusLedRuntime::effectiveMode(uint32_t nowMs) const {
    if (!config_.enabled || config_.pin < 0) return TrackerStatusLedMode::Disabled;
    if (identifyUntilMs_ != 0 && static_cast<int32_t>(nowMs - identifyUntilMs_) < 0) {
        return TrackerStatusLedMode::Identify;
    }
    if (manualOverride_) return manualMode_;
    return requestedMode_;
}

bool StatusLedRuntime::update(uint32_t nowMs) {
    if (!configured_) {
        const uint32_t before = writes_;
        writeOutput(false);
        lastUpdateMs_ = nowMs;
        return writes_ != before;
    }

    if (lastUpdateMs_ != 0 && nowMs - lastUpdateMs_ < config_.updateIntervalMs) {
        return false;
    }
    lastUpdateMs_ = nowMs;

    const uint32_t before = writes_;
    const TrackerStatusLedMode mode = effectiveMode(nowMs);
    const StatusLedPattern pattern = patternFor(mode, config_);
    const uint32_t elapsed = nowMs - modeSinceMs_;
    writeOutput(patternOutputAt(pattern, elapsed));
    return writes_ != before;
}

void StatusLedRuntime::resetCounters() {
    writes_ = 0;
}

StatusLedRuntimeStatus StatusLedRuntime::status() const {
    StatusLedRuntimeStatus s;
    s.configured = configured_;
    s.enabled = config_.enabled;
    s.pin = config_.pin;
    s.activeLow = config_.activeLow;
    s.outputOn = outputOn_;
    s.manualOverride = manualOverride_;
    s.requestedMode = requestedMode_;
    s.effectiveMode = effectiveMode(lastUpdateMs_);
    s.lastUpdateMs = lastUpdateMs_;
    s.modeSinceMs = modeSinceMs_;
    s.identifyUntilMs = identifyUntilMs_;
    s.writes = writes_;
    return s;
}

StatusLedPattern StatusLedRuntime::patternFor(TrackerStatusLedMode mode, const StatusLedRuntimeConfig& config) {
    StatusLedPattern p;
    switch (mode) {
        case TrackerStatusLedMode::Disabled:
        case TrackerStatusLedMode::Off:
        case TrackerStatusLedMode::ManualOff:
            p.forced = true;
            p.forcedOn = false;
            return p;
        case TrackerStatusLedMode::ManualOn:
            p.forced = true;
            p.forcedOn = true;
            return p;
        case TrackerStatusLedMode::Normal:
            p.blinkCount = 1;
            p.onMs = config.normalBlinkOnMs;
            p.offMs = 0;
            p.periodMs = config.normalBlinkPeriodMs;
            return p;
        case TrackerStatusLedMode::Boot:
        case TrackerStatusLedMode::WifiConnecting:
            p.blinkCount = 1;
            p.onMs = config.shortBlinkOnMs;
            p.offMs = config.shortBlinkOffMs;
            p.periodMs = 1000UL;
            return p;
        case TrackerStatusLedMode::ServerDiscovering:
        case TrackerStatusLedMode::ConnectionError:
            p.blinkCount = 3;
            p.onMs = config.longBlinkOnMs;
            p.offMs = config.longBlinkOffMs;
            p.periodMs = config.errorBlinkPeriodMs;
            return p;
        case TrackerStatusLedMode::SensorError:
            p.blinkCount = 2;
            p.onMs = config.longBlinkOnMs;
            p.offMs = config.longBlinkOffMs;
            p.periodMs = config.errorBlinkPeriodMs;
            return p;
        case TrackerStatusLedMode::HardwareError:
            p.blinkCount = 4;
            p.onMs = config.longBlinkOnMs;
            p.offMs = config.longBlinkOffMs;
            p.periodMs = config.errorBlinkPeriodMs;
            return p;
        case TrackerStatusLedMode::Identify:
            p.blinkCount = 255;
            p.onMs = config.identifyBlinkOnMs;
            p.offMs = config.identifyBlinkOffMs;
            p.periodMs = static_cast<uint32_t>(config.identifyBlinkOnMs) + config.identifyBlinkOffMs;
            return p;
    }
    p.forced = true;
    p.forcedOn = false;
    return p;
}

bool StatusLedRuntime::patternOutputAt(const StatusLedPattern& pattern, uint32_t elapsedMs) {
    if (pattern.forced) return pattern.forcedOn;
    if (pattern.blinkCount == 0 || pattern.onMs == 0 || pattern.periodMs == 0) return false;

    const uint32_t t = elapsedMs % pattern.periodMs;
    const uint32_t cycleMs = static_cast<uint32_t>(pattern.onMs) + pattern.offMs;
    if (cycleMs == 0) return false;

    const uint32_t clusterLen = static_cast<uint32_t>(pattern.blinkCount) * cycleMs;
    if (t >= clusterLen) return false;
    return (t % cycleMs) < pattern.onMs;
}

void StatusLedRuntime::writeOutput(bool on) {
    if (outputOn_ == on && writes_ != 0) return;
    outputOn_ = on;
    if (sink_) sink_->write(on);
    ++writes_;
}

} // namespace tracker
