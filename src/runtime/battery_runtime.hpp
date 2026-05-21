#pragma once

#include <cstdint>

namespace tracker {

using BatteryAdcReadMillivoltsFn = bool (*)(uint16_t& outMillivolts, void* user);

struct BatteryRuntimeConfig {
    bool enabled = true;
    int adcPin = -1;
    float rTopOhms = 180000.0f;
    float rBottomOhms = 180000.0f;
    float voltageScale = 1.0f;
    float voltageOffset = 0.0f;
    float emptyVoltage = 3.30f;
    float fullVoltage = 4.20f;
    float presentVoltageMin = 1.00f;
    float emaAlpha = 0.25f;
    float maxFilterStepVoltage = 0.0f;
    uint32_t sampleIntervalMs = 1000;
    uint8_t startupSamples = 3;
};

struct BatteryRuntimeStatus {
    bool enabled = false;
    bool configured = false;
    bool lastReadOk = false;
    bool present = false;
    bool filteredValid = false;
    int adcPin = -1;
    uint16_t lastAdcMillivolts = 0;
    float lastAdcVoltage = 0.0f;
    float voltage = 0.0f;
    float percentage = 0.0f;
    uint32_t samples = 0;
    uint32_t readFailures = 0;
    uint32_t invalidSamples = 0;
    uint32_t noBatterySamples = 0;
    uint32_t glitchRejectedSamples = 0;
    uint32_t lastSampleMs = 0;
};

class BatteryRuntime {
public:
    void begin(BatteryAdcReadMillivoltsFn readMillivolts, void* user = nullptr);
    void configure(const BatteryRuntimeConfig& config);
    void reset();
    bool update(uint32_t nowMs);

    BatteryRuntimeStatus status() const { return status_; }
    bool telemetry(float& voltage, float& percentage) const;

    static float dividerRatio(float rTopOhms, float rBottomOhms);
    static float batteryVoltageFromAdcMillivolts(uint16_t adcMillivolts,
                                                 float rTopOhms,
                                                 float rBottomOhms,
                                                 float scale = 1.0f,
                                                 float offset = 0.0f);
    static float percentageFromVoltage(float voltage, float emptyVoltage, float fullVoltage);

private:
    bool shouldSample(uint32_t nowMs) const;
    void applySample(uint32_t nowMs, uint16_t adcMillivolts, float batteryVoltage);
    void applyNoBatterySample(uint32_t nowMs, uint16_t adcMillivolts, float batteryVoltage);

    BatteryRuntimeConfig config_;
    BatteryRuntimeStatus status_;
    BatteryAdcReadMillivoltsFn readMillivolts_ = nullptr;
    void* readUser_ = nullptr;
    float filteredVoltage_ = 0.0f;
    uint8_t startupSamplesRemaining_ = 0;
};

} // namespace tracker
