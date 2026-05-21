#include "runtime/battery_runtime.hpp"

#include <cmath>

namespace tracker {

namespace {

float clampFloat(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

bool finitePositive(float value) {
    return std::isfinite(value) && value > 0.0f;
}

} // namespace

void BatteryRuntime::begin(BatteryAdcReadMillivoltsFn readMillivolts, void* user) {
    readMillivolts_ = readMillivolts;
    readUser_ = user;
    reset();
}

void BatteryRuntime::configure(const BatteryRuntimeConfig& config) {
    config_ = config;
    if (config_.sampleIntervalMs == 0) config_.sampleIntervalMs = 1000;
    if (!finitePositive(config_.rTopOhms)) config_.rTopOhms = 180000.0f;
    if (!finitePositive(config_.rBottomOhms)) config_.rBottomOhms = 180000.0f;
    if (!finitePositive(config_.voltageScale)) config_.voltageScale = 1.0f;
    if (!std::isfinite(config_.voltageOffset)) config_.voltageOffset = 0.0f;
    if (!finitePositive(config_.fullVoltage)) config_.fullVoltage = 4.20f;
    if (!finitePositive(config_.emptyVoltage)) config_.emptyVoltage = 3.30f;
    if (config_.fullVoltage <= config_.emptyVoltage + 0.01f) {
        config_.emptyVoltage = 3.30f;
        config_.fullVoltage = 4.20f;
    }
    if (!std::isfinite(config_.presentVoltageMin) || config_.presentVoltageMin < 0.0f) {
        config_.presentVoltageMin = 1.00f;
    }
    if (!std::isfinite(config_.emaAlpha) || config_.emaAlpha <= 0.0f) config_.emaAlpha = 0.25f;
    if (config_.emaAlpha > 1.0f) config_.emaAlpha = 1.0f;
    if (!std::isfinite(config_.maxFilterStepVoltage) || config_.maxFilterStepVoltage < 0.0f) {
        config_.maxFilterStepVoltage = 0.0f;
    }

    status_.enabled = config_.enabled;
    status_.configured = config_.enabled && readMillivolts_ != nullptr && config_.adcPin >= 0;
    status_.adcPin = config_.adcPin;
    startupSamplesRemaining_ = config_.startupSamples;
    if (!status_.configured) {
        status_.lastReadOk = false;
        status_.present = false;
        status_.filteredValid = false;
        status_.lastAdcMillivolts = 0;
        status_.lastAdcVoltage = 0.0f;
        status_.voltage = 0.0f;
        status_.percentage = 0.0f;
        filteredVoltage_ = 0.0f;
    }
}

void BatteryRuntime::reset() {
    status_ = BatteryRuntimeStatus{};
    status_.adcPin = config_.adcPin;
    status_.enabled = config_.enabled;
    status_.configured = config_.enabled && readMillivolts_ != nullptr && config_.adcPin >= 0;
    filteredVoltage_ = 0.0f;
    startupSamplesRemaining_ = config_.startupSamples;
}

bool BatteryRuntime::update(uint32_t nowMs) {
    status_.enabled = config_.enabled;
    status_.configured = config_.enabled && readMillivolts_ != nullptr && config_.adcPin >= 0;
    status_.adcPin = config_.adcPin;
    if (!status_.configured) return false;
    if (!shouldSample(nowMs)) return false;

    uint16_t adcMv = 0;
    const bool ok = readMillivolts_(adcMv, readUser_);
    status_.lastReadOk = ok;
    if (!ok) {
        ++status_.readFailures;
        status_.lastSampleMs = nowMs;
        // Do not collapse an already valid battery estimate to 0% on a
        // transient ADC read failure. If no valid sample has ever
        // been accepted, telemetry() still reports the safe absent-battery
        // value: 0.000 V / 0.0%.
        status_.present = status_.filteredValid;
        if (!status_.filteredValid) {
            status_.voltage = 0.0f;
            status_.percentage = 0.0f;
        }
        return true;
    }

    const float batteryVoltage = batteryVoltageFromAdcMillivolts(
        adcMv,
        config_.rTopOhms,
        config_.rBottomOhms,
        config_.voltageScale,
        config_.voltageOffset
    );

    if (!std::isfinite(batteryVoltage) || batteryVoltage < 0.0f || batteryVoltage > 8.0f) {
        ++status_.invalidSamples;
        status_.lastAdcMillivolts = adcMv;
        status_.lastAdcVoltage = static_cast<float>(adcMv) / 1000.0f;
        status_.lastSampleMs = nowMs;
        status_.present = false;
        status_.filteredValid = false;
        filteredVoltage_ = 0.0f;
        status_.voltage = 0.0f;
        status_.percentage = 0.0f;
        return true;
    }

    if (batteryVoltage < config_.presentVoltageMin) {
        applyNoBatterySample(nowMs, adcMv, batteryVoltage);
        return true;
    }

    if (status_.filteredValid && config_.maxFilterStepVoltage > 0.0f &&
        std::fabs(batteryVoltage - filteredVoltage_) > config_.maxFilterStepVoltage) {
        ++status_.glitchRejectedSamples;
        status_.lastAdcMillivolts = adcMv;
        status_.lastAdcVoltage = static_cast<float>(adcMv) / 1000.0f;
        status_.lastSampleMs = nowMs;
        status_.lastReadOk = true;
        status_.present = true;
        // Keep the previous filtered voltage/percentage. A real 1S battery
        // cannot jump by hundreds of millivolts between sparse samples.
        return true;
    }

    applySample(nowMs, adcMv, batteryVoltage);
    return true;
}

bool BatteryRuntime::telemetry(float& voltage, float& percentage) const {
    if (!status_.enabled || !status_.configured) {
        voltage = 0.0f;
        percentage = 0.0f;
        return false;
    }
    voltage = status_.present ? status_.voltage : 0.0f;
    percentage = status_.present ? status_.percentage : 0.0f;
    return true;
}

float BatteryRuntime::dividerRatio(float rTopOhms, float rBottomOhms) {
    if (!finitePositive(rTopOhms) || !finitePositive(rBottomOhms)) return 1.0f;
    return (rTopOhms + rBottomOhms) / rBottomOhms;
}

float BatteryRuntime::batteryVoltageFromAdcMillivolts(uint16_t adcMillivolts,
                                                      float rTopOhms,
                                                      float rBottomOhms,
                                                      float scale,
                                                      float offset) {
    const float adcVoltage = static_cast<float>(adcMillivolts) / 1000.0f;
    return adcVoltage * dividerRatio(rTopOhms, rBottomOhms) * scale + offset;
}

float BatteryRuntime::percentageFromVoltage(float voltage, float emptyVoltage, float fullVoltage) {
    if (!std::isfinite(voltage) || !std::isfinite(emptyVoltage) || !std::isfinite(fullVoltage)) return 0.0f;
    if (fullVoltage <= emptyVoltage + 0.01f) return 0.0f;
    return clampFloat(((voltage - emptyVoltage) * 100.0f) / (fullVoltage - emptyVoltage), 0.0f, 100.0f);
}

bool BatteryRuntime::shouldSample(uint32_t nowMs) const {
    if (startupSamplesRemaining_ > 0) return true;
    if (status_.lastSampleMs == 0) return true;
    return nowMs - status_.lastSampleMs >= config_.sampleIntervalMs;
}

void BatteryRuntime::applySample(uint32_t nowMs, uint16_t adcMillivolts, float batteryVoltage) {
    ++status_.samples;
    status_.lastAdcMillivolts = adcMillivolts;
    status_.lastAdcVoltage = static_cast<float>(adcMillivolts) / 1000.0f;
    status_.lastSampleMs = nowMs;
    status_.present = true;

    if (!status_.filteredValid || startupSamplesRemaining_ > 0) {
        filteredVoltage_ = batteryVoltage;
        status_.filteredValid = true;
        if (startupSamplesRemaining_ > 0) --startupSamplesRemaining_;
    } else {
        filteredVoltage_ = filteredVoltage_ + config_.emaAlpha * (batteryVoltage - filteredVoltage_);
    }

    status_.voltage = filteredVoltage_;
    status_.percentage = percentageFromVoltage(status_.voltage, config_.emptyVoltage, config_.fullVoltage);
}

void BatteryRuntime::applyNoBatterySample(uint32_t nowMs, uint16_t adcMillivolts, float batteryVoltage) {
    ++status_.samples;
    ++status_.noBatterySamples;
    status_.lastAdcMillivolts = adcMillivolts;
    status_.lastAdcVoltage = static_cast<float>(adcMillivolts) / 1000.0f;
    status_.lastSampleMs = nowMs;
    status_.present = false;
    status_.filteredValid = false;
    filteredVoltage_ = 0.0f;
    status_.voltage = 0.0f;
    status_.percentage = 0.0f;
    if (startupSamplesRemaining_ > 0) --startupSamplesRemaining_;
    (void)batteryVoltage;
}

} // namespace tracker
