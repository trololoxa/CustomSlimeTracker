#pragma once

#include <Arduino.h>
#include <cstdint>

#include "connection/lsm6dsv_driver.hpp"
#include "sensor/imu_quality.hpp"

namespace tracker {

class RuntimeMotionDiagnostics {
public:
    struct WindowStats {
        bool enabled = false;
        bool configured = false;
        uint32_t resetMs = 0;
        uint32_t lastRecordMs = 0;
        uint64_t lastTimestampUs = 0;

        uint32_t samples = 0;
        uint32_t ahrsUsableSamples = 0;
        uint32_t ahrsSkippedSamples = 0;
        uint32_t accelCorrectionDisabledSamples = 0;
        uint32_t estimatedDroppedSamples = 0;

        uint32_t hwTimestampSamples = 0;
        uint32_t fallbackTimestampSamples = 0;
        uint32_t largeGapSamples = 0;
        uint32_t nonMonotonicTimestampSamples = 0;
        uint32_t fifoRecoveryRequests = 0;

        uint32_t gyroSaturatedSamples = 0;
        uint32_t gyroNearSaturatedSamples = 0;
        uint32_t accelSaturatedSamples = 0;
        uint32_t accelNearSaturatedSamples = 0;
        uint32_t accelNormOutliers = 0;

        double dtSumUs = 0.0;
        uint32_t dtMinUs = 0;
        uint32_t dtMaxUs = 0;

        double gyroNormSumDps = 0.0;
        float gyroNormMaxDps = 0.0f;
        float gyroNormLastDps = 0.0f;

        double accelNormSumG = 0.0;
        float accelNormMinG = 0.0f;
        float accelNormMaxG = 0.0f;
        float accelNormLastG = 0.0f;

        float confidenceLast = 0.0f;
        double confidenceSum = 0.0;

        float tempStartC = 0.0f;
        float tempLastC = 0.0f;
        bool tempValid = false;

        uint32_t qualityFlagsOr = 0;
        uint32_t lastQualityFlags = 0;
    };

    void begin(uint32_t nowMs);
    void setEnabled(bool enabled, uint32_t nowMs);
    bool enabled() const { return stats_.enabled; }
    void reset(uint32_t nowMs);

    void recordSample(const Lsm6dsv::RawSample& raw,
                      const Lsm6dsv::Sample& calibrated,
                      const ImuQualityResult& quality,
                      uint32_t nowMs);

    const WindowStats& stats() const { return stats_; }
    uint32_t windowMs(uint32_t nowMs) const;

    void printStatus(Stream& out, uint32_t nowMs) const;

private:
    static float sampleRateHz(const WindowStats& s, uint32_t windowMs);
    static float avgDtUs(const WindowStats& s);
    static float avgGyroNormDps(const WindowStats& s);
    static float avgAccelNormG(const WindowStats& s);
    static float avgConfidence(const WindowStats& s);
    static void updateMinMaxU32(uint32_t value, uint32_t& minValue, uint32_t& maxValue, bool first);
    static void updateMinMaxF(float value, float& minValue, float& maxValue, bool first);

    WindowStats stats_;
};

} // namespace tracker
