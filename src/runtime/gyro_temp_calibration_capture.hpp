#pragma once

#include <Arduino.h>
#include <cstdint>

#include "connection/lsm6dsv_driver.hpp"
#include "runtime/static_test_types.hpp"
#include "sensor/imu_quality.hpp"

namespace tracker {

struct GyroTempCalibrationCaptureConfig {
    // A short rolling stationary window rejects motion without making the
    // guided calibration harder to use. At the production IMU rate, 256
    // samples are well below one second; a brief touch only discards the
    // current window rather than the complete warm-up capture.
    uint32_t targetWindowSamples = 256;
    uint32_t minWindowSamples = 128;

    // Immediate gates stop obviously moving/faulty samples from entering a
    // candidate window. Window-level statistics below reject slow motion and
    // vibration that can pass an instantaneous norm check.
    float maxInstantGyroNormDps = 3.0f;
    float maxInstantAccelNormErrorG = 0.12f;
    float minInstantAccelConfidence = 0.60f;

    // The temperature fit needs an accurate window mean, not unrealistically
    // quiet individual 960 Hz samples. Keep a hard vibration ceiling, then
    // gate the standard error of the mean and consistency between independent
    // windows. The first accepted window must remain close to the just-fitted
    // rest bias; later windows may move only along a physically plausible
    // temperature-drift envelope.
    float maxInitialGyroMeanNormDps = 0.35f;
    float maxGyroStdNormDps = 1.20f;
    float maxGyroStdAxisDps = 0.80f;
    float maxGyroMeanStdErrorNormDps = 0.10f;
    float maxGyroMeanStdErrorAxisDps = 0.060f;
    float gyroAnchorSlackDps = 0.12f;
    float gyroAdjacentSlackDps = 0.10f;
    float maxThermalSlopeDpsPerC = 0.12f;
    float maxAccelNormMeanErrorG = 0.08f;
    float maxAccelNormStdG = 0.020f;
    float minAccelConfidenceMean = 0.75f;
    float maxWindowTempSpanC = 0.20f;
};

struct GyroTempCalibrationCaptureDiagnostics {
    uint32_t acceptedSamples = 0;
    uint32_t rejectedSamples = 0;
    uint32_t acceptedWindows = 0;
    uint32_t rejectedWindows = 0;
    uint32_t qualityRejectedSamples = 0;
    uint32_t motionRejectedSamples = 0;
    uint32_t motionWindowResets = 0;
    uint32_t gyroRejectedWindows = 0;
    uint32_t gyroHardNoiseRejectedWindows = 0;
    uint32_t gyroMeanPrecisionRejectedWindows = 0;
    uint32_t gyroInitialMeanRejectedWindows = 0;
    uint32_t gyroThermalConsistencyRejectedWindows = 0;
    uint32_t accelRejectedWindows = 0;
    uint32_t temperatureRejectedWindows = 0;
    uint32_t discardedPartialSamples = 0;
    uint32_t currentWindowSamples = 0;

    Vec3 lastGyroMeanDps = Vec3::zero();
    Vec3 lastGyroStdDps = Vec3::zero();
    Vec3 lastGyroMeanStdErrorDps = Vec3::zero();
    Vec3 lastGyroAnchorResidualDps = Vec3::zero();
    Vec3 lastGyroAdjacentResidualDps = Vec3::zero();
    float lastAccelNormMeanG = 0.0f;
    float lastAccelNormStdG = 0.0f;
    float lastAccelConfidenceMean = 0.0f;
    float lastTempSpanC = 0.0f;
};

// Dedicated gyro temperature calibration capture used by guided setup.
// It records the same temperature-bin data needed by the production temp fit,
// but it is intentionally separate from StaticTestRunner so `setup calibration`
// does not depend on the developer-only `test static` command path.
//
// Samples are committed to temperature bins only after a short contiguous
// stationary window passes mean/variance gates. Motion pauses collection and
// discards only the current short window; all previously accepted bins remain.
class GyroTempCalibrationCapture {
public:
    explicit GyroTempCalibrationCapture(
        const GyroTempCalibrationCaptureConfig& cfg = GyroTempCalibrationCaptureConfig{}
    );

    void setConfig(const GyroTempCalibrationCaptureConfig& cfg);
    const GyroTempCalibrationCaptureConfig& config() const;

    void start(uint32_t nowMs, uint32_t maxDurationMs);
    void stop(uint32_t nowMs);
    void reset();

    bool active() const;
    bool completed() const;
    uint32_t elapsedMs(uint32_t nowMs) const;

    void updateSample(const Lsm6dsv::Sample& calibrated,
                      const ImuQualityResult& quality,
                      uint32_t nowMs);

    const StaticRuntimeTest& capture() const;
    StaticRuntimeTest& capture();
    const GyroTempCalibrationCaptureDiagnostics& diagnostics() const;

    uint32_t usableTempBins(uint32_t minSamplesPerBin = 512) const;
    float tempRangeC() const;

private:
    struct PendingWindow {
        int8_t binIndex = -1;
        ScalarStats tempC;
        Vec3Stats gyroRadS;
        ScalarStats accelNormG;
        ScalarStats accelConfidence;

        void reset();
        uint32_t samples() const;
    };

    bool baseQualityGood(const Lsm6dsv::Sample& calibrated,
                         const ImuQualityResult& quality) const;
    bool immediateStationaryGood(const Lsm6dsv::Sample& calibrated,
                                 const ImuQualityResult& quality) const;
    void recordRejectedSample(int bin, bool motionRejected);
    void rejectPendingWindow(bool motionReset);
    void finalizePendingWindow(bool allowShortWindow);
    void resetPendingWindow();

    GyroTempCalibrationCaptureConfig cfg_;
    GyroTempCalibrationCaptureDiagnostics diagnostics_;
    StaticRuntimeTest capture_;
    PendingWindow pending_;
    bool haveAcceptedGyroReference_ = false;
    float acceptedReferenceTempC_ = 0.0f;
    Vec3 acceptedReferenceGyroMeanDps_ = Vec3::zero();
    float lastAcceptedTempC_ = 0.0f;
    Vec3 lastAcceptedGyroMeanDps_ = Vec3::zero();
    bool completed_ = false;
};

} // namespace tracker
