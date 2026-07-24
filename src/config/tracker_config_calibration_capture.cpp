#include "config/tracker_config_runtime.hpp"

#include "sensor/accel_6pos_calibration.hpp"

namespace tracker {

namespace {

void captureAccelQualityPayload(TrackerConfig& config, const Accel6PosCalibration& cal) {
    const Accel6PosCalibration::Result& r = cal.result();
    config.data.accelCalQuality.qualityFlags = r.qualityFlags;
    config.data.accelCalQuality.qualityScore = r.qualityScore;
    config.data.accelCalQuality.maxFaceNormErrorG = r.maxFaceNormErrorG;
    config.data.accelCalQuality.maxAxisResidualG = r.maxAxisResidualG;

    for (uint8_t i = 0; i < 6; ++i) {
        const auto face = static_cast<Accel6PosCalibration::Face>(i);
        config.data.accelCalQuality.faceSamples[i] = cal.faceData(face).samples;
        config.data.accelCalQuality.faceNormErrorG[i] = r.faceNormErrorG[i];
        config.data.accelCalQuality.faceAxisResidualG[i] = r.faceAxisResidualG[i];
    }
}

} // namespace

void TrackerConfig::captureFromAccelCalibrationQuality(const Accel6PosCalibration& cal, uint32_t uptimeMs) {
    captureAccelQualityPayload(*this, cal);
    data.accelCalQuality.calibrationUptimeMs = uptimeMs;
    updateCrc();
}

} // namespace tracker
