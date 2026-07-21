#include "config/tracker_config_runtime.hpp"

#include "sensor/accel_6pos_calibration.hpp"

namespace tracker {

void TrackerConfig::captureFromAccelCalibrationQuality(const Accel6PosCalibration& cal, uint32_t uptimeMs) {
    const Accel6PosCalibration::Result& r = cal.result();
    data.accelCalQuality.calibrationUptimeMs = uptimeMs;
    data.accelCalQuality.qualityFlags = r.qualityFlags;
    data.accelCalQuality.qualityScore = r.qualityScore;
    data.accelCalQuality.maxFaceNormErrorG = r.maxFaceNormErrorG;
    data.accelCalQuality.maxAxisResidualG = r.maxAxisResidualG;

    for (uint8_t i = 0; i < 6; ++i) {
        const auto face = static_cast<Accel6PosCalibration::Face>(i);
        data.accelCalQuality.faceSamples[i] = cal.faceData(face).samples;
        data.accelCalQuality.faceNormErrorG[i] = r.faceNormErrorG[i];
        data.accelCalQuality.faceAxisResidualG[i] = r.faceAxisResidualG[i];
    }
    updateCrc();
}

} // namespace tracker
