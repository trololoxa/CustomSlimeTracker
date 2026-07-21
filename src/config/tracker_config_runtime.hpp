#pragma once

#include <Arduino.h>
#include <cstdint>

#include "config/tracker_config_schema.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/imu_quality.hpp"

namespace tracker {

struct ImuCalibration;
class GyroTempCompensator;
class Accel6PosCalibration;
struct MagCalibrationResult;

class TrackerConfig {
public:
    TrackerConfigBlob data;

    void resetDefaults();
    bool validate() const;
    bool validateContent() const;
    uint32_t computeCrc() const;
    void updateCrc();
    void sanitize();

    Lsm6dsv::Config makeLsmConfig() const;
    Lsm6dsvFifoReader::Config makeFifoConfig() const;
    Ahrs6DofConfig makeAhrsConfig() const;
    ImuQualityConfig makeQualityConfig() const;

    void applyToImuCalibration(ImuCalibration& imuCal) const;
    void captureFromImuCalibration(const ImuCalibration& imuCal);
    void noteGyroBiasCalibrationCaptured(uint32_t uptimeMs);
    void captureFromAccelCalibrationQuality(const Accel6PosCalibration& cal, uint32_t uptimeMs);
    void captureFromMagCalibrationResult(const MagCalibrationResult& result,
                                         uint32_t sampleCount,
                                         uint32_t rejectedSamples,
                                         uint32_t saturatedSamples,
                                         float normMin,
                                         float normMean,
                                         float normMax,
                                         uint32_t uptimeMs);
    void applyToGyroTempComp(GyroTempCompensator& tempComp) const;
    void captureFromGyroTempComp(const GyroTempCompensator& tempComp);
};

// One-time schema-gated migration of historical performance defaults.
// sanitize() is called before return and updates schema versions/CRC.
void trackerMigratePerformanceDefaults(TrackerConfig& config);

} // namespace tracker
