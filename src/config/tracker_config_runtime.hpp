#pragma once

#include <Arduino.h>
#include <cstdint>

#include "config/tracker_config_schema.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/imu_quality.hpp"

namespace tracker {

// Only for deployed slot v3 copies. Calibration bytes are never repaired.
class TrackerConfig;
bool trackerNormalizeDeployedV3(TrackerConfig& config);

enum class TrackerSemanticConfigError : uint8_t {
    None = 0,
    Schema,
    Hardware,
    ImuEnum,
    FifoEnum,
    ImuFifoMismatch,
    GyroCalibration,
    TemperatureModel,
    AccelCalibration,
    MagCalibration,
    FrameTransform,
    MagYawGates,
    AhrsGates,
    QualityGates,
    Output,
    Dependency,
    ReservedState,
};

const char* trackerSemanticConfigErrorName(TrackerSemanticConfigError error);

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
    // Strict runtime/candidate validation. Unlike sanitize(), this never edits
    // the candidate and therefore cannot silently turn rejected input into a
    // different accepted configuration.
    bool validateSemanticConfig(TrackerSemanticConfigError* error = nullptr) const;
    uint32_t computeCrc() const;
    void updateCrc();
    void sanitize();

    SlimeVRMotionPacketPolicy slimevrMotionPacketPolicy() const;
    void setSlimeVRMotionPacketPolicy(SlimeVRMotionPacketPolicy policy);

    Lsm6dsv::Config makeLsmConfig() const;
    Lsm6dsvFifoReader::Config makeFifoConfig() const;
    Ahrs6DofConfig makeAhrsConfig() const;
    ImuQualityConfig makeQualityConfig() const;

    void applyToImuCalibration(ImuCalibration& imuCal) const;
    void captureGyroFromImuCalibration(const ImuCalibration& imuCal);
    void captureAccelFromImuCalibration(const ImuCalibration& imuCal);
    void captureFromImuCalibration(const ImuCalibration& imuCal);
    void noteGyroBiasCalibrationCaptured(uint32_t uptimeMs);
    // Snapshot helpers copy the current runtime calibration/evidence without
    // manufacturing a new calibration event. Update helpers are reserved for
    // commands that actually computed or replaced a calibration model.
    void captureFromAccelCalibrationQuality(const Accel6PosCalibration& cal, uint32_t uptimeMs);
    void clearGyroCalibrationPreservingPolicy();
    void clearAccelCalibration();
    void clearMagCalibrationPreservingDriver();
    void clearAllCalibrationPreservingPolicy();

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
    void captureFromGyroTempCompUpdate(const GyroTempCompensator& tempComp, uint32_t uptimeMs, uint32_t sampleCount = 0);
};

// One-time value-gated migration of historical performance defaults. This
// changes only those exact defaults and updates the CRC; it is not a general
// candidate repair path.
void trackerMigratePerformanceDefaults(TrackerConfig& config);

} // namespace tracker
