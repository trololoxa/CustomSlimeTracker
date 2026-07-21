#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "defines.h"
#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/calibration.hpp"
#if TRACKER_HAS_CALIBRATION_UI
#include "sensor/fifo_calibrations.hpp"
#endif
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"
#include "runtime/runtime_bias_types.hpp"
#if TRACKER_HAS_SERIAL_STREAM_STATE
#include "serial/tracker_serial_context.hpp"
#endif

namespace tracker {

struct TrackerBootstrapPins {
    int sck = cfg::PIN_LSM_SCK;
    int miso = cfg::PIN_LSM_MISO;
    int mosi = cfg::PIN_LSM_MOSI;
    int cs = cfg::PIN_LSM_CS;
    int int1 = cfg::PIN_LSM_INT1;
};

struct TrackerBootstrapDeps {
    Stream* out = nullptr;
    SPIClass* spi = nullptr;

    ArduinoLsm6dsvSpiTransport* lsmBus = nullptr;
    Lsm6dsv* lsm = nullptr;
    Lsm6dsvFifoReader* fifo = nullptr;

    TrackerConfig* config = nullptr;
    TrackerConfigStore* configStore = nullptr;
    bool* configLoadedFromNvs = nullptr;

    ImuCalibration* imuCal = nullptr;
    GyroTempCompensator* gyroTempComp = nullptr;
    RuntimeGyroBiasEstimator* runtimeBias = nullptr;
    ImuQualityMonitor* quality = nullptr;
    Ahrs6Dof* ahrs = nullptr;
#if TRACKER_HAS_SERIAL_STREAM_STATE
    TrackerSerialStreamState* streamState = nullptr;
#endif

#if TRACKER_HAS_CALIBRATION_UI
    FifoCalibrationIo* calibrationIo = nullptr;
    Lsm6dsv::RawSample* calibrationRawBuffer = nullptr;
    size_t calibrationRawBufferCapacity = 0;
    bool (*waitForCalibrationFifoEvent)(uint32_t timeoutMs, void* user) = nullptr;
    void* waitForCalibrationFifoEventUser = nullptr;
#endif
    float latestTempC = 25.0f;

    TrackerBootstrapPins pins;
    float magHubPeriodUs = cfg::MAG_HUB_PERIOD_US;
};

const char* trackerBootstrapLsmErrorName(Lsm6dsv::Error e);

void trackerBootstrapEnforceProductCalibrationValidity(TrackerConfig& config);
void trackerBootstrapApplySpiConfigToTransport(TrackerConfig& config,
                                               ArduinoLsm6dsvSpiTransport& lsmBus);
bool trackerBootstrapSetRuntimeSpiFrequency(TrackerConfig& config,
                                            ArduinoLsm6dsvSpiTransport& lsmBus,
                                            uint32_t hz);
bool trackerBootstrapLoadConfigAndApplyRuntime(const TrackerBootstrapDeps& deps);
bool trackerBootstrapInitLsm(const TrackerBootstrapDeps& deps);
bool trackerBootstrapInitFifo(const TrackerBootstrapDeps& deps);
void trackerBootstrapSetupCalibrationIo(const TrackerBootstrapDeps& deps);

} // namespace tracker
