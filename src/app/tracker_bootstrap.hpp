#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "defines.h"
#include "config/tracker_config.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/calibration.hpp"
#include "sensor/fifo_calibrations.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"
#include "serial/tracker_serial_commands.hpp"

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
    ImuQualityMonitor* quality = nullptr;
    Ahrs6Dof* ahrs = nullptr;
    TrackerSerialStreamState* streamState = nullptr;

    FifoCalibrationIo* calibrationIo = nullptr;
    Lsm6dsv::RawSample* calibrationRawBuffer = nullptr;
    size_t calibrationRawBufferCapacity = 0;
    bool (*waitForCalibrationFifoEvent)(uint32_t timeoutMs, void* user) = nullptr;
    void* waitForCalibrationFifoEventUser = nullptr;
    float latestTempC = 25.0f;

    TrackerBootstrapPins pins;
    float magHubPeriodUs = cfg::MAG_HUB_PERIOD_US;
};

inline const char* trackerBootstrapLsmErrorName(Lsm6dsv::Error e) {
    switch (e) {
        case Lsm6dsv::Error::None:           return "None";
        case Lsm6dsv::Error::BusReadFailed:  return "BusReadFailed";
        case Lsm6dsv::Error::BusWriteFailed: return "BusWriteFailed";
        case Lsm6dsv::Error::WrongWhoAmI:    return "WrongWhoAmI";
        case Lsm6dsv::Error::ResetTimeout:   return "ResetTimeout";
        case Lsm6dsv::Error::InvalidConfig:  return "InvalidConfig";
    }
    return "Unknown";
}

inline void trackerBootstrapEnforceProductCalibrationValidity(TrackerConfig& config) {
    // Production rule: never seed a universal accel calibration into a new
    // device. A missing accel calibration must remain explicit so mag yaw
    // correction cannot silently run with another device's tilt calibration.
    if (!config.data.accelCal.valid) {
        config.data.accelCal.biasG = Vec3::zero();
        config.data.accelCal.scale = Mat3::identity();
        config.data.magYaw.applyEnabled = false;
    }

    if (!config.data.accelCal.valid || !config.data.magCal.calibrationValid) {
        config.data.magYaw.applyEnabled = false;
    }

    config.updateCrc();
}

inline void trackerBootstrapMigrateRuntimeConfigForPerformance(TrackerConfig& config) {
    // Old NVS configs used the original conservative 1 MHz SPI default.
    // Upgrade that legacy value in RAM so the optimized build benefits
    // immediately, while still allowing explicit config save/revert commands.
    if (config.data.hardware.spiHz == tracker_config_detail::LEGACY_SPI_HZ) {
        config.data.hardware.spiHz = tracker_config_detail::DEFAULT_SPI_HZ;
    }

    config.sanitize();
    config.updateCrc();
}

inline void trackerBootstrapApplySpiConfigToTransport(TrackerConfig& config,
                                                      ArduinoLsm6dsvSpiTransport& lsmBus) {
    config.sanitize();
    lsmBus.setSettings(config.data.hardware.spiHz, config.data.hardware.spiMode);
}

inline bool trackerBootstrapSetRuntimeSpiFrequency(TrackerConfig& config,
                                                   ArduinoLsm6dsvSpiTransport& lsmBus,
                                                   uint32_t hz) {
    config.data.hardware.spiHz = hz;
    config.sanitize();
    config.updateCrc();
    lsmBus.setSettings(config.data.hardware.spiHz, config.data.hardware.spiMode);
    return true;
}

inline bool trackerBootstrapLoadConfigAndApplyRuntime(const TrackerBootstrapDeps& deps) {
    deps.configStore->loadOrDefaults(*deps.config, deps.configLoadedFromNvs);
    deps.config->sanitize();
    trackerBootstrapMigrateRuntimeConfigForPerformance(*deps.config);
    trackerBootstrapEnforceProductCalibrationValidity(*deps.config);

    deps.config->applyToImuCalibration(*deps.imuCal);
    deps.config->applyToGyroTempComp(*deps.gyroTempComp);
    deps.ahrs->setConfig(deps.config->makeAhrsConfig());

    deps.quality->setConfig(deps.config->makeQualityConfig());
    deps.quality->reset();

    deps.streamState->rateHz = deps.config->data.output.outputRateHz;
    deps.streamState->mode = deps.config->data.output.quaternionOutputEnabled
        ? TrackerStreamMode::Quat
        : TrackerStreamMode::Off;
    return true;
}

inline bool trackerBootstrapInitLsm(const TrackerBootstrapDeps& deps) {
    deps.spi->begin(deps.pins.sck, deps.pins.miso, deps.pins.mosi, deps.pins.cs);
    deps.lsmBus->begin();
    trackerBootstrapApplySpiConfigToTransport(*deps.config, *deps.lsmBus);

    Lsm6dsv::Config cfg = deps.config->makeLsmConfig();

    if (!deps.lsm->begin(cfg)) {
        deps.out->print("# ERR LSM6DSV init failed error=");
        deps.out->print(trackerBootstrapLsmErrorName(deps.lsm->lastError()));
        deps.out->print(" who=0x");
        deps.out->println(deps.lsm->lastWhoAmI(), HEX);
        return false;
    }

    uint8_t who = 0;
    deps.lsm->readWhoAmI(who);

    deps.out->println("# OK LSM6DSV init");
    deps.out->print("# WHO_AM_I=0x"); deps.out->println(who, HEX);
    deps.out->print("# SPI_Hz="); deps.out->println(deps.lsmBus->spiHz());
    deps.out->print("# ODR_Hz="); deps.out->println(Lsm6dsv::odrHz(deps.config->data.imu.imuOdr), 3);
    return true;
}

inline bool trackerBootstrapInitFifo(const TrackerBootstrapDeps& deps) {
    Lsm6dsvFifoReader::Config fifoCfg = deps.config->makeFifoConfig();
    fifoCfg.enableSensorHubSlave0 = deps.config->data.magCal.driverEnabled;
    fifoCfg.sensorHubSlave0PeriodUs = deps.config->data.magCal.driverEnabled ? deps.magHubPeriodUs : 0.0f;

    if (!deps.fifo->configure(fifoCfg)) {
        deps.out->println("# ERR FIFO configure failed");
        return false;
    }

    Lsm6dsvFifoReader::Status st;
    deps.fifo->readStatus(st);

    const auto& fs = deps.fifo->stats();
    deps.out->println("# OK FIFO v2 init");
    deps.out->print("# fifo_watermark_words="); deps.out->println(deps.config->data.fifo.watermarkWords);
    deps.out->print("# fifo_initial_unread_words="); deps.out->println(st.unreadWords);
    deps.out->print("# timestamp_tick_us="); deps.out->println(fs.timestampTickUs, 6);
    deps.out->print("# sample_period_us="); deps.out->println(fs.samplePeriodUs, 3);
    deps.out->print("# mag_fifo_parser="); deps.out->println(fifoCfg.enableSensorHubSlave0 ? "on" : "off");
    deps.out->print("# internal_freq_fine="); deps.out->println(fs.internalFreqFine);
    return true;
}

inline void trackerBootstrapSetupCalibrationIo(const TrackerBootstrapDeps& deps) {
    deps.calibrationIo->lsm = deps.lsm;
    deps.calibrationIo->fifo = deps.fifo;
    deps.calibrationIo->rawBuffer = deps.calibrationRawBuffer;
    deps.calibrationIo->rawBufferCapacity = deps.calibrationRawBufferCapacity;
    deps.calibrationIo->maxWordsPerDrain = deps.config->data.fifo.maxWordsPerDrain;
    deps.calibrationIo->maxDrainRoundsPerEvent = deps.config->data.fifo.maxDrainRoundsPerEvent;
    deps.calibrationIo->waitForFifoEvent = deps.waitForCalibrationFifoEvent;
    deps.calibrationIo->waitUser = deps.waitForCalibrationFifoEventUser;
    deps.calibrationIo->latestTempC = deps.latestTempC;
}

} // namespace tracker
