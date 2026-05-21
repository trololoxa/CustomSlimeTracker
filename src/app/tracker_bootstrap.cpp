#include "app/tracker_bootstrap.hpp"

namespace tracker {

const char* trackerBootstrapLsmErrorName(Lsm6dsv::Error e) {
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

void trackerBootstrapEnforceProductCalibrationValidity(TrackerConfig& config) {
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

void trackerBootstrapMigrateRuntimeConfigForPerformance(TrackerConfig& config) {
    // Old NVS configs used the original conservative 1 MHz SPI default.
    // Upgrade that legacy value in RAM so the optimized build benefits
    // immediately, while still allowing explicit config save/revert commands.
    if (config.data.hardware.spiHz == tracker_config_detail::LEGACY_SPI_HZ) {
        config.data.hardware.spiHz = tracker_config_detail::DEFAULT_SPI_HZ;
    }

    // Early SlimeVR UDP builds used a 48-word FIFO watermark. It was stable,
    // but with the network loop outside FIFO batch processing it produced only
    // ~23 Hz effective RotationData because the prepared quaternion snapshot
    // changed once per FIFO drain. Runtime testing showed that 12 words keeps
    // the full firmware stable while producing ~98 Hz RotationData. Migrate
    // only the known legacy default; explicit user-tuned values are preserved.
    if (config.data.fifo.watermarkWords == cfg::LEGACY_FIFO_WATERMARK_WORDS) {
        config.data.fifo.watermarkWords = cfg::FIFO_WATERMARK_WORDS;
    }

    config.sanitize();
    config.updateCrc();
}

void trackerBootstrapApplySpiConfigToTransport(TrackerConfig& config,
                                                      ArduinoLsm6dsvSpiTransport& lsmBus) {
    config.sanitize();
    lsmBus.setSettings(config.data.hardware.spiHz, config.data.hardware.spiMode);
}

bool trackerBootstrapSetRuntimeSpiFrequency(TrackerConfig& config,
                                                   ArduinoLsm6dsvSpiTransport& lsmBus,
                                                   uint32_t hz) {
    config.data.hardware.spiHz = hz;
    config.sanitize();
    config.updateCrc();
    lsmBus.setSettings(config.data.hardware.spiHz, config.data.hardware.spiMode);
    return true;
}

bool trackerBootstrapLoadConfigAndApplyRuntime(const TrackerBootstrapDeps& deps) {
    deps.configStore->loadOrDefaults(*deps.config, deps.configLoadedFromNvs);
    deps.config->sanitize();
    trackerBootstrapMigrateRuntimeConfigForPerformance(*deps.config);
    trackerBootstrapEnforceProductCalibrationValidity(*deps.config);

    deps.config->applyToImuCalibration(*deps.imuCal);
    deps.config->applyToGyroTempComp(*deps.gyroTempComp);

    if (deps.runtimeBias) {
        const bool requested = (deps.config->data.ahrsRuntime.reserved &
                                tracker_config_detail::AHRS_RUNTIME_FLAG_RUNTIME_BIAS_ENABLED) != 0;
        const bool baseReady = deps.imuCal->gyroBiasValid &&
                               deps.imuCal->accelCalValid &&
                               deps.gyroTempComp->valid() &&
                               deps.config->data.gyroCal.tempCompEnabled &&
                               deps.config->data.gyroTempQuality.fitQuality > 0.0f;
        deps.runtimeBias->runtimeTrimRadS = Vec3::zero();
        deps.runtimeBias->resetCounters();
        deps.runtimeBias->enabled = requested && baseReady;
    }

    deps.ahrs->setConfig(deps.config->makeAhrsConfig());

    deps.quality->setConfig(deps.config->makeQualityConfig());
    deps.quality->reset();

#if TRACKER_HAS_SERIAL_STREAM_STATE
    if (deps.streamState != nullptr) {
        deps.streamState->rateHz = deps.config->data.output.outputRateHz;
        deps.streamState->mode = deps.config->data.output.quaternionOutputEnabled
            ? TrackerStreamMode::Quat
            : TrackerStreamMode::Off;
    }
#endif
    return true;
}

bool trackerBootstrapInitLsm(const TrackerBootstrapDeps& deps) {
    deps.spi->begin(deps.pins.sck, deps.pins.miso, deps.pins.mosi, deps.pins.cs);
    deps.lsmBus->begin();
    trackerBootstrapApplySpiConfigToTransport(*deps.config, *deps.lsmBus);

    Lsm6dsv::Config cfg = deps.config->makeLsmConfig();

    if (!deps.lsm->begin(cfg)) {
#if TRACKER_HAS_SERIAL_CONSOLE
        deps.out->print("# ERR LSM6DSV init failed error=");
        deps.out->print(trackerBootstrapLsmErrorName(deps.lsm->lastError()));
        deps.out->print(" who=0x");
        deps.out->println(deps.lsm->lastWhoAmI(), HEX);
#endif
        return false;
    }

#if TRACKER_HAS_SERIAL_CONSOLE
    uint8_t who = 0;
    deps.lsm->readWhoAmI(who);

    deps.out->println("# OK LSM6DSV init");
    deps.out->print("# WHO_AM_I=0x"); deps.out->println(who, HEX);
    deps.out->print("# SPI_Hz="); deps.out->println(deps.lsmBus->spiHz());
    deps.out->print("# ODR_Hz="); deps.out->println(Lsm6dsv::odrHz(deps.config->data.imu.imuOdr), 3);
#endif
    return true;
}

bool trackerBootstrapInitFifo(const TrackerBootstrapDeps& deps) {
    Lsm6dsvFifoReader::Config fifoCfg = deps.config->makeFifoConfig();
    fifoCfg.enableSensorHubSlave0 = deps.config->data.magCal.driverEnabled;
    fifoCfg.sensorHubSlave0PeriodUs = deps.config->data.magCal.driverEnabled ? deps.magHubPeriodUs : 0.0f;

    if (!deps.fifo->configure(fifoCfg)) {
#if TRACKER_HAS_SERIAL_CONSOLE
        deps.out->println("# ERR FIFO configure failed");
#endif
        return false;
    }

#if TRACKER_HAS_SERIAL_CONSOLE
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
#endif
    return true;
}

void trackerBootstrapSetupCalibrationIo(const TrackerBootstrapDeps& deps) {
#if TRACKER_HAS_CALIBRATION_UI
    if (deps.calibrationIo == nullptr) {
        return;
    }
    deps.calibrationIo->lsm = deps.lsm;
    deps.calibrationIo->fifo = deps.fifo;
    deps.calibrationIo->rawBuffer = deps.calibrationRawBuffer;
    deps.calibrationIo->rawBufferCapacity = deps.calibrationRawBufferCapacity;
    deps.calibrationIo->maxWordsPerDrain = deps.config->data.fifo.maxWordsPerDrain;
    deps.calibrationIo->maxDrainRoundsPerEvent = deps.config->data.fifo.maxDrainRoundsPerEvent;
    deps.calibrationIo->waitForFifoEvent = deps.waitForCalibrationFifoEvent;
    deps.calibrationIo->waitUser = deps.waitForCalibrationFifoEventUser;
    deps.calibrationIo->latestTempC = deps.latestTempC;
#else
    (void)deps;
#endif
}

} // namespace tracker
