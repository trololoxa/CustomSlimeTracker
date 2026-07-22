#include "connection/lsm6dsv_driver.hpp"

namespace tracker {

Lsm6dsv::Lsm6dsv(Lsm6dsvTransport& transport) : bus_(transport) {}

bool Lsm6dsv::begin() {
        Config config;
        return begin(config);
    }

bool Lsm6dsv::begin(const Config& config) {
        cfg_ = config;

        uint8_t who = 0;
        if (!readWhoAmI(who)) {
            return fail(Error::BusReadFailed);
        }
        if (who != WHO_AM_I_EXPECTED) {
            lastWhoAmI_ = who;
            return fail(Error::WrongWhoAmI);
        }

        if (cfg_.doSoftwareReset) {
            if (!softwareReset()) {
                return false;
            }
        }

        // Return to main register page in case previous experiments enabled embedded pages.
        if (!writeReg(Reg::FUNC_CFG_ACCESS, 0x00)) {
            return fail(Error::BusWriteFailed);
        }

        if (cfg_.disableI2cAndI3c) {
            // 4-wire SPI, I2C/I3C disabled, interrupt push-pull active-high.
            if (!writeReg(Reg::IF_CFG, IF_CFG_I2C_I3C_DISABLE)) {
                return fail(Error::BusWriteFailed);
            }
        } else {
            // Keep 4-wire SPI mode and do not touch pull-ups unless requested later.
            if (!writeReg(Reg::IF_CFG, 0x00)) {
                return fail(Error::BusWriteFailed);
            }
        }

        if (!configureCtrl3()) {
            return false;
        }

        if (!configureCtrl4()) {
            return false;
        }

        if (!disableFifo()) {
            return false;
        }

        if (!configureFiltersAndFullScale()) {
            return false;
        }

        if (!configureInterrupts()) {
            return false;
        }

        if (!setOdrAndModes(cfg_.accelOdr, cfg_.gyroOdr, cfg_.accelMode, cfg_.gyroMode)) {
            return false;
        }

        bus_.delayMs(10);

        initialized_ = true;
        return true;
    }

bool Lsm6dsv::isInitialized() const {
        return initialized_;
    }

uint8_t Lsm6dsv::lastWhoAmI() const {
        return lastWhoAmI_;
    }

Lsm6dsv::Error Lsm6dsv::lastError() const {
        return lastError_;
    }

bool Lsm6dsv::readWhoAmI(uint8_t& out) {
        if (!readReg(Reg::WHO_AM_I, out)) {
            return false;
        }
        lastWhoAmI_ = out;
        return true;
    }

bool Lsm6dsv::softwareReset() {
        if (!writeReg(Reg::CTRL1, 0x00)) return fail(Error::BusWriteFailed);
        if (!writeReg(Reg::CTRL2, 0x00)) return fail(Error::BusWriteFailed);

        if (!writeReg(Reg::CTRL3, CTRL3_SW_RESET)) {
            return fail(Error::BusWriteFailed);
        }

        for (int i = 0; i < 100; ++i) {
            uint8_t ctrl3 = 0;
            if (!readReg(Reg::CTRL3, ctrl3)) {
                return fail(Error::BusReadFailed);
            }
            if ((ctrl3 & CTRL3_SW_RESET) == 0) {
                bus_.delayMs(2);
                return true;
            }
            bus_.delayMs(1);
        }

        return fail(Error::ResetTimeout);
    }

bool Lsm6dsv::readStatus(Status& status) {
        uint8_t raw = 0;
        if (!readReg(Reg::STATUS_REG, raw)) {
            return fail(Error::BusReadFailed);
        }

        status.raw = raw;
        status.accelDataReady = (raw & STATUS_XLDA) != 0;
        status.gyroDataReady = (raw & STATUS_GDA) != 0;
        status.tempDataReady = (raw & STATUS_TDA) != 0;
        status.gyroEisDataReady = (raw & STATUS_GDA_EIS) != 0;
        status.oisDataReady = (raw & STATUS_OIS_DRDY) != 0;
        status.timestampEndcount = (raw & STATUS_TIMESTAMP_ENDCOUNT) != 0;
        return true;
    }

bool Lsm6dsv::hasNewImuData(bool requireAccel, bool requireGyro) {
        Status s;
        if (!readStatus(s)) {
            return false;
        }
        const bool accelOk = !requireAccel || s.accelDataReady;
        const bool gyroOk = !requireGyro || s.gyroDataReady;
        return accelOk && gyroOk;
    }

bool Lsm6dsv::readRawSample(RawSample& out, uint64_t t_us, bool readStatusFirst) {
        uint16_t flags = FLAG_NONE;
        uint8_t statusRaw = 0;

        if (readStatusFirst) {
            Status s;
            if (!readStatus(s)) {
                return false;
            }
            statusRaw = s.raw;
            flags |= FLAG_READ_STATUS_OK;
            if (s.accelDataReady) flags |= FLAG_STATUS_XLDA;
            if (s.gyroDataReady)  flags |= FLAG_STATUS_GDA;
            if (s.tempDataReady)  flags |= FLAG_STATUS_TDA;
        }

        uint8_t b[14] = {};
        if (!read(Reg::OUT_TEMP_L, b, sizeof(b))) {
            return fail(Error::BusReadFailed);
        }
        flags |= FLAG_READ_OUTPUT_OK;

        out.t_us = t_us;
        out.temp = le16(&b[0]);
        out.gx = le16(&b[2]);
        out.gy = le16(&b[4]);
        out.gz = le16(&b[6]);
        out.ax = le16(&b[8]);
        out.ay = le16(&b[10]);
        out.az = le16(&b[12]);
        out.statusRaw = statusRaw;
        out.components = SAMPLE_COMPONENT_COMPLETE;
        out.coherency = SampleCoherency::Coherent;

        if (isRawSaturated(out.ax) || isRawSaturated(out.ay) || isRawSaturated(out.az)) {
            flags |= FLAG_ACCEL_SATURATED;
        }
        if (isRawSaturated(out.gx) || isRawSaturated(out.gy) || isRawSaturated(out.gz)) {
            flags |= FLAG_GYRO_SATURATED;
        }

        out.flags = flags;
        return true;
    }

bool Lsm6dsv::readSample(Sample& out, uint64_t t_us, bool readStatusFirst) {
        RawSample raw;
        if (!readRawSample(raw, t_us, readStatusFirst)) {
            return false;
        }
        out = scale(raw);
        return true;
    }

Lsm6dsv::Sample Lsm6dsv::scale(const RawSample& raw) const {
        Sample s;
        s.t_us = raw.t_us;
        s.statusRaw = raw.statusRaw;
        s.flags = raw.flags;

        const float gyroRadPerLSB = gyroSensitivityRadPerSecPerLSB(cfg_.gyroFs);
        const float accelGPerLSB = accelSensitivityGPerLSB(cfg_.accelFs);

        s.gyro_rad_s = Vec3(
            static_cast<float>(raw.gx) * gyroRadPerLSB,
            static_cast<float>(raw.gy) * gyroRadPerLSB,
            static_cast<float>(raw.gz) * gyroRadPerLSB
        );

        s.accel_g = Vec3(
            static_cast<float>(raw.ax) * accelGPerLSB,
            static_cast<float>(raw.ay) * accelGPerLSB,
            static_cast<float>(raw.az) * accelGPerLSB
        );

        s.temp_c = temperatureC(raw.temp);
        return s;
    }

bool Lsm6dsv::setAccelFullScale(AccelFs fs) {
        cfg_.accelFs = fs;
        return writeMasked(Reg::CTRL8, CTRL8_FS_XL_MASK, static_cast<uint8_t>(fs));
    }

bool Lsm6dsv::setGyroFullScale(GyroFs fs) {
        cfg_.gyroFs = fs;
        return writeMasked(Reg::CTRL6, CTRL6_FS_G_MASK, static_cast<uint8_t>(fs));
    }

bool Lsm6dsv::setAccelOdr(Odr odr, AccelMode mode) {
        cfg_.accelOdr = odr;
        cfg_.accelMode = mode;
        const uint8_t value =
            static_cast<uint8_t>((static_cast<uint8_t>(mode) & 0x07u) << 4) |
            static_cast<uint8_t>(static_cast<uint8_t>(odr) & 0x0Fu);
        return writeReg(Reg::CTRL1, value);
    }

bool Lsm6dsv::setGyroOdr(Odr odr, GyroMode mode) {
        if (odr == Odr::Hz1_875) {
            return fail(Error::InvalidConfig);
        }

        cfg_.gyroOdr = odr;
        cfg_.gyroMode = mode;
        const uint8_t value =
            static_cast<uint8_t>((static_cast<uint8_t>(mode) & 0x07u) << 4) |
            static_cast<uint8_t>(static_cast<uint8_t>(odr) & 0x0Fu);
        return writeReg(Reg::CTRL2, value);
    }

bool Lsm6dsv::powerDown() {
        bool ok = true;
        ok = writeReg(Reg::CTRL1, 0x00) && ok;
        ok = writeReg(Reg::CTRL2, 0x00) && ok;
        return ok;
    }

bool Lsm6dsv::disableFifo() {
        bool ok = true;
        ok = writeReg(Reg::FIFO_CTRL4, 0x00) && ok; // bypass mode
        ok = writeReg(Reg::FIFO_CTRL3, 0x00) && ok; // no accel/gyro batching
        ok = writeReg(Reg::FIFO_CTRL2, 0x00) && ok; // no compression, no stop-on-wtm
        ok = writeReg(Reg::FIFO_CTRL1, 0x00) && ok; // watermark 0
        if (!ok) return fail(Error::BusWriteFailed);
        return true;
    }

bool Lsm6dsv::readFifoUnreadCount(uint16_t& unreadFrames) {
        uint8_t b[2] = {};
        if (!read(Reg::FIFO_STATUS1, b, sizeof(b))) {
            return fail(Error::BusReadFailed);
        }
        unreadFrames = static_cast<uint16_t>(b[0]) |
                       static_cast<uint16_t>((b[1] & 0x01u) << 8);
        return true;
    }

bool Lsm6dsv::configureDrdyOnInt1(bool accel, bool gyro) {
        cfg_.int1DrdyAccel = accel;
        cfg_.int1DrdyGyro = gyro;
        uint8_t value = 0;
        if (gyro)  value |= INT_CTRL_DRDY_G;
        if (accel) value |= INT_CTRL_DRDY_XL;
        return writeReg(Reg::INT1_CTRL, value);
    }

bool Lsm6dsv::configureDrdyOnInt2(bool accel, bool gyro) {
        cfg_.int2DrdyAccel = accel;
        cfg_.int2DrdyGyro = gyro;
        uint8_t value = 0;
        if (gyro)  value |= INT_CTRL_DRDY_G;
        if (accel) value |= INT_CTRL_DRDY_XL;
        return writeReg(Reg::INT2_CTRL, value);
    }

bool Lsm6dsv::configureTapDetection(const TapConfig& config) {
        if (!config.enabled) {
            bool ok = true;
            ok = writeMasked(Reg::MD1_CFG, MD1_CFG_TAP_MASK, 0x00) && ok;
            ok = writeMasked(Reg::TAP_CFG0, TAP_CFG0_CONTROLLED_MASK, 0x00) && ok;
            ok = writeMasked(Reg::WAKE_UP_THS, WAKE_UP_THS_TAP_MASK, 0x00) && ok;
            if (!ok) return fail(lastError_ == Error::BusReadFailed ? Error::BusReadFailed : Error::BusWriteFailed);
            return true;
        }

        bool ok = true;
        const TapRegisterSnapshot expected = expectedTapRegistersFor(config);
        const TapRegisterSnapshot mask = tapRegisterMaskFor(config);

        // LSM6DSV basic interrupts (including tap/single-tap) are globally gated
        // by FUNCTIONS_ENABLE.INTERRUPTS_ENABLE. Preserve other bits here: FIFO
        // timestamp commonly uses the same register, so a direct write would be
        // a regression for the high-rate FIFO path.
        ok = writeMasked(Reg::FUNCTIONS_ENABLE, mask.functionsEnable, expected.functionsEnable) && ok;
        ok = writeMasked(Reg::TAP_CFG1, mask.tapCfg1, expected.tapCfg1) && ok;
        ok = writeMasked(Reg::TAP_CFG2, mask.tapCfg2, expected.tapCfg2) && ok;
        ok = writeMasked(Reg::TAP_THS_6D, mask.tapThs6d, expected.tapThs6d) && ok;
        ok = writeMasked(Reg::TAP_DUR, mask.tapDur, expected.tapDur) && ok;
        ok = writeMasked(Reg::WAKE_UP_THS, mask.wakeUpThs, expected.wakeUpThs) && ok;
        ok = writeMasked(Reg::TAP_CFG0, mask.tapCfg0, expected.tapCfg0) && ok;
        ok = writeMasked(Reg::MD1_CFG, mask.md1Cfg, expected.md1Cfg) && ok;
        if (!ok) return fail(lastError_ == Error::BusReadFailed ? Error::BusReadFailed : Error::BusWriteFailed);

        TapSource discard;
        (void)readTapSource(discard);
        return true;
    }

Lsm6dsv::TapRegisterSnapshot Lsm6dsv::expectedTapRegistersFor(const TapConfig& config) {
        TapRegisterSnapshot out;

        if (!config.enabled) {
            return out;
        }

        const uint8_t thsX = static_cast<uint8_t>(config.thresholdX & TAP_CFG2_THS_MASK);
        const uint8_t thsY = static_cast<uint8_t>(config.thresholdY & TAP_CFG2_THS_MASK);
        const uint8_t thsZ = static_cast<uint8_t>(config.thresholdZ & TAP_THS_6D_THS_MASK);
        const uint8_t priority = static_cast<uint8_t>((config.priority & 0x07u) << 5);

        out.functionsEnable = FUNCTIONS_ENABLE_INTERRUPTS_ENABLE;

        if (config.latchedInterrupt) out.tapCfg0 |= TAP_CFG0_LIR;
        if (config.enableZ) out.tapCfg0 |= TAP_CFG0_TAP_Z_EN;
        if (config.enableY) out.tapCfg0 |= TAP_CFG0_TAP_Y_EN;
        if (config.enableX) out.tapCfg0 |= TAP_CFG0_TAP_X_EN;
        if (config.maskDuringAccelSettling) out.tapCfg0 |= TAP_CFG0_HW_FUNC_MASK_XL_SETTL;

        out.tapCfg1 = static_cast<uint8_t>(priority | thsX);
        out.tapCfg2 = thsY;
        out.tapThs6d = thsZ;
        out.tapDur = static_cast<uint8_t>(((config.duration & 0x0Fu) << 4) |
                                          ((config.quiet & 0x03u) << 2) |
                                          (config.shock & 0x03u));
        out.wakeUpThs = static_cast<uint8_t>(config.enableDoubleTap ? WAKE_UP_THS_SINGLE_DOUBLE_TAP : 0u);
        if (config.routeSingleTapToInt1) out.md1Cfg |= MD1_CFG_INT1_SINGLE_TAP;
        if (config.routeDoubleTapToInt1) out.md1Cfg |= MD1_CFG_INT1_DOUBLE_TAP;
        return out;
    }

Lsm6dsv::TapRegisterSnapshot Lsm6dsv::tapRegisterMaskFor(const TapConfig& config) {
        TapRegisterSnapshot out;
        out.tapCfg0 = TAP_CFG0_CONTROLLED_MASK;
        out.wakeUpThs = WAKE_UP_THS_TAP_MASK;
        out.md1Cfg = MD1_CFG_TAP_MASK;
        if (config.enabled) {
            out.functionsEnable = FUNCTIONS_ENABLE_INTERRUPTS_ENABLE;
            out.tapCfg1 = TAP_CFG1_CONTROLLED_MASK;
            out.tapCfg2 = TAP_CFG2_THS_MASK;
            out.tapThs6d = TAP_THS_6D_THS_MASK;
            out.tapDur = TAP_DUR_CONTROLLED_MASK;
        }
        return out;
    }

bool Lsm6dsv::readTapConfigRegisters(TapRegisterSnapshot& snapshot) {
        if (!readReg(Reg::FUNCTIONS_ENABLE, snapshot.functionsEnable)) {
            return fail(Error::BusReadFailed);
        }

        uint8_t b[9] = {};
        if (!read(Reg::TAP_CFG0, b, sizeof(b))) {
            return fail(Error::BusReadFailed);
        }
        snapshot.tapCfg0 = b[0];
        snapshot.tapCfg1 = b[1];
        snapshot.tapCfg2 = b[2];
        snapshot.tapThs6d = b[3];
        snapshot.tapDur = b[4];
        snapshot.wakeUpThs = b[5];
        snapshot.md1Cfg = b[8];
        return true;
    }

bool Lsm6dsv::verifyTapDetection(const TapConfig& config, TapRegisterVerification& verification) {
        verification = TapRegisterVerification{};
        verification.expected = expectedTapRegistersFor(config);
        verification.mask = tapRegisterMaskFor(config);
        if (!readTapConfigRegisters(verification.actual)) {
            return false;
        }

        const auto maskedEq = [](uint8_t actual, uint8_t expected, uint8_t mask) {
            return (actual & mask) == (expected & mask);
        };

        verification.ok =
            maskedEq(verification.actual.functionsEnable, verification.expected.functionsEnable, verification.mask.functionsEnable) &&
            maskedEq(verification.actual.tapCfg0, verification.expected.tapCfg0, verification.mask.tapCfg0) &&
            maskedEq(verification.actual.tapCfg1, verification.expected.tapCfg1, verification.mask.tapCfg1) &&
            maskedEq(verification.actual.tapCfg2, verification.expected.tapCfg2, verification.mask.tapCfg2) &&
            maskedEq(verification.actual.tapThs6d, verification.expected.tapThs6d, verification.mask.tapThs6d) &&
            maskedEq(verification.actual.tapDur, verification.expected.tapDur, verification.mask.tapDur) &&
            maskedEq(verification.actual.wakeUpThs, verification.expected.wakeUpThs, verification.mask.wakeUpThs) &&
            maskedEq(verification.actual.md1Cfg, verification.expected.md1Cfg, verification.mask.md1Cfg);
        return true;
    }

bool Lsm6dsv::readTapSource(TapSource& source) {
        uint8_t raw = 0;
        if (!readReg(Reg::TAP_SRC, raw)) {
            return fail(Error::BusReadFailed);
        }
        source.raw = raw;
        source.tapDetected = (raw & TAP_SRC_TAP_IA) != 0;
        source.singleTap = (raw & TAP_SRC_SINGLE_TAP) != 0;
        source.doubleTap = (raw & TAP_SRC_DOUBLE_TAP) != 0;
        source.negative = (raw & TAP_SRC_TAP_SIGN) != 0;
        source.x = (raw & TAP_SRC_X_TAP) != 0;
        source.y = (raw & TAP_SRC_Y_TAP) != 0;
        source.z = (raw & TAP_SRC_Z_TAP) != 0;
        return true;
    }


bool Lsm6dsv::configureMotionWake(const MotionWakeConfig& config) {
    if (config.threshold > WAKE_UP_THS_MOTION_MASK ||
        config.duration > WAKE_UP_DUR_MOTION_MASK ||
        config.accelOdr == Odr::PowerDown) {
        return fail(Error::InvalidConfig);
    }

    if (!config.enabled) {
        bool ok = true;
        ok = writeMasked(Reg::MD1_CFG, MD1_CFG_INT1_WAKE_UP, 0x00) && ok;
        ok = writeMasked(Reg::TAP_CFG0,
                         static_cast<uint8_t>(TAP_CFG0_LIR | TAP_CFG0_SLOPE_FDS),
                         0x00) && ok;
        ok = writeMasked(Reg::WAKE_UP_THS, WAKE_UP_THS_MOTION_MASK, 0x00) && ok;
        ok = writeMasked(Reg::WAKE_UP_DUR, WAKE_UP_DUR_MOTION_MASK, 0x00) && ok;
        MotionWakeSource discard;
        (void)readMotionWakeSource(discard);
        if (!ok) return fail(lastError_ == Error::BusReadFailed ? Error::BusReadFailed : Error::BusWriteFailed);
        return true;
    }

    // INT1 is shared with the FIFO path in this tracker. Do not leave any
    // FIFO/DRDY/tap source armed while GPIO light-sleep wake is level-based.
    TapConfig tapsOff;
    tapsOff.enabled = false;

    bool ok = true;
    ok = disableFifo() && ok;
    ok = configureDrdyOnInt1(false, false) && ok;
    ok = configureTapDetection(tapsOff) && ok;
    ok = setAccelFullScale(config.accelFs) && ok;
    ok = setAccelOdr(config.accelOdr, config.accelMode) && ok;
    ok = setGyroOdr(Odr::PowerDown) && ok;
    ok = writeMasked(Reg::FUNCTIONS_ENABLE,
                     FUNCTIONS_ENABLE_INTERRUPTS_ENABLE,
                     FUNCTIONS_ENABLE_INTERRUPTS_ENABLE) && ok;

    // Use the high-pass wake-up path and set WU_INACT_THS_W=011 so the
    // configured threshold has the documented 62.5 mg/code resolution.
    uint8_t tapCfg0 = 0;
    if (config.latchedInterrupt) tapCfg0 |= TAP_CFG0_LIR;
    if (config.highPassFilter) tapCfg0 |= TAP_CFG0_SLOPE_FDS;
    if (config.maskDuringAccelSettling) tapCfg0 |= TAP_CFG0_HW_FUNC_MASK_XL_SETTL;
    ok = writeMasked(Reg::TAP_CFG0,
                     static_cast<uint8_t>(TAP_CFG0_LIR |
                                          TAP_CFG0_SLOPE_FDS |
                                          TAP_CFG0_HW_FUNC_MASK_XL_SETTL),
                     tapCfg0) && ok;
    ok = writeMasked(Reg::INACTIVITY_DUR,
                     INACTIVITY_DUR_WU_THS_WEIGHT_MASK,
                     INACTIVITY_DUR_WU_THS_WEIGHT_62_5MG) && ok;
    ok = writeMasked(Reg::WAKE_UP_THS,
                     WAKE_UP_THS_MOTION_MASK,
                     static_cast<uint8_t>(config.threshold & WAKE_UP_THS_MOTION_MASK)) && ok;
    ok = writeMasked(Reg::WAKE_UP_DUR,
                     WAKE_UP_DUR_MOTION_MASK,
                     static_cast<uint8_t>(config.duration & WAKE_UP_DUR_MOTION_MASK)) && ok;
    ok = writeMasked(Reg::MD1_CFG,
                     MD1_CFG_INT1_WAKE_UP,
                     config.routeToInt1 ? MD1_CFG_INT1_WAKE_UP : 0x00) && ok;

    if (!ok) return fail(lastError_ == Error::BusReadFailed ? Error::BusReadFailed : Error::BusWriteFailed);

    // Let the low-power accelerometer settle before its source/latch is used
    // by ESP32 GPIO wake, then clear any stale condition from the old runtime.
    bus_.delayMs(10);
    MotionWakeSource discard;
    return readMotionWakeSource(discard);
}

bool Lsm6dsv::readMotionWakeSource(MotionWakeSource& source) {
    uint8_t raw = 0;
    if (!readReg(Reg::WAKE_UP_SRC, raw)) {
        return fail(Error::BusReadFailed);
    }

    source.raw = raw;
    source.z = (raw & WAKE_UP_SRC_Z) != 0;
    source.y = (raw & WAKE_UP_SRC_Y) != 0;
    source.x = (raw & WAKE_UP_SRC_X) != 0;
    source.wakeUp = (raw & WAKE_UP_SRC_IA) != 0;
    return true;
}

float Lsm6dsv::odrHz(Odr odr) {
        switch (odr) {
            case Odr::PowerDown: return 0.0f;
            case Odr::Hz1_875:   return 1.875f;
            case Odr::Hz7_5:     return 7.5f;
            case Odr::Hz15:      return 15.0f;
            case Odr::Hz30:      return 30.0f;
            case Odr::Hz60:      return 60.0f;
            case Odr::Hz120:     return 120.0f;
            case Odr::Hz240:     return 240.0f;
            case Odr::Hz480:     return 480.0f;
            case Odr::Hz960:     return 960.0f;
            case Odr::Hz1920:    return 1920.0f;
            case Odr::Hz3840:    return 3840.0f;
            case Odr::Hz7680:    return 7680.0f;
        }
        return 0.0f;
    }

float Lsm6dsv::accelSensitivityGPerLSB(AccelFs fs) {
        switch (fs) {
            case AccelFs::G2:  return 0.061f * 1.0e-3f;
            case AccelFs::G4:  return 0.122f * 1.0e-3f;
            case AccelFs::G8:  return 0.244f * 1.0e-3f;
            case AccelFs::G16: return 0.488f * 1.0e-3f;
        }
        return 0.0f;
    }

float Lsm6dsv::gyroSensitivityDpsPerLSB(GyroFs fs) {
        switch (fs) {
            case GyroFs::Dps125:  return 4.375f * 1.0e-3f;
            case GyroFs::Dps250:  return 8.75f  * 1.0e-3f;
            case GyroFs::Dps500:  return 17.50f * 1.0e-3f;
            case GyroFs::Dps1000: return 35.0f  * 1.0e-3f;
            case GyroFs::Dps2000: return 70.0f  * 1.0e-3f;
            case GyroFs::Dps4000: return 140.0f * 1.0e-3f;
        }
        return 0.0f;
    }

float Lsm6dsv::gyroSensitivityRadPerSecPerLSB(GyroFs fs) {
        return gyroSensitivityDpsPerLSB(fs) * MATH_DEG_TO_RAD;
    }

float Lsm6dsv::temperatureC(int16_t rawTemp) {
        return 25.0f + static_cast<float>(rawTemp) / 256.0f;
    }

bool Lsm6dsv::configureCtrl3() {
        uint8_t value = 0;
        if (cfg_.autoIncrement) {
            value |= CTRL3_IF_INC;
        }
        if (cfg_.blockDataUpdate) {
            value |= CTRL3_BDU;
        }
        if (!writeReg(Reg::CTRL3, value)) {
            return fail(Error::BusWriteFailed);
        }
        return true;
    }

bool Lsm6dsv::configureCtrl4() {
        uint8_t value = 0;

        if (cfg_.drdyPulsed) {
            value |= CTRL4_DRDY_PULSED;
        }

        if (!writeReg(Reg::CTRL4, value)) {
            return fail(Error::BusWriteFailed);
        }

        return true;
    }

bool Lsm6dsv::configureFiltersAndFullScale() {
        uint8_t ctrl6 = static_cast<uint8_t>(cfg_.gyroFs) & CTRL6_FS_G_MASK;
        ctrl6 |= static_cast<uint8_t>((cfg_.gyroLpf1BandwidthBits & 0x07u) << 4);
        if (!writeReg(Reg::CTRL6, ctrl6)) {
            return fail(Error::BusWriteFailed);
        }

        uint8_t ctrl7 = 0;
        if (cfg_.gyroLpf1Enable) {
            ctrl7 |= CTRL7_LPF1_G_EN;
        }
        if (!writeReg(Reg::CTRL7, ctrl7)) {
            return fail(Error::BusWriteFailed);
        }

        const uint8_t ctrl8 = static_cast<uint8_t>(cfg_.accelFs) & CTRL8_FS_XL_MASK;
        if (!writeReg(Reg::CTRL8, ctrl8)) {
            return fail(Error::BusWriteFailed);
        }

        // CTRL9: keep accel LPF2/HPF disabled for now.
        if (!writeReg(Reg::CTRL9, 0x00)) {
            return fail(Error::BusWriteFailed);
        }

        return true;
    }

bool Lsm6dsv::configureInterrupts() {
        if (!configureDrdyOnInt1(cfg_.int1DrdyAccel, cfg_.int1DrdyGyro)) {
            return false;
        }
        if (!configureDrdyOnInt2(cfg_.int2DrdyAccel, cfg_.int2DrdyGyro)) {
            return false;
        }
        return true;
    }

bool Lsm6dsv::setOdrAndModes(Odr accelOdr, Odr gyroOdr, AccelMode accelMode, GyroMode gyroMode) {
        if (!setAccelOdr(accelOdr, accelMode)) {
            return false;
        }
        if (!setGyroOdr(gyroOdr, gyroMode)) {
            return false;
        }
        return true;
    }

bool Lsm6dsv::writeMasked(Reg reg, uint8_t mask, uint8_t value) {
        uint8_t oldValue = 0;
        if (!readReg(reg, oldValue)) {
            return fail(Error::BusReadFailed);
        }

        const uint8_t newValue = static_cast<uint8_t>((oldValue & ~mask) | (value & mask));
        if (!writeReg(reg, newValue)) {
            return fail(Error::BusWriteFailed);
        }
        return true;
    }

bool Lsm6dsv::read(Reg reg, uint8_t* dst, size_t len) {
        return bus_.read(static_cast<uint8_t>(reg), dst, len);
    }

bool Lsm6dsv::write(Reg reg, const uint8_t* src, size_t len) {
        return bus_.write(static_cast<uint8_t>(reg), src, len);
    }

bool Lsm6dsv::readReg(Reg reg, uint8_t& value) {
        return bus_.readReg(static_cast<uint8_t>(reg), value);
    }

bool Lsm6dsv::writeReg(Reg reg, uint8_t value) {
        return bus_.writeReg(static_cast<uint8_t>(reg), value);
    }

int16_t Lsm6dsv::le16(const uint8_t* p) {
        return static_cast<int16_t>(
            static_cast<uint16_t>(p[0]) |
            static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8)
        );
    }

bool Lsm6dsv::isRawSaturated(int16_t v) {
        return v == INT16_MAX || v == INT16_MIN;
    }

bool Lsm6dsv::fail(Error e) {
        lastError_ = e;
        return false;
    }

} // namespace tracker
