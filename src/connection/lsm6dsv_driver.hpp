#pragma once

#include <cstddef>
#include <cstdint>
#include <cmath>
#include "core/math.hpp"

#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>
#endif

namespace tracker {

// ============================================================
// LSM6DSV driver conventions
// ============================================================
// - Primary interface: SPI 4-wire.
// - Register access: 7-bit address, read command = 0x80 | reg.
// - Output units after scaling:
//     gyro_rad_s : rad/s
//     accel_g    : g
//     temp_c     : deg C
// - This driver intentionally does not contain AHRS logic.
// - FIFO parsing is not implemented yet; direct output-register reads first.
// ============================================================

class Lsm6dsvTransport {
public:
    virtual ~Lsm6dsvTransport() = default;

    virtual bool read(uint8_t reg, uint8_t* dst, size_t len) = 0;
    virtual bool write(uint8_t reg, const uint8_t* src, size_t len) = 0;
    virtual void delayMs(uint32_t ms) = 0;

    bool readReg(uint8_t reg, uint8_t& value) {
        return read(reg, &value, 1);
    }

    bool writeReg(uint8_t reg, uint8_t value) {
        return write(reg, &value, 1);
    }
};

#ifdef ARDUINO

class ArduinoLsm6dsvSpiTransport final : public Lsm6dsvTransport {
public:
    ArduinoLsm6dsvSpiTransport(SPIClass& spi,
                               int csPin,
                               uint32_t spiHz = 1000000,
                               uint8_t spiMode = SPI_MODE0)
        : spi_(spi),
          csPin_(csPin),
          spiHz_(spiHz),
          spiMode_(spiMode),
          settings_(spiHz_, MSBFIRST, spiMode_) {}

    void begin() {
        pinMode(csPin_, OUTPUT);
        digitalWrite(csPin_, HIGH);
    }

    void setSettings(uint32_t spiHz, uint8_t spiMode) {
        if (spiHz == 0) {
            return;
        }
        spiHz_ = spiHz;
        spiMode_ = spiMode;
        settings_ = SPISettings(spiHz_, MSBFIRST, spiMode_);
    }

    uint32_t spiHz() const { return spiHz_; }
    uint8_t spiMode() const { return spiMode_; }

    bool read(uint8_t reg, uint8_t* dst, size_t len) override {
        if (dst == nullptr || len == 0) {
            return false;
        }

        spi_.beginTransaction(settings_);
        digitalWrite(csPin_, LOW);

        spi_.transfer(static_cast<uint8_t>(0x80u | (reg & 0x7Fu)));
#if defined(ARDUINO_ARCH_ESP32)
        static uint8_t zeros[32] = {};
        if (len <= sizeof(zeros)) {
            spi_.transferBytes(zeros, dst, len);
        } else
#endif
        {
            for (size_t i = 0; i < len; ++i) {
                dst[i] = spi_.transfer(0x00);
            }
        }

        digitalWrite(csPin_, HIGH);
        spi_.endTransaction();
        return true;
    }

    bool write(uint8_t reg, const uint8_t* src, size_t len) override {
        if (src == nullptr || len == 0) {
            return false;
        }

        spi_.beginTransaction(settings_);
        digitalWrite(csPin_, LOW);

        spi_.transfer(static_cast<uint8_t>(reg & 0x7Fu));
#if defined(ARDUINO_ARCH_ESP32)
        spi_.transferBytes(src, nullptr, len);
#else
        for (size_t i = 0; i < len; ++i) {
            spi_.transfer(src[i]);
        }
#endif

        digitalWrite(csPin_, HIGH);
        spi_.endTransaction();
        return true;
    }

    void delayMs(uint32_t ms) override {
        delay(ms);
    }

private:
    SPIClass& spi_;
    int csPin_;
    uint32_t spiHz_;
    uint8_t spiMode_;
    SPISettings settings_;
};

#endif // ARDUINO

class Lsm6dsv {
public:
    static constexpr uint8_t WHO_AM_I_EXPECTED = 0x70;

    enum class Odr : uint8_t {
        PowerDown = 0x00,
        Hz1_875   = 0x01, // accelerometer only
        Hz7_5     = 0x02,
        Hz15      = 0x03,
        Hz30      = 0x04,
        Hz60      = 0x05,
        Hz120     = 0x06,
        Hz240     = 0x07,
        Hz480     = 0x08,
        Hz960     = 0x09,
        Hz1920    = 0x0A,
        Hz3840    = 0x0B,
        Hz7680    = 0x0C,
    };

    enum class AccelFs : uint8_t {
        G2  = 0x00,
        G4  = 0x01,
        G8  = 0x02,
        G16 = 0x03,
    };

    enum class GyroFs : uint8_t {
        Dps125  = 0x00,
        Dps250  = 0x01,
        Dps500  = 0x02,
        Dps1000 = 0x03,
        Dps2000 = 0x04,
        Dps4000 = 0x0C,
    };

    enum class AccelMode : uint8_t {
        HighPerformance = 0x00,
        HighAccuracyOdr = 0x01,
        OdrTriggered    = 0x03,
        LowPower1       = 0x04,
        LowPower2       = 0x05,
        LowPower3       = 0x06,
        Normal          = 0x07,
    };

    enum class GyroMode : uint8_t {
        HighPerformance = 0x00,
        HighAccuracyOdr = 0x01,
        OdrTriggered    = 0x03,
        Sleep           = 0x04,
        LowPower        = 0x05,
    };

    enum SampleFlags : uint16_t {
        FLAG_NONE            = 0,
        FLAG_STATUS_XLDA     = 1u << 0,
        FLAG_STATUS_GDA      = 1u << 1,
        FLAG_STATUS_TDA      = 1u << 2,
        FLAG_ACCEL_SATURATED = 1u << 3,
        FLAG_GYRO_SATURATED  = 1u << 4,
        FLAG_READ_STATUS_OK  = 1u << 5,
        FLAG_READ_OUTPUT_OK  = 1u << 6,
    };

    struct Config {
        Odr accelOdr = Odr::Hz960;
        Odr gyroOdr  = Odr::Hz960;

        AccelFs accelFs = AccelFs::G8;
        GyroFs gyroFs   = GyroFs::Dps1000;

        AccelMode accelMode = AccelMode::HighPerformance;
        GyroMode gyroMode   = GyroMode::HighPerformance;

        bool doSoftwareReset = true;
        bool disableI2cAndI3c = true;
        bool blockDataUpdate = true;
        bool autoIncrement = true;

        // Start with digital LPFs disabled. Filtering/tuning comes later.
        bool gyroLpf1Enable = false;
        uint8_t gyroLpf1BandwidthBits = 0;

        // Direct DRDY interrupt routing is optional for now.
        bool int1DrdyAccel = false;
        bool int1DrdyGyro = false;
        bool int2DrdyAccel = false;
        bool int2DrdyGyro = false;
        bool drdyPulsed = true;
    };

    struct Status {
        uint8_t raw = 0;
        bool accelDataReady = false;
        bool gyroDataReady = false;
        bool tempDataReady = false;
        bool gyroEisDataReady = false;
        bool oisDataReady = false;
        bool timestampEndcount = false;
    };

    struct RawSample {
        uint64_t t_us = 0;

        int16_t gx = 0;
        int16_t gy = 0;
        int16_t gz = 0;

        int16_t ax = 0;
        int16_t ay = 0;
        int16_t az = 0;

        int16_t temp = 0;

        uint8_t statusRaw = 0;
        uint16_t flags = FLAG_NONE;
    };

    struct Sample {
        uint64_t t_us = 0;
        Vec3 gyro_rad_s = Vec3::zero();
        Vec3 accel_g = Vec3::zero();
        float temp_c = 25.0f;
        uint8_t statusRaw = 0;
        uint16_t flags = FLAG_NONE;
    };

    explicit Lsm6dsv(Lsm6dsvTransport& transport) : bus_(transport) {}

    bool begin() {
        Config config;
        return begin(config);
    }

    bool begin(const Config& config) {
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

    bool isInitialized() const {
        return initialized_;
    }

    uint8_t lastWhoAmI() const {
        return lastWhoAmI_;
    }

    enum class Error : uint8_t {
        None = 0,
        BusReadFailed,
        BusWriteFailed,
        WrongWhoAmI,
        ResetTimeout,
        InvalidConfig,
    };

    Error lastError() const {
        return lastError_;
    }

    bool readWhoAmI(uint8_t& out) {
        if (!readReg(Reg::WHO_AM_I, out)) {
            return false;
        }
        lastWhoAmI_ = out;
        return true;
    }

    bool softwareReset() {
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

    bool readStatus(Status& status) {
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

    bool hasNewImuData(bool requireAccel = true, bool requireGyro = true) {
        Status s;
        if (!readStatus(s)) {
            return false;
        }
        const bool accelOk = !requireAccel || s.accelDataReady;
        const bool gyroOk = !requireGyro || s.gyroDataReady;
        return accelOk && gyroOk;
    }

    bool readRawSample(RawSample& out, uint64_t t_us = 0, bool readStatusFirst = true) {
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

        if (isRawSaturated(out.ax) || isRawSaturated(out.ay) || isRawSaturated(out.az)) {
            flags |= FLAG_ACCEL_SATURATED;
        }
        if (isRawSaturated(out.gx) || isRawSaturated(out.gy) || isRawSaturated(out.gz)) {
            flags |= FLAG_GYRO_SATURATED;
        }

        out.flags = flags;
        return true;
    }

    bool readSample(Sample& out, uint64_t t_us = 0, bool readStatusFirst = true) {
        RawSample raw;
        if (!readRawSample(raw, t_us, readStatusFirst)) {
            return false;
        }
        out = scale(raw);
        return true;
    }

    Sample scale(const RawSample& raw) const {
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

    bool setAccelFullScale(AccelFs fs) {
        cfg_.accelFs = fs;
        return writeMasked(Reg::CTRL8, CTRL8_FS_XL_MASK, static_cast<uint8_t>(fs));
    }

    bool setGyroFullScale(GyroFs fs) {
        cfg_.gyroFs = fs;
        return writeMasked(Reg::CTRL6, CTRL6_FS_G_MASK, static_cast<uint8_t>(fs));
    }

    bool setAccelOdr(Odr odr, AccelMode mode = AccelMode::HighPerformance) {
        cfg_.accelOdr = odr;
        cfg_.accelMode = mode;
        const uint8_t value =
            static_cast<uint8_t>((static_cast<uint8_t>(mode) & 0x07u) << 4) |
            static_cast<uint8_t>(static_cast<uint8_t>(odr) & 0x0Fu);
        return writeReg(Reg::CTRL1, value);
    }

    bool setGyroOdr(Odr odr, GyroMode mode = GyroMode::HighPerformance) {
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

    bool powerDown() {
        bool ok = true;
        ok = writeReg(Reg::CTRL1, 0x00) && ok;
        ok = writeReg(Reg::CTRL2, 0x00) && ok;
        return ok;
    }

    bool disableFifo() {
        bool ok = true;
        ok = writeReg(Reg::FIFO_CTRL4, 0x00) && ok; // bypass mode
        ok = writeReg(Reg::FIFO_CTRL3, 0x00) && ok; // no accel/gyro batching
        ok = writeReg(Reg::FIFO_CTRL2, 0x00) && ok; // no compression, no stop-on-wtm
        ok = writeReg(Reg::FIFO_CTRL1, 0x00) && ok; // watermark 0
        if (!ok) return fail(Error::BusWriteFailed);
        return true;
    }

    bool readFifoUnreadCount(uint16_t& unreadFrames) {
        uint8_t b[2] = {};
        if (!read(Reg::FIFO_STATUS1, b, sizeof(b))) {
            return fail(Error::BusReadFailed);
        }
        unreadFrames = static_cast<uint16_t>(b[0]) |
                       static_cast<uint16_t>((b[1] & 0x01u) << 8);
        return true;
    }

    bool configureDrdyOnInt1(bool accel, bool gyro) {
        cfg_.int1DrdyAccel = accel;
        cfg_.int1DrdyGyro = gyro;
        uint8_t value = 0;
        if (gyro)  value |= INT_CTRL_DRDY_G;
        if (accel) value |= INT_CTRL_DRDY_XL;
        return writeReg(Reg::INT1_CTRL, value);
    }

    bool configureDrdyOnInt2(bool accel, bool gyro) {
        cfg_.int2DrdyAccel = accel;
        cfg_.int2DrdyGyro = gyro;
        uint8_t value = 0;
        if (gyro)  value |= INT_CTRL_DRDY_G;
        if (accel) value |= INT_CTRL_DRDY_XL;
        return writeReg(Reg::INT2_CTRL, value);
    }

    static float odrHz(Odr odr) {
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

    static float accelSensitivityGPerLSB(AccelFs fs) {
        switch (fs) {
            case AccelFs::G2:  return 0.061f * 1.0e-3f;
            case AccelFs::G4:  return 0.122f * 1.0e-3f;
            case AccelFs::G8:  return 0.244f * 1.0e-3f;
            case AccelFs::G16: return 0.488f * 1.0e-3f;
        }
        return 0.0f;
    }

    static float gyroSensitivityDpsPerLSB(GyroFs fs) {
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

    static float gyroSensitivityRadPerSecPerLSB(GyroFs fs) {
        return gyroSensitivityDpsPerLSB(fs) * MATH_DEG_TO_RAD;
    }

    static float temperatureC(int16_t rawTemp) {
        return 25.0f + static_cast<float>(rawTemp) / 256.0f;
    }

private:
    enum Reg : uint8_t {
        FUNC_CFG_ACCESS = 0x01,
        REG_PIN_CTRL    = 0x02,
        IF_CFG          = 0x03,

        FIFO_CTRL1      = 0x07,
        FIFO_CTRL2      = 0x08,
        FIFO_CTRL3      = 0x09,
        FIFO_CTRL4      = 0x0A,

        INT1_CTRL       = 0x0D,
        INT2_CTRL       = 0x0E,
        WHO_AM_I        = 0x0F,

        CTRL1           = 0x10,
        CTRL2           = 0x11,
        CTRL3           = 0x12,
        CTRL4           = 0x13,
        CTRL5           = 0x14,
        CTRL6           = 0x15,
        CTRL7           = 0x16,
        CTRL8           = 0x17,
        CTRL9           = 0x18,
        CTRL10          = 0x19,

        FIFO_STATUS1    = 0x1B,
        FIFO_STATUS2    = 0x1C,
        STATUS_REG      = 0x1E,

        OUT_TEMP_L      = 0x20,
        OUT_TEMP_H      = 0x21,
        OUTX_L_G        = 0x22,
        OUTX_H_G        = 0x23,
        OUTY_L_G        = 0x24,
        OUTY_H_G        = 0x25,
        OUTZ_L_G        = 0x26,
        OUTZ_H_G        = 0x27,
        OUTX_L_A        = 0x28,
        OUTX_H_A        = 0x29,
        OUTY_L_A        = 0x2A,
        OUTY_H_A        = 0x2B,
        OUTZ_L_A        = 0x2C,
        OUTZ_H_A        = 0x2D,
    };

    static constexpr uint8_t IF_CFG_I2C_I3C_DISABLE = 1u << 0;

    static constexpr uint8_t CTRL3_SW_RESET = 1u << 0;
    static constexpr uint8_t CTRL3_IF_INC   = 1u << 2;
    static constexpr uint8_t CTRL3_BDU      = 1u << 6;
    static constexpr uint8_t CTRL3_BOOT     = 1u << 7;

    static constexpr uint8_t CTRL6_FS_G_MASK = 0x0F;
    static constexpr uint8_t CTRL8_FS_XL_MASK = 0x03;

    static constexpr uint8_t CTRL7_LPF1_G_EN = 1u << 0;

    static constexpr uint8_t STATUS_XLDA = 1u << 0;
    static constexpr uint8_t STATUS_GDA  = 1u << 1;
    static constexpr uint8_t STATUS_TDA  = 1u << 2;
    static constexpr uint8_t STATUS_GDA_EIS = 1u << 4;
    static constexpr uint8_t STATUS_OIS_DRDY = 1u << 5;
    static constexpr uint8_t STATUS_TIMESTAMP_ENDCOUNT = 1u << 7;

    static constexpr uint8_t INT_CTRL_DRDY_XL = 1u << 0;
    static constexpr uint8_t INT_CTRL_DRDY_G  = 1u << 1;

    static constexpr uint8_t CTRL4_DRDY_PULSED = 1u << 1;

    bool configureCtrl3() {
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

    bool configureCtrl4() {
        uint8_t value = 0;

        if (cfg_.drdyPulsed) {
            value |= CTRL4_DRDY_PULSED;
        }

        if (!writeReg(Reg::CTRL4, value)) {
            return fail(Error::BusWriteFailed);
        }

        return true;
    }

    bool configureFiltersAndFullScale() {
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

    bool configureInterrupts() {
        if (!configureDrdyOnInt1(cfg_.int1DrdyAccel, cfg_.int1DrdyGyro)) {
            return false;
        }
        if (!configureDrdyOnInt2(cfg_.int2DrdyAccel, cfg_.int2DrdyGyro)) {
            return false;
        }
        return true;
    }

    bool setOdrAndModes(Odr accelOdr, Odr gyroOdr, AccelMode accelMode, GyroMode gyroMode) {
        if (!setAccelOdr(accelOdr, accelMode)) {
            return false;
        }
        if (!setGyroOdr(gyroOdr, gyroMode)) {
            return false;
        }
        return true;
    }

    bool writeMasked(Reg reg, uint8_t mask, uint8_t value) {
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

    bool read(Reg reg, uint8_t* dst, size_t len) {
        return bus_.read(static_cast<uint8_t>(reg), dst, len);
    }

    bool write(Reg reg, const uint8_t* src, size_t len) {
        return bus_.write(static_cast<uint8_t>(reg), src, len);
    }

    bool readReg(Reg reg, uint8_t& value) {
        return bus_.readReg(static_cast<uint8_t>(reg), value);
    }

    bool writeReg(Reg reg, uint8_t value) {
        return bus_.writeReg(static_cast<uint8_t>(reg), value);
    }

    static int16_t le16(const uint8_t* p) {
        return static_cast<int16_t>(
            static_cast<uint16_t>(p[0]) |
            static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8)
        );
    }

    static bool isRawSaturated(int16_t v) {
        return v == INT16_MAX || v == INT16_MIN;
    }

    bool fail(Error e) {
        lastError_ = e;
        return false;
    }

    Lsm6dsvTransport& bus_;
    Config cfg_;
    bool initialized_ = false;
    Error lastError_ = Error::None;
    uint8_t lastWhoAmI_ = 0x00;
};

} // namespace tracker
