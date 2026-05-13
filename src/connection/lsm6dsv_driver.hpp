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
// - FIFO parsing lives in lsm6dsv_fifo.*; this driver keeps direct register/config access separate.
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
        static uint8_t zeros[64] = {};
        size_t offset = 0;
        while (offset < len) {
            size_t chunk = len - offset;
            if (chunk > sizeof(zeros)) {
                chunk = sizeof(zeros);
            }
            spi_.transferBytes(zeros, dst + offset, chunk);
            offset += chunk;
        }
#else
        for (size_t i = 0; i < len; ++i) {
            dst[i] = spi_.transfer(0x00);
        }
#endif

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

    explicit Lsm6dsv(Lsm6dsvTransport& transport);

    bool begin();

    bool begin(const Config& config);

    bool isInitialized() const;

    uint8_t lastWhoAmI() const;

    enum class Error : uint8_t {
        None = 0,
        BusReadFailed,
        BusWriteFailed,
        WrongWhoAmI,
        ResetTimeout,
        InvalidConfig,
    };

    Error lastError() const;

    bool readWhoAmI(uint8_t& out);

    bool softwareReset();

    bool readStatus(Status& status);

    bool hasNewImuData(bool requireAccel = true, bool requireGyro = true);

    bool readRawSample(RawSample& out, uint64_t t_us = 0, bool readStatusFirst = true);

    bool readSample(Sample& out, uint64_t t_us = 0, bool readStatusFirst = true);

    Sample scale(const RawSample& raw) const;

    bool setAccelFullScale(AccelFs fs);

    bool setGyroFullScale(GyroFs fs);

    bool setAccelOdr(Odr odr, AccelMode mode = AccelMode::HighPerformance);

    bool setGyroOdr(Odr odr, GyroMode mode = GyroMode::HighPerformance);

    bool powerDown();

    bool disableFifo();

    bool readFifoUnreadCount(uint16_t& unreadFrames);

    bool configureDrdyOnInt1(bool accel, bool gyro);

    bool configureDrdyOnInt2(bool accel, bool gyro);

    static float odrHz(Odr odr);

    static float accelSensitivityGPerLSB(AccelFs fs);

    static float gyroSensitivityDpsPerLSB(GyroFs fs);

    static float gyroSensitivityRadPerSecPerLSB(GyroFs fs);

    static float temperatureC(int16_t rawTemp);

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

    bool configureCtrl3();

    bool configureCtrl4();

    bool configureFiltersAndFullScale();

    bool configureInterrupts();

    bool setOdrAndModes(Odr accelOdr, Odr gyroOdr, AccelMode accelMode, GyroMode gyroMode);

    bool writeMasked(Reg reg, uint8_t mask, uint8_t value);

    bool read(Reg reg, uint8_t* dst, size_t len);

    bool write(Reg reg, const uint8_t* src, size_t len);

    bool readReg(Reg reg, uint8_t& value);

    bool writeReg(Reg reg, uint8_t value);

    static int16_t le16(const uint8_t* p);

    static bool isRawSaturated(int16_t v);

    bool fail(Error e);

    Lsm6dsvTransport& bus_;
    Config cfg_;
    bool initialized_ = false;
    Error lastError_ = Error::None;
    uint8_t lastWhoAmI_ = 0x00;
};

} // namespace tracker
