#pragma once

#include <cstddef>
#include <cstdint>

#include "core/math.hpp"
#include "connection/lsm6dsv_sensorhub.hpp"

namespace tracker {

// ============================================================
// QMC6309 magnetometer over LSM6DSV Sensor Hub
// ============================================================
// This is not a direct I2C driver. It talks to QMC6309 through
// Lsm6dsvSensorHub one-shot read/write transactions.
// ============================================================

class Qmc6309 {
public:
    static constexpr uint8_t DEFAULT_ADDR7 = 0x7C;
    static constexpr uint8_t EXPECTED_CHIP_ID = 0x90;

    static constexpr uint8_t REG_CHIP_ID = 0x00;
    static constexpr uint8_t REG_DATA_X_L = 0x01;
    static constexpr uint8_t REG_STATUS = 0x09;
    static constexpr uint8_t REG_CTRL1 = 0x0A;
    static constexpr uint8_t REG_CTRL2 = 0x0B;
    static constexpr uint8_t REG_SELF_TEST = 0x0E;
    static constexpr uint8_t REG_SELF_TEST_X = 0x13;

    enum class Error : uint8_t {
        None = 0,
        BusFailed,
        WrongChipId,
        InvalidArgument,
        NotConfigured,
    };

    enum class Mode : uint8_t {
        Suspend = 0x00,
        Normal = 0x01,
        Single = 0x02,
        Continuous = 0x03,
    };

    enum class Odr : uint8_t {
        Hz1 = 0x00,
        Hz10 = 0x01,
        Hz50 = 0x02,
        Hz100 = 0x03,
        Hz200 = 0x04,
    };

    enum class Range : uint8_t {
        G32 = 0x00,
        G16 = 0x01,
        G8  = 0x02,
    };

    enum class SetResetMode : uint8_t {
        SetAndResetOn = 0x00,
        SetOnlyOn = 0x01,
        SetAndResetOff = 0x03,
    };

    enum class Osr1 : uint8_t {
        Ratio8 = 0x00,
        Ratio4 = 0x01,
        Ratio2 = 0x02,
        Ratio1 = 0x03,
    };

    enum class Osr2 : uint8_t {
        Lpf1 = 0x00,
        Lpf2 = 0x01,
        Lpf4 = 0x02,
        Lpf8 = 0x03,
        Lpf16 = 0x04,
    };

    struct Config {
        uint8_t addr7 = DEFAULT_ADDR7;
        Mode mode = Mode::Normal;
        Odr odr = Odr::Hz100;
        Range range = Range::G32;
        SetResetMode setResetMode = SetResetMode::SetAndResetOn;
        Osr1 osr1 = Osr1::Ratio8;
        Osr2 osr2 = Osr2::Lpf8;
        bool softResetFirst = true;
        uint8_t settleMs = 20;
    };

    struct Status {
        uint8_t raw = 0;
        bool dataReady = false;
        bool overflow = false;
        bool selfTestReady = false;
        bool nvmReady = false;
        bool nvmLoadDone = false;
    };

    struct RawSample {
        int16_t x = 0;
        int16_t y = 0;
        int16_t z = 0;
        uint8_t statusRaw = 0;
        bool dataReady = false;
        bool overflow = false;
        uint32_t seq = 0;
    };

    explicit Qmc6309(Lsm6dsvSensorHub& hub);

    Error lastError() const;

    const char* lastErrorName() const;

    const Config& config() const;

    bool probe(uint8_t* chipIdOut = nullptr);

    bool softReset();

    bool configure(const Config& cfg);

    bool configureNormal100Hz();

    bool configureNormal200Hz();

    bool suspend();

    bool readStatus(Status& out);

    bool readRaw(RawSample& out, bool readStatusFirst = true);

    bool readReg(uint8_t reg, uint8_t& value);

    bool readRegs(uint8_t reg, uint8_t* dst, uint8_t len);

    bool writeReg(uint8_t reg, uint8_t value);    // For future FIFO path: QMC is configured once, then LSM6DSV SLV0 can be
    // armed to read REG_DATA_X_L..REG_DATA_X_L+5 continuously.
    bool armHubFifoRead(Lsm6dsvSensorHub::ShubOdr hubOdr = Lsm6dsvSensorHub::ShubOdr::Hz60);

    float lsbPerGauss() const;

    static float lsbPerGauss(Range range);

    Vec3 scaleGauss(const RawSample& raw) const;

    static Status decodeStatus(uint8_t raw);

    static RawSample decode6(const uint8_t* b);

    static uint8_t makeCtrl1(Mode mode, Osr1 osr1, Osr2 osr2);

    static uint8_t makeCtrl2(Odr odr, Range range, SetResetMode setResetMode);

private:
    static int16_t le16(const uint8_t* p);

    static bool saturated(int16_t v);

    void delayMs(uint32_t ms);

    bool setError(Error e);

    Lsm6dsvSensorHub& hub_;
    Config cfg_;
    Error lastError_ = Error::None;
    bool configured_ = false;
    Range range_ = Range::G32;
    uint32_t seq_ = 0;
};

} // namespace tracker
