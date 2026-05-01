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

    explicit Qmc6309(Lsm6dsvSensorHub& hub) : hub_(hub) {}

    Error lastError() const { return lastError_; }

    const char* lastErrorName() const {
        switch (lastError_) {
            case Error::None: return "None";
            case Error::BusFailed: return "BusFailed";
            case Error::WrongChipId: return "WrongChipId";
            case Error::InvalidArgument: return "InvalidArgument";
            case Error::NotConfigured: return "NotConfigured";
        }
        return "Unknown";
    }

    const Config& config() const { return cfg_; }

    bool probe(uint8_t* chipIdOut = nullptr) {
        uint8_t id = 0;
        if (!readReg(REG_CHIP_ID, id)) return setError(Error::BusFailed);
        if (chipIdOut) *chipIdOut = id;
        if (id != EXPECTED_CHIP_ID) return setError(Error::WrongChipId);
        lastError_ = Error::None;
        return true;
    }

    bool softReset() {
        if (!writeReg(REG_CTRL2, 0x80)) return setError(Error::BusFailed);
        hub_.lastStatus();
        delayMs(5);
        if (!writeReg(REG_CTRL2, 0x00)) return setError(Error::BusFailed);
        delayMs(10);
        configured_ = false;
        return true;
    }

    bool configure(const Config& cfg) {
        cfg_ = cfg;
        if (cfg_.addr7 > 0x7F) return setError(Error::InvalidArgument);

        if (cfg_.softResetFirst) {
            if (!softReset()) return false;
        }

        uint8_t id = 0;
        if (!probe(&id)) return false;

        // Datasheet recommends passing through suspend between mode changes.
        if (!writeReg(REG_CTRL1, makeCtrl1(Mode::Suspend, cfg_.osr1, cfg_.osr2))) {
            return setError(Error::BusFailed);
        }
        delayMs(3);

        // CTRL2: ODR, range, set/reset mode. Keep SOFT_RST bit clear.
        if (!writeReg(REG_CTRL2, makeCtrl2(cfg_.odr, cfg_.range, cfg_.setResetMode))) {
            return setError(Error::BusFailed);
        }

        // CTRL1: OSR + final mode.
        if (!writeReg(REG_CTRL1, makeCtrl1(cfg_.mode, cfg_.osr1, cfg_.osr2))) {
            return setError(Error::BusFailed);
        }

        delayMs(cfg_.settleMs);
        configured_ = true;
        range_ = cfg_.range;
        return true;
    }

    bool configureNormal100Hz() {
        Config c;
        c.addr7 = cfg_.addr7;
        c.mode = Mode::Normal;
        c.odr = Odr::Hz100;
        c.range = Range::G32;
        c.osr1 = Osr1::Ratio8;
        c.osr2 = Osr2::Lpf8;
        c.setResetMode = SetResetMode::SetAndResetOn;
        c.softResetFirst = true;
        c.settleMs = 20;
        return configure(c);
    }

    bool configureNormal200Hz() {
        Config c;
        c.addr7 = cfg_.addr7;
        c.mode = Mode::Normal;
        c.odr = Odr::Hz200;
        c.range = Range::G32;
        c.osr1 = Osr1::Ratio8;
        c.osr2 = Osr2::Lpf8;
        c.setResetMode = SetResetMode::SetAndResetOn;
        c.softResetFirst = true;
        c.settleMs = 20;
        return configure(c);
    }

    bool suspend() {
        if (!writeReg(REG_CTRL1, makeCtrl1(Mode::Suspend, cfg_.osr1, cfg_.osr2))) {
            return setError(Error::BusFailed);
        }
        configured_ = false;
        return true;
    }

    bool readStatus(Status& out) {
        uint8_t raw = 0;
        if (!readReg(REG_STATUS, raw)) return setError(Error::BusFailed);
        out = decodeStatus(raw);
        return true;
    }

    bool readRaw(RawSample& out, bool readStatusFirst = true) {
        Status st;
        if (readStatusFirst) {
            if (!readStatus(st)) return false;
        }

        uint8_t b[6] = {};
        if (!readRegs(REG_DATA_X_L, b, sizeof(b))) return setError(Error::BusFailed);

        out.x = le16(&b[0]);
        out.y = le16(&b[2]);
        out.z = le16(&b[4]);
        out.statusRaw = st.raw;
        out.dataReady = st.dataReady;
        out.overflow = st.overflow || saturated(out.x) || saturated(out.y) || saturated(out.z);
        out.seq = ++seq_;
        return true;
    }

    bool readReg(uint8_t reg, uint8_t& value) {
        if (!hub_.readExternalReg(cfg_.addr7, reg, value)) return setError(Error::BusFailed);
        return true;
    }

    bool readRegs(uint8_t reg, uint8_t* dst, uint8_t len) {
        if (dst == nullptr || len == 0) return setError(Error::InvalidArgument);
        if (!hub_.readExternal(cfg_.addr7, reg, dst, len)) return setError(Error::BusFailed);
        return true;
    }

    bool writeReg(uint8_t reg, uint8_t value) {
        if (!hub_.writeExternalReg(cfg_.addr7, reg, value)) return setError(Error::BusFailed);
        return true;
    }

    // For future FIFO path: QMC is configured once, then LSM6DSV SLV0 can be
    // armed to read REG_DATA_X_L..REG_DATA_X_L+5 continuously.
    bool armHubFifoRead(Lsm6dsvSensorHub::ShubOdr hubOdr = Lsm6dsvSensorHub::ShubOdr::Hz60) {
        if (!configured_) return setError(Error::NotConfigured);
        if (!hub_.configureSlave0ContinuousRead(cfg_.addr7, REG_DATA_X_L, 6, hubOdr, true)) {
            return setError(Error::BusFailed);
        }
        return true;
    }

    float lsbPerGauss() const { return lsbPerGauss(range_); }

    static float lsbPerGauss(Range range) {
        switch (range) {
            case Range::G32: return 1000.0f;
            case Range::G16: return 2000.0f;
            case Range::G8:  return 4000.0f;
        }
        return 1000.0f;
    }

    Vec3 scaleGauss(const RawSample& raw) const {
        const float s = 1.0f / lsbPerGauss();
        return Vec3(static_cast<float>(raw.x) * s,
                    static_cast<float>(raw.y) * s,
                    static_cast<float>(raw.z) * s);
    }

    static Status decodeStatus(uint8_t raw) {
        Status s;
        s.raw = raw;
        s.dataReady = (raw & 0x01u) != 0;
        s.overflow = (raw & 0x02u) != 0;
        s.selfTestReady = (raw & 0x04u) != 0;
        s.nvmReady = (raw & 0x08u) != 0;
        s.nvmLoadDone = (raw & 0x10u) != 0;
        return s;
    }

    static RawSample decode6(const uint8_t* b) {
        RawSample out;
        if (b == nullptr) return out;
        out.x = le16(&b[0]);
        out.y = le16(&b[2]);
        out.z = le16(&b[4]);
        out.overflow = saturated(out.x) || saturated(out.y) || saturated(out.z);
        return out;
    }

    static uint8_t makeCtrl1(Mode mode, Osr1 osr1, Osr2 osr2) {
        return static_cast<uint8_t>(((static_cast<uint8_t>(osr2) & 0x07u) << 5) |
                                    ((static_cast<uint8_t>(osr1) & 0x03u) << 3) |
                                    (static_cast<uint8_t>(mode) & 0x03u));
    }

    static uint8_t makeCtrl2(Odr odr, Range range, SetResetMode setResetMode) {
        return static_cast<uint8_t>(((static_cast<uint8_t>(odr) & 0x07u) << 4) |
                                    ((static_cast<uint8_t>(range) & 0x03u) << 2) |
                                    (static_cast<uint8_t>(setResetMode) & 0x03u));
    }

private:
    static int16_t le16(const uint8_t* p) {
        return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                    static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8));
    }

    static bool saturated(int16_t v) {
        return v >= 32767 || v <= -32768;
    }

    void delayMs(uint32_t ms) {
        // Use the hub bus delay through a tiny benign operation? Keep this class
        // platform-independent by using the hub transaction cadence for real I/O.
        // The hub itself has no public delay, so rely on Arduino when available.
#ifdef ARDUINO
        delay(ms);
#else
        (void)ms;
#endif
    }

    bool setError(Error e) {
        lastError_ = e;
        return false;
    }

    Lsm6dsvSensorHub& hub_;
    Config cfg_;
    Error lastError_ = Error::None;
    bool configured_ = false;
    Range range_ = Range::G32;
    uint32_t seq_ = 0;
};

} // namespace tracker
