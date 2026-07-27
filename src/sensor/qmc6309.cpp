#include "sensor/qmc6309.hpp"

namespace tracker {

static_assert(Qmc6309::RAW_REGISTER_AXES_RIGHT_HANDED,
              "mag axis calibration requires a right-handed QMC6309 driver frame");

Qmc6309::Qmc6309(Lsm6dsvSensorHub& hub) : hub_(hub) {}

Qmc6309::Error Qmc6309::lastError() const { return lastError_; }

const char* Qmc6309::lastErrorName() const {
        switch (lastError_) {
            case Error::None: return "None";
            case Error::BusFailed: return "BusFailed";
            case Error::WrongChipId: return "WrongChipId";
            case Error::InvalidArgument: return "InvalidArgument";
            case Error::NotConfigured: return "NotConfigured";
        }
        return "Unknown";
    }

const Qmc6309::Config& Qmc6309::config() const { return cfg_; }

bool Qmc6309::probe(uint8_t* chipIdOut) {
        uint8_t id = 0;
        if (!readReg(REG_CHIP_ID, id)) return setError(Error::BusFailed);
        if (chipIdOut) *chipIdOut = id;
        if (id != EXPECTED_CHIP_ID) return setError(Error::WrongChipId);
        lastError_ = Error::None;
        return true;
    }

bool Qmc6309::softReset() {
        if (!writeReg(REG_CTRL2, 0x80)) return setError(Error::BusFailed);
        hub_.lastStatus();
        delayMs(5);
        if (!writeReg(REG_CTRL2, 0x00)) return setError(Error::BusFailed);
        delayMs(10);
        configured_ = false;
        return true;
    }

bool Qmc6309::configure(const Config& cfg) {
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

bool Qmc6309::configureNormal100Hz() {
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

bool Qmc6309::configureNormal200Hz() {
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

bool Qmc6309::suspend() {
        if (!writeReg(REG_CTRL1, makeCtrl1(Mode::Suspend, cfg_.osr1, cfg_.osr2))) {
            return setError(Error::BusFailed);
        }
        configured_ = false;
        return true;
    }

bool Qmc6309::readStatus(Status& out) {
        uint8_t raw = 0;
        if (!readReg(REG_STATUS, raw)) return setError(Error::BusFailed);
        out = decodeStatus(raw);
        return true;
    }

bool Qmc6309::readRaw(RawSample& out, bool readStatusFirst) {
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

bool Qmc6309::readReg(uint8_t reg, uint8_t& value) {
        if (!hub_.readExternalReg(cfg_.addr7, reg, value)) return setError(Error::BusFailed);
        return true;
    }

bool Qmc6309::readRegs(uint8_t reg, uint8_t* dst, uint8_t len) {
        if (dst == nullptr || len == 0) return setError(Error::InvalidArgument);
        if (!hub_.readExternal(cfg_.addr7, reg, dst, len)) return setError(Error::BusFailed);
        return true;
    }

bool Qmc6309::writeReg(uint8_t reg, uint8_t value) {
        if (!hub_.writeExternalReg(cfg_.addr7, reg, value)) return setError(Error::BusFailed);
        return true;
    }

// For future FIFO path: QMC is configured once, then LSM6DSV SLV0 can be
    // armed to read REG_DATA_X_L..REG_DATA_X_L+5 continuously.
    bool Qmc6309::armHubFifoRead(Lsm6dsvSensorHub::ShubOdr hubOdr) {
        if (!configured_) return setError(Error::NotConfigured);
        if (!hub_.configureSlave0ContinuousRead(cfg_.addr7, REG_DATA_X_L, 6, hubOdr, true)) {
            return setError(Error::BusFailed);
        }
        return true;
    }

float Qmc6309::lsbPerGauss() const { return lsbPerGauss(range_); }

float Qmc6309::lsbPerGauss(Range range) {
        switch (range) {
            case Range::G32: return 1000.0f;
            case Range::G16: return 2000.0f;
            case Range::G8:  return 4000.0f;
        }
        return 1000.0f;
    }

Vec3 Qmc6309::scaleGauss(const RawSample& raw) const {
        const float s = 1.0f / lsbPerGauss();
        return Vec3(static_cast<float>(raw.x) * s,
                    static_cast<float>(raw.y) * s,
                    static_cast<float>(raw.z) * s);
    }

Qmc6309::Status Qmc6309::decodeStatus(uint8_t raw) {
        Status s;
        s.raw = raw;
        s.dataReady = (raw & 0x01u) != 0;
        s.overflow = (raw & 0x02u) != 0;
        s.selfTestReady = (raw & 0x04u) != 0;
        s.nvmReady = (raw & 0x08u) != 0;
        s.nvmLoadDone = (raw & 0x10u) != 0;
        return s;
    }

Qmc6309::RawSample Qmc6309::decode6(const uint8_t* b) {
        RawSample out;
        if (b == nullptr) return out;
        out.x = le16(&b[0]);
        out.y = le16(&b[2]);
        out.z = le16(&b[4]);
        out.overflow = saturated(out.x) || saturated(out.y) || saturated(out.z);
        return out;
    }

uint8_t Qmc6309::makeCtrl1(Mode mode, Osr1 osr1, Osr2 osr2) {
        return static_cast<uint8_t>(((static_cast<uint8_t>(osr2) & 0x07u) << 5) |
                                    ((static_cast<uint8_t>(osr1) & 0x03u) << 3) |
                                    (static_cast<uint8_t>(mode) & 0x03u));
    }

uint8_t Qmc6309::makeCtrl2(Odr odr, Range range, SetResetMode setResetMode) {
        return static_cast<uint8_t>(((static_cast<uint8_t>(odr) & 0x07u) << 4) |
                                    ((static_cast<uint8_t>(range) & 0x03u) << 2) |
                                    (static_cast<uint8_t>(setResetMode) & 0x03u));
    }

int16_t Qmc6309::le16(const uint8_t* p) {
        return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                    static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8));
    }

bool Qmc6309::saturated(int16_t v) {
        const int32_t wide = static_cast<int32_t>(v);
        const int32_t magnitude = wide < 0 ? -wide : wide;
        return magnitude >= static_cast<int32_t>(RAW_SATURATION_ABS_COUNTS);
    }

void Qmc6309::delayMs(uint32_t ms) {
        // Use the hub bus delay through a tiny benign operation? Keep this class
        // platform-independent by using the hub transaction cadence for real I/O.
        // The hub itself has no public delay, so rely on Arduino when available.
#ifdef ARDUINO
        delay(ms);
#else
        (void)ms;
#endif
    }

bool Qmc6309::setError(Error e) {
        lastError_ = e;
        return false;
    }

} // namespace tracker
