#pragma once

#include <cstddef>
#include <cstdint>

#include "connection/lsm6dsv_driver.hpp"

namespace tracker {

// ============================================================
// LSM6DSV Sensor Hub helper
// ============================================================
// Host path: ESP32-C3 -> SPI -> LSM6DSV -> sensor-hub I2C master -> QMC6309
//
// This helper deliberately uses only Lsm6dsvTransport and the documented
// sensor-hub register page. It does not depend on the high-level Lsm6dsv class.
//
// Important hardware/LSM notes:
// - Sensor-hub registers are visible only while FUNC_CFG_ACCESS.SHUB_REG_ACCESS=1.
// - External transactions are triggered by accel/gyro DRDY unless START_CONFIG is
//   changed. Keep XL/GY ODR enabled before using one-shot read/write helpers.
// - PASS_THROUGH_MODE is intentionally not used. In SPI-host designs the host
//   should access the external I2C sensor via MASTER_ON transactions.
// - SLV0_CONFIG Slave0_numop is 3 bits. This helper allows 1..7 bytes per SLV0
//   transaction. QMC6309 data burst is 6 bytes, so it fits.
// ============================================================

class Lsm6dsvSensorHub {
public:
    static constexpr uint8_t QMC6309_DEFAULT_ADDR7 = 0x7C;

    enum class ShubOdr : uint8_t {
        Hz1_875 = 0x00,
        Hz15    = 0x01,
        Hz30    = 0x02,
        Hz60    = 0x03,
        Hz120   = 0x04,
        Hz240   = 0x05,
        Hz480   = 0x06,
    };

    enum class Error : uint8_t {
        None = 0,
        InvalidArgument,
        BusReadFailed,
        BusWriteFailed,
        Timeout,
        Slave0Nack,
        Slave1Nack,
        Slave2Nack,
        Slave3Nack,
    };

    struct Config {
        bool resetMasterOnBegin = true;

        // Useful on small modules if external pull-ups are weak/missing.
        // If proper external pull-ups are mounted, leaving this true is still
        // normally harmless for diagnostics.
        bool enableInternalShubPullups = true;

        // Preserve the existing IF_CFG.I2C_I3C_disable bit unless explicitly set.
        bool forceDisablePrimaryI2cI3c = false;

        ShubOdr transactionOdr = ShubOdr::Hz120;
        uint16_t transactionTimeoutMs = 120;
    };

    struct MasterStatus {
        uint8_t raw = 0;
        bool writeOnceDone = false;
        bool slave3Nack = false;
        bool slave2Nack = false;
        bool slave1Nack = false;
        bool slave0Nack = false;
        bool endop = false;

        bool anyNack() const {
            return slave0Nack || slave1Nack || slave2Nack || slave3Nack;
        }
    };

    explicit Lsm6dsvSensorHub(Lsm6dsvTransport& bus) : bus_(bus) {}

    bool begin() { Config cfg = Config{}; return begin(cfg); }
    bool begin(const Config& config) {
        cfg_ = config;
        lastError_ = Error::None;
        lastStatus_ = MasterStatus{};

        // Always return to main page first.
        if (!setShubRegisterAccess(false)) return false;

        if (cfg_.enableInternalShubPullups || cfg_.forceDisablePrimaryI2cI3c) {
            uint8_t ifCfg = 0;
            if (!readMainReg(REG_IF_CFG, ifCfg)) return setError(Error::BusReadFailed);
            if (cfg_.enableInternalShubPullups) {
                ifCfg |= IF_CFG_SHUB_PU_EN;
            }
            if (cfg_.forceDisablePrimaryI2cI3c) {
                ifCfg |= IF_CFG_I2C_I3C_DISABLE;
            }
            if (!writeMainReg(REG_IF_CFG, ifCfg)) return setError(Error::BusWriteFailed);
        }

        if (cfg_.resetMasterOnBegin) {
            if (!resetMaster()) return false;
        }
        return true;
    }

    Error lastError() const { return lastError_; }
    MasterStatus lastStatus() const { return lastStatus_; }

    const char* lastErrorName() const {
        switch (lastError_) {
            case Error::None: return "None";
            case Error::InvalidArgument: return "InvalidArgument";
            case Error::BusReadFailed: return "BusReadFailed";
            case Error::BusWriteFailed: return "BusWriteFailed";
            case Error::Timeout: return "Timeout";
            case Error::Slave0Nack: return "Slave0Nack";
            case Error::Slave1Nack: return "Slave1Nack";
            case Error::Slave2Nack: return "Slave2Nack";
            case Error::Slave3Nack: return "Slave3Nack";
        }
        return "Unknown";
    }

    bool resetMaster() {
        if (!enterShubPage()) return false;
        bool ok = true;
        ok = writeShubReg(REG_MASTER_CONFIG, MASTER_RST_MASTER_REGS) && ok;
        bus_.delayMs(2);
        ok = writeShubReg(REG_MASTER_CONFIG, 0x00) && ok;
        bus_.delayMs(2);
        leaveShubPageBestEffort();
        if (!ok) return setError(Error::BusWriteFailed);
        return true;
    }

    bool stopMaster() {
        if (!enterShubPage()) return false;
        const bool ok = stopMasterOnPage();
        leaveShubPageBestEffort();
        if (!ok) return setError(Error::BusWriteFailed);
        return true;
    }

    bool readMasterStatus(MasterStatus& out) {
        if (!enterShubPage()) return false;
        const bool ok = readMasterStatusOnPage(out);
        leaveShubPageBestEffort();
        return ok;
    }

    bool readExternalReg(uint8_t addr7, uint8_t reg, uint8_t& value) {
        return readExternal(addr7, reg, &value, 1);
    }

    bool readExternal(uint8_t addr7, uint8_t reg, uint8_t* dst, uint8_t len) {
        if (dst == nullptr || len == 0 || len > 7 || addr7 > 0x7F) {
            return setError(Error::InvalidArgument);
        }

        if (!enterShubPage()) return false;

        bool ok = true;
        ok = stopMasterOnPage() && ok;
        ok = writeShubReg(REG_SLV0_ADD, slvAddrByte(addr7, true)) && ok;
        ok = writeShubReg(REG_SLV0_SUBADD, reg) && ok;
        ok = writeShubReg(REG_SLV0_CONFIG, slv0Config(cfg_.transactionOdr, false, len)) && ok;
        ok = writeShubReg(REG_DATAWRITE_SLV0, 0x00) && ok;
        ok = writeShubReg(REG_MASTER_CONFIG, static_cast<uint8_t>(MASTER_WRITE_ONCE | MASTER_ON)) && ok;

        MasterStatus status;
        if (ok) {
            ok = waitEndopOnPage(status, cfg_.transactionTimeoutMs, false);
        }

        if (ok && status.slave0Nack) {
            ok = false;
            lastError_ = Error::Slave0Nack;
        }

        if (ok) {
            ok = bus_.read(REG_SENSOR_HUB_1, dst, len);
            if (!ok) lastError_ = Error::BusReadFailed;
        }

        stopMasterOnPage();
        leaveShubPageBestEffort();

        if (!ok && lastError_ == Error::None) {
            lastError_ = Error::BusWriteFailed;
        }
        return ok;
    }

    bool writeExternalReg(uint8_t addr7, uint8_t reg, uint8_t value) {
        if (addr7 > 0x7F) {
            return setError(Error::InvalidArgument);
        }

        if (!enterShubPage()) return false;

        bool ok = true;
        ok = stopMasterOnPage() && ok;
        ok = writeShubReg(REG_DATAWRITE_SLV0, value) && ok;
        ok = writeShubReg(REG_SLV0_ADD, slvAddrByte(addr7, false)) && ok;
        ok = writeShubReg(REG_SLV0_SUBADD, reg) && ok;
        ok = writeShubReg(REG_SLV0_CONFIG, slv0Config(cfg_.transactionOdr, false, 1)) && ok;

        // WRITE_ONCE is essential: otherwise a write-configured SLV0 can repeat
        // the same write every hub cycle until MASTER_ON is cleared.
        ok = writeShubReg(REG_MASTER_CONFIG, static_cast<uint8_t>(MASTER_WRITE_ONCE | MASTER_ON)) && ok;

        MasterStatus status;
        if (ok) {
            ok = waitEndopOnPage(status, cfg_.transactionTimeoutMs, true);
        }

        if (ok && status.slave0Nack) {
            ok = false;
            lastError_ = Error::Slave0Nack;
        }

        stopMasterOnPage();
        leaveShubPageBestEffort();

        if (!ok && lastError_ == Error::None) {
            lastError_ = Error::BusWriteFailed;
        }
        return ok;
    }

    // Arm SLV0 as a continuous read source. If batchToFifo=true, SLV0 words
    // should later appear in FIFO tagged as sensor-hub slave0 when FIFO itself
    // is configured accordingly.
    bool configureSlave0ContinuousRead(uint8_t addr7,
                                       uint8_t startReg,
                                       uint8_t len,
                                       ShubOdr odr,
                                       bool batchToFifo) {
        if (len == 0 || len > 7 || addr7 > 0x7F) {
            return setError(Error::InvalidArgument);
        }

        if (!enterShubPage()) return false;
        bool ok = true;
        ok = stopMasterOnPage() && ok;
        ok = writeShubReg(REG_SLV0_ADD, slvAddrByte(addr7, true)) && ok;
        ok = writeShubReg(REG_SLV0_SUBADD, startReg) && ok;
        ok = writeShubReg(REG_SLV0_CONFIG, slv0Config(odr, batchToFifo, len)) && ok;
        ok = writeShubReg(REG_MASTER_CONFIG, static_cast<uint8_t>(MASTER_WRITE_ONCE | MASTER_ON)) && ok;
        leaveShubPageBestEffort();

        if (!ok) return setError(Error::BusWriteFailed);
        return true;
    }

    bool readSensorHubOutput(uint8_t* dst, uint8_t len) {
        if (dst == nullptr || len == 0 || len > 18) {
            return setError(Error::InvalidArgument);
        }
        if (!enterShubPage()) return false;
        const bool ok = bus_.read(REG_SENSOR_HUB_1, dst, len);
        leaveShubPageBestEffort();
        if (!ok) return setError(Error::BusReadFailed);
        return true;
    }

private:
    static constexpr uint8_t REG_FUNC_CFG_ACCESS = 0x01;
    static constexpr uint8_t REG_IF_CFG = 0x03;

    // FUNC_CFG_ACCESS bits on main page.
    static constexpr uint8_t FUNC_SHUB_REG_ACCESS = 1u << 6;

    // IF_CFG bits on main page.
    static constexpr uint8_t IF_CFG_SHUB_PU_EN = 1u << 6;
    static constexpr uint8_t IF_CFG_I2C_I3C_DISABLE = 1u << 0;

    // Sensor-hub page registers.
    static constexpr uint8_t REG_SENSOR_HUB_1 = 0x02;
    static constexpr uint8_t REG_MASTER_CONFIG = 0x14;
    static constexpr uint8_t REG_SLV0_ADD = 0x15;
    static constexpr uint8_t REG_SLV0_SUBADD = 0x16;
    static constexpr uint8_t REG_SLV0_CONFIG = 0x17;
    static constexpr uint8_t REG_DATAWRITE_SLV0 = 0x21;
    static constexpr uint8_t REG_STATUS_MASTER = 0x22;

    static constexpr uint8_t MASTER_RST_MASTER_REGS = 1u << 7;
    static constexpr uint8_t MASTER_WRITE_ONCE = 1u << 6;
    static constexpr uint8_t MASTER_START_CONFIG_INT2 = 1u << 5;
    static constexpr uint8_t MASTER_PASS_THROUGH = 1u << 4;
    static constexpr uint8_t MASTER_ON = 1u << 2;

    static constexpr uint8_t STATUS_WR_ONCE_DONE = 1u << 7;
    static constexpr uint8_t STATUS_SLAVE3_NACK = 1u << 6;
    static constexpr uint8_t STATUS_SLAVE2_NACK = 1u << 5;
    static constexpr uint8_t STATUS_SLAVE1_NACK = 1u << 4;
    static constexpr uint8_t STATUS_SLAVE0_NACK = 1u << 3;
    static constexpr uint8_t STATUS_SENS_HUB_ENDOP = 1u << 0;

    bool readMainReg(uint8_t reg, uint8_t& value) {
        if (!bus_.readReg(reg, value)) return setError(Error::BusReadFailed);
        return true;
    }

    bool writeMainReg(uint8_t reg, uint8_t value) {
        if (!bus_.writeReg(reg, value)) return setError(Error::BusWriteFailed);
        return true;
    }

    bool setShubRegisterAccess(bool enable) {
        const uint8_t v = enable ? FUNC_SHUB_REG_ACCESS : 0x00;
        if (!bus_.writeReg(REG_FUNC_CFG_ACCESS, v)) return setError(Error::BusWriteFailed);
        return true;
    }

    bool enterShubPage() {
        return setShubRegisterAccess(true);
    }

    void leaveShubPageBestEffort() {
        bus_.writeReg(REG_FUNC_CFG_ACCESS, 0x00);
    }

    bool writeShubReg(uint8_t reg, uint8_t value) {
        if (!bus_.writeReg(reg, value)) return setError(Error::BusWriteFailed);
        return true;
    }

    bool readShubReg(uint8_t reg, uint8_t& value) {
        if (!bus_.readReg(reg, value)) return setError(Error::BusReadFailed);
        return true;
    }

    bool stopMasterOnPage() {
        return writeShubReg(REG_MASTER_CONFIG, 0x00);
    }

    bool readMasterStatusOnPage(MasterStatus& out) {
        uint8_t raw = 0;
        if (!readShubReg(REG_STATUS_MASTER, raw)) return false;
        out.raw = raw;
        out.writeOnceDone = (raw & STATUS_WR_ONCE_DONE) != 0;
        out.slave3Nack = (raw & STATUS_SLAVE3_NACK) != 0;
        out.slave2Nack = (raw & STATUS_SLAVE2_NACK) != 0;
        out.slave1Nack = (raw & STATUS_SLAVE1_NACK) != 0;
        out.slave0Nack = (raw & STATUS_SLAVE0_NACK) != 0;
        out.endop = (raw & STATUS_SENS_HUB_ENDOP) != 0;
        lastStatus_ = out;
        return true;
    }

    bool waitEndopOnPage(MasterStatus& out, uint16_t timeoutMs, bool requireWriteOnceDone) {
        const uint16_t polls = static_cast<uint16_t>(timeoutMs > 0 ? timeoutMs : 1);
        for (uint16_t i = 0; i < polls; ++i) {
            if (!readMasterStatusOnPage(out)) return false;
            if (out.slave0Nack) return setError(Error::Slave0Nack);
            if (out.slave1Nack) return setError(Error::Slave1Nack);
            if (out.slave2Nack) return setError(Error::Slave2Nack);
            if (out.slave3Nack) return setError(Error::Slave3Nack);
            if (out.endop && (!requireWriteOnceDone || out.writeOnceDone)) {
                return true;
            }
            bus_.delayMs(1);
        }
        return setError(Error::Timeout);
    }

    static uint8_t slvAddrByte(uint8_t addr7, bool read) {
        return static_cast<uint8_t>(((addr7 & 0x7Fu) << 1) | (read ? 1u : 0u));
    }

    static uint8_t slv0Config(ShubOdr odr, bool batchToFifo, uint8_t len) {
        uint8_t v = static_cast<uint8_t>((static_cast<uint8_t>(odr) & 0x07u) << 5);
        if (batchToFifo) v |= 0x08u;
        v |= static_cast<uint8_t>(len & 0x07u);
        return v;
    }

    bool setError(Error e) {
        lastError_ = e;
        return false;
    }

    Lsm6dsvTransport& bus_;
    Config cfg_;
    Error lastError_ = Error::None;
    MasterStatus lastStatus_;
};

} // namespace tracker
