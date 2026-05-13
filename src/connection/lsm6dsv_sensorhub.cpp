#include "connection/lsm6dsv_sensorhub.hpp"

namespace tracker {

Lsm6dsvSensorHub::Lsm6dsvSensorHub(Lsm6dsvTransport& bus) : bus_(bus) {}

bool Lsm6dsvSensorHub::begin() { Config cfg = Config{}; return begin(cfg); }

bool Lsm6dsvSensorHub::begin(const Config& config) {
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

Lsm6dsvSensorHub::Error Lsm6dsvSensorHub::lastError() const { return lastError_; }

Lsm6dsvSensorHub::MasterStatus Lsm6dsvSensorHub::lastStatus() const { return lastStatus_; }

const char* Lsm6dsvSensorHub::lastErrorName() const {
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

bool Lsm6dsvSensorHub::resetMaster() {
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

bool Lsm6dsvSensorHub::stopMaster() {
        if (!enterShubPage()) return false;
        const bool ok = stopMasterOnPage();
        leaveShubPageBestEffort();
        if (!ok) return setError(Error::BusWriteFailed);
        return true;
    }

bool Lsm6dsvSensorHub::readMasterStatus(MasterStatus& out) {
        if (!enterShubPage()) return false;
        const bool ok = readMasterStatusOnPage(out);
        leaveShubPageBestEffort();
        return ok;
    }

bool Lsm6dsvSensorHub::readExternalReg(uint8_t addr7, uint8_t reg, uint8_t& value) {
        return readExternal(addr7, reg, &value, 1);
    }

bool Lsm6dsvSensorHub::readExternal(uint8_t addr7, uint8_t reg, uint8_t* dst, uint8_t len) {
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

bool Lsm6dsvSensorHub::writeExternalReg(uint8_t addr7, uint8_t reg, uint8_t value) {
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
    bool Lsm6dsvSensorHub::configureSlave0ContinuousRead(uint8_t addr7,
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

bool Lsm6dsvSensorHub::readSensorHubOutput(uint8_t* dst, uint8_t len) {
        if (dst == nullptr || len == 0 || len > 18) {
            return setError(Error::InvalidArgument);
        }
        if (!enterShubPage()) return false;
        const bool ok = bus_.read(REG_SENSOR_HUB_1, dst, len);
        leaveShubPageBestEffort();
        if (!ok) return setError(Error::BusReadFailed);
        return true;
    }

bool Lsm6dsvSensorHub::readMainReg(uint8_t reg, uint8_t& value) {
        if (!bus_.readReg(reg, value)) return setError(Error::BusReadFailed);
        return true;
    }

bool Lsm6dsvSensorHub::writeMainReg(uint8_t reg, uint8_t value) {
        if (!bus_.writeReg(reg, value)) return setError(Error::BusWriteFailed);
        return true;
    }

bool Lsm6dsvSensorHub::setShubRegisterAccess(bool enable) {
        const uint8_t v = enable ? FUNC_SHUB_REG_ACCESS : 0x00;
        if (!bus_.writeReg(REG_FUNC_CFG_ACCESS, v)) return setError(Error::BusWriteFailed);
        return true;
    }

bool Lsm6dsvSensorHub::enterShubPage() {
        return setShubRegisterAccess(true);
    }

void Lsm6dsvSensorHub::leaveShubPageBestEffort() {
        bus_.writeReg(REG_FUNC_CFG_ACCESS, 0x00);
    }

bool Lsm6dsvSensorHub::writeShubReg(uint8_t reg, uint8_t value) {
        if (!bus_.writeReg(reg, value)) return setError(Error::BusWriteFailed);
        return true;
    }

bool Lsm6dsvSensorHub::readShubReg(uint8_t reg, uint8_t& value) {
        if (!bus_.readReg(reg, value)) return setError(Error::BusReadFailed);
        return true;
    }

bool Lsm6dsvSensorHub::stopMasterOnPage() {
        return writeShubReg(REG_MASTER_CONFIG, 0x00);
    }

bool Lsm6dsvSensorHub::readMasterStatusOnPage(MasterStatus& out) {
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

bool Lsm6dsvSensorHub::waitEndopOnPage(MasterStatus& out, uint16_t timeoutMs, bool requireWriteOnceDone) {
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

uint8_t Lsm6dsvSensorHub::slvAddrByte(uint8_t addr7, bool read) {
        return static_cast<uint8_t>(((addr7 & 0x7Fu) << 1) | (read ? 1u : 0u));
    }

uint8_t Lsm6dsvSensorHub::slv0Config(ShubOdr odr, bool batchToFifo, uint8_t len) {
        uint8_t v = static_cast<uint8_t>((static_cast<uint8_t>(odr) & 0x07u) << 5);
        if (batchToFifo) v |= 0x08u;
        v |= static_cast<uint8_t>(len & 0x07u);
        return v;
    }

bool Lsm6dsvSensorHub::setError(Error e) {
        lastError_ = e;
        return false;
    }

} // namespace tracker
