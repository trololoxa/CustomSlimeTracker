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

    explicit Lsm6dsvSensorHub(Lsm6dsvTransport& bus);

    bool begin();

    bool begin(const Config& config);

    Error lastError() const;

    MasterStatus lastStatus() const;

    const char* lastErrorName() const;

    bool resetMaster();

    bool stopMaster();

    bool readMasterStatus(MasterStatus& out);

    bool readExternalReg(uint8_t addr7, uint8_t reg, uint8_t& value);

    bool readExternal(uint8_t addr7, uint8_t reg, uint8_t* dst, uint8_t len);

    bool writeExternalReg(uint8_t addr7, uint8_t reg, uint8_t value);    // Arm SLV0 as a continuous read source. If batchToFifo=true, SLV0 words
    // should later appear in FIFO tagged as sensor-hub slave0 when FIFO itself
    // is configured accordingly.
    bool configureSlave0ContinuousRead(uint8_t addr7,
                                       uint8_t startReg,
                                       uint8_t len,
                                       ShubOdr odr,
                                       bool batchToFifo);

    bool readSensorHubOutput(uint8_t* dst, uint8_t len);

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

    bool readMainReg(uint8_t reg, uint8_t& value);

    bool writeMainReg(uint8_t reg, uint8_t value);

    bool setShubRegisterAccess(bool enable);

    bool enterShubPage();

    void leaveShubPageBestEffort();

    bool writeShubReg(uint8_t reg, uint8_t value);

    bool readShubReg(uint8_t reg, uint8_t& value);

    bool stopMasterOnPage();

    bool readMasterStatusOnPage(MasterStatus& out);

    bool waitEndopOnPage(MasterStatus& out, uint16_t timeoutMs, bool requireWriteOnceDone);

    static uint8_t slvAddrByte(uint8_t addr7, bool read);

    static uint8_t slv0Config(ShubOdr odr, bool batchToFifo, uint8_t len);

    bool setError(Error e);

    Lsm6dsvTransport& bus_;
    Config cfg_;
    Error lastError_ = Error::None;
    MasterStatus lastStatus_;
};

} // namespace tracker
