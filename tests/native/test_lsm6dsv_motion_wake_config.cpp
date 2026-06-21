#include <array>
#include <cstddef>
#include <cstdint>

#include "connection/lsm6dsv_driver.hpp"
#include "test_common.hpp"

namespace {

class RegisterTransport final : public tracker::Lsm6dsvTransport {
public:
    bool read(uint8_t reg, uint8_t* dst, size_t len) override {
        if (!dst) return false;
        for (size_t i = 0; i < len; ++i) dst[i] = reg_[static_cast<uint8_t>(reg + i)];
        return true;
    }

    bool write(uint8_t reg, const uint8_t* src, size_t len) override {
        if (!src) return false;
        for (size_t i = 0; i < len; ++i) reg_[static_cast<uint8_t>(reg + i)] = src[i];
        return true;
    }

    void delayMs(uint32_t) override {}

    std::array<uint8_t, 256> reg_{};
};

constexpr uint8_t REG_CTRL1 = 0x10;
constexpr uint8_t REG_CTRL2 = 0x11;
constexpr uint8_t REG_FIFO_CTRL1 = 0x07;
constexpr uint8_t REG_FIFO_CTRL4 = 0x0A;
constexpr uint8_t REG_INT1_CTRL = 0x0D;
constexpr uint8_t REG_WAKE_UP_SRC = 0x45;
constexpr uint8_t REG_FUNCTIONS_ENABLE = 0x50;
constexpr uint8_t REG_INACTIVITY_DUR = 0x54;
constexpr uint8_t REG_TAP_CFG0 = 0x56;
constexpr uint8_t REG_WAKE_UP_THS = 0x5B;
constexpr uint8_t REG_WAKE_UP_DUR = 0x5C;
constexpr uint8_t REG_MD1_CFG = 0x5E;

} // namespace

int main() {
    TestContext ctx;
    RegisterTransport bus;
    tracker::Lsm6dsv imu(bus);

    // Preserve timestamp enable while enabling basic-function interrupts.
    bus.reg_[REG_FUNCTIONS_ENABLE] = 0x01u;
    bus.reg_[REG_WAKE_UP_SRC] = 0x0fu;

    tracker::Lsm6dsv::MotionWakeConfig cfg;
    cfg.enabled = true;
    cfg.threshold = 12;
    cfg.duration = 2;

    CHECK(ctx, imu.configureMotionWake(cfg));
    CHECK(ctx, bus.reg_[REG_CTRL1] == 0x45u); // LPM1 + 60 Hz
    CHECK(ctx, bus.reg_[REG_CTRL2] == 0x00u); // gyro power-down
    CHECK(ctx, bus.reg_[REG_FIFO_CTRL1] == 0x00u);
    CHECK(ctx, bus.reg_[REG_FIFO_CTRL4] == 0x00u);
    CHECK(ctx, bus.reg_[REG_INT1_CTRL] == 0x00u);
    CHECK(ctx, (bus.reg_[REG_FUNCTIONS_ENABLE] & 0x81u) == 0x81u);
    CHECK(ctx, (bus.reg_[REG_TAP_CFG0] & 0x31u) == 0x31u); // latch + HPF + settle mask
    CHECK(ctx, (bus.reg_[REG_INACTIVITY_DUR] & 0x38u) == 0x18u); // 62.5 mg/code
    CHECK(ctx, (bus.reg_[REG_WAKE_UP_THS] & 0x3fu) == 12u);
    CHECK(ctx, (bus.reg_[REG_WAKE_UP_DUR] & 0x03u) == 2u);
    CHECK(ctx, (bus.reg_[REG_MD1_CFG] & (1u << 5)) != 0);

    tracker::Lsm6dsv::MotionWakeSource source;
    CHECK(ctx, imu.readMotionWakeSource(source));
    CHECK(ctx, source.wakeUp);
    CHECK(ctx, source.x && source.y && source.z);

    cfg.enabled = false;
    CHECK(ctx, imu.configureMotionWake(cfg));
    CHECK(ctx, (bus.reg_[REG_MD1_CFG] & (1u << 5)) == 0);
    CHECK(ctx, (bus.reg_[REG_TAP_CFG0] & 0x11u) == 0);
    CHECK(ctx, (bus.reg_[REG_WAKE_UP_THS] & 0x3fu) == 0u);
    CHECK(ctx, (bus.reg_[REG_WAKE_UP_DUR] & 0x03u) == 0u);

    cfg.enabled = true;
    cfg.threshold = 64;
    CHECK(ctx, !imu.configureMotionWake(cfg));
    CHECK(ctx, imu.lastError() == tracker::Lsm6dsv::Error::InvalidConfig);

    return ctx.finish("lsm6dsv_motion_wake_config");
}
