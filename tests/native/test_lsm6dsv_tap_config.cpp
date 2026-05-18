#include "test_common.hpp"

#include "connection/lsm6dsv_driver.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

using namespace tracker;

namespace {

class MockTransport final : public Lsm6dsvTransport {
public:
    std::array<uint8_t, 256> reg{};
    bool failReads = false;
    bool failWrites = false;

    bool read(uint8_t addr, uint8_t* dst, size_t len) override {
        if (failReads || dst == nullptr || len == 0) return false;
        for (size_t i = 0; i < len; ++i) {
            dst[i] = reg[static_cast<uint8_t>(addr + i)];
        }
        return true;
    }

    bool write(uint8_t addr, const uint8_t* src, size_t len) override {
        if (failWrites || src == nullptr || len == 0) return false;
        for (size_t i = 0; i < len; ++i) {
            reg[static_cast<uint8_t>(addr + i)] = src[i];
        }
        return true;
    }

    void delayMs(uint32_t) override {}
};

} // namespace

int main() {
    TestContext ctx;

    constexpr uint8_t REG_FUNCTIONS_ENABLE = 0x50;
    constexpr uint8_t REG_TAP_CFG0 = 0x56;
    constexpr uint8_t REG_WAKE_UP_THS = 0x5B;
    constexpr uint8_t REG_MD1_CFG = 0x5E;
    constexpr uint8_t FUNCTIONS_TIMESTAMP_EN = 1u << 6;
    constexpr uint8_t FUNCTIONS_INTERRUPTS_ENABLE = 1u << 7;

    MockTransport bus;
    bus.reg[REG_FUNCTIONS_ENABLE] = FUNCTIONS_TIMESTAMP_EN;

    Lsm6dsv imu(bus);
    Lsm6dsv::TapConfig cfg;
    cfg.enabled = true;
    cfg.enableDoubleTap = false;
    cfg.routeDoubleTapToInt1 = false;
    cfg.thresholdX = 4;
    cfg.thresholdY = 4;
    cfg.thresholdZ = 4;

    CHECK(ctx, imu.configureTapDetection(cfg));
    CHECK(ctx, (bus.reg[REG_FUNCTIONS_ENABLE] & FUNCTIONS_TIMESTAMP_EN) != 0);
    CHECK(ctx, (bus.reg[REG_FUNCTIONS_ENABLE] & FUNCTIONS_INTERRUPTS_ENABLE) != 0);
    CHECK(ctx, (bus.reg[REG_WAKE_UP_THS] & 0x80u) == 0); // hardware double-tap disabled
    CHECK(ctx, (bus.reg[REG_MD1_CFG] & (1u << 6)) != 0); // INT1 single-tap routed
    CHECK(ctx, (bus.reg[REG_MD1_CFG] & (1u << 3)) == 0); // INT1 double-tap not routed

    Lsm6dsv::TapRegisterVerification verify;
    CHECK(ctx, imu.verifyTapDetection(cfg, verify));
    CHECK(ctx, verify.ok);
    CHECK(ctx, verify.actual.functionsEnable == (FUNCTIONS_TIMESTAMP_EN | FUNCTIONS_INTERRUPTS_ENABLE));
    CHECK(ctx, verify.expected.functionsEnable == FUNCTIONS_INTERRUPTS_ENABLE);
    CHECK(ctx, verify.mask.functionsEnable == FUNCTIONS_INTERRUPTS_ENABLE);

    bus.reg[REG_FUNCTIONS_ENABLE] &= static_cast<uint8_t>(~FUNCTIONS_INTERRUPTS_ENABLE);
    CHECK(ctx, imu.verifyTapDetection(cfg, verify));
    CHECK(ctx, !verify.ok);

    bus.reg[REG_FUNCTIONS_ENABLE] |= FUNCTIONS_INTERRUPTS_ENABLE;
    cfg.enabled = false;
    CHECK(ctx, imu.configureTapDetection(cfg));
    CHECK(ctx, (bus.reg[REG_FUNCTIONS_ENABLE] & FUNCTIONS_TIMESTAMP_EN) != 0);
    CHECK(ctx, (bus.reg[REG_FUNCTIONS_ENABLE] & FUNCTIONS_INTERRUPTS_ENABLE) != 0);
    CHECK(ctx, (bus.reg[REG_TAP_CFG0] & 0x2Fu) == 0);
    CHECK(ctx, (bus.reg[REG_WAKE_UP_THS] & 0x80u) == 0);
    CHECK(ctx, (bus.reg[REG_MD1_CFG] & ((1u << 6) | (1u << 3))) == 0);

    return ctx.finish("lsm6dsv_tap_config");
}
