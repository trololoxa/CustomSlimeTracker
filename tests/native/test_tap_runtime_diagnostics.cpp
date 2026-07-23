#include "test_common.hpp"

#include "connection/lsm6dsv_driver.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "runtime/tap_runtime_controller.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

using namespace tracker;

namespace {

class MockTransport final : public Lsm6dsvTransport {
public:
    std::array<uint8_t, 256> reg{};

    bool read(uint8_t addr, uint8_t* dst, size_t len) override {
        if (!dst || len == 0) return false;
        for (size_t i = 0; i < len; ++i) {
            dst[i] = reg[static_cast<uint8_t>(addr + i)];
        }
        return true;
    }

    bool write(uint8_t addr, const uint8_t* src, size_t len) override {
        if (!src || len == 0) return false;
        for (size_t i = 0; i < len; ++i) {
            reg[static_cast<uint8_t>(addr + i)] = src[i];
        }
        return true;
    }

    void delayMs(uint32_t) override {}
};

struct CapturedEvents {
    std::array<TapDiagnosticEvent, 24> events{};
    size_t count = 0;

    static void sink(const TapDiagnosticEvent& event, void* user) {
        auto* self = static_cast<CapturedEvents*>(user);
        if (self && self->count < self->events.size()) {
            self->events[self->count++] = event;
        }
    }

    bool saw(TapDiagnosticKind kind) const {
        for (size_t i = 0; i < count; ++i) {
            if (events[i].kind == kind) return true;
        }
        return false;
    }

    const TapDiagnosticEvent* first(TapDiagnosticKind kind) const {
        for (size_t i = 0; i < count; ++i) {
            if (events[i].kind == kind) return &events[i];
        }
        return nullptr;
    }
};

} // namespace

int main() {
    TestContext ctx;

    constexpr uint8_t REG_TAP_SRC = 0x46;
    constexpr uint8_t TAP_IA = 1u << 6;
    constexpr uint8_t SINGLE_TAP = 1u << 5;
    constexpr uint8_t X_TAP = 1u << 2;

    MockTransport bus;
    Lsm6dsv imu(bus);
    SlimeVROutputRuntime slimevr;
    TapRuntimeController tap;
    CapturedEvents captured;

    tap.setDiagnosticSink(CapturedEvents::sink, &captured);
    tap.setDiagnosticLogging(true);
    tap.begin(imu, slimevr);

    TapRuntimeConfig config;
    config.enabled = true;
    config.pollIntervalMs = 1;
    config.aggregationWindowMs = 100;
    config.duplicateSuppressMs = 10;
    config.minCount = 2;
    CHECK(ctx, tap.configure(config));
    CHECK(ctx, captured.saw(TapDiagnosticKind::HardwareConfigured));
    tap.setPhysicalTapUserAction(SlimeVRUserAction::YawReset);
    CHECK(ctx, tap.status().physicalTapUserAction == SlimeVRUserAction::YawReset);
    tap.setPhysicalTapUserAction(SlimeVRUserAction::None);

    captured.count = 0;
    bus.reg[REG_TAP_SRC] = static_cast<uint8_t>(TAP_IA | SINGLE_TAP | X_TAP);
    CHECK(ctx, tap.update(100));
    CHECK(ctx, captured.saw(TapDiagnosticKind::SourceObserved));
    CHECK(ctx, captured.saw(TapDiagnosticKind::PhysicalTap));
    CHECK(ctx, captured.saw(TapDiagnosticKind::AccumulatorQueued));

    const TapDiagnosticEvent* source = captured.first(TapDiagnosticKind::SourceObserved);
    CHECK(ctx, source != nullptr);
    if (source) {
        CHECK(ctx, source->rawSource == static_cast<uint8_t>(TAP_IA | SINGLE_TAP | X_TAP));
        CHECK(ctx, source->tapDetected);
        CHECK(ctx, source->singleTap);
        CHECK(ctx, !source->doubleTap);
        CHECK(ctx, source->x);
    }

    captured.count = 0;
    bus.reg[REG_TAP_SRC] = 0;
    CHECK(ctx, tap.update(200));
    CHECK(ctx, captured.saw(TapDiagnosticKind::SuppressedBelowMin));

    captured.count = 0;
    bus.reg[REG_TAP_SRC] = static_cast<uint8_t>(TAP_IA | SINGLE_TAP | X_TAP);
    CHECK(ctx, tap.update(300));
    bus.reg[REG_TAP_SRC] = static_cast<uint8_t>(TAP_IA | SINGLE_TAP | X_TAP);
    CHECK(ctx, tap.update(320));
    bus.reg[REG_TAP_SRC] = 0;
    CHECK(ctx, tap.update(420));
    CHECK(ctx, captured.saw(TapDiagnosticKind::PacketReady));
    CHECK(ctx, captured.saw(TapDiagnosticKind::SlimeVrNoServer));

    return ctx.finish("tap_runtime_diagnostics");
}
