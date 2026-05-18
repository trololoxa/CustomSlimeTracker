#include "test_common.hpp"

#include <cstdint>

#include "runtime/battery_runtime.hpp"

using namespace tracker;

namespace {

struct FakeAdc {
    bool ok = true;
    uint16_t millivolts = 0;
    uint32_t calls = 0;

    static bool read(uint16_t& outMillivolts, void* user) {
        auto* self = static_cast<FakeAdc*>(user);
        ++self->calls;
        if (!self->ok) return false;
        outMillivolts = self->millivolts;
        return true;
    }
};

BatteryRuntimeConfig baseConfig() {
    BatteryRuntimeConfig cfg;
    cfg.enabled = true;
    cfg.adcPin = 4;
    cfg.rTopOhms = 180000.0f;
    cfg.rBottomOhms = 180000.0f;
    cfg.emptyVoltage = 3.30f;
    cfg.fullVoltage = 4.20f;
    cfg.presentVoltageMin = 1.00f;
    cfg.emaAlpha = 1.0f;
    cfg.sampleIntervalMs = 1000;
    cfg.startupSamples = 1;
    return cfg;
}

} // namespace

int main() {
    TestContext ctx;

    CHECK_NEAR(ctx, BatteryRuntime::dividerRatio(180000.0f, 180000.0f), 2.0f, 1.0e-6f);
    CHECK_NEAR(ctx, BatteryRuntime::batteryVoltageFromAdcMillivolts(2100, 180000.0f, 180000.0f), 4.20f, 1.0e-6f);
    CHECK_NEAR(ctx, BatteryRuntime::percentageFromVoltage(3.30f, 3.30f, 4.20f), 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, BatteryRuntime::percentageFromVoltage(3.75f, 3.30f, 4.20f), 50.0f, 1.0e-4f);
    CHECK_NEAR(ctx, BatteryRuntime::percentageFromVoltage(4.20f, 3.30f, 4.20f), 100.0f, 1.0e-6f);

    FakeAdc adc;
    BatteryRuntime rt;
    rt.begin(FakeAdc::read, &adc);
    rt.configure(baseConfig());

    adc.millivolts = 2100; // 4.20 V after the 1:1 divider.
    rt.update(1);
    BatteryRuntimeStatus st = rt.status();
    CHECK(ctx, st.configured);
    CHECK(ctx, st.present);
    CHECK(ctx, st.filteredValid);
    CHECK(ctx, st.lastReadOk);
    CHECK(ctx, st.lastAdcMillivolts == 2100);
    CHECK_NEAR(ctx, st.voltage, 4.20f, 1.0e-6f);
    CHECK_NEAR(ctx, st.percentage, 100.0f, 1.0e-6f);
    float voltage = -1.0f;
    float percentage = -1.0f;
    CHECK(ctx, rt.telemetry(voltage, percentage));
    CHECK_NEAR(ctx, voltage, 4.20f, 1.0e-6f);
    CHECK_NEAR(ctx, percentage, 100.0f, 1.0e-6f);

    adc.millivolts = 1875; // 3.75 V => ~50%.
    rt.update(1001);
    st = rt.status();
    CHECK(ctx, st.present);
    CHECK_NEAR(ctx, st.voltage, 3.75f, 1.0e-6f);
    CHECK_NEAR(ctx, st.percentage, 50.0f, 1.0e-4f);

    adc.millivolts = 0; // Battery absent / divider pulled to ground.
    rt.update(2001);
    st = rt.status();
    CHECK(ctx, !st.present);
    CHECK(ctx, !st.filteredValid);
    CHECK(ctx, st.noBatterySamples == 1u);
    CHECK_NEAR(ctx, st.voltage, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, st.percentage, 0.0f, 1.0e-6f);
    CHECK(ctx, rt.telemetry(voltage, percentage));
    CHECK_NEAR(ctx, voltage, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, percentage, 0.0f, 1.0e-6f);

    adc.ok = false;
    rt.update(3001);
    st = rt.status();
    CHECK(ctx, !st.lastReadOk);
    CHECK(ctx, st.readFailures == 1u);
    CHECK(ctx, !st.present);
    CHECK(ctx, rt.telemetry(voltage, percentage));
    CHECK_NEAR(ctx, voltage, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, percentage, 0.0f, 1.0e-6f);

    adc.ok = true;
    adc.millivolts = 1900; // 3.80 V valid again.
    rt.update(4001);
    CHECK(ctx, rt.status().present);
    adc.ok = false; // Transient failure keeps the last accepted estimate.
    rt.update(6001);
    st = rt.status();
    CHECK(ctx, !st.lastReadOk);
    CHECK(ctx, st.readFailures == 2u);
    CHECK(ctx, st.present);
    CHECK(ctx, rt.telemetry(voltage, percentage));
    CHECK_NEAR(ctx, voltage, 3.80f, 1.0e-6f);

    adc.ok = true;
    adc.millivolts = 5000; // Implausibly high after divider -> invalid, safe 0%.
    rt.update(7001);
    st = rt.status();
    CHECK(ctx, st.invalidSamples == 1u);
    CHECK(ctx, !st.present);
    CHECK(ctx, rt.telemetry(voltage, percentage));
    CHECK_NEAR(ctx, voltage, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, percentage, 0.0f, 1.0e-6f);

    BatteryRuntime disabled;
    disabled.begin(FakeAdc::read, &adc);
    BatteryRuntimeConfig disabledCfg = baseConfig();
    disabledCfg.enabled = false;
    disabled.configure(disabledCfg);
    CHECK(ctx, !disabled.telemetry(voltage, percentage));
    CHECK_NEAR(ctx, voltage, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, percentage, 0.0f, 1.0e-6f);

    return ctx.finish("battery_runtime");
}
