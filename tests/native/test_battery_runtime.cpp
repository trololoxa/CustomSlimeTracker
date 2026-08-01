#include "test_common.hpp"

#include <cstdint>

#include "runtime/battery_runtime.hpp"
#include "runtime/battery_adc_batch_sampler.hpp"

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

    adc.millivolts = 0; // One low ADC glitch must not collapse an existing estimate.
    rt.update(2001);
    st = rt.status();
    CHECK(ctx, st.present);
    CHECK(ctx, st.filteredValid);
    CHECK(ctx, st.noBatterySamples == 1u);
    CHECK(ctx, st.glitchRejectedSamples == 1u);
    CHECK_NEAR(ctx, st.voltage, 3.75f, 1.0e-6f);
    CHECK_NEAR(ctx, st.percentage, 50.0f, 1.0e-4f);
    CHECK(ctx, rt.telemetry(voltage, percentage));
    CHECK_NEAR(ctx, voltage, 3.75f, 1.0e-6f);
    CHECK_NEAR(ctx, percentage, 50.0f, 1.0e-4f);

    adc.ok = false;
    rt.update(3001);
    st = rt.status();
    CHECK(ctx, !st.lastReadOk);
    CHECK(ctx, st.readFailures == 1u);
    CHECK(ctx, st.present);
    CHECK(ctx, rt.telemetry(voltage, percentage));
    CHECK_NEAR(ctx, voltage, 3.75f, 1.0e-6f);
    CHECK_NEAR(ctx, percentage, 50.0f, 1.0e-4f);

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
    adc.millivolts = 5000; // Implausibly high after divider -> rejected, previous estimate kept.
    rt.update(7001);
    st = rt.status();
    CHECK(ctx, st.invalidSamples == 1u);
    CHECK(ctx, st.glitchRejectedSamples == 2u);
    CHECK(ctx, st.present);
    CHECK(ctx, rt.telemetry(voltage, percentage));
    CHECK_NEAR(ctx, voltage, 3.80f, 1.0e-6f);
    CHECK_NEAR(ctx, percentage, 55.55556f, 1.0e-4f);

    BatteryRuntime absent;
    absent.begin(FakeAdc::read, &adc);
    absent.configure(baseConfig());
    adc.ok = true;
    adc.millivolts = 0;
    absent.update(1);
    st = absent.status();
    CHECK(ctx, !st.present);
    CHECK(ctx, !st.filteredValid);
    CHECK(ctx, st.noBatterySamples == 1u);
    CHECK(ctx, absent.telemetry(voltage, percentage));
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


    struct SequenceAdc {
        uint16_t values[10] = {100u, 200u, 1000u, 1100u, 1200u,
                               1300u, 1400u, 1500u, 1600u, 10000u};
        uint8_t index = 0u;
        static bool read(uint16_t& out, void* user) {
            auto& self = *static_cast<SequenceAdc*>(user);
            if (self.index >= 10u) return false;
            out = self.values[self.index++];
            return true;
        }
    } sequence;
    BatteryAdcBatchSampler sampler;
    sampler.begin(SequenceAdc::read, &sequence);
    BatteryAdcBatchSamplerConfig samplerCfg;
    samplerCfg.totalReads = 10u;
    samplerCfg.discardReads = 2u;
    sampler.configure(samplerCfg);
    CHECK(ctx, sampler.request());
    for (uint8_t i = 0u; i < 5u; ++i) {
        CHECK(ctx, sampler.service(2u));
        CHECK(ctx, sequence.index <= static_cast<uint8_t>((i + 1u) * 2u));
    }
    CHECK(ctx, !sampler.active());
    CHECK(ctx, sampler.resultReady());
    CHECK(ctx, sampler.status().maxReadsPerService == 2u);
    CHECK(ctx, sampler.status().serviceCalls == 5u);
    uint16_t batchMv = 0u;
    CHECK(ctx, sampler.consume(batchMv));
    // After discarding 100/200 and trimming one low/high sample from the
    // remaining eight values, the exact legacy trimmed mean is 1350 mV.
    CHECK(ctx, batchMv == 1350u);
    CHECK(ctx, !sampler.resultReady());

    return ctx.finish("battery_runtime");
}
