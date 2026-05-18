#include "test_common.hpp"

#include "runtime/status_led_runtime.hpp"

using namespace tracker;

class FakeLedSink final : public IStatusLedSink {
public:
    void begin(int8_t ledPin, bool ledActiveLow) override {
        begun = true;
        pin = ledPin;
        activeLow = ledActiveLow;
        writes = 0;
        on = false;
    }

    void write(bool v) override {
        on = v;
        ++writes;
    }

    bool begun = false;
    int8_t pin = -1;
    bool activeLow = true;
    bool on = false;
    uint32_t writes = 0;
};

int main() {
    TestContext ctx;

    StatusLedRuntimeConfig cfg;
    cfg.enabled = true;
    cfg.pin = 8;
    cfg.activeLow = true;
    cfg.updateIntervalMs = 1;
    cfg.normalBlinkPeriodMs = 10000;
    cfg.normalBlinkOnMs = 30;
    cfg.shortBlinkOnMs = 80;
    cfg.shortBlinkOffMs = 160;
    cfg.longBlinkOnMs = 450;
    cfg.longBlinkOffMs = 250;
    cfg.errorBlinkPeriodMs = 5000;
    cfg.identifyBlinkOnMs = 100;
    cfg.identifyBlinkOffMs = 100;

    FakeLedSink sink;
    StatusLedRuntime led;
    led.begin(sink, cfg);
    CHECK(ctx, sink.begun);
    CHECK(ctx, sink.pin == 8);
    CHECK(ctx, sink.activeLow);

    led.setMode(TrackerStatusLedMode::Normal, 0);
    led.update(0);
    CHECK(ctx, sink.on);
    led.update(31);
    CHECK(ctx, !sink.on);
    led.update(10000);
    CHECK(ctx, sink.on);

    led.setMode(TrackerStatusLedMode::ConnectionError, 1000);
    led.update(1000);
    CHECK(ctx, sink.on);
    led.update(1451);
    CHECK(ctx, !sink.on);
    led.update(1700);
    CHECK(ctx, sink.on);
    led.update(2400);
    CHECK(ctx, sink.on); // third long blink is still active
    led.update(3101);
    CHECK(ctx, !sink.on); // cluster finished until next 5 s period

    led.identify(5000, 1000);
    led.update(5000);
    CHECK(ctx, led.status().effectiveMode == TrackerStatusLedMode::Identify);
    CHECK(ctx, sink.on);
    led.update(5101);
    CHECK(ctx, !sink.on);
    led.update(6001);
    CHECK(ctx, led.effectiveMode(6001) == TrackerStatusLedMode::ConnectionError);

    led.setManualOverride(TrackerStatusLedMode::ManualOn, 7000);
    led.update(7000);
    CHECK(ctx, led.status().manualOverride);
    CHECK(ctx, sink.on);
    led.clearManualOverride(8000);
    led.update(8000);
    CHECK(ctx, !led.status().manualOverride);

    TrackerStatusLedMode parsed = TrackerStatusLedMode::Off;
    CHECK(ctx, trackerStatusLedModeFromName("wifi", parsed));
    CHECK(ctx, parsed == TrackerStatusLedMode::WifiConnecting);
    CHECK(ctx, trackerStatusLedModeFromName("sensor_error", parsed));
    CHECK(ctx, parsed == TrackerStatusLedMode::SensorError);
    CHECK(ctx, !trackerStatusLedModeFromName("nope", parsed));

    return ctx.finish("status_led_runtime");
}
