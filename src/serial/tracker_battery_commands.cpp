#include "serial/tracker_battery_commands.hpp"

#include "runtime/battery_runtime.hpp"
#include "serial/tracker_serial_parse.hpp"
#include "serial/tracker_serial_print.hpp"
#include "defines.h"

namespace tracker {
namespace {

void printBatteryHelp(Stream& out) {
    out.println("battery status");
    out.println("battery reset       - reset battery runtime counters/filter");
}

void printBatteryStatus(Stream& out, const BatteryRuntime& battery) {
    const BatteryRuntimeStatus s = battery.status();
    out.println("# BATTERY STATUS");
    out.print("configured="); out.println(s.configured ? "yes" : "no");
    out.print("enabled="); out.println(s.enabled ? "yes" : "no");
    out.print("pin="); out.println(s.adcPin);
#if defined(CONFIG_IDF_TARGET_ESP32C3) && (TRACKER_BATTERY_ADC_PIN >= 0) && (TRACKER_BATTERY_ADC_PIN <= 4)
    out.println("adc_backend=esp32c3_adc1_mv");
    out.println("adc1_c3=yes");
#else
    out.println("adc_backend=arduino_analog_mv");
    out.println("adc1_c3=no");
#endif
    out.print("sample_interval_ms="); out.println((uint32_t)TRACKER_BATTERY_ADC_SAMPLE_INTERVAL_MS);
    out.print("oversample_count="); out.println((uint32_t)TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT);
    out.print("discard_count="); out.println((uint32_t)TRACKER_BATTERY_ADC_DISCARD_COUNT);
    out.print("ema_alpha="); out.println((float)TRACKER_BATTERY_ADC_EMA_ALPHA, 3);
    out.print("max_filter_step_v="); out.println((float)TRACKER_BATTERY_MAX_FILTER_STEP_V, 3);
    out.print("adc_max_mv="); out.println((uint32_t)TRACKER_BATTERY_ADC_MAX_MV);
    out.print("present_min_v="); out.println((float)TRACKER_BATTERY_PRESENT_MIN_VOLTAGE, 3);
    out.print("present_max_v="); out.println((float)TRACKER_BATTERY_PRESENT_MAX_VOLTAGE, 3);
    out.print("last_read_ok="); out.println(s.lastReadOk ? "yes" : "no");
    out.print("present="); out.println(s.present ? "yes" : "no");
    out.print("filtered_valid="); out.println(s.filteredValid ? "yes" : "no");
    out.print("last_adc_mv="); out.println(s.lastAdcMillivolts);
    out.print("last_adc_v="); out.println(s.lastAdcVoltage, 4);
    out.print("voltage_v="); out.println(s.voltage, 4);
    out.print("percentage="); out.println(s.percentage, 2);
    out.print("samples="); out.println(s.samples);
    out.print("read_failures="); out.println(s.readFailures);
    out.print("invalid_samples="); out.println(s.invalidSamples);
    out.print("no_battery_samples="); out.println(s.noBatterySamples);
    out.print("glitch_rejected_samples="); out.println(s.glitchRejectedSamples);
    out.print("last_sample_ms="); out.println(s.lastSampleMs);
}

} // namespace

void trackerSerialDispatchBatteryCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = ctx.io ? *ctx.io : Serial;
    if (!ctx.batteryRuntime) {
        tracker_serial_detail::printErr(out, "battery runtime is not wired");
        return;
    }

    if (argc < 2 || tracker_serial_detail::eqIgnoreCase(argv[1], "status")) {
        printBatteryStatus(out, *ctx.batteryRuntime);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "help") || tracker_serial_detail::eqIgnoreCase(argv[1], "?")) {
        printBatteryHelp(out);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "reset")) {
        ctx.batteryRuntime->reset();
        tracker_serial_detail::printOk(out, "battery runtime counters/filter reset");
        return;
    }

    tracker_serial_detail::printErr(out, "unknown battery command; use battery help");
}

} // namespace tracker
