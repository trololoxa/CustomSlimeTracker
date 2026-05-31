#include "serial/tracker_perf_commands.hpp"

#include <Arduino.h>

#include "runtime/runtime_profiler.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "network/wifi_manager.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/imu_quality.hpp"
#include "serial/tracker_serial_parse.hpp"
#include "serial/tracker_serial_print.hpp"

namespace tracker {

namespace {

Stream& streamFor(TrackerSerialCommandContext& ctx) {
    return *ctx.io;
}

void printSlimeQuick(Stream& out, const SlimeVROutputRuntime* slime) {
    if (slime == nullptr) return;
    const SlimeVROutputRuntimeStatus s = slime->status();
    out.println("# SLIMEVR PERF CORRELATION");
    out.print("slime_enabled="); out.println(s.enabled ? "yes" : "no");
    out.print("slime_server_found="); out.println(s.serverFound ? "yes" : "no");
    out.print("slime_rotation_rate_hz="); out.println(s.rotationRateHz);
    out.print("slime_rotation_sent="); out.println(s.rotationSent);
    out.print("slime_rotation_send_due="); out.println(s.rotationSendDue);
    out.print("slime_rotation_rate_limited="); out.println(s.rotationRateLimited);
    out.print("slime_rotation_no_snapshot="); out.println(s.rotationNoSnapshot);
    out.print("slime_rotation_duplicate_snapshot="); out.println(s.rotationDuplicateSnapshot);
    out.print("slime_rotation_send_failures="); out.println(s.rotationSendFailures);
    out.print("slime_send_failures="); out.println(s.sendFailures);
    out.print("slime_service_updates="); out.println(s.serviceUpdates);
    out.print("slime_service_skips="); out.println(s.serviceSkips);
    out.print("slime_prepared_output_available="); out.println(s.preparedOutputAvailable ? "yes" : "no");
    out.print("slime_last_rotation_age_ms="); out.println(s.lastRotationMs == 0 ? 0 : millis() - s.lastRotationMs);
    out.print("slime_last_rotation_snapshot_age_us="); out.println(s.lastRotationSnapshotAgeUs);
    out.print("slime_last_rotation_confidence="); out.println(s.lastRotationConfidence, 4);
}


void printThermalSystemQuick(Stream& out, TrackerSerialCommandContext& ctx) {
    out.println("# THERMAL/SYSTEM PERF CORRELATION");
    float imuTempC = 0.0f;
    bool imuTempValid = false;
    if (ctx.fifo != nullptr) {
        const auto& fs = ctx.fifo->stats();
        imuTempC = fs.latestTempC;
        imuTempValid = fs.latestTempValid;
    } else if (ctx.lastCalibratedSample != nullptr) {
        imuTempC = ctx.lastCalibratedSample->temp_c;
        imuTempValid = true;
    }
    out.print("imu_temp_valid="); out.println(imuTempValid ? "yes" : "no");
    out.print("imu_temp_c="); out.println(imuTempC, 2);
#if defined(ARDUINO_ARCH_ESP32)
    out.print("cpu_freq_mhz="); out.println(getCpuFrequencyMhz());
    out.print("heap_free_bytes="); out.println(ESP.getFreeHeap());
    out.print("heap_min_free_bytes="); out.println(ESP.getMinFreeHeap());
    out.print("heap_max_alloc_bytes="); out.println(ESP.getMaxAllocHeap());
#else
    out.println("cpu_freq_mhz=unknown");
    out.println("heap_free_bytes=unknown");
    out.println("heap_min_free_bytes=unknown");
    out.println("heap_max_alloc_bytes=unknown");
#endif
    if (ctx.quality != nullptr) {
        const auto& qc = ctx.quality->counters();
        out.print("quality_samples="); out.println(qc.samples);
        out.print("quality_ahrs_skipped_samples="); out.println(qc.ahrsSkippedSamples);
        out.print("quality_accel_correction_disabled_samples="); out.println(qc.accelCorrectionDisabledSamples);
        out.print("quality_estimated_dropped_samples="); out.println(qc.estimatedDroppedSamples);
    }
}

void printWifiQuick(Stream& out, const TrackerWifiManager* wifi) {
    if (wifi == nullptr) return;
    const TrackerWifiManagerStatus s = wifi->status();
    out.println("# WIFI PERF CORRELATION");
    out.print("wifi_state="); out.println(trackerWifiStateName(s.state));
    out.print("wifi_link_status="); out.println(wifiLinkStatusName(s.linkStatus));
    out.print("wifi_connected="); out.println(s.connected ? "yes" : "no");
    out.print("wifi_rssi_dbm="); out.println(s.rssiDbm);
    out.print("wifi_ps_mode="); out.println(wifiPowerSaveModeName(s.powerSaveMode));
    out.print("wifi_tx_power_valid="); out.println(s.txPowerValid ? "yes" : "no");
    out.print("wifi_tx_power_qdbm="); out.println(s.txPowerQuarterDbm);
    out.print("wifi_attempts="); out.println(s.attempts);
    out.print("wifi_connect_timeouts="); out.println(s.connectTimeouts);
    out.print("wifi_disconnects="); out.println(s.disconnects);
    out.print("wifi_connected_age_ms="); out.println(s.connectedSinceMs == 0u ? 0u : millis() - s.connectedSinceMs);
}

void printUsage(Stream& out) {
    out.println("perf status | top | on | off | reset");
    out.println("  perf on: enable rolling loop-section timing from serial/telnet");
    out.println("  perf status: section avg/max/slow + temp/heap/wifi/SlimeVR counters");
}

} // namespace

void trackerSerialDispatchPerfCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = streamFor(ctx);
    if (ctx.runtimeProfiler == nullptr) {
        tracker_serial_detail::printErr(out, "runtime profiler not wired/compiled");
        return;
    }

    if (argc < 2 || tracker_serial_detail::eqIgnoreCase(argv[1], "status")) {
        ctx.runtimeProfiler->printStatus(out, millis());
        printThermalSystemQuick(out, ctx);
        printWifiQuick(out, ctx.wifiManager);
        printSlimeQuick(out, ctx.slimevrRuntime);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "top")) {
        out.println("# RUNTIME PERF TOP");
        ctx.runtimeProfiler->printTop(out, millis());
        printThermalSystemQuick(out, ctx);
        printWifiQuick(out, ctx.wifiManager);
        printSlimeQuick(out, ctx.slimevrRuntime);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "on") ||
        tracker_serial_detail::eqIgnoreCase(argv[1], "enable")) {
        ctx.runtimeProfiler->setEnabled(true, millis());
        tracker_serial_detail::printOk(out, "runtime profiler enabled and reset");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "off") ||
        tracker_serial_detail::eqIgnoreCase(argv[1], "disable")) {
        ctx.runtimeProfiler->setEnabled(false, millis());
        tracker_serial_detail::printOk(out, "runtime profiler disabled");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "reset")) {
        ctx.runtimeProfiler->reset(millis());
        tracker_serial_detail::printOk(out, "runtime profiler reset");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "help")) {
        printUsage(out);
        return;
    }

    tracker_serial_detail::printErr(out, "usage: perf status|top|on|off|reset");
}

} // namespace tracker
