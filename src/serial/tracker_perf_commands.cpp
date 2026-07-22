#include "serial/tracker_perf_commands.hpp"

#include <Arduino.h>

#include "runtime/runtime_profiler.hpp"
#include "runtime/fifo_runtime_processor.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "runtime/tracking_state_controller.hpp"
#include "network/wifi_manager.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/imu_quality.hpp"
#include "serial/tracker_serial_parse.hpp"
#include "serial/tracker_serial_print.hpp"

namespace tracker {

namespace {

struct TrackingPerfBaseline {
    bool valid = false;
    uint32_t startMs = 0;
    Lsm6dsvFifoReader::DrainStats fifo;
    ImuQualityCounters quality;
    FifoRuntimeQueueStats queue;
    TrackingStateController::Snapshot tracking;
    SlimeVROutputRuntimeStatus slime;
};

TrackingPerfBaseline g_trackingPerfBaseline;

uint32_t deltaU32(uint32_t now, uint32_t before) {
    return now - before;
}

float ratePerSecond(uint32_t count, uint32_t windowMs) {
    if (windowMs == 0u) return 0.0f;
    return 1000.0f * static_cast<float>(count) / static_cast<float>(windowMs);
}

void captureTrackingBaseline(TrackerSerialCommandContext& ctx, uint32_t nowMs) {
    g_trackingPerfBaseline = TrackingPerfBaseline{};
    g_trackingPerfBaseline.valid = true;
    g_trackingPerfBaseline.startMs = nowMs;
    if (ctx.fifo) g_trackingPerfBaseline.fifo = ctx.fifo->stats();
    if (ctx.quality) g_trackingPerfBaseline.quality = ctx.quality->counters();
    if (ctx.fifoRuntime) g_trackingPerfBaseline.queue = ctx.fifoRuntime->queueStats();
    if (ctx.trackingState) g_trackingPerfBaseline.tracking = ctx.trackingState->snapshot();
    if (ctx.slimevrRuntime) g_trackingPerfBaseline.slime = ctx.slimevrRuntime->status();
}

void printTrackingPerf(Stream& out, TrackerSerialCommandContext& ctx) {
    if (ctx.serviceNonCliRuntime) {
        (void)ctx.serviceNonCliRuntime(ctx.serviceNonCliRuntimeUser);
    }

    const uint32_t nowMs = millis();
    if (!g_trackingPerfBaseline.valid) {
        // Before the first explicit reset, report counters since boot rather
        // than silently capturing a baseline that makes every delta zero.
        g_trackingPerfBaseline = TrackingPerfBaseline{};
        g_trackingPerfBaseline.valid = true;
        g_trackingPerfBaseline.startMs = 0u;
    }

    const uint32_t windowMs = nowMs - g_trackingPerfBaseline.startMs;
    const auto fifo = ctx.fifo ? ctx.fifo->stats() : Lsm6dsvFifoReader::DrainStats{};
    const auto quality = ctx.quality ? ctx.quality->counters() : ImuQualityCounters{};
    const auto queue = ctx.fifoRuntime ? ctx.fifoRuntime->queueStats() : FifoRuntimeQueueStats{};
    const auto tracking = ctx.trackingState
        ? ctx.trackingState->snapshot()
        : TrackingStateController::Snapshot{};
    const auto slime = ctx.slimevrRuntime
        ? ctx.slimevrRuntime->status()
        : SlimeVROutputRuntimeStatus{};

    const uint32_t rotationDue = deltaU32(slime.rotationSendDue, g_trackingPerfBaseline.slime.rotationSendDue);
    const uint32_t rotationSent = deltaU32(slime.rotationSent, g_trackingPerfBaseline.slime.rotationSent);
    const float deliveryPct = rotationDue == 0u
        ? 0.0f
        : 100.0f * static_cast<float>(rotationSent) / static_cast<float>(rotationDue);

    out.println("# TRACKING PERF");
    out.print("window_ms="); out.println(windowMs);
    out.print("fifo_overrun_delta="); out.println(deltaU32(fifo.overrunEvents, g_trackingPerfBaseline.fifo.overrunEvents));
    out.print("fifo_full_delta="); out.println(deltaU32(fifo.fullEvents, g_trackingPerfBaseline.fifo.fullEvents));
    out.print("fifo_max_unread_words_seen="); out.println(fifo.maxUnreadWordsSeen);
    out.print("quality_dropped_delta="); out.println(deltaU32(quality.estimatedDroppedSamples, g_trackingPerfBaseline.quality.estimatedDroppedSamples));
    out.print("quality_recovery_delta="); out.println(deltaU32(quality.fifoRecoveryRequests, g_trackingPerfBaseline.quality.fifoRecoveryRequests));
    out.print("recovery_overrun_delta="); out.println(deltaU32(quality.fifoRecoveryOverrunRequests, g_trackingPerfBaseline.quality.fifoRecoveryOverrunRequests));
    out.print("recovery_full_delta="); out.println(deltaU32(quality.fifoRecoveryFullRequests, g_trackingPerfBaseline.quality.fifoRecoveryFullRequests));
    out.print("recovery_timestamp_backwards_delta="); out.println(deltaU32(quality.fifoRecoveryTimestampBackwardsRequests, g_trackingPerfBaseline.quality.fifoRecoveryTimestampBackwardsRequests));
    out.print("runtime_raw_queue_depth="); out.println(ctx.fifoRuntime ? ctx.fifoRuntime->rawQueueDepth() : 0u);
    out.print("runtime_raw_queue_high_water="); out.println(queue.rawQueueHighWater);
    out.print("runtime_raw_queue_overflow_delta="); out.println(deltaU32(queue.rawQueueOverflow, g_trackingPerfBaseline.queue.rawQueueOverflow));
    out.print("runtime_mag_queue_depth="); out.println(ctx.fifoRuntime ? ctx.fifoRuntime->magQueueDepth() : 0u);
    out.print("runtime_mag_queue_high_water="); out.println(queue.magQueueHighWater);
    out.print("runtime_mag_queue_overflow_delta="); out.println(deltaU32(queue.magQueueOverflow, g_trackingPerfBaseline.queue.magQueueOverflow));
    out.print("tracking_recovery_enter_delta="); out.println(deltaU32(tracking.recoveryEnterCount, g_trackingPerfBaseline.tracking.recoveryEnterCount));
    out.print("tracking_recovery_bootstrap_bypass_delta="); out.println(deltaU32(tracking.recoveryBootstrapBypassCount, g_trackingPerfBaseline.tracking.recoveryBootstrapBypassCount));
    out.print("tracking_soft_recovery_active="); out.println(tracking.softRecoveryActive ? "yes" : "no");
    out.print("tracking_soft_recovery_good_samples="); out.println(tracking.softRecoveryGoodSamples);
    out.print("tracking_soft_recovery_enter_delta="); out.println(deltaU32(tracking.softRecoveryEnterCount, g_trackingPerfBaseline.tracking.softRecoveryEnterCount));
    out.print("tracking_soft_recovery_complete_delta="); out.println(deltaU32(tracking.softRecoveryCompleteCount, g_trackingPerfBaseline.tracking.softRecoveryCompleteCount));
    out.print("tracking_recovery_last_reason="); out.println(trackingRecoveryReasonName(tracking.recoveryLastReason));
    out.print("tracking_recovery_fifo_quality_delta="); out.println(deltaU32(tracking.recoveryFifoQualityRequests, g_trackingPerfBaseline.tracking.recoveryFifoQualityRequests));
    out.print("tracking_recovery_timestamp_gap_delta="); out.println(deltaU32(tracking.recoveryTimestampGapRequests, g_trackingPerfBaseline.tracking.recoveryTimestampGapRequests));
    out.print("tracking_recovery_manual_reset_delta="); out.println(deltaU32(tracking.recoveryManualResetRequests, g_trackingPerfBaseline.tracking.recoveryManualResetRequests));
    out.print("tracking_recovery_blocking_operation_delta="); out.println(deltaU32(tracking.recoveryBlockingOperationRequests, g_trackingPerfBaseline.tracking.recoveryBlockingOperationRequests));
    out.print("tracking_recovery_reconfigure_delta="); out.println(deltaU32(tracking.recoveryRuntimeReconfigureRequests, g_trackingPerfBaseline.tracking.recoveryRuntimeReconfigureRequests));
    out.print("rotation_due_delta="); out.println(rotationDue);
    out.print("rotation_sent_delta="); out.println(rotationSent);
    out.print("rotation_sent_rate_hz="); out.println(ratePerSecond(rotationSent, windowMs), 3);
    out.print("rotation_delivery_pct="); out.println(deliveryPct, 2);
    out.print("rotation_missed_deadlines_delta="); out.println(deltaU32(slime.rotationMissedDeadlines, g_trackingPerfBaseline.slime.rotationMissedDeadlines));
    out.print("rotation_late_events_delta="); out.println(deltaU32(slime.rotationLateEvents, g_trackingPerfBaseline.slime.rotationLateEvents));
    out.print("rotation_lateness_max_ms="); out.println(slime.rotationLatenessMaxMs);
    out.print("rotation_no_snapshot_delta="); out.println(deltaU32(slime.rotationNoSnapshot, g_trackingPerfBaseline.slime.rotationNoSnapshot));
    out.print("rotation_send_failures_delta="); out.println(deltaU32(slime.rotationSendFailures, g_trackingPerfBaseline.slime.rotationSendFailures));
    out.print("acceleration_sent_delta="); out.println(deltaU32(slime.accelerationSent, g_trackingPerfBaseline.slime.accelerationSent));
    out.print("acceleration_skipped_invalid_delta="); out.println(deltaU32(slime.accelerationSkippedInvalid, g_trackingPerfBaseline.slime.accelerationSkippedInvalid));
    out.print("acceleration_skipped_configuration_delta="); out.println(deltaU32(slime.accelerationSkippedConfiguration, g_trackingPerfBaseline.slime.accelerationSkippedConfiguration));
    out.print("acceleration_skipped_component_missing_delta="); out.println(deltaU32(slime.accelerationSkippedComponentMissing, g_trackingPerfBaseline.slime.accelerationSkippedComponentMissing));
    out.print("acceleration_skipped_pair_degraded_delta="); out.println(deltaU32(slime.accelerationSkippedPairDegraded, g_trackingPerfBaseline.slime.accelerationSkippedPairDegraded));
    out.print("acceleration_skipped_saturated_delta="); out.println(deltaU32(slime.accelerationSkippedSaturated, g_trackingPerfBaseline.slime.accelerationSkippedSaturated));
    out.print("acceleration_skipped_nonfinite_delta="); out.println(deltaU32(slime.accelerationSkippedNonFinite, g_trackingPerfBaseline.slime.accelerationSkippedNonFinite));
    out.print("acceleration_skipped_other_delta="); out.println(deltaU32(slime.accelerationSkippedOther, g_trackingPerfBaseline.slime.accelerationSkippedOther));
    out.print("acceleration_send_failures_delta="); out.println(deltaU32(slime.accelerationSendFailures, g_trackingPerfBaseline.slime.accelerationSendFailures));
    out.print("udp_send_failures_delta="); out.println(deltaU32(slime.sendFailures, g_trackingPerfBaseline.slime.sendFailures));
}

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
    out.print("slime_rotation_missed_deadlines="); out.println(s.rotationMissedDeadlines);
    out.print("slime_rotation_late_events="); out.println(s.rotationLateEvents);
    out.print("slime_rotation_lateness_max_ms="); out.println(s.rotationLatenessMaxMs);
    out.print("slime_rotation_no_snapshot="); out.println(s.rotationNoSnapshot);
    out.print("slime_rotation_duplicate_snapshot="); out.println(s.rotationDuplicateSnapshot);
    out.print("slime_rotation_send_failures="); out.println(s.rotationSendFailures);
    out.print("slime_acceleration_sent="); out.println(s.accelerationSent);
    out.print("slime_acceleration_skipped_invalid="); out.println(s.accelerationSkippedInvalid);
    out.print("slime_acceleration_skipped_configuration="); out.println(s.accelerationSkippedConfiguration);
    out.print("slime_acceleration_skipped_component_missing="); out.println(s.accelerationSkippedComponentMissing);
    out.print("slime_acceleration_skipped_pair_degraded="); out.println(s.accelerationSkippedPairDegraded);
    out.print("slime_acceleration_skipped_saturated="); out.println(s.accelerationSkippedSaturated);
    out.print("slime_acceleration_skipped_nonfinite="); out.println(s.accelerationSkippedNonFinite);
    out.print("slime_acceleration_skipped_other="); out.println(s.accelerationSkippedOther);
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
        out.print("quality_fifo_recovery_requests="); out.println(qc.fifoRecoveryRequests);
        out.print("quality_fifo_recovery_overrun_requests="); out.println(qc.fifoRecoveryOverrunRequests);
        out.print("quality_fifo_recovery_full_requests="); out.println(qc.fifoRecoveryFullRequests);
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
    out.println("perf status | top | tracking [reset] | on | off | reset");
    out.println("  perf on: enable rolling loop-section timing from serial/telnet");
    out.println("  perf status: section avg/max/slow + temp/heap/wifi/SlimeVR counters");
    out.println("  perf tracking reset: capture a non-destructive tracking/FIFO baseline");
    out.println("  perf tracking: compact deltas/rates since that baseline");
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

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "tracking") ||
        tracker_serial_detail::eqIgnoreCase(argv[1], "brief")) {
        if (argc >= 3 && tracker_serial_detail::eqIgnoreCase(argv[2], "reset")) {
            captureTrackingBaseline(ctx, millis());
            tracker_serial_detail::printOk(out, "tracking perf baseline reset");
            return;
        }
        if (argc >= 3) {
            tracker_serial_detail::printErr(out, "usage: perf tracking [reset]");
            return;
        }
        printTrackingPerf(out, ctx);
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
        const uint32_t nowMs = millis();
        ctx.runtimeProfiler->reset(nowMs);
        captureTrackingBaseline(ctx, nowMs);
        tracker_serial_detail::printOk(out, "runtime profiler and tracking baseline reset");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "help")) {
        printUsage(out);
        return;
    }

    tracker_serial_detail::printErr(out, "usage: perf status|top|tracking [reset]|on|off|reset");
}

} // namespace tracker
