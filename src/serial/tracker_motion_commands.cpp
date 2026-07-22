#include "serial/tracker_motion_commands.hpp"

#include "connection/lsm6dsv_fifo.hpp"
#include "runtime/runtime_motion_diagnostics.hpp"
#include "runtime/runtime_bias_types.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "sensor/imu_quality.hpp"
#include "serial/tracker_serial_parse.hpp"
#include "serial/tracker_serial_print.hpp"

namespace tracker {

namespace {

Stream& streamFor(TrackerSerialCommandContext& ctx) {
    return *ctx.io;
}

float ratePerSecond(uint32_t count, uint32_t windowMs) {
    if (windowMs == 0u) return 0.0f;
    return 1000.0f * static_cast<float>(count) / static_cast<float>(windowMs);
}

uint32_t deltaU32(uint32_t now, uint32_t then) {
    return now - then;
}

struct MotionSlimeBaseline {
    bool valid = false;
    uint32_t rotationSent = 0;
    uint32_t rotationSendDue = 0;
    uint32_t rotationRateLimited = 0;
    uint32_t rotationMissedDeadlines = 0;
    uint32_t rotationLateEvents = 0;
    uint32_t rotationNoSnapshot = 0;
    uint32_t rotationDuplicateSnapshot = 0;
    uint32_t rotationSendFailures = 0;
    uint32_t sendFailures = 0;
    uint32_t packetsReceived = 0;
    uint32_t pingReceived = 0;
    uint32_t pongSent = 0;
    uint32_t signalStrengthSent = 0;
    uint32_t temperatureSent = 0;
    uint32_t batterySent = 0;
};

MotionSlimeBaseline g_motionSlimeBaseline;

void captureMotionSlimeBaseline(const SlimeVROutputRuntime* slime) {
    if (slime == nullptr) {
        g_motionSlimeBaseline = MotionSlimeBaseline{};
        return;
    }
    const SlimeVROutputRuntimeStatus s = slime->status();
    g_motionSlimeBaseline.valid = true;
    g_motionSlimeBaseline.rotationSent = s.rotationSent;
    g_motionSlimeBaseline.rotationSendDue = s.rotationSendDue;
    g_motionSlimeBaseline.rotationRateLimited = s.rotationRateLimited;
    g_motionSlimeBaseline.rotationMissedDeadlines = s.rotationMissedDeadlines;
    g_motionSlimeBaseline.rotationLateEvents = s.rotationLateEvents;
    g_motionSlimeBaseline.rotationNoSnapshot = s.rotationNoSnapshot;
    g_motionSlimeBaseline.rotationDuplicateSnapshot = s.rotationDuplicateSnapshot;
    g_motionSlimeBaseline.rotationSendFailures = s.rotationSendFailures;
    g_motionSlimeBaseline.sendFailures = s.sendFailures;
    g_motionSlimeBaseline.packetsReceived = s.packetsReceived;
    g_motionSlimeBaseline.pingReceived = s.pingReceived;
    g_motionSlimeBaseline.pongSent = s.pongSent;
    g_motionSlimeBaseline.signalStrengthSent = s.signalStrengthSent;
    g_motionSlimeBaseline.temperatureSent = s.temperatureSent;
    g_motionSlimeBaseline.batterySent = s.batterySent;
}

void printFifoMotion(Stream& out, const Lsm6dsvFifoReader* fifo) {
    if (fifo == nullptr) return;
    const auto& s = fifo->stats();
    out.println("# FIFO MOTION CORRELATION");
    out.print("fifo_drain_calls="); out.println(s.drainCalls);
    out.print("fifo_imu_samples_produced="); out.println(s.imuSamplesProduced);
    out.print("fifo_words_read="); out.println(s.fifoWordsRead);
    out.print("fifo_max_burst_words_read="); out.println(s.maxBurstWordsRead);
    out.print("fifo_max_unread_words_seen="); out.println(s.maxUnreadWordsSeen);
    out.print("fifo_watermark_events="); out.println(s.watermarkEvents);
    out.print("fifo_overrun_events="); out.println(s.overrunEvents);
    out.print("fifo_full_events="); out.println(s.fullEvents);
    out.print("fifo_accel_saturation_count="); out.println(s.accelSaturationCount);
    out.print("fifo_gyro_saturation_count="); out.println(s.gyroSaturationCount);
    out.print("fifo_hw_timestamp_assigned="); out.println(s.hwTimestampAssigned);
    out.print("fifo_fallback_timestamp_assigned="); out.println(s.fallbackTimestampAssigned);
    out.print("fifo_timestamp_large_gap="); out.println(s.timestampLargeGap);
    out.print("fifo_timestamp_queue_overflow="); out.println(s.timestampQueueOverflow);
    out.print("fifo_waiting_sample_queue_overflow="); out.println(s.waitingSampleQueueOverflow);
    out.print("fifo_timestamp_backwards="); out.println(s.timestampBackwards);
    out.print("fifo_tag_counter_jumps="); out.println(s.tagCounterJumps);
    out.print("fifo_gyro_tag_counter_jumps="); out.println(s.gyroTagCounterJumps);
    out.print("fifo_accel_tag_counter_jumps="); out.println(s.accelTagCounterJumps);
    out.print("fifo_sample_period_us="); out.println(s.samplePeriodUs, 3);
    out.print("fifo_latest_temp_valid="); out.println(s.latestTempValid ? "yes" : "no");
    out.print("fifo_latest_temp_c="); out.println(s.latestTempC, 2);
}

void printQualityMotion(Stream& out, const ImuQualityMonitor* quality) {
    if (quality == nullptr) return;
    const auto& c = quality->counters();
    out.println("# QUALITY MOTION CORRELATION");
    out.print("quality_samples="); out.println(c.samples);
    out.print("quality_mean_dt_us="); out.println(c.meanDtUs(), 3);
    out.print("quality_min_dt_us="); out.println(c.minDtUs, 3);
    out.print("quality_max_dt_us="); out.println(c.maxDtUs, 3);
    out.print("quality_estimated_dropped_samples="); out.println(c.estimatedDroppedSamples);
    out.print("quality_large_gap_samples="); out.println(c.largeGapSamples);
    out.print("quality_fallback_timestamp_samples="); out.println(c.fallbackTimestampSamples);
    out.print("quality_fifo_overrun_events="); out.println(c.fifoOverrunEvents);
    out.print("quality_fifo_full_events="); out.println(c.fifoFullEvents);
    out.print("quality_gyro_saturated_samples="); out.println(c.gyroSaturatedSamples);
    out.print("quality_accel_saturated_samples="); out.println(c.accelSaturatedSamples);
    out.print("quality_gyro_near_saturated_samples="); out.println(c.gyroNearSaturatedSamples);
    out.print("quality_accel_near_saturated_samples="); out.println(c.accelNearSaturatedSamples);
    out.print("quality_accel_norm_outliers="); out.println(c.accelNormOutliers);
    out.print("quality_ahrs_skipped_samples="); out.println(c.ahrsSkippedSamples);
    out.print("quality_accel_correction_disabled_samples="); out.println(c.accelCorrectionDisabledSamples);
    out.print("quality_fifo_recovery_requests="); out.println(c.fifoRecoveryRequests);
}

void printBiasMotion(Stream& out, const RuntimeGyroBiasEstimator* bias) {
    if (bias == nullptr) return;
    out.println("# RUNTIME BIAS MOTION CORRELATION");
    out.print("bias_enabled="); out.println(bias->enabled ? "yes" : "no");
    out.print("bias_windows="); out.println(bias->windows);
    out.print("bias_stationary_windows="); out.println(bias->stationaryWindows);
    out.print("bias_priming_windows="); out.println(bias->primingWindows);
    out.print("bias_updates="); out.println(bias->updates);
    out.print("bias_accepted="); out.println(bias->accepted);
    out.print("bias_rejected="); out.println(bias->rejected);
    out.print("bias_motion_rejects="); out.println(bias->motionRejects);
    out.print("bias_accel_rejects="); out.println(bias->accelRejects);
    out.print("bias_saturation_rejects="); out.println(bias->saturationRejects);
    out.print("bias_temp_rejects="); out.println(bias->tempRejects);
    out.print("bias_bad_timing_rejects="); out.println(bias->badTimingRejects);
    out.print("bias_last_gyro_std_dps=");
    out.print(bias->lastGyroStdDps.x, 5); out.print(',');
    out.print(bias->lastGyroStdDps.y, 5); out.print(',');
    out.println(bias->lastGyroStdDps.z, 5);
    out.print("bias_last_accel_norm_mean_g="); out.println(bias->lastAccelNormMeanG, 5);
    out.print("bias_last_accel_norm_std_g="); out.println(bias->lastAccelNormStdG, 5);
    out.print("bias_last_temp_span_c="); out.println(bias->lastTempSpanC, 4);
    out.print("bias_runtime_trim_dps=");
    out.print(bias->runtimeTrimRadS.x * MATH_RAD_TO_DEG, 5); out.print(',');
    out.print(bias->runtimeTrimRadS.y * MATH_RAD_TO_DEG, 5); out.print(',');
    out.println(bias->runtimeTrimRadS.z * MATH_RAD_TO_DEG, 5);
}

void printSlimeMotion(Stream& out, const SlimeVROutputRuntime* slime, uint32_t windowMs) {
    if (slime == nullptr) return;
    const SlimeVROutputRuntimeStatus s = slime->status();
    out.println("# SLIMEVR MOTION CORRELATION");
    out.print("slime_state="); out.println(slimevrOutputStateName(s.state));
    out.print("slime_server_found="); out.println(s.serverFound ? "yes" : "no");
    out.print("slime_rotation_rate_hz="); out.println(s.rotationRateHz);
    const bool hasBaseline = g_motionSlimeBaseline.valid;
    const uint32_t rotationSentDelta = hasBaseline ? deltaU32(s.rotationSent, g_motionSlimeBaseline.rotationSent) : s.rotationSent;
    const uint32_t rotationDueDelta = hasBaseline ? deltaU32(s.rotationSendDue, g_motionSlimeBaseline.rotationSendDue) : s.rotationSendDue;
    const uint32_t rotationRateLimitedDelta = hasBaseline ? deltaU32(s.rotationRateLimited, g_motionSlimeBaseline.rotationRateLimited) : s.rotationRateLimited;
    const uint32_t rotationMissedDeadlineDelta = hasBaseline ? deltaU32(s.rotationMissedDeadlines, g_motionSlimeBaseline.rotationMissedDeadlines) : s.rotationMissedDeadlines;
    const uint32_t rotationLateEventDelta = hasBaseline ? deltaU32(s.rotationLateEvents, g_motionSlimeBaseline.rotationLateEvents) : s.rotationLateEvents;
    const uint32_t rotationNoSnapshotDelta = hasBaseline ? deltaU32(s.rotationNoSnapshot, g_motionSlimeBaseline.rotationNoSnapshot) : s.rotationNoSnapshot;
    const uint32_t rotationDuplicateDelta = hasBaseline ? deltaU32(s.rotationDuplicateSnapshot, g_motionSlimeBaseline.rotationDuplicateSnapshot) : s.rotationDuplicateSnapshot;
    const uint32_t rotationSendFailuresDelta = hasBaseline ? deltaU32(s.rotationSendFailures, g_motionSlimeBaseline.rotationSendFailures) : s.rotationSendFailures;
    const uint32_t sendFailuresDelta = hasBaseline ? deltaU32(s.sendFailures, g_motionSlimeBaseline.sendFailures) : s.sendFailures;

    out.print("slime_motion_baseline_valid="); out.println(hasBaseline ? "yes" : "no");
    out.print("slime_rotation_sent="); out.println(s.rotationSent);
    out.print("slime_rotation_sent_delta="); out.println(rotationSentDelta);
    out.print("slime_rotation_sent_rate_hz="); out.println(ratePerSecond(rotationSentDelta, windowMs), 3);
    out.print("slime_rotation_send_due="); out.println(s.rotationSendDue);
    out.print("slime_rotation_send_due_delta="); out.println(rotationDueDelta);
    out.print("slime_rotation_send_due_rate_hz="); out.println(ratePerSecond(rotationDueDelta, windowMs), 3);
    out.print("slime_rotation_rate_limited="); out.println(s.rotationRateLimited);
    out.print("slime_rotation_rate_limited_delta="); out.println(rotationRateLimitedDelta);
    out.print("slime_rotation_missed_deadlines="); out.println(s.rotationMissedDeadlines);
    out.print("slime_rotation_missed_deadlines_delta="); out.println(rotationMissedDeadlineDelta);
    out.print("slime_rotation_late_events_delta="); out.println(rotationLateEventDelta);
    out.print("slime_rotation_lateness_max_ms="); out.println(s.rotationLatenessMaxMs);
    out.print("slime_rotation_no_snapshot="); out.println(s.rotationNoSnapshot);
    out.print("slime_rotation_no_snapshot_delta="); out.println(rotationNoSnapshotDelta);
    out.print("slime_rotation_duplicate_snapshot="); out.println(s.rotationDuplicateSnapshot);
    out.print("slime_rotation_duplicate_snapshot_delta="); out.println(rotationDuplicateDelta);
    out.print("slime_rotation_send_failures="); out.println(s.rotationSendFailures);
    out.print("slime_rotation_send_failures_delta="); out.println(rotationSendFailuresDelta);
    out.print("slime_send_failures="); out.println(s.sendFailures);
    out.print("slime_send_failures_delta="); out.println(sendFailuresDelta);
    out.print("slime_packets_received_delta="); out.println(hasBaseline ? deltaU32(s.packetsReceived, g_motionSlimeBaseline.packetsReceived) : s.packetsReceived);
    out.print("slime_ping_received_delta="); out.println(hasBaseline ? deltaU32(s.pingReceived, g_motionSlimeBaseline.pingReceived) : s.pingReceived);
    out.print("slime_pong_sent_delta="); out.println(hasBaseline ? deltaU32(s.pongSent, g_motionSlimeBaseline.pongSent) : s.pongSent);
    out.print("slime_signal_strength_sent_delta="); out.println(hasBaseline ? deltaU32(s.signalStrengthSent, g_motionSlimeBaseline.signalStrengthSent) : s.signalStrengthSent);
    out.print("slime_temperature_sent_delta="); out.println(hasBaseline ? deltaU32(s.temperatureSent, g_motionSlimeBaseline.temperatureSent) : s.temperatureSent);
    out.print("slime_battery_sent_delta="); out.println(hasBaseline ? deltaU32(s.batterySent, g_motionSlimeBaseline.batterySent) : s.batterySent);
    out.print("slime_prepared_output_available="); out.println(s.preparedOutputAvailable ? "yes" : "no");
    out.print("slime_last_rotation_snapshot_age_us="); out.println(s.lastRotationSnapshotAgeUs);
    out.print("slime_last_rotation_quality_flags=0x"); out.println(s.lastRotationQualityFlags, HEX);
    out.print("slime_last_rotation_confidence="); out.println(s.lastRotationConfidence, 4);
    out.print("slime_last_rssi_dbm="); out.println(s.lastRssiDbm);
}

void printUsage(Stream& out) {
    out.println("motion status | on | off | reset");
    out.println("  motion on: enable and reset per-sample movement diagnostics");
    out.println("  motion status: motion window + FIFO/quality/bias/SlimeVR correlation");
}

} // namespace

void trackerSerialDispatchMotionCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = streamFor(ctx);
    RuntimeMotionDiagnostics* motion = ctx.motionDiagnostics;
    if (motion == nullptr) {
        tracker_serial_detail::printErr(out, "motion diagnostics not wired/compiled");
        return;
    }

    const uint32_t nowMs = millis();
    if (argc < 2 || tracker_serial_detail::eqIgnoreCase(argv[1], "status")) {
        motion->printStatus(out, nowMs);
        const uint32_t windowMs = motion->windowMs(nowMs);
        printFifoMotion(out, ctx.fifo);
        printQualityMotion(out, ctx.quality);
        printBiasMotion(out, ctx.runtimeBias);
        printSlimeMotion(out, ctx.slimevrRuntime, windowMs);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "on") ||
        tracker_serial_detail::eqIgnoreCase(argv[1], "enable")) {
        motion->setEnabled(true, nowMs);
        captureMotionSlimeBaseline(ctx.slimevrRuntime);
        tracker_serial_detail::printOk(out, "motion diagnostics enabled and reset");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "off") ||
        tracker_serial_detail::eqIgnoreCase(argv[1], "disable")) {
        motion->setEnabled(false, nowMs);
        g_motionSlimeBaseline = MotionSlimeBaseline{};
        tracker_serial_detail::printOk(out, "motion diagnostics disabled");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "reset")) {
        motion->reset(nowMs);
        captureMotionSlimeBaseline(ctx.slimevrRuntime);
        tracker_serial_detail::printOk(out, "motion diagnostics reset");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "help")) {
        printUsage(out);
        return;
    }

    tracker_serial_detail::printErr(out, "usage: motion status|on|off|reset");
}

} // namespace tracker
