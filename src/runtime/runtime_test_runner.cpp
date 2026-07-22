#include "runtime/runtime_test_runner.hpp"

#include <cmath>

#include "defines.h"

#include "build_config/build_identity.hpp"

namespace tracker {

namespace {

constexpr uint32_t RUNTIME_TEST_SLOW_LOOP_US = 5000;
constexpr uint32_t RUNTIME_TEST_SLOW_NETWORK_US = 2000;
constexpr uint32_t RUNTIME_TEST_SLOW_FIFO_US = 20000;

const char* yn(bool v) { return v ? "yes" : "no"; }

void printAvgMax(Stream& out, const char* prefix, const RuntimeTestRunner::Stats& s) {
    out.print(prefix); out.print("_calls="); out.println(s.count);
    out.print(prefix); out.print("_avg_us="); out.println(s.mean(), 3);
    out.print(prefix); out.print("_max_us="); out.println(s.max);
}

void printWifiTxPower(Stream& out, const char* key, const TrackerWifiManagerStatus& wifi) {
    out.print(key); out.print("_quarter_dbm: ");
    if (!wifi.txPowerValid) {
        out.println("unknown");
        return;
    }
    out.println(static_cast<int>(wifi.txPowerQuarterDbm));
    out.print(key); out.print("_dbm: ");
    out.println(static_cast<float>(wifi.txPowerQuarterDbm) / 4.0f, 2);
}


void printRuntimeBuildConfig(Stream& out) {
    out.println("------------------------------------------------------------------------------");
    out.println("Build/runtime config");
    out.print("build_profile: "); out.println(trackerBuildProfileName());
    out.print("build_pio_env: "); out.println(trackerBuildPioEnvironment());
    out.print("build_git: "); out.println(trackerBuildIdentityString());
    out.print("cli_level: "); out.println(trackerCliLevelName());
    out.print("wifi_power_save_compile: "); out.println(TRACKER_WIFI_POWER_SAVE_MODE);
    out.print("wifi_tx_power_compile_quarter_dbm: ");
#ifdef TRACKER_WIFI_TX_POWER_QUARTER_DBM
    out.println(TRACKER_WIFI_TX_POWER_QUARTER_DBM);
#else
    out.println("enum");
#endif
    out.print("network_update_interval_ms: "); out.println((uint32_t)TRACKER_NETWORK_RUNTIME_UPDATE_INTERVAL_MS);
    out.print("slime_service_update_interval_ms: "); out.println((uint32_t)TRACKER_SLIMEVR_SERVICE_UPDATE_INTERVAL_MS);
    out.print("slime_live_state_refresh_ms: "); out.println((uint32_t)TRACKER_SLIMEVR_LIVE_STATE_REFRESH_MS);
    out.print("slime_runtime_config_refresh_ms: "); out.println((uint32_t)TRACKER_SLIMEVR_RUNTIME_CONFIG_REFRESH_MS);
    out.print("slime_output_rate_max_hz: "); out.println((uint32_t)TRACKER_SLIMEVR_OUTPUT_RATE_HZ_MAX);
    out.print("slime_signal_telemetry_interval_ms: "); out.println((uint32_t)TRACKER_SLIMEVR_SIGNAL_TELEMETRY_INTERVAL_MS);
    out.print("slime_temperature_telemetry_interval_ms: "); out.println((uint32_t)TRACKER_SLIMEVR_TEMPERATURE_TELEMETRY_INTERVAL_MS);
    out.print("slime_battery_telemetry_interval_ms: "); out.println((uint32_t)TRACKER_SLIMEVR_BATTERY_TELEMETRY_INTERVAL_MS);
    out.print("battery_adc_sample_interval_ms: "); out.println((uint32_t)TRACKER_BATTERY_ADC_SAMPLE_INTERVAL_MS);
    out.print("battery_adc_oversample_count: "); out.println((uint32_t)TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT);
    out.print("battery_adc_discard_count: "); out.println((uint32_t)TRACKER_BATTERY_ADC_DISCARD_COUNT);
    out.print("battery_adc_ema_alpha: "); out.println((float)TRACKER_BATTERY_ADC_EMA_ALPHA, 3);
    out.print("tap_poll_interval_ms: "); out.println((uint32_t)TRACKER_TAP_POLL_INTERVAL_MS);
    out.print("cli_bytes_per_loop: "); out.println((uint32_t)TRACKER_CLI_BYTES_PER_LOOP);
    out.print("cli_second_poll_enabled: "); out.println(TRACKER_CLI_SECOND_POLL_ENABLED ? "yes" : "no");
    out.print("idle_yield_enabled: "); out.println(TRACKER_ENABLE_IDLE_YIELD ? "yes" : "no");
    out.print("idle_yield_mode: "); out.println((uint32_t)TRACKER_IDLE_YIELD_MODE);
    out.print("idle_yield_every_n_idle_loops: "); out.println((uint32_t)TRACKER_IDLE_YIELD_EVERY_N_IDLE_LOOPS);
}
} // namespace

void RuntimeTestRunner::Stats::reset() {
    count = 0;
    sum = 0;
    min = 0;
    max = 0;
}

void RuntimeTestRunner::Stats::push(uint32_t v) {
    if (count == 0) {
        min = v;
        max = v;
    } else {
        if (v < min) min = v;
        if (v > max) max = v;
    }
    ++count;
    sum += v;
}

float RuntimeTestRunner::Stats::mean() const {
    return count == 0 ? 0.0f : static_cast<float>(sum) / static_cast<float>(count);
}

void RuntimeTestRunner::begin(const Dependencies& deps) {
    deps_ = deps;
    reset();
}

bool RuntimeTestRunner::active() const {
#if !TRACKER_ENABLE_RUNTIME_TEST
    return false;
#else
    return active_;
#endif
}

bool RuntimeTestRunner::start(uint32_t durationMs, uint32_t nowMs, Stream& out) {
#if !TRACKER_ENABLE_RUNTIME_TEST
    (void)durationMs;
    (void)nowMs;
    out.println("# ERR runtime test disabled in this build");
    return false;
#else
    if (!ready() || active_ || durationMs == 0) return false;
    reset();
    active_ = true;
    durationMs_ = durationMs;
    startMs_ = nowMs;
    lastProgressMs_ = nowMs;
    start_ = makeSnapshot();
    last_ = start_;
    if (deps_.latestTempC != nullptr) {
        tempStartC_ = *deps_.latestTempC;
        tempEndC_ = tempStartC_;
        tempValid_ = true;
        pushTempHistory(nowMs);
    }

    out.println("# RUNTIME TEST STARTED");
    out.print("# duration_s="); out.println(durationMs / 1000UL);
    out.println("# Measures full loop + FIFO + Wi-Fi + SlimeVR runtime. Stop with: test stop");
    return true;
#endif
}

bool RuntimeTestRunner::stop() {
#if !TRACKER_ENABLE_RUNTIME_TEST
    return false;
#else
    if (!active_) return false;
    stopRequested_ = true;
    return true;
#endif
}

void RuntimeTestRunner::printStatus(Stream& out, uint32_t nowMs) const {
    out.print("runtime_test_active="); out.println(active_ ? "yes" : "no");
    if (!active_) return;
    const uint32_t elapsed = nowMs - startMs_;
    out.print("runtime_elapsed_s="); out.println(elapsed / 1000UL);
    out.print("runtime_duration_s="); out.println(durationMs_ / 1000UL);
    out.print("runtime_loop_count="); out.println(loopCount_);
    out.print("runtime_loop_avg_us="); out.println(loopUs_.mean(), 3);
    out.print("runtime_loop_max_us="); out.println(loopUs_.max);
    out.print("runtime_work_loops="); out.println(workLoopCount_);
    out.print("runtime_idle_candidate_loops="); out.println(idleCandidateLoopCount_);
    out.print("runtime_idle_yield_count="); out.println(idleYieldCount_);
    out.print("runtime_network_avg_us="); out.println(networkUs_.mean(), 3);
    out.print("runtime_network_max_us="); out.println(networkUs_.max);
    out.print("runtime_fifo_avg_us="); out.println(fifoUs_.mean(), 3);
    out.print("runtime_fifo_max_us="); out.println(fifoUs_.max);
    if (tempValid_) {
        out.print("runtime_temp_start_c="); out.println(tempStartC_, 2);
        out.print("runtime_temp_now_c="); out.println(tempEndC_, 2);
    }
}

void RuntimeTestRunner::recordLoopTiming(const RuntimeLoopTimingSample& timing) {
#if TRACKER_ENABLE_RUNTIME_TEST
    if (!active_) return;
    ++loopCount_;
    loopUs_.push(timing.loopUs);
    cliUs_.push(timing.cliUs);
    fifoUs_.push(timing.fifoUs);
    networkUs_.push(timing.networkUs);
    heartbeatUs_.push(timing.heartbeatUs);
    if (timing.loopUs > RUNTIME_TEST_SLOW_LOOP_US) ++slowLoopCount_;
    if (timing.networkUs > RUNTIME_TEST_SLOW_NETWORK_US) ++slowNetworkCount_;
    if (timing.fifoUs > RUNTIME_TEST_SLOW_FIFO_US) ++slowFifoCount_;
    if (timing.anyWork) ++workLoopCount_;
    else ++idleCandidateLoopCount_;
    if (timing.fifoWorked) ++fifoWorkCount_;
    if (timing.batteryWorked) ++batteryWorkCount_;
    if (timing.networkWorked) ++networkWorkCount_;
    if (timing.tapWorked) ++tapWorkCount_;
    if (timing.ledWorked) ++ledWorkCount_;
    if (timing.heartbeatWorked) ++heartbeatWorkCount_;
    if (timing.idleYielded) ++idleYieldCount_;
    if (deps_.latestTempC != nullptr) {
        tempEndC_ = *deps_.latestTempC;
        tempValid_ = true;
    }
#else
    (void)timing;
#endif
}

void RuntimeTestRunner::update(uint32_t nowMs, Stream& out) {
#if TRACKER_ENABLE_RUNTIME_TEST
    if (!active_) return;
    if ((nowMs - lastProgressMs_) >= deps_.progressPeriodMs) {
        pushTempHistory(nowMs);
        printProgress(out, nowMs);
        lastProgressMs_ = nowMs;
        last_ = makeSnapshot();
    }
    if (stopRequested_ || (nowMs - startMs_) >= durationMs_) {
        finish(nowMs, out);
    }
#else
    (void)nowMs;
    (void)out;
#endif
}

bool RuntimeTestRunner::ready() const {
    return deps_.perf != nullptr &&
           deps_.fifo != nullptr &&
           deps_.quality != nullptr &&
           deps_.wifi != nullptr &&
           deps_.slimevr != nullptr &&
           deps_.trackingState != nullptr &&
           deps_.runtimeSamples != nullptr;
}

RuntimeTestRunner::Snapshot RuntimeTestRunner::makeSnapshot() const {
    Snapshot s;
    if (deps_.runtimeSamples) s.runtimeSamples = *deps_.runtimeSamples;
    if (deps_.trackingState) s.recoveryEnterCount = deps_.trackingState->recoveryEnterCount();
    if (deps_.perf) s.perf = *deps_.perf;
    if (deps_.quality) s.quality = deps_.quality->counters();
    if (deps_.fifo) s.fifo = deps_.fifo->stats();
    if (deps_.wifi) s.wifi = deps_.wifi->status();
    if (deps_.slimevr) s.slime = deps_.slimevr->status();
    if (deps_.battery) s.battery = deps_.battery->status();
    return s;
}

void RuntimeTestRunner::reset() {
    active_ = false;
    stopRequested_ = false;
    durationMs_ = 0;
    startMs_ = 0;
    lastProgressMs_ = 0;
    loopCount_ = 0;
    slowLoopCount_ = 0;
    slowNetworkCount_ = 0;
    slowFifoCount_ = 0;
    workLoopCount_ = 0;
    idleCandidateLoopCount_ = 0;
    fifoWorkCount_ = 0;
    batteryWorkCount_ = 0;
    networkWorkCount_ = 0;
    tapWorkCount_ = 0;
    ledWorkCount_ = 0;
    heartbeatWorkCount_ = 0;
    idleYieldCount_ = 0;
    tempStartC_ = 0.0f;
    tempEndC_ = 0.0f;
    tempValid_ = false;
    lastTempHistoryMs_ = 0;
    tempHistoryCount_ = 0;
    tempHistoryNext_ = 0;
    for (auto& sample : tempHistory_) sample = TempHistorySample{};
    start_ = Snapshot{};
    last_ = Snapshot{};
    loopUs_.reset();
    cliUs_.reset();
    fifoUs_.reset();
    networkUs_.reset();
    heartbeatUs_.reset();
}

void RuntimeTestRunner::pushTempHistory(uint32_t nowMs) {
    if (!tempValid_) return;
    if (lastTempHistoryMs_ != 0 && (nowMs - lastTempHistoryMs_) < 30000UL) return;
    TempHistorySample& slot = tempHistory_[tempHistoryNext_];
    slot.elapsedMs = nowMs - startMs_;
    slot.tempC = tempEndC_;
    slot.valid = true;
    tempHistoryNext_ = static_cast<uint8_t>((tempHistoryNext_ + 1u) % 16u);
    if (tempHistoryCount_ < 16u) ++tempHistoryCount_;
    lastTempHistoryMs_ = nowMs;
}

bool RuntimeTestRunner::computeTempSlope(uint32_t elapsedMs, float& fullSlopeCPerMin, float& recentSlopeCPerMin) const {
    fullSlopeCPerMin = 0.0f;
    recentSlopeCPerMin = 0.0f;
    if (!tempValid_ || elapsedMs == 0) return false;

    const float totalMinutes = static_cast<float>(elapsedMs) / 60000.0f;
    if (totalMinutes > 0.0f) {
        fullSlopeCPerMin = (tempEndC_ - tempStartC_) / totalMinutes;
    }

    const uint32_t windowMs = 300000UL;
    const uint32_t targetElapsed = elapsedMs > windowMs ? elapsedMs - windowMs : 0UL;
    const TempHistorySample* best = nullptr;
    for (const auto& sample : tempHistory_) {
        if (!sample.valid) continue;
        if (sample.elapsedMs > targetElapsed) continue;
        if (best == nullptr || sample.elapsedMs > best->elapsedMs) best = &sample;
    }

    if (best != nullptr && elapsedMs > best->elapsedMs) {
        const float recentMinutes = static_cast<float>(elapsedMs - best->elapsedMs) / 60000.0f;
        if (recentMinutes > 0.0f) {
            recentSlopeCPerMin = (tempEndC_ - best->tempC) / recentMinutes;
        }
    } else {
        recentSlopeCPerMin = fullSlopeCPerMin;
    }
    return true;
}

void RuntimeTestRunner::printProgress(Stream& out, uint32_t nowMs) const {
    const Snapshot now = makeSnapshot();
    const float elapsedS = static_cast<float>(nowMs - startMs_) / 1000.0f;
    out.print("# runtime progress elapsed_s="); out.print(nowMs - startMs_ >= 1000 ? (nowMs - startMs_) / 1000UL : 0UL);
    out.print(" loops="); out.print(loopCount_);
    out.print(" samples="); out.print(deltaU32(now.runtimeSamples, start_.runtimeSamples));
    out.print(" slime_rot="); out.print(deltaU32(now.slime.rotationSent, start_.slime.rotationSent));
    out.print(" send_fail="); out.print(deltaU32(now.slime.sendFailures, start_.slime.sendFailures));
    out.print(" wifi_disc="); out.print(deltaU32(now.wifi.disconnects, start_.wifi.disconnects));
    out.print(" recovery="); out.print(deltaU32(now.recoveryEnterCount, start_.recoveryEnterCount));
    out.print(" loop_max_us="); out.print(loopUs_.max);
    out.print(" net_max_us="); out.print(networkUs_.max);
    out.print(" fifo_max_us="); out.print(fifoUs_.max);
    out.print(" temp_c="); out.print(tempValid_ ? tempEndC_ : 0.0f, 2);
    out.print(" sample_rate_hz="); out.println(safeRate(deltaU32(now.runtimeSamples, start_.runtimeSamples), elapsedS), 3);
}

void RuntimeTestRunner::finish(uint32_t nowMs, Stream& out) {
    pushTempHistory(nowMs);
    const Snapshot end = makeSnapshot();
    const uint32_t elapsedMs = nowMs - startMs_;
    const float durationS = elapsedMs > 0 ? static_cast<float>(elapsedMs) / 1000.0f : 0.0f;

    out.println("==============================================================================");
    out.println("COMMAND RUNTIME TEST REPORT");
    out.println("==============================================================================");
    printRuntimeBuildConfig(out);
    out.println("------------------------------------------------------------------------------");
    out.print("stopped_by_command: "); out.println(stopRequested_ ? "yes" : "no");
    out.print("duration_s: "); out.println(durationS, 3);
    out.print("loop_count: "); out.println(loopCount_);
    out.print("runtime_samples_delta: "); out.println(deltaU32(end.runtimeSamples, start_.runtimeSamples));
    out.print("sample_rate_hz: "); out.println(safeRate(deltaU32(end.runtimeSamples, start_.runtimeSamples), durationS), 3);
    out.print("tracking_recovery_delta: "); out.println(deltaU32(end.recoveryEnterCount, start_.recoveryEnterCount));
    if (tempValid_) {
        out.print("temp_start_c: "); out.println(tempStartC_, 2);
        out.print("temp_end_c: "); out.println(tempEndC_, 2);
        out.print("temp_delta_c: "); out.println(tempEndC_ - tempStartC_, 2);
        float tempSlopeCPerMin = 0.0f;
        float tempRecentSlopeCPerMin = 0.0f;
        if (computeTempSlope(elapsedMs, tempSlopeCPerMin, tempRecentSlopeCPerMin)) {
            out.print("temp_slope_c_per_min: "); out.println(tempSlopeCPerMin, 4);
            out.print("temp_recent_slope_c_per_min: "); out.println(tempRecentSlopeCPerMin, 4);
        }
    }

    out.println("------------------------------------------------------------------------------");
    out.println("Loop timing during test");
    printAvgMax(out, "loop", loopUs_);
    printAvgMax(out, "cli", cliUs_);
    printAvgMax(out, "fifo_loop_section", fifoUs_);
    printAvgMax(out, "network_loop_section", networkUs_);
    printAvgMax(out, "heartbeat_loop_section", heartbeatUs_);
    out.print("slow_loop_count_gt_5000us: "); out.println(slowLoopCount_);
    out.print("slow_network_count_gt_2000us: "); out.println(slowNetworkCount_);
    out.print("slow_fifo_count_gt_20000us: "); out.println(slowFifoCount_);
    out.print("work_loop_count: "); out.println(workLoopCount_);
    out.print("idle_candidate_loop_count: "); out.println(idleCandidateLoopCount_);
    out.print("idle_candidate_ratio: "); out.println(loopCount_ ? static_cast<float>(idleCandidateLoopCount_) / static_cast<float>(loopCount_) : 0.0f, 6);
    out.print("fifo_work_count: "); out.println(fifoWorkCount_);
    out.print("battery_work_count: "); out.println(batteryWorkCount_);
    out.print("network_work_count: "); out.println(networkWorkCount_);
    out.print("tap_work_count: "); out.println(tapWorkCount_);
    out.print("led_work_count: "); out.println(ledWorkCount_);
    out.print("heartbeat_work_count: "); out.println(heartbeatWorkCount_);
    out.print("idle_yield_count: "); out.println(idleYieldCount_);

    out.println("------------------------------------------------------------------------------");
    out.println("Perf counter delta during test");
    const uint32_t sampleCalls = deltaU32(end.perf.sampleProcessCalls, start_.perf.sampleProcessCalls);
    const uint32_t fifoCalls = deltaU32(end.perf.fifoProcessCalls, start_.perf.fifoProcessCalls);
    out.print("perf_sample_process_calls: "); out.println(sampleCalls);
    out.print("perf_sample_process_avg_us: "); out.println(sampleCalls ? static_cast<float>(deltaU64(end.perf.sampleProcessSumUs, start_.perf.sampleProcessSumUs)) / static_cast<float>(sampleCalls) : 0.0f, 3);
    out.print("perf_sample_process_max_us_absolute: "); out.println(end.perf.sampleProcessMaxUs);
    out.print("perf_fifo_process_calls: "); out.println(fifoCalls);
    out.print("perf_fifo_process_avg_us: "); out.println(fifoCalls ? static_cast<float>(deltaU64(end.perf.fifoProcessSumUs, start_.perf.fifoProcessSumUs)) / static_cast<float>(fifoCalls) : 0.0f, 3);
    out.print("perf_fifo_process_max_us_absolute: "); out.println(end.perf.fifoProcessMaxUs);
    out.print("perf_fifo_empty_polls_delta: "); out.println(deltaU32(end.perf.fifoEmptyPolls, start_.perf.fifoEmptyPolls));
    out.print("perf_fifo_irq_events_delta: "); out.println(deltaU32(end.perf.fifoIrqEvents, start_.perf.fifoIrqEvents));
    out.print("perf_fifo_fallback_status_polls_delta: "); out.println(deltaU32(end.perf.fifoFallbackStatusPolls, start_.perf.fifoFallbackStatusPolls));
    out.print("perf_fifo_fallback_events_delta: "); out.println(deltaU32(end.perf.fifoFallbackEvents, start_.perf.fifoFallbackEvents));

    out.println("------------------------------------------------------------------------------");
    out.println("FIFO / quality delta during test");
    out.print("fifo_overrun_delta: "); out.println(deltaU32(end.fifo.overrunEvents, start_.fifo.overrunEvents));
    out.print("fifo_full_delta: "); out.println(deltaU32(end.fifo.fullEvents, start_.fifo.fullEvents));
    out.print("fifo_unknown_delta: "); out.println(deltaU32(end.fifo.unknownWords, start_.fifo.unknownWords));
    out.print("fifo_hw_ts_delta: "); out.println(deltaU32(end.fifo.hwTimestampAssigned, start_.fifo.hwTimestampAssigned));
    out.print("fifo_fb_ts_delta: "); out.println(deltaU32(end.fifo.fallbackTimestampAssigned, start_.fifo.fallbackTimestampAssigned));
    out.print("quality_samples_delta: "); out.println(deltaU32(end.quality.samples, start_.quality.samples));
    out.print("quality_estimated_dropped_delta: "); out.println(deltaU32(end.quality.estimatedDroppedSamples, start_.quality.estimatedDroppedSamples));
    out.print("quality_recovery_requests_delta: "); out.println(deltaU32(end.quality.fifoRecoveryRequests, start_.quality.fifoRecoveryRequests));
    out.print("quality_ahrs_skipped_delta: "); out.println(deltaU32(end.quality.ahrsSkippedSamples, start_.quality.ahrsSkippedSamples));
    out.print("quality_accel_disabled_delta: "); out.println(deltaU32(end.quality.accelCorrectionDisabledSamples, start_.quality.accelCorrectionDisabledSamples));

    if (deps_.battery != nullptr) {
        out.println("------------------------------------------------------------------------------");
        out.println("Battery runtime delta during test");
        out.print("battery_status_enabled_start: "); out.println(yn(start_.battery.enabled));
        out.print("battery_status_enabled_end: "); out.println(yn(end.battery.enabled));
        out.print("battery_status_configured_start: "); out.println(yn(start_.battery.configured));
        out.print("battery_status_configured_end: "); out.println(yn(end.battery.configured));
        out.print("battery_samples_delta: "); out.println(deltaU32(end.battery.samples, start_.battery.samples));
        out.print("battery_read_failures_delta: "); out.println(deltaU32(end.battery.readFailures, start_.battery.readFailures));
        out.print("battery_invalid_samples_delta: "); out.println(deltaU32(end.battery.invalidSamples, start_.battery.invalidSamples));
        out.print("battery_no_battery_samples_delta: "); out.println(deltaU32(end.battery.noBatterySamples, start_.battery.noBatterySamples));
        out.print("battery_glitch_rejected_delta: "); out.println(deltaU32(end.battery.glitchRejectedSamples, start_.battery.glitchRejectedSamples));
        out.print("battery_last_sample_age_ms_end: ");
        out.println(end.battery.lastSampleMs == 0 ? 0UL : static_cast<unsigned long>(nowMs - end.battery.lastSampleMs));
        out.print("battery_voltage_end_v: "); out.println(end.battery.voltage, 3);
        out.print("battery_percentage_end: "); out.println(end.battery.percentage, 1);
    }

    out.println("------------------------------------------------------------------------------");
    out.println("Wi-Fi / SlimeVR delta during test");
    out.print("wifi_connected_start: "); out.println(yn(start_.wifi.connected));
    out.print("wifi_connected_end: "); out.println(yn(end.wifi.connected));
    out.print("wifi_disconnects_delta: "); out.println(deltaU32(end.wifi.disconnects, start_.wifi.disconnects));
    out.print("wifi_connect_timeouts_delta: "); out.println(deltaU32(end.wifi.connectTimeouts, start_.wifi.connectTimeouts));
    out.print("wifi_rssi_end_dbm: "); out.println(end.wifi.rssiDbm);
    out.print("wifi_power_save_start: "); out.println(wifiPowerSaveModeName(start_.wifi.powerSaveMode));
    out.print("wifi_power_save_end: "); out.println(wifiPowerSaveModeName(end.wifi.powerSaveMode));
    printWifiTxPower(out, "wifi_tx_power_start", start_.wifi);
    printWifiTxPower(out, "wifi_tx_power_end", end.wifi);
    out.print("slime_server_found_start: "); out.println(yn(start_.slime.serverFound));
    out.print("slime_server_found_end: "); out.println(yn(end.slime.serverFound));
    out.print("slime_motion_packet_mode: "); out.println(slimevrMotionPacketModeName(end.slime.motionPacketMode));
    out.print("slime_rotation_sent_delta: "); out.println(deltaU32(end.slime.rotationSent, start_.slime.rotationSent));
    out.print("slime_acceleration_sent_delta: "); out.println(deltaU32(end.slime.accelerationSent, start_.slime.accelerationSent));
    out.print("slime_bundled_motion_sent_delta: "); out.println(deltaU32(end.slime.bundledMotionSent, start_.slime.bundledMotionSent));
    out.print("slime_bundled_motion_send_failures_delta: "); out.println(deltaU32(end.slime.bundledMotionSendFailures, start_.slime.bundledMotionSendFailures));
    out.print("slime_compact_motion_sent_delta: "); out.println(deltaU32(end.slime.compactMotionSent, start_.slime.compactMotionSent));
    out.print("slime_compact_motion_send_failures_delta: "); out.println(deltaU32(end.slime.compactMotionSendFailures, start_.slime.compactMotionSendFailures));
    out.print("slime_acceleration_rate_limited_delta: "); out.println(deltaU32(end.slime.accelerationRateLimited, start_.slime.accelerationRateLimited));
    out.print("slime_feature_flags_sent_delta: "); out.println(deltaU32(end.slime.featureFlagsSent, start_.slime.featureFlagsSent));
    out.print("slime_rotation_send_due_delta: "); out.println(deltaU32(end.slime.rotationSendDue, start_.slime.rotationSendDue));
    out.print("slime_rotation_rate_limited_delta: "); out.println(deltaU32(end.slime.rotationRateLimited, start_.slime.rotationRateLimited));
    out.print("slime_rotation_missed_deadlines_delta: "); out.println(deltaU32(end.slime.rotationMissedDeadlines, start_.slime.rotationMissedDeadlines));
    out.print("slime_rotation_late_events_delta: "); out.println(deltaU32(end.slime.rotationLateEvents, start_.slime.rotationLateEvents));
    out.print("slime_rotation_lateness_max_ms_end: "); out.println(end.slime.rotationLatenessMaxMs);
    out.print("slime_service_updates_delta: "); out.println(deltaU32(end.slime.serviceUpdates, start_.slime.serviceUpdates));
    out.print("slime_service_skips_delta: "); out.println(deltaU32(end.slime.serviceSkips, start_.slime.serviceSkips));
    out.print("slime_rotation_no_snapshot_delta: "); out.println(deltaU32(end.slime.rotationNoSnapshot, start_.slime.rotationNoSnapshot));
    out.print("slime_rotation_duplicate_snapshot_delta: "); out.println(deltaU32(end.slime.rotationDuplicateSnapshot, start_.slime.rotationDuplicateSnapshot));
    out.print("slime_rotation_snapshot_age_us_end: "); out.println(end.slime.lastRotationSnapshotAgeUs);
    out.print("slime_rotation_sample_lag_end: "); out.println(deltaU32(end.runtimeSamples, end.slime.lastRotationRuntimeSample));
    out.print("slime_rotation_rate_hz_observed: "); out.println(safeRate(deltaU32(end.slime.rotationSent, start_.slime.rotationSent), durationS), 3);
    out.print("slime_send_failures_delta: "); out.println(deltaU32(end.slime.sendFailures, start_.slime.sendFailures));
    out.print("slime_rotation_send_failures_delta: "); out.println(deltaU32(end.slime.rotationSendFailures, start_.slime.rotationSendFailures));
    out.print("slime_control_send_failures_delta: "); out.println(deltaU32(end.slime.controlSendFailures, start_.slime.controlSendFailures));
    out.print("slime_telemetry_send_failures_delta: "); out.println(deltaU32(end.slime.telemetrySendFailures, start_.slime.telemetrySendFailures));
    out.print("slime_discovery_send_failures_delta: "); out.println(deltaU32(end.slime.discoverySendFailures, start_.slime.discoverySendFailures));
    out.print("slime_tap_transport_send_failures_delta: "); out.println(deltaU32(end.slime.tapTransportSendFailures, start_.slime.tapTransportSendFailures));
    out.print("slime_consecutive_send_failures_end: "); out.println(end.slime.consecutiveSendFailures);
    out.print("slime_udp_begin_failures_delta: "); out.println(deltaU32(end.slime.udpBeginFailures, start_.slime.udpBeginFailures));
    out.print("slime_packets_received_delta: "); out.println(deltaU32(end.slime.packetsReceived, start_.slime.packetsReceived));
    out.print("slime_ping_received_delta: "); out.println(deltaU32(end.slime.pingReceived, start_.slime.pingReceived));
    out.print("slime_pong_sent_delta: "); out.println(deltaU32(end.slime.pongSent, start_.slime.pongSent));
    out.print("slime_unknown_packets_delta: "); out.println(deltaU32(end.slime.unknownPacketsReceived, start_.slime.unknownPacketsReceived));
    out.print("slime_server_silence_resets_delta: "); out.println(deltaU32(end.slime.serverSilenceResets, start_.slime.serverSilenceResets));
    out.print("slime_udp_reopen_requests_delta: "); out.println(deltaU32(end.slime.udpReopenRequests, start_.slime.udpReopenRequests));
    out.print("slime_udp_reopen_suppressed_recent_rx_delta: "); out.println(deltaU32(end.slime.udpReopenSuppressedRecentRx, start_.slime.udpReopenSuppressedRecentRx));
    out.println("RUNTIME TEST DONE");
    out.println("==============================================================================");

    reset();
}

uint32_t RuntimeTestRunner::deltaU32(uint32_t current, uint32_t start) {
    return current - start;
}

uint64_t RuntimeTestRunner::deltaU64(uint64_t current, uint64_t start) {
    return current - start;
}

float RuntimeTestRunner::safeRate(uint32_t delta, float durationS) {
    return durationS > 0.0f ? static_cast<float>(delta) / durationS : 0.0f;
}

} // namespace tracker
