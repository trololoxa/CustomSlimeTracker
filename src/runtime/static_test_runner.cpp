#include "runtime/static_test_runner.hpp"

#include "build_config/build_identity.hpp"
#include "runtime/diagnostic_test_summary.hpp"

namespace tracker {

float staticTestAvgUs(uint64_t sumUs, uint32_t calls) {
    return calls == 0 ? 0.0f : static_cast<float>(sumUs) / static_cast<float>(calls);
}

float staticTestAngleDiffDeg(float a, float b) {
    return wrapPi((b - a) * MATH_DEG_TO_RAD) * MATH_RAD_TO_DEG;
}

void StaticTestRunner::begin(const Dependencies& deps) {
    deps_ = deps;
    output_ = nullptr;
    statsBlock_.reset();
}

bool StaticTestRunner::active() const {
    return deps_.activeTest != nullptr && deps_.activeTest->active;
}

const StaticRuntimeTest* StaticTestRunner::lastCompleted() const {
    return (deps_.lastCompletedValid != nullptr && *deps_.lastCompletedValid && deps_.lastCompletedTest != nullptr)
        ? deps_.lastCompletedTest
        : nullptr;
}

bool StaticTestRunner::start(uint32_t durationMs, Stream& out, float magErrorStartDeg) {
#if !TRACKER_ENABLE_STATIC_TEST
    (void)durationMs;
    (void)magErrorStartDeg;
    out.println("# ERR static test disabled in this build");
    return false;
#else
    if (!ready()) return false;

    StaticRuntimeTest& t = *deps_.activeTest;
    if (t.active) return false;

    t.reset();
    statsBlock_.reset();
    output_ = &out;
    t.active = true;
    t.durationMs = durationMs;
    t.startMs = millis();
    t.lastProgressMs = t.startMs;

    // Start with a clean quality/FIFO baseline. The user may have issued
    // `fifo reset` just before the test, but the counters can still contain
    // stale deltas from the reset/recovery boundary. Syncing here makes the
    // first test window measure only events that happen during the test.
    deps_.quality->clearRecoveryRequest();
    deps_.quality->syncFifoStats(deps_.fifo->stats());

    const auto& fs = deps_.fifo->stats();
    t.fifoOverrunAtStart = fs.overrunEvents;
    t.fifoFullAtStart = fs.fullEvents;
    t.fifoUnknownAtStart = fs.unknownWords;
    t.fifoHwTsAtStart = fs.hwTimestampAssigned;
    t.fifoFbTsAtStart = fs.fallbackTimestampAssigned;

    if (deps_.wifi != nullptr && deps_.slimevr != nullptr) {
        const TrackerWifiManagerStatus wifi = deps_.wifi->status();
        const SlimeVROutputRuntimeStatus slime = deps_.slimevr->status();
        t.networkMetricsValid = true;
        t.wifiDisconnectsAtStart = wifi.disconnects;
        t.wifiConnectTimeoutsAtStart = wifi.connectTimeouts;
        t.udpSendFailuresAtStart = slime.sendFailures;
        t.rotationSendFailuresAtStart = slime.rotationSendFailures;
        t.rotationMissedDeadlinesAtStart = slime.rotationMissedDeadlines;
        t.rotationLateEventsAtStart = slime.rotationLateEvents;
        t.txPressureFailuresAtStart = slime.txPressureFailures;
        t.txOtherFailuresAtStart = slime.txOtherFailures;
        t.udpRebindSuccessesAtStart = slime.udpTransportRebindSuccesses;
        t.udpRebindFailuresAtStart = slime.udpTransportRebindFailures;
        t.udpFullReopensAtStart = slime.udpFullReopenEscalations;
    }

    snapshotPerfCounters(t);

    const auto& ms = deps_.magProcessor->stats();
    const auto& hs = deps_.magHeading->stats();
    const auto& ys = deps_.magYawCorrection->stats();

    t.magEnabledAtStart = deps_.config->data.magCal.driverEnabled;
    t.magYawApplyEnabledAtStart = deps_.config->data.magYaw.applyEnabled;
    t.magRefValidAtStart = deps_.magHeadingRef->valid;
    t.magErrorStartDeg = t.magRefValidAtStart && std::isfinite(magErrorStartDeg) ? magErrorStartDeg : 0.0f;
    t.magErrorEndDeg = t.magErrorStartDeg;

    t.magTrustedAtStart = ms.trustedSamples;
    t.magRejectedAtStart = ms.rejectedSamples;

    t.magHeadingValidAtStart = hs.valid;
    t.magHeadingRejectedAtStart = hs.rejected;

    t.magYawUpdatesAtStart = ys.updates;
    t.magYawGateOpenAtStart = ys.gateOpenCount;
    t.magYawGateClosedAtStart = ys.gateClosedCount;
    t.magYawApplyAllowedAtStart = ys.applyAllowedCount;
    t.magYawAppliedAtStart = ys.appliedCount;

    t.magYawRejectNoReferenceAtStart = ys.rejectNoReference;
    t.magYawRejectHeadingInvalidAtStart = ys.rejectHeadingInvalid;
    t.magYawRejectMagNotTrustedAtStart = ys.rejectMagNotTrusted;
    t.magYawRejectMagStaleAtStart = ys.rejectMagStale;
    t.magYawRejectHorizontalBadAtStart = ys.rejectHorizontalBad;
    t.magYawRejectInnovationTooLargeAtStart = ys.rejectInnovationTooLarge;
    t.magYawRejectGyroMovingAtStart = ys.rejectGyroMoving;
    t.magYawRejectAccelNotTrustedAtStart = ys.rejectAccelNotTrusted;

    out.println("# STATIC TEST STARTED");
    out.print("# duration_s="); out.println(durationMs / 1000UL);
    out.println("# stop with: test stop");
    return true;
#endif
}

bool StaticTestRunner::stop(Stream& out, bool force) {
#if !TRACKER_ENABLE_STATIC_TEST
    return false;
#else
    if (!ready()) return false;
    StaticRuntimeTest& t = *deps_.activeTest;
    if (!t.active) return false;
    if (!force && output_ != &out) return false;
    t.stopRequested = true;
    return true;
#endif
}

bool StaticTestRunner::abortOutput(Stream& out) {
#if !TRACKER_ENABLE_STATIC_TEST
    (void)out;
    return false;
#else
    if (!active() || output_ != &out) return false;
    deps_.activeTest->reset();
    statsBlock_.reset();
    output_ = nullptr;
    return true;
#endif
}

void StaticTestRunner::printStatus(Stream& out) const {
    const bool isActive = active();
    out.print("test_active="); out.println(isActive ? "yes" : "no");
    const bool completedValid = deps_.lastCompletedValid != nullptr && *deps_.lastCompletedValid;
    out.print("last_completed_valid="); out.println(completedValid ? "yes" : "no");
    if (!isActive) {
        if (completedValid && deps_.lastCompletedTest != nullptr && deps_.lastCompletedFinishedMs != nullptr) {
            const StaticRuntimeTest& completed = *deps_.lastCompletedTest;
            out.print("last_completed_age_s="); out.println((millis() - *deps_.lastCompletedFinishedMs) / 1000UL);
            out.print("last_completed_samples="); out.println(completed.samples);
            out.print("last_completed_temp_mean_c="); out.println(completed.tempC.mean(), 3);
            out.print("last_completed_gyro_mean_dps_norm=");
            out.println((completed.gyroAfterRadS.mean() * MATH_RAD_TO_DEG).norm(), 6);
        }
        return;
    }

    const StaticRuntimeTest& t = *deps_.activeTest;
    const uint32_t elapsed = millis() - t.startMs;
    out.print("elapsed_s="); out.println(elapsed / 1000UL);
    out.print("duration_s="); out.println(t.durationMs / 1000UL);
    out.print("samples="); out.println(t.samples);
    out.print("stats_pending_samples="); out.println(statsBlock_.bufferedSamples);
    out.print("hw_ts="); out.println(t.hwTs);
    out.print("fallback_ts="); out.println(t.fallbackTs);
    out.print("bad_ts="); out.println(t.badTs);
    out.print("estimated_dropped="); out.println(t.droppedEstimate);
    out.print("accel_norm_mean="); out.println(t.accelNormG.mean(), 6);
    out.print("gyro_after_mean_dps_norm="); out.println((t.gyroAfterRadS.mean() * MATH_RAD_TO_DEG).norm(), 6);

    const PerfDelta perf = perfDelta(t);
    out.print("sample_process_avg_us="); out.println(staticTestAvgUs(perf.sampleSumUs, perf.sampleCalls), 3);
    out.print("sample_process_max_us="); out.println(t.maxSampleProcessUs);
    out.print("fifo_process_avg_us="); out.println(staticTestAvgUs(perf.fifoSumUs, perf.fifoCalls), 3);
    out.print("fifo_process_max_us="); out.println(t.maxFifoProcessUs);
    out.print("fifo_empty_polls_delta="); out.println(perf.emptyPolls);
    out.print("fifo_fallback_status_polls_delta="); out.println(perf.fallbackPolls);
}

void StaticTestRunner::recordMagYawSample(float headingErrorDeg,
                        const MagHeadingSample& heading,
                        const MagYawCorrectionOutput& yaw) {
    if (!active()) return;

    StaticRuntimeTest& t = *deps_.activeTest;
    if (deps_.magHeadingRef != nullptr && deps_.magHeadingRef->valid && heading.valid) {
        if (std::isfinite(headingErrorDeg)) {
            t.magErrorEndDeg = headingErrorDeg;

            const float absError = std::fabs(headingErrorDeg);
            t.magErrorAbsSumDeg += static_cast<double>(absError);
            t.magErrorSamples++;

            if (t.magErrorSamples == 1 || absError > t.magErrorAbsMaxDeg) {
                t.magErrorAbsMaxDeg = absError;
            }
        }
    }

    if (heading.valid && std::isfinite(heading.horizontalNorm)) {
        t.magHeadingHorizontalNorm.push(heading.horizontalNorm);
    }

    if (yaw.valid) {
        if (std::isfinite(yaw.combinedTrust)) {
            t.magYawCombinedTrust.push(yaw.combinedTrust);
        }

        if (std::isfinite(yaw.correctionRateDegS)) {
            t.magYawCorrectionRateDegS.push(yaw.correctionRateDegS);
        }

        if (std::isfinite(yaw.correctionStepDeg)) {
            t.magYawCorrectionStepDeg.push(yaw.correctionStepDeg);
        }
    }
}

void StaticTestRunner::updateSample(const Lsm6dsv::Sample& calibrated,
                  const ImuQualityResult& quality) {
#if !TRACKER_ENABLE_STATIC_TEST
    (void)calibrated;
    (void)quality;
    return;
#else
    if (!active()) return;
    if (output_ == nullptr) {
        deps_.activeTest->reset();
        return;
    }
    Stream& out = *output_;

    StaticRuntimeTest& t = *deps_.activeTest;
    const bool sampleClock = t.stopRequested ||
        (t.updateDecimator++ % TRACKER_DIAGNOSTIC_TIMING_SAMPLE_DIVISOR) == 0u;
    const uint32_t updateStartUs = sampleClock ? micros() : 0u;
    const uint32_t nowMs = sampleClock ? millis() : t.lastProgressMs;
    const uint32_t elapsedMs = nowMs - t.startMs;

    if (quality.has(imu_quality_flags::TIMESTAMP_HARDWARE)) t.hwTs++;
    if (quality.has(imu_quality_flags::TIMESTAMP_FALLBACK)) t.fallbackTs++;
    if (quality.has(imu_quality_flags::TIMESTAMP_ZERO) || quality.has(imu_quality_flags::TIMESTAMP_NON_MONOTONIC)) t.badTs++;
    if (quality.has(imu_quality_flags::SAMPLE_DROPPED_BEFORE)) t.droppedEstimate += quality.estimatedDroppedBefore;
    if (quality.shouldRequestFifoRecovery) t.recoveryRequests++;
    if (!quality.shouldUpdateAhrs) t.ahrsSkipped++;
    if (!quality.shouldUseAccelCorrection) t.accelDisabled++;

    const float accelNormG = quality.accelNormValid ? quality.accelNormG : calibrated.accel_g.norm();
    if (!t.tempCaptured) {
        t.tempCaptured = true;
        t.tempStartC = calibrated.temp_c;
    }

    t.tempEndC = calibrated.temp_c;

    const int tempBinIdx = staticTempBinIndex(calibrated.temp_c);
    const bool staticSampleGoodForTempFit = quality.shouldUpdateAhrs &&
        !quality.shouldRequestFifoRecovery &&
        !quality.has(imu_quality_flags::TIMESTAMP_ZERO) &&
        !quality.has(imu_quality_flags::TIMESTAMP_NON_MONOTONIC) &&
        !quality.has(imu_quality_flags::TIMESTAMP_BACKWARDS) &&
        !quality.has(imu_quality_flags::TIMESTAMP_LARGE_GAP) &&
        !quality.has(imu_quality_flags::FIFO_OVERRUN) &&
        !quality.has(imu_quality_flags::FIFO_FULL) &&
        !quality.has(imu_quality_flags::GYRO_SATURATED) &&
        !quality.has(imu_quality_flags::ACCEL_SATURATED);
    if (!statsBlock_.push(quality.dtUs,
                          quality.accelConfidence,
                          calibrated.temp_c,
                          calibrated.gyro_rad_s,
                          accelNormG,
                          tempBinIdx,
                          staticSampleGoodForTempFit)) {
        // A physically implausible >4 C span inside one 64-sample block must
        // not lose or double-count a sample. Flush the bounded sparse bins and
        // retry into the now-empty block.
        flushStats(t);
        (void)statsBlock_.push(quality.dtUs,
                               quality.accelConfidence,
                               calibrated.temp_c,
                               calibrated.gyro_rad_s,
                               accelNormG,
                               tempBinIdx,
                               staticSampleGoodForTempFit);
    }
    if (tempBinIdx < 0) {
        t.tempBinOutOfRangeSamples++;
    }
    t.samples++;
    if (statsBlock_.bufferedSamples >= TRACKER_STATIC_TEST_STATS_BLOCK_SAMPLES) {
        flushStats(t);
    }

    if (deps_.ahrs != nullptr && deps_.ahrs->initialized()) {
        const Quat q = deps_.ahrs->quaternionPositiveW();
        if (!t.poseCaptured) {
            t.poseCaptured = true;
            t.qStart = q;
            // Euler conversion is expensive on ESP32-C3; do it only once at
            // start and once in finish(), not for every FIFO sample.
            t.eulerStartDeg = q.toEulerXYZ() * MATH_RAD_TO_DEG;
        }
        t.qEnd = q;
        t.poseSamples++;
    }

    if (sampleClock && nowMs - t.lastProgressMs >= deps_.progressPeriodMs) {
        flushStats(t);
        t.lastProgressMs = nowMs;
        printProgress(out, elapsedMs);
    }

    if (sampleClock) {
        ++t.updateTimingSamples;
        const uint32_t updateUs = micros() - updateStartUs;
        if (updateUs > t.maxUpdateUs) t.maxUpdateUs = updateUs;
        if (updateUs > 1000UL) t.slowUpdateCount++;
    }

    if (t.stopRequested || (sampleClock && elapsedMs >= t.durationMs)) {
        finish();
    }
#endif
}

void StaticTestRunner::recordSampleProcessTime(uint32_t processUs) {
    if (!active()) return;
    StaticRuntimeTest& t = *deps_.activeTest;
    if (processUs > t.maxSampleProcessUs) {
        t.maxSampleProcessUs = processUs;
    }
}

void StaticTestRunner::recordFifoProcessTime(uint32_t processUs) {
    if (!active()) return;
    StaticRuntimeTest& t = *deps_.activeTest;
    if (processUs > t.maxFifoProcessUs) {
        t.maxFifoProcessUs = processUs;
    }
}

bool StaticTestRunner::printLastReport(Stream& out) const {
    const StaticRuntimeTest* completed = lastCompleted();
    if (completed == nullptr) {
        out.println("# ERR no completed static test report");
        return false;
    }
    const StaticRuntimeTest& t = *completed;
    const uint32_t elapsedMs = t.finishedElapsedMs;
    const float durationS = static_cast<float>(elapsedMs) / 1000.0f;
    const float sampleRate = durationS > 0.0f ? static_cast<float>(t.samples) / durationS : 0.0f;

    const Vec3 gyroAfterMeanDps = t.gyroAfterRadS.mean() * MATH_RAD_TO_DEG;
    const Vec3 gyroAfterStdDps = t.gyroAfterRadS.stddev() * MATH_RAD_TO_DEG;
    const Vec3 gyroAfterMinDps = t.gyroAfterRadS.minValue * MATH_RAD_TO_DEG;
    const Vec3 gyroAfterMaxDps = t.gyroAfterRadS.maxValue * MATH_RAD_TO_DEG;

    const float dRoll = staticTestAngleDiffDeg(t.eulerStartDeg.x, t.eulerEndDeg.x);
    const float dPitch = staticTestAngleDiffDeg(t.eulerStartDeg.y, t.eulerEndDeg.y);
    const float dYaw = staticTestAngleDiffDeg(t.eulerStartDeg.z, t.eulerEndDeg.z);

    const float durationMin = durationS > 0.0f ? durationS / 60.0f : 0.0f;
    const float yawDriftDegPerMin = durationMin > 0.0f ? dYaw / durationMin : 0.0f;

    out.println();
    out.println("==============================================================================");
    out.println("COMMAND STATIC TEST REPORT");
    out.println("==============================================================================");
    out.print("build_profile: "); out.println(trackerBuildProfileName());
    out.print("build_pio_env: "); out.println(trackerBuildPioEnvironment());
    out.print("build_git: "); out.println(trackerBuildIdentityString());
    out.print("stopped_by_command: "); out.println(t.stoppedByCommand ? "yes" : "no");
    out.print("duration_s: "); out.println(durationS, 3);
    out.print("samples: "); out.println(t.samples);
    out.print("sample_rate_hz: "); out.println(sampleRate, 3);
    out.print("hw_timestamp_samples: "); out.println(t.hwTs);
    out.print("fallback_timestamp_samples: "); out.println(t.fallbackTs);
    out.print("bad_timestamp_samples: "); out.println(t.badTs);
    out.print("estimated_dropped_samples: "); out.println(t.droppedEstimate);
    out.print("recovery_requests: "); out.println(t.recoveryRequests);
    out.print("ahrs_skipped_samples: "); out.println(t.ahrsSkipped);
    out.print("accel_correction_disabled_samples: "); out.println(t.accelDisabled);
    out.print("static_update_max_us: "); out.println(t.maxUpdateUs);
    out.print("static_update_timing_samples: "); out.println(t.updateTimingSamples);
    out.print("diagnostic_timing_sample_divisor: ");
    out.println((uint32_t)TRACKER_DIAGNOSTIC_TIMING_SAMPLE_DIVISOR);
    out.print("static_stats_block_samples: ");
    out.println((uint32_t)TRACKER_STATIC_TEST_STATS_BLOCK_SAMPLES);
    out.print("sample_process_max_us: "); out.println(t.maxSampleProcessUs);
    out.print("fifo_process_max_us: "); out.println(t.maxFifoProcessUs);
    out.print("static_update_slow_count: "); out.println(t.slowUpdateCount);

    PerfDelta perf;
    perf.sampleCalls = t.perfSampleCallsDelta;
    perf.sampleSumUs = t.perfSampleSumUsDelta;
    perf.fifoCalls = t.perfFifoCallsDelta;
    perf.fifoSumUs = t.perfFifoSumUsDelta;
    perf.emptyPolls = t.perfEmptyPollsDelta;
    perf.irqEvents = t.perfIrqEventsDelta;
    perf.fallbackPolls = t.perfFallbackPollsDelta;
    perf.fallbackEvents = t.perfFallbackEventsDelta;
    out.println("------------------------------------------------------------------------------");
    out.println("PERF delta during test");
    out.print("perf_sample_process_calls: "); out.println(perf.sampleCalls);
    out.print("perf_sample_process_avg_us: "); out.println(staticTestAvgUs(perf.sampleSumUs, perf.sampleCalls), 3);
    out.print("perf_sample_process_max_us: "); out.println(t.maxSampleProcessUs);
    out.print("perf_fifo_process_calls: "); out.println(perf.fifoCalls);
    out.print("perf_fifo_process_avg_us: "); out.println(staticTestAvgUs(perf.fifoSumUs, perf.fifoCalls), 3);
    out.print("perf_fifo_process_max_us: "); out.println(t.maxFifoProcessUs);
    out.print("perf_fifo_empty_polls: "); out.println(perf.emptyPolls);
    out.print("perf_fifo_empty_polls_per_s: "); out.println(durationS > 0.0f ? static_cast<float>(perf.emptyPolls) / durationS : 0.0f, 3);
    out.print("perf_fifo_irq_events: "); out.println(perf.irqEvents);
    out.print("perf_fifo_fallback_status_polls: "); out.println(perf.fallbackPolls);
    out.print("perf_fifo_fallback_status_polls_per_s: "); out.println(durationS > 0.0f ? static_cast<float>(perf.fallbackPolls) / durationS : 0.0f, 3);
    out.print("perf_fifo_fallback_events: "); out.println(perf.fallbackEvents);

    out.println("------------------------------------------------------------------------------");
    out.println("FIFO delta during test");
    out.print("fifo_overrun_delta: "); out.println(t.fifoOverrunDelta);
    out.print("fifo_full_delta: "); out.println(t.fifoFullDelta);
    out.print("fifo_unknown_delta: "); out.println(t.fifoUnknownDelta);
    out.print("fifo_hw_ts_delta: "); out.println(t.fifoHwTsDelta);
    out.print("fifo_fb_ts_delta: "); out.println(t.fifoFbTsDelta);

    out.println("------------------------------------------------------------------------------");
    out.println("Timing / motion stats");
    out.print("dt_mean_us: "); out.println(t.dtUs.mean(), 6);
    out.print("dt_min_us: "); out.println(t.dtUs.minValue, 6);
    out.print("dt_max_us: "); out.println(t.dtUs.maxValue, 6);
    out.print("dt_std_us: "); out.println(t.dtUs.stddev(), 6);
    out.print("accel_norm_mean_g: "); out.println(t.accelNormG.mean(), 6);
    out.print("accel_norm_min_g: "); out.println(t.accelNormG.minValue, 6);
    out.print("accel_norm_max_g: "); out.println(t.accelNormG.maxValue, 6);
    out.print("accel_norm_std_g: "); out.println(t.accelNormG.stddev(), 8);
    out.print("gyro_after_mean_dps_norm: ");
    out.println(gyroAfterMeanDps.norm(), 6);

    out.print("gyro_after_mean_dps_xyz: ");
    out.print(gyroAfterMeanDps.x, 8); out.print(',');
    out.print(gyroAfterMeanDps.y, 8); out.print(',');
    out.println(gyroAfterMeanDps.z, 8);

    out.print("gyro_after_std_dps_xyz: ");
    out.print(gyroAfterStdDps.x, 8); out.print(',');
    out.print(gyroAfterStdDps.y, 8); out.print(',');
    out.println(gyroAfterStdDps.z, 8);

    out.print("gyro_after_min_dps_xyz: ");
    out.print(gyroAfterMinDps.x, 8); out.print(',');
    out.print(gyroAfterMinDps.y, 8); out.print(',');
    out.println(gyroAfterMinDps.z, 8);

    out.print("gyro_after_max_dps_xyz: ");
    out.print(gyroAfterMaxDps.x, 8); out.print(',');
    out.print(gyroAfterMaxDps.y, 8); out.print(',');
    out.println(gyroAfterMaxDps.z, 8);

    out.print("temp_start_c: ");
    out.println(t.tempStartC, 3);

    out.print("temp_end_c: ");
    out.println(t.tempEndC, 3);

    out.print("temp_delta_c: ");
    out.println(t.tempEndC - t.tempStartC, 3);

    out.print("temp_mean_c: ");
    out.println(t.tempC.mean(), 3);

    out.print("temp_min_c: ");
    out.println(t.tempC.minValue, 3);

    out.print("temp_max_c: ");
    out.println(t.tempC.maxValue, 3);

    uint32_t tempBinsUsed = 0;
    for (uint8_t i = 0; i < STATIC_TEMP_BIN_COUNT; ++i) {
        if (t.tempBins[i].gyroAfterRadS.count >= 512) tempBinsUsed++;
    }
    out.print("temp_bins_used: ");
    out.println(tempBinsUsed);
    out.print("temp_bin_out_of_range_samples: ");
    out.println(t.tempBinOutOfRangeSamples);
    for (uint8_t i = 0; i < STATIC_TEMP_BIN_COUNT; ++i) {
        const StaticTempBinStats& b = t.tempBins[i];
        if (b.gyroAfterRadS.count < 512) continue;
        const Vec3 gm = b.gyroAfterRadS.mean() * MATH_RAD_TO_DEG;
        const Vec3 gs = b.gyroAfterRadS.stddev() * MATH_RAD_TO_DEG;
        out.print("TEMPBIN,");
        out.print(i);
        out.print(','); out.print(b.tempC.minValue, 3);
        out.print(','); out.print(b.tempC.maxValue, 3);
        out.print(','); out.print(b.tempC.mean(), 3);
        out.print(','); out.print(b.gyroAfterRadS.count);
        out.print(','); out.print(gm.x, 8);
        out.print(','); out.print(gm.y, 8);
        out.print(','); out.print(gm.z, 8);
        out.print(','); out.print(gs.x, 8);
        out.print(','); out.print(gs.y, 8);
        out.print(','); out.print(gs.z, 8);
        out.print(','); out.print(b.accelNormG.mean(), 6);
        out.print(','); out.print(b.accelNormG.stddev(), 6);
        out.print(','); out.println(b.badQualitySamples);
    }

    out.println("------------------------------------------------------------------------------");
    out.println("Orientation");
    out.print("euler_start_deg: ");
    out.print(t.eulerStartDeg.x, 3); out.print(',');
    out.print(t.eulerStartDeg.y, 3); out.print(',');
    out.println(t.eulerStartDeg.z, 3);
    out.print("euler_end_deg: ");
    out.print(t.eulerEndDeg.x, 3); out.print(',');
    out.print(t.eulerEndDeg.y, 3); out.print(',');
    out.println(t.eulerEndDeg.z, 3);
    out.print("euler_delta_deg: roll="); out.print(dRoll, 3);
    out.print(" pitch="); out.print(dPitch, 3);
    out.print(" yaw="); out.println(dYaw, 3);
    out.print("yaw_drift_rate_deg_min: ");
    out.println(yawDriftDegPerMin, 6);

    out.println("------------------------------------------------------------------------------");
    out.println("MAG / YAW CORRECTION");

    const float magErrorAbsMeanDeg =
        t.magErrorSamples > 0
            ? static_cast<float>(t.magErrorAbsSumDeg / static_cast<double>(t.magErrorSamples))
            : 0.0f;

    out.print("mag_enabled_start: ");
    out.println(t.magEnabledAtStart ? "yes" : "no");

    out.print("mag_yaw_apply_enabled_start: ");
    out.println(t.magYawApplyEnabledAtStart ? "yes" : "no");

    out.print("mag_ref_valid_start: ");
    out.println(t.magRefValidAtStart ? "yes" : "no");

    out.print("mag_ref_valid_end: ");
    out.println(t.magRefValidEnd ? "yes" : "no");

    out.print("mag_error_start_deg: ");
    out.println(t.magErrorStartDeg, 6);

    out.print("mag_error_end_deg: ");
    out.println(t.magErrorEndDeg, 6);

    out.print("mag_error_delta_deg: ");
    out.println(t.magErrorEndDeg - t.magErrorStartDeg, 6);

    out.print("mag_error_abs_mean_deg: ");
    out.println(magErrorAbsMeanDeg, 6);

    out.print("mag_error_abs_max_deg: ");
    out.println(t.magErrorAbsMaxDeg, 6);

    out.print("mag_error_samples: ");
    out.println(t.magErrorSamples);

    out.print("mag_heading_horizontal_norm_mean: ");
    out.println(t.magHeadingHorizontalNorm.mean(), 6);

    out.print("mag_heading_horizontal_norm_min: ");
    out.println(t.magHeadingHorizontalNorm.minValue, 6);

    out.print("mag_heading_horizontal_norm_max: ");
    out.println(t.magHeadingHorizontalNorm.maxValue, 6);

    out.print("mag_yaw_combined_trust_mean: ");
    out.println(t.magYawCombinedTrust.mean(), 6);

    out.print("mag_yaw_combined_trust_min: ");
    out.println(t.magYawCombinedTrust.minValue, 6);

    out.print("mag_yaw_combined_trust_max: ");
    out.println(t.magYawCombinedTrust.maxValue, 6);

    out.print("mag_yaw_correction_rate_mean_deg_s: ");
    out.println(t.magYawCorrectionRateDegS.mean(), 8);

    out.print("mag_yaw_correction_rate_min_deg_s: ");
    out.println(t.magYawCorrectionRateDegS.minValue, 8);

    out.print("mag_yaw_correction_rate_max_deg_s: ");
    out.println(t.magYawCorrectionRateDegS.maxValue, 8);

    out.print("mag_yaw_correction_step_mean_deg: ");
    out.println(t.magYawCorrectionStepDeg.mean(), 9);

    out.print("mag_yaw_correction_step_min_deg: ");
    out.println(t.magYawCorrectionStepDeg.minValue, 9);

    out.print("mag_yaw_correction_step_max_deg: ");
    out.println(t.magYawCorrectionStepDeg.maxValue, 9);

    out.print("mag_trusted_delta: ");
    out.println(t.magTrustedDelta);

    out.print("mag_rejected_delta: ");
    out.println(t.magRejectedDelta);

    out.print("mag_heading_valid_delta: ");
    out.println(t.magHeadingValidDelta);

    out.print("mag_heading_rejected_delta: ");
    out.println(t.magHeadingRejectedDelta);

    out.print("mag_yaw_updates_delta: ");
    out.println(t.magYawUpdatesDelta);

    out.print("mag_yaw_gate_open_delta: ");
    out.println(t.magYawGateOpenDelta);

    out.print("mag_yaw_gate_closed_delta: ");
    out.println(t.magYawGateClosedDelta);

    out.print("mag_yaw_apply_allowed_delta: ");
    out.println(t.magYawApplyAllowedDelta);

    out.print("mag_yaw_applied_delta: ");
    out.println(t.magYawAppliedDelta);

    out.print("mag_yaw_last_error_deg: ");
    out.println(t.lastMagYawErrorDeg, 6);

    out.print("mag_yaw_last_correction_rate_deg_s: ");
    out.println(t.lastMagYawCorrectionRateDegS, 6);

    out.print("mag_yaw_last_correction_step_deg: ");
    out.println(t.lastMagYawCorrectionStepDeg, 6);

    out.print("mag_yaw_reject_no_reference_delta: ");
    out.println(t.magYawRejectNoReferenceDelta);

    out.print("mag_yaw_reject_heading_invalid_delta: ");
    out.println(t.magYawRejectHeadingInvalidDelta);

    out.print("mag_yaw_reject_mag_not_trusted_delta: ");
    out.println(t.magYawRejectMagNotTrustedDelta);

    out.print("mag_yaw_reject_mag_stale_delta: ");
    out.println(t.magYawRejectMagStaleDelta);

    out.print("mag_yaw_reject_horizontal_bad_delta: ");
    out.println(t.magYawRejectHorizontalBadDelta);

    out.print("mag_yaw_reject_innovation_too_large_delta: ");
    out.println(t.magYawRejectInnovationTooLargeDelta);

    out.print("mag_yaw_reject_gyro_moving_delta: ");
    out.println(t.magYawRejectGyroMovingDelta);

    out.print("mag_yaw_reject_accel_not_trusted_delta: ");
    out.println(t.magYawRejectAccelNotTrustedDelta);
    out.println("==============================================================================");
    out.println("STATIC TEST REPORT END");
    out.println("==============================================================================");
    return true;
}

bool StaticTestRunner::printLastSummary(Stream& out) const {
    const StaticRuntimeTest* completed = lastCompleted();
    if (completed == nullptr) {
        out.println("# ERR no completed static test summary");
        return false;
    }

    const StaticRuntimeTest& t = *completed;
    DiagnosticTestSummary summary;
    summary.kind = DiagnosticTestKind::Static;
    summary.durationMs = t.finishedElapsedMs;
    summary.stoppedByCommand = t.stoppedByCommand;
    summary.samples = t.samples;
    summary.hwTimestampSamples = t.hwTs;
    summary.fallbackTimestampSamples = t.fallbackTs;
    summary.badTimestampSamples = t.badTs;
    summary.estimatedDroppedSamples = t.droppedEstimate;
    summary.recoveryRequests = t.recoveryRequests;
    summary.fifoOverruns = t.fifoOverrunDelta;
    summary.fifoFull = t.fifoFullDelta;
    summary.fifoUnknown = t.fifoUnknownDelta;
    summary.networkMetricsValid = t.networkMetricsValid;
    summary.wifiDisconnects = t.wifiDisconnectsDelta;
    summary.wifiConnectTimeouts = t.wifiConnectTimeoutsDelta;
    summary.udpSendFailures = t.udpSendFailuresDelta;
    summary.rotationSendFailures = t.rotationSendFailuresDelta;
    summary.rotationMissedDeadlines = t.rotationMissedDeadlinesDelta;
    summary.rotationLateEvents = t.rotationLateEventsDelta;
    summary.txPressureFailures = t.txPressureFailuresDelta;
    summary.txOtherFailures = t.txOtherFailuresDelta;
    summary.udpRebindSuccesses = t.udpRebindSuccessesDelta;
    summary.udpRebindFailures = t.udpRebindFailuresDelta;
    summary.udpFullReopens = t.udpFullReopensDelta;
    summary.staticMetricsValid = true;
    summary.gyroMeanDps = (t.gyroAfterRadS.mean() * MATH_RAD_TO_DEG).norm();
    summary.gyroStdDps = (t.gyroAfterRadS.stddev() * MATH_RAD_TO_DEG).norm();
    summary.accelNormMeanG = t.accelNormG.mean();
    summary.accelNormStdG = t.accelNormG.stddev();
    summary.tempStartC = t.tempStartC;
    summary.tempEndC = t.tempEndC;
    summary.magMetricsValid = true;
    summary.magTrusted = t.magTrustedDelta;
    summary.magRejected = t.magRejectedDelta;
    diagnosticTestPrintSummary(out, summary);
    return true;
}

void StaticTestRunner::finish() {
    if (!active() || output_ == nullptr) return;
    Stream& out = *output_;
    StaticRuntimeTest& t = *deps_.activeTest;

    // No text formatting occurs before the measured window has closed. The
    // last partial block and immutable counter snapshots are finalized once;
    // the large human-readable report is emitted only on an explicit command.
    flushStats(t);
    const uint32_t finishedMs = millis();
    t.finishedElapsedMs = finishedMs - t.startMs;
    t.stoppedByCommand = t.stopRequested;
    if (t.poseCaptured) {
        t.eulerEndDeg = t.qEnd.toEulerXYZ() * MATH_RAD_TO_DEG;
    }

    const PerfDelta perf = perfDelta(t);
    t.perfSampleCallsDelta = perf.sampleCalls;
    t.perfSampleSumUsDelta = perf.sampleSumUs;
    t.perfFifoCallsDelta = perf.fifoCalls;
    t.perfFifoSumUsDelta = perf.fifoSumUs;
    t.perfEmptyPollsDelta = perf.emptyPolls;
    t.perfIrqEventsDelta = perf.irqEvents;
    t.perfFallbackPollsDelta = perf.fallbackPolls;
    t.perfFallbackEventsDelta = perf.fallbackEvents;

    const auto& fs = deps_.fifo->stats();
    t.fifoOverrunDelta = fs.overrunEvents - t.fifoOverrunAtStart;
    t.fifoFullDelta = fs.fullEvents - t.fifoFullAtStart;
    t.fifoUnknownDelta = fs.unknownWords - t.fifoUnknownAtStart;
    t.fifoHwTsDelta = fs.hwTimestampAssigned - t.fifoHwTsAtStart;
    t.fifoFbTsDelta = fs.fallbackTimestampAssigned - t.fifoFbTsAtStart;

    if (t.networkMetricsValid && deps_.wifi != nullptr && deps_.slimevr != nullptr) {
        const TrackerWifiManagerStatus wifi = deps_.wifi->status();
        const SlimeVROutputRuntimeStatus slime = deps_.slimevr->status();
        t.wifiDisconnectsDelta = wifi.disconnects - t.wifiDisconnectsAtStart;
        t.wifiConnectTimeoutsDelta = wifi.connectTimeouts - t.wifiConnectTimeoutsAtStart;
        t.udpSendFailuresDelta = slime.sendFailures - t.udpSendFailuresAtStart;
        t.rotationSendFailuresDelta = slime.rotationSendFailures - t.rotationSendFailuresAtStart;
        t.rotationMissedDeadlinesDelta =
            slime.rotationMissedDeadlines - t.rotationMissedDeadlinesAtStart;
        t.rotationLateEventsDelta = slime.rotationLateEvents - t.rotationLateEventsAtStart;
        t.txPressureFailuresDelta = slime.txPressureFailures - t.txPressureFailuresAtStart;
        t.txOtherFailuresDelta = slime.txOtherFailures - t.txOtherFailuresAtStart;
        t.udpRebindSuccessesDelta =
            slime.udpTransportRebindSuccesses - t.udpRebindSuccessesAtStart;
        t.udpRebindFailuresDelta =
            slime.udpTransportRebindFailures - t.udpRebindFailuresAtStart;
        t.udpFullReopensDelta =
            slime.udpFullReopenEscalations - t.udpFullReopensAtStart;
    }

    const auto& ms = deps_.magProcessor->stats();
    const auto& hs = deps_.magHeading->stats();
    const auto& ys = deps_.magYawCorrection->stats();
    t.magRefValidEnd = deps_.magHeadingRef != nullptr && deps_.magHeadingRef->valid;
    t.magTrustedDelta = ms.trustedSamples - t.magTrustedAtStart;
    t.magRejectedDelta = ms.rejectedSamples - t.magRejectedAtStart;
    t.magHeadingValidDelta = hs.valid - t.magHeadingValidAtStart;
    t.magHeadingRejectedDelta = hs.rejected - t.magHeadingRejectedAtStart;
    t.magYawUpdatesDelta = ys.updates - t.magYawUpdatesAtStart;
    t.magYawGateOpenDelta = ys.gateOpenCount - t.magYawGateOpenAtStart;
    t.magYawGateClosedDelta = ys.gateClosedCount - t.magYawGateClosedAtStart;
    t.magYawApplyAllowedDelta = ys.applyAllowedCount - t.magYawApplyAllowedAtStart;
    t.magYawAppliedDelta = ys.appliedCount - t.magYawAppliedAtStart;
    t.magYawRejectNoReferenceDelta =
        ys.rejectNoReference - t.magYawRejectNoReferenceAtStart;
    t.magYawRejectHeadingInvalidDelta =
        ys.rejectHeadingInvalid - t.magYawRejectHeadingInvalidAtStart;
    t.magYawRejectMagNotTrustedDelta =
        ys.rejectMagNotTrusted - t.magYawRejectMagNotTrustedAtStart;
    t.magYawRejectMagStaleDelta =
        ys.rejectMagStale - t.magYawRejectMagStaleAtStart;
    t.magYawRejectHorizontalBadDelta =
        ys.rejectHorizontalBad - t.magYawRejectHorizontalBadAtStart;
    t.magYawRejectInnovationTooLargeDelta =
        ys.rejectInnovationTooLarge - t.magYawRejectInnovationTooLargeAtStart;
    t.magYawRejectGyroMovingDelta =
        ys.rejectGyroMoving - t.magYawRejectGyroMovingAtStart;
    t.magYawRejectAccelNotTrustedDelta =
        ys.rejectAccelNotTrusted - t.magYawRejectAccelNotTrustedAtStart;
    if (deps_.lastMagYawCorrection != nullptr) {
        t.lastMagYawErrorDeg = deps_.lastMagYawCorrection->errorDeg;
        t.lastMagYawCorrectionRateDegS =
            deps_.lastMagYawCorrection->correctionRateDegS;
        t.lastMagYawCorrectionStepDeg =
            deps_.lastMagYawCorrection->correctionStepDeg;
    }

    const bool completedOk = t.samples > 0u &&
                             t.gyroAfterRadS.count > 0u &&
                             t.tempC.count > 0u;
    if (completedOk) {
        *deps_.lastCompletedTest = t;
        deps_.lastCompletedTest->active = false;
        deps_.lastCompletedTest->stopRequested = false;
        *deps_.lastCompletedValid = true;
        *deps_.lastCompletedFinishedMs = finishedMs;
        out.println("STATIC TEST DONE");
        out.println("# detailed report retained; use: test report static");
    } else {
        out.println("# ERR static test completed without usable samples");
    }

    t.reset();
    statsBlock_.reset();
    output_ = nullptr;
}

void StaticTestRunner::flushStats(StaticRuntimeTest& test) {
    if (statsBlock_.bufferedSamples == 0u) return;
    statsBlock_.mergeInto(test);
    statsBlock_.reset();
}

bool StaticTestRunner::ready() const {
    return deps_.activeTest != nullptr &&
           deps_.lastCompletedTest != nullptr &&
           deps_.lastCompletedValid != nullptr &&
           deps_.lastCompletedFinishedMs != nullptr &&
           deps_.quality != nullptr &&
           deps_.fifo != nullptr &&
           deps_.perf != nullptr &&
           deps_.config != nullptr &&
           deps_.wifi != nullptr &&
           deps_.slimevr != nullptr &&
           deps_.magProcessor != nullptr &&
           deps_.magHeading != nullptr &&
           deps_.magYawCorrection != nullptr &&
           deps_.magHeadingRef != nullptr;
}

void StaticTestRunner::snapshotPerfCounters(StaticRuntimeTest& t) const {
    t.perfFifoProcessCallsAtStart = deps_.perf->fifoProcessCalls;
    t.perfFifoProcessSumUsAtStart = deps_.perf->fifoProcessSumUs;
    t.perfSampleProcessCallsAtStart = deps_.perf->sampleProcessCalls;
    t.perfSampleProcessSumUsAtStart = deps_.perf->sampleProcessSumUs;
    t.perfFifoEmptyPollsAtStart = deps_.perf->fifoEmptyPolls;
    t.perfFifoIrqEventsAtStart = deps_.perf->fifoIrqEvents;
    t.perfFifoFallbackStatusPollsAtStart = deps_.perf->fifoFallbackStatusPolls;
    t.perfFifoFallbackEventsAtStart = deps_.perf->fifoFallbackEvents;
}

StaticTestRunner::PerfDelta StaticTestRunner::perfDelta(const StaticRuntimeTest& t) const {
    PerfDelta d;
    d.sampleCalls = deps_.perf->sampleProcessCalls - t.perfSampleProcessCallsAtStart;
    d.sampleSumUs = deps_.perf->sampleProcessSumUs - t.perfSampleProcessSumUsAtStart;
    d.fifoCalls = deps_.perf->fifoProcessCalls - t.perfFifoProcessCallsAtStart;
    d.fifoSumUs = deps_.perf->fifoProcessSumUs - t.perfFifoProcessSumUsAtStart;
    d.emptyPolls = deps_.perf->fifoEmptyPolls - t.perfFifoEmptyPollsAtStart;
    d.irqEvents = deps_.perf->fifoIrqEvents - t.perfFifoIrqEventsAtStart;
    d.fallbackPolls = deps_.perf->fifoFallbackStatusPolls - t.perfFifoFallbackStatusPollsAtStart;
    d.fallbackEvents = deps_.perf->fifoFallbackEvents - t.perfFifoFallbackEventsAtStart;
    return d;
}

void StaticTestRunner::printProgress(Stream& out, uint32_t elapsedMs) const {
    const StaticRuntimeTest& t = *deps_.activeTest;
    const PerfDelta perf = perfDelta(t);
    out.print("# test progress elapsed_s="); out.print(elapsedMs / 1000UL);
    out.print(" samples="); out.print(t.samples);
    out.print(" hw_ts="); out.print(t.hwTs);
    out.print(" fb_ts="); out.print(t.fallbackTs);
    out.print(" dropped_est="); out.print(t.droppedEstimate);
    out.print(" accel_norm_mean="); out.print(t.accelNormG.mean(), 6);
    out.print(" gyro_after_mean_dps_norm="); out.print((t.gyroAfterRadS.mean() * MATH_RAD_TO_DEG).norm(), 6);
    out.print(" static_update_max_us="); out.print(t.maxUpdateUs);
    out.print(" sample_process_avg_us="); out.print(staticTestAvgUs(perf.sampleSumUs, perf.sampleCalls), 3);
    out.print(" sample_process_max_us="); out.print(t.maxSampleProcessUs);
    out.print(" fifo_process_avg_us="); out.print(staticTestAvgUs(perf.fifoSumUs, perf.fifoCalls), 3);
    out.print(" fifo_process_max_us="); out.println(t.maxFifoProcessUs);
}

} // namespace tracker
