#include "runtime/runtime_status_reporter.hpp"

namespace tracker {


const char* runtimeStatusStreamModeName(TrackerStreamMode mode) {
    switch (mode) {
        case TrackerStreamMode::Off: return "off";
        case TrackerStreamMode::Raw: return "raw";
        case TrackerStreamMode::Scaled: return "scaled";
        case TrackerStreamMode::Quat: return "quat";
        case TrackerStreamMode::Heartbeat: return "heartbeat";
        case TrackerStreamMode::Debug:
        default: return "debug";
    }
}

float runtimeStatusMagHeadingErrorDeg(const MagHeadingReferenceState& ref,
                                             const MagHeadingSample& heading) {
    if (!ref.valid || !heading.valid) {
        return 0.0f;
    }
    return wrapPi(heading.magneticNorthWorldYawRad - ref.worldYawRad) * MATH_RAD_TO_DEG;
}

void runtimeStatusPrint(Stream& out, const RuntimeStatusReporterDeps& deps) {
    out.print("uptime_ms="); out.println(millis());
    out.print("config_loaded_from_nvs="); out.println(deps.configLoadedFromNvs ? "yes" : "no");
    out.print("spi_hz="); out.println(deps.spiHz);
    out.print("runtime_samples="); out.println(deps.runtimeSamples);
    out.print("fifo_int_count="); out.println(deps.fifoIntCount);
    out.print("fifo_int_missed="); out.println(deps.fifoEvents ? deps.fifoEvents->missedIrqCount() : 0UL);
    out.print("fifo_status_fallback_events="); out.println(deps.fifoEvents ? deps.fifoEvents->fallbackEvents() : 0UL);
    out.print("fifo_wait_timeouts="); out.println(deps.fifoEvents ? deps.fifoEvents->waitTimeouts() : 0UL);
    out.print("latest_temp_c="); out.println(deps.latestTempC, 3);

    out.print("mag_enabled="); out.println(deps.config && deps.config->data.magCal.driverEnabled ? "yes" : "no");
    out.print("mag_runtime_samples="); out.println(deps.magState ? deps.magState->samples : 0UL);
    out.print("mag_fifo_armed="); out.println(deps.magState && deps.magState->fifoArmed ? "yes" : "no");

    out.print("gyro_bias_valid="); out.println(deps.imuCal && deps.imuCal->gyroBiasValid ? "yes" : "no");
    const GyroTempCompSnapshot tempSnap = deps.gyroTempComp
        ? deps.gyroTempComp->snapshot(deps.latestTempC)
        : GyroTempCompSnapshot{};
    out.print("gyro_bias_model_valid="); out.println(tempSnap.valid ? "yes" : "no");
    out.print("gyro_temp_valid="); out.println(tempSnap.temperatureModelValid ? "yes" : "no");
    out.print("gyro_temp_enabled="); out.println(tempSnap.enabled ? "yes" : "no");
    out.print("gyro_temp_range_valid="); out.println(tempSnap.hasCalibratedRange ? "yes" : "no");
    out.print("gyro_temp_out_of_range="); out.println(tempSnap.tempOutOfRange ? "yes" : "no");
    out.print("gyro_temp_soft_extrapolated="); out.println(tempSnap.tempSoftExtrapolated ? "yes" : "no");
    out.print("gyro_temp_hard_extrapolated="); out.println(tempSnap.tempHardExtrapolated ? "yes" : "no");
    out.print("gyro_temp_distance_to_range_c="); out.println(tempSnap.tempDistanceToRangeC, 6);
    out.print("gyro_temp_extrapolation_confidence="); out.println(tempSnap.extrapolationConfidence, 6);
    out.print("gyro_temp_quality_flag=0x");
    out.println((tempSnap.valid && tempSnap.enabled && tempSnap.hasCalibratedRange && tempSnap.tempOutOfRange)
        ? imu_quality_flags::TEMP_COMP_OUT_OF_RANGE
        : 0u, HEX);
    out.print("gyro_temp_fit_quality="); out.println(tempSnap.fitQuality, 6);

    out.print("runtime_bias_enabled="); out.println(deps.runtimeBias && deps.runtimeBias->enabled ? "yes" : "no");
    out.print("runtime_bias_updates="); out.println(deps.runtimeBias ? deps.runtimeBias->updates : 0UL);
    const float runtimeTrimNormDps = deps.runtimeBias
        ? deps.runtimeBias->runtimeTrimRadS.norm() * MATH_RAD_TO_DEG
        : 0.0f;
    out.print("runtime_bias_trim_norm_dps="); out.println(runtimeTrimNormDps, 8);

    out.print("accel_cal_valid="); out.println(deps.imuCal && deps.imuCal->accelCalValid ? "yes" : "no");
    out.print("tracking_state="); out.println(deps.trackingStateName ? deps.trackingStateName : "UNKNOWN");
    if (deps.trackingState != nullptr) {
        deps.trackingState->printRecoveryStatus(out);
    }
    out.print("last_output_confidence="); out.println(deps.lastOutputConfidence, 6);
    out.print("quality_recovery_requested="); out.println(deps.quality && deps.quality->recoveryRequested() ? "yes" : "no");

    const Ahrs6DofConfig& acfg = deps.ahrs->config();
    const Ahrs6DofStats& ast = deps.ahrs->stats();
    out.print("ahrs_accel_enabled="); out.println(acfg.accelCorrectionEnabled ? "yes" : "no");
    out.print("ahrs_adaptive_accel="); out.println(acfg.adaptiveAccelCorrection ? "yes" : "no");
    out.print("ahrs_accel_kp="); out.println(acfg.accelKp, 6);
    out.print("ahrs_accel_trust="); out.println(ast.lastAccelGate.trust, 6);
    out.print("ahrs_accel_variance_trust="); out.println(ast.lastAccelNormVarianceTrust, 6);
    out.print("ahrs_gyro_motion_trust="); out.println(ast.lastGyroMotionTrust, 6);
    out.print("ahrs_accel_norm_variance_g2="); out.println(ast.accelNormVarianceG2, 9);
    out.print("ahrs_startup_accel_rejects="); out.println(ast.startupAccelRejectedCount);

    out.print("stream_mode="); out.println(deps.streamState ? runtimeStatusStreamModeName(deps.streamState->mode) : "off");
    out.print("stream_rate_hz="); out.println(deps.streamState ? deps.streamState->rateHz : 0UL);

    const bool preparedEnabled = deps.preparedOutput && deps.config && deps.preparedOutput->enabled(*deps.config);
    out.print("prepared_output_enabled="); out.println(preparedEnabled ? "yes" : "no");
    TrackerPreparedOutputSnapshot ps;
    const bool preparedValid = deps.preparedOutput ? deps.preparedOutput->copy(ps) : false;
    out.print("prepared_output_valid="); out.println(preparedValid ? "yes" : "no");
    out.print("prepared_output_seq="); out.println(ps.sequence);
    out.print("prepared_output_sample="); out.println(ps.runtimeSample);
    out.print("prepared_output_quality_flags=0x"); out.println(ps.qualityFlags, HEX);
    out.print("prepared_output_confidence="); out.println(ps.confidence, 6);
    if (preparedValid && ps.timestampUs > 0) {
        const uint64_t nowUs = static_cast<uint64_t>(micros());
        const uint64_t ageUs = nowUs >= ps.timestampUs ? (nowUs - ps.timestampUs) : 0;
        out.print("prepared_output_age_ms="); out.println(static_cast<uint32_t>(ageUs / 1000ULL));
    }

    const Vec3 e = deps.ahrs->eulerDeg();
    const Quat q = deps.ahrs->quaternionPositiveW();
    out.print("quat=");
    out.print(q.w, 7); out.print(',');
    out.print(q.x, 7); out.print(',');
    out.print(q.y, 7); out.print(',');
    out.println(q.z, 7);
    out.print("euler_deg=");
    out.print(e.x, 3); out.print(',');
    out.print(e.y, 3); out.print(',');
    out.println(e.z, 3);
}

void runtimeStatusPrintHealth(Stream& out, const RuntimeStatusReporterDeps& deps) {
    runtimeStatusPrint(out, deps);
    out.println("# FIFO SUMMARY");
    const auto& fs = deps.fifo->stats();
    out.print("fifo_words_read="); out.println(fs.fifoWordsRead);
    out.print("imu_samples_produced="); out.println(fs.imuSamplesProduced);
    out.print("gyro_words="); out.println(fs.gyroWords);
    out.print("accel_words="); out.println(fs.accelWords);
    out.print("timestamp_words="); out.println(fs.timestampWords);
    out.print("temperature_words="); out.println(fs.tempWords);
    out.print("sensorhub0_words="); out.println(fs.sensorHubSlave0Words);
    out.print("sensorhub_nack_words="); out.println(fs.sensorHubNackWords);
    out.print("mag_samples_produced="); out.println(fs.magSamplesProduced);
    out.print("mag_queue_overflow="); out.println(fs.magQueueOverflow);

    const auto& ms = deps.magProcessor->stats();
    out.print("mag_processed_samples="); out.println(ms.processedSamples);
    out.print("mag_trusted_samples="); out.println(ms.trustedSamples);
    out.print("mag_rejected_samples="); out.println(ms.rejectedSamples);
    out.print("mag_last_reject_flags=0x"); out.println(deps.lastMagProcessed->rejectFlags, HEX);
    out.print("mag_last_body_norm="); out.println(deps.lastMagProcessed->bodyNorm, 6);
    out.print("mag_last_trusted="); out.println(deps.lastMagProcessed->trusted ? "yes" : "no");

    const auto& hs = deps.magHeading->stats();
    out.print("mag_heading_valid="); out.println(deps.lastMagHeading->valid ? "yes" : "no");
    out.print("mag_heading_reject_flags=0x"); out.println(deps.lastMagHeading->rejectFlags, HEX);
    out.print("mag_heading_valid_count="); out.println(hs.valid);
    out.print("mag_heading_north_minus_ahrs_yaw_deg="); out.println(deps.lastMagHeading->yawInnovationDeg, 6);
    out.print("mag_heading_ref_valid="); out.println(deps.magHeadingRef->valid ? "yes" : "no");
    out.print("mag_heading_auto_ref_done="); out.println(deps.magHeadingAutoRef->done ? "yes" : "no");
    out.print("mag_heading_auto_ref_reject_flags=0x"); out.println(deps.magHeadingAutoRef->lastRejectFlags, HEX);
    out.print("mag_heading_error_to_ref_deg=");
    out.println(runtimeStatusMagHeadingErrorDeg(*deps.magHeadingRef, *deps.lastMagHeading), 6);
    out.print("mag_yaw_gate_open=");
    out.println(deps.lastMagYawCorrection->gateOpen ? "yes" : "no");
    out.print("mag_yaw_reject_flags=0x");
    out.println(deps.lastMagYawCorrection->rejectFlags, HEX);
    out.print("mag_yaw_cooldown_active=");
    out.println(deps.lastMagYawCorrection->cooldownActive ? "yes" : "no");

    out.print("mag_yaw_cooldown_remaining_ms=");
    out.println(deps.lastMagYawCorrection->cooldownRemainingMs);
    out.print("mag_yaw_error_deg=");
    out.println(deps.lastMagYawCorrection->errorDeg, 6);
    out.print("mag_yaw_correction_rate_deg_s=");
    out.println(deps.lastMagYawCorrection->correctionRateDegS, 6);
    out.print("mag_yaw_correction_step_deg=");
    out.println(deps.lastMagYawCorrection->correctionStepDeg, 6);

    out.print("mag_yaw_apply_enabled=");
    out.println(deps.config->data.magYaw.applyEnabled ? "yes" : "no");
    out.print("mag_yaw_applied_last=");
    out.println(deps.lastMagYawCorrection->applied ? "yes" : "no");

    out.print("unknown_words="); out.println(fs.unknownWords);
    out.print("overrun_events="); out.println(fs.overrunEvents);
    out.print("full_events="); out.println(fs.fullEvents);
    out.print("tag_counter_jumps="); out.println(fs.tagCounterJumps);
    out.print("hw_ts_assigned="); out.println(fs.hwTimestampAssigned);
    out.print("fb_ts_assigned="); out.println(fs.fallbackTimestampAssigned);

    out.println("# QUALITY SUMMARY");
    const auto& qc = deps.quality->counters();
    out.print("quality_samples="); out.println(qc.samples);
    out.print("estimated_dropped_samples="); out.println(qc.estimatedDroppedSamples);
    out.print("large_gap_samples="); out.println(qc.largeGapSamples);
    out.print("gyro_saturated_samples="); out.println(qc.gyroSaturatedSamples);
    out.print("accel_saturated_samples="); out.println(qc.accelSaturatedSamples);
    out.print("fifo_recovery_requests="); out.println(qc.fifoRecoveryRequests);
    out.print("mean_dt_us="); out.println(qc.meanDtUs(), 6);
}

} // namespace tracker

