#include "runtime/mag_status_reporter.hpp"

namespace tracker {


const char* magAxisDeferredActionName(MagAxisAlignmentDeferredAction action) {
    switch (action) {
        case MagAxisAlignmentDeferredAction::None: return "none";
        case MagAxisAlignmentDeferredAction::Solve: return "solve";
        case MagAxisAlignmentDeferredAction::Stage: return "stage";
        case MagAxisAlignmentDeferredAction::CheckCandidateSlot: return "check_candidate_slot";
    }
    return "unknown";
}

float magStatusHeadingErrorToReferenceRad(const MagHeadingReferenceState& ref,
                                                 const MagHeadingSample& heading) {
    if (!ref.valid || !heading.valid) {
        return 0.0f;
    }
    return wrapPi(heading.magneticNorthWorldYawRad - ref.worldYawRad);
}

void magStatusPrintRuntime(Stream& out, const MagStatusReporterDeps& deps) {
    const MagRuntimeState& state = *deps.state;

    out.print("mag_runtime_enabled="); out.println(state.runtimeEnabled ? "yes" : "no");
    out.print("mag_hub_initialized="); out.println(state.hubInitialized ? "yes" : "no");
    out.print("mag_fifo_armed="); out.println(state.fifoArmed ? "yes" : "no");
    out.print("mag_last_init_ok="); out.println(state.lastInitOk ? "yes" : "no");
    out.print("mag_enable_failures="); out.println(state.enableFailures);
    out.print("mag_runtime_samples="); out.println(state.samples);
    out.print("mag_last_age_ms="); out.println(state.samples > 0 ? millis() - state.lastSampleMs : 0UL);
    out.print("mag_last_t_us="); outputPrintU64Dec(out, state.lastRaw.t_us); out.println();
    out.print("mag_last_xyz=");
    out.print(state.lastRaw.x); out.print(',');
    out.print(state.lastRaw.y); out.print(',');
    out.println(state.lastRaw.z);
    out.print("mag_last_norm_raw="); out.println(state.lastNormRaw, 3);
    out.print("mag_last_flags=0x"); out.println(state.lastRaw.flags, HEX);
    out.print("mag_qmc_error="); out.println(deps.qmc ? deps.qmc->lastErrorName() : "unavailable");
    out.print("mag_hub_error="); out.println(deps.hub ? deps.hub->lastErrorName() : "unavailable");
}

void magStatusPrintProcessed(Stream& out, const MagStatusReporterDeps& deps) {
    const MagProcessedSample& m = *deps.lastProcessed;
    const MagRuntimeStats& s = deps.processor->stats();
    const MagRuntimeConfig& cfg = deps.runtimeConfig;

    const uint32_t nowMs = millis();
    const uint32_t ageMs = MagRuntimeProcessor::ageMsForUse(m, nowMs);
    const uint32_t rejectFlagsNow = MagRuntimeProcessor::rejectFlagsForUse(m, cfg, nowMs);
    const bool trustedNow = MagRuntimeProcessor::trustedForUse(m, cfg, nowMs);

    out.println("# MAG PROCESSED");

    out.print("processed_valid="); out.println(m.valid ? "yes" : "no");
    out.print("trusted="); out.println(trustedNow ? "yes" : "no");
    out.print("reject_flags=0x"); out.println(rejectFlagsNow, HEX);
    out.print("process_reject_flags=0x"); out.println(m.rejectFlags, HEX);
    out.print("seq="); out.println(m.seq);
    out.print("age_ms="); out.println(ageMs);
    out.print("received_ms="); out.println(m.receivedMs);
    out.print("t_us="); outputPrintU64Dec(out, m.t_us); out.println();
    out.print("gyro_endpoint_valid="); out.println(m.gyroEndpointValid ? "yes" : "no");
    out.print("gyro_endpoint_t_us="); outputPrintU64Dec(out, m.gyroTimestampUs); out.println();
    out.print("gyro_endpoint_skew_us="); out.println(m.gyroEndpointSkewUs);

    out.print("raw=");
    out.print(m.raw.x, 3); out.print(',');
    out.print(m.raw.y, 3); out.print(',');
    out.println(m.raw.z, 3);

    out.print("calibrated_mag_frame=");
    out.print(m.calibratedMagFrame.x, 6); out.print(',');
    out.print(m.calibratedMagFrame.y, 6); out.print(',');
    out.println(m.calibratedMagFrame.z, 6);

    out.print("body=");
    out.print(m.body.x, 6); out.print(',');
    out.print(m.body.y, 6); out.print(',');
    out.println(m.body.z, 6);

    out.print("norms_raw_cal_body=");
    out.print(m.rawNorm, 6); out.print(',');
    out.print(m.calibratedNorm, 6); out.print(',');
    out.println(m.bodyNorm, 6);

    out.print("config_cal_valid="); out.println(deps.config->data.magCal.calibrationValid ? "yes" : "no");
    out.print("config_axis_valid="); out.println(deps.config->data.magCal.axisAlignmentValid ? "yes" : "no");

    out.print("trust_norm_min_max=");
    out.print(deps.config->data.magCal.minTrustNorm, 6); out.print(',');
    out.println(deps.config->data.magCal.maxTrustNorm, 6);

    out.println("# MAG TRUST STATS");
    out.print("raw_samples="); out.println(s.rawSamples);
    out.print("processed_samples="); out.println(s.processedSamples);
    out.print("trusted_samples="); out.println(s.trustedSamples);
    out.print("rejected_samples="); out.println(s.rejectedSamples);

    out.print("raw_norm_min_mean_max=");
    out.print(s.rawNormMin, 6); out.print(',');
    out.print(s.rawNormMean(), 6); out.print(',');
    out.println(s.rawNormMax, 6);

    out.print("body_norm_min_mean_max=");
    out.print(s.bodyNormMin, 6); out.print(',');
    out.print(s.bodyNormMean(), 6); out.print(',');
    out.println(s.bodyNormMax, 6);

    out.print("trusted_body_norm_min_mean_max=");
    out.print(s.trustedBodyNormMin, 6); out.print(',');
    out.print(s.trustedBodyNormMean(), 6); out.print(',');
    out.println(s.trustedBodyNormMax, 6);

    out.print("reject_disabled="); out.println(s.rejectedDisabled);
    out.print("reject_raw_saturated="); out.println(s.rejectedRawSaturated);
    out.print("reject_raw_nonfinite="); out.println(s.rejectedRawNonfinite);
    out.print("reject_not_calibrated="); out.println(s.rejectedNotCalibrated);
    out.print("reject_axis_not_aligned="); out.println(s.rejectedAxisNotAligned);
    out.print("reject_norm_too_low="); out.println(s.rejectedNormTooLow);
    out.print("reject_norm_too_high="); out.println(s.rejectedNormTooHigh);
    out.print("reject_stale_process_time="); out.println(s.rejectedStale);
    out.print("reject_zero_norm="); out.println(s.rejectedZeroNorm);
}

void magStatusPrintHeading(Stream& out, const MagStatusReporterDeps& deps) {
    const MagHeadingSample& h = *deps.lastHeading;
    const MagHeadingStats& s = deps.headingEstimator->stats();
    const MagHeadingReferenceState& ref = *deps.headingRef;
    const MagHeadingAutoReferenceState& autoRef = *deps.headingAutoRef;

    const float errorToRefRad = magStatusHeadingErrorToReferenceRad(ref, h);
    const float errorToRefDeg = errorToRefRad * MATH_RAD_TO_DEG;

    out.println("# MAG HEADING");

    out.print("heading_valid="); out.println(h.valid ? "yes" : "no");
    out.print("heading_reject_flags=0x"); out.println(h.rejectFlags, HEX);
    out.print("mag_seq="); out.println(h.magSeq);
    out.print("mag_t_us="); outputPrintU64Dec(out, h.magTimestampUs); out.println();
    out.print("mag_received_ms="); out.println(h.magReceivedMs);

    out.print("mag_body=");
    out.print(h.magBody.x, 6); out.print(',');
    out.print(h.magBody.y, 6); out.print(',');
    out.println(h.magBody.z, 6);

    out.print("mag_world=");
    out.print(h.magWorld.x, 6); out.print(',');
    out.print(h.magWorld.y, 6); out.print(',');
    out.println(h.magWorld.z, 6);

    out.print("dip_deg="); out.println(h.dipDeg, 6);
    out.print("dip_rad="); out.println(h.dipRad, 9);

    out.print("mag_world_horizontal=");
    out.print(h.magWorldHorizontal.x, 6); out.print(',');
    out.print(h.magWorldHorizontal.y, 6); out.print(',');
    out.println(h.magWorldHorizontal.z, 6);

    out.print("norms_body_world_horizontal=");
    out.print(h.magBodyNorm, 6); out.print(',');
    out.print(h.magWorldNorm, 6); out.print(',');
    out.println(h.horizontalNorm, 6);

    out.print("magnetic_field_world_yaw_deg="); out.println(h.magneticFieldWorldYawDeg, 6);
    out.print("magnetic_north_world_yaw_deg="); out.println(h.magneticNorthWorldYawDeg, 6);
    out.print("ahrs_yaw_deg="); out.println(h.currentAhrsYawDeg, 6);

    // Old diagnostic, kept for visibility but do not use it for yaw correction.
    out.print("north_minus_ahrs_yaw_deg="); out.println(h.yawInnovationDeg, 6);

    out.println("# MAG HEADING REFERENCE");
    out.print("mag_ref_valid="); out.println(ref.valid ? "yes" : "no");
    out.print("mag_ref_world_yaw_deg="); out.println(ref.valid ? ref.worldYawDeg() : 0.0f, 6);
    out.print("mag_ref_age_ms="); out.println(ref.valid ? millis() - ref.setMs : 0UL);
    out.print("mag_ref_seq="); out.println(ref.magSeq);
    out.print("mag_error_to_ref_deg="); out.println(ref.valid && h.valid ? errorToRefDeg : 0.0f, 6);
    out.print("mag_error_to_ref_rad="); out.println(ref.valid && h.valid ? errorToRefRad : 0.0f, 9);

    out.println("# MAG HEADING AUTO REFERENCE");
    out.print("mag_auto_ref_enabled="); out.println(autoRef.enabled ? "yes" : "no");
    out.print("mag_auto_ref_done="); out.println(autoRef.done ? "yes" : "no");
    out.print("mag_auto_ref_stable_ms="); out.println(autoRef.stableSinceMs == 0 ? 0UL : millis() - autoRef.stableSinceMs);
    out.print("mag_auto_ref_set_count="); out.println(autoRef.setCount);
    out.print("mag_auto_ref_last_set_age_ms="); out.println(autoRef.setCount > 0 ? millis() - autoRef.lastSetMs : 0UL);
    out.print("mag_auto_ref_last_reject_flags=0x"); out.println(autoRef.lastRejectFlags, HEX);
    out.print("mag_auto_ref_last_gyro_norm_dps="); out.println(autoRef.lastGyroNormDps, 6);
    out.print("mag_auto_ref_last_accel_trust="); out.println(autoRef.lastAccelTrust, 6);
    out.print("mag_auto_ref_last_horizontal_trust="); out.println(autoRef.lastHorizontalTrust, 6);


    if (deps.lastFieldReliability) {
        const MagFieldReliabilityOutput& f = *deps.lastFieldReliability;
        out.println("# MAG FIELD RELIABILITY");
        out.print("field_state="); out.println(MagFieldReliabilityMonitor::stateName(f.state));
        out.print("field_trusted_for_yaw="); out.println(f.trustedForYaw ? "yes" : "no");
        out.print("field_flags=0x"); out.println(f.flags, HEX);
        out.print("field_reference_valid="); out.println(f.referenceValid ? "yes" : "no");
        out.print("field_stable_ms="); out.println(f.stableMs);
        out.print("field_norm_reference="); out.print(f.fieldNorm, 6); out.print(','); out.println(f.referenceNorm, 6);
        out.print("field_norm_relative_error="); out.println(f.normRelativeError, 6);
        out.print("field_dip_reference_deg="); out.print(f.dipDeg, 6); out.print(','); out.println(f.referenceDipDeg, 6);
        out.print("field_dip_error_deg="); out.println(f.dipErrorDeg, 6);
        out.print("field_reference_heading_yaw_deg="); out.println(f.referenceHeadingYawDeg, 6);
        out.print("field_reference_heading_error_deg="); out.println(f.referenceHeadingErrorDeg, 6);
        out.print("field_heading_step_deg="); out.println(f.headingStepDeg, 6);
        out.print("field_heading_instant_rate_deg_s="); out.println(f.headingInstantRateDegS, 6);
        out.print("field_heading_rate_deg_s="); out.println(f.headingRateDegS, 6);
        out.print("field_stationary_window_delta_deg="); out.println(f.stationaryWindowHeadingDeltaDeg, 6);
        out.print("field_stationary_window_rate_deg_s="); out.println(f.stationaryWindowHeadingRateDegS, 6);
        out.print("field_stationary_heading_jump_latched="); out.println(f.stationaryHeadingJumpLatched ? "yes" : "no");
        if (deps.fieldReliability) {
            const auto& fs = deps.fieldReliability->stats();
            out.print("field_state_transitions="); out.println(fs.stateTransitions);
            out.print("field_entered_disturbed="); out.println(fs.enteredDisturbed);
            out.print("field_entered_recovering="); out.println(fs.enteredRecovering);
            out.print("field_environment_changes_detected="); out.println(fs.environmentChangesDetected);
            out.print("field_references_acquired="); out.println(fs.referencesAcquired);
            out.print("field_stationary_heading_jump_rejects="); out.println(fs.stationaryHeadingJumpRejects);
            out.print("field_stationary_heading_jumps_latched="); out.println(fs.stationaryHeadingJumpsLatched);
            out.print("field_stationary_heading_returns="); out.println(fs.stationaryHeadingReturns);
        }
    }

    if (deps.axisAlignmentCollector && deps.axisAlignmentState) {
        const auto& c = *deps.axisAlignmentCollector;
        const auto& a = *deps.axisAlignmentState;
        out.println("# MAG AXIS BACKGROUND CANDIDATE");
        out.print("axis_candidate_intervals="); out.println(c.intervalCount());
        out.print("axis_candidate_independent_windows="); out.println(c.independentWindows());
        out.print("axis_candidate_excited_axes="); out.println(c.excitedAxes());
        out.print("axis_candidate_partition_confirmed_axes="); out.println(c.partitionConfirmedAxes());
        out.print("axis_candidate_ready="); out.println(c.readyToSolve() ? "yes" : "no");
        out.print("axis_candidate_rejected_gyro_skew="); out.println(c.stats().intervalsRejectedGyroSkew);
        out.print("axis_candidate_skipped_cadence="); out.println(c.stats().intervalsSkippedCadence);
        out.print("axis_candidate_rejected_capacity="); out.println(c.stats().intervalsRejectedCapacity);
        out.print("axis_candidate_reservoir_replaced="); out.println(c.stats().intervalsReservoirReplaced);
        out.print("axis_candidate_reservoir_skipped="); out.println(c.stats().intervalsReservoirSkipped);
        out.print("axis_candidate_staged="); out.println(a.candidateStaged ? "yes" : "no");
        out.print("axis_active_alignment_confirmed="); out.println(a.activeAlignmentConfirmed ? "yes" : "no");
        out.print("axis_candidate_blocked_existing="); out.println(a.blockedByExistingCandidate ? "yes" : "no");
        out.print("axis_solve_pending="); out.println(a.solvePending ? "yes" : "no");
        out.print("axis_stage_pending="); out.println(a.stagePending ? "yes" : "no");
        out.print("axis_deferred_action="); out.println(magAxisDeferredActionName(a.pendingAction));
        out.print("axis_last_solve_valid="); out.println(a.lastSolveValid ? "yes" : "no");
        out.print("axis_last_failure_reason="); out.println(magAxisAlignmentFailureReasonName(a.lastResult.failureReason));
        out.print("axis_last_refined="); out.println(a.lastResult.refined ? "yes" : "no");
        out.print("axis_last_improves_active="); out.println(a.lastResult.improvesActive ? "yes" : "no");
        out.print("axis_last_validation_passed="); out.println(a.lastResult.validationPassed ? "yes" : "no");
        out.print("axis_last_validation_winner_matches_training="); out.println(a.lastResult.validationWinnerMatchesTraining ? "yes" : "no");
        out.print("axis_last_coarse_winner_matches_training="); out.println(a.lastResult.coarseWinnerMatchesTraining ? "yes" : "no");
        out.print("axis_last_continuous_refinement_agreement="); out.println(a.lastResult.continuousRefinementAgreement ? "yes" : "no");
        out.print("axis_last_coarse_consensus_fallback_used="); out.println(a.lastResult.coarseConsensusFallbackUsed ? "yes" : "no");
        out.print("axis_last_score_deg="); out.println(a.lastResult.score, 6);
        out.print("axis_last_training_score_deg="); out.println(a.lastResult.trainingScore, 6);
        out.print("axis_last_validation_score_deg="); out.println(a.lastResult.validationScore, 6);
        out.print("axis_last_coarse_score_deg="); out.println(a.lastResult.coarseScore, 6);
        out.print("axis_last_second_score_deg="); out.println(a.lastResult.secondBestScore, 6);
        out.print("axis_last_training_second_score_deg="); out.println(a.lastResult.trainingSecondBestScore, 6);
        out.print("axis_last_validation_second_score_deg="); out.println(a.lastResult.validationSecondBestScore, 6);
        out.print("axis_last_normalized_separation="); out.println(a.lastResult.normalizedSeparation, 6);
        out.print("axis_last_training_validation_rotation_difference_deg="); out.println(a.lastResult.trainingValidationRotationDifferenceDeg, 6);
        out.print("axis_last_active_score_deg="); out.println(a.lastResult.activeScore, 6);
        out.print("axis_last_active_training_score_deg="); out.println(a.lastResult.activeTrainingScore, 6);
        out.print("axis_last_direction_error_deg="); out.println(a.lastResult.meanDirectionError, 6);
        out.print("axis_last_rotation_magnitude_error_deg="); out.println(a.lastResult.meanMagnitudeError, 6);
        out.print("axis_last_refinement_angle_deg="); out.println(a.lastResult.refinementAngleDeg, 6);
        out.print("axis_last_quality_score="); out.println(a.lastResult.qualityScore, 6);
        out.print("axis_last_refinement_evaluations="); out.println(a.lastResult.refinementEvaluations);
        out.print("axis_last_used_intervals="); out.println(a.lastResult.usedIntervals);
        out.print("axis_last_training_intervals="); out.println(a.lastResult.trainingUsedIntervals);
        out.print("axis_last_validation_intervals="); out.println(a.lastResult.validationUsedIntervals);
        out.print("axis_last_training_windows="); out.println(a.lastResult.trainingWindows);
        out.print("axis_last_validation_windows="); out.println(a.lastResult.validationWindows);
        out.print("axis_last_mean_observable_step_deg="); out.println(a.lastResult.meanObservableStepDeg, 6);
        out.print("axis_last_total_observable_rotation_deg="); out.println(a.lastResult.totalObservableRotationDeg, 6);
        out.print("axis_stage_attempts="); out.println(a.stageAttempts);
        out.print("axis_stage_successes="); out.println(a.stageSuccesses);
        out.print("axis_stage_failures="); out.println(a.stageFailures);
        out.print("axis_solve_service_calls="); out.println(a.solveServiceCalls);
        out.print("axis_storage_service_calls="); out.println(a.storageServiceCalls);
        out.print("axis_service_deferrals="); out.println(a.serviceDeferrals);
        out.print("axis_service_deferral_software_fifo="); out.println(a.serviceDeferralSoftwareFifo);
        out.print("axis_service_deferral_hardware_status="); out.println(a.serviceDeferralHardwareStatus);
        out.print("axis_service_deferral_hardware_busy="); out.println(a.serviceDeferralHardwareBusy);
        out.print("axis_service_deferral_output_deadline="); out.println(a.serviceDeferralOutputDeadline);
        out.print("axis_evidence_queued="); out.println(a.evidenceQueued);
        out.print("axis_evidence_processed="); out.println(a.evidenceProcessed);
        out.print("axis_evidence_dropped="); out.println(a.evidenceDropped);
        out.print("axis_evidence_stale_dropped="); out.println(a.evidenceStaleDropped);
        out.print("axis_evidence_service_deferrals="); out.println(a.evidenceServiceDeferrals);
        out.print("axis_evidence_queue_high_water="); out.println(a.evidenceQueueHighWater);
        out.print("axis_last_deferred_fifo_unread_words="); out.println(a.lastDeferredFifoUnreadWords);
        out.print("axis_max_deferred_fifo_unread_words="); out.println(a.maxDeferredFifoUnreadWords);
        out.print("axis_last_deferred_rotation_slack_ms="); out.println(a.lastDeferredRotationSlackMs);
        out.print("axis_last_deferred_reject_flags=0x"); out.println(a.lastDeferredRejectFlags, HEX);
        out.print("axis_last_solve_us="); out.println(a.lastSolveUs);
        out.print("axis_max_solve_us="); out.println(a.maxSolveUs);
        out.print("axis_last_storage_us="); out.println(a.lastStorageUs);
        out.print("axis_max_storage_us="); out.println(a.maxStorageUs);
        out.print("axis_last_solve_attempt_ms="); out.println(a.lastSolveAttemptMs);
        out.print("axis_last_storage_check_ms="); out.println(a.lastStorageCheckMs);
    }

    out.println("# MAG HEADING STATS");
    out.print("heading_attempts="); out.println(s.attempts);
    out.print("heading_valid_count="); out.println(s.valid);
    out.print("heading_rejected_count="); out.println(s.rejected);
    out.print("reject_mag_invalid="); out.println(s.rejectMagInvalid);
    out.print("reject_mag_not_trusted="); out.println(s.rejectMagNotTrusted);
    out.print("reject_quat_invalid="); out.println(s.rejectQuatInvalid);
    out.print("reject_world_nonfinite="); out.println(s.rejectWorldNonfinite);
    out.print("reject_horizontal_small="); out.println(s.rejectHorizontalSmall);
    out.print("last_valid_age_ms="); out.println(s.valid > 0 ? millis() - s.lastValidMs : 0UL);
}

void magStatusPrintYawCorrection(Stream& out, const MagStatusReporterDeps& deps) {
    const MagYawCorrectionConfig& cfg = deps.yawConfig;
    const MagYawCorrectionOutput& y = *deps.lastYawCorrection;
    const MagYawCorrectionStats& s = deps.yawCorrection->stats();

    out.println("# MAG YAW CORRECTION");

    out.print("enabled="); out.println(cfg.enabled ? "yes" : "no");
    out.print("apply_enabled="); out.println(cfg.applyEnabled ? "yes" : "no");
    out.print("gate_open="); out.println(y.gateOpen ? "yes" : "no");
    out.print("apply_allowed="); out.println(y.applyAllowed ? "yes" : "no");
    out.print("applied_last="); out.println(y.applied ? "yes" : "no");
    out.print("mode="); out.println(y.mode == MagYawCorrectionMode::Reacquiring ? "reacquiring" : "normal");
    out.print("reacquire_pending="); out.println(y.reacquirePending ? "yes" : "no");
    out.print("reacquire_active="); out.println(y.reacquireActive ? "yes" : "no");
    out.print("field_stable_ms="); out.println(y.fieldStableMs);
    out.print("mag_heading_rate_deg_s="); out.println(y.magneticHeadingRateDegS, 6);
    out.print("reject_flags=0x"); out.println(y.rejectFlags, HEX);
    out.print("cooldown_active="); out.println(y.cooldownActive ? "yes" : "no");
    out.print("cooldown_remaining_ms="); out.println(y.cooldownRemainingMs);
    out.print("cooldown_reason_flags=0x"); out.println(y.cooldownReasonFlags, HEX);
    out.print("mag_seq="); out.println(y.magSeq);
    out.print("mag_t_us="); outputPrintU64Dec(out, y.magTimestampUs); out.println();
    out.print("mag_age_ms="); out.println(y.magAgeMs);
    out.print("dt_ms="); out.println(y.dtMs);
    out.print("horizontal_norm="); out.println(y.horizontalNorm, 6);
    out.print("horizontal_trust="); out.println(y.horizontalTrust, 6);
    out.print("gyro_norm_dps="); out.println(y.gyroNormDps, 6);
    out.print("gyro_trust="); out.println(y.gyroTrust, 6);
    out.print("accel_trust="); out.println(y.accelTrust, 6);
    out.print("accel_gate_trust="); out.println(y.accelGateTrust, 6);
    out.print("combined_trust="); out.println(y.combinedTrust, 6);
    out.print("error_to_ref_deg="); out.println(y.errorDeg, 6);
    out.print("error_to_ref_rad="); out.println(y.errorRad, 9);
    out.print("correction_rate_deg_s="); out.println(y.correctionRateDegS, 6);
    out.print("correction_rate_rad_s="); out.println(y.correctionRateRadS, 9);
    out.print("correction_step_deg="); out.println(y.correctionStepDeg, 6);
    out.print("correction_step_rad="); out.println(y.correctionStepRad, 9);

    out.println("# CONFIG");
    out.print("max_innovation_deg="); out.println(cfg.maxInnovationDeg, 3);
    out.print("horizontal_norm_bad_good=");
    out.print(cfg.horizontalNormBad, 3); out.print(','); out.println(cfg.horizontalNormGood, 3);
    out.print("gyro_norm_good_bad_dps=");
    out.print(cfg.gyroNormGoodDps, 3); out.print(','); out.println(cfg.gyroNormBadDps, 3);
    out.print("accel_trust_bad_good=");
    out.print(cfg.accelTrustBad, 3); out.print(','); out.println(cfg.accelTrustGood, 3);
    out.print("max_mag_age_ms="); out.println(cfg.maxMagAgeMs);
    out.print("time_constant_s="); out.println(cfg.timeConstantS, 3);
    out.print("max_rate_deg_s="); out.println(cfg.maxCorrectionRateDegS, 3);
    out.print("max_step_deg="); out.println(cfg.maxCorrectionStepDeg, 3);
    out.print("reacquire_innovation_max_deg="); out.println(cfg.reacquireInnovationMaxDeg, 3);
    out.print("reacquire_min_field_stable_ms="); out.println(cfg.reacquireMinFieldStableMs);
    out.print("reacquire_max_heading_rate_deg_s="); out.println(cfg.reacquireMaxHeadingRateDegS, 3);
    out.print("reacquire_time_constant_s="); out.println(cfg.reacquireTimeConstantS, 3);

    out.println("# STATS");
    out.print("updates="); out.println(s.updates);
    out.print("gate_open_count="); out.println(s.gateOpenCount);
    out.print("gate_closed_count="); out.println(s.gateClosedCount);
    out.print("apply_allowed_count="); out.println(s.applyAllowedCount);
    out.print("applied_count="); out.println(s.appliedCount);
    out.print("last_abs_error_deg="); out.println(s.lastAbsErrorDeg, 6);
    out.print("mean_abs_error_deg="); out.println(s.meanAbsErrorDeg(), 6);
    out.print("max_abs_error_deg="); out.println(s.maxAbsErrorDeg, 6);
    out.print("last_correction_step_deg="); out.println(s.lastCorrectionStepDeg, 6);
    out.print("max_abs_correction_step_deg="); out.println(s.maxAbsCorrectionStepDeg, 6);
    out.print("reject_disabled="); out.println(s.rejectDisabled);
    out.print("reject_apply_disabled="); out.println(s.rejectApplyDisabled);
    out.print("reject_no_reference="); out.println(s.rejectNoReference);
    out.print("reject_heading_invalid="); out.println(s.rejectHeadingInvalid);
    out.print("reject_mag_not_trusted="); out.println(s.rejectMagNotTrusted);
    out.print("reject_mag_stale="); out.println(s.rejectMagStale);
    out.print("reject_horizontal_bad="); out.println(s.rejectHorizontalBad);
    out.print("reject_innovation_too_large="); out.println(s.rejectInnovationTooLarge);
    out.print("reject_gyro_moving="); out.println(s.rejectGyroMoving);
    out.print("reject_accel_not_trusted="); out.println(s.rejectAccelNotTrusted);
    out.print("reject_cooldown="); out.println(s.rejectCooldown);
    out.print("reject_reacquire_pending="); out.println(s.rejectReacquirePending);
    out.print("reacquire_gate_open_count="); out.println(s.reacquireGateOpenCount);
    out.print("reacquire_applied_count="); out.println(s.reacquireAppliedCount);
    out.print("reacquire_completed_count="); out.println(s.reacquireCompletedCount);
    out.print("reject_dt_invalid="); out.println(s.rejectDtInvalid);
    out.print("reject_nonfinite="); out.println(s.rejectNonfinite);
}

void magStatusPrintCalibration(Stream& out, const MagStatusReporterDeps& deps) {
    MagCalibrationCollector& collector = *deps.calibrationCollector;

    MagCalibrationResult result;
    const bool canCompute = collector.compute(result);

    out.println("# MAG CAL");
    out.print("mag_cal_active="); out.println(collector.active() ? "yes" : "no");
    const MagCalibrationFitSetDiagnostics fitSet = collector.fitSetDiagnostics();
    out.print("mag_cal_samples="); out.println(collector.samples());
    out.print("mag_cal_stored_fit_samples="); out.println(fitSet.samples);
    out.print("mag_cal_reservoir_replacements="); out.println(collector.reservoirReplacements());
    out.print("mag_cal_reservoir_skipped="); out.println(collector.reservoirSkipped());
    out.print("mag_cal_rejected="); out.println(collector.rejected());
    out.print("mag_cal_saturated="); out.println(collector.saturated());
    out.print("mag_cal_elapsed_s=");
    out.println(collector.startMs() == 0 ? 0UL : (millis() - collector.startMs()) / 1000UL);
    out.print("mag_cal_last_age_ms=");
    out.println(collector.samples() > 0 ? millis() - collector.lastSampleMs() : 0UL);

    out.print("min_xyz=");
    out.print(collector.minX(), 3); out.print(',');
    out.print(collector.minY(), 3); out.print(',');
    out.println(collector.minZ(), 3);

    out.print("max_xyz=");
    out.print(collector.maxX(), 3); out.print(',');
    out.print(collector.maxY(), 3); out.print(',');
    out.println(collector.maxZ(), 3);

    out.print("capture_span_xyz=");
    out.print(collector.spanX(), 3); out.print(',');
    out.print(collector.spanY(), 3); out.print(',');
    out.println(collector.spanZ(), 3);

    out.print("fit_span_xyz=");
    out.print(fitSet.max.x - fitSet.min.x, 3); out.print(',');
    out.print(fitSet.max.y - fitSet.min.y, 3); out.print(',');
    out.println(fitSet.max.z - fitSet.min.z, 3);

    out.print("mean_xyz=");
    out.print(collector.meanX(), 3); out.print(',');
    out.print(collector.meanY(), 3); out.print(',');
    out.println(collector.meanZ(), 3);

    out.print("capture_norm_min_mean_max=");
    out.print(collector.normMin(), 3); out.print(',');
    out.print(collector.normMean(), 3); out.print(',');
    out.println(collector.normMax(), 3);

    out.print("fit_norm_min_mean_max=");
    out.print(fitSet.normMin, 3); out.print(',');
    out.print(fitSet.normMean, 3); out.print(',');
    out.println(fitSet.normMax, 3);

    out.print("can_compute="); out.println(canCompute ? "yes" : "no");
    if (!canCompute) {
        out.print("compute_failure_reason=");
        out.println(collector.lastFailureReasonName());
        magStatusPrintCalibrationFitQuality(out, collector);
    }

    if (canCompute) {
        out.print("computed_hard_iron=");
        out.print(result.hardIron.x, 6); out.print(',');
        out.print(result.hardIron.y, 6); out.print(',');
        out.println(result.hardIron.z, 6);

        out.print("computed_soft_iron=");
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                if (r != 0 || c != 0) out.print(',');
                out.print(result.softIron.m[r][c], 6);
            }
        }
        out.println();

        out.print("computed_expected_norm="); out.println(result.expectedNorm, 6);
        out.print("computed_residual_rms="); out.println(result.residualRms, 6);
        out.print("computed_geometric_residual_rms="); out.println(result.geometricResidualRms, 6);
        out.print("computed_normalized_residual_rms="); out.println(result.normalizedResidualRms, 6);
        out.print("computed_coverage_score="); out.println(result.coverageScore, 6);
        out.print("computed_directional_coverage_score="); out.println(result.directionalCoverageScore, 6);
        out.print("computed_axis_ratio="); out.println(result.axisRatio, 6);
        out.print("computed_inliers="); out.print(result.inlierSamples); out.print('/'); out.println(fitSet.samples);
        out.print("computed_inlier_ratio="); out.println(result.inlierRatio, 6);

        out.print("computed_radius_xyz=");
        out.print(result.radiusX, 3); out.print(',');
        out.print(result.radiusY, 3); out.print(',');
        out.println(result.radiusZ, 3);

        out.print("suggested_trust_norm_min_max=");
        out.print(result.minTrustNorm, 6); out.print(',');
        out.println(result.maxTrustNorm, 6);
    }

    out.println("# Rotate the whole final assembly through all orientations.");
    out.println("# Use: mag cal apply save");
}

} // namespace tracker
