#include "runtime/machine_log_runtime.hpp"

namespace tracker {

void machineLogPrintU64Dec(Stream& out, uint64_t v) {
    char buf[21];
    size_t i = sizeof(buf);
    buf[--i] = '\0';
    if (v == 0) {
        buf[--i] = '0';
    } else {
        while (v > 0 && i > 0) {
            buf[--i] = static_cast<char>('0' + (v % 10));
            v /= 10;
        }
    }
    out.print(&buf[i]);
}

const char* machineLogModeName(TrackerLogMode mode) {
    switch (mode) {
        case TrackerLogMode::Off:   return "off";
        case TrackerLogMode::Basic: return "basic";
        case TrackerLogMode::Full:  return "full";
    }
    return "unknown";
}

bool machineLogDue(TrackerSerialLogState& state, uint32_t nowUs) {
    if (!state.enabled()) return false;
    const uint32_t period = state.periodUs();
    if (state.lastEmitUs == 0 || static_cast<uint32_t>(nowUs - state.lastEmitUs) >= period) {
        state.lastEmitUs = nowUs;
        return true;
    }
    return false;
}

bool machineLogMagDue(TrackerSerialLogState& state, uint32_t nowUs) {
    if (!state.enabled()) return false;
    const uint32_t period = state.periodUs();
    if (state.lastMagEmitUs == 0 || static_cast<uint32_t>(nowUs - state.lastMagEmitUs) >= period) {
        state.lastMagEmitUs = nowUs;
        return true;
    }
    return false;
}

bool machineLogBiasDue(TrackerSerialLogState& state,
                              uint32_t& lastBiasEmitUs,
                              uint32_t biasPeriodUs,
                              uint32_t nowUs) {
    if (!state.enabled()) return false;
    if (lastBiasEmitUs == 0 || static_cast<uint32_t>(nowUs - lastBiasEmitUs) >= biasPeriodUs) {
        lastBiasEmitUs = nowUs;
        return true;
    }
    return false;
}

void machineLogResetCounters(MachineLogCounters& counters, uint32_t& lastBiasEmitUs) {
    counters = MachineLogCounters{};
    lastBiasEmitUs = 0;
}

void machineLogEmitHeader(Stream& out,
                                 const TrackerSerialLogState& state,
                                 const TrackerConfig& config) {
    out.print("LOGVER,2,E0,mode,"); out.print(machineLogModeName(state.mode));
    out.print(",rate_hz,"); out.print(state.rateHz);
    out.print(",config_crc,0x"); out.print(config.data.crc32, HEX);
    out.print(",config_version,"); out.println(config.data.version);
    out.println("LOGFMT,Q,t_us,seq,dt_us,w,x,y,z,qflags,conf,state,acc_trust,acc_norm_g,acc_var_g2,gyro_trust,gyro_dps,recovery");
    out.println("LOGFMT,FIFO,t_us,seq,dt_us,hw_ts,fb_ts,dropped_before,overrun,full,unknown,quality_flags");
    out.println("LOGFMT,CAL,t_us,seq,ax_g,ay_g,az_g,gx_rads,gy_rads,gz_rads,temp_c,quality_flags");
    out.println("LOGFMT,BIAS,t_us,seq,temp_c,bx_dps,by_dps,bz_dps,source,quality,flags,rt_enabled,rt_updates");
    out.println("LOGFMT,BIASUPD,t_us,seq,temp_c,rx_dps,ry_dps,rz_dps,sx_dps,sy_dps,sz_dps,dx_dps,dy_dps,dz_dps,trim_x_dps,trim_y_dps,trim_z_dps,flags");
    out.println("LOGFMT,MAG,t_us,seq,mag_seq,age_ms,raw_norm,body_norm,horiz_norm,heading_valid,heading_yaw_deg,heading_innov_deg,trusted,reject_flags");
    out.println("LOGFMT,MAGR,t_us,seq,mag_seq,raw_x,raw_y,raw_z,cal_x,cal_y,cal_z,body_x,body_y,body_z,raw_norm,cal_norm,body_norm,raw_flags,reject_flags,trusted");
    out.println("LOGFMT,YAW,t_us,seq,valid,gate_open,apply_allowed,applied,error_deg,step_deg,trust,reject_flags,cooldown_ms");
    out.println("LOGFMT,STATE,t_us,seq,state,reason,flags,conf");
    out.println("LOGFMT,LOGSUM,uptime_ms,mode,rate_hz,q,cal,fifo,mag,yaw,state,bias,samples,quality_samples,fifo_overruns,fifo_full,large_gaps,recoveries,mag_trusted,mag_rejected,yaw_applied");
}

void machineLogEmitStateEvent(Stream& out,
                                     TrackerSerialLogState& state,
                                     MachineLogCounters& counters,
                                     const char* eventState,
                                     const char* reason,
                                     uint64_t tUs,
                                     uint32_t flags,
                                     float confidence) {
    if (!state.enabled()) return;
    const uint32_t seq = state.sequence++;
    out.print("STATE,"); machineLogPrintU64Dec(out, tUs);
    out.print(','); out.print(seq);
    out.print(','); out.print(eventState ? eventState : "UNKNOWN");
    out.print(','); out.print(reason ? reason : "none");
    out.print(",0x"); out.print(flags, HEX);
    out.print(','); out.println(confidence, 4);
    counters.state++;
}

void machineLogPrintSummary(Stream& out,
                                   const TrackerSerialLogState& state,
                                   const MachineLogCounters& counters,
                                   uint32_t runtimeSamples,
                                   uint32_t trackingRecoveryEnterCount,
                                   const ImuQualityMonitor& quality,
                                   const Lsm6dsvFifoReader& fifo,
                                   const MagRuntimeProcessor& magProcessor,
                                   const MagYawCorrectionController& magYawCorrection,
                                   const Ahrs6Dof& ahrs,
                                   const MagProcessedSample& lastMagProcessed,
                                   const MagYawCorrectionOutput& lastMagYawCorrection,
                                   const RuntimeGyroBiasEstimator& runtimeBias) {
    const auto& qc = quality.counters();
    const auto& fs = fifo.stats();
    const auto& ms = magProcessor.stats();
    const auto& ys = magYawCorrection.stats();

    out.print("LOGSUM,"); out.print(millis());
    out.print(','); out.print(machineLogModeName(state.mode));
    out.print(','); out.print(state.rateHz);
    out.print(','); out.print(counters.q);
    out.print(','); out.print(counters.cal);
    out.print(','); out.print(counters.fifo);
    out.print(','); out.print(counters.mag);
    out.print(','); out.print(counters.yaw);
    out.print(','); out.print(counters.state);
    out.print(','); out.print(counters.bias);
    out.print(','); out.print(runtimeSamples);
    out.print(','); out.print(qc.samples);
    out.print(','); out.print(fs.overrunEvents);
    out.print(','); out.print(fs.fullEvents);
    out.print(','); out.print(qc.largeGapSamples);
    out.print(','); out.print(trackingRecoveryEnterCount);
    out.print(','); out.print(ms.trustedSamples);
    out.print(','); out.print(ms.rejectedSamples);
    out.print(','); out.println(ys.appliedCount);

    out.print("LOGSTAT,AHRS,");
    const Ahrs6DofStats& ast = ahrs.stats();
    out.print(ast.updateCount); out.print(',');
    out.print(ast.gyroPredictCount); out.print(',');
    out.print(ast.accelUpdateCount); out.print(',');
    out.print(ast.accelRejectedCount); out.print(',');
    out.print(ast.skippedBadDt); out.print(',');
    out.println(ast.clampedLargeDt);

    out.print("LOGSTAT,QUALITY,");
    out.print(qc.samples); out.print(',');
    out.print(qc.hwTimestampSamples); out.print(',');
    out.print(qc.fallbackTimestampSamples); out.print(',');
    out.print(qc.largeGapSamples); out.print(',');
    out.print(qc.estimatedDroppedSamples); out.print(',');
    out.println(qc.fifoRecoveryRequests);

    out.print("LOGSTAT,MAG,");
    out.print(ms.processedSamples); out.print(',');
    out.print(ms.trustedSamples); out.print(',');
    out.print(ms.rejectedSamples); out.print(',');
    out.print(lastMagProcessed.rejectFlags, HEX); out.print(',');
    out.println(lastMagYawCorrection.rejectFlags, HEX);

    out.print("LOGSTAT,BIAS,");
    out.print(runtimeBias.enabled ? 1 : 0); out.print(',');
    out.print(runtimeBias.windows); out.print(',');
    out.print(runtimeBias.accepted); out.print(',');
    out.print(runtimeBias.rejected); out.print(',');
    out.print(runtimeBias.updates); out.print(',');
    out.print(runtimeBias.lastResidualDps.norm(), 8); out.print(',');
    out.print(runtimeBias.lastAppliedDeltaDps.norm(), 8); out.print(',');
    out.print(runtimeBias.runtimeTrimRadS.norm() * MATH_RAD_TO_DEG, 8); out.print(',');
    out.println(counters.biasUpdate);
}

const char* machineLogBiasSource(const RuntimeGyroBiasEstimator& runtimeBias,
                                        const ImuCalibration& imuCal,
                                        const GyroTempCompensator& gyroTempComp) {
    if (runtimeBias.runtimeTrimRadS.norm() > (0.00001f * MATH_DEG_TO_RAD)) {
        return gyroTempComp.valid() ? "temp+rt" : (imuCal.gyroBiasValid ? "bias+rt" : "rt");
    }
    return gyroTempComp.valid() ? "temp" : (imuCal.gyroBiasValid ? "bias" : "none");
}

void machineLogEmitFrame(Stream& out,
                                TrackerSerialLogState& state,
                                MachineLogCounters& counters,
                                uint32_t& lastBiasEmitUs,
                                uint32_t biasPeriodUs,
                                const Lsm6dsv::RawSample& raw,
                                const Lsm6dsv::Sample& calibrated,
                                const ImuQualityResult& quality,
                                const Ahrs6Dof& ahrs,
                                const char* trackingState,
                                bool trackingRecovering,
                                const GyroTempCompensator& gyroTempComp,
                                const ImuCalibration& imuCal,
                                const RuntimeGyroBiasEstimator& runtimeBias,
                                const Vec3& currentBiasDps,
                                uint32_t gyroBiasFlags) {
#if !TRACKER_ENABLE_MACHINE_LOG
    (void)out;
    (void)state;
    (void)counters;
    (void)lastBiasEmitUs;
    (void)biasPeriodUs;
    (void)raw;
    (void)calibrated;
    (void)quality;
    (void)ahrs;
    (void)trackingState;
    (void)trackingRecovering;
    (void)gyroTempComp;
    (void)imuCal;
    (void)runtimeBias;
    (void)currentBiasDps;
    (void)gyroBiasFlags;
    return;
#else
    // Hot-path fast return: avoid micros() and log-rate bookkeeping when the
    // machine log is disabled, which is the normal tracking/WiFi path.
    if (!state.enabled()) return;
    if (!machineLogDue(state, micros())) return;

    const uint32_t seq = state.sequence++;
    const Ahrs6DofStats& ast = ahrs.stats();
    const Quat q = ahrs.quaternionPositiveW();
    const float gyroDps = calibrated.gyro_rad_s.norm() * MATH_RAD_TO_DEG;
    const bool hwTs = (quality.flags & imu_quality_flags::TIMESTAMP_HARDWARE) != 0;
    const bool fbTs = (quality.flags & imu_quality_flags::TIMESTAMP_FALLBACK) != 0;

    out.print("Q,"); machineLogPrintU64Dec(out, raw.t_us);
    out.print(','); out.print(seq);
    out.print(','); out.print(quality.dtUs);
    out.print(','); out.print(q.w, 7);
    out.print(','); out.print(q.x, 7);
    out.print(','); out.print(q.y, 7);
    out.print(','); out.print(q.z, 7);
    out.print(",0x"); out.print(quality.flags, HEX);
    out.print(','); out.print(quality.overallConfidence, 4);
    out.print(','); out.print(trackingState ? trackingState : "UNKNOWN");
    out.print(','); out.print(ast.lastAccelGate.trust, 4);
    out.print(','); out.print(ast.lastAccelGate.normG, 5);
    out.print(','); out.print(ast.accelNormVarianceG2, 8);
    out.print(','); out.print(ast.lastGyroMotionTrust, 4);
    out.print(','); out.print(gyroDps, 4);
    out.print(','); out.println(trackingRecovering ? 1 : 0);
    counters.q++;

    out.print("FIFO,"); machineLogPrintU64Dec(out, raw.t_us);
    out.print(','); out.print(seq);
    out.print(','); out.print(quality.dtUs);
    out.print(','); out.print(hwTs ? 1 : 0);
    out.print(','); out.print(fbTs ? 1 : 0);
    out.print(','); out.print(quality.estimatedDroppedBefore);
    out.print(','); out.print(quality.has(imu_quality_flags::FIFO_OVERRUN) ? 1 : 0);
    out.print(','); out.print(quality.has(imu_quality_flags::FIFO_FULL) ? 1 : 0);
    out.print(','); out.print(quality.has(imu_quality_flags::FIFO_UNKNOWN_TAG) ? 1 : 0);
    out.print(",0x"); out.println(quality.flags, HEX);
    counters.fifo++;

    if (machineLogBiasDue(state, lastBiasEmitUs, biasPeriodUs, static_cast<uint32_t>(raw.t_us))) {
        const GyroTempCompSnapshot tempSnap = gyroTempComp.snapshot(calibrated.temp_c);
        out.print("BIAS,"); machineLogPrintU64Dec(out, raw.t_us);
        out.print(','); out.print(seq);
        out.print(','); out.print(calibrated.temp_c, 3);
        out.print(','); out.print(currentBiasDps.x, 8);
        out.print(','); out.print(currentBiasDps.y, 8);
        out.print(','); out.print(currentBiasDps.z, 8);
        out.print(','); out.print(machineLogBiasSource(runtimeBias, imuCal, gyroTempComp));
        out.print(','); out.print(tempSnap.fitQuality, 4);
        out.print(",0x"); out.print(gyroBiasFlags, HEX);
        out.print(','); out.print(runtimeBias.enabled ? 1 : 0);
        out.print(','); out.println(runtimeBias.updates);
        counters.bias++;
    }

    if (state.mode == TrackerLogMode::Full) {
        out.print("CAL,"); machineLogPrintU64Dec(out, raw.t_us);
        out.print(','); out.print(seq);
        out.print(','); out.print(calibrated.accel_g.x, 6);
        out.print(','); out.print(calibrated.accel_g.y, 6);
        out.print(','); out.print(calibrated.accel_g.z, 6);
        out.print(','); out.print(calibrated.gyro_rad_s.x, 8);
        out.print(','); out.print(calibrated.gyro_rad_s.y, 8);
        out.print(','); out.print(calibrated.gyro_rad_s.z, 8);
        out.print(','); out.print(calibrated.temp_c, 3);
        out.print(",0x"); out.println(quality.flags, HEX);
        counters.cal++;
    }
#endif
}

void machineLogEmitMagFrame(Stream& out,
                                   TrackerSerialLogState& state,
                                   MachineLogCounters& counters,
                                   const MagProcessedSample& mag,
                                   const MagHeadingSample& heading,
                                   const MagYawCorrectionOutput& yaw,
                                   uint32_t rejectFlagsForUse,
                                   bool trustedForUse) {
#if !TRACKER_ENABLE_MACHINE_LOG
    (void)out;
    (void)state;
    (void)counters;
    (void)mag;
    (void)heading;
    (void)yaw;
    (void)rejectFlagsForUse;
    (void)trustedForUse;
    return;
#else
    if (!state.enabled()) return;
    if (!machineLogMagDue(state, micros())) return;

    const uint32_t seq = state.sequence++;
    const uint32_t nowMs = millis();
    const uint32_t ageMs = mag.receivedMs == 0 ? 0UL : nowMs - mag.receivedMs;

    out.print("MAG,"); machineLogPrintU64Dec(out, mag.t_us);
    out.print(','); out.print(seq);
    out.print(','); out.print(mag.seq);
    out.print(','); out.print(ageMs);
    out.print(','); out.print(mag.rawNorm, 5);
    out.print(','); out.print(mag.bodyNorm, 5);
    out.print(','); out.print(heading.horizontalNorm, 5);
    out.print(','); out.print(heading.valid ? 1 : 0);
    out.print(','); out.print(heading.magneticNorthWorldYawDeg, 4);
    out.print(','); out.print(heading.yawInnovationDeg, 4);
    out.print(','); out.print(trustedForUse ? 1 : 0);
    out.print(",0x"); out.println(rejectFlagsForUse, HEX);
    counters.mag++;

    if (state.mode == TrackerLogMode::Full) {
        out.print("MAGR,"); machineLogPrintU64Dec(out, mag.t_us);
        out.print(','); out.print(seq);
        out.print(','); out.print(mag.seq);
        out.print(','); out.print(mag.raw.x, 5);
        out.print(','); out.print(mag.raw.y, 5);
        out.print(','); out.print(mag.raw.z, 5);
        out.print(','); out.print(mag.calibratedMagFrame.x, 5);
        out.print(','); out.print(mag.calibratedMagFrame.y, 5);
        out.print(','); out.print(mag.calibratedMagFrame.z, 5);
        out.print(','); out.print(mag.body.x, 5);
        out.print(','); out.print(mag.body.y, 5);
        out.print(','); out.print(mag.body.z, 5);
        out.print(','); out.print(mag.rawNorm, 5);
        out.print(','); out.print(mag.calibratedNorm, 5);
        out.print(','); out.print(mag.bodyNorm, 5);
        out.print(",0x"); out.print(mag.rawFlags, HEX);
        out.print(",0x"); out.print(rejectFlagsForUse, HEX);
        out.print(','); out.println(trustedForUse ? 1 : 0);
    }

    out.print("YAW,"); machineLogPrintU64Dec(out, yaw.magTimestampUs != 0 ? yaw.magTimestampUs : mag.t_us);
    out.print(','); out.print(seq);
    out.print(','); out.print(yaw.valid ? 1 : 0);
    out.print(','); out.print(yaw.gateOpen ? 1 : 0);
    out.print(','); out.print(yaw.applyAllowed ? 1 : 0);
    out.print(','); out.print(yaw.applied ? 1 : 0);
    out.print(','); out.print(yaw.errorDeg, 5);
    out.print(','); out.print(yaw.correctionStepDeg, 7);
    out.print(','); out.print(yaw.combinedTrust, 4);
    out.print(",0x"); out.print(yaw.rejectFlags, HEX);
    out.print(','); out.println(yaw.cooldownRemainingMs);
    counters.yaw++;
#endif
}

} // namespace tracker
