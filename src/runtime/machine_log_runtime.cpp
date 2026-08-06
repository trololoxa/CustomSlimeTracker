#include "runtime/machine_log_runtime.hpp"

#include "build_config/build_identity.hpp"
#include "runtime/runtime_gyro_bias_controller.hpp"

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

namespace {

constexpr size_t MACHINE_LOG_LINE_RESERVE_BYTES = 384u;

bool machineLogOutputHasRoom(Stream& out, size_t reserveBytes) {
    const int writable = out.availableForWrite();
    return writable > 0 && static_cast<size_t>(writable) >= reserveBytes;
}

} // namespace

static bool machineLogDue(TrackerSerialLogState& state, uint32_t nowUs) {
    if (!state.accepting()) return false;
    const uint32_t period = state.periodUs();
    if (state.lastEmitUs == 0 || static_cast<uint32_t>(nowUs - state.lastEmitUs) >= period) {
        state.lastEmitUs = nowUs;
        return true;
    }
    return false;
}

static bool machineLogMagDue(TrackerSerialLogState& state, uint32_t nowUs) {
    if (!state.accepting()) return false;
    const uint32_t period = state.periodUs();
    if (state.lastMagEmitUs == 0 || static_cast<uint32_t>(nowUs - state.lastMagEmitUs) >= period) {
        state.lastMagEmitUs = nowUs;
        return true;
    }
    return false;
}

static bool machineLogNetworkDue(TrackerSerialLogState& state, uint32_t nowUs) {
    if (!state.accepting()) return false;
    if (state.lastNetworkEmitUs == 0u ||
        static_cast<uint32_t>(nowUs - state.lastNetworkEmitUs) >=
            TRACKER_MACHINE_LOG_NETWORK_PERIOD_US) {
        state.lastNetworkEmitUs = nowUs;
        return true;
    }
    return false;
}

static bool machineLogBiasDue(TrackerSerialLogState& state,
                              uint32_t& lastBiasEmitUs,
                              uint32_t biasPeriodUs,
                              uint32_t nowUs) {
    if (!state.accepting()) return false;
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
    out.print("LOGVER,3,E1,mode,"); out.print(machineLogModeName(state.mode));
    out.print(",rate_hz,"); out.print(state.rateHz);
    out.print(",config_crc,0x"); out.print(config.data.crc32, HEX);
    out.print(",config_version,"); out.print(config.data.version);
    out.print(",build_profile,"); out.print(trackerBuildProfileName());
    out.print(",pio_env,"); out.print(trackerBuildPioEnvironment());
    out.print(",git,"); out.println(trackerBuildIdentityString());
    out.println("LOGFMT,Q,t_us,seq,dt_us,w,x,y,z,qflags,conf,state,acc_trust,acc_norm_g,acc_var_g2,gyro_trust,gyro_dps,recovery");
    out.println("LOGFMT,FIFO,t_us,seq,dt_us,hw_ts,fb_ts,dropped_before,overrun,full,unknown,quality_flags");
    out.println("LOGFMT,CAL,t_us,seq,ax_g,ay_g,az_g,gx_rads,gy_rads,gz_rads,temp_c,quality_flags");
    out.println("LOGFMT,BIAS,t_us,seq,temp_c,bx_dps,by_dps,bz_dps,source,quality,flags,rt_enabled,rt_updates");
    out.println("LOGFMT,BIASUPD,t_us,seq,temp_c,rx_dps,ry_dps,rz_dps,sx_dps,sy_dps,sz_dps,dx_dps,dy_dps,dz_dps,trim_x_dps,trim_y_dps,trim_z_dps,flags");
    out.println("LOGFMT,MAG,t_us,seq,mag_seq,age_ms,raw_norm,body_norm,horiz_norm,heading_valid,heading_yaw_deg,heading_innov_deg,trusted,reject_flags,dip_deg,field_state,field_trusted,field_flags,field_norm_error,field_dip_error_deg,field_heading_error_deg");
    out.println("LOGFMT,MAGR,t_us,seq,mag_seq,raw_x,raw_y,raw_z,cal_x,cal_y,cal_z,body_x,body_y,body_z,raw_norm,cal_norm,body_norm,raw_flags,reject_flags,trusted");
    out.println("LOGFMT,YAW,t_us,seq,valid,gate_open,apply_allowed,applied,error_deg,step_deg,trust,reject_flags,cooldown_ms,mode,reacquire_pending,reacquire_active,field_stable_ms,heading_rate_deg_s");
    out.println("LOGFMT,STATE,t_us,seq,state,reason,flags,conf");
    out.println("LOGFMT,NET,t_us,seq,wifi_connected,wifi_disc,wifi_timeouts,rssi_dbm,slime_state,udp_ready,server_found,rotation_sent,rotation_due,rot_missed,rot_late,send_failures,rot_fail,control_fail,telemetry_fail,discovery_fail,tx_pressure_fail,tx_other_fail,rebind_ok,rebind_fail,full_reopens,consecutive_fail,last_udp_error,motion_tx_age_ms,tx_pressure_state,tx_recovery_reason");
    out.println("LOGFMT,TESTSUM,kind,duration_ms,stopped,samples,hw_ts,fb_ts,bad_ts,dropped,recoveries,fifo_overruns,fifo_full,fifo_unknown,net_valid,wifi_disc,wifi_timeouts,udp_fail,rot_fail,rot_missed,rot_late,tx_pressure_fail,tx_other_fail,rebind_ok,rebind_fail,full_reopens,static_valid,gyro_mean_dps,gyro_std_dps,accel_mean_g,accel_std_g,temp_start_c,temp_end_c,mag_valid,mag_trusted,mag_rejected");
    out.println("LOGFMT,LOGSUM,uptime_ms,mode,rate_hz,q,cal,fifo,mag,yaw,state,bias,net,samples,quality_samples,fifo_overruns,fifo_full,large_gaps,recoveries,mag_trusted,mag_rejected,yaw_applied");
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
    out.print(','); out.print(counters.network);
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

    out.print("LOGSTAT,BACKPRESSURE,");
    out.println(counters.backpressureDrop);
    out.print("LOGSTAT,SESSION,");
    out.println(counters.disconnectAbort);
}

void MachineLogDeferredRuntime::begin(TrackerSerialLogState* state,
                                      MachineLogCounters* counters,
                                      uint32_t* lastBiasEmitUs,
                                      uint32_t biasPeriodUs) {
    state_ = state;
    counters_ = counters;
    lastBiasEmitUs_ = lastBiasEmitUs;
    biasPeriodUs_ = biasPeriodUs;
    reset(false);
}

bool MachineLogDeferredRuntime::ready() const {
    return state_ != nullptr && counters_ != nullptr && lastBiasEmitUs_ != nullptr;
}

void MachineLogDeferredRuntime::reset(bool countPendingAsDropped) {
    const uint8_t pending = count_;
    head_ = 0u;
    tail_ = 0u;
    count_ = 0u;
    headLine_ = 0u;
    if (countPendingAsDropped && pending != 0u && counters_ != nullptr) {
        counters_->shutdownDrop += pending;
        counters_->backpressureDrop += pending;
        status_.queued = 0u;
    } else {
        status_ = MachineLogDeferredStatus{};
    }
}

void MachineLogDeferredRuntime::copyText(char* dst, size_t capacity, const char* src) {
    if (!dst || capacity == 0u) return;
    size_t i = 0u;
    if (src != nullptr) {
        while (i + 1u < capacity && src[i] != '\0') {
            const char value = src[i];
            // LOGVER3 is deliberately unquoted CSV. Internal state/reason
            // strings must never be able to create an extra field or record.
            dst[i] = (value == ',' || value == '"' || value == '\r' || value == '\n')
                ? '_'
                : value;
            ++i;
        }
    }
    dst[i] = '\0';
}

MachineLogDeferredRecord* MachineLogDeferredRuntime::reserve(
    MachineLogDeferredRecordType type,
    TrackerLogMode mode,
    uint32_t sequence) {
    if (!ready()) return nullptr;
    if (count_ >= TRACKER_MACHINE_LOG_QUEUE_RECORDS) {
        ++counters_->producerQueueDrop;
        ++counters_->backpressureDrop;
        return nullptr;
    }
    MachineLogDeferredRecord& record = records_[tail_];
    switch (type) {
        case MachineLogDeferredRecordType::Imu:
            record.payload.imu = MachineLogDeferredImuRecord{};
            break;
        case MachineLogDeferredRecordType::Mag:
            record.payload.mag = MachineLogDeferredMagRecord{};
            break;
        case MachineLogDeferredRecordType::State:
            record.payload.state = MachineLogDeferredStateRecord{};
            break;
        case MachineLogDeferredRecordType::BiasUpdate:
            record.payload.biasUpdate = MachineLogDeferredBiasUpdateRecord{};
            break;
        case MachineLogDeferredRecordType::Network:
            record.payload.network = MachineLogDeferredNetworkRecord{};
            break;
    }
    record.type = type;
    record.mode = mode;
    record.sequence = sequence;
    record.queuedAtUs = micros();
    tail_ = static_cast<uint8_t>((tail_ + 1u) % TRACKER_MACHINE_LOG_QUEUE_RECORDS);
    ++count_;
    ++status_.enqueued;
    status_.queued = count_;
    if (count_ > status_.highWater) status_.highWater = count_;
    return &record;
}

bool MachineLogDeferredRuntime::enqueueImu(
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
    if (!ready() || !state_->accepting()) return false;
    if (!machineLogDue(*state_, static_cast<uint32_t>(raw.t_us))) return false;

    MachineLogDeferredRecord* record = reserve(
        MachineLogDeferredRecordType::Imu, state_->mode, state_->sequence);
    if (record == nullptr) return false;
    ++state_->sequence;

    MachineLogDeferredImuRecord& dst = record->payload.imu;
    const Ahrs6DofStats& ast = ahrs.stats();
    const Quat quaternion = ahrs.quaternionPositiveW();
    dst.timestampUs = raw.t_us;
    dst.dtUs = quality.dtUs;
    dst.quaternionW = quaternion.w;
    dst.quaternionX = quaternion.x;
    dst.quaternionY = quaternion.y;
    dst.quaternionZ = quaternion.z;
    dst.qualityFlags = quality.flags;
    dst.estimatedDroppedBefore = quality.estimatedDroppedBefore;
    dst.confidence = quality.overallConfidence;
    copyText(dst.trackingState, sizeof(dst.trackingState),
             trackingState ? trackingState : "UNKNOWN");
    dst.accelTrust = ast.lastAccelGate.trust;
    dst.accelNormG = ast.lastAccelGate.normG;
    dst.accelVarianceG2 = ast.accelNormVarianceG2;
    dst.gyroTrust = ast.lastGyroMotionTrust;
    dst.gyroDps = calibrated.gyro_rad_s.norm() * MATH_RAD_TO_DEG;
    dst.trackingRecovering = trackingRecovering;
    dst.hardwareTimestamp = quality.has(imu_quality_flags::TIMESTAMP_HARDWARE);
    dst.fallbackTimestamp = quality.has(imu_quality_flags::TIMESTAMP_FALLBACK);
    dst.fifoOverrun = quality.has(imu_quality_flags::FIFO_OVERRUN);
    dst.fifoFull = quality.has(imu_quality_flags::FIFO_FULL);
    dst.fifoUnknown = quality.has(imu_quality_flags::FIFO_UNKNOWN_TAG);
    dst.calibratedAccelX = calibrated.accel_g.x;
    dst.calibratedAccelY = calibrated.accel_g.y;
    dst.calibratedAccelZ = calibrated.accel_g.z;
    dst.calibratedGyroX = calibrated.gyro_rad_s.x;
    dst.calibratedGyroY = calibrated.gyro_rad_s.y;
    dst.calibratedGyroZ = calibrated.gyro_rad_s.z;
    dst.temperatureC = calibrated.temp_c;

    dst.emitBias = machineLogBiasDue(*state_, *lastBiasEmitUs_, biasPeriodUs_,
                                     static_cast<uint32_t>(raw.t_us));
    if (dst.emitBias) {
        const GyroTempCompSnapshot tempSnap = gyroTempComp.snapshot(calibrated.temp_c);
        dst.currentBiasX = currentBiasDps.x;
        dst.currentBiasY = currentBiasDps.y;
        dst.currentBiasZ = currentBiasDps.z;
        copyText(dst.biasSource, sizeof(dst.biasSource),
                 runtimeBiasSourceName(runtimeBias, imuCal, gyroTempComp));
        dst.biasFitQuality = tempSnap.fitQuality;
        dst.biasFlags = gyroBiasFlags;
        dst.runtimeBiasEnabled = runtimeBias.enabled;
        dst.runtimeBiasUpdates = runtimeBias.updates;
    }
    return true;
}

bool MachineLogDeferredRuntime::enqueueMag(
    const MagProcessedSample& mag,
    const MagHeadingSample& heading,
    const MagFieldReliabilityOutput& reliability,
    const MagYawCorrectionOutput& yaw,
    uint32_t rejectFlagsForUse,
    bool trustedForUse) {
    if (!ready() || !state_->accepting()) return false;
    if (!machineLogMagDue(*state_, static_cast<uint32_t>(mag.t_us))) return false;

    MachineLogDeferredRecord* record = reserve(
        MachineLogDeferredRecordType::Mag, state_->mode, state_->sequence);
    if (record == nullptr) return false;
    ++state_->sequence;
    MachineLogDeferredMagRecord& dst = record->payload.mag;
    dst.timestampUs = mag.t_us;
    dst.yawTimestampUs = yaw.magTimestampUs != 0u ? yaw.magTimestampUs : mag.t_us;
    dst.magSequence = mag.seq;
    dst.ageMs = mag.valid ? reliability.nowMs - mag.receivedMs : 0u;
    dst.rejectFlagsForUse = rejectFlagsForUse;
    dst.reliabilityFlags = reliability.flags;
    dst.yawRejectFlags = yaw.rejectFlags;
    dst.yawCooldownRemainingMs = yaw.cooldownRemainingMs;
    dst.yawFieldStableMs = yaw.fieldStableMs;
    dst.rawFlags = mag.rawFlags;
    dst.reliabilityState = static_cast<uint8_t>(reliability.state);
    dst.yawMode = static_cast<uint8_t>(yaw.mode);
    dst.headingValid = heading.valid;
    dst.trustedForUse = trustedForUse;
    dst.reliabilityTrustedForYaw = reliability.trustedForYaw;
    dst.yawValid = yaw.valid;
    dst.yawGateOpen = yaw.gateOpen;
    dst.yawApplyAllowed = yaw.applyAllowed;
    dst.yawApplied = yaw.applied;
    dst.yawReacquirePending = yaw.reacquirePending;
    dst.yawReacquireActive = yaw.reacquireActive;
    dst.rawX = mag.raw.x;
    dst.rawY = mag.raw.y;
    dst.rawZ = mag.raw.z;
    dst.calibratedX = mag.calibratedMagFrame.x;
    dst.calibratedY = mag.calibratedMagFrame.y;
    dst.calibratedZ = mag.calibratedMagFrame.z;
    dst.bodyX = mag.body.x;
    dst.bodyY = mag.body.y;
    dst.bodyZ = mag.body.z;
    dst.rawNorm = mag.rawNorm;
    dst.calibratedNorm = mag.calibratedNorm;
    dst.bodyNorm = mag.bodyNorm;
    dst.horizontalNorm = heading.horizontalNorm;
    dst.headingYawDeg = heading.magneticNorthWorldYawDeg;
    dst.headingInnovationDeg = heading.yawInnovationDeg;
    dst.dipDeg = heading.dipDeg;
    dst.reliabilityNormError = reliability.normRelativeError;
    dst.reliabilityDipErrorDeg = reliability.dipErrorDeg;
    dst.reliabilityHeadingErrorDeg = reliability.referenceHeadingErrorDeg;
    dst.yawErrorDeg = yaw.errorDeg;
    dst.yawCorrectionStepDeg = yaw.correctionStepDeg;
    dst.yawCombinedTrust = yaw.combinedTrust;
    dst.yawMagneticHeadingRateDegS = yaw.magneticHeadingRateDegS;
    return true;
}

bool MachineLogDeferredRuntime::enqueueState(const char* eventState,
                                             const char* reason,
                                             uint64_t timestampUs,
                                             uint32_t flags,
                                             float confidence) {
    if (!ready() || !state_->accepting()) return false;
    MachineLogDeferredRecord* record = reserve(
        MachineLogDeferredRecordType::State, state_->mode, state_->sequence);
    if (record == nullptr) return false;
    ++state_->sequence;
    MachineLogDeferredStateRecord& dst = record->payload.state;
    dst.timestampUs = timestampUs;
    dst.flags = flags;
    dst.confidence = confidence;
    copyText(dst.state, sizeof(dst.state),
             eventState ? eventState : "UNKNOWN");
    copyText(dst.reason, sizeof(dst.reason),
             reason ? reason : "none");
    return true;
}

bool MachineLogDeferredRuntime::enqueueBiasUpdate(uint64_t timestampUs,
                                                  float temperatureC,
                                                  const Vec3& residualDps,
                                                  const Vec3& stdDps,
                                                  const Vec3& deltaDps,
                                                  const Vec3& trimDps,
                                                  uint32_t flags) {
    if (!ready() || !state_->accepting()) return false;
    MachineLogDeferredRecord* record = reserve(
        MachineLogDeferredRecordType::BiasUpdate, state_->mode, state_->sequence);
    if (record == nullptr) return false;
    ++state_->sequence;
    MachineLogDeferredBiasUpdateRecord& dst = record->payload.biasUpdate;
    dst.timestampUs = timestampUs;
    dst.temperatureC = temperatureC;
    dst.residualX = residualDps.x;
    dst.residualY = residualDps.y;
    dst.residualZ = residualDps.z;
    dst.stdX = stdDps.x;
    dst.stdY = stdDps.y;
    dst.stdZ = stdDps.z;
    dst.deltaX = deltaDps.x;
    dst.deltaY = deltaDps.y;
    dst.deltaZ = deltaDps.z;
    dst.trimX = trimDps.x;
    dst.trimY = trimDps.y;
    dst.trimZ = trimDps.z;
    dst.flags = flags;
    return true;
}

bool MachineLogDeferredRuntime::enqueueNetwork(
    uint64_t timestampUs,
    const TrackerWifiManager& wifi,
    const SlimeVROutputRuntime& slimevr) {
    if (!ready() || !state_->accepting() || timestampUs == 0u) return false;
    if (!machineLogNetworkDue(*state_, static_cast<uint32_t>(timestampUs))) {
        return false;
    }

    MachineLogDeferredRecord* record = reserve(
        MachineLogDeferredRecordType::Network, state_->mode, state_->sequence);
    if (record == nullptr) return false;
    ++state_->sequence;

    const TrackerWifiManagerStatus wifiStatus = wifi.status();
    const SlimeVROutputRuntimeStatus slimeStatus = slimevr.status();
    MachineLogDeferredNetworkRecord& dst = record->payload.network;
    dst.timestampUs = timestampUs;
    dst.wifiConnected = wifiStatus.connected;
    dst.wifiDisconnects = wifiStatus.disconnects;
    dst.wifiConnectTimeouts = wifiStatus.connectTimeouts;
    dst.wifiRssiDbm = wifiStatus.rssiDbm;
    dst.slimeState = static_cast<uint8_t>(slimeStatus.state);
    dst.udpReady = slimeStatus.udpReady;
    dst.serverFound = slimeStatus.serverFound;
    dst.rotationSent = slimeStatus.rotationSent;
    dst.rotationSendDue = slimeStatus.rotationSendDue;
    dst.rotationMissedDeadlines = slimeStatus.rotationMissedDeadlines;
    dst.rotationLateEvents = slimeStatus.rotationLateEvents;
    dst.sendFailures = slimeStatus.sendFailures;
    dst.rotationSendFailures = slimeStatus.rotationSendFailures;
    dst.controlSendFailures = slimeStatus.controlSendFailures;
    dst.telemetrySendFailures = slimeStatus.telemetrySendFailures;
    dst.discoverySendFailures = slimeStatus.discoverySendFailures;
    dst.txPressureFailures = slimeStatus.txPressureFailures;
    dst.txOtherFailures = slimeStatus.txOtherFailures;
    dst.udpRebindSuccesses = slimeStatus.udpTransportRebindSuccesses;
    dst.udpRebindFailures = slimeStatus.udpTransportRebindFailures;
    dst.udpFullReopenEscalations = slimeStatus.udpFullReopenEscalations;
    dst.consecutiveSendFailures = slimeStatus.consecutiveSendFailures;
    dst.lastUdpSendError = slimeStatus.lastUdpSendError;
    dst.lastSuccessfulMotionTxAgeMs = slimeStatus.lastSuccessfulMotionTxAgeMs;
    dst.txPressureState = static_cast<uint8_t>(slimeStatus.txPressureState);
    dst.txRecoveryReason = static_cast<uint8_t>(slimeStatus.txRecoveryReason);
    return true;
}

bool MachineLogDeferredRuntime::outputHasRoom() const {
    if (state_ == nullptr || state_->output == nullptr) return false;
    return machineLogOutputHasRoom(*state_->output, MACHINE_LOG_LINE_RESERVE_BYTES);
}

uint8_t MachineLogDeferredRuntime::lineCount(
    const MachineLogDeferredRecord& record) {
    switch (record.type) {
        case MachineLogDeferredRecordType::Imu:
            return static_cast<uint8_t>(2u +
                (record.payload.imu.emitBias ? 1u : 0u) +
                (record.mode == TrackerLogMode::Full ? 1u : 0u));
        case MachineLogDeferredRecordType::Mag:
            return static_cast<uint8_t>(2u +
                (record.mode == TrackerLogMode::Full ? 1u : 0u));
        case MachineLogDeferredRecordType::State:
        case MachineLogDeferredRecordType::BiasUpdate:
        case MachineLogDeferredRecordType::Network:
            return 1u;
    }
    return 1u;
}

bool MachineLogDeferredRuntime::service(uint8_t maxLines) {
    if (!ready() || count_ == 0u || maxLines == 0u) return false;
    ++status_.serviceCalls;
    if (!state_->enabled() || state_->output == nullptr) {
        reset(true);
        return false;
    }

    bool worked = false;
    uint8_t serviced = 0u;
    while (count_ != 0u && serviced < maxLines) {
        const MachineLogDeferredRecord& record = records_[head_];
        if (!outputHasRoom()) {
            ++counters_->serviceDeferral;
            break;
        }
        const uint32_t ageUs = micros() - record.queuedAtUs;
        if (ageUs > status_.maxRecordAgeUs) status_.maxRecordAgeUs = ageUs;
        writeRecordLine(record, headLine_, *state_->output);
        ++headLine_;
        ++serviced;
        worked = true;
        if (headLine_ >= lineCount(record)) {
            headLine_ = 0u;
            head_ = static_cast<uint8_t>((head_ + 1u) % TRACKER_MACHINE_LOG_QUEUE_RECORDS);
            --count_;
            ++status_.serialized;
            status_.queued = count_;
        }
    }
    return worked;
}

void MachineLogDeferredRuntime::writeRecordLine(
    const MachineLogDeferredRecord& record,
    uint8_t line,
    Stream& out) {
    switch (record.type) {
        case MachineLogDeferredRecordType::Imu: writeImuLine(record, line, out); break;
        case MachineLogDeferredRecordType::Mag: writeMagLine(record, line, out); break;
        case MachineLogDeferredRecordType::State:
            if (line == 0u) writeState(record, out);
            break;
        case MachineLogDeferredRecordType::BiasUpdate:
            if (line == 0u) writeBiasUpdate(record, out);
            break;
        case MachineLogDeferredRecordType::Network:
            if (line == 0u) writeNetwork(record, out);
            break;
    }
}

void MachineLogDeferredRuntime::writeImuLine(
    const MachineLogDeferredRecord& record,
    uint8_t line,
    Stream& out) {
    const MachineLogDeferredImuRecord& r = record.payload.imu;
    if (line == 0u) {
        out.print("Q,"); machineLogPrintU64Dec(out, r.timestampUs);
        out.print(','); out.print(record.sequence);
        out.print(','); out.print(r.dtUs);
        out.print(','); out.print(r.quaternionW, 7);
        out.print(','); out.print(r.quaternionX, 7);
        out.print(','); out.print(r.quaternionY, 7);
        out.print(','); out.print(r.quaternionZ, 7);
        out.print(",0x"); out.print(r.qualityFlags, HEX);
        out.print(','); out.print(r.confidence, 4);
        out.print(','); out.print(r.trackingState);
        out.print(','); out.print(r.accelTrust, 4);
        out.print(','); out.print(r.accelNormG, 5);
        out.print(','); out.print(r.accelVarianceG2, 8);
        out.print(','); out.print(r.gyroTrust, 4);
        out.print(','); out.print(r.gyroDps, 4);
        out.print(','); out.println(r.trackingRecovering ? 1 : 0);
        ++counters_->q;
        return;
    }

    if (line == 1u) {
        out.print("FIFO,"); machineLogPrintU64Dec(out, r.timestampUs);
        out.print(','); out.print(record.sequence);
        out.print(','); out.print(r.dtUs);
        out.print(','); out.print(r.hardwareTimestamp ? 1 : 0);
        out.print(','); out.print(r.fallbackTimestamp ? 1 : 0);
        out.print(','); out.print(r.estimatedDroppedBefore);
        out.print(','); out.print(r.fifoOverrun ? 1 : 0);
        out.print(','); out.print(r.fifoFull ? 1 : 0);
        out.print(','); out.print(r.fifoUnknown ? 1 : 0);
        out.print(",0x"); out.println(r.qualityFlags, HEX);
        ++counters_->fifo;
        return;
    }

    uint8_t nextLine = 2u;
    if (r.emitBias) {
        if (line == nextLine) {
            out.print("BIAS,"); machineLogPrintU64Dec(out, r.timestampUs);
            out.print(','); out.print(record.sequence);
            out.print(','); out.print(r.temperatureC, 3);
            out.print(','); out.print(r.currentBiasX, 8);
            out.print(','); out.print(r.currentBiasY, 8);
            out.print(','); out.print(r.currentBiasZ, 8);
            out.print(','); out.print(r.biasSource);
            out.print(','); out.print(r.biasFitQuality, 4);
            out.print(",0x"); out.print(r.biasFlags, HEX);
            out.print(','); out.print(r.runtimeBiasEnabled ? 1 : 0);
            out.print(','); out.println(r.runtimeBiasUpdates);
            ++counters_->bias;
            return;
        }
        ++nextLine;
    }

    if (record.mode == TrackerLogMode::Full && line == nextLine) {
        out.print("CAL,"); machineLogPrintU64Dec(out, r.timestampUs);
        out.print(','); out.print(record.sequence);
        out.print(','); out.print(r.calibratedAccelX, 6);
        out.print(','); out.print(r.calibratedAccelY, 6);
        out.print(','); out.print(r.calibratedAccelZ, 6);
        out.print(','); out.print(r.calibratedGyroX, 8);
        out.print(','); out.print(r.calibratedGyroY, 8);
        out.print(','); out.print(r.calibratedGyroZ, 8);
        out.print(','); out.print(r.temperatureC, 3);
        out.print(",0x"); out.println(r.qualityFlags, HEX);
        ++counters_->cal;
    }
}

void MachineLogDeferredRuntime::writeMagLine(
    const MachineLogDeferredRecord& record,
    uint8_t line,
    Stream& out) {
    const MachineLogDeferredMagRecord& r = record.payload.mag;
    if (line == 0u) {
        out.print("MAG,"); machineLogPrintU64Dec(out, r.timestampUs);
        out.print(','); out.print(record.sequence);
        out.print(','); out.print(r.magSequence);
        out.print(','); out.print(r.ageMs);
        out.print(','); out.print(r.rawNorm, 5);
        out.print(','); out.print(r.bodyNorm, 5);
        out.print(','); out.print(r.horizontalNorm, 5);
        out.print(','); out.print(r.headingValid ? 1 : 0);
        out.print(','); out.print(r.headingYawDeg, 4);
        out.print(','); out.print(r.headingInnovationDeg, 4);
        out.print(','); out.print(r.trustedForUse ? 1 : 0);
        out.print(",0x"); out.print(r.rejectFlagsForUse, HEX);
        out.print(','); out.print(r.dipDeg, 4);
        out.print(','); out.print(r.reliabilityState);
        out.print(','); out.print(r.reliabilityTrustedForYaw ? 1 : 0);
        out.print(",0x"); out.print(r.reliabilityFlags, HEX);
        out.print(','); out.print(r.reliabilityNormError, 5);
        out.print(','); out.print(r.reliabilityDipErrorDeg, 4);
        out.print(','); out.println(r.reliabilityHeadingErrorDeg, 4);
        ++counters_->mag;
        return;
    }

    uint8_t yawLine = 1u;
    if (record.mode == TrackerLogMode::Full) {
        if (line == 1u) {
            out.print("MAGR,"); machineLogPrintU64Dec(out, r.timestampUs);
            out.print(','); out.print(record.sequence);
            out.print(','); out.print(r.magSequence);
            out.print(','); out.print(r.rawX, 5);
            out.print(','); out.print(r.rawY, 5);
            out.print(','); out.print(r.rawZ, 5);
            out.print(','); out.print(r.calibratedX, 5);
            out.print(','); out.print(r.calibratedY, 5);
            out.print(','); out.print(r.calibratedZ, 5);
            out.print(','); out.print(r.bodyX, 5);
            out.print(','); out.print(r.bodyY, 5);
            out.print(','); out.print(r.bodyZ, 5);
            out.print(','); out.print(r.rawNorm, 5);
            out.print(','); out.print(r.calibratedNorm, 5);
            out.print(','); out.print(r.bodyNorm, 5);
            out.print(",0x"); out.print(r.rawFlags, HEX);
            out.print(",0x"); out.print(r.rejectFlagsForUse, HEX);
            out.print(','); out.println(r.trustedForUse ? 1 : 0);
            return;
        }
        yawLine = 2u;
    }

    if (line != yawLine) return;
    out.print("YAW,");
    machineLogPrintU64Dec(out, r.yawTimestampUs);
    out.print(','); out.print(record.sequence);
    out.print(','); out.print(r.yawValid ? 1 : 0);
    out.print(','); out.print(r.yawGateOpen ? 1 : 0);
    out.print(','); out.print(r.yawApplyAllowed ? 1 : 0);
    out.print(','); out.print(r.yawApplied ? 1 : 0);
    out.print(','); out.print(r.yawErrorDeg, 5);
    out.print(','); out.print(r.yawCorrectionStepDeg, 7);
    out.print(','); out.print(r.yawCombinedTrust, 4);
    out.print(",0x"); out.print(r.yawRejectFlags, HEX);
    out.print(','); out.print(r.yawCooldownRemainingMs);
    out.print(','); out.print(r.yawMode);
    out.print(','); out.print(r.yawReacquirePending ? 1 : 0);
    out.print(','); out.print(r.yawReacquireActive ? 1 : 0);
    out.print(','); out.print(r.yawFieldStableMs);
    out.print(','); out.println(r.yawMagneticHeadingRateDegS, 4);
    ++counters_->yaw;
}

void MachineLogDeferredRuntime::writeState(const MachineLogDeferredRecord& record,
                                           Stream& out) {
    const MachineLogDeferredStateRecord& r = record.payload.state;
    out.print("STATE,"); machineLogPrintU64Dec(out, r.timestampUs);
    out.print(','); out.print(record.sequence);
    out.print(','); out.print(r.state);
    out.print(','); out.print(r.reason);
    out.print(",0x"); out.print(r.flags, HEX);
    out.print(','); out.println(r.confidence, 4);
    ++counters_->state;
}

void MachineLogDeferredRuntime::writeBiasUpdate(const MachineLogDeferredRecord& record,
                                                Stream& out) {
    const MachineLogDeferredBiasUpdateRecord& r = record.payload.biasUpdate;
    out.print("BIASUPD,"); machineLogPrintU64Dec(out, r.timestampUs);
    out.print(','); out.print(record.sequence);
    out.print(','); out.print(r.temperatureC, 3);
    out.print(','); out.print(r.residualX, 8);
    out.print(','); out.print(r.residualY, 8);
    out.print(','); out.print(r.residualZ, 8);
    out.print(','); out.print(r.stdX, 8);
    out.print(','); out.print(r.stdY, 8);
    out.print(','); out.print(r.stdZ, 8);
    out.print(','); out.print(r.deltaX, 8);
    out.print(','); out.print(r.deltaY, 8);
    out.print(','); out.print(r.deltaZ, 8);
    out.print(','); out.print(r.trimX, 8);
    out.print(','); out.print(r.trimY, 8);
    out.print(','); out.print(r.trimZ, 8);
    out.print(",0x"); out.println(r.flags, HEX);
    ++counters_->biasUpdate;
}

void MachineLogDeferredRuntime::writeNetwork(
    const MachineLogDeferredRecord& record,
    Stream& out) {
    const MachineLogDeferredNetworkRecord& r = record.payload.network;
    out.print("NET,"); machineLogPrintU64Dec(out, r.timestampUs);
    out.print(','); out.print(record.sequence);
    out.print(','); out.print(r.wifiConnected ? 1 : 0);
    out.print(','); out.print(r.wifiDisconnects);
    out.print(','); out.print(r.wifiConnectTimeouts);
    out.print(','); out.print(r.wifiRssiDbm);
    out.print(','); out.print(r.slimeState);
    out.print(','); out.print(r.udpReady ? 1 : 0);
    out.print(','); out.print(r.serverFound ? 1 : 0);
    out.print(','); out.print(r.rotationSent);
    out.print(','); out.print(r.rotationSendDue);
    out.print(','); out.print(r.rotationMissedDeadlines);
    out.print(','); out.print(r.rotationLateEvents);
    out.print(','); out.print(r.sendFailures);
    out.print(','); out.print(r.rotationSendFailures);
    out.print(','); out.print(r.controlSendFailures);
    out.print(','); out.print(r.telemetrySendFailures);
    out.print(','); out.print(r.discoverySendFailures);
    out.print(','); out.print(r.txPressureFailures);
    out.print(','); out.print(r.txOtherFailures);
    out.print(','); out.print(r.udpRebindSuccesses);
    out.print(','); out.print(r.udpRebindFailures);
    out.print(','); out.print(r.udpFullReopenEscalations);
    out.print(','); out.print(r.consecutiveSendFailures);
    out.print(','); out.print(r.lastUdpSendError);
    out.print(','); out.print(r.lastSuccessfulMotionTxAgeMs);
    out.print(','); out.print(r.txPressureState);
    out.print(','); out.println(r.txRecoveryReason);
    ++counters_->network;
}

MachineLogDeferredStatus MachineLogDeferredRuntime::status() const {
    MachineLogDeferredStatus result = status_;
    result.queued = count_;
    return result;
}

void MachineLogDeferredRuntime::printStatus(Stream& out) const {
    const MachineLogDeferredStatus s = status();
    out.print("LOGSTAT,PIPELINE,");
    out.print(s.queued); out.print(',');
    out.print(s.highWater); out.print(',');
    out.print(s.enqueued); out.print(',');
    out.print(s.serialized); out.print(',');
    out.print(s.serviceCalls); out.print(',');
    out.println(s.maxRecordAgeUs);
    if (counters_ != nullptr) {
        out.print("LOGSTAT,DROPS,");
        out.print(counters_->producerQueueDrop); out.print(',');
        out.print(counters_->serviceDeferral); out.print(',');
        out.print(counters_->shutdownDrop); out.print(',');
        out.println(counters_->disconnectAbort);
    }
}

} // namespace tracker
