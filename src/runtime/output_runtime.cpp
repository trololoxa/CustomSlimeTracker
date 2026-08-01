#include "runtime/output_runtime.hpp"

#include "build_config/tracking_tuning.hpp"

namespace tracker {

void outputPrintU64Dec(Stream& out, uint64_t v) {
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


bool PreparedOutputRuntime::enabled(const TrackerConfig& config) const {
#if TRACKER_ENABLE_PREPARED_OUTPUT_SNAPSHOT
    // Prepared quaternion snapshots are a runtime service, not a serial/output
    // mode. SlimeVR UDP consumes these snapshots independently from the local
    // serial stream state, so keep them fresh whenever the feature is compiled
    // in. The copy path remains lock-free and cheap.
    (void)config;
    return true;
#else
    (void)config;
    return false;
#endif
}

void PreparedOutputRuntime::reset() {
    snapshot_ = TrackerPreparedOutputSnapshot{};
    seqLock_ = 0;
    lastPublishedTimestampUs_ = 0;
}

bool PreparedOutputRuntime::update(const TrackerConfig& config,
                                   uint32_t runtimeSamples,
                                   uint64_t timestampUs,
                                   const Ahrs6Dof& ahrs,
                                   const ImuQualityResult& quality,
                                   const Vec3& accelDeviceG,
                                   uint32_t softwareQueueAgeUs) {
#if TRACKER_ENABLE_PREPARED_OUTPUT_SNAPSHOT
    if (!enabled(config)) return false;

    const Ahrs6DofStats& ast = ahrs.stats();
    const bool orientationCoherent =
        ahrs.initialized() && timestampUs != 0 && ast.lastIntegratedTimestampUs == timestampUs;

    // Never retain a stale valid snapshot across an AHRS-rejected sample. Valid
    // snapshots may otherwise be rate-limited because SlimeVR consumes at 100
    // Hz and does not benefit from rebuilding gravity/linear acceleration at
    // the full 960 Hz IMU ODR.
    if (orientationCoherent && lastPublishedTimestampUs_ != 0u &&
        timestampUs > lastPublishedTimestampUs_ &&
        timestampUs - lastPublishedTimestampUs_ < cfg::PREPARED_OUTPUT_MIN_INTERVAL_US) {
        return false;
    }

    // Read the MCU clock only for an actual publication (roughly the output
    // snapshot rate), not for every 960 Hz IMU sample.
    const uint32_t publishedAtMcuUs = micros();

    uint32_t startSeq = seqLock_ + 1u;
    if ((startSeq & 1u) == 0u) startSeq++;
    seqLock_ = startSeq;

    snapshot_.valid = orientationCoherent;
    snapshot_.linearAccelerationValid = false;
    snapshot_.linearAccelerationInvalidFlags = prepared_output_motion_flags::NONE;
    snapshot_.sequence++;
    snapshot_.runtimeSample = runtimeSamples;
    snapshot_.ahrsUpdateCount = ast.updateCount;
    snapshot_.timestampUs = timestampUs;
    snapshot_.publishedAtMcuUs = publishedAtMcuUs;
    snapshot_.softwareQueueAgeUs = softwareQueueAgeUs;
    snapshot_.q = orientationCoherent ? ahrs.quaternionPositiveW() : Quat::identity();
    snapshot_.linearAccelerationDeviceG = Vec3::zero();
    snapshot_.qualityFlags = quality.flags;
    snapshot_.confidence = quality.overallConfidence;

    const bool motionCalibrationReady =
        config.data.accelCal.valid && config.data.frame.sensorToDeviceValid;
    uint8_t motionInvalidFlags = prepared_output_motion_flags::NONE;
    if (!motionCalibrationReady) {
        motionInvalidFlags |= prepared_output_motion_flags::CONFIGURATION_NOT_READY;
    }
    if (quality.has(imu_quality_flags::ACCEL_COMPONENT_MISSING)) {
        motionInvalidFlags |= prepared_output_motion_flags::ACCEL_COMPONENT_MISSING;
    }
    if (quality.has(imu_quality_flags::FIFO_PAIR_DEGRADED)) {
        motionInvalidFlags |= prepared_output_motion_flags::PAIR_COHERENCY_DEGRADED;
    }
    if (quality.has(imu_quality_flags::ACCEL_SATURATED)) {
        motionInvalidFlags |= prepared_output_motion_flags::ACCEL_SATURATED;
    }
    if (!accelDeviceG.isFinite()) {
        motionInvalidFlags |= prepared_output_motion_flags::NON_FINITE;
    }

    // ACCEL_NORM_OUTLIER intentionally does not invalidate motion output. It
    // disables gravity correction in the AHRS because the sample contains
    // dynamic acceleration, but that same dynamic component is the signal
    // SlimeVR packet 4 and step mounting need.
    if (orientationCoherent && quality.shouldUseAccelOutput && motionInvalidFlags == 0u) {
        const Vec3 worldUp = ahrs.config().worldUp;
        const Vec3 gravityDeviceG = snapshot_.q.inverseRotate(worldUp);
        const Vec3 linearDeviceG = accelDeviceG - gravityDeviceG;
        if (gravityDeviceG.isFinite() && linearDeviceG.isFinite()) {
            snapshot_.linearAccelerationDeviceG = linearDeviceG;
            snapshot_.linearAccelerationValid = true;
        } else {
            motionInvalidFlags |= prepared_output_motion_flags::NON_FINITE;
        }
    }
    snapshot_.linearAccelerationInvalidFlags = motionInvalidFlags;

    lastPublishedTimestampUs_ = orientationCoherent ? timestampUs : 0u;
    seqLock_ = startSeq + 1u;
    return true;
#else
    (void)config;
    (void)runtimeSamples;
    (void)timestampUs;
    (void)ahrs;
    (void)quality;
    (void)softwareQueueAgeUs;
    (void)accelDeviceG;
#endif
}

bool PreparedOutputRuntime::copy(TrackerPreparedOutputSnapshot& out) const {
#if TRACKER_ENABLE_PREPARED_OUTPUT_SNAPSHOT
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        const uint32_t seqBefore = seqLock_;
        if ((seqBefore & 1u) != 0u) continue;

        const TrackerPreparedOutputSnapshot tmp = snapshot_;
        const uint32_t seqAfter = seqLock_;

        if (seqBefore == seqAfter && (seqAfter & 1u) == 0u) {
            out = tmp;
            return tmp.valid;
        }
    }
#endif
    out = TrackerPreparedOutputSnapshot{};
    return false;
}

void emitSerialStreamIfNeeded(TrackerSerialStreamState& streamState,
                                     Stream& out,
                                     const Lsm6dsv::RawSample& raw,
                                     const Lsm6dsv::Sample& scaled,
                                     const Lsm6dsv::Sample& calibrated,
                                     const Ahrs6Dof& ahrs,
                                     const ImuQualityResult& quality,
                                     uint32_t nowUs) {
#if !TRACKER_ENABLE_SERIAL_STREAM
    (void)streamState;
    (void)out;
    (void)raw;
    (void)scaled;
    (void)calibrated;
    (void)ahrs;
    (void)quality;
    (void)nowUs;
    return;
#else
    (void)scaled;
    // Hot-path fast return: in production/WiFi mode stream is normally off.
    // Avoid calling micros() on every IMU sample just to discover that.
    if (streamState.mode == TrackerStreamMode::Off ||
        streamState.mode == TrackerStreamMode::Heartbeat) {
        return;
    }

    if (!trackerSerialStreamDue(streamState, nowUs)) return;

    switch (streamState.mode) {
        case TrackerStreamMode::Off:
            break;
        case TrackerStreamMode::Raw:
            trackerSerialEmitRaw(out, raw, quality.flags);
            break;
        case TrackerStreamMode::Scaled:
            trackerSerialEmitScaled(out, raw.t_us, calibrated, quality.flags);
            break;
        case TrackerStreamMode::Quat:
            trackerSerialEmitQuat(out, raw.t_us, ahrs.quaternionPositiveW(), quality.flags, quality.overallConfidence);
            break;
        case TrackerStreamMode::Heartbeat:
            break;
        case TrackerStreamMode::Debug:
            out.print("DBG,t="); outputPrintU64Dec(out, raw.t_us);
            out.print(",euler=");
            {
                const Vec3 e = ahrs.eulerDeg();
                out.print(e.x, 2); out.print(',');
                out.print(e.y, 2); out.print(',');
                out.print(e.z, 2);
            }
            out.print(",temp="); out.print(calibrated.temp_c, 2);
            out.print(",qflags=0x"); out.println(quality.flags, HEX);
            break;
    }
#endif
}

bool maybePrintBootHeartbeat(Stream& out,
                                    const TrackerSerialStreamState& streamState,
                                    bool staticTestActive,
                                    uint32_t nowMs,
                                    uint32_t& lastHeartbeatMs,
                                    uint32_t runtimeSamples,
                                    uint32_t fifoIntCount,
                                    float latestTempC,
                                    uint32_t magSamples) {
#if !TRACKER_ENABLE_BOOT_HEARTBEAT
    (void)out;
    (void)streamState;
    (void)staticTestActive;
    (void)nowMs;
    (void)lastHeartbeatMs;
    (void)runtimeSamples;
    (void)fifoIntCount;
    (void)latestTempC;
    (void)magSamples;
    return false;
#else
    if (streamState.mode != TrackerStreamMode::Heartbeat) return false;
    if (staticTestActive) return false;

    if (nowMs - lastHeartbeatMs < 60000UL) return false;
    lastHeartbeatMs = nowMs;

    out.print("# alive uptime_s="); out.print(nowMs / 1000UL);
    out.print(" samples="); out.print(runtimeSamples);
    out.print(" fifo_int="); out.print(fifoIntCount);
    out.print(" temp_c="); out.print(latestTempC, 2);
    out.print(" mag="); out.print(magSamples);
    out.print(" stream="); out.println("heartbeat");
    return true;
#endif
}

} // namespace tracker
