#include "runtime/output_runtime.hpp"

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
    return config.data.output.quaternionOutputEnabled;
#else
    (void)config;
    return false;
#endif
}

void PreparedOutputRuntime::reset() {
    snapshot_ = TrackerPreparedOutputSnapshot{};
    seqLock_ = 0;
}

void PreparedOutputRuntime::update(const TrackerConfig& config,
                                   uint32_t runtimeSamples,
                                   uint64_t timestampUs,
                                   const Ahrs6Dof& ahrs,
                                   const ImuQualityResult& quality) {
#if TRACKER_ENABLE_PREPARED_OUTPUT_SNAPSHOT
    if (!enabled(config)) return;

    uint32_t startSeq = seqLock_ + 1u;
    if ((startSeq & 1u) == 0u) startSeq++;
    seqLock_ = startSeq;

    const Ahrs6DofStats& ast = ahrs.stats();
    snapshot_.valid = ahrs.initialized();
    snapshot_.sequence++;
    snapshot_.runtimeSample = runtimeSamples;
    snapshot_.ahrsUpdateCount = ast.updateCount;
    snapshot_.timestampUs = timestampUs;
    snapshot_.q = ahrs.quaternionPositiveW();
    snapshot_.qualityFlags = quality.flags;
    snapshot_.confidence = quality.overallConfidence;

    seqLock_ = startSeq + 1u;
#else
    (void)config;
    (void)runtimeSamples;
    (void)timestampUs;
    (void)ahrs;
    (void)quality;
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

void maybePrintBootHeartbeat(Stream& out,
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
    return;
#else
    if (streamState.mode != TrackerStreamMode::Heartbeat) return;
    if (staticTestActive) return;

    if (nowMs - lastHeartbeatMs < 60000UL) return;
    lastHeartbeatMs = nowMs;

    out.print("# alive uptime_s="); out.print(nowMs / 1000UL);
    out.print(" samples="); out.print(runtimeSamples);
    out.print(" fifo_int="); out.print(fifoIntCount);
    out.print(" temp_c="); out.print(latestTempC, 2);
    out.print(" mag="); out.print(magSamples);
    out.print(" stream="); out.println("heartbeat");
#endif
}

} // namespace tracker
