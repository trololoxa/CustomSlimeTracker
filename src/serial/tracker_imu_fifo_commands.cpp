#include "serial/tracker_imu_fifo_commands.hpp"

#include <Arduino.h>
#include <cstdint>

#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/imu_quality.hpp"
#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "serial/tracker_serial_context.hpp"
#include "serial/tracker_mag_commands.hpp"

namespace tracker {

const char* trackerSerialOdrName(Lsm6dsv::Odr odr) {
    switch (odr) {
        case Lsm6dsv::Odr::PowerDown: return "0";
        case Lsm6dsv::Odr::Hz1_875: return "1.875";
        case Lsm6dsv::Odr::Hz7_5: return "7.5";
        case Lsm6dsv::Odr::Hz15: return "15";
        case Lsm6dsv::Odr::Hz30: return "30";
        case Lsm6dsv::Odr::Hz60: return "60";
        case Lsm6dsv::Odr::Hz120: return "120";
        case Lsm6dsv::Odr::Hz240: return "240";
        case Lsm6dsv::Odr::Hz480: return "480";
        case Lsm6dsv::Odr::Hz960: return "960";
        case Lsm6dsv::Odr::Hz1920: return "1920";
        case Lsm6dsv::Odr::Hz3840: return "3840";
        case Lsm6dsv::Odr::Hz7680: return "7680";
    }
    return "?";
}

bool trackerSerialParseRuntimeOdr(const char* token, Lsm6dsv::Odr& outOdr) {
    if (!token) return false;
    uint32_t hz = 0;
    if (!tracker_serial_detail::parseU32(token, hz)) return false;
    switch (hz) {
        case 120: outOdr = Lsm6dsv::Odr::Hz120; return true;
        case 240: outOdr = Lsm6dsv::Odr::Hz240; return true;
        case 480: outOdr = Lsm6dsv::Odr::Hz480; return true;
        case 960: outOdr = Lsm6dsv::Odr::Hz960; return true;
        default: return false;
    }
}

void trackerSerialPrintFifoStats(Stream& out, const Lsm6dsvFifoReader::DrainStats& fs) {
    out.println("# FIFO STATS");
    out.print("fifo_words_read="); out.println(fs.fifoWordsRead);
    out.print("imu_samples="); out.println(fs.imuSamplesProduced);
    out.print("gyro_words="); out.println(fs.gyroWords);
    out.print("accel_words="); out.println(fs.accelWords);
    out.print("timestamp_words="); out.println(fs.timestampWords);
    out.print("temperature_words="); out.println(fs.tempWords);
    out.print("unknown_words="); out.println(fs.unknownWords);
    out.print("overrun_events="); out.println(fs.overrunEvents);
    out.print("full_events="); out.println(fs.fullEvents);
    out.print("watermark_events="); out.println(fs.watermarkEvents);
    out.print("max_unread_words="); out.println(fs.maxUnreadWordsSeen);
    out.print("tag_counter_jumps="); out.println(fs.tagCounterJumps);
    out.print("gyro_tag_counter_jumps="); out.println(fs.gyroTagCounterJumps);
    out.print("accel_tag_counter_jumps="); out.println(fs.accelTagCounterJumps);
    out.print("hw_timestamp_assigned="); out.println(fs.hwTimestampAssigned);
    out.print("fallback_timestamp_assigned="); out.println(fs.fallbackTimestampAssigned);
    out.print("timestamp_backwards="); out.println(fs.timestampBackwards);
    out.print("timestamp_duplicate="); out.println(fs.timestampDuplicate);
    out.print("timestamp_large_gap="); out.println(fs.timestampLargeGap);
    out.print("timestamp_meta_bdr_xl_mismatch="); out.println(fs.timestampMetaBdrXlMismatch);
    out.print("timestamp_meta_bdr_gy_mismatch="); out.println(fs.timestampMetaBdrGyMismatch);
    out.print("gyro_saturation_count="); out.println(fs.gyroSaturationCount);
    out.print("accel_saturation_count="); out.println(fs.accelSaturationCount);
    out.print("sensorhub0_words="); out.println(fs.sensorHubSlave0Words);
    out.print("sensorhub_nack_words="); out.println(fs.sensorHubNackWords);
    out.print("mag_samples_produced="); out.println(fs.magSamplesProduced);
    out.print("mag_queue_overflow="); out.println(fs.magQueueOverflow);
    out.print("mag_tag_counter_jumps="); out.println(fs.magTagCounterJumps);
    out.print("mag_raw_saturation_count="); out.println(fs.magRawSaturationCount);
    out.print("mag_last_xyz="); out.print(fs.lastMagX); out.print(','); out.print(fs.lastMagY); out.print(','); out.println(fs.lastMagZ);
    out.print("mag_last_norm_raw="); out.println(fs.lastMagRawNorm, 3);
    out.print("mag_dt_last_us="); out.println(fs.lastMagDtUs);
    out.print("mag_dt_min_us="); out.println(fs.minMagDtUs);
    out.print("mag_dt_max_us="); out.println(fs.maxMagDtUs);
    out.print("latest_temp_valid="); out.println(fs.latestTempValid ? "yes" : "no");
    out.print("latest_temp_c="); out.println(fs.latestTempC, 3);
    out.print("sample_period_us="); out.println(fs.samplePeriodUs, 3);
    out.print("timestamp_tick_us="); out.println(fs.timestampTickUs, 6);
}

void trackerSerialPrintQualityStats(Stream& out, const ImuQualityCounters& qc) {
    out.println("# QUALITY STATS");
    out.print("samples="); out.println(qc.samples);
    out.print("hw_timestamp_samples="); out.println(qc.hwTimestampSamples);
    out.print("fallback_timestamp_samples="); out.println(qc.fallbackTimestampSamples);
    out.print("zero_timestamp_samples="); out.println(qc.zeroTimestampSamples);
    out.print("nonmonotonic_timestamp_samples="); out.println(qc.nonMonotonicTimestampSamples);
    out.print("large_gap_samples="); out.println(qc.largeGapSamples);
    out.print("estimated_dropped_samples="); out.println(qc.estimatedDroppedSamples);
    out.print("fifo_overrun_events="); out.println(qc.fifoOverrunEvents);
    out.print("fifo_full_events="); out.println(qc.fifoFullEvents);
    out.print("fifo_unknown_tag_events="); out.println(qc.fifoUnknownTagEvents);
    out.print("fifo_tag_counter_jumps="); out.println(qc.fifoTagCounterJumps);
    out.print("fifo_gyro_tag_counter_jumps="); out.println(qc.fifoGyroTagCounterJumps);
    out.print("fifo_accel_tag_counter_jumps="); out.println(qc.fifoAccelTagCounterJumps);
    out.print("gyro_saturated_samples="); out.println(qc.gyroSaturatedSamples);
    out.print("accel_saturated_samples="); out.println(qc.accelSaturatedSamples);
    out.print("gyro_near_saturated_samples="); out.println(qc.gyroNearSaturatedSamples);
    out.print("accel_near_saturated_samples="); out.println(qc.accelNearSaturatedSamples);
    out.print("accel_norm_outliers="); out.println(qc.accelNormOutliers);
    out.print("ahrs_skipped_samples="); out.println(qc.ahrsSkippedSamples);
    out.print("accel_correction_disabled_samples="); out.println(qc.accelCorrectionDisabledSamples);
    out.print("fifo_recovery_requests="); out.println(qc.fifoRecoveryRequests);
    out.print("mean_dt_us="); out.println(qc.meanDtUs(), 6);
    out.print("min_dt_us="); out.println(qc.minDtUs, 6);
    out.print("max_dt_us="); out.println(qc.maxDtUs, 6);
}

void trackerSerialPrintImuRuntimeStatus(TrackerSerialCommandContext& ctx, Stream& out) {
    out.print("imu_initialized="); out.println(ctx.lsm && ctx.lsm->isInitialized() ? "yes" : "no");
    if (ctx.lsm) {
        out.print("imu_last_who=0x"); out.println(ctx.lsm->lastWhoAmI(), HEX);
        out.print("imu_last_error="); out.println(static_cast<int>(ctx.lsm->lastError()));
    }
    if (ctx.config) {
        out.print("imu_config_odr_hz="); out.println(trackerSerialOdrName(ctx.config->data.imu.imuOdr));
        out.print("fifo_accel_bdr_hz="); out.println(trackerSerialOdrName(ctx.config->data.fifo.accelBdr));
        out.print("fifo_gyro_bdr_hz="); out.println(trackerSerialOdrName(ctx.config->data.fifo.gyroBdr));
        out.print("fifo_watermark_words="); out.println(ctx.config->data.fifo.watermarkWords);
        out.print("fifo_max_words_per_drain="); out.println(ctx.config->data.fifo.maxWordsPerDrain);
        out.print("fifo_max_drain_rounds_per_event="); out.println(ctx.config->data.fifo.maxDrainRoundsPerEvent);
    }
    if (ctx.fifo) {
        const auto& fs = ctx.fifo->stats();
        out.print("fifo_configured="); out.println(ctx.fifo->isConfigured() ? "yes" : "no");
        out.print("fifo_sample_period_us="); out.println(fs.samplePeriodUs, 3);
        out.print("fifo_timestamp_tick_us="); out.println(fs.timestampTickUs, 6);
        out.print("fifo_internal_freq_fine="); out.println(fs.internalFreqFine);
        out.print("fifo_max_unread_words="); out.println(fs.maxUnreadWordsSeen);
        out.print("fifo_overrun_events="); out.println(fs.overrunEvents);
        out.print("fifo_full_events="); out.println(fs.fullEvents);
    }
}

bool trackerSerialLiveReconfigureImuFifo(TrackerSerialCommandContext& ctx, Stream& out) {
    if (!ctx.config || !ctx.lsm || !ctx.fifo) {
        tracker_serial_detail::printErr(out, "imu/fifo/config not available");
        return false;
    }

    if (ctx.streamState) ctx.streamState->mode = TrackerStreamMode::Off;
    if (ctx.logState) ctx.logState->mode = TrackerLogMode::Off;

    if (!ctx.lsm->begin(ctx.config->makeLsmConfig())) {
        out.print("# ERR imu reconfigure failed last_error=");
        out.println(static_cast<int>(ctx.lsm->lastError()));
        return false;
    }

    if (!ctx.fifo->configure(ctx.config->makeFifoConfig())) {
        tracker_serial_detail::printErr(out, "fifo reconfigure failed");
        return false;
    }

    ctx.fifo->resetTimestampReconstruction(0);
    if (ctx.quality) {
        ctx.quality->reset();
        ctx.quality->syncFifoStats(ctx.fifo->stats());
    }
    if (ctx.resetFifoRuntime) ctx.resetFifoRuntime(ctx.resetFifoRuntimeUser);
    if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
    rearmMagIfNeeded(ctx);
    return true;
}

bool trackerSerialSaveConfigIfRequested(TrackerSerialCommandContext& ctx, Stream& out, bool save) {
    if (!save) return true;
    if (!ctx.config || !ctx.configStore) {
        tracker_serial_detail::printErr(out, "config store not available; changed in RAM only");
        return false;
    }
    ctx.config->updateCrc();
    if (!ctx.configStore->save(*ctx.config)) {
        out.print("# ERR save failed: ");
        out.println(ctx.configStore->lastErrorName());
        return false;
    }
    return true;
}

void trackerSerialDispatchImuCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = ctx.io ? *ctx.io : Serial;
    if (!ctx.lsm) {
        tracker_serial_detail::printErr(out, "imu not available");
        return;
    }
    if (argc < 2 || tracker_serial_detail::eqIgnoreCase(argv[1], "status")) {
        trackerSerialPrintImuRuntimeStatus(ctx, out);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "rate") || tracker_serial_detail::eqIgnoreCase(argv[1], "odr")) {
        if (argc < 3) {
            trackerSerialPrintImuRuntimeStatus(ctx, out);
            out.println("# usage: imu rate <120|240|480|960> [save]");
            return;
        }
        if (!ctx.config) {
            tracker_serial_detail::printErr(out, "config not available");
            return;
        }
        Lsm6dsv::Odr odr;
        if (!trackerSerialParseRuntimeOdr(argv[2], odr)) {
            tracker_serial_detail::printErr(out, "unsupported ODR; expected 120, 240, 480 or 960");
            return;
        }
        const bool save = (argc >= 4 && tracker_serial_detail::eqIgnoreCase(argv[3], "save"));
        ctx.config->data.imu.imuOdr = odr;
        ctx.config->data.fifo.accelBdr = odr;
        ctx.config->data.fifo.gyroBdr = odr;
        ctx.config->data.fifo.samplePeriodUsOverride = 0.0f;
        ctx.config->sanitize();
        ctx.config->updateCrc();

        if (!trackerSerialLiveReconfigureImuFifo(ctx, out)) return;
        if (!trackerSerialSaveConfigIfRequested(ctx, out, save)) return;

        out.print("# OK imu/fifo rate set hz="); out.print(trackerSerialOdrName(odr));
        out.println(save ? " saved=yes" : " saved=no");
        trackerSerialPrintImuRuntimeStatus(ctx, out);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "whoami")) {
        uint8_t who = 0;
        const bool ok = ctx.lsm->readWhoAmI(who);
        out.print(ok ? "# OK WHO_AM_I=0x" : "# ERR WHO_AM_I read failed, last=0x");
        out.println(who, HEX);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "read")) {
        Lsm6dsv::Sample s;
        if (!ctx.lsm->readSample(s, micros(), true)) {
            tracker_serial_detail::printErr(out, "imu read failed");
            return;
        }
        tracker_serial_detail::printVec3(out, "accel_g", s.accel_g, 6);
        out.print(" norm="); out.println(s.accel_g.norm(), 6);
        tracker_serial_detail::printVec3(out, "gyro_rad_s", s.gyro_rad_s, 7);
        out.print(" norm="); out.println(s.gyro_rad_s.norm(), 7);
        out.print("temp_c="); out.println(s.temp_c, 3);
        return;
    }

    tracker_serial_detail::printErr(out, "unknown imu command");
}

void trackerSerialDispatchFifoCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = ctx.io ? *ctx.io : Serial;
    if (!ctx.fifo) {
        tracker_serial_detail::printErr(out, "fifo not available");
        return;
    }

    if (argc < 2 || tracker_serial_detail::eqIgnoreCase(argv[1], "status")) {
        Lsm6dsvFifoReader::Status st;
        if (!ctx.fifo->readStatus(st)) {
            tracker_serial_detail::printErr(out, "fifo status read failed");
            return;
        }
        out.print("fifo_unread_words="); out.println(st.unreadWords);
        out.print("fifo_status1=0x"); out.println(st.rawStatus1, HEX);
        out.print("fifo_status2=0x"); out.println(st.rawStatus2, HEX);
        out.print("fifo_watermark="); out.println(st.watermark ? "yes" : "no");
        out.print("fifo_overrun="); out.println(st.overrun ? "yes" : "no");
        out.print("fifo_full="); out.println(st.full ? "yes" : "no");
        if (ctx.config) {
            out.print("fifo_config_watermark_words="); out.println(ctx.config->data.fifo.watermarkWords);
            out.print("fifo_config_max_words_per_drain="); out.println(ctx.config->data.fifo.maxWordsPerDrain);
            out.print("fifo_config_max_drain_rounds_per_event="); out.println(ctx.config->data.fifo.maxDrainRoundsPerEvent);
        }
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "stats")) {
        trackerSerialPrintFifoStats(out, ctx.fifo->stats());
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "watermark") || tracker_serial_detail::eqIgnoreCase(argv[1], "wm")) {
        if (!ctx.config) {
            tracker_serial_detail::printErr(out, "config not available");
            return;
        }
        if (argc < 3) {
            tracker_serial_detail::printErr(out, "usage: fifo watermark <1..255> [save]");
            return;
        }
        uint32_t words = 0;
        if (!tracker_serial_detail::parseU32(argv[2], words) || words < 1 || words > 255) {
            tracker_serial_detail::printErr(out, "invalid watermark; expected 1..255 words");
            return;
        }
        const bool save = (argc >= 4 && tracker_serial_detail::eqIgnoreCase(argv[3], "save"));
        ctx.config->data.fifo.watermarkWords = static_cast<uint8_t>(words);
        ctx.config->sanitize();
        ctx.config->updateCrc();
        if (!trackerSerialLiveReconfigureImuFifo(ctx, out)) return;
        if (!trackerSerialSaveConfigIfRequested(ctx, out, save)) return;
        out.print("# OK fifo watermark set words="); out.print(words);
        out.println(save ? " saved=yes" : " saved=no");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "drain")) {
        if (!ctx.config) {
            tracker_serial_detail::printErr(out, "config not available");
            return;
        }
        if (argc < 4) {
            tracker_serial_detail::printErr(out, "usage: fifo drain <max_words_per_drain> <rounds_per_event> [save]");
            return;
        }
        uint32_t maxWords = 0;
        uint32_t rounds = 0;
        if (!tracker_serial_detail::parseU32(argv[2], maxWords) || maxWords < 16 || maxWords > 4096) {
            tracker_serial_detail::printErr(out, "invalid max_words_per_drain; expected 16..4096");
            return;
        }
        if (!tracker_serial_detail::parseU32(argv[3], rounds) || rounds < 1 || rounds > 32) {
            tracker_serial_detail::printErr(out, "invalid rounds_per_event; expected 1..32");
            return;
        }
        const bool save = (argc >= 5 && tracker_serial_detail::eqIgnoreCase(argv[4], "save"));
        ctx.config->data.fifo.maxWordsPerDrain = static_cast<uint16_t>(maxWords);
        ctx.config->data.fifo.maxDrainRoundsPerEvent = static_cast<uint8_t>(rounds);
        ctx.config->sanitize();
        ctx.config->updateCrc();
        if (!trackerSerialLiveReconfigureImuFifo(ctx, out)) return;
        if (!trackerSerialSaveConfigIfRequested(ctx, out, save)) return;
        out.print("# OK fifo drain set max_words="); out.print(maxWords);
        out.print(" rounds="); out.print(rounds);
        out.println(save ? " saved=yes" : " saved=no");
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "reset")) {
        const uint64_t lastTs = ctx.fifo->stats().lastAssignedTimestampUs;
        const bool ok = ctx.fifo->resetFifo();
        ctx.fifo->resetTimestampReconstruction(lastTs);
        if (ctx.quality) {
            ctx.quality->reset();
            ctx.quality->syncFifoStats(ctx.fifo->stats());
        }
        if (ctx.resetFifoRuntime) ctx.resetFifoRuntime(ctx.resetFifoRuntimeUser);
        if (ok) tracker_serial_detail::printOk(out, "fifo reset");
        else tracker_serial_detail::printErr(out, "fifo reset failed");
        return;
    }

    tracker_serial_detail::printErr(out, "unknown fifo command");
}

void trackerSerialDispatchQualityCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = ctx.io ? *ctx.io : Serial;
    if (!ctx.quality) {
        tracker_serial_detail::printErr(out, "quality monitor not available");
        return;
    }
    if (argc < 2 || tracker_serial_detail::eqIgnoreCase(argv[1], "stats")) {
        trackerSerialPrintQualityStats(out, ctx.quality->counters());
        return;
    }
    if (tracker_serial_detail::eqIgnoreCase(argv[1], "reset")) {
        ctx.quality->reset();
        if (ctx.fifo) ctx.quality->syncFifoStats(ctx.fifo->stats());
        tracker_serial_detail::printOk(out, "quality counters reset");
        return;
    }
    tracker_serial_detail::printErr(out, "unknown quality command");
}

} // namespace tracker
