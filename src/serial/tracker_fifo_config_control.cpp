#include "serial/tracker_fifo_config_control.hpp"

#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/imu_quality.hpp"
#include "serial/tracker_serial_parse.hpp"
#include "serial/tracker_serial_print.hpp"
#include "defines.h"

#if TRACKER_ENABLE_MAG_COMMANDS
#include "serial/tracker_mag_commands.hpp"
#endif

namespace tracker {

namespace {

bool is(const char* a, const char* b) {
    return tracker_serial_detail::eqIgnoreCase(a, b);
}

bool validateSaveArg(int argc, char** argv, int index, bool& save, Stream& out) {
    save = false;
    if (argc <= index) return true;
    if (argc == index + 1 && is(argv[index], "save")) {
        save = true;
        return true;
    }
    tracker_serial_detail::printErr(out, "unexpected argument; only optional 'save' is allowed");
    return false;
}

bool persistCandidate(TrackerSerialCommandContext& ctx,
                      Stream& out,
                      TrackerConfig candidate) {
    if (!ctx.configStore) {
        tracker_serial_detail::printErr(out, "config store not available");
        return false;
    }
    if (!ctx.configStore->save(candidate)) {
        out.print("# ERR config save failed: ");
        out.println(ctx.configStore->lastErrorName());
        return false;
    }
    return true;
}

bool restorePersistedPrevious(TrackerSerialCommandContext& ctx,
                              Stream& out,
                              TrackerConfig previous) {
    if (!ctx.configStore) return false;
    if (ctx.configStore->save(previous)) {
        out.println("# WARN previous config restored in NVS");
        return true;
    }
    out.print("# ERR previous config NVS restore failed: ");
    out.println(ctx.configStore->lastErrorName());
    return false;
}

void requestReconfigureRecovery(TrackerSerialCommandContext& ctx,
                                uint64_t timestampUs) {
    if (!ctx.requestTrackingRecovery) return;
    ctx.requestTrackingRecovery(
        imu_quality_flags::FIFO_RECOVERY_REQUESTED,
        "imu_fifo_reconfigure",
        timestampUs,
        ctx.requestTrackingRecoveryUser
    );
}

bool applyImuFifoHardware(TrackerSerialCommandContext& ctx,
                          Stream& out,
                          bool rebeginImu,
                          bool requestRecovery) {
    if (!ctx.config || !ctx.fifo || (rebeginImu && !ctx.lsm)) {
        tracker_serial_detail::printErr(out, "imu/fifo/config not available");
        return false;
    }

    if (ctx.streamState) ctx.streamState->mode = TrackerStreamMode::Off;
    if (ctx.logState) ctx.logState->mode = TrackerLogMode::Off;

    const uint64_t lastTimestampUs = ctx.fifo->stats().lastAssignedTimestampUs;
    if (rebeginImu && ctx.setSpiFrequency &&
        !ctx.setSpiFrequency(ctx.config->data.hardware.spiHz, ctx.setSpiFrequencyUser)) {
        tracker_serial_detail::printErr(out, "failed to apply SPI clock before IMU reconfigure");
        return false;
    }
    if (rebeginImu && !ctx.lsm->begin(ctx.config->makeLsmConfig())) {
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
        ctx.quality->resetStreamRecoveryState();
        ctx.quality->syncFifoStats(ctx.fifo->stats());
    }
    if (ctx.resetFifoRuntime) ctx.resetFifoRuntime(ctx.resetFifoRuntimeUser);
    if (requestRecovery) requestReconfigureRecovery(ctx, lastTimestampUs);

#if TRACKER_ENABLE_MAG_COMMANDS
    if (!rearmMagIfNeeded(ctx)) {
        tracker_serial_detail::printErr(out, "mag runtime re-arm failed");
        return false;
    }
#endif
    return true;
}

bool rollbackImuFifo(TrackerSerialCommandContext& ctx,
                     Stream& out,
                     const TrackerConfig& previous,
                     bool rebeginImu) {
    *ctx.config = previous;
    if (!applyImuFifoHardware(ctx, out, rebeginImu, true)) {
        tracker_serial_detail::printErr(out, "rollback failed; reboot required");
        return false;
    }
    out.println("# WARN runtime transaction rolled back to previous IMU/FIFO config");
    return true;
}

bool commitImuFifoCandidate(TrackerSerialCommandContext& ctx,
                            Stream& out,
                            TrackerConfig candidate,
                            bool save,
                            bool rebeginImu) {
    if (!ctx.config) {
        tracker_serial_detail::printErr(out, "config not available");
        return false;
    }

    candidate.sanitize();
    candidate.updateCrc();
    const TrackerConfig previous = *ctx.config;
    *ctx.config = candidate;

    if (!applyImuFifoHardware(ctx, out, rebeginImu, true)) {
        (void)rollbackImuFifo(ctx, out, previous, rebeginImu);
        return false;
    }

    if (save && !persistCandidate(ctx, out, *ctx.config)) {
        (void)rollbackImuFifo(ctx, out, previous, rebeginImu);
        (void)restorePersistedPrevious(ctx, out, previous);
        return false;
    }
    return true;
}

} // namespace

bool trackerSerialCommitFullHardwareConfig(TrackerSerialCommandContext& ctx,
                                           Stream& out,
                                           TrackerConfig candidate) {
    return commitImuFifoCandidate(ctx, out, candidate, false, true);
}

void trackerSerialPrintFifoTuning(Stream& out, const TrackerSerialCommandContext& ctx) {
    if (!ctx.config) {
        tracker_serial_detail::printErr(out, "config not available");
        return;
    }
    out.println("# FIFO TUNING");
    out.print("fifo_watermark_words="); out.println(ctx.config->data.fifo.watermarkWords);
    out.print("fifo_max_words_per_drain="); out.println(ctx.config->data.fifo.maxWordsPerDrain);
    out.print("fifo_max_drain_rounds_per_event="); out.println(ctx.config->data.fifo.maxDrainRoundsPerEvent);
    if (ctx.fifo) {
        const auto& stats = ctx.fifo->stats();
        out.print("fifo_configured="); out.println(ctx.fifo->isConfigured() ? "yes" : "no");
        out.print("fifo_max_unread_words_seen="); out.println(stats.maxUnreadWordsSeen);
        out.print("fifo_overrun_events="); out.println(stats.overrunEvents);
        out.print("fifo_full_events="); out.println(stats.fullEvents);
    }
}

bool trackerSerialCommitSpiFrequency(TrackerSerialCommandContext& ctx,
                                     Stream& out,
                                     uint32_t hz,
                                     bool save) {
    if (!ctx.config || !ctx.setSpiFrequency) {
        tracker_serial_detail::printErr(out, "SPI runtime/config not available");
        return false;
    }

    const TrackerConfig previous = *ctx.config;
    TrackerConfig candidate = previous;
    candidate.data.hardware.spiHz = hz;
    candidate.sanitize();
    candidate.updateCrc();
    *ctx.config = candidate;

    if (!ctx.setSpiFrequency(ctx.config->data.hardware.spiHz, ctx.setSpiFrequencyUser)) {
        *ctx.config = previous;
        (void)ctx.setSpiFrequency(previous.data.hardware.spiHz, ctx.setSpiFrequencyUser);
        tracker_serial_detail::printErr(out, "failed to apply SPI clock");
        return false;
    }

    if (save && !persistCandidate(ctx, out, *ctx.config)) {
        *ctx.config = previous;
        (void)ctx.setSpiFrequency(previous.data.hardware.spiHz, ctx.setSpiFrequencyUser);
        (void)restorePersistedPrevious(ctx, out, previous);
        out.println("# WARN SPI runtime setting rolled back");
        return false;
    }
    return true;
}

bool trackerSerialCommitFifoWatermark(TrackerSerialCommandContext& ctx,
                                      Stream& out,
                                      uint8_t watermarkWords,
                                      bool save) {
    if (!ctx.config) {
        tracker_serial_detail::printErr(out, "config not available");
        return false;
    }
    TrackerConfig candidate = *ctx.config;
    candidate.data.fifo.watermarkWords = watermarkWords;
    return commitImuFifoCandidate(ctx, out, candidate, save, false);
}

bool trackerSerialCommitFifoDrain(TrackerSerialCommandContext& ctx,
                                  Stream& out,
                                  uint16_t maxWordsPerDrain,
                                  uint8_t maxDrainRoundsPerEvent,
                                  bool save) {
    if (!ctx.config) {
        tracker_serial_detail::printErr(out, "config not available");
        return false;
    }
    TrackerConfig candidate = *ctx.config;
    candidate.data.fifo.maxWordsPerDrain = maxWordsPerDrain;
    candidate.data.fifo.maxDrainRoundsPerEvent = maxDrainRoundsPerEvent;
    candidate.sanitize();
    candidate.updateCrc();

    if (save && !persistCandidate(ctx, out, candidate)) return false;
    *ctx.config = candidate;
    return true;
}

bool trackerSerialCommitImuRate(TrackerSerialCommandContext& ctx,
                                Stream& out,
                                Lsm6dsv::Odr odr,
                                bool save) {
    if (!ctx.config) {
        tracker_serial_detail::printErr(out, "config not available");
        return false;
    }
    TrackerConfig candidate = *ctx.config;
    candidate.data.imu.imuOdr = odr;
    candidate.data.fifo.accelBdr = odr;
    candidate.data.fifo.gyroBdr = odr;
    candidate.data.fifo.samplePeriodUsOverride = 0.0f;
    return commitImuFifoCandidate(ctx, out, candidate, save, true);
}

bool trackerSerialDispatchBasicFifoCommand(TrackerSerialCommandContext& ctx,
                                           int argc,
                                           char** argv) {
    if (argc <= 0 || !argv || !argv[0]) return false;
    if (!is(argv[0], "fifo")) return false;

    Stream& out = ctx.io ? *ctx.io : Serial;
    if (argc < 2 || is(argv[1], "status")) {
        trackerSerialPrintFifoTuning(out, ctx);
        return true;
    }

    if (is(argv[1], "watermark") || is(argv[1], "wm")) {
        if (argc < 3) {
            tracker_serial_detail::printErr(out, "usage: fifo watermark <1..255> [save]");
            return true;
        }
        uint32_t words = 0;
        bool save = false;
        if (!tracker_serial_detail::parseU32(argv[2], words) || words < 1u || words > 255u) {
            tracker_serial_detail::printErr(out, "invalid watermark; expected 1..255 words");
            return true;
        }
        if (!validateSaveArg(argc, argv, 3, save, out)) return true;
        if (!trackerSerialCommitFifoWatermark(ctx, out, static_cast<uint8_t>(words), save)) return true;
        out.print("# OK fifo watermark set words="); out.print(words);
        out.println(save ? " saved=yes" : " saved=no");
        return true;
    }

    if (is(argv[1], "drain")) {
        if (argc < 4) {
            tracker_serial_detail::printErr(out, "usage: fifo drain <16..4096> <1..32> [save]");
            return true;
        }
        uint32_t maxWords = 0;
        uint32_t rounds = 0;
        bool save = false;
        if (!tracker_serial_detail::parseU32(argv[2], maxWords) || maxWords < 16u || maxWords > 4096u) {
            tracker_serial_detail::printErr(out, "invalid max_words_per_drain; expected 16..4096");
            return true;
        }
        if (!tracker_serial_detail::parseU32(argv[3], rounds) || rounds < 1u || rounds > 32u) {
            tracker_serial_detail::printErr(out, "invalid rounds_per_event; expected 1..32");
            return true;
        }
        if (!validateSaveArg(argc, argv, 4, save, out)) return true;
        if (!trackerSerialCommitFifoDrain(ctx, out, static_cast<uint16_t>(maxWords), static_cast<uint8_t>(rounds), save)) return true;
        out.print("# OK fifo drain set max_words="); out.print(maxWords);
        out.print(" rounds="); out.print(rounds);
        out.println(save ? " saved=yes" : " saved=no");
        return true;
    }

    tracker_serial_detail::printErr(out, "usage: fifo status|watermark|drain");
    return true;
}

} // namespace tracker
