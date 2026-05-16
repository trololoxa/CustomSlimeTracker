#include "serial/tracker_config_commands.hpp"

#include <Arduino.h>

#include "config/tracker_config.hpp"
#include "sensor/fifo_calibrations.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

Stream& trackerSerialConfigStream(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

bool trackerSerialConfigIs(const char* a, const char* b) {
    return tracker_serial_detail::eqIgnoreCase(a, b);
}

void trackerSerialApplyConfigToRuntime(TrackerSerialCommandContext& ctx) {
    if (!ctx.config) return;

    if (ctx.imuCal) ctx.config->applyToImuCalibration(*ctx.imuCal);
    if (ctx.gyroTempComp) ctx.config->applyToGyroTempComp(*ctx.gyroTempComp);
    if (ctx.ahrs) ctx.ahrs->setConfig(ctx.config->makeAhrsConfig());

    if (ctx.quality) {
        ctx.quality->setConfig(ctx.config->makeQualityConfig());
        ctx.quality->reset();
        if (ctx.fifo) ctx.quality->syncFifoStats(ctx.fifo->stats());
    }

    if (ctx.streamState) {
        ctx.streamState->rateHz = ctx.config->data.output.outputRateHz;
        ctx.streamState->mode = ctx.config->data.output.quaternionOutputEnabled
            ? TrackerStreamMode::Quat
            : TrackerStreamMode::Off;
    }

    if (ctx.setSpiFrequency) ctx.setSpiFrequency(ctx.config->data.hardware.spiHz, ctx.setSpiFrequencyUser);
    if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
}

void trackerSerialCaptureRuntimeToConfig(TrackerSerialCommandContext& ctx) {
    if (!ctx.config) return;

    if (ctx.imuCal) ctx.config->captureFromImuCalibration(*ctx.imuCal);
    if (ctx.gyroTempComp) ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
    if (ctx.accelCalRunner && ctx.accelCalRunner->calibration().result().valid) {
        ctx.config->captureFromAccelCalibrationQuality(ctx.accelCalRunner->calibration(), millis());
    }

    if (ctx.streamState) {
        ctx.config->data.output.outputRateHz = ctx.streamState->rateHz;
        ctx.config->data.output.quaternionOutputEnabled = (ctx.streamState->mode == TrackerStreamMode::Quat);
        ctx.config->data.output.serialDebugEnabled = (ctx.streamState->mode == TrackerStreamMode::Debug);
        ctx.config->data.output.packetFormat = 0;
    }

    ctx.config->updateCrc();
}

void trackerSerialPrintConfigNvsInfo(Stream& out, TrackerConfigStore& store) {
    TrackerConfigNvsInfo info;
    store.inspect(info);

    out.println("# CONFIG NVS");
    out.print("begin_ok="); out.println(info.beginOk ? "yes" : "no");
    out.print("exists="); out.println(info.exists ? "yes" : "no");
    out.print("stored_len="); out.println(static_cast<uint32_t>(info.storedLen));
    out.print("expected_len="); out.println(static_cast<uint32_t>(info.expectedLen));
    out.print("header_readable="); out.println(info.headerReadable ? "yes" : "no");
    out.print("stored_magic=0x"); out.println(info.storedMagic, HEX);
    out.print("stored_version="); out.println(info.storedVersion);
    out.print("stored_size="); out.println(info.storedSize);
    out.print("stored_crc=0x"); out.println(info.storedCrc, HEX);
    out.print("full_readable="); out.println(info.fullReadable ? "yes" : "no");
    out.print("valid="); out.println(info.valid ? "yes" : "no");
    out.print("error="); out.println(TrackerConfigStore::errorName(info.error));
}

void trackerSerialDispatchConfigCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = trackerSerialConfigStream(ctx);

    if (!ctx.config) {
        tracker_serial_detail::printErr(out, "config not available");
        return;
    }
    if (argc < 2) {
        tracker_serial_detail::printErr(out, "usage: config print|load|save|defaults|erase|crc|nvs|spi");
        return;
    }

    if (trackerSerialConfigIs(argv[1], "print")) {
        printTrackerConfigSummary(out, *ctx.config);
        return;
    }

    if (trackerSerialConfigIs(argv[1], "crc")) {
        out.print("# config_valid="); out.println(ctx.config->validate() ? "yes" : "no");
        out.print("# stored_crc=0x"); out.println(ctx.config->data.crc32, HEX);
        out.print("# computed_crc=0x"); out.println(ctx.config->computeCrc(), HEX);
        out.print("# version="); out.println(ctx.config->data.version);
        out.print("# size="); out.println(ctx.config->data.size);
        return;
    }

    if (trackerSerialConfigIs(argv[1], "nvs")) {
        if (!ctx.configStore) {
            tracker_serial_detail::printErr(out, "config store not available");
            return;
        }
        trackerSerialPrintConfigNvsInfo(out, *ctx.configStore);
        return;
    }

    if (trackerSerialConfigIs(argv[1], "spi")) {
        if (argc < 3) {
            tracker_serial_detail::printErr(out, "usage: config spi <hz> [save]");
            return;
        }

        uint32_t hz = 0;
        if (!tracker_serial_detail::parseU32(argv[2], hz) ||
            hz < tracker_config_detail::MIN_SPI_HZ ||
            hz > tracker_config_detail::MAX_SPI_HZ) {
            tracker_serial_detail::printErr(out, "invalid SPI Hz; expected 100000..10000000");
            return;
        }

        const bool save = argc >= 4 && trackerSerialConfigIs(argv[3], "save");
        if (argc >= 4 && !save) {
            tracker_serial_detail::printErr(out, "usage: config spi <hz> [save]");
            return;
        }

        ctx.config->data.hardware.spiHz = hz;
        ctx.config->sanitize();
        ctx.config->updateCrc();

        if (ctx.setSpiFrequency && !ctx.setSpiFrequency(ctx.config->data.hardware.spiHz, ctx.setSpiFrequencyUser)) {
            tracker_serial_detail::printErr(out, "failed to apply SPI clock");
            return;
        }

        if (save) {
            if (!ctx.configStore) {
                tracker_serial_detail::printErr(out, "config store not available; changed in RAM only");
                return;
            }
            if (!ctx.configStore->save(*ctx.config)) {
                out.print("# ERR config save failed: ");
                out.println(ctx.configStore->lastErrorName());
                return;
            }
        }

        tracker_serial_detail::printOk(out, save ? "SPI clock set and saved" : "SPI clock set");
        out.print("# spi_hz="); out.println(ctx.config->data.hardware.spiHz);
        return;
    }

    if (trackerSerialConfigIs(argv[1], "defaults")) {
        ctx.config->resetDefaults();
        trackerSerialApplyConfigToRuntime(ctx);
        tracker_serial_detail::printOk(out, "config defaults loaded into RAM");
        return;
    }

    if (!ctx.configStore) {
        tracker_serial_detail::printErr(out, "config store not available");
        return;
    }

    if (trackerSerialConfigIs(argv[1], "load")) {
        if (ctx.configStore->load(*ctx.config)) {
            trackerSerialApplyConfigToRuntime(ctx);
            tracker_serial_detail::printOk(out, "config loaded from NVS");
        } else {
            out.print("# ERR config load failed: ");
            out.println(ctx.configStore->lastErrorName());
        }
        return;
    }

    if (trackerSerialConfigIs(argv[1], "save")) {
        trackerSerialCaptureRuntimeToConfig(ctx);
        if (ctx.configStore->save(*ctx.config)) {
            tracker_serial_detail::printOk(out, "config saved to NVS");
        } else {
            out.print("# ERR config save failed: ");
            out.println(ctx.configStore->lastErrorName());
        }
        return;
    }

    if (trackerSerialConfigIs(argv[1], "erase")) {
        if (ctx.configStore->erase()) {
            tracker_serial_detail::printOk(out, "config erased from NVS");
        } else {
            out.print("# ERR config erase failed: ");
            out.println(ctx.configStore->lastErrorName());
        }
        return;
    }

    tracker_serial_detail::printErr(out, "unknown config command");
}

} // namespace tracker
