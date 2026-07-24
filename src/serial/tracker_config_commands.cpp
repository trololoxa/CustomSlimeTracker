#include "serial/tracker_config_commands.hpp"

#include <Arduino.h>

#include "config/tracker_config.hpp"
#include "runtime/runtime_gyro_bias_controller.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "sensor/fifo_calibrations.hpp"
#include "serial/tracker_fifo_config_control.hpp"
#include "serial/tracker_serial_context.hpp"
#include "serial/tracker_calibration_commands.hpp"

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

    // Full config replacement invalidates adaptive state learned against the
    // previous calibration model. Keep all dependent runtime owners in the
    // same epoch as the newly applied config.
    if (ctx.runtimeBias) runtimeBiasReset(*ctx.runtimeBias);
    trackerSerialResetCalibrationWorkspaces(ctx);
    if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
    if (ctx.clearMagHeadingReference) {
        ctx.clearMagHeadingReference(ctx.clearMagHeadingReferenceUser);
    }
    if (ctx.resetMagYawCorrection) {
        ctx.resetMagYawCorrection(ctx.resetMagYawCorrectionUser);
    }
    // SensorInfo advertises rest-calibration and sensor capabilities. A full
    // config replacement may change either, so resynchronize the acknowledged
    // server view after the runtime has accepted the config.
    if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();
}

void trackerSerialCaptureRuntimeToConfig(TrackerSerialCommandContext& ctx, TrackerConfig& target) {
    if (ctx.imuCal) target.captureFromImuCalibration(*ctx.imuCal);
    if (ctx.gyroTempComp) target.captureFromGyroTempComp(*ctx.gyroTempComp);
    // Runtime stream state is transient (SlimeVR and FIFO reconfigure may
    // temporarily force it Off). User-facing output commands update the config
    // at the moment policy changes, so persistence must not snapshot runtime
    // stream state back into the authoritative blob.
    target.sanitize();
    target.updateCrc();
}

void trackerSerialCaptureRuntimeToConfig(TrackerSerialCommandContext& ctx) {
    if (!ctx.config) return;
    trackerSerialCaptureRuntimeToConfig(ctx, *ctx.config);
}

void trackerSerialPrintConfigNvsInfo(Stream& out, TrackerConfigStore& store) {
    TrackerConfigNvsInfo info;
    store.inspect(info);

    out.println("# CONFIG STORAGE");
    out.print("load_status="); out.println(store.lastLoadStatusName());
    out.print("load_error="); out.println(TrackerConfigStore::errorName(store.lastLoadError()));
    out.print("storage_degraded=");
    out.println(info.storage.storageDegradedLatched ? "yes" : "no");
    out.print("authoritative_apply_pending=");
    out.println(info.storage.authoritativeApplyPending ? "yes" : "no");
    out.print("begin_ok="); out.println(info.beginOk ? "yes" : "no");
    out.print("active_exists="); out.println(info.exists ? "yes" : "no");
    out.print("active_valid="); out.println(info.valid ? "yes" : "no");
    out.print("active_slot="); out.println(trackerConfigSlotName(info.storage.selectedSlot));
    out.print("active_generation="); out.println(info.storage.selectedGeneration);
    out.print("selector_exists="); out.println(info.storage.selectorExists ? "yes" : "no");
    out.print("selector_valid="); out.println(info.storage.selectorValid ? "yes" : "no");
    out.print("selected_by_fallback="); out.println(info.storage.selectedByFallback ? "yes" : "no");

    out.print("slot_a_exists="); out.println(info.storage.slotA.exists ? "yes" : "no");
    out.print("slot_a_readable="); out.println(info.storage.slotA.readable ? "yes" : "no");
    out.print("slot_a_valid="); out.println(info.storage.slotA.valid ? "yes" : "no");
    out.print("slot_a_legacy_committed="); out.println(info.storage.slotA.legacyCommitted ? "yes" : "no");
    out.print("slot_a_commit_marker_exists="); out.println(info.storage.slotA.commitMarkerExists ? "yes" : "no");
    out.print("slot_a_commit_marker_valid="); out.println(info.storage.slotA.commitMarkerValid ? "yes" : "no");
    out.print("slot_a_generation="); out.println(info.storage.slotA.generation);
    out.print("slot_a_signature_crc=0x"); out.println(info.storage.slotA.signature.crc32, HEX);
    out.print("slot_a_quality="); out.println(info.storage.slotA.quality.overallScore, 6);
    out.print("slot_a_provenance=");
    out.println(trackerCalibrationProvenanceName(
        trackerCalibrationQualityProvenance(info.storage.slotA.quality)));

    out.print("slot_b_exists="); out.println(info.storage.slotB.exists ? "yes" : "no");
    out.print("slot_b_readable="); out.println(info.storage.slotB.readable ? "yes" : "no");
    out.print("slot_b_valid="); out.println(info.storage.slotB.valid ? "yes" : "no");
    out.print("slot_b_legacy_committed="); out.println(info.storage.slotB.legacyCommitted ? "yes" : "no");
    out.print("slot_b_commit_marker_exists="); out.println(info.storage.slotB.commitMarkerExists ? "yes" : "no");
    out.print("slot_b_commit_marker_valid="); out.println(info.storage.slotB.commitMarkerValid ? "yes" : "no");
    out.print("slot_b_generation="); out.println(info.storage.slotB.generation);
    out.print("slot_b_signature_crc=0x"); out.println(info.storage.slotB.signature.crc32, HEX);
    out.print("slot_b_quality="); out.println(info.storage.slotB.quality.overallScore, 6);
    out.print("slot_b_provenance=");
    out.println(trackerCalibrationProvenanceName(
        trackerCalibrationQualityProvenance(info.storage.slotB.quality)));

    out.print("legacy_exists="); out.println(info.storage.legacyExists ? "yes" : "no");
    out.print("legacy_valid="); out.println(info.storage.legacyValid ? "yes" : "no");
    out.print("candidate_exists="); out.println(info.storage.candidate.exists ? "yes" : "no");
    out.print("candidate_valid="); out.println(info.storage.candidate.valid ? "yes" : "no");
    out.print("candidate_dirty_ram="); out.println(info.storage.candidate.dirtyInRam ? "yes" : "no");
    out.print("candidate_generation="); out.println(info.storage.candidate.generation);
    out.print("candidate_comparison=");
    out.println(trackerCalibrationComparisonName(info.storage.candidate.metadata.lastComparison));
    out.print("candidate_comparison_flags=0x");
    out.println(info.storage.candidate.metadata.comparisonFlags, HEX);

    out.print("successful_active_writes="); out.println(info.storage.successfulActiveWrites);
    out.print("successful_migrations="); out.println(info.storage.successfulMigrations);
    out.print("successful_promotions="); out.println(info.storage.successfulPromotions);
    out.print("load_fallbacks="); out.println(info.storage.loadFallbacks);
    out.print("selector_repair_failures="); out.println(info.storage.selectorRepairFailures);
    out.print("candidate_stage_count="); out.println(info.storage.candidateStageCount);
    out.print("candidate_flush_count="); out.println(info.storage.candidateFlushCount);
    out.print("candidate_flush_throttled="); out.println(info.storage.candidateFlushThrottled);
    out.print("candidate_rejected_count="); out.println(info.storage.candidateRejectedCount);
    out.print("legacy_cleanup_pending="); out.println(info.storage.legacyCleanupPending ? "yes" : "no");
    out.print("legacy_cleanup_failures="); out.println(info.storage.legacyCleanupFailures);
    out.print("commit_uncertain_count="); out.println(info.storage.commitUncertainCount);
    out.print("commit_marker_repair_failures="); out.println(info.storage.commitMarkerRepairFailures);
    out.print("noop_save_count="); out.println(info.storage.noOpSaveCount);
    out.print("degraded_write_blocks="); out.println(info.storage.degradedWriteBlocks);
    out.print("apply_pending_write_blocks="); out.println(info.storage.applyPendingWriteBlocks);
    out.print("candidate_promotion_state_writes="); out.println(info.storage.candidatePromotionStateWrites);
    out.print("candidate_promotion_state_write_failures="); out.println(info.storage.candidatePromotionStateWriteFailures);
    out.print("error="); out.println(TrackerConfigStore::errorName(info.error));
}

void trackerSerialDispatchConfigCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = trackerSerialConfigStream(ctx);

    if (!ctx.config) {
        tracker_serial_detail::printErr(out, "config not available");
        return;
    }
    if (argc < 2) {
        tracker_serial_detail::printErr(out, "usage: config print|load|save|defaults|erase|crc|nvs|slots|verify|migrate|spi|fifo");
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

    if (trackerSerialConfigIs(argv[1], "nvs") || trackerSerialConfigIs(argv[1], "slots")) {
        if (!ctx.configStore) {
            tracker_serial_detail::printErr(out, "config store not available");
            return;
        }
        trackerSerialPrintConfigNvsInfo(out, *ctx.configStore);
        return;
    }

    if (trackerSerialConfigIs(argv[1], "spi")) {
        if (argc < 3) {
            out.print("# spi_hz="); out.println(ctx.config->data.hardware.spiHz);
            out.println("# usage: config spi <hz> [save]");
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
        if (!trackerSerialCommitSpiFrequency(ctx, out, hz, save)) return;

        tracker_serial_detail::printOk(out, save ? "SPI clock set and saved" : "SPI clock set");
        out.print("# spi_hz="); out.println(ctx.config->data.hardware.spiHz);
        return;
    }

    if (trackerSerialConfigIs(argv[1], "fifo")) {
        if (argc < 3 || trackerSerialConfigIs(argv[2], "status")) {
            trackerSerialPrintFifoTuning(out, ctx);
            return;
        }

        char* fifoArgv[8] = {};
        fifoArgv[0] = const_cast<char*>("fifo");
        int fifoArgc = 1;
        for (int k = 2; k < argc && fifoArgc < 8; ++k) {
            fifoArgv[fifoArgc++] = argv[k];
        }
        (void)trackerSerialDispatchBasicFifoCommand(ctx, fifoArgc, fifoArgv);
        return;
    }

    if (trackerSerialConfigIs(argv[1], "defaults")) {
        TrackerConfig candidate;
        candidate.resetDefaults();
        if (!trackerSerialCommitFullHardwareConfig(ctx, out, candidate)) {
            tracker_serial_detail::printErr(out, "config defaults hardware apply failed; previous config restored");
            return;
        }
        trackerSerialApplyConfigToRuntime(ctx);
        tracker_serial_detail::printOk(out, "config defaults loaded into RAM and hardware");
        return;
    }

    if (!ctx.configStore) {
        tracker_serial_detail::printErr(out, "config store not available");
        return;
    }

    if (trackerSerialConfigIs(argv[1], "verify")) {
        TrackerConfig verified;
        if (!ctx.configStore->verify(verified)) {
            out.print("# ERR config storage verify failed: ");
            out.println(ctx.configStore->lastErrorName());
            return;
        }
        tracker_serial_detail::printOk(out, "active config slot and selector verified");
        trackerSerialPrintConfigNvsInfo(out, *ctx.configStore);
        return;
    }

    if (trackerSerialConfigIs(argv[1], "migrate")) {
        TrackerConfigStorageInfo storage;
        if (!ctx.configStore->inspectStorage(storage)) {
            out.print("# ERR config storage inspect failed: ");
            out.println(ctx.configStore->lastErrorName());
            return;
        }
        if (!storage.legacyExists) {
            tracker_serial_detail::printOk(out, "no legacy config remains; dual-slot storage already active");
            return;
        }
        if (!ctx.configStore->migrateLegacy()) {
            out.print("# ERR legacy config migration failed: ");
            out.println(ctx.configStore->lastErrorName());
            return;
        }
        tracker_serial_detail::printOk(out, "legacy config migrated to dual-slot storage");
        return;
    }

    if (trackerSerialConfigIs(argv[1], "load")) {
        TrackerConfig candidate;
        if (!ctx.configStore->load(candidate)) {
            out.print("# ERR config load failed: ");
            out.println(ctx.configStore->lastErrorName());
            return;
        }
        if (!trackerSerialCommitFullHardwareConfig(ctx, out, candidate)) {
            ctx.configStore->markAuthoritativeConfigApplyFailed();
            tracker_serial_detail::printErr(out, "loaded config hardware apply failed; previous config restored; persistent writes remain blocked");
            return;
        }
        trackerSerialApplyConfigToRuntime(ctx);
        ctx.configStore->confirmAuthoritativeConfigApplied();
        tracker_serial_detail::printOk(out, "config loaded from NVS and applied to hardware");
        return;
    }

    if (trackerSerialConfigIs(argv[1], "save")) {
        ctx.config->sanitize();
        ctx.config->updateCrc();
        if (ctx.configStore->save(*ctx.config, TrackerCalibrationProvenance::Manual)) {
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
