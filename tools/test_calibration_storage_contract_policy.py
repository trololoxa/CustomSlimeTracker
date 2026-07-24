#!/usr/bin/env python3
"""Guard the 0021d storage/model/promotion contract against regression."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing calibration-storage contract: {label}: {needle}")


def forbid(text: str, needle: str, label: str) -> None:
    if needle in text:
        raise SystemExit(f"forbidden calibration-storage contract: {label}: {needle}")


def main() -> int:
    schema = (ROOT / "src/config/tracker_config_storage.hpp").read_text(encoding="utf-8")
    storage = (ROOT / "src/config/tracker_config_storage.cpp").read_text(encoding="utf-8")
    runtime = (ROOT / "src/config/tracker_config_runtime.cpp").read_text(encoding="utf-8")
    runtime_header = (ROOT / "src/config/tracker_config_runtime.hpp").read_text(encoding="utf-8")
    store = (ROOT / "src/config/tracker_config_store.cpp").read_text(encoding="utf-8")
    store_header = (ROOT / "src/config/tracker_config_store.hpp").read_text(encoding="utf-8")
    cfg_cli = (ROOT / "src/serial/tracker_config_commands.cpp").read_text(encoding="utf-8")
    cal_cli = (ROOT / "src/serial/tracker_calibration_commands.cpp").read_text(encoding="utf-8")
    mag_runtime = (ROOT / "src/runtime/mag_runtime_controller.cpp").read_text(encoding="utf-8")
    mag_hooks = (ROOT / "src/app/hooks/tracker_app_mag_hooks.hpp").read_text(encoding="utf-8")
    app = (ROOT / "src/app/tracker_app.cpp").read_text(encoding="utf-8")
    version = (ROOT / "src/build_config/firmware_feature_version.hpp").read_text(encoding="utf-8")
    features = (ROOT / "src/build_config/feature_flags.hpp").read_text(encoding="utf-8")
    scenarios = (ROOT / "tests/native/test_storage_real_scenarios.cpp").read_text(encoding="utf-8")

    require(schema, "TrackerConfigSlotRecord", "full active slot record")
    require(schema, "TrackerConfigSelectorRecord", "selector record")
    require(schema, "TrackerConfigCommitRecord", "per-slot commit marker")
    require(schema, "COMMIT_MAGIC", "commit-marker magic")
    require(schema, "TrackerSensorSignature", "sensor signature")
    require(schema, "TrackerCalibrationCandidateRecord", "candidate record")
    require(schema, "activeCalibrationRevisionAtCreation", "calibration-revision freshness")
    require(schema, "METADATA_REVISION_CANDIDATE_VERSION = 2", "v2 candidate compatibility")
    require(schema, "CANDIDATE_VERSION = 3", "model-only v3 candidates")
    require(schema, "AlreadyPromoted", "terminal promoted candidate state")
    require(schema, "minCandidateWriteIntervalMs", "wear interval")
    require(storage, "calibrationModelRevision", "model-only calibration revision")
    require(storage, "calibrationRevisionLegacyV2", "v2 metadata revision compatibility")
    require(storage, "trackerCalibrationModelEqual", "model/evidence separation")
    require(storage, "trackerCalibrationEvidenceEqual", "calibration evidence equality")
    require(storage, "minTrustNorm = candidate.data.magCal.minTrustNorm", "mag trust-bound promotion")
    require(storage, "trackerCalibrationQualitySetProvenance", "active provenance encoding")

    require(store, "writeSlotVerified(target, record)", "slot read-back before selection")
    require(store, "removeCommitMarker(target)", "prepared slot marker invalidation")
    require(store, "writeSelectorVerified(nextSelector)", "selector commit")
    require(store, "writeCommitVerified(target, generation)", "slot commit marker")
    require(store, "activeHasDurableV2Commit", "no-op requires durable v2 active")
    require(store, "noOpSaveCount_", "content-addressed no-op save")
    require(store, "migrateLegacy", "legacy migration")
    require(store, "clearUncommittedActiveArtifactsForLegacyRecovery", "interrupted migration recovery")
    require(store, "TrackerConfigError::StorageDegraded", "degraded write latch")
    require(store, "TrackerConfigError::ApplyPending", "authoritative apply latch")
    require(store, "confirmAuthoritativeConfigApplied", "explicit recovery confirmation")
    require(store, "prepareCandidatePromotion", "two-phase promotion prepare")
    require(store, "commitPreparedPromotion", "two-phase promotion commit")
    require(store, "StaleActiveGeneration", "stale calibration gate")
    require(store, "CandidateAlreadyPromoted", "repeat-promotion hard block")
    require(store, "promotedMetadataRetry", "promoted metadata retry")
    require(store, "invalidatePreparedSlot", "prepared slot cleanup")
    require(store, "trackerComposeCalibrationCandidate", "calibration-only candidate composition")
    require(store, "TrackerConfigError::RuntimeCalibrationDiverged", "unsaved runtime calibration protection")
    require(store, "runtimeCalibration", "promotion receives current runtime calibration")

    require(cfg_cli, "configStore->verify", "read-only config verify")
    require(cfg_cli, "ctx.config->sanitize();", "config save normalizes the authoritative config")
    config_save_start = cfg_cli.index('if (trackerSerialConfigIs(argv[1], "save"))')
    config_save_end = cfg_cli.index('if (trackerSerialConfigIs(argv[1], "erase"))', config_save_start)
    config_save = cfg_cli[config_save_start:config_save_end]
    forbid(config_save, "trackerSerialCaptureRuntimeToConfig", "generic config save must not manufacture calibration events")
    forbid(cfg_cli, "trackerSerialCaptureConfigurationRuntimeToConfig", "transient stream runtime must not be persisted")
    require(cfg_cli, "markAuthoritativeConfigApplyFailed", "failed hardware apply latch")
    require(cfg_cli, "confirmAuthoritativeConfigApplied", "successful hardware apply confirmation")
    require(cfg_cli, "runtimeBiasReset", "full config apply resets runtime bias")
    require(cfg_cli, "requestSensorInfoRefresh", "full config apply resynchronizes SensorInfo")
    require(cal_cli, "no sensor/FIFO restart", "calibration-only runtime apply")
    require(cal_cli, "trackerApplyCalibrationCandidateToConfig", "field-wise runtime promotion")
    require(cal_cli, "clearMagHeadingReference", "promotion clears old magnetic heading reference")
    require(cal_cli, "resetMagYawCorrection", "promotion resets yaw correction state")
    require(cal_cli, "selector commit failed; runtime rolled back", "selector failure rollback")
    require(cal_cli, "captureGyroFromImuCalibration", "gyro-only save ownership")
    require(cal_cli, "captureAccelFromImuCalibration", "accel-only save ownership")
    require(cal_cli, "force, previous.get()", "runtime calibration snapshot passed to promotion")
    require(cal_cli, "accelRunnerMatchesConfig", "stale accel runner evidence guard")
    require(cal_cli, "requestSensorInfoRefresh", "rest-calibration SensorInfo refresh")
    require(app, "confirmAuthoritativeConfigApplied", "boot apply confirmation")
    require(runtime_header, "clearAllCalibrationPreservingPolicy", "calibration-only clear API")
    require(runtime_header, "captureFromGyroTempCompUpdate", "explicit temperature-model event API")
    require(runtime, "previousUpdatedUptimeMs", "temperature snapshot preserves event timestamp")
    require(runtime, "!data.gyroCal.biasValid && data.gyroCal.tempCompValid", "impossible temp model normalization")
    require(runtime, "invalidMagFieldModel", "nonfinite magnetic calibration invalidation")
    require(runtime, "!data.magCal.axisAlignmentValid", "mag yaw requires axis calibration")
    require(runtime, "tempComp.config().enabled", "temp policy survives invalid-model snapshot")
    require(runtime, "tempModelSampleCount =", "temperature fit sample-count persistence")
    require(runtime, "clearMagCalibrationPreservingDriver", "mag clear preserves driver policy")
    require(cal_cli, "captureFromAccelCalibrationQuality(ctx.accelCalRunner->calibration(), millis())", "accel evidence updated only at compute event")
    forbid(cal_cli, "captureFromAccelCalibrationQualitySnapshot", "stale accel runner evidence snapshot")
    require(cal_cli, "temperature slope requires a valid gyro bias/reference model", "temperature slope precondition")
    require(cal_cli, "candidateTempComp.temperatureModelValid()", "temperature slope postcondition")
    require(cal_cli, "clearMagHeadingReference", "calibration clear/promotion invalidates heading reference")
    require(mag_runtime, "requires valid accel, field and axis calibration", "honest mag-yaw enable precondition")
    require(mag_runtime, "candidateConfig.data.magYaw.applyEnabled != enabled", "mag-yaw sanitize result verification")
    require(mag_runtime, "hardwareAlreadyMatches", "mag runtime state-aware no-op")
    require(mag_runtime, "if (changed) resetYawCorrectionRuntime();", "yaw no-op preserves correction epoch")
    require(mag_hooks, "advertisedBefore != g_config.data.magCal.driverEnabled", "mag driver SensorInfo refresh only on advertised change")
    require(mag_hooks, "requestSensorInfoRefresh", "mag driver SensorInfo refresh")
    require((ROOT / "src/runtime/gyro_temp_static_fit.cpp").read_text(encoding="utf-8"),
            "captureFromGyroTempCompUpdate(candidateTempComp, millis(), inlierSamples)",
            "temperature fit records accepted sample count")

    require(scenarios, "emptyNvs", "empty NVS scenario")
    require(scenarios, "corruptedNvs", "corrupt NVS scenario")
    require(scenarios, "transientReadFailure", "transient read scenario")
    require(scenarios, "firstSavePowerLossWindow", "first-save interruption scenario")
    require(scenarios, "noOpAndCandidate", "no-op/candidate scenario")

    require(version, '"c3-6dsv-calibration-epoch-field-safe"', "0021e feature identity")
    require(features, "TRACKER_ENABLE_CALIBRATION_CANDIDATES", "candidate profile gate")
    require(store_header, "#if TRACKER_ENABLE_CALIBRATION_CANDIDATES", "Slim RAM candidate removal")

    print("# calibration_storage_contract_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
