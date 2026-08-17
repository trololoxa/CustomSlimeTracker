#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def require(text: str, needle: str, description: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {description}: {needle}")


def forbid(text: str, needle: str, description: str) -> None:
    if needle in text:
        raise SystemExit(f"forbidden {description}: {needle}")


def main() -> int:
    controller = (ROOT / "src/runtime/mag_runtime_controller.cpp").read_text(encoding="utf-8")
    controller_h = (ROOT / "src/runtime/mag_runtime_controller.hpp").read_text(encoding="utf-8")
    reliability = (ROOT / "src/sensor/mag_field_reliability.cpp").read_text(encoding="utf-8")
    reliability_h = (ROOT / "src/sensor/mag_field_reliability.hpp").read_text(encoding="utf-8")
    axis = (ROOT / "src/sensor/mag_axis_alignment.cpp").read_text(encoding="utf-8")
    axis_h = (ROOT / "src/sensor/mag_axis_alignment.hpp").read_text(encoding="utf-8")
    yaw = (ROOT / "src/sensor/mag_yaw_correction.cpp").read_text(encoding="utf-8")
    setup = (ROOT / "src/serial/tracker_setup_commands.cpp").read_text(encoding="utf-8")
    mag_cli = (ROOT / "src/serial/tracker_mag_commands.cpp").read_text(encoding="utf-8")
    store = (ROOT / "src/config/tracker_config_store.cpp").read_text(encoding="utf-8")
    storage_h = (ROOT / "src/config/tracker_config_storage.hpp").read_text(encoding="utf-8")
    app = (ROOT / "src/app/tracker_app.cpp").read_text(encoding="utf-8")
    app_hooks = (ROOT / "src/app/hooks/tracker_app_mag_hooks.hpp").read_text(encoding="utf-8")
    runner = (ROOT / "tools/run_standalone_tests.py").read_text(encoding="utf-8")
    machine = (ROOT / "src/runtime/machine_log_runtime.cpp").read_text(encoding="utf-8")

    require(controller, "fieldReliability->update", "runtime field reliability integration")
    require(controller, "serviceDeferred()", "deferred solver/storage service")
    require(controller, "solvePending = true", "sample-path solve deferral")
    require(controller, "stagePending = true", "separate stage deferral")
    require(controller, "candidateExists(candidateExists)", "lightweight candidate-slot query")
    require(controller, "stageCandidate(candidate, metadata, nowMs)", "candidate-only background staging")
    require(controller, "ALIGNMENT_MEASURED", "measured alignment quality ownership")
    require(controller, "result.improvesActive", "validation-set active comparison gate")
    forbid(controller, "inspectStorage(", "full storage scan in magnetic runtime")
    forbid(controller, "flushCandidate(", "automatic candidate persistence")
    require(controller, "calibrationCollector->active()", "background pause during hard/soft calibration")
    require(controller, "lastImuTimestampUs", "gyro/magnetometer timestamp coherence gate")

    require(app, "updateMagDeferredRuntime", "main-loop deferred service")
    require(app, "Deferred calibration solving/storage intentionally pauses", "blocking-command pause")
    require(app_hooks, "g_fifoRuntime.hasPendingWork() || g_fifoRuntime.urgent()", "software FIFO deferred gate")
    require(app_hooks, "lsmFifo.readStatus", "hardware FIFO slack gate")
    require(app_hooks, "rotationDeadlineSlackMs", "rotation output deadline gate")
    require(app_hooks, "kMaxDeferredUnreadWords", "bounded hardware FIFO unread gate")

    require(reliability, "filteredHeadingRateDegS_", "filtered heading-rate state")
    require(reliability, "headingRateFilterTimeConstantS", "heading-rate low-pass")
    require(reliability, "MAG_FIELD_FLAG_ENVIRONMENT_CHANGED", "changed-environment observability")
    require(reliability, "stationaryHeadingJumpLatched_", "stationary heading discontinuity latch")
    require(reliability, "stationaryHeadingJumpRateDegS", "drift-safe discontinuity rate gate")
    require(reliability, "referenceReturnStableMs", "stable original-environment return gate")
    require(reliability, "wrapPi(heading.yawInnovationRad)", "AHRS-yaw-invariant stationary field signal")
    require(reliability, "stationaryLatchReferenceFieldYawRad_", "pre-jump stationary field recovery baseline")
    require(reliability, "stationaryLatchMotionSeen_", "physical-motion invalidation of relative recovery")
    require(reliability, "stationaryHeadingReturnsViaStationaryField", "stationary yaw-drift recovery diagnostics")
    require(reliability, "state_ == MagFieldReliabilityState::Disturbed", "fail-closed changed environment")
    require(reliability, "acquireHeadingSinSum_", "window-averaged heading reference")
    require(reliability_h, "restartAcquisition", "explicit environment reacquisition API")

    require(yaw, "MAG_YAW_REJECT_REACQUIRE_PENDING", "large-error reacquisition pending state")
    require(yaw, "reacquireMinFieldStableMs", "reacquisition stability gate")
    require(yaw, "reacquireMaxCorrectionRateDegS", "bounded reacquisition rate")

    require(axis, "Quat::fromRotationVector(rotationVector)", "finite SO(3) interval prediction")
    require(axis, "refineCandidate", "continuous SO(3) refinement")
    require(axis, "maxRefinementDeg", "bounded local refinement")
    require(axis, "DatasetSubset::Training", "independent training partition")
    require(axis, "DatasetSubset::Validation", "independent validation partition")
    require(axis, "validationWinnerMatchesTraining", "holdout winner consistency gate")
    require(axis, "normalizedSeparation", "rate-normalized solver confidence")
    require(axis, "activeAlignment", "validation-set active scoring")
    require(axis, "properRotation(c.matrix)", "proper-rotation-only coarse solver")
    require(axis_h, "coarseMagToImu", "coarse/fine result separation")
    require(axis_h, "qualityScore", "measured solver quality")
    require(axis_h, "lastSolveUs", "deferred timing diagnostics")

    require(setup, "solveMagAxisAlignmentDataset", "shared setup/runtime solver")
    require(setup, "setupPrintMagAxisMatrix", "continuous matrix setup diagnostics")
    require(setup, "isProperRotationMatrix(m", "setup proper-rotation guard")
    require(mag_cli, "isProperRotationMatrix(out", "manual proper-rotation guard")

    require(store, "candidateExists(bool& outExists)", "lightweight store presence API")
    require(store, "candidateHasMeasuredAlignment", "measured-vs-derived comparison bridge")
    require(storage_h, "ALIGNMENT_MEASURED", "alignment evidence quality flag")

    require(runner, 'pathlib.Path("src/sensor/mag_field_reliability.cpp")', "field reliability host compilation")
    require(runner, 'pathlib.Path("src/sensor/mag_axis_alignment.cpp")', "axis alignment host compilation")
    require(machine, "field_heading_error_deg", "machine-log field reliability observability")
    require(machine, "reacquire_pending", "machine-log reacquisition observability")

    require(controller_h, "axisAlignmentCandidateWorkspace", "explicit candidate workspace dependency")
    require(controller_h, "evaluateDeferredServiceGate", "structured deferred service composition gate")
    require(controller_h, "MAG_DEFERRED_REJECT_HARDWARE_FIFO_BUSY", "hardware FIFO deferral reason")
    require(controller_h, "MAG_DEFERRED_REJECT_OUTPUT_DEADLINE", "output deadline deferral reason")
    require(controller, "axisIndependentRejects", "bad-active-axis-independent learning path")
    print("PASS magnetic heading reliability policy")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
