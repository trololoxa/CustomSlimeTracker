#!/usr/bin/env python3
"""Guard the calibration_autonomy safe background calibration autonomy contract."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, forbid, function_body, parse_stack_usage, require, require_limit, run_contract_command,
)


from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    controller = (ROOT / "src/runtime/calibration_autonomy_controller.cpp").read_text(encoding="utf-8")
    controller_h = (ROOT / "src/runtime/calibration_autonomy_controller.hpp").read_text(encoding="utf-8")
    store = (ROOT / "src/runtime/calibration_autonomy_store.cpp").read_text(encoding="utf-8")
    store_h = (ROOT / "src/runtime/calibration_autonomy_store.hpp").read_text(encoding="utf-8")
    config_store = (ROOT / "src/config/tracker_config_store.cpp").read_text(encoding="utf-8")
    storage_h = (ROOT / "src/config/tracker_config_storage.hpp").read_text(encoding="utf-8")
    app = (ROOT / "src/app/tracker_app.cpp").read_text(encoding="utf-8")
    command = (ROOT / "src/serial/tracker_calibration_commands.cpp").read_text(encoding="utf-8")
    help_text = (ROOT / "src/serial/tracker_system_commands.cpp").read_text(encoding="utf-8")
    profiler = (ROOT / "src/runtime/runtime_profiler.cpp").read_text(encoding="utf-8")
    feature = (ROOT / "src/build_config/firmware_feature_version.hpp").read_text(encoding="utf-8")
    config_detail = (ROOT / "src/config/tracker_config_detail.hpp").read_text(encoding="utf-8")
    runner = (ROOT / "tools/run_standalone_tests.py").read_text(encoding="utf-8")

    require(storage_h, "AUTONOMY_0022", "0022 candidate ownership flag")
    require(storage_h, "AUTONOMY_0023", "calibration_autonomy candidate ownership flag")
    require(storage_h, "GYRO_MEASURED", "measured gyro evidence flag")
    require(storage_h, "ACCEL_MEASURED", "measured accel evidence flag")

    require(store_h, 'KEY_JOURNAL_A = "journal_a"', "first CRC journal slot")
    require(store_h, 'KEY_JOURNAL_B = "journal_b"', "second CRC journal slot")
    require(store, "writeRecordVerified", "journal read-back verification")
    require(store, "sequenceNewer", "dual-slot journal ordering")
    require(controller, "journal_.state = CalibrationAutonomyJournalState::PromotionPending", "write-ahead promotion journal state")
    require(controller, "autonomyStore->writeJournal(journal_)", "write-ahead promotion journal persistence")
    require(controller, "setAutonomyProbationWriteBarrier(true)", "probation rollback-anchor write barrier")
    require(controller, "setAutonomyProbationWriteBarrier(false)", "write barrier release")
    require(controller, "prepareCandidatePromotion", "two-phase promotion preparation")
    require(controller, "commitPreparedPromotion", "atomic promotion commit")
    require(controller, "rollbackPromotion", "automatic rollback path")
    require(controller, "FreshEvidenceRegression", "fresh probation regression rejection")
    require(controller, "candidateOwnedByAutonomy", "manual candidate ownership protection")
    require(controller, "kMinSessionSeparationMs", "independent-session separation")
    require(controller, "kProbationMinMs", "minimum probation duration")
    require(controller, "probationTransportVerified_", "separate transport verification state")
    require(controller, "componentNonRegression", "per-axis non-regression gate")

    observe = function_body(controller, "void CalibrationAutonomyController::observeImuSample")
    for forbidden in ("configStore", "autonomyStore", "stageCandidate", "flushCandidate", "Preferences", "writeJournal"):
        forbid(observe, forbidden, "storage work in sample callback")

    require(app, "Calibration0022", "0022 profiler integration")
    require(app, "Calibration0023", "calibration_autonomy profiler integration")
    require(profiler, 'return "calibration_0022"', "0022 profiler section name")
    require(profiler, 'return "calibration_0023"', "calibration_autonomy profiler section name")
    require(command, 'is(argv[1], "autonomy")', "cal autonomy dispatcher")
    require(command, 'is(argv[2], "0022")', "0022 runtime switch")
    require(command, 'is(argv[2], "0023")', "calibration_autonomy runtime switch")
    require(command, 'is(argv[4], "save")', "explicit persistence suffix")
    require(help_text, "cal autonomy status", "autonomy CLI help")
    require(help_text, "0022 on|off [save]", "0022 help switch")
    require(help_text, "0023 on|off [save]", "calibration_autonomy help switch")

    require(config_store, "autonomyProbationWriteBarrier_", "config save write barrier")
    require(runner, 'pathlib.Path("src/runtime/calibration_autonomy_store.cpp")', "autonomy store host compilation")
    require(runner, 'pathlib.Path("src/runtime/calibration_autonomy_controller.cpp")', "autonomy controller host compilation")

    # Persistent tracker config and candidate record formats deliberately remain unchanged.

    with project_temp_directory(ROOT, "tracker-autonomy-stack-") as tmp:
        tmp_path = Path(tmp)
        cxx = compiler()
        common = [
            cxx,
            "-std=c++17",
            "-O2",
            "-fstack-usage",
            "-I",
            str(ROOT / "src"),
            "-I",
            str(ROOT / "tests/native"),
            "-c",
        ]
        for source in (
            ROOT / "src/runtime/calibration_autonomy_store.cpp",
            ROOT / "src/runtime/calibration_autonomy_controller.cpp",
        ):
            run_contract_command(common + [str(source), "-o", str(tmp_path / (source.stem + ".o"))], check=True)
        usage = parse_stack_usage(tmp_path)

    for function in (
        "CalibrationAutonomyController::service(",
        "CalibrationAutonomyController::beginPromotion",
        "CalibrationAutonomyController::rollbackPromotion",
        "CalibrationAutonomyController::acceptPromotion",
        "CalibrationAutonomyController::buildGyroBiasProposal",
        "CalibrationAutonomyController::buildGyroTemperatureProposal",
        "CalibrationAutonomyStore::writeJournal",
    ):
        require_limit(usage, function, 768)
    # Keep a large cross-ABI margin. Linux/GCC measured 224 bytes after moving
    # the six validation sessions out of the local frame; Windows/MSYS2 GCC is
    # known to add substantially more ABI spill space than Linux GCC.
    require_limit(usage, "CalibrationAutonomyController::buildAccelProposal", 640)

    print("# calibration_autonomy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
