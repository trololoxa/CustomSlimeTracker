#!/usr/bin/env python3
"""Guard the 0023 safe background calibration autonomy contract."""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[1]


def require(text: str, needle: str, description: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {description}: {needle}")


def forbid(text: str, needle: str, description: str) -> None:
    if needle in text:
        raise SystemExit(f"forbidden {description}: {needle}")


def function_body(text: str, signature: str) -> str:
    start = text.index(signature)
    brace = text.index("{", start)
    depth = 0
    for i in range(brace, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[brace + 1:i]
    raise SystemExit(f"unterminated function: {signature}")


def compiler() -> str:
    explicit = os.environ.get("CXX")
    if explicit and shutil.which(explicit):
        return explicit
    for name in ("g++", "clang++", "c++"):
        found = shutil.which(name)
        if found:
            return found
    raise SystemExit("no C++ compiler available for autonomy stack policy")


def parse_stack_usage(directory: Path) -> dict[str, int]:
    usage: dict[str, int] = {}
    for path in directory.rglob("*.su"):
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            parts = line.split("\t")
            if len(parts) < 2:
                continue
            try:
                size = int(parts[1])
            except ValueError:
                continue
            name = parts[0].split(":", 3)[-1]
            usage[name] = max(size, usage.get(name, 0))
    return usage


def require_limit(usage: dict[str, int], needle: str, limit: int) -> None:
    matches = [(name, size) for name, size in usage.items() if needle in name]
    if not matches:
        raise SystemExit(f"autonomy stack policy did not find function: {needle}")
    worst_name, worst_size = max(matches, key=lambda item: item[1])
    if worst_size > limit:
        raise SystemExit(
            f"autonomy stack budget exceeded: {worst_name} uses {worst_size} bytes, limit {limit}"
        )


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

    require(feature, '"c3-6dsv-safe-calibration-autonomy-hardened"', "0023 firmware identity")
    require(storage_h, "AUTONOMY_0022", "0022 candidate ownership flag")
    require(storage_h, "AUTONOMY_0023", "0023 candidate ownership flag")
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
    require(controller, "probationHealthFailed", "realtime health probation gate")
    require(controller, "componentNonRegression", "per-axis non-regression gate")
    require(controller, "uint8_t secondIndex[6]", "compact accel validation indices")
    require(controller, "sessions_[secondIndex[face]]", "held-out session lookup by index")

    observe = function_body(controller, "void CalibrationAutonomyController::observeImuSample")
    for forbidden in ("configStore", "autonomyStore", "stageCandidate", "flushCandidate", "Preferences", "writeJournal"):
        forbid(observe, forbidden, "storage work in sample callback")

    require(app, "Calibration0022", "0022 profiler integration")
    require(app, "Calibration0023", "0023 profiler integration")
    require(profiler, 'return "calibration_0022"', "0022 profiler section name")
    require(profiler, 'return "calibration_0023"', "0023 profiler section name")
    require(command, 'is(argv[1], "autonomy")', "cal autonomy dispatcher")
    require(command, 'is(argv[2], "0022")', "0022 runtime switch")
    require(command, 'is(argv[2], "0023")', "0023 runtime switch")
    require(command, 'is(argv[4], "save")', "explicit persistence suffix")
    require(help_text, "cal autonomy status", "autonomy CLI help")
    require(help_text, "0022 on|off [save]", "0022 help switch")
    require(help_text, "0023 on|off [save]", "0023 help switch")

    require(config_store, "autonomyProbationWriteBarrier_", "config save write barrier")
    require(runner, 'pathlib.Path("src/runtime/calibration_autonomy_store.cpp")', "autonomy store host compilation")
    require(runner, 'pathlib.Path("src/runtime/calibration_autonomy_controller.cpp")', "autonomy controller host compilation")

    # Persistent tracker config and candidate record formats deliberately remain unchanged.
    require(config_detail, "CONFIG_VERSION = 2", "unchanged config schema v2")
    require(storage_h, "CANDIDATE_VERSION = 3", "unchanged candidate format v3")

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
            subprocess.run(common + [str(source), "-o", str(tmp_path / (source.stem + ".o"))], check=True)
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

    print("# calibration_autonomy_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
