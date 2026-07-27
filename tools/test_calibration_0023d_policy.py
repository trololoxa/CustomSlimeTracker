#!/usr/bin/env python3
"""Guard passive suspended-storage motion-light-sleep hardening."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def require(text: str, needle: str, description: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {description}: {needle}")


def main() -> int:
    controller = (ROOT / "src/runtime/calibration_autonomy_controller.cpp").read_text(
        encoding="utf-8"
    )
    tests = (ROOT / "tests/native/test_calibration_autonomy.cpp").read_text(
        encoding="utf-8"
    )
    app = (ROOT / "src/app/tracker_app.cpp").read_text(encoding="utf-8")
    docs = (ROOT / "docs/calibration_autonomy.md").read_text(encoding="utf-8")

    marker = "bool CalibrationAutonomyController::blocksMotionLightSleep() const {"
    end = "bool CalibrationAutonomyController::clearPersistentCalibrationState()"
    require(controller, marker, "sleep blocker implementation")
    sleep_block = controller.split(marker, 1)[1].split(end, 1)[0]

    require(sleep_block, "if (!begun_) return false;", "unavailable autonomy permits sleep")
    require(
        sleep_block,
        "CalibrationAutonomyStore::valid(journal_)",
        "valid durable transaction blocks sleep",
    )
    require(
        sleep_block,
        "passive fail-closed storage suspension",
        "passive storage suspension rationale",
    )
    if "CalibrationAutonomyState::SuspendedStorage" in sleep_block:
        raise SystemExit("passive suspended_storage still blocks motion light sleep")

    require(controller, "autonomy_motion_sleep_blocked=", "sleep blocker diagnostic")
    require(controller, "autonomy_motion_sleep_block_reason=", "sleep blocker reason diagnostic")
    require(tests, "!controller.blocksMotionLightSleep()", "passive suspension regression test")
    require(tests, "controller.blocksMotionLightSleep()", "active transaction regression test")
    require(app, "calibrationBlocksMotionSleep", "app sleep-admission wiring")
    require(docs, "0023d passive storage suspension and light sleep", "0023d documentation")

    print("# calibration_0023d_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
