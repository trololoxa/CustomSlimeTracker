#!/usr/bin/env python3
"""Guard passive suspended-storage motion-light-sleep hardening."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import require


ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    controller = (ROOT / "src/runtime/calibration_autonomy_controller.cpp").read_text(
        encoding="utf-8"
    )
    tests = (ROOT / "tests/native/test_calibration_autonomy.cpp").read_text(
        encoding="utf-8"
    )
    app = (ROOT / "src/app/tracker_app.cpp").read_text(encoding="utf-8")

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

    print("# sleep_epoch_recovery: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
