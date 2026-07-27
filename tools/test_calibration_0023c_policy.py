#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing 0023c contract: {label}: {needle}")

store_h = (ROOT / "src/runtime/calibration_autonomy_store.hpp").read_text()
store_cpp = (ROOT / "src/runtime/calibration_autonomy_store.cpp").read_text()
controller_h = (ROOT / "src/runtime/calibration_autonomy_controller.hpp").read_text()
controller_cpp = (ROOT / "src/runtime/calibration_autonomy_controller.cpp").read_text()
cli = (ROOT / "src/serial/tracker_calibration_commands.cpp").read_text()
help_cpp = (ROOT / "src/serial/tracker_system_commands.cpp").read_text()

require(store_h, "CalibrationAutonomyJournalRecordV1", "read-only legacy journal layout")
require(store_cpp, "loadLegacyJournalV1", "legacy journal loader")
require(controller_cpp, "recoverLegacyJournalV1AtBoot", "conservative legacy rollback")
require(controller_cpp, "trackerApplyCalibrationCandidateToConfig(rollbackConfig_, previousConfig_)", "policy-preserving rollback composition")
require(controller_h, "preparePersistentCalibrationErase", "power-loss erase marker API")
require(controller_h, "forceClearPersistentCalibrationStateForErase", "destructive recovery API")
require(cli, 'if (is(argv[1], "erase_all"))', "erase dispatch")
require(cli, "cmdCalEraseAll(ctx, argc, argv);", "erase bypasses manual preflight")
require(cli, "preparePersistentCalibrationErase(clean)", "verified erase marker before slot removal")
require(cli, "ctx.configStore->erase()", "calibration slots erased before journal")
require(cli, "forceClearPersistentCalibrationStateForErase", "journal force cleanup")
require(cli, 'if (is(argv[1], "status"))', "cal status alias")
require(help_cpp, "  cal status", "cal status help")

erase_pos = cli.index('if (is(argv[1], "erase_all"))')
ownership_pos = cli.index("CalibrationManualOwnershipScope ownership")
if erase_pos > ownership_pos:
    raise SystemExit("0023c erase recovery must bypass manual ownership preflight")

erase_marker = cli.index("preparePersistentCalibrationErase(clean)")
erase_storage = cli.index("ctx.configStore->erase()")
erase_journal = cli.index("forceClearPersistentCalibrationStateForErase")
if not (erase_marker < erase_storage < erase_journal):
    raise SystemExit("0023c destructive ordering must be marker -> config erase/save -> metadata cleanup")

print("calibration 0023c policy: ok")
