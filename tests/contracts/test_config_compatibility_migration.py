#!/usr/bin/env python3
"""Guard additive config_compatibility_migration compatibility migration without weakening v3."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    between, read_native_test, require,
)

import ast


ROOT = Path(__file__).resolve().parents[2]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def main() -> int:
    storage_h = read("src/config/tracker_config_storage.hpp")
    storage = read("src/config/tracker_config_storage.cpp")
    store = read("src/config/tracker_config_store.cpp")
    commands = read("src/serial/tracker_config_commands.cpp")
    tests = read_native_test(ROOT / "tests/native/test_config_storage.cpp")
    check_all = read("tools/check_all.py")

    require(storage_h, "TRANSITIONAL_SLOT_VERSION = 2", "deployed v2 identity")
    require(storage_h, "DEPLOYED_V3_SLOT_VERSION = 3", "deployed v3 identity")
    require(storage_h, "SLOT_VERSION = 4", "versioned strict current writer")
    require(storage, "record.version != tracker_config_storage_detail::TRANSITIONAL_SLOT_VERSION",
            "v2 envelope admission")
    current_guard = between(
        storage,
        "bool trackerValidateConfigSlotRecord(",
        "uint32_t trackerConfigCommitRecordCrc",
        "slot validator",
    )
    require(current_guard, "record.version == tracker_config_storage_detail::SLOT_VERSION",
            "strict semantics only for current writer")
    require(current_guard, "!config.validateSemanticConfig()", "strict v3 semantic admission")

    migration = between(store, "bool TrackerConfigStore::load(", "bool TrackerConfigStore::verify(", "load migration")
    for needle, label in (
        ("record->version != tracker_config_storage_detail::SLOT_VERSION", "all non-current slots migrate"),
        ("migratedSlot->sanitize()", "bounded compatibility normalization"),
        ("trackerMigratePerformanceDefaults(*migratedSlot)", "performance-default migration"),
        ("migratedSlot->validateSemanticConfig()", "post-migration semantic proof"),
        ("saveInternal(*migratedSlot, true", "persist-before-apply v3 commit"),
        ("out = *migratedSlot", "explicit read-only volatile result"),
    ):
        require(migration, needle, label)

    require(commands, "slot_a_semantic_error=", "slot-A semantic diagnostics")
    require(commands, "slot_b_semantic_error=", "slot-B semantic diagnostics")
    for test in (
        "testTransitionalV2MigratesWithoutLosingMagOrCalibration",
        "testTransitionalV2ReadOnlyMigrationIsVolatileAndPreservesSource",
        "testInterruptedTransitionalV2MigrationKeepsOldGood",
        "testTransitionalV2ImpossibleCalibrationStillFailsClosed",
    ):
        require(tests, test, test)
    for preserved in (
        "loaded.data.magCal.driverEnabled",
        "loaded.data.magCal.calibrationValid",
        "loaded.data.magCal.axisAlignmentValid",
        "loaded.data.magYaw.applyEnabled",
        "loaded.data.gyroCal.biasValid",
        "loaded.data.accelCal.valid",
    ):
        require(tests, preserved, f"preserved state {preserved}")

    ast.parse(check_all, filename="tools/check_all.py")
    print("# config_compatibility_migration: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
