#!/usr/bin/env python3
"""Guard patch 0028 semantic validation and persistent transaction contracts."""

from __future__ import annotations

import ast
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def source(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {label}: {needle}")


def forbid(text: str, needle: str, label: str) -> None:
    if needle in text:
        raise SystemExit(f"forbidden {label}: {needle}")


def main() -> int:
    config = source("src/config/tracker_config_runtime.cpp")
    store = source("src/config/tracker_config_store.cpp")
    network = source("src/config/tracker_network_config.cpp")
    parser = source("src/serial/tracker_command_line_parser.hpp")
    serial = source("src/serial/tracker_serial_commands.hpp")
    setup = source("src/serial/tracker_setup_commands.cpp")
    reset_h = source("src/config/factory_reset_coordinator.hpp")
    reset = source("src/config/factory_reset_coordinator.cpp")
    autonomy = source("src/runtime/calibration_autonomy_controller.cpp")
    math = source("src/core/math.hpp")
    ahrs = source("src/sensor/ahrs_6dof.cpp")
    check_all = source("tools/check_all.py")
    testing = source("docs/testing.md")
    project = source("docs/project_status.md")
    report = source("docs/0028_semantic_config_and_calibration_transaction_report.md")

    require(config, "bool TrackerConfig::validateSemanticConfig(", "semantic validator")
    for invariant in (
        "validCalibrationMatrix", "isProperRotationMatrix", "ImuFifoMismatch",
        "MagYawGates", "AhrsGates", "QualityGates", "ReservedState",
    ):
        require(config, invariant, f"config invariant {invariant}")
    require(store, "if (!candidate.validateSemanticConfig())", "immutable store admission")
    migration_start = config.find("void trackerMigratePerformanceDefaults(")
    migration_end = config.find("\n}\n\n} // namespace tracker", migration_start)
    forbid(config[migration_start:migration_end], "sanitize();", "general repair in performance migration")
    require(store, "legacy->sanitize();", "explicit main-config legacy migration")
    forbid(
        re.sub(r"//.*?$|/\*.*?\*/", "", store, flags=re.MULTILINE | re.DOTALL),
        "candidate.sanitize();",
        "candidate repair inside persistent store",
    )

    require(network, "tmp.computeCrc() != tmp.data.crc32", "legacy envelope proof")
    require(network, 'std::strcmp(tmp.data.deviceName, "c3_6dsv_tracker") == 0', "exact legacy network migration")
    forbid(network, "TrackerNetworkConfig::sanitize", "general network candidate repair")
    require(network, "std::memcmp(&verify.data, &candidate.data", "network exact readback")

    for status in ("TooManyArguments", "UnclosedQuote", "TrailingCharactersAfterQuote"):
        require(parser, status, f"parser status {status}")
        require(serial, f"TrackerCommandLineParseStatus::{status}", f"parser rejection {status}")
    require(serial, 'printErr(*ctx_->io, "line too long")', "line overflow rejection")
    require(setup, "complete line rejected", "setup overflow rejection")

    require(reset_h, "MARKER_ENCODED_SIZE = 16u", "fixed-width reset marker")
    require(reset, "encodeMarker", "reset marker encoder")
    require(reset, "decodeMarker", "reset marker decoder")
    require(reset, "writeMarker(marker)", "verified reset checkpoints")
    forbid(reset_h + reset, "calibrationReplacement", "raw config in reset marker")
    forbid(reset, "putBytes(\n        factory_reset_detail::NVS_KEY_PENDING, &marker", "raw marker ABI persistence")

    require(autonomy, "probationDeadlineMs_ = nowMs + kProbationMaxMs", "absolute probation deadline")
    require(autonomy, "probationSensorHealthFailed", "sensor probation verdict")
    require(autonomy, "probationTransportHealthFailed", "transport probation verdict")
    fault_block = autonomy[autonomy.find("if (sensorFailed || transportFailed)"):
                           autonomy.find("} else if (probationCanAccept", autonomy.find("if (sensorFailed || transportFailed)"))]
    forbid(fault_block, "probationStartedMs_ =", "transient extending probation")

    require(math, "bool tryNormalized(Quat& out,", "fallible quaternion normalization")
    require(ahrs, "invalidQuaternionRejectedCount", "invalid quaternion hard rejection")

    entry = '("tools/test_0028_semantic_config_transaction_policy.py", "0028 semantic config/transaction policy")'
    ast.parse(check_all, filename="tools/check_all.py")
    checks_start = check_all.find("    checks = (", check_all.find("def run_tool_smokes("))
    checks_end = check_all.find("\n    )", checks_start)
    entry_at = check_all.find(entry)
    if not checks_start < entry_at < checks_end:
        raise SystemExit("0028 aggregate gate entry is outside run_tool_smokes checks tuple")
    require(testing, "0028 semantic config and calibration transaction", "testing documentation")
    require(project, "0028_semantic_config_and_calibration_transaction", "project status entry")
    require(report, "Not verified", "honest unverified section")

    print("# 0028_semantic_config_transaction_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
