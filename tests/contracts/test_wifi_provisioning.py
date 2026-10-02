#!/usr/bin/env python3
"""Guard wifi_provisioning SlimeVR serial Wi-Fi provisioning compatibility."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, forbid, isolate, require, run_contract_command,
)


from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "src/serial/tracker_slimevr_serial_compat_commands.cpp"


def main() -> int:
    compat = SOURCE.read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")

    # This is SlimeVR's WiFiReconnectionStatus protocol, not Arduino WL_*.
    require(compat, "enum class SlimeVrWifiReconnectionStatus : uint8_t", "named protocol enum")
    for name, value in (
        ("NotSetup", 0),
        ("SavedAttempt", 1),
        ("HardcodeAttempt", 2),
        ("ServerCredAttempt", 3),
        ("Failed", 4),
        ("Success", 5),
    ):
        require(compat, f"{name} = {value}", f"{name} protocol value")

    require(
        compat,
        "case TrackerWifiState::Connecting:",
        "connecting state mapping",
    )
    require(compat, "g_slimeVrServerCredentialAttempt = saved;", "server-attempt ownership latch")
    require(compat, "? SlimeVrWifiReconnectionStatus::ServerCredAttempt", "server-attempt mapping")
    require(compat, ": SlimeVrWifiReconnectionStatus::SavedAttempt", "saved-attempt mapping")
    require(
        compat,
        "case TrackerWifiState::Backoff:\n            return static_cast<uint8_t>(SlimeVrWifiReconnectionStatus::Failed);",
        "failed/backoff mapping",
    )
    require(
        compat,
        "case TrackerWifiState::Connected:\n            return static_cast<uint8_t>(SlimeVrWifiReconnectionStatus::Success);",
        "successful connection mapping",
    )
    require(compat, "static_assert(slimeVrWifiStateCode(TrackerWifiState::Connecting, false) == 1);",
            "saved-attempt compile-time contract")
    require(compat, "static_assert(slimeVrWifiStateCode(TrackerWifiState::Connecting, true) == 3);",
            "server-attempt compile-time contract")
    require(compat, "static_assert(slimeVrWifiStateCode(TrackerWifiState::Backoff, true) == 4);",
            "failure compile-time contract")
    require(compat, "static_assert(slimeVrWifiStateCode(TrackerWifiState::Connected, true) == 5);",
            "success compile-time contract")

    require(compat, "CMD SET WIFI OK: New wifi credentials set, reconnecting", "exact SET WIFI ACK")
    require(compat, "CMD SET BWIFI OK: New wifi credentials set, reconnecting", "exact SET BWIFI ACK")
    forbid(compat, "New wifi credentials saved to NVS, reconnecting", "non-upstream provisioning ACK")

    setter = isolate(compat, "bool setWifiCredentials(", "\nint base64Value(")
    forbid(setter, "delay(", "blocking provisioning delay")
    forbid(setter, "while (", "blocking provisioning wait loop")
    require(setter, "TrackerNetworkConfig candidate = *ctx.networkConfig;", "transactional credential candidate")
    require(setter, "const bool saved = ctx.networkConfigStore->save(candidate);", "persistent credential commit")
    require(setter, "if (!saved) return false;", "failed-commit short circuit")
    require(setter, "*ctx.networkConfig = candidate;", "post-commit live activation")
    require(
        setter,
        "TrackerSlimeVRRuntimeApplyMode::RestartSession",
        "explicit post-provisioning SlimeVR session restart",
    )
    if setter.index("ctx.networkConfigStore->save(candidate)") > setter.index("*ctx.networkConfig = candidate"):
        raise SystemExit("live network config is changed before the persistent commit")
    if setter.index("*ctx.networkConfig = candidate") > setter.index("ctx.wifiManager->reset()"):
        raise SystemExit("Wi-Fi reset happens before the committed candidate becomes live")


    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-wifi_provisioning-") as tmp_name:
        tmp = Path(tmp_name)
        common = [
            cxx,
            "-std=c++20",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Wshadow",
            "-Wdouble-promotion",
            "-Wformat=2",
            "-Wno-unused-parameter",
            "-I", str(ROOT / "src"),
            "-I", str(ROOT / "tests/native"),
            "-c", str(SOURCE),
        ]
        run_contract_command([*common, "-o", str(tmp / "compat_native.o")], check=True)
        run_contract_command(
            [
                *common,
                "-DARDUINO",
                "-DARDUINO_ARCH_ESP32",
                "-DTRACKER_BUILD_PROFILE=TRACKER_PROFILE_PRODUCTION",
                "-o", str(tmp / "compat_production.o"),
            ],
            check=True,
        )

    print("# wifi_provisioning: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
