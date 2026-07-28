#!/usr/bin/env python3
"""Guard 0023gh SlimeVR serial Wi-Fi provisioning compatibility."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src/serial/tracker_slimevr_serial_compat_commands.cpp"


def require(text: str, needle: str, description: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {description}: {needle}")


def forbid(text: str, needle: str, description: str) -> None:
    if needle in text:
        raise SystemExit(f"forbidden {description}: {needle}")


def compiler() -> str:
    explicit = os.environ.get("CXX")
    if explicit and shutil.which(explicit):
        return explicit
    for name in ("g++", "clang++", "c++"):
        found = shutil.which(name)
        if found:
            return found
    raise SystemExit("no C++ compiler available for 0023gh policy")


def isolate(text: str, begin: str, end: str) -> str:
    start = text.find(begin)
    if start < 0:
        raise SystemExit(f"could not find section start: {begin}")
    finish = text.find(end, start)
    if finish < 0:
        raise SystemExit(f"could not find section end: {end}")
    return text[start:finish]


def main() -> int:
    compat = SOURCE.read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")
    cli = (ROOT / "docs/cli_reference.md").read_text(encoding="utf-8")
    report = (ROOT / "docs/0023gh_slimevr_wifi_provisioning_compat_hardening_report.md").read_text(
        encoding="utf-8"
    )

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


    require(
        check_all,
        '("tools/test_slimevr_wifi_provisioning_0023gh_policy.py", "0023gh SlimeVR Wi-Fi provisioning compatibility")',
        "aggregate runner entry",
    )
    require(cli, "WiFiReconnectionStatus", "CLI protocol documentation")
    require(report, "Success=5", "corrected protocol finding")
    require(report, "no tracking hot-path", "hot-path audit conclusion")

    cxx = compiler()
    with tempfile.TemporaryDirectory(prefix="tracker-0023gh-") as tmp_name:
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
        subprocess.run([*common, "-o", str(tmp / "compat_native.o")], check=True)
        subprocess.run(
            [
                *common,
                "-DARDUINO",
                "-DARDUINO_ARCH_ESP32",
                "-DTRACKER_BUILD_PROFILE=TRACKER_PROFILE_PRODUCTION",
                "-o", str(tmp / "compat_production.o"),
            ],
            check=True,
        )

    print("# slimevr_wifi_provisioning_0023gh_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
