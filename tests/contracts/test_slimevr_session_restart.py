#!/usr/bin/env python3
"""Guard slimevr_session_restart already-connected Connect Trackers session restart semantics."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, forbid, isolate, read_native_test, require, run_contract_command,
)


from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[2]
COMMANDS_HPP = ROOT / "src/serial/tracker_slimevr_commands.hpp"
COMMANDS_CPP = ROOT / "src/serial/tracker_slimevr_commands.cpp"
COMPAT_CPP = ROOT / "src/serial/tracker_slimevr_serial_compat_commands.cpp"
RUNTIME_TEST = ROOT / "tests/native/test_slimevr_output_runtime.cpp"


def main() -> int:
    commands_hpp = COMMANDS_HPP.read_text(encoding="utf-8")
    commands_cpp = COMMANDS_CPP.read_text(encoding="utf-8")
    compat = COMPAT_CPP.read_text(encoding="utf-8")
    runtime_test = read_native_test(RUNTIME_TEST)
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")

    require(commands_hpp, "enum class TrackerSlimeVRRuntimeApplyMode", "typed runtime apply mode")
    require(commands_hpp, "PreserveSession = 0", "non-disruptive start mode")
    require(commands_hpp, "RestartSession = 1", "explicit restart mode")
    require(commands_hpp, "trackerSerialApplySlimeVRRuntimeConfig", "shared runtime apply API")

    helper = isolate(
        commands_cpp,
        "bool trackerSerialApplySlimeVRRuntimeConfig(",
        "\nbool trackerSerialDispatchSlimeVRCommand(",
    )
    require(helper, "ctx.slimevrRuntime->configure(", "shared runtime configuration")
    require(helper, "mode == TrackerSlimeVRRuntimeApplyMode::RestartSession", "typed restart decision")
    require(helper, "ctx.slimevrRuntime->restart();", "session reset and discovery restart")
    require(helper, "return true;", "successful shared apply")

    start = isolate(
        commands_cpp,
        'if (tracker_serial_detail::eqIgnoreCase(argv[1], "start"))',
        '\n    if (tracker_serial_detail::eqIgnoreCase(argv[1], "stop"))',
    )
    require(start, "TrackerSlimeVRRuntimeApplyMode::PreserveSession", "non-disruptive slime start")

    reconnect = isolate(
        commands_cpp,
        'if (tracker_serial_detail::eqIgnoreCase(argv[1], "reconnect")',
        '\n\n    if (tracker_serial_detail::eqIgnoreCase(argv[1], "rate"))',
    )
    require(reconnect, "TrackerSlimeVRRuntimeApplyMode::RestartSession", "explicit CLI reconnect")

    setter = isolate(compat, "bool setWifiCredentials(", "\nint base64Value(")
    require(setter, "TrackerNetworkConfig candidate = *ctx.networkConfig;", "transactional candidate")
    require(setter, "ctx.networkConfigStore->save(candidate)", "candidate persistence")
    require(setter, "if (!saved) return false;", "failed NVS commit leaves live state alone")
    require(setter, "*ctx.networkConfig = candidate;", "post-commit activation")
    require(setter, "ctx.wifiManager->reset();", "forced station reconnect")
    require(setter, "TrackerSlimeVRRuntimeApplyMode::RestartSession", "forced SlimeVR session restart")
    forbid(compat, "trackerSerialDispatchSlimeVRCommand(ctx", "recursive CLI dispatch from compatibility path")
    forbid(setter, "delay(", "blocking provisioning delay")
    forbid(setter, "while (", "blocking provisioning wait")

    save_pos = setter.index("ctx.networkConfigStore->save(candidate)")
    activate_pos = setter.index("*ctx.networkConfig = candidate")
    wifi_reset_pos = setter.index("ctx.wifiManager->reset()")
    session_restart_pos = setter.index("TrackerSlimeVRRuntimeApplyMode::RestartSession")
    if not (save_pos < activate_pos < wifi_reset_pos < session_restart_pos):
        raise SystemExit("provisioning transaction/restart order is not commit -> activate -> Wi-Fi reset -> session restart")

    require(runtime_test, "CHECK(ctx, wifi.connected());", "same-credentials connected-station scenario")
    require(runtime_test, "CHECK(ctx, !reconnectStatus.serverFound);", "server session cleared")
    require(runtime_test, "SlimeVRSensorInfoSyncState::Dirty", "SensorInfo dirtied on restart")
    require(runtime_test, "readU64BeLocal(reconnectUdp.sent.back().data.data() + 4) == 0u", "fresh zero-sequence discovery")
    require(runtime_test, "reconnectRt.status().sensorInfoSent == 2u", "fresh SensorInfo registration")
    require(runtime_test, "SlimeVRSensorInfoSyncState::Acknowledged", "fresh SensorInfo acknowledgement")


    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-slimevr_session_restart-") as tmp_name:
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
        ]
        run_contract_command([*common, "-c", str(COMMANDS_CPP), "-o", str(tmp / "commands.o")], check=True)
        run_contract_command([*common, "-c", str(COMPAT_CPP), "-o", str(tmp / "compat.o")], check=True)
        run_contract_command(
            [
                *common,
                "-DARDUINO",
                "-DARDUINO_ARCH_ESP32",
                "-DTRACKER_BUILD_PROFILE=TRACKER_PROFILE_PRODUCTION",
                "-c", str(COMPAT_CPP),
                "-o", str(tmp / "compat_production.o"),
            ],
            check=True,
        )

        runtime_exe = tmp / "test_slimevr_output_runtime"
        run_contract_command(
            [
                *common,
                str(RUNTIME_TEST),
                str(ROOT / "src/runtime/slimevr_output_runtime.cpp"),
                str(ROOT / "src/output/slimevr_packet_writer.cpp"),
                str(ROOT / "src/network/wifi_manager.cpp"),
                str(ROOT / "src/network/udp_transport.cpp"),
                "-o", str(runtime_exe),
            ],
            check=True,
        )
        run_contract_command([str(runtime_exe)], check=True)

    print("# slimevr_session_restart: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
