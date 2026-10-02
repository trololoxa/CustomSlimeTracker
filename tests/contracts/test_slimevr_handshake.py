#!/usr/bin/env python3
"""Guard slimevr_handshake SlimeVR Connect Trackers handshake and build-date compatibility."""

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
COMPAT = ROOT / "src/serial/tracker_slimevr_serial_compat_commands.cpp"
WRITER = ROOT / "src/output/slimevr_packet_writer.cpp"
RUNTIME = ROOT / "src/runtime/slimevr_output_runtime_impl.inc"
BUILD_IDENTITY = ROOT / "tools/build_identity.py"


def main() -> int:
    compat = COMPAT.read_text(encoding="utf-8")
    writer = WRITER.read_text(encoding="utf-8")
    runtime = RUNTIME.read_text(encoding="utf-8")
    build_identity = BUILD_IDENTITY.read_text(encoding="utf-8")
    build_header = (ROOT / "src/build_config/build_identity.hpp").read_text(encoding="utf-8")
    packet_test = (ROOT / "tests/native/test_slimevr_packet_writer.cpp").read_text(encoding="utf-8")
    runtime_test = read_native_test(ROOT / "tests/native/test_slimevr_output_runtime.cpp")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")

    status = isolate(compat, "constexpr uint8_t slimeVrTrackerStatusCode", "\nenum class SlimeVrWifiReconnectionStatus")
    require(status, "return imuInitialized ? 0u : 3u;", "healthy tracker status contract")
    require(status, "static_assert(slimeVrTrackerStatusCode(true) == 0);", "healthy status assertion")
    forbid(status, "wifiManager", "Wi-Fi state inside tracker-health status")
    forbid(status, "slimevrRuntime", "server-discovery state inside tracker-health status")

    handshake = isolate(
        writer,
        "SlimeVRPacketWriteResult SlimeVRPacketWriter::writeHandshake",
        "\nSlimeVRPacketWriteResult SlimeVRPacketWriter::writeSensorInfo",
    )
    require(handshake, "cursor.writeU64Be(0)", "fixed zero discovery packet number")
    forbid(handshake, "writePacketHeader", "session sequence consumption by discovery")
    forbid(handshake, "++nextPacketNumber_", "discovery increment of session sequence")
    forbid(handshake, "nextPacketNumber_ =", "discovery assignment to session sequence")
    require(packet_test, "sequenceBeforeHandshake", "packet sequence preservation regression")
    require(packet_test, "readU64Be(packet + 4) == 0u", "discovery retry zero-sequence regression")
    require(runtime_test, "readU64BeLocal(udp.sent[0].data.data() + 4) == 0u", "runtime discovery zero sequence")

    require(runtime, "info.firmwareVersion = trackerBuildSlimeVRFirmwareVersion();", "dated UDP firmware string")
    require(compat, "out.print(trackerBuildSlimeVRFirmwareVersion());", "dated serial firmware string")
    require(compat, "out.println(trackerBuildDateUtc());", "serial build date")
    require(compat, "out.println(trackerBuildIdentityString());", "real Git/worktree identity")

    require(build_identity, "SOURCE_DATE_EPOCH", "reproducible build-date source")
    require(build_identity, "datetime.now(timezone.utc)", "UTC build date")
    require(build_identity, "+build.{compact}", "server-visible dated firmware version")
    require(build_header, "TRACKER_BUILD_DATE_UTC", "native build-date fallback")
    require(build_header, "trackerBuildSlimeVRFirmwareVersion", "server firmware accessor")


    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-slimevr_handshake-") as tmp_name:
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
        run_contract_command([*common, "-c", str(COMPAT), "-o", str(tmp / "compat_native.o")], check=True)
        run_contract_command(
            [
                *common,
                "-DARDUINO",
                "-DARDUINO_ARCH_ESP32",
                "-DTRACKER_BUILD_PROFILE=TRACKER_PROFILE_PRODUCTION",
                "-c", str(COMPAT),
                "-o", str(tmp / "compat_production.o"),
            ],
            check=True,
        )
        run_contract_command(
            [*common, "-c", str(ROOT / "src/runtime/slimevr_output_runtime.cpp"), "-o", str(tmp / "runtime.o")],
            check=True,
        )
        run_contract_command(
            [*common, "-c", str(ROOT / "src/serial/tracker_system_commands.cpp"), "-o", str(tmp / "system.o")],
            check=True,
        )
        packet_exe = tmp / "test_slimevr_packet_writer"
        run_contract_command(
            [
                *common,
                str(ROOT / "tests/native/test_slimevr_packet_writer.cpp"),
                str(ROOT / "src/output/slimevr_packet_writer.cpp"),
                "-o", str(packet_exe),
            ],
            check=True,
        )
        run_contract_command([str(packet_exe)], check=True)

    print("# slimevr_handshake: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
