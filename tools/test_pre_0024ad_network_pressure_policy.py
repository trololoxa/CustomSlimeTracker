#!/usr/bin/env python3
"""Guard pre-0024ad multi-tracker UDP pressure pacing/recovery semantics."""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

from quality_gate_runtime import asan_ubsan_environment, project_temp_directory

ROOT = Path(__file__).resolve().parents[1]


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
    raise SystemExit("no C++ compiler available for pre-0024ad policy")


def compile_runtime(cxx: str, exe: Path, extra: list[str]) -> None:
    subprocess.run([
        cxx, "-std=c++20", *extra,
        "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
        str(ROOT / "tests/native/test_slimevr_output_runtime.cpp"),
        str(ROOT / "tests/native/sanitizer_runtime_options.cpp"),
        str(ROOT / "src/runtime/slimevr_output_runtime.cpp"),
        str(ROOT / "src/output/slimevr_packet_writer.cpp"),
        str(ROOT / "src/network/wifi_manager.cpp"),
        str(ROOT / "src/network/udp_transport.cpp"),
        "-o", str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True, env=asan_ubsan_environment(ROOT))


def stack_usage(tmp: Path, symbol: str) -> int:
    found: list[int] = []
    for su in tmp.glob("*.su"):
        for line in su.read_text(encoding="utf-8", errors="replace").splitlines():
            if symbol not in line:
                continue
            fields = line.rsplit("\t", 2)
            if len(fields) >= 2:
                m = re.search(r"(\d+)", fields[-2])
                if m:
                    found.append(int(m.group(1)))
    if not found:
        raise SystemExit(f"stack usage missing for {symbol}")
    return max(found)


def main() -> int:
    tuning = (ROOT / "src/build_config/network_tuning.hpp").read_text(encoding="utf-8")
    header = (ROOT / "src/runtime/slimevr_output_runtime.hpp").read_text(encoding="utf-8")
    runtime = (ROOT / "src/runtime/slimevr_output_runtime_impl.inc").read_text(encoding="utf-8")
    test = (ROOT / "tests/native/test_slimevr_output_runtime.cpp").read_text(encoding="utf-8")
    status = (ROOT / "src/serial/tracker_slimevr_commands.cpp").read_text(encoding="utf-8")
    perf = (ROOT / "src/serial/tracker_perf_commands.cpp").read_text(encoding="utf-8")
    motion = (ROOT / "src/serial/tracker_motion_commands.cpp").read_text(encoding="utf-8")
    runtime_test = (ROOT / "src/runtime/runtime_test_runner.cpp").read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")
    testing = (ROOT / "docs/testing.md").read_text(encoding="utf-8")
    project = (ROOT / "docs/project_status.md").read_text(encoding="utf-8")
    cli = (ROOT / "docs/cli_reference.md").read_text(encoding="utf-8")
    report = (ROOT / "docs/pre_0024ad_network_pressure_pacing_and_recovery_hardening_report.md").read_text(encoding="utf-8")

    for needle, label in (
        ("TRACKER_SLIMEVR_TX_BACKOFF_INITIAL_MS 10UL", "10 ms initial pressure backoff"),
        ("TRACKER_SLIMEVR_TX_BACKOFF_MAX_MS 80UL", "80 ms maximum pressure backoff"),
        ("TRACKER_SLIMEVR_TX_PRESSURE_REBIND_STALL_MS 500UL", "motion-stall rebind gate"),
        ("TRACKER_SLIMEVR_TX_POST_REBIND_REOPEN_STALL_MS 1000UL", "post-rebind reopen gate"),
        ("TRACKER_SLIMEVR_TX_PRESSURE_STABLE_RESET_MS 1000UL", "stable episode close gate"),
        ("TRACKER_SLIMEVR_TX_REBIND_COOLDOWN_MS 5000UL", "local rebind cooldown"),
        ("TRACKER_SLIMEVR_TX_FULL_REOPEN_COOLDOWN_MS 10000UL", "full reopen cooldown"),
    ):
        require(tuning, needle, label)
    forbid(tuning, "TRACKER_SLIMEVR_TX_REBIND_ESCALATION_MS", "superseded second-burst escalation")

    for needle, label in (
        ("enum class SlimeVRTxPressureState", "pressure state enum"),
        ("enum class SlimeVRTxRecoveryReason", "recovery reason enum"),
        ("PacketPurpose::ControlBackground", "background-control packet class"),
        ("lastSuccessfulMotionTxValid_", "explicit successful-motion timestamp validity"),
        ("lastMotionTxAttemptValid_", "explicit motion-attempt timestamp validity"),
        ("lastUdpTransportRebindValid_", "explicit rebind timestamp validity"),
        ("lastUdpFullReopenValid_", "explicit full-reopen timestamp validity"),
    ):
        require(header + runtime, needle, label)

    for needle, label in (
        ("beginTxPressureEpisode(nowMs)", "pressure episode start"),
        ("activelyFailingMotion", "fresh failed-motion evidence"),
        ("TRACKER_SLIMEVR_TX_PRESSURE_REBIND_STALL_MS", "elapsed no-success recovery"),
        ("SlimeVRTxPressureState::AwaitingPostRebindSuccess", "post-rebind state"),
        ("SlimeVRTxRecoveryReason::PostRebindMotionStall", "post-rebind escalation reason"),
        ("udpRebindSuppressedCooldown_", "rebind cooldown diagnostic"),
        ("udpFullReopenSuppressedCooldown_", "reopen cooldown diagnostic"),
        ("physicalDatagramsSent_", "physical datagram counter"),
        ("txBackoffUntilMs_ = serverFoundSendGraceUntilMs_", "post-rebind motion/background grace"),
        ("case PacketPurpose::ControlBackground:\n            return true;", "background control pressure yielding"),
        ("fallbackAccelerationAllowed()", "negotiation acceleration gate"),
        ("accelerationSuppressedDuringNegotiation_", "reconnect datagram suppression"),
        ("refreshRotationPhaseOffset()", "deterministic motion phase"),
        ("hash *= 16777619UL", "bounded MAC phase hash"),
        ("rotationPhaseOffsetMs_", "phase diagnostic"),
    ):
        require(runtime, needle, label)

    forbid(runtime, "std::vector", "heap-backed pressure history")
    forbid(runtime, "new ", "heap allocation in pressure recovery")

    for needle, label in (
        ("continuing successful motion prevents socket churn", "intermittent pressure regression"),
        ("txPressureStableResets == 1u", "stable episode reset regression"),
        ("udpTransportRebindRequests == 1u", "sustained pressure local rebind regression"),
        ("SlimeVRTxPressureState::AwaitingPostRebindSuccess", "post-rebind state regression"),
        ("udpFullReopenEscalations == 1u", "post-rebind full reopen regression"),
        ("udpRebindSuppressedCooldown == 1u", "single-count rebind cooldown regression"),
        ("accelerationSuppressedDuringNegotiation > 0u", "no 150 Hz reconnect fallback regression"),
        ("separateAccelerationDatagramsSent == 0u", "no separate acceleration during negotiation"),
        ("Optional background control yields to the active TX-pressure backoff", "background-control retry regression"),
    ):
        require(test, needle, label)

    diagnostics = (
        "tx_pressure_state", "tx_recovery_reason", "tx_pressure_episode_count",
        "last_successful_motion_tx_age_ms", "udp_rebind_suppressed_cooldown",
        "udp_full_reopen_suppressed_cooldown", "physical_datagrams_sent", "motion_datagrams_sent",
        "separate_rotation_datagrams_sent", "separate_acceleration_datagrams_sent",
        "acceleration_suppressed_during_negotiation", "rotation_phase_offset_ms",
    )
    combined = "\n".join((status, perf, motion, runtime_test, cli))
    for field in diagnostics:
        require(combined, field, f"network-pressure diagnostic {field}")

    require(check_all, "test_pre_0024ad_network_pressure_policy.py", "aggregate policy entry")
    require(testing, "pre-0024ad network-pressure pacing/recovery regression", "testing entry")
    require(project, "pre_0024ad_network_pressure_pacing_and_recovery_hardening", "project status entry")
    require(report, "28,883", "five-hour hardware pressure evidence")
    require(report, "815", "five-hour full-reopen evidence")

    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-pre0024ad-") as tmp_name:
        tmp = Path(tmp_name)
        compile_runtime(cxx, tmp / "runtime", ["-O2", "-Wall", "-Wextra", "-Werror"])
        compile_runtime(cxx, tmp / "runtime_san", [
            "-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        ])
        subprocess.run([
            cxx, "-std=c++20", "-O2", "-fstack-usage",
            "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
            "-c", str(ROOT / "src/runtime/slimevr_output_runtime.cpp"),
            "-o", str(tmp / "runtime.o"),
        ], check=True)
        for symbol, ceiling in (
            ("SlimeVROutputRuntime::update", 160),
            ("SlimeVROutputRuntime::sendPacket", 160),
            ("SlimeVROutputRuntime::serviceTxRecovery", 128),
            ("SlimeVROutputRuntime::rebindUdpPreservingSession", 128),
        ):
            measured = stack_usage(tmp, symbol)
            if measured > ceiling:
                raise SystemExit(f"{symbol} stack {measured} exceeds {ceiling}")

    print("# pre_0024ad_network_pressure_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
