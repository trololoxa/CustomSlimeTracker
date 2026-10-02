#!/usr/bin/env python3
"""Guard network_pressure_pacing multi-tracker UDP pressure pacing/recovery semantics."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, forbid, read_native_test, require, run_contract_command, stack_usage,
)

import re

from quality_gate_runtime import (
    project_temp_directory,
)

ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    tuning = (ROOT / "src/build_config/network_tuning.hpp").read_text(encoding="utf-8")
    header = (ROOT / "src/runtime/slimevr_output_runtime.hpp").read_text(encoding="utf-8")
    runtime = (ROOT / "src/runtime/slimevr_output_runtime_impl.inc").read_text(encoding="utf-8")
    test = read_native_test(ROOT / "tests/native/test_slimevr_output_runtime.cpp")
    status = (ROOT / "src/serial/tracker_slimevr_commands.cpp").read_text(encoding="utf-8")
    perf = (ROOT / "src/serial/tracker_perf_commands.cpp").read_text(encoding="utf-8")
    motion = (ROOT / "src/serial/tracker_motion_commands.cpp").read_text(encoding="utf-8")
    runtime_test = (ROOT / "src/runtime/runtime_test_runner.cpp").read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")

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
    runtime_code = re.sub(r"//.*?$|/\*.*?\*/", "", runtime, flags=re.MULTILINE | re.DOTALL)
    forbid(runtime_code, "new ", "heap allocation in pressure recovery")

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
    combined = "\n".join((status, perf, motion, runtime_test))
    for field in diagnostics:
        require(combined, field, f"network-pressure diagnostic {field}")


    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-pre0024ad-") as tmp_name:
        tmp = Path(tmp_name)
        # Runtime -O2 and ASan/UBSan variants are owned by test_udp_tx_recovery.
        # Keep this distinct pressure-state stack budget and source admission check.
        run_contract_command([
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

    print("# network_pressure_pacing: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
