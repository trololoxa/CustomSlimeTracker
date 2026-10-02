#!/usr/bin/env python3
"""Guard udp_tx_recovery SlimeVR UDP TX-pressure recovery semantics."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, forbid, isolate, read_native_test, require, run_contract_command, stack_usage,
)

import re

from quality_gate_runtime import (
    asan_ubsan_environment,
    project_temp_directory,
    strongest_supported_sanitizer_flags,
)

ROOT = Path(__file__).resolve().parents[2]


def compile_runtime(cxx: str, exe: Path, extra: list[str]) -> None:
    run_contract_command(
        [
            cxx,
            "-std=c++20",
            *extra,
            "-I", str(ROOT / "src"),
            "-I", str(ROOT / "tests/native"),
            str(ROOT / "tests/native/test_slimevr_output_runtime.cpp"),
            str(ROOT / "tests/native/sanitizer_runtime_options.cpp"),
            str(ROOT / "src/runtime/slimevr_output_runtime.cpp"),
            str(ROOT / "src/output/slimevr_packet_writer.cpp"),
            str(ROOT / "src/network/wifi_manager.cpp"),
            str(ROOT / "src/network/udp_transport.cpp"),
            "-o", str(exe),
        ],
        check=True,
    )
    run_contract_command([str(exe)], check=True, env=asan_ubsan_environment(ROOT))


def main() -> int:
    udp = (ROOT / "src/network/udp_transport.hpp").read_text(encoding="utf-8")
    esp_h = (ROOT / "src/network/esp32_udp_transport.hpp").read_text(encoding="utf-8")
    esp = (ROOT / "src/network/esp32_udp_transport.cpp").read_text(encoding="utf-8")
    tuning = (ROOT / "src/build_config/network_tuning.hpp").read_text(encoding="utf-8")
    header = (ROOT / "src/runtime/slimevr_output_runtime.hpp").read_text(encoding="utf-8")
    runtime = (ROOT / "src/runtime/slimevr_output_runtime_impl.inc").read_text(encoding="utf-8")
    test = read_native_test(ROOT / "tests/native/test_slimevr_output_runtime.cpp")
    perf = (ROOT / "src/serial/tracker_perf_commands.cpp").read_text(encoding="utf-8")
    status = (ROOT / "src/serial/tracker_slimevr_commands.cpp").read_text(encoding="utf-8")
    motion = (ROOT / "src/serial/tracker_motion_commands.cpp").read_text(encoding="utf-8")
    runtime_test = (ROOT / "src/runtime/runtime_test_runner.cpp").read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")

    require(udp, "virtual int lastSendError() const", "errno-aware transport interface")
    require(esp_h, "int lastSendError() const override", "ESP transport error accessor")
    require(esp, "errno = 0;", "errno capture before WiFiUDP calls")
    require(esp, "lastSendError_ = errno != 0 ? errno : EIO", "physical send error capture")
    require(esp, "lastSendError_ = EMSGSIZE", "partial write classification")

    # udp_tx_recovery introduced errno-aware bounded pressure handling. Successor
    # udp_tx_recovery intentionally replaces the original burst-trigger thresholds,
    # so this predecessor gate protects the durable transport invariants rather
    # than freezing superseded timing constants.
    for needle, label in (
        ("TRACKER_SLIMEVR_TX_BACKOFF_INITIAL_MS", "bounded initial backoff"),
        ("TRACKER_SLIMEVR_TX_BACKOFF_MAX_MS", "bounded maximum backoff"),
        ("TRACKER_SLIMEVR_TX_OTHER_REBIND_CONSECUTIVE_FAILURES", "non-pressure consecutive recovery gate"),
        ("TRACKER_SLIMEVR_TX_FAILURE_WINDOW_ATTEMPTS 32U", "exact outcome window"),
        ("TRACKER_SLIMEVR_TX_FAILURE_WINDOW_FAILURES 8U", "density diagnostic gate"),
        ("TRACKER_SLIMEVR_TX_REBIND_SEND_GRACE_MS", "bounded rebind grace"),
    ):
        require(tuning, needle, label)

    require(header, "enum class PacketSendResult", "tri-state physical send result")
    require(header, "uint32_t txOutcomeWindowBits_", "bounded exact outcome bitmap")
    require(runtime, "packetMayBeDroppedDuringTxBackoff(purpose)", "selective stale-packet backoff")
    require(runtime, "case PacketPurpose::MotionCombined", "combined motion drop policy")
    require(runtime, "case PacketPurpose::Control", "control bypass from pose backoff")
    require(runtime, "isUdpTxPressureError(lastUdpSendError_)", "TX-pressure classification")
    require(runtime, "txFailureWindowExceeded()", "density gate checked on outcomes")
    require(runtime, "recordPhysicalSendOutcome(true, purpose, false, nowMs);", "successful physical attempts in density window")
    require(runtime, "udpTransportRebindRequested_ = true", "session-preserving recovery request")
    require(runtime, "lastUdpTransportRebindMs_", "rebind escalation memory")
    require(runtime, "udpFullReopenEscalations_", "full-reopen escalation counter")

    invalid_packet = isolate(
        runtime,
        "if (!udp_ || !packet.ok || packet.size == 0)",
        "\n\n    if (!udp_->send",
    )
    require(invalid_packet, "txOtherFailures_", "serialization/internal failure diagnostic")
    forbid(invalid_packet, "recordPhysicalSendOutcome", "internal failure counted as physical TX")
    forbid(invalid_packet, "requestUdpTxRecovery", "socket recovery for internal serialization failure")

    rebind = isolate(
        runtime,
        "bool SlimeVROutputRuntime::rebindUdpPreservingSession(",
        "\nbool SlimeVROutputRuntime::isUdpTxPressureError",
    )
    require(rebind, "udp_->stop();", "local socket close")
    require(rebind, "udp_->begin(localPort_)", "same-port local socket bind")
    require(rebind, "serverFoundSendGraceUntilMs_", "post-rebind grace")
    forbid(rebind, "serverFound_ = false", "session invalidation during local rebind")
    forbid(rebind, "resetServerFeatureNegotiation", "feature renegotiation during local rebind")
    forbid(rebind, "markSensorInfoDirty", "SensorInfo invalidation during local rebind")

    forbid(runtime, "udpReopenSuppressedRecentRx", "RX-liveness recovery suppression")
    forbid(runtime, "TRACKER_SLIMEVR_SEND_FAILURE_REOPEN_THRESHOLD", "legacy consecutive-only recovery")
    runtime_code = re.sub(r"//.*?$|/\*.*?\*/", "", runtime, flags=re.MULTILINE | re.DOTALL)
    forbid(runtime_code, "new ", "heap allocation in TX recovery")

    for needle, label in (
        ("sendsAfterFirstPressure", "no-extra-send backoff regression"),
        ("txBackoffDrops == 1u", "backoff counter regression"),
        ("udpTransportRebindSuccesses == 1u", "successful local rebind regression"),
        ("txFailureWindowTrips >= 1u", "intermittent-density diagnostic regression"),
        ("udpFullReopenEscalations == 1u", "post-rebind escalation regression"),
        ("staleRt.status().udpReopenRequests == 1u", "stale RX bounded reopen regression"),
        ("rebindFailRt.status().udpTransportRebindFailures == 1u", "failed rebind regression"),
    ):
        require(test, needle, label)

    perf_on = isolate(
        perf,
        'if (tracker_serial_detail::eqIgnoreCase(argv[1], "on")',
        '\n\n    if (tracker_serial_detail::eqIgnoreCase(argv[1], "off")',
    )
    require(perf_on, "runtimeProfiler->setEnabled(true", "profiler-only perf on")
    for forbidden in ("slimevrRuntime", "wifiManager", "restart(", "resetConnection", "reconnect"):
        forbid(perf_on, forbidden, f"network side effect in perf on: {forbidden}")

    diagnostics = (
        "udp_transport_rebind_successes", "udp_full_reopen_escalations",
        "tx_backoff_drops", "tx_pressure_failures", "tx_failure_window_trips",
        "last_udp_send_error",
    )
    combined = "\n".join((status, perf, motion, runtime_test))
    for field in diagnostics:
        require(combined, field, f"diagnostic {field}")


    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-udp_tx_recovery-") as tmp_name:
        tmp = Path(tmp_name)
        compile_runtime(cxx, tmp / "test_runtime", ["-O2", "-Wall", "-Wextra", "-Werror"])
        sanitizer_name, sanitizer_flags = strongest_supported_sanitizer_flags(cxx, ROOT)
        if sanitizer_flags:
            print(f"# udp_tx_recovery sanitizer={sanitizer_name}")
            compile_runtime(
                cxx,
                tmp / "test_runtime_san",
                ["-O1", "-g", *sanitizer_flags],
            )
        else:
            print("# udp_tx_recovery sanitizer: SKIP (toolchain cannot link ASan/UBSan)")
        run_contract_command(
            [
                cxx, "-std=c++20", "-O2", "-fstack-usage",
                "-I", str(ROOT / "src"),
                "-I", str(ROOT / "tests/native"),
                "-c", str(ROOT / "src/runtime/slimevr_output_runtime.cpp"),
                "-o", str(tmp / "runtime.o"),
            ],
            check=True,
        )
        for symbol, ceiling in (
            ("SlimeVROutputRuntime::update", 128),
            ("SlimeVROutputRuntime::sendPacket", 128),
            ("SlimeVROutputRuntime::rebindUdpPreservingSession", 128),
        ):
            measured = stack_usage(tmp, symbol)
            if measured > ceiling:
                raise SystemExit(f"{symbol} stack {measured} exceeds {ceiling}")

    print("# udp_tx_recovery: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
