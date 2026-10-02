#!/usr/bin/env python3
"""Guard remote_console_parity USB/TCP CLI parity and dirty capture identity."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import forbid, require


ROOT = Path(__file__).resolve().parents[2]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def main() -> int:
    dispatcher = read("src/serial/tracker_serial_commands.cpp")
    origin = read("src/serial/tracker_command_origin.hpp")
    system = read("src/serial/tracker_system_commands.cpp")
    output = read("src/serial/tracker_output_commands.cpp")
    tests = read("src/serial/tracker_test_commands.cpp")
    config_print = read("src/config/tracker_config_print.cpp")
    capture = read("tools/capture_telnet_log.py")
    capture_tests = read("tests/tooling/test_capture_telnet_log.py")
    native = read("tests/native/test_cli_transport_parity.cpp")

    forbid(dispatcher, "trackerRemoteDiagnosticCommandAllowed", "remote dispatcher allowlist")
    forbid(dispatcher, "remote command not allowed", "remote rejection path")
    forbid(origin, "trackerRemoteDiagnosticCommandAllowed", "allowlist API")
    forbid(system, "TRACKER REMOTE DIAGNOSTIC COMMANDS", "restricted remote help")
    require(system, "origin=", "common help origin identity")

    require(output, "constexpr uint32_t maxHz = 200u", "shared log-rate limit")
    forbid(output, "invalid remote log rate", "remote-only log-rate error")
    forbid(output, "log is owned by another session", "cross-transport log control restriction")
    require(tests, "constexpr uint32_t maxSeconds = 21600UL", "shared test duration")
    require(tests, "constexpr bool force = true", "shared force-stop behavior")
    forbid(tests, "invalid remote duration", "remote-only duration error")

    require(capture, "1 <= args.seconds <= 21600", "capture duration parity")
    require(capture, "validate_firmware_identity", "identity validator")
    require(capture, 'build_dirty == "yes"', "dirty build acceptance")
    require(capture, 'f"{git_head}+{worktree}-dirty"', "dirty fingerprint binding")
    require(capture_tests, "test_dirty_identity_is_accepted_and_fingerprint_bound", "dirty identity regression")
    require(native, '"21600"', "remote 21600-second native boundary")
    require(native, '"200"', "remote 200-Hz native boundary")

    require(config_print, "#if TRACKER_ENABLE_FULL_CONFIG_PRINT", "compact/full config-print TU guard")
    print("# remote_console_parity: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
