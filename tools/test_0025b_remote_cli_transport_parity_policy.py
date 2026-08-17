#!/usr/bin/env python3
"""Guard 0025b USB/TCP CLI parity and dirty capture identity."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {label}: {needle}")


def forbid(text: str, needle: str, label: str) -> None:
    if needle in text:
        raise SystemExit(f"forbidden {label}: {needle}")


def main() -> int:
    dispatcher = read("src/serial/tracker_serial_commands.cpp")
    origin = read("src/serial/tracker_command_origin.hpp")
    system = read("src/serial/tracker_system_commands.cpp")
    output = read("src/serial/tracker_output_commands.cpp")
    tests = read("src/serial/tracker_test_commands.cpp")
    config_print = read("src/config/tracker_config_print.cpp")
    capture = read("tools/capture_telnet_log.py")
    capture_tests = read("tools/test_capture_telnet_log.py")
    native = read("tests/native/test_cli_transport_parity.cpp")
    docs = read("docs/wifi_remote_console.md") + read("docs/cli_reference.md")

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
    require(docs, "same command dispatcher", "documented transport parity")
    print("# 0025b_remote_cli_transport_parity_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
