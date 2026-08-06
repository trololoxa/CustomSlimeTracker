#!/usr/bin/env python3
"""Capture a session-bound LOGVER3 static or runtime diagnostic over TCP."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import socket
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "replay"))
from logver3_contract import validate_logver3  # noqa: E402

GIT_RE = re.compile(r"[0-9a-f]{40}\Z")
WORKTREE_RE = re.compile(r"[0-9a-f]{8}\Z")
TELNET_IAC_NOP = b"\xff\xf1"
KEEPALIVE_INTERVAL_S = 5.0


class CaptureError(RuntimeError):
    pass


class CaptureValidationError(CaptureError):
    def __init__(self, report: dict[str, object]) -> None:
        self.report = report
        failures = report.get("failures")
        if isinstance(failures, list) and failures:
            detail = "; ".join(str(item) for item in failures[:5])
        else:
            detail = "validator returned passed=false"
        super().__init__(f"strict LOGVER3 validation failed: {detail}")


class CaptureSocket:
    def __init__(self, host: str, port: int, max_bytes: int) -> None:
        self.socket = socket.create_connection((host, port), timeout=10.0)
        self.socket.settimeout(1.0)
        self.raw = bytearray()
        self.pending = bytearray()
        self.lines: list[str] = []
        self.max_bytes = max_bytes
        self.last_keepalive = 0.0

    def close(self) -> None:
        self.socket.close()

    def send(self, command: str) -> None:
        if "\r" in command or "\n" in command:
            raise CaptureError("command contains a newline")
        self.socket.sendall(command.encode("ascii") + b"\n")
        self.last_keepalive = time.monotonic()

    def _maybe_keepalive(self, *, force: bool = False) -> None:
        now = time.monotonic()
        if force or now - self.last_keepalive >= KEEPALIVE_INTERVAL_S:
            # IAC NOP is consumed by the firmware Telnet filter, renews the
            # application lease, and never enters the ASCII command buffer.
            self.socket.sendall(TELNET_IAC_NOP)
            self.last_keepalive = now

    def _receive(self) -> None:
        try:
            chunk = self.socket.recv(65536)
        except socket.timeout:
            return
        if not chunk:
            raise CaptureError("remote console disconnected")
        self.raw.extend(chunk)
        if len(self.raw) > self.max_bytes:
            raise CaptureError(f"capture exceeded {self.max_bytes} bytes")
        self.pending.extend(chunk)
        while b"\n" in self.pending:
            raw_line, _, remainder = self.pending.partition(b"\n")
            self.pending = bytearray(remainder)
            try:
                line = raw_line.rstrip(b"\r").decode("utf-8", errors="strict")
            except UnicodeError as exc:
                raise CaptureError(f"non-UTF-8 console output: {exc}") from exc
            self.lines.append(line)

    def wait_for(self, predicate: Callable[[list[str]], bool], timeout_s: float, description: str) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate(self.lines):
                return
            self._maybe_keepalive()
            self._receive()
        raise CaptureError(f"timeout waiting for {description}")

    def wait_line(self, text: str, timeout_s: float = 10.0) -> None:
        start = len(self.lines)
        self.wait_for(lambda lines: any(text in line for line in lines[start:]), timeout_s, text)


def key_values(lines: list[str]) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in lines:
        if "=" in line and "," not in line and not line.startswith("#"):
            key, value = line.split("=", 1)
            if key and " " not in key:
                result[key] = value
    return result


def last_pipeline(lines: list[str]) -> tuple[int, int, int] | None:
    for line in reversed(lines):
        if not line.startswith("LOGSTAT,PIPELINE,"):
            continue
        parts = line.split(",")
        if len(parts) != 8:
            return None
        try:
            return int(parts[2]), int(parts[4]), int(parts[5])
        except ValueError:
            return None
    return None


def atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def paths_alias(first: Path, second: Path) -> bool:
    if first.resolve() == second.resolve():
        return True
    try:
        return first.exists() and second.exists() and first.samefile(second)
    except OSError:
        return False


def promote_validated_capture(
    path: Path,
    data: bytes,
    validator: Callable[[Path], dict[str, object]],
) -> dict[str, object]:
    """Replace *path* only after *data* passes validation.

    The candidate lives beside the destination so the final rename remains
    atomic. A rejected candidate cannot destroy a previous known-good capture
    even when the caller explicitly allowed replacement.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=path.name + ".candidate-",
        suffix=".tmp",
        dir=path.parent,
    )
    candidate = Path(temporary)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        report = validator(candidate)
        if report.get("passed") is not True:
            raise CaptureValidationError(report)
        os.replace(candidate, path)
        return report
    finally:
        candidate.unlink(missing_ok=True)


def _write_candidate(path: Path, data: bytes, label: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=path.name + f".{label}-",
        suffix=".tmp",
        dir=path.parent,
    )
    candidate = Path(temporary)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
    except BaseException:
        candidate.unlink(missing_ok=True)
        raise
    return candidate


def _backup_path(path: Path) -> Path:
    fd, temporary = tempfile.mkstemp(
        prefix=path.name + ".rollback-", suffix=".tmp", dir=path.parent
    )
    os.close(fd)
    backup = Path(temporary)
    backup.unlink()
    return backup


def _best_effort_unlink(path: Path | None) -> None:
    if path is None:
        return
    try:
        path.unlink(missing_ok=True)
    except OSError:
        # A stale candidate/backup is safer than converting a successfully
        # published or restored pair into a false failure.
        pass


def promote_validated_capture_bundle(
    capture_path: Path,
    manifest_path: Path,
    capture_data: bytes,
    validator: Callable[[Path], dict[str, object]],
    manifest_factory: Callable[[dict[str, object], bytes], dict[str, object]],
) -> dict[str, object]:
    """Validate and prepare both files before replacing either destination.

    Replacement errors roll both paths back to their previous generation. A
    manifest construction/write failure therefore cannot publish a new log by
    itself. The manifest hash remains the authoritative pair-consistency check
    after an unclean host power loss between the two filesystem renames.
    """
    if paths_alias(capture_path, manifest_path):
        raise ValueError("capture and manifest paths must be distinct")

    capture_candidate = _write_candidate(capture_path, capture_data, "candidate")
    manifest_candidate: Path | None = None
    capture_backup: Path | None = None
    manifest_backup: Path | None = None
    capture_existed = capture_path.exists()
    manifest_existed = manifest_path.exists()
    try:
        report = validator(capture_candidate)
        if report.get("passed") is not True:
            raise CaptureValidationError(report)
        manifest_data = manifest_factory(report, capture_data)
        manifest_bytes = (
            json.dumps(manifest_data, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
        ).encode("utf-8")
        # Serialization and both fsyncs complete before the first destination
        # can change.
        manifest_candidate = _write_candidate(manifest_path, manifest_bytes, "candidate")

        capture_backup = _backup_path(capture_path)
        manifest_backup = _backup_path(manifest_path)
        capture_moved = False
        manifest_moved = False
        capture_installed = False
        manifest_installed = False
        try:
            if capture_existed:
                os.replace(capture_path, capture_backup)
                capture_moved = True
            if manifest_existed:
                os.replace(manifest_path, manifest_backup)
                manifest_moved = True
            os.replace(capture_candidate, capture_path)
            capture_installed = True
            os.replace(manifest_candidate, manifest_path)
            manifest_installed = True
        except BaseException as install_error:
            rollback_errors: list[str] = []

            # When an old generation exists, os.replace below can overwrite a
            # partially installed new file. If no old generation exists, the
            # newly created destination must be removed explicitly.
            if capture_installed and not capture_moved:
                try:
                    capture_path.unlink(missing_ok=True)
                except OSError as exc:
                    rollback_errors.append(f"remove new capture: {exc}")
            if manifest_installed and not manifest_moved:
                try:
                    manifest_path.unlink(missing_ok=True)
                except OSError as exc:
                    rollback_errors.append(f"remove new manifest: {exc}")

            # Attempt both restorations independently. A failure restoring the
            # log must never prevent restoration of the manifest (or vice
            # versa), and an unrecovered backup must remain on disk.
            if capture_moved and capture_backup.exists():
                try:
                    os.replace(capture_backup, capture_path)
                except OSError as exc:
                    rollback_errors.append(
                        f"restore capture from {capture_backup}: {exc}"
                    )
            if manifest_moved and manifest_backup.exists():
                try:
                    os.replace(manifest_backup, manifest_path)
                except OSError as exc:
                    rollback_errors.append(
                        f"restore manifest from {manifest_backup}: {exc}"
                    )
            if rollback_errors:
                raise CaptureError(
                    "capture bundle promotion failed and rollback is incomplete; "
                    + "; ".join(rollback_errors)
                ) from install_error
            raise

        # The new pair is complete. Old backups are no longer authoritative;
        # failure to remove one is harmless and leaves a recoverable stale file.
        _best_effort_unlink(capture_backup)
        _best_effort_unlink(manifest_backup)
        return report
    finally:
        _best_effort_unlink(capture_candidate)
        _best_effort_unlink(manifest_candidate)
        # Backups are deliberately not removed here. Normal success cleans
        # them above; successful rollback moves them back; incomplete rollback
        # retains the only recoverable old generation for the operator.


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture a cable-free LOGVER3 diagnostic run")
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=7777)
    parser.add_argument("--seconds", type=int, default=600)
    parser.add_argument("--rate", type=int, default=20, choices=(20,))
    parser.add_argument("--mode", default="full", choices=("full",))
    parser.add_argument("--capture", default="static", choices=("static", "runtime"))
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--max-capture-mib", type=int, default=64)
    args = parser.parse_args()

    if not 1 <= args.port <= 65535:
        parser.error("--port must be 1..65535")
    if not 60 <= args.seconds <= 900:
        parser.error("--seconds must be 60..900 for remote capture")
    if args.max_capture_mib <= 0:
        parser.error("--max-capture-mib must be positive")
    output = args.output.resolve()
    manifest = (args.manifest or output.with_suffix(output.suffix + ".manifest.json")).resolve()
    if paths_alias(output, manifest):
        parser.error("--output and --manifest must name different files")
    for path in (output, manifest):
        if path.exists() and not args.overwrite:
            parser.error(f"refusing to overwrite {path}; pass --overwrite")

    commands: list[str] = []
    started_utc = datetime.now(timezone.utc).isoformat()
    transport: CaptureSocket | None = None
    log_started = False
    test_started = False
    preflight: dict[str, dict[str, str]] = {}
    try:
        transport = CaptureSocket(args.host, args.port, args.max_capture_mib * 1024 * 1024)
        transport._maybe_keepalive(force=True)
        transport.wait_line("# Type: help", 15.0)

        def command(text: str, marker: str, timeout_s: float = 10.0) -> list[str]:
            start = len(transport.lines)
            commands.append(text)
            transport.send(text)
            transport.wait_line(marker, timeout_s)
            return transport.lines[start:]

        command("version", "command_session=")
        identity = key_values(transport.lines)
        required_identity = {
            "build_profile": "ProductionDiag",
            "build_pio_env": "BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG",
            "build_dirty": "no",
        }
        for key, expected in required_identity.items():
            if identity.get(key) != expected:
                raise CaptureError(f"{key}={identity.get(key)!r}, expected {expected!r}")
        if not GIT_RE.fullmatch(identity.get("build_git_head", "")):
            raise CaptureError("build_git_head is unknown or not a full clean commit")
        if identity.get("build_git") != identity.get("build_git_head"):
            raise CaptureError("clean build_git identity does not equal build_git_head")
        if not WORKTREE_RE.fullmatch(identity.get("build_worktree", "")):
            raise CaptureError("build_worktree is not an 8-hex source fingerprint")
        if identity.get("command_origin") != "remote_tcp":
            raise CaptureError("version response did not originate from remote_tcp")
        try:
            command_session = int(identity.get("command_session", "0"), 10)
        except ValueError as exc:
            raise CaptureError("command_session is not an integer") from exc
        if command_session <= 0:
            raise CaptureError("command_session is not a live non-zero session")

        mag_runtime = key_values(command("mag status", "mag_hub_error="))
        preflight["mag_runtime"] = mag_runtime

        mag_processed: dict[str, str] = {}
        for attempt in range(15):
            mag_processed = key_values(command("mag processed", "mag_preflight_complete=yes"))
            if (mag_processed.get("trusted") == "yes" and
                    mag_processed.get("heading_valid") == "yes" and
                    mag_processed.get("field_trusted_for_yaw") == "yes" and
                    mag_processed.get("yaw_apply_allowed") == "yes"):
                break
            if attempt != 14:
                time.sleep(1.0)
        preflight["mag_processed"] = mag_processed

        bias = key_values(command("bias status", "current_bias_dps="))
        network = key_values(command("net status", "rssi_dbm="))
        slime = key_values(command("slime status", "# use 'slime debug' for full counters/timestamps"))
        preflight["bias"] = bias
        preflight["network"] = network
        preflight["slime"] = slime

        if args.capture == "static":
            for key in (
                "mag_runtime_enabled", "mag_hub_initialized", "mag_fifo_armed", "mag_last_init_ok"
            ):
                if mag_runtime.get(key) != "yes":
                    raise CaptureError(
                        f"magnetometer preflight failed: {key}={mag_runtime.get(key)!r}"
                    )
            try:
                mag_enable_failures = int(mag_runtime.get("mag_enable_failures", "-1"), 10)
                mag_runtime_samples = int(mag_runtime.get("mag_runtime_samples", "0"), 10)
            except ValueError as exc:
                raise CaptureError("magnetometer preflight counters are not integers") from exc
            if mag_enable_failures != 0 or mag_runtime_samples <= 0:
                raise CaptureError(
                    "magnetometer preflight failed: "
                    f"enable_failures={mag_enable_failures}, runtime_samples={mag_runtime_samples}"
                )
            for key in (
                "processed_valid", "trusted", "config_cal_valid", "config_axis_valid",
                "heading_valid", "field_trusted_for_yaw", "field_reference_valid",
                "mag_ref_valid", "yaw_enabled", "yaw_apply_enabled", "yaw_gate_open",
                "yaw_apply_allowed",
            ):
                if mag_processed.get(key) != "yes":
                    raise CaptureError(
                        f"magnetometer preflight failed: {key}={mag_processed.get(key)!r}"
                    )
            source = bias.get("bias_source")
            if bias.get("base_bias_valid") != "yes" or source in {None, "none", "rt"}:
                raise CaptureError(f"bias preflight failed: source={source!r}")
            if source in {"temp", "temp+rt"}:
                for key in (
                    "temp_comp_valid", "temp_comp_enabled", "temp_comp_has_calibrated_range"
                ):
                    if bias.get(key) != "yes":
                        raise CaptureError(f"bias preflight failed: {key}={bias.get(key)!r}")
                if bias.get("temp_comp_out_of_range") != "no":
                    raise CaptureError("bias preflight failed: temperature is outside calibrated range")
            if network.get("connected") != "yes":
                raise CaptureError("network preflight failed: Wi-Fi is not connected")
            for key in ("wifi_connected", "udp_ready", "server_found"):
                if slime.get(key) != "yes":
                    raise CaptureError(f"SlimeVR preflight failed: {key}={slime.get(key)!r}")

        command("console reset", "# OK console output queues and counters reset")
        command("log reset", "# OK log counters reset")
        command(f"log rate {args.rate}", "# OK log rate set")
        commands.append(f"log start {args.mode}")
        transport.send(commands[-1])
        transport.wait_line("LOGFMT,LOGSUM,", 15.0)
        log_started = True

        command(f"test {args.capture} {args.seconds}", f"# OK {args.capture} test started")
        test_started = True
        transport.wait_line(f"{args.capture.upper()} TEST DONE", args.seconds + 180.0)
        test_started = False

        command("log finish", "# OK log producer stopped")
        command(
            f"test summary {args.capture}",
            f"TESTSUM,{args.capture},",
            15.0,
        )
        drained = False
        for _ in range(30):
            time.sleep(0.1)
            commands.append("log summary")
            transport.send("log summary")
            start = len(transport.lines)
            transport.wait_for(
                lambda lines: any(line.startswith("LOGSTAT,DROPS,") for line in lines[start:]),
                10.0,
                "LOGSTAT,DROPS",
            )
            pipeline = last_pipeline(transport.lines)
            if pipeline is not None and pipeline[0] == 0 and pipeline[1] == pipeline[2]:
                drained = True
                break
        if not drained:
            raise CaptureError("deferred log pipeline did not drain after log finish")

        command("log off", "# OK log stopped")
        log_started = False
        status = key_values(command("test status", "runtime_report_ready="))
        if status.get("test_active") != "no" or status.get("runtime_test_active") != "no":
            raise CaptureError("test lifecycle did not return to inactive state")
        commands.append("console status")
        transport.send("console status")
        transport.wait_line("remote_console_output_pending_drop_notice_records=", 15.0)
        # Collect the tail already admitted after the final status marker.
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            transport._receive()

        capture_bytes = bytes(transport.raw)
        finished_utc = datetime.now(timezone.utc).isoformat()

        def validate(candidate: Path) -> dict[str, object]:
            return validate_logver3(
                candidate,
                capture_kind=args.capture,
                min_duration_s=max(0.0, args.seconds - 10.0),
                min_rate_hz=19.0,
                max_rate_hz=21.0,
                max_record_age_us=250_000,
                expected_profile="ProductionDiag",
                expected_environment="BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG",
                require_console_status=True,
            )

        def make_manifest(report: dict[str, object], data: bytes) -> dict[str, object]:
            return {
                "capture_contract": f"LOGVER3-E1-{args.capture}-v1",
                "capture_kind": args.capture,
                "capture_sha256": hashlib.sha256(data).hexdigest(),
                "capture_bytes": len(data),
                "started_utc": started_utc,
                "finished_utc": finished_utc,
                "host": args.host,
                "port": args.port,
                "requested_seconds": args.seconds,
                "requested_rate_hz": args.rate,
                "requested_mode": args.mode,
                "commands": commands,
                "firmware_identity": identity,
                "preflight": preflight,
                "validation": report,
            }

        report = promote_validated_capture_bundle(
            output,
            manifest,
            capture_bytes,
            validate,
            make_manifest,
        )
        print(json.dumps({"capture": str(output), "manifest": str(manifest), **report}, indent=2))
        return 0
    except (CaptureError, OSError) as exc:
        print(f"capture failed: {exc}", file=sys.stderr)
        if transport is not None and transport.raw:
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
            partial = output.with_name(f"{output.name}.partial-{stamp}")
            partial_manifest = partial.with_suffix(partial.suffix + ".json")
            try:
                atomic_write(partial, bytes(transport.raw))
                atomic_write(
                    partial_manifest,
                    (json.dumps({
                        "capture_contract": f"LOGVER3-E1-{args.capture}-v1",
                        "capture_kind": args.capture,
                        "complete": False,
                        "error": str(exc),
                        "capture_sha256": hashlib.sha256(bytes(transport.raw)).hexdigest(),
                        "capture_bytes": len(transport.raw),
                        "started_utc": started_utc,
                        "failed_utc": datetime.now(timezone.utc).isoformat(),
                        "host": args.host,
                        "port": args.port,
                        "commands": commands,
                        "preflight": preflight,
                        "validation": exc.report if isinstance(exc, CaptureValidationError) else None,
                    }, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode("utf-8"),
                )
                print(f"partial capture saved: {partial}", file=sys.stderr)
            except OSError as save_exc:
                print(f"could not save partial capture: {save_exc}", file=sys.stderr)
        return 2
    finally:
        if transport is not None:
            if test_started:
                try:
                    transport.send("test stop")
                except OSError:
                    pass
            if log_started:
                for cleanup in ("log finish", "log off"):
                    try:
                        transport.send(cleanup)
                    except OSError:
                        break
            transport.close()


if __name__ == "__main__":
    raise SystemExit(main())
