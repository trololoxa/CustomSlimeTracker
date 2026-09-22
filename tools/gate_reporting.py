"""Per-command evidence for existing runners. Never a build/result cache."""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Mapping, Sequence

from quality_gate_runtime import run_bounded_process

SCHEMA = "tracker-gate-run-v1"
REPLACE_RETRY_DELAYS_S = (0.01, 0.02, 0.04, 0.08, 0.16, 0.32)
PROGRESS_NOTICE_S = 30.0


def replace_report(temporary: Path, destination: Path) -> int:
    """Retry only Windows access/sharing/lock errors; keep old JSON until commit."""
    for attempt in range(len(REPLACE_RETRY_DELAYS_S) + 1):
        try:
            os.replace(temporary, destination)
            return attempt
        except OSError as exc:
            if (getattr(exc, "winerror", None) not in (5, 32, 33)
                    or attempt == len(REPLACE_RETRY_DELAYS_S)):
                raise
            time.sleep(REPLACE_RETRY_DELAYS_S[attempt])
    raise AssertionError("unreachable")


def command_label(command: Sequence[str]) -> str:
    def name(value: str) -> str:
        return Path(value).name.replace("\n", " ").replace("\r", " ")[:120]
    label = name(command[0])
    source = next((part for part in command[1:] if part.endswith((".py", ".cpp"))), None)
    if source:
        label += " " + name(source)
    if "-e" in command and command.index("-e") + 1 < len(command):
        label += " " + name(command[command.index("-e") + 1])
    return label


def source_snapshot(root: Path) -> dict:
    values = []
    for args in (("rev-parse", "HEAD"), ("status", "--porcelain", "--untracked-files=normal")):
        try:
            result = run_bounded_process(["git", "-C", str(root), *args], timeout_s=5,
                                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            values.append(result.stdout.strip() if result.returncode == 0 else None)
        except (OSError, subprocess.TimeoutExpired):
            values.append(None)
    return {"commit": values[0] if values[0] and re.fullmatch(r"[0-9a-fA-F]{40}", values[0]) else None,
            "dirty": bool(values[1]) if values[1] is not None else None,
            "scope": "startup metadata; no worktree content fingerprint"}


def excerpt(path: Path) -> str:
    # Bounded RAM and console output, including logs with huge single lines.
    with path.open("rb") as stream:
        head = stream.read(4096)
        stream.seek(0, os.SEEK_END)
        size = stream.tell()
        stream.seek(max(4096, size - 8192))
        tail = stream.read(8192) if size > 4096 else b""
    text = head.decode("utf-8", errors="replace")
    if tail:
        text += "\n... [full output in log] ...\n" + tail.decode("utf-8", errors="replace")
    text = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", text)
    lines = [line[:400] for line in text.splitlines() if line.strip()]
    if len(lines) > 12:
        lines = lines[:6] + ["... [full output in log] ..."] + lines[-6:]
    return "\n".join(lines)


class GateReport:
    def __init__(self, root: Path, runner: str, *, verbose: bool = False):
        self.verbose = verbose
        self.runner = runner
        self._last_notice_at = time.monotonic()
        folder = root / "build/gate_runs"
        folder.mkdir(parents=True, exist_ok=True)
        self.directory = Path(tempfile.mkdtemp(prefix=runner + "-", dir=folder))
        self.path = self.directory / "summary.json"
        self.data: dict = {"schema": SCHEMA, "runner": runner, "state": "running",
                           "returncode": None, "scope": "unspecified", "selection": [],
                           "commands": [], "notes": [], "started_at_unix": time.time()}
        self.save()
        self.data["source"] = source_snapshot(root)
        self.save()
        print(f"# report={self.path}", flush=True)

    def save(self) -> None:
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        retries = replace_report(temporary, self.path)
        if retries:
            print(f"# report write recovered after {retries} retry(s): {self.path}", flush=True)

    def configure(self, scope: str, selection: Sequence[str], **metadata: object) -> None:
        self.data.update(scope=scope, selection=list(selection), metadata=metadata)
        self.save()

    def note(self, message: str) -> None:
        self.data["notes"].append(message)
        self.save()

    def run(self, cmd: Sequence[str], *, cwd: Path, timeout_s: float,
            env: Mapping[str, str] | None) -> subprocess.CompletedProcess[str]:
        command = list(cmd)
        log = self.directory / f"{len(self.data['commands']) + 1:04d}.log"
        record = {"command": command, "cwd": str(cwd), "timeout_s": timeout_s,
                  "log": log.name, "state": "running", "returncode": None}
        self.data["commands"].append(record)
        self.save()
        start = time.monotonic()
        code, state = 130, "interrupted"
        label = command_label(command)
        if self.runner == "check-all" or start - self._last_notice_at >= PROGRESS_NOTICE_S:
            print(f"# RUN {log.name}: {label}", flush=True)
            self._last_notice_at = start

        def progress(elapsed: float) -> None:
            now = time.monotonic()
            if now - self._last_notice_at >= PROGRESS_NOTICE_S:
                print(f"# WAIT {log.name}: {label}; elapsed={elapsed:.0f}s; log={log}", flush=True)
                self._last_notice_at = now

        if self.verbose:
            print("$ " + repr(command), flush=True)
        try:
            with log.open("wb") as output:
                try:
                    result = run_bounded_process(command, cwd=cwd, env=env,
                                                 timeout_s=timeout_s, stdout=output,
                                                 stderr=subprocess.STDOUT,
                                                 on_progress=progress, progress_interval_s=10.0)
                    code, state = result.returncode, "completed"
                except subprocess.TimeoutExpired:
                    code, state = 124, "timeout"
                    output.write(f"\ncommand timed out after {timeout_s:g}s\n".encode())
                except OSError as exc:
                    code, state = 127, "launch_error"
                    output.write(f"\ncommand launch failed: {exc}\n".encode(errors="replace"))
        finally:
            record.update(returncode=code, state=state, elapsed_s=round(time.monotonic() - start, 3))
            self.save()
        detail = excerpt(log) if code else ""
        if self.verbose:
            with log.open(encoding="utf-8", errors="replace") as stream:
                shutil.copyfileobj(stream, sys.stdout)
        return subprocess.CompletedProcess(command, code, "", detail + (f"\nlog={log}" if code else ""))

    def finish(self, code: int, *, interrupted: bool = False) -> None:
        self.data.update(returncode=code, state="interrupted" if interrupted else "completed",
                         finished_at_unix=time.time())
        self.save()
        for record in self.data["commands"]:
            if record["returncode"]:
                log = self.directory / record["log"]
                print(f"# command failed: {record['log']} exit={record['returncode']} ({record['state']})")
                if log.exists():
                    print(excerpt(log))
                print(f"# log={log}")
        print(f"# report={self.path}", flush=True)


def failed_checks(path: Path, registered: Mapping[str, str]) -> list[str]:
    """Recover failed IDs only. Never execute stored argv/interpreter paths."""
    if path.stat().st_size > 8 * 1024 * 1024:
        raise ValueError("report too large")
    data = json.loads(path.read_text(encoding="utf-8"))
    if (not isinstance(data, dict) or data.get("schema") != SCHEMA
            or data.get("runner") != "check-all" or data.get("scope") != "focused"
            or data.get("state") != "completed" or type(data.get("returncode")) is not int
            or data.get("returncode") != 1):
        raise ValueError("expected a completed failed focused check_all report")
    selection, commands = data.get("selection"), data.get("commands")
    if not isinstance(selection, list) or not selection or not isinstance(commands, list):
        raise ValueError("invalid report selection/commands")
    if any(not isinstance(s, str) or s not in registered for s in selection):
        raise ValueError("report contains unknown checks; select current checks explicitly")
    by_script = {registered[name]: name for name in selection}
    result, seen = [], set()
    for item in commands:
        if not isinstance(item, dict):
            raise ValueError("invalid command record")
        cmd, code = item.get("command"), item.get("returncode")
        if (not isinstance(cmd, list) or len(cmd) != 2 or not isinstance(cmd[1], str)
                or cmd[1] not in by_script or type(code) is not int
                or item.get("state") not in ("completed", "timeout", "launch_error")):
            raise ValueError("report is incomplete or not a focused check run")
        name = by_script[cmd[1]]
        if name in seen:
            raise ValueError("duplicate check record")
        seen.add(name)
        if code:
            result.append(name)
    if seen != set(selection) or len(selection) != len(seen) or not result:
        raise ValueError("report does not contain a complete failed selection")
    return result
