#!/usr/bin/env python3
"""Shared process, temporary-directory and sanitizer policy for host gates."""

from __future__ import annotations

import os
import math
import time
import signal
import subprocess
import tempfile
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence


SANITIZER_FLAGS = {
    "none": (),
    "undefined": (
        "-O1",
        "-g",
        "-fsanitize=undefined",
        "-fno-sanitize-recover=undefined",
        "-fno-omit-frame-pointer",
    ),
    "address-undefined": (
        "-O1",
        "-g",
        "-fsanitize=address,undefined",
        "-fno-sanitize-recover=undefined",
        "-fno-omit-frame-pointer",
    ),
    "leak": (
        "-O1",
        "-g",
        "-fsanitize=leak",
        "-fno-omit-frame-pointer",
        "-DTRACKER_ENABLE_LSAN=1",
    ),
}

def _set_option(raw: str, key: str, value: str) -> str:
    """Set one colon-separated sanitizer option without duplicating it."""
    prefix = key + "="
    options = [item for item in raw.split(":") if item and not item.startswith(prefix)]
    options.append(prefix + value)
    return ":".join(options)


def project_temp_root(root: Path, scope: str = "quality-gates") -> Path:
    """Return a repository-local, ignored temp root and make it usable by tools."""
    temp_root = root.resolve() / "build" / "tmp" / scope
    temp_root.mkdir(parents=True, exist_ok=True)
    return temp_root


class ProjectTemporaryDirectory(tempfile.TemporaryDirectory[str]):
    """TemporaryDirectory that restores process-global temp settings on cleanup."""

    def __init__(self, root: Path, prefix: str) -> None:
        self._saved_environment = {
            name: os.environ.get(name) for name in ("TMPDIR", "TEMP", "TMP")
        }
        self._saved_tempdir = tempfile.tempdir
        self._settings_restored = False
        temp_root = project_temp_root(root)

        # Some minimal CI/container images do not provide /tmp. Configure both
        # Python and compiler children for this directory's lifetime so neither
        # tempfile nor GCC falls back to littering the repository root.
        for name in self._saved_environment:
            os.environ[name] = str(temp_root)
        tempfile.tempdir = str(temp_root)
        try:
            super().__init__(
                prefix=prefix,
                dir=temp_root,
                ignore_cleanup_errors=True,
            )
        except BaseException:
            self._restore_settings()
            raise

    def _restore_settings(self) -> None:
        if self._settings_restored:
            return
        for name, value in self._saved_environment.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value
        tempfile.tempdir = self._saved_tempdir
        self._settings_restored = True

    def cleanup(self) -> None:
        try:
            super().cleanup()
        finally:
            self._restore_settings()


def project_temp_directory(root: Path, prefix: str) -> ProjectTemporaryDirectory:
    """Create a self-cleaning temp directory below build/tmp instead of the repo root."""
    return ProjectTemporaryDirectory(root, prefix)


def quality_gate_environment(
    root: Path,
    *,
    scope: str = "quality-gates",
    base: Mapping[str, str] | None = None,
) -> dict[str, str]:
    """Build a child environment with valid temp paths and no Python bytecode litter."""
    env = dict(os.environ if base is None else base)
    temp_root = str(project_temp_root(root, scope))
    env["TMPDIR"] = temp_root
    env["TEMP"] = temp_root
    env["TMP"] = temp_root
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    return env


def asan_ubsan_environment(
    root: Path,
    *,
    scope: str = "sanitizers",
    base: Mapping[str, str] | None = None,
) -> dict[str, str]:
    """Run ASan/UBSan deterministically while leaving leak checks to the LSan gate."""
    env = quality_gate_environment(root, scope=scope, base=base)
    env["ASAN_OPTIONS"] = _set_option(env.get("ASAN_OPTIONS", ""), "detect_leaks", "0")
    env["ASAN_OPTIONS"] = _set_option(env["ASAN_OPTIONS"], "halt_on_error", "1")
    # GCC links liblsan into address-sanitized binaries on Linux. Some builds
    # read the common flag from LSAN_OPTIONS rather than ASAN_OPTIONS.
    env["LSAN_OPTIONS"] = _set_option(env.get("LSAN_OPTIONS", ""), "detect_leaks", "0")
    env["UBSAN_OPTIONS"] = _set_option(env.get("UBSAN_OPTIONS", ""), "halt_on_error", "1")
    env["UBSAN_OPTIONS"] = _set_option(env["UBSAN_OPTIONS"], "print_stacktrace", "1")
    return env


def strongest_supported_sanitizer_flags(
    cxx: str,
    root: Path,
) -> tuple[str, tuple[str, ...]]:
    """Development fallback only: prefer an executable, effective sanitizer.

    Explicit native/release sanitizer gates never call this fallback selector.
    A weaker result is reported by the caller as reduced coverage, not ASan PASS.
    Full probe evidence is retained even when this development path falls back.
    """
    from sanitizer_probe import probe_sanitizer, write_probe_report

    for name in ("address-undefined", "undefined"):
        result = probe_sanitizer(cxx, root, name)
        report = write_probe_report(result, root)
        print(f"# sanitizer capability {name}={result.status} report={report}")
        if result.supported:
            # Preserve caller optimization/debug flags in legacy development gates.
            return name, tuple(flag for flag in SANITIZER_FLAGS[name] if flag not in ("-O1", "-g"))
    return "none", ()


def lsan_environment(
    root: Path,
    *,
    scope: str = "leak-sanitizer",
    base: Mapping[str, str] | None = None,
) -> dict[str, str]:
    """Return an environment for an explicit leak-only sanitizer run."""
    env = quality_gate_environment(root, scope=scope, base=base)
    env["LSAN_OPTIONS"] = _set_option(env.get("LSAN_OPTIONS", ""), "exitcode", "23")
    env["LSAN_OPTIONS"] = _set_option(env["LSAN_OPTIONS"], "detect_leaks", "1")
    return env


def _kill_process_tree(process: subprocess.Popen[Any]) -> None:
    """Best-effort force termination of a bounded command and its descendants."""
    if os.name == "nt":
        # CREATE_NEW_PROCESS_GROUP alone does not make Popen.kill() recursive.
        # taskkill /T owns the Windows process-tree contract; killing the root
        # process remains a fallback when taskkill is unavailable.
        try:
            killer = subprocess.Popen(
                ["taskkill", "/PID", str(process.pid), "/T", "/F"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            try:
                killer.communicate(timeout=10.0)
            except subprocess.TimeoutExpired:
                killer.kill()
                killer.communicate()
        except OSError:
            pass
        if process.poll() is None:
            process.kill()
        return

    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    except OSError:
        if process.poll() is None:
            process.kill()


def _kill_and_collect(process: subprocess.Popen[Any]) -> tuple[Any, Any]:
    _kill_process_tree(process)
    try:
        return process.communicate(timeout=10.0)
    except subprocess.TimeoutExpired:
        process.kill()
        return process.communicate()


def run_bounded_process(
    args: Sequence[str],
    *,
    cwd: str | os.PathLike[str] | None = None,
    env: Mapping[str, str] | None = None,
    timeout_s: float,
    stdout: Any = None,
    stderr: Any = None,
    text: bool = False,
    on_progress: Callable[[float], None] | None = None,
    progress_interval_s: float = 30.0,
) -> subprocess.CompletedProcess[Any]:
    """Run one command; optional wait notices never renew its absolute deadline."""
    if on_progress is not None and (
            not math.isfinite(progress_interval_s) or progress_interval_s <= 0
            or not math.isfinite(timeout_s) or timeout_s <= 0):
        raise ValueError("progress interval and timeout must be finite and positive")
    popen_kwargs: dict[str, Any] = {
        "cwd": cwd,
        "env": env,
        "stdout": stdout,
        "stderr": stderr,
        "text": text,
    }
    if os.name == "nt":
        popen_kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        popen_kwargs["start_new_session"] = True

    process = subprocess.Popen(list(args), **popen_kwargs)
    try:
        if on_progress is None:
            captured_stdout, captured_stderr = process.communicate(timeout=timeout_s)
        else:
            started = time.monotonic()
            deadline = started + timeout_s
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise subprocess.TimeoutExpired(list(args), timeout_s)
                try:
                    captured_stdout, captured_stderr = process.communicate(
                        timeout=min(progress_interval_s, remaining))
                    break
                except subprocess.TimeoutExpired:
                    now = time.monotonic()
                    if now >= deadline:
                        raise
                    on_progress(now - started)
    except subprocess.TimeoutExpired:
        captured_stdout, captured_stderr = _kill_and_collect(process)
        raise subprocess.TimeoutExpired(
            list(args),
            timeout_s,
            output=captured_stdout,
            stderr=captured_stderr,
        )
    except BaseException:
        # The child is in a new session/process group, so Ctrl-C or another
        # interruption delivered to this Python process may not reach compiler
        # descendants. Never abandon that tree while unwinding.
        _kill_and_collect(process)
        raise
    return subprocess.CompletedProcess(
        list(args),
        process.returncode,
        captured_stdout,
        captured_stderr,
    )
