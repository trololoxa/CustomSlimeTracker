"""Bounded, fail-closed host sanitizer self-checks; no firmware dependencies."""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Sequence

from quality_gate_runtime import (
    SANITIZER_FLAGS, asan_ubsan_environment, lsan_environment,
    project_temp_directory, run_bounded_process,
)


@dataclass
class ProbeResult:
    sanitizer: str
    compiler: str
    status: str = "unavailable"
    reason: str = "probe incomplete"
    stages: list[dict[str, object]] = field(default_factory=list)

    @property
    def supported(self) -> bool:
        return self.status == "supported"


def _run(result: ProbeResult, stage: str, command: list[str], root: Path,
         env: dict[str, str], timeout_s: float) -> subprocess.CompletedProcess[str]:
    try:
        proc = run_bounded_process(command, cwd=root, env=env,
                                   timeout_s=timeout_s, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True)
        status = "completed"
    except subprocess.TimeoutExpired as exc:
        def decoded(value: object) -> str:
            return value.decode(errors="replace") if isinstance(value, bytes) else str(value or "")
        proc = subprocess.CompletedProcess(command, 124, decoded(exc.stdout), decoded(exc.stderr))
        status = "timeout"
    except OSError as exc:
        proc = subprocess.CompletedProcess(command, 127, "", str(exc))
        status = "launch_error"
    result.stages.append({"stage": stage, "command": command, "status": status,
                          "returncode": proc.returncode,
                          "stdout": proc.stdout or "", "stderr": proc.stderr or ""})
    return proc


def probe_sanitizer(cxx: str, root: Path, name: str, *,
                    extra_flags: Sequence[str] = (), timeout_s: float = 30.0) -> ProbeResult:
    """Require clean execution AND the expected diagnostic on each known fault.

    ASan+UBSan proves both runtimes independently. Missing diagnostics, ordinary
    crashes, linker errors and timeouts cannot establish sanitizer support.
    There are no retries: an unstable runtime is not suitable gate evidence.
    """
    if name not in SANITIZER_FLAGS or name == "none":
        raise ValueError(f"not a sanitizer gate: {name}")
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("probe timeout must be finite and positive")
    result = ProbeResult(name, cxx)
    kinds = ("address", "undefined") if name == "address-undefined" else (name,)
    markers = {"address": "AddressSanitizer: heap-buffer-overflow",
               "undefined": "runtime error: signed integer overflow",
               "leak": "LeakSanitizer: detected memory leaks"}
    env = lsan_environment(root) if name == "leak" else asan_ubsan_environment(root)
    with project_temp_directory(root, "sanitizer-probe-") as raw:
        for kind in kinds:
            source = root / "tests" / "fixtures" / "sanitizers" / f"{kind}.cpp"
            exe = Path(raw) / (kind + (".exe" if os.name == "nt" else ""))
            # Match the native suite's compiled-in ASan/LSan separation too;
            # environment-only options are not effective on every host runner.
            options = root / "tests" / "native" / "sanitizer_runtime_options.cpp"
            command = [cxx, "-std=c++20", *extra_flags, *SANITIZER_FLAGS[name],
                       str(source), str(options), "-o", str(exe)]
            compiled = _run(result, f"{kind}:compile", command, root, env, timeout_s)
            if compiled.returncode != 0 or not exe.is_file() or exe.stat().st_size == 0:
                result.reason = f"{kind}: compile/link failed or executable missing"
                return result
            clean = _run(result, f"{kind}:clean", [str(exe)], root, env, timeout_s)
            if clean.returncode != 0:
                result.reason = f"{kind}: clean executable failed"
                return result
            fault = _run(result, f"{kind}:fault", [str(exe), "fault"], root, env, timeout_s)
            if (result.stages[-1]["status"] != "completed" or fault.returncode == 0
                    or markers[kind] not in (fault.stdout or "") + (fault.stderr or "")):
                result.reason = f"{kind}: expected nonzero exit and sanitizer diagnostic missing"
                return result
    result.status = "supported"
    result.reason = "clean execution and deliberate fault detection verified"
    return result


def write_probe_report(result: ProbeResult, root: Path) -> Path:
    """Retain complete evidence outside the short console summary; no secrets dump."""
    directory = root / "build" / "sanitizer-probes"
    directory.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", suffix=".json",
                                     prefix=result.sanitizer + "-", dir=directory,
                                     delete=False) as output:
        json.dump(asdict(result), output, ensure_ascii=False, indent=2)
        output.write("\n")
        return Path(output.name)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default=os.environ.get("CXX") or "g++")
    parser.add_argument("--sanitizer", choices=[n for n in SANITIZER_FLAGS if n != "none"],
                        action="append", required=True)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        parser.error("--timeout-s must be finite and positive")
    root = Path(__file__).resolve().parents[1]
    cxx = shutil.which(args.cxx) or args.cxx
    failed = False
    for name in dict.fromkeys(args.sanitizer):
        result = probe_sanitizer(cxx, root, name, timeout_s=args.timeout_s)
        report = write_probe_report(result, root)
        print(f"{'PASS' if result.supported else 'FAIL'} sanitizer={name}: {result.reason}")
        print(f"report={report}")
        failed |= not result.supported
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
