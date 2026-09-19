"""Bounded Tracker environment checks. Does not install, flash or mutate config."""
from __future__ import annotations

import argparse
import configparser
import hashlib
import json
import math
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path

from lock_dev_wheels import bootstrap_version
from quality_gate_runtime import project_temp_directory, quality_gate_environment, run_bounded_process
from sanitizer_probe import probe_sanitizer

ROOT = Path(__file__).resolve().parents[1]
OPTIONAL_TOOLS = ("clangd", "gdb", "openocd", "llvm-symbolizer")


@dataclass
class Doctor:
    root: Path
    profile: str
    timeout: float
    checks: list[dict[str, object]] = field(default_factory=list)
    commands: list[dict[str, object]] = field(default_factory=list)
    env: dict[str, str] = field(default_factory=dict)

    def add(self, name: str, status: str, detail: str) -> None:
        self.checks.append({"name": name, "status": status, "detail": detail})

    def run(self, label: str, command: list[str]) -> subprocess.CompletedProcess[str]:
        try:
            result = run_bounded_process(command, cwd=self.root, env=self.env,
                                         timeout_s=self.timeout, stdout=subprocess.PIPE,
                                         stderr=subprocess.PIPE, text=True)
            state = "completed"
        except subprocess.TimeoutExpired as exc:
            def decoded(v: object) -> str:
                return v.decode(errors="replace") if isinstance(v, bytes) else str(v or "")
            result = subprocess.CompletedProcess(command, 124, decoded(exc.stdout), decoded(exc.stderr))
            state = "timeout"
        except OSError as exc:
            result = subprocess.CompletedProcess(command, 127, "", str(exc))
            state = "launch_error"
        self.commands.append({"label": label, "command": command, "state": state,
                              "returncode": result.returncode,
                              "stdout": result.stdout or "", "stderr": result.stderr or ""})
        return result

    def version(self, label: str, executable: str | None, required: bool) -> str | None:
        if not executable:
            self.add(label, "FAIL" if required else "SKIP", "executable not found")
            return None
        result = self.run(label, [executable, "--version"])
        value = (result.stdout or result.stderr).strip()
        good = result.returncode == 0 and bool(value)
        self.add(label, "PASS" if good else "FAIL" if required else "WARN",
                 f"{executable}: {value.splitlines()[0][:200]}" if good else "version command failed; see report")
        return value if good else None


def executable(explicit: str | None, names: tuple[str, ...]) -> str | None:
    # An explicit invalid path never falls back to an unrelated PATH compiler.
    if explicit is not None:
        return shutil.which(explicit)
    return next((p for n in names if (p := shutil.which(n))), None)


def target_pins(root: Path) -> dict[str, tuple[str, str]]:
    config = configparser.ConfigParser(interpolation=None, inline_comment_prefixes=(";", "#"))
    config.read(root / "platformio.ini", encoding="utf-8")
    base = config["env:BOARD_LOLIN_C3_MINI"]
    specs = [base["platform"].strip(), *base["platform_packages"].split()]
    pins = {}
    for spec in specs:
        match = re.fullmatch(r"([A-Za-z0-9_-]+)/([A-Za-z0-9_-]+)@([0-9][A-Za-z0-9.+-]*)", spec)
        if not match:
            raise ValueError("target packages must have exact owner/name@version pins")
        owner, name, version = match.groups()
        if name in pins:
            raise ValueError("duplicate target package pin")
        pins[name] = (owner, version)
    return pins


def core_directory(raw: str) -> Path:
    data = json.loads(raw)
    if not isinstance(data, dict):
        raise ValueError("PlatformIO system info must be an object")
    entry = data.get("core_dir")
    if isinstance(entry, dict):
        entry = entry.get("value")
    if not isinstance(entry, str) or not entry:
        raise ValueError("PlatformIO system info has no usable core_dir")
    directory = Path(entry)
    if not directory.is_absolute():
        raise ValueError("PlatformIO core_dir must be absolute")
    return directory


def check_target(doctor: Doctor, pio: str | None) -> None:
    try:
        expected = bootstrap_version(doctor.root)
    except (OSError, ValueError) as exc:
        doctor.add("platformio.pin", "FAIL", str(exc))
        return
    version = doctor.version("platformio.core", pio, required=True)
    if version is None:
        return
    if not re.search(r"\bversion\s+" + re.escape(expected) + r"\s*$", version, re.IGNORECASE):
        doctor.add("platformio.pin", "FAIL", f"expected Core {expected}; no automatic downgrade/update")
        return
    doctor.add("platformio.pin", "PASS", expected)
    info = doctor.run("platformio.paths", [pio, "system", "info", "--json-output"])
    try:
        if info.returncode:
            raise ValueError("PlatformIO system info failed")
        core = core_directory(info.stdout)
        pins = target_pins(doctor.root)
    except (ValueError, KeyError, configparser.Error) as exc:
        doctor.add("platformio.packages", "FAIL", str(exc))
        return
    for name, (_, expected_version) in pins.items():
        directory = core / ("platforms" if name == "espressif32" else "packages") / name
        metadata = directory / ("platform.json" if name == "espressif32" else "package.json")
        try:
            manifest = json.loads(metadata.read_text(encoding="utf-8"))
            if not isinstance(manifest, dict) or not isinstance(manifest.get("version"), str):
                raise ValueError("invalid package metadata")
            installed = manifest["version"]
            good = installed == expected_version
            doctor.add(f"package.{name}", "PASS" if good else "FAIL",
                       f"expected={expected_version} installed={installed}")
        except (OSError, ValueError, KeyError):
            doctor.add(f"package.{name}", "FAIL", "canonical package metadata missing/invalid; install pinned packages")
            continue
        if name == "toolchain-riscv32-esp" and good:
            compiler = directory / "bin" / ("riscv32-esp-elf-g++" + (".exe" if os.name == "nt" else ""))
            if doctor.version("target.cxx", str(compiler), True):
                target = doctor.run("target.cxx.abi", [str(compiler), "-dumpmachine"])
                if target.returncode or target.stdout.strip() != "riscv32-esp-elf":
                    doctor.add("target.cxx.abi", "FAIL", "expected riscv32-esp-elf")
                    continue
                doctor.add("target.cxx.abi", "PASS", "riscv32-esp-elf")
                with project_temp_directory(doctor.root, "doctor-target-") as raw:
                    source, obj = Path(raw) / "probe.cpp", Path(raw) / "probe.o"
                    source.write_text("unsigned tracker_target_probe(unsigned x) { return x + 1; }\n")
                    result = doctor.run("target.compile", [str(compiler), "-std=gnu++2a", "-c", str(source), "-o", str(obj)])
                    good = result.returncode == 0 and obj.is_file() and obj.stat().st_size > 0
                    doctor.add("target.compile", "PASS" if good else "FAIL", "object compile only; firmware link NOT verified")


def check_host(doctor: Doctor, cxx: str | None) -> None:
    if doctor.version("host.cxx", cxx, True) is None:
        return
    target = doctor.run("host.cxx.target", [cxx, "-dumpmachine"])
    doctor.add("host.cxx.target", "PASS" if target.returncode == 0 and target.stdout.strip() else "FAIL",
               target.stdout.strip()[:160] or "compiler target unavailable")
    with project_temp_directory(doctor.root, "doctor-host-") as raw:
        exe = Path(raw) / ("probe.exe" if os.name == "nt" else "probe")
        built = doctor.run("host.compile", [cxx, "-std=c++20", "-O2",
                           str(doctor.root / "tests/fixtures/host_toolchain_probe.cpp"), "-o", str(exe)])
        if built.returncode or not exe.is_file() or exe.stat().st_size == 0:
            doctor.add("host.cxx20", "FAIL", "compile/link failed or executable missing")
            return
        ran = doctor.run("host.execute", [str(exe)])
        good = ran.returncode == 0 and re.fullmatch(r"TRACKER_HOST_CXX20_OK pointer_bits=(32|64)\s*", ran.stdout)
        doctor.add("host.cxx20", "PASS" if good else "FAIL", ran.stdout.strip()[:120] if good else "host executable failed or invalid proof")


def perform(doctor: Doctor, cxx: str | None, pio: str | None, required_tools: list[str]) -> None:
    doctor.add("python", "PASS" if sys.version_info >= (3, 10) else "FAIL",
               f"{sys.executable}: {platform.python_version()} (minimum 3.10)")
    git = executable(None, ("git",))
    if doctor.version("git", git, True):
        head = doctor.run("source.head", [git, "rev-parse", "HEAD"])
        status = doctor.run("source.status", [git, "status", "--porcelain", "--untracked-files=normal"])
        if head.returncode or status.returncode:
            doctor.add("source.identity", "WARN", "Git identity unavailable; release not verified")
        else:
            doctor.add("source.identity", "WARN" if status.stdout else "PASS",
                       head.stdout.strip() + (" dirty" if status.stdout else " clean"))
    check_host(doctor, cxx)
    if doctor.profile == "sanitized":
        for name in ("address-undefined", "leak"):
            if not cxx:
                doctor.add(f"sanitizer.{name}", "FAIL", "no selected compiler")
                continue
            result = probe_sanitizer(cxx, doctor.root, name, timeout_s=doctor.timeout)
            doctor.commands.append({"label": f"sanitizer.{name}", "evidence": asdict(result)})
            doctor.add(f"sanitizer.{name}", "PASS" if result.supported else "FAIL", result.reason)
    else:
        doctor.add("sanitizers", "SKIP", "not requested; use --profile sanitized")
    if doctor.profile == "firmware":
        check_target(doctor, pio)
    else:
        doctor.version("platformio.core", pio, False)
    for name in OPTIONAL_TOOLS:
        doctor.version(name, executable(None, (name,)), name in required_tools)
    doctor.add("firmware.hardware", "SKIP", "no firmware build, flash, attach or hardware test performed")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("host", "sanitized", "firmware"), default="host")
    parser.add_argument("--cxx", default=os.environ.get("CXX"))
    parser.add_argument("--pio-bin", default=os.environ.get("PIO"))
    parser.add_argument("--require-tool", choices=OPTIONAL_TOOLS, action="append", default=[])
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        parser.error("timeout must be finite and positive")
    doctor = Doctor(ROOT, args.profile, args.timeout_s, env=quality_gate_environment(ROOT, scope="doctor"))
    try:
        perform(doctor, executable(args.cxx, ("g++", "clang++", "c++")),
                executable(args.pio_bin, ("pio", "platformio")), args.require_tool)
    except (OSError, ValueError, configparser.Error) as exc:
        doctor.add("environment", "FAIL", str(exc))
    try:
        config_hash = hashlib.sha256((ROOT / "platformio.ini").read_bytes()).hexdigest()
    except OSError:
        config_hash = None
        doctor.add("source.config", "FAIL", "platformio.ini unavailable")
    failed = any(c["status"] == "FAIL" for c in doctor.checks)
    report = {"schema": "tracker-doctor-v1", "timestamp_utc": datetime.now(timezone.utc).isoformat(),
              "profile": args.profile, "os": platform.system(), "machine": platform.machine(),
              "python": platform.python_version(), "root": str(ROOT), "failed": failed,
              "platformio_ini_sha256": config_hash,
              "checks": doctor.checks, "commands": doctor.commands}
    directory = ROOT / "build/doctor"
    directory.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", prefix=args.profile + "-",
                                     suffix=".json", dir=directory, delete=False) as output:
        json.dump(report, output, ensure_ascii=False, indent=2)
        output.write("\n")
        report_path = output.name
    for check in doctor.checks:
        print(f"{check['status']} {check['name']}: {check['detail']}")
    print(f"doctor: {'FAIL' if failed else 'PASS'} (selected checks only); report={report_path}")
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
