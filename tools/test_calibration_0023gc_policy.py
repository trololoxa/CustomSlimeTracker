#!/usr/bin/env python3
"""Guard 0023gc cross-ABI mag-fit stack and compact-profile reporter wiring."""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

from quality_gate_runtime import project_temp_directory

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
    raise SystemExit("no C++ compiler available for 0023gc policy")


def parse_stack_usage(directory: Path) -> dict[str, int]:
    usage: dict[str, int] = {}
    for path in directory.rglob("*.su"):
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            parts = line.split("\t")
            if len(parts) < 2:
                continue
            try:
                size = int(parts[1])
            except ValueError:
                continue
            name = parts[0].split(":", 3)[-1]
            usage[name] = max(size, usage.get(name, 0))
    return usage


def require_limit(usage: dict[str, int], needle: str, limit: int) -> None:
    matches = [(name, size) for name, size in usage.items() if needle in name]
    if not matches:
        raise SystemExit(f"0023gc stack policy did not find function: {needle}")
    worst_name, worst_size = max(matches, key=lambda item: item[1])
    if worst_size > limit:
        raise SystemExit(
            f"0023gc stack budget exceeded: {worst_name} uses {worst_size} bytes, limit {limit}"
        )


def compile_profile_helper(cxx: str, tmp: Path, profile: str) -> None:
    source = tmp / f"fit_quality_{profile}.cpp"
    source.write_text(
        "#define ARDUINO_ARCH_ESP32 1\n"
        f"#define TRACKER_BUILD_PROFILE {profile}\n"
        '#include "defines.h"\n'
        '#include "runtime/mag_calibration_fit_quality_reporter.hpp"\n'
        "using namespace tracker;\n"
        "void profileFitQualityCompile(Stream& out, const MagCalibrationCollector& collector) {\n"
        "    magStatusPrintCalibrationFitQuality(out, collector);\n"
        "}\n",
        encoding="utf-8",
    )
    subprocess.run(
        [
            cxx, "-std=c++20", "-O2",
            "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
            "-c", str(source), "-o", str(tmp / f"{profile}.o"),
        ],
        check=True,
    )


def main() -> int:
    mag = (ROOT / "src/sensor/mag_calibration.cpp").read_text(encoding="utf-8")
    fit_report = (ROOT / "src/runtime/mag_calibration_fit_quality_reporter.hpp").read_text(encoding="utf-8")
    status_h = (ROOT / "src/runtime/mag_status_reporter.hpp").read_text(encoding="utf-8")
    app_hooks = (ROOT / "src/app/tracker_app_hooks.hpp").read_text(encoding="utf-8")
    hooks = (ROOT / "src/app/hooks/tracker_app_mag_hooks.hpp").read_text(encoding="utf-8")
    gb_policy = (ROOT / "tools/test_calibration_0023gb_policy.py").read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")
    project = (ROOT / "docs/project_status.md").read_text(encoding="utf-8")
    testing = (ROOT / "docs/testing.md").read_text(encoding="utf-8")
    report = (ROOT / "docs/0023gc_mag_fit_stack_and_profile_build_hardening_report.md").read_text(encoding="utf-8")

    require(mag, "FitAccumulator fitAccumulator;", "single mag-fit accumulator workspace")
    require(mag, "const uint32_t rawFitSamples = fitAccumulator.count;", "raw-fit count retained before workspace reuse")
    require(mag, "fitNormalization, fitAccumulator, membership)", "inlier pass workspace reuse")
    forbid(mag, "FitAccumulator all;", "first simultaneously-live accumulator")
    forbid(mag, "FitAccumulator inlierAcc;", "second simultaneously-live accumulator")
    require(mag, "safe cross-ABI stack margin", "stack rationale")

    require(fit_report, "magStatusPrintCalibrationFitQuality", "minimal fit-quality reporter")
    require(fit_report, "compact Production/Slim command hooks", "source-filter ownership rationale")
    require(status_h, '#include "runtime/mag_calibration_fit_quality_reporter.hpp"', "detailed reporter helper include")
    require(app_hooks, '#include "runtime/mag_calibration_fit_quality_reporter.hpp"', "always-available app helper include")
    if app_hooks.index('#include "runtime/mag_calibration_fit_quality_reporter.hpp"') > app_hooks.index("#if TRACKER_ENABLE_DETAILED_MAG_STATUS"):
        raise SystemExit("fit-quality reporter include must not depend on detailed-mag profile gate")
    require(hooks, "magStatusPrintCalibrationFitQuality(out, g_magCalCollector);", "compact profile helper call")
    forbid(status_h, "TRACKER_MAG_STATUS_NOINLINE inline void magStatusPrintCalibrationFitQuality", "helper trapped behind detailed reporter header")
    require(gb_policy, 'fit_report_h = (ROOT / "src/runtime/mag_calibration_fit_quality_reporter.hpp")', "updated 0023gb helper contract")

    require(check_all, '("tools/test_calibration_0023gc_policy.py", "0023gc mag fit stack/profile build policy")', "aggregate runner entry")
    require(project, "0023gc_mag_fit_stack_and_profile_build_hardening", "project status entry")
    require(testing, "0023gc cross-ABI mag-fit stack/profile regression", "testing entry")
    require(report, "2240 bytes", "recorded MSYS2 stack failure")
    require(report, "magStatusPrintCalibrationFitQuality", "recorded profile compile failure")

    with project_temp_directory(ROOT, "tracker-0023gc-") as tmp_name:
        tmp = Path(tmp_name)
        cxx = compiler()
        subprocess.run(
            [
                cxx, "-std=c++20", "-O2", "-fstack-usage",
                "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
                "-c", str(ROOT / "src/sensor/mag_calibration.cpp"),
                "-o", str(tmp / "mag.o"),
            ],
            check=True,
        )
        usage = parse_stack_usage(tmp)
        require_limit(usage, "MagCalibrationCollector::compute", 1792)
        compile_profile_helper(cxx, tmp, "TRACKER_PROFILE_PRODUCTION")
        compile_profile_helper(cxx, tmp, "TRACKER_PROFILE_SLIM")

    print("# calibration_0023gc_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
