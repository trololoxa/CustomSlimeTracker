#!/usr/bin/env python3
"""Guard 0023gk magnetometer robust-fit acceptance hardening."""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

from quality_gate_runtime import (
    asan_ubsan_environment,
    project_temp_directory,
    strongest_supported_sanitizer_flags,
)

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
    raise SystemExit("no C++ compiler available for 0023gk policy")


def stack_usage(tmp: Path, symbol: str) -> int:
    best: int | None = None
    for su in tmp.glob("*.su"):
        for line in su.read_text(encoding="utf-8", errors="replace").splitlines():
            if symbol not in line:
                continue
            fields = line.rsplit("\t", 2)
            if len(fields) < 2:
                continue
            match = re.search(r"(\d+)", fields[-2])
            if match:
                value = int(match.group(1))
                best = value if best is None else max(best, value)
    if best is None:
        raise SystemExit(f"stack usage missing for {symbol}")
    return best


def compile_and_run(cxx: str, exe: Path, extra: list[str]) -> None:
    subprocess.run(
        [
            cxx,
            "-std=c++20",
            *extra,
            "-I", str(ROOT / "src"),
            "-I", str(ROOT / "tests/native"),
            str(ROOT / "tests/native/test_mag_calibration.cpp"),
            str(ROOT / "tests/native/sanitizer_runtime_options.cpp"),
            str(ROOT / "src/sensor/mag_calibration.cpp"),
            "-o", str(exe),
        ],
        check=True,
    )
    subprocess.run([str(exe)], check=True, env=asan_ubsan_environment(ROOT))


def main() -> int:
    mag_h = (ROOT / "src/sensor/mag_calibration.hpp").read_text(encoding="utf-8")
    mag = (ROOT / "src/sensor/mag_calibration.cpp").read_text(encoding="utf-8")
    reporter = (ROOT / "src/runtime/mag_calibration_fit_quality_reporter.hpp").read_text(encoding="utf-8")
    test = (ROOT / "tests/native/test_mag_calibration.cpp").read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")
    project = (ROOT / "docs/project_status.md").read_text(encoding="utf-8")
    testing = (ROOT / "docs/testing.md").read_text(encoding="utf-8")
    cli = (ROOT / "docs/cli_reference.md").read_text(encoding="utf-8")
    report = (ROOT / "docs/0023gk_magnetometer_robust_fit_acceptance_hardening_report.md").read_text(encoding="utf-8")

    require(mag_h, "magCalibrationEffectiveMaxAlgebraicResidualRms", "geometric-compatible algebraic backstop")
    require(mag_h, "2.0f + magCalibrationRobustResidualCapFactor(params)", "threshold-aware algebraic/geometric relation")
    require(mag_h, "algebraicPerRadial * params.maxGeometricResidualRmsFactor", "effective algebraic ceiling")
    require(mag_h, "magCalibrationRobustResidualCapFactor", "bounded robust threshold helper")
    require(mag_h, "1.5f * params.maxGeometricResidualRmsFactor", "quality-derived robust cap")
    require(mag_h, "robustRefitPasses", "refit diagnostic")
    require(mag_h, "robustInlierThresholdFactor", "threshold diagnostic")

    require(mag, "const float byQualityCap = magCalibrationRobustResidualCapFactor(params) * fit.expectedNorm", "bounded sigma threshold")
    require(mag, "constexpr uint8_t kMaxRobustRefitPasses = 3u", "bounded refit loop")
    require(mag, "std::memcmp(membership, previousMembership", "exact inlier-set convergence")
    require(mag, "kInlierMembershipWords", "bounded full-membership bitmap")
    forbid(mag, "membershipSignature", "collision-prone membership hash")
    require(mag, "replaceFitFromAccumulator", "stack-isolated candidate construction")
    require(mag, "TRACKER_MAG_FIT_NOINLINE bool replaceFitFromAccumulator", "no-inline refit helper")
    require(mag, "FitAccumulator fitAccumulator;", "single reused accumulator")
    forbid(mag, "FitAccumulator inlierAcc", "second large accumulator")
    forbid(mag, "finalFit.algebraicResidualRms > params_.maxAlgebraicResidualRms", "contradictory raw algebraic gate")
    require(mag, "finalFit.algebraicResidualRms > magCalibrationEffectiveMaxAlgebraicResidualRms(params_)", "effective algebraic gate")
    require(mag, "finalThreshold / finalFit.expectedNorm", "actual robust threshold diagnostic")

    require(reporter, "last_fit_robust_refit_passes=", "refit status field")
    require(reporter, "last_fit_robust_threshold_factor=", "threshold status field")
    require(reporter, "magCalibrationEffectiveMaxAlgebraicResidualRms(params)", "actual printed algebraic limit")

    require(test, "The centered algebraic residual is approximately twice radial", "hardware log 2 regression")
    require(test, "Hardware-shaped moderate contamination", "moderate contamination regression")
    require(test, "disturbed population exceeds the configured 18% outlier budget", "fail-closed inlier regression")
    require(test, "params.outlierMinResidualFactor = 0.50f", "isolated geometric rejection regression")

    require(check_all, '("tools/test_calibration_0023gk_policy.py", "0023gk magnetometer robust-fit acceptance policy")', "aggregate policy entry")
    require(project, "0023gk_magnetometer_robust_fit_acceptance_hardening", "project status entry")
    require(testing, "0023gk magnetometer robust-fit acceptance regression", "testing entry")
    require(cli, "last_fit_robust_refit_passes", "CLI diagnostics")
    require(report, "0.146809,0.074308", "second hardware failure evidence")
    require(report, "0.310533,0.164865", "first hardware failure evidence")

    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-0023gk-") as tmp_name:
        tmp = Path(tmp_name)
        compile_and_run(cxx, tmp / "test_mag_calibration", ["-O2", "-Wall", "-Wextra", "-Werror"])
        sanitizer_name, sanitizer_flags = strongest_supported_sanitizer_flags(cxx, ROOT)
        if sanitizer_flags:
            print(f"# 0023gk sanitizer={sanitizer_name}")
            compile_and_run(
                cxx,
                tmp / "test_mag_calibration_san",
                ["-O1", "-g", *sanitizer_flags],
            )
        else:
            print("# 0023gk sanitizer: SKIP (toolchain cannot link ASan/UBSan)")
        subprocess.run(
            [
                cxx, "-std=c++20", "-O2", "-fstack-usage",
                "-I", str(ROOT / "src"),
                "-c", str(ROOT / "src/sensor/mag_calibration.cpp"),
                "-o", str(tmp / "mag.o"),
            ],
            check=True,
        )
        compute_stack = stack_usage(tmp, "MagCalibrationCollector::compute")
        refit_stack = stack_usage(tmp, "replaceFitFromAccumulator")
        if compute_stack > 1792:
            raise SystemExit(f"MagCalibrationCollector::compute stack {compute_stack} exceeds 1792")
        if refit_stack > 384:
            raise SystemExit(f"replaceFitFromAccumulator stack {refit_stack} exceeds 384")

    print("# calibration_0023gk_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
