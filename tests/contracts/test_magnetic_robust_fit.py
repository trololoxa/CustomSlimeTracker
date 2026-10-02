#!/usr/bin/env python3
"""Guard magnetic_robust_fit magnetometer robust-fit acceptance hardening."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, forbid, require, run_contract_command, stack_usage,
)


from quality_gate_runtime import (
    asan_ubsan_environment,
    project_temp_directory,
    strongest_supported_sanitizer_flags,
)

ROOT = Path(__file__).resolve().parents[2]


def compile_and_run(cxx: str, exe: Path, extra: list[str]) -> None:
    run_contract_command(
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
    run_contract_command([str(exe)], check=True, env=asan_ubsan_environment(ROOT))


def main() -> int:
    mag_h = (ROOT / "src/sensor/mag_calibration.hpp").read_text(encoding="utf-8")
    mag = (ROOT / "src/sensor/mag_calibration.cpp").read_text(encoding="utf-8")
    reporter = (ROOT / "src/runtime/mag_calibration_fit_quality_reporter.hpp").read_text(encoding="utf-8")
    test = (ROOT / "tests/native/test_mag_calibration.cpp").read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")

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
    require(mag, "TRACKER_MAG_FIT_NOINLINE bool replaceFitFromAccumulator", "no-inline refit helper")
    forbid(mag, "FitAccumulator inlierAcc", "second large accumulator")
    forbid(mag, "finalFit.algebraicResidualRms > params_.maxAlgebraicResidualRms", "contradictory raw algebraic gate")
    require(mag, "finalFit.algebraicResidualRms > magCalibrationEffectiveMaxAlgebraicResidualRms(params_)", "effective algebraic gate")
    require(mag, "finalThreshold / finalFit.expectedNorm", "actual robust threshold diagnostic")

    require(reporter, "last_fit_robust_refit_passes=", "refit status field")
    require(reporter, "last_fit_robust_threshold_factor=", "threshold status field")

    require(test, "The centered algebraic residual is approximately twice radial", "hardware log 2 regression")
    require(test, "Hardware-shaped moderate contamination", "moderate contamination regression")
    require(test, "disturbed population exceeds the configured 18% outlier budget", "fail-closed inlier regression")
    require(test, "params.outlierMinResidualFactor = 0.50f", "isolated geometric rejection regression")


    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-magnetic_robust_fit-") as tmp_name:
        tmp = Path(tmp_name)
        compile_and_run(cxx, tmp / "test_mag_calibration", ["-O2", "-Wall", "-Wextra", "-Werror"])
        sanitizer_name, sanitizer_flags = strongest_supported_sanitizer_flags(cxx, ROOT)
        if sanitizer_flags:
            print(f"# magnetic_robust_fit sanitizer={sanitizer_name}")
            compile_and_run(
                cxx,
                tmp / "test_mag_calibration_san",
                ["-O1", "-g", *sanitizer_flags],
            )
        else:
            print("# magnetic_robust_fit sanitizer: SKIP (toolchain cannot link ASan/UBSan)")
        run_contract_command(
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

    print("# magnetic_robust_fit: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
