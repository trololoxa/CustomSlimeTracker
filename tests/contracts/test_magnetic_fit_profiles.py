#!/usr/bin/env python3
"""Guard magnetic_fit_profiles cross-ABI mag-fit stack and compact-profile reporter wiring."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, forbid, parse_stack_usage, require, require_limit, run_contract_command,
)


from quality_gate_runtime import project_temp_directory

ROOT = Path(__file__).resolve().parents[2]


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
    run_contract_command(
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
    gb_policy = (ROOT / "tests/contracts/test_magnetic_fit_metrics.py").read_text(encoding="utf-8")
    check_all = (ROOT / "tools/check_all.py").read_text(encoding="utf-8")

    require(mag, "FitAccumulator fitAccumulator;", "single mag-fit accumulator workspace")
    require(mag, "const uint32_t rawFitSamples = fitAccumulator.count;", "raw-fit count retained before workspace reuse")
    require(mag, "fitNormalization, fitAccumulator, membership)", "inlier pass workspace reuse")
    forbid(mag, "FitAccumulator all;", "first simultaneously-live accumulator")
    forbid(mag, "FitAccumulator inlierAcc;", "second simultaneously-live accumulator")
    require(mag, "safe cross-ABI stack margin", "stack rationale")

    require(fit_report, "magStatusPrintCalibrationFitQuality", "minimal fit-quality reporter")
    require(fit_report, "compact Production/Slim command hooks", "source-filter ownership rationale")
    if app_hooks.index('#include "runtime/mag_calibration_fit_quality_reporter.hpp"') > app_hooks.index("#if TRACKER_ENABLE_DETAILED_MAG_STATUS"):
        raise SystemExit("fit-quality reporter include must not depend on detailed-mag profile gate")
    forbid(status_h, "TRACKER_MAG_STATUS_NOINLINE inline void magStatusPrintCalibrationFitQuality", "helper trapped behind detailed reporter header")
    require(gb_policy, 'fit_report_h = (ROOT / "src/runtime/mag_calibration_fit_quality_reporter.hpp")', "updated magnetic_fit_profiles helper contract")


    with project_temp_directory(ROOT, "tracker-magnetic_fit_profiles-") as tmp_name:
        tmp = Path(tmp_name)
        cxx = compiler()
        run_contract_command(
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

    print("# magnetic_fit_profiles: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
