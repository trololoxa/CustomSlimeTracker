#!/usr/bin/env python3
"""Guard 0023gb magnetometer fit-metric normalization and failure diagnostics."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

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
    raise SystemExit("no C++ compiler available for 0023gb policy")


def main() -> int:
    mag_h = (ROOT / "src/sensor/mag_calibration.hpp").read_text(encoding="utf-8")
    mag = (ROOT / "src/sensor/mag_calibration.cpp").read_text(encoding="utf-8")
    hooks = (ROOT / "src/app/hooks/tracker_app_mag_hooks.hpp").read_text(encoding="utf-8")
    controller = (ROOT / "src/runtime/mag_runtime_controller.cpp").read_text(encoding="utf-8")
    fit_report_h = (ROOT / "src/runtime/mag_calibration_fit_quality_reporter.hpp").read_text(encoding="utf-8")
    status_h = (ROOT / "src/runtime/mag_status_reporter.hpp").read_text(encoding="utf-8")
    app_hooks = (ROOT / "src/app/tracker_app_hooks.hpp").read_text(encoding="utf-8")
    status = (ROOT / "src/runtime/mag_status_reporter.cpp").read_text(encoding="utf-8")
    test = (ROOT / "tests/native/test_mag_calibration.cpp").read_text(encoding="utf-8")
    project = (ROOT / "docs/project_status.md").read_text(encoding="utf-8")
    testing = (ROOT / "docs/testing.md").read_text(encoding="utf-8")
    report = (ROOT / "docs/0023gb_magnetometer_fit_metric_normalization_report.md").read_text(encoding="utf-8")
    platformio = (ROOT / "platformio.ini").read_text(encoding="utf-8")

    require(mag_h, "Minimum centered, dimensionless ellipsoid-equation sanity ceiling", "normalized parameter contract")
    require(mag, "std::sqrt(residualVar) / std::fabs(k)", "centered algebraic residual normalization")
    require(mag, "translation-invariant and approximately twice", "physical metric rationale")
    forbid(mag, "const double residualRms = std::sqrt(residualVar);", "origin-dependent residual gate")

    metrics = mag.index("// Keep the best finite fit diagnostics")
    inlier_gate = mag.index("if (finalMetrics.inliers < minSamples", metrics)
    algebraic_gate = mag.index("finalFit.algebraicResidualRms > magCalibrationEffectiveMaxAlgebraicResidualRms(params_)", metrics)
    if not metrics < inlier_gate < algebraic_gate:
        raise SystemExit("0023gb diagnostics must be captured before quality-gate returns")
    require(mag, "lastResult_ = out;", "rejected fit persistence")

    require(fit_report_h, "TRACKER_MAG_CAL_FIT_REPORT_NOINLINE inline void magStatusPrintCalibrationFitQuality", "header-only source-filter-safe reporter")
    require(fit_report_h, "last_fit_quality=", "shared fit-quality tuple")
    require(fit_report_h, "last_fit_quality_limits=", "shared fit-quality limits tuple")
    require(fit_report_h, "fit.residualRms", "algebraic quality component")
    require(fit_report_h, "fit.normalizedResidualRms", "geometric quality component")
    require(fit_report_h, "fit.inlierRatio", "inlier quality component")
    require(fit_report_h, "magCalibrationEffectiveMaxAlgebraicResidualRms(params)", "effective algebraic quality limit")
    require(status_h, '#include "runtime/mag_calibration_fit_quality_reporter.hpp"', "detailed reporter helper include")
    require(app_hooks, '#include "runtime/mag_calibration_fit_quality_reporter.hpp"', "compact-profile helper include")
    forbid(status, "void magStatusPrintCalibrationFitQuality(", "out-of-line helper in excluded source")
    require(hooks, "magStatusPrintCalibrationFitQuality(out, g_magCalCollector);", "compact status helper call")
    require(controller, 'magStatusPrintCalibrationFitQuality(stream(), *deps_.calibrationCollector, "# mag_cal_");', "apply failure helper call")
    require(status, "magStatusPrintCalibrationFitQuality(out, collector);", "detailed status helper call")
    require(platformio, "-<runtime/mag_status_reporter.cpp>", "production exclusion contract")

    require(test, "algebraic fit quality must describe the centered", "translation-invariance regression")
    require(test, "Mild non-ellipsoidal field variation", "false-rejection regression")
    require(test, "Strongly non-ellipsoidal data still fails", "quality-preservation regression")
    require(test, "centeredResult.residualRms, translatedResult.residualRms", "translated residual equality assertion")
    require(test, "MagCalibrationFailureReason::GeometricResidualTooHigh", "bad-field rejection assertion")

    require(project, "0023gb_magnetometer_fit_metric_normalization", "project status entry")
    require(testing, "0023gb magnetometer fit-metric regression", "testing entry")
    require(report, "algebraic_residual_too_high", "recorded hardware failure")
    require(report, "x^T A x + b^T x = 1", "equation analysis")

    with tempfile.TemporaryDirectory(prefix="tracker-0023gb-") as tmp:
        exe = Path(tmp) / "test_mag_calibration"
        subprocess.run(
            [
                compiler(), "-std=c++20", "-O2",
                "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
                str(ROOT / "tests/native/test_mag_calibration.cpp"),
                str(ROOT / "src/sensor/mag_calibration.cpp"),
                "-o", str(exe),
            ],
            check=True,
        )
        subprocess.run([str(exe)], check=True)

    subprocess.run([sys.executable, str(ROOT / "tools/test_calibration_0023ga_policy.py")], check=True)
    print("# calibration_0023gb_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
