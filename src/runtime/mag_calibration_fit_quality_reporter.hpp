#pragma once

#include <Arduino.h>

#include "sensor/mag_calibration.hpp"

namespace tracker {

#if defined(__GNUC__) || defined(__clang__)
#define TRACKER_MAG_CAL_FIT_REPORT_NOINLINE __attribute__((noinline))
#else
#define TRACKER_MAG_CAL_FIT_REPORT_NOINLINE
#endif

// Kept in a small header because compact Production/Slim command hooks need
// these failure metrics even when the detailed mag_status_reporter.cpp source
// is intentionally excluded by the profile source filter.
TRACKER_MAG_CAL_FIT_REPORT_NOINLINE inline void magStatusPrintCalibrationFitQuality(
    Stream& out,
    const MagCalibrationCollector& collector,
    const char* prefix = "") {
    if (!prefix) prefix = "";
    const MagCalibrationResult& fit = collector.lastResult();
    const bool available = fit.fitAvailable;

    out.print(prefix); out.print("last_fit_available="); out.println(available ? "yes" : "no");
    out.print(prefix); out.print("last_fit_solver_stage=");
    out.println(magCalibrationSolverStageName(fit.solverStage));
    out.print(prefix); out.print("last_fit_solver_samples="); out.println(fit.solverSamples);
    out.print(prefix); out.print("last_fit_solver_pivot_ratio="); out.println(fit.solverPivotRatio, 9);
    out.print(prefix); out.print("last_fit_normalization_center=");
    out.print(fit.fitNormalizationCenter.x, 6); out.print(',');
    out.print(fit.fitNormalizationCenter.y, 6); out.print(',');
    out.println(fit.fitNormalizationCenter.z, 6);
    out.print(prefix); out.print("last_fit_normalization_scale=");
    out.print(fit.fitNormalizationScale.x, 6); out.print(',');
    out.print(fit.fitNormalizationScale.y, 6); out.print(',');
    out.println(fit.fitNormalizationScale.z, 6);
    if (!available) return;

    out.print(prefix); out.print("last_fit_quality=");
    out.print(fit.residualRms, 6); out.print(',');
    out.print(fit.normalizedResidualRms, 6); out.print(',');
    out.print(fit.directionalCoverageScore, 6); out.print(',');
    out.print(fit.inlierRatio, 6); out.print(',');
    out.print(fit.axisRatio, 6); out.print(',');
    out.println(fit.coverageScore, 6);

    const MagCalibrationParams& params = collector.params();
    out.print(prefix); out.print("last_fit_quality_limits=");
    out.print(params.maxAlgebraicResidualRms, 6); out.print(',');
    out.print(params.maxGeometricResidualRmsFactor, 6); out.print(',');
    out.print(params.minDirectionalCoverageScore, 6); out.print(',');
    out.print(params.minInlierRatio, 6); out.print(',');
    out.print(params.maxAxisRatio, 6); out.print(',');
    out.println(magCalibrationEffectiveMinBoxCoverage(params), 6);
}

#undef TRACKER_MAG_CAL_FIT_REPORT_NOINLINE

} // namespace tracker
