#include "sensor/mag_calibration.hpp"

#include "core/deterministic_reservoir.hpp"

#include <cmath>
#include <cstring>

namespace tracker {

const char* magCalibrationFailureReasonName(MagCalibrationFailureReason reason) {
    switch (reason) {
        case MagCalibrationFailureReason::None: return "none";
        case MagCalibrationFailureReason::InsufficientSamples: return "insufficient_samples";
        case MagCalibrationFailureReason::AxisRadiusTooSmall: return "axis_radius_too_small";
        case MagCalibrationFailureReason::BoxCoverageTooLow: return "box_coverage_too_low";
        case MagCalibrationFailureReason::RawNormFilterFailed: return "raw_norm_filter_failed";
        case MagCalibrationFailureReason::FitNormalizationFailed: return "fit_normalization_failed";
        case MagCalibrationFailureReason::LinearSolveFailed: return "linear_solve_failed";
        case MagCalibrationFailureReason::QuadraticCenterFailed: return "quadratic_center_failed";
        case MagCalibrationFailureReason::NonPositiveDefiniteShape: return "non_positive_definite_shape";
        case MagCalibrationFailureReason::EllipsoidFitFailed: return "ellipsoid_fit_failed";
        case MagCalibrationFailureReason::InlierRatioTooLow: return "inlier_ratio_too_low";
        case MagCalibrationFailureReason::GeometricResidualTooHigh: return "geometric_residual_too_high";
        case MagCalibrationFailureReason::DirectionalCoverageTooLow: return "directional_coverage_too_low";
        case MagCalibrationFailureReason::AlgebraicResidualTooHigh: return "algebraic_residual_too_high";
    }
    return "unknown";
}

const char* magCalibrationSolverStageName(MagCalibrationSolverStage stage) {
    switch (stage) {
        case MagCalibrationSolverStage::None: return "none";
        case MagCalibrationSolverStage::Normalized: return "normalized";
        case MagCalibrationSolverStage::LinearSolved: return "linear_solved";
        case MagCalibrationSolverStage::CenterSolved: return "center_solved";
        case MagCalibrationSolverStage::ShapeSolved: return "shape_solved";
        case MagCalibrationSolverStage::EigenSolved: return "eigen_solved";
        case MagCalibrationSolverStage::CandidateBuilt: return "candidate_built";
    }
    return "unknown";
}

namespace {

constexpr int kMagFitTerms = 9;
constexpr uint16_t kInlierMembershipWords =
    static_cast<uint16_t>((MAG_CALIBRATION_MAX_STORED_SAMPLES + 31u) / 32u);

#if defined(__GNUC__) || defined(__clang__)
#define TRACKER_MAG_FIT_NOINLINE __attribute__((noinline))
#else
#define TRACKER_MAG_FIT_NOINLINE
#endif

struct FitNormalization {
    double center[3] = {};
    double scale[3] = {1.0, 1.0, 1.0};
    bool valid = false;
};

struct FitAccumulator {
    double normal[kMagFitTerms][kMagFitTerms] = {};
    double rhs[kMagFitTerms] = {};
    uint32_t count = 0;
};

struct FitCandidate {
    bool valid = false;
    MagCalibrationFailureReason failureReason = MagCalibrationFailureReason::EllipsoidFitFailed;
    MagCalibrationSolverStage solverStage = MagCalibrationSolverStage::None;
    FitNormalization normalization;
    float solverPivotRatio = 0.0f;
    uint32_t solverSamples = 0;
    Vec3 hardIron = Vec3::zero();
    Mat3 softIron = Mat3::identity();
    float expectedNorm = 1.0f;
    float radiusX = 0.0f;
    float radiusY = 0.0f;
    float radiusZ = 0.0f;
    float axisRatio = 0.0f;
    float algebraicResidualRms = 999.0f;
};

struct GeometricMetrics {
    uint32_t inliers = 0;
    float inlierRatio = 0.0f;
    float rms = 999.0f;
    float normalizedRms = 999.0f;
    float directionalCoverageScore = 0.0f;
};

Vec3 storedToVec3(const MagCalibrationStoredSample& s) {
    return Vec3(static_cast<float>(s.x), static_cast<float>(s.y), static_cast<float>(s.z));
}

void resetAccumulator(FitAccumulator& acc) {
    acc.count = 0;
    for (int i = 0; i < kMagFitTerms; ++i) {
        acc.rhs[i] = 0.0;
        for (int j = 0; j < kMagFitTerms; ++j) {
            acc.normal[i][j] = 0.0;
        }
    }
}

void accumulateSample(FitAccumulator& acc,
                      const Vec3& sample,
                      const FitNormalization& normalization) {
    const double x = (static_cast<double>(sample.x) - normalization.center[0]) / normalization.scale[0];
    const double y = (static_cast<double>(sample.y) - normalization.center[1]) / normalization.scale[1];
    const double z = (static_cast<double>(sample.z) - normalization.center[2]) / normalization.scale[2];
    const double terms[kMagFitTerms] = {
        x * x,
        y * y,
        z * z,
        2.0 * x * y,
        2.0 * x * z,
        2.0 * y * z,
        x,
        y,
        z,
    };
    for (int i = 0; i < kMagFitTerms; ++i) {
        acc.rhs[i] += terms[i];
        for (int j = 0; j < kMagFitTerms; ++j) {
            acc.normal[i][j] += terms[i] * terms[j];
        }
    }
    acc.count++;
}

TRACKER_MAG_FIT_NOINLINE bool solveLinear9(
    const double inA[kMagFitTerms][kMagFitTerms],
    const double inB[kMagFitTerms],
    double outX[kMagFitTerms],
    float& pivotRatioOut) {
    // Equilibrate the normal equations before elimination. Quadratic, cross
    // and linear columns naturally have very different magnitudes even after
    // sample-space normalization. Unit-diagonal scaling makes the pivot test
    // a meaningful conditioning check instead of a raw-unit accident.
    double columnScale[kMagFitTerms] = {};
    for (int i = 0; i < kMagFitTerms; ++i) {
        const double diagonal = std::fabs(inA[i][i]);
        if (!std::isfinite(diagonal) || diagonal <= 1.0e-18) return false;
        columnScale[i] = std::sqrt(diagonal);
    }

    double a[kMagFitTerms][kMagFitTerms + 1] = {};
    for (int r = 0; r < kMagFitTerms; ++r) {
        for (int c = 0; c < kMagFitTerms; ++c) {
            a[r][c] = inA[r][c] / (columnScale[r] * columnScale[c]);
        }
        a[r][kMagFitTerms] = inB[r] / columnScale[r];
    }

    double minPivot = 1.0e300;
    double maxPivot = 0.0;
    for (int col = 0; col < kMagFitTerms; ++col) {
        int pivot = col;
        double pivotAbs = std::fabs(a[col][col]);
        for (int row = col + 1; row < kMagFitTerms; ++row) {
            const double av = std::fabs(a[row][col]);
            if (av > pivotAbs) {
                pivotAbs = av;
                pivot = row;
            }
        }

        if (!std::isfinite(pivotAbs) || pivotAbs <= 1.0e-12) return false;
        if (pivotAbs < minPivot) minPivot = pivotAbs;
        if (pivotAbs > maxPivot) maxPivot = pivotAbs;

        if (pivot != col) {
            for (int c = col; c <= kMagFitTerms; ++c) {
                const double tmp = a[col][c];
                a[col][c] = a[pivot][c];
                a[pivot][c] = tmp;
            }
        }

        const double invPivot = 1.0 / a[col][col];
        for (int c = col; c <= kMagFitTerms; ++c) a[col][c] *= invPivot;

        for (int row = 0; row < kMagFitTerms; ++row) {
            if (row == col) continue;
            const double f = a[row][col];
            if (f == 0.0) continue;
            for (int c = col; c <= kMagFitTerms; ++c) a[row][c] -= f * a[col][c];
        }
    }

    if (!std::isfinite(minPivot) || !std::isfinite(maxPivot) || maxPivot <= 0.0) return false;
    const double pivotRatio = minPivot / maxPivot;
    if (!std::isfinite(pivotRatio) || pivotRatio <= 1.0e-10) return false;
    pivotRatioOut = static_cast<float>(pivotRatio);

    for (int i = 0; i < kMagFitTerms; ++i) {
        outX[i] = a[i][kMagFitTerms] / columnScale[i];
        if (!std::isfinite(outX[i])) return false;
    }
    return true;
}

bool inverseSymmetric3(const double a[3][3], double out[3][3]) {
    const double c00 = a[1][1] * a[2][2] - a[1][2] * a[1][2];
    const double c01 = a[0][2] * a[1][2] - a[0][1] * a[2][2];
    const double c02 = a[0][1] * a[1][2] - a[0][2] * a[1][1];
    const double c11 = a[0][0] * a[2][2] - a[0][2] * a[0][2];
    const double c12 = a[0][1] * a[0][2] - a[0][0] * a[1][2];
    const double c22 = a[0][0] * a[1][1] - a[0][1] * a[0][1];
    const double det = a[0][0] * c00 + a[0][1] * c01 + a[0][2] * c02;
    const double scale = std::fabs(a[0][0]) + std::fabs(a[1][1]) + std::fabs(a[2][2]);
    const double eps = std::max(1.0e-15, scale * scale * scale * 1.0e-12);
    if (!std::isfinite(det) || std::fabs(det) <= eps) return false;
    const double invDet = 1.0 / det;
    out[0][0] = c00 * invDet; out[0][1] = c01 * invDet; out[0][2] = c02 * invDet;
    out[1][0] = c01 * invDet; out[1][1] = c11 * invDet; out[1][2] = c12 * invDet;
    out[2][0] = c02 * invDet; out[2][1] = c12 * invDet; out[2][2] = c22 * invDet;
    return true;
}

bool jacobiEigenSymmetric3(const double inA[3][3], double eigVec[3][3], double eigVal[3]) {
    double a[3][3] = {
        {inA[0][0], inA[0][1], inA[0][2]},
        {inA[1][0], inA[1][1], inA[1][2]},
        {inA[2][0], inA[2][1], inA[2][2]},
    };

    eigVec[0][0] = 1.0; eigVec[0][1] = 0.0; eigVec[0][2] = 0.0;
    eigVec[1][0] = 0.0; eigVec[1][1] = 1.0; eigVec[1][2] = 0.0;
    eigVec[2][0] = 0.0; eigVec[2][1] = 0.0; eigVec[2][2] = 1.0;

    for (int iter = 0; iter < 40; ++iter) {
        int p = 0;
        int q = 1;
        double maxOff = std::fabs(a[0][1]);
        const double a02 = std::fabs(a[0][2]);
        const double a12 = std::fabs(a[1][2]);
        if (a02 > maxOff) { maxOff = a02; p = 0; q = 2; }
        if (a12 > maxOff) { maxOff = a12; p = 1; q = 2; }

        const double diagScale = std::fabs(a[0][0]) + std::fabs(a[1][1]) + std::fabs(a[2][2]);
        if (maxOff <= (diagScale > 0.0 ? diagScale * 1.0e-12 : 1.0e-12)) break;

        const double app = a[p][p];
        const double aqq = a[q][q];
        const double apq = a[p][q];
        const double phi = 0.5 * std::atan2(2.0 * apq, aqq - app);
        const double c = std::cos(phi);
        const double s = std::sin(phi);

        for (int k = 0; k < 3; ++k) {
            if (k == p || k == q) continue;
            const double akp = a[k][p];
            const double akq = a[k][q];
            const double newKp = c * akp - s * akq;
            const double newKq = s * akp + c * akq;
            a[k][p] = a[p][k] = newKp;
            a[k][q] = a[q][k] = newKq;
        }

        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        a[p][q] = a[q][p] = 0.0;

        for (int k = 0; k < 3; ++k) {
            const double vkp = eigVec[k][p];
            const double vkq = eigVec[k][q];
            eigVec[k][p] = c * vkp - s * vkq;
            eigVec[k][q] = s * vkp + c * vkq;
        }
    }

    for (int i = 0; i < 3; ++i) {
        eigVal[i] = a[i][i];
        if (!std::isfinite(eigVal[i])) return false;
        for (int j = 0; j < 3; ++j) {
            if (!std::isfinite(eigVec[i][j])) return false;
        }
    }
    return true;
}

Mat3 makeSoftIronFromEigen(const double eigVec[3][3], const double eigVal[3], double targetRadius) {
    double gain[3] = {};
    for (int k = 0; k < 3; ++k) gain[k] = targetRadius * std::sqrt(eigVal[k]);

    return Mat3(
        static_cast<float>(eigVec[0][0] * gain[0] * eigVec[0][0] + eigVec[0][1] * gain[1] * eigVec[0][1] + eigVec[0][2] * gain[2] * eigVec[0][2]),
        static_cast<float>(eigVec[0][0] * gain[0] * eigVec[1][0] + eigVec[0][1] * gain[1] * eigVec[1][1] + eigVec[0][2] * gain[2] * eigVec[1][2]),
        static_cast<float>(eigVec[0][0] * gain[0] * eigVec[2][0] + eigVec[0][1] * gain[1] * eigVec[2][1] + eigVec[0][2] * gain[2] * eigVec[2][2]),

        static_cast<float>(eigVec[1][0] * gain[0] * eigVec[0][0] + eigVec[1][1] * gain[1] * eigVec[0][1] + eigVec[1][2] * gain[2] * eigVec[0][2]),
        static_cast<float>(eigVec[1][0] * gain[0] * eigVec[1][0] + eigVec[1][1] * gain[1] * eigVec[1][1] + eigVec[1][2] * gain[2] * eigVec[1][2]),
        static_cast<float>(eigVec[1][0] * gain[0] * eigVec[2][0] + eigVec[1][1] * gain[1] * eigVec[2][1] + eigVec[1][2] * gain[2] * eigVec[2][2]),

        static_cast<float>(eigVec[2][0] * gain[0] * eigVec[0][0] + eigVec[2][1] * gain[1] * eigVec[0][1] + eigVec[2][2] * gain[2] * eigVec[0][2]),
        static_cast<float>(eigVec[2][0] * gain[0] * eigVec[1][0] + eigVec[2][1] * gain[1] * eigVec[1][1] + eigVec[2][2] * gain[2] * eigVec[1][2]),
        static_cast<float>(eigVec[2][0] * gain[0] * eigVec[2][0] + eigVec[2][1] * gain[1] * eigVec[2][1] + eigVec[2][2] * gain[2] * eigVec[2][2])
    );
}

TRACKER_MAG_FIT_NOINLINE bool fitFromAccumulator(
    const FitAccumulator& acc,
    const FitNormalization& normalization,
    const MagCalibrationParams& params,
    FitCandidate& out) {
    out = FitCandidate{};
    out.normalization = normalization;
    out.solverSamples = acc.count;
    if (!normalization.valid || acc.count < params.minSamples) {
        out.failureReason = MagCalibrationFailureReason::FitNormalizationFailed;
        return false;
    }
    out.solverStage = MagCalibrationSolverStage::Normalized;

    double coeff[kMagFitTerms] = {};
    if (!solveLinear9(acc.normal, acc.rhs, coeff, out.solverPivotRatio)) {
        out.failureReason = MagCalibrationFailureReason::LinearSolveFailed;
        return false;
    }
    out.solverStage = MagCalibrationSolverStage::LinearSolved;

    const double quad[3][3] = {
        {coeff[0], coeff[3], coeff[4]},
        {coeff[3], coeff[1], coeff[5]},
        {coeff[4], coeff[5], coeff[2]},
    };
    double quadInv[3][3] = {};
    if (!inverseSymmetric3(quad, quadInv)) {
        out.failureReason = MagCalibrationFailureReason::QuadraticCenterFailed;
        return false;
    }

    const double lin[3] = {coeff[6], coeff[7], coeff[8]};
    double centerNormalized[3] = {};
    for (int r = 0; r < 3; ++r) {
        centerNormalized[r] = -0.5 * (
            quadInv[r][0] * lin[0] +
            quadInv[r][1] * lin[1] +
            quadInv[r][2] * lin[2]);
        if (!std::isfinite(centerNormalized[r])) {
            out.failureReason = MagCalibrationFailureReason::QuadraticCenterFailed;
            return false;
        }
    }
    out.solverStage = MagCalibrationSolverStage::CenterSolved;

    double centerQuad = 0.0;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            centerQuad += centerNormalized[r] * quad[r][c] * centerNormalized[c];
        }
    }
    const double k = 1.0 + centerQuad;
    if (!std::isfinite(k) || k <= 1.0e-12) {
        out.failureReason = MagCalibrationFailureReason::NonPositiveDefiniteShape;
        return false;
    }

    // Convert the centered normalized-coordinate shape back to raw sensor
    // coordinates. For x = origin + D*y, Sx = D^-T * Sy * D^-1.
    double shapeRaw[3][3] = {};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            shapeRaw[r][c] = quad[r][c] /
                (k * normalization.scale[r] * normalization.scale[c]);
            if (!std::isfinite(shapeRaw[r][c])) {
                out.failureReason = MagCalibrationFailureReason::NonPositiveDefiniteShape;
                return false;
            }
        }
    }
    out.solverStage = MagCalibrationSolverStage::ShapeSolved;

    double eigVec[3][3] = {};
    double eigVal[3] = {};
    if (!jacobiEigenSymmetric3(shapeRaw, eigVec, eigVal)) {
        out.failureReason = MagCalibrationFailureReason::NonPositiveDefiniteShape;
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(eigVal[i]) || eigVal[i] <= 0.0) {
            out.failureReason = MagCalibrationFailureReason::NonPositiveDefiniteShape;
            return false;
        }
    }
    out.solverStage = MagCalibrationSolverStage::EigenSolved;

    const double r0 = 1.0 / std::sqrt(eigVal[0]);
    const double r1 = 1.0 / std::sqrt(eigVal[1]);
    const double r2 = 1.0 / std::sqrt(eigVal[2]);
    double minRadius = std::min(r0, std::min(r1, r2));
    double maxRadius = std::max(r0, std::max(r1, r2));
    if (!std::isfinite(minRadius) || !std::isfinite(maxRadius) ||
        minRadius < static_cast<double>(params.minAxisRadius)) {
        out.failureReason = MagCalibrationFailureReason::EllipsoidFitFailed;
        return false;
    }
    const double axisRatio = maxRadius / minRadius;
    if (!std::isfinite(axisRatio) || axisRatio > static_cast<double>(params.maxAxisRatio)) {
        out.failureReason = MagCalibrationFailureReason::EllipsoidFitFailed;
        return false;
    }
    const double targetRadius = (r0 + r1 + r2) / 3.0;
    if (!std::isfinite(targetRadius) || targetRadius <= 0.0) {
        out.failureReason = MagCalibrationFailureReason::EllipsoidFitFailed;
        return false;
    }

    const Vec3 centerRaw(
        static_cast<float>(normalization.center[0] + normalization.scale[0] * centerNormalized[0]),
        static_cast<float>(normalization.center[1] + normalization.scale[1] * centerNormalized[1]),
        static_cast<float>(normalization.center[2] + normalization.scale[2] * centerNormalized[2]));
    if (!centerRaw.isFinite()) {
        out.failureReason = MagCalibrationFailureReason::QuadraticCenterFailed;
        return false;
    }

    double pNp = 0.0;
    for (int i = 0; i < kMagFitTerms; ++i) {
        for (int j = 0; j < kMagFitTerms; ++j) {
            pNp += coeff[i] * acc.normal[i][j] * coeff[j];
        }
    }
    double pRhs = 0.0;
    for (int i = 0; i < kMagFitTerms; ++i) pRhs += coeff[i] * acc.rhs[i];
    double residualVar = (pNp - 2.0 * pRhs + static_cast<double>(acc.count)) /
        static_cast<double>(acc.count);
    if (residualVar < 0.0 && residualVar > -1.0e-9) residualVar = 0.0;
    if (!std::isfinite(residualVar) || residualVar < 0.0) {
        out.failureReason = MagCalibrationFailureReason::EllipsoidFitFailed;
        return false;
    }
    // The translation-invariant and approximately twice first-order radial
    // error for a normalized ellipsoid. Divide by the completed-square scale
    // so the metric is independent of hard-iron translation.
    const double residualRms = std::sqrt(residualVar) / std::fabs(k);
    if (!std::isfinite(residualRms)) {
        out.failureReason = MagCalibrationFailureReason::EllipsoidFitFailed;
        return false;
    }

    const Mat3 softIron = makeSoftIronFromEigen(eigVec, eigVal, targetRadius);
    Mat3 softIronInv;
    if (!softIron.isFinite() || softIron.determinant() <= 0.0f ||
        !softIron.inverse(softIronInv, 1.0e-9f)) {
        out.failureReason = MagCalibrationFailureReason::NonPositiveDefiniteShape;
        return false;
    }

    out.valid = true;
    out.failureReason = MagCalibrationFailureReason::None;
    out.solverStage = MagCalibrationSolverStage::CandidateBuilt;
    out.hardIron = centerRaw;
    out.softIron = softIron;
    out.expectedNorm = static_cast<float>(targetRadius);
    out.radiusX = static_cast<float>(r0);
    out.radiusY = static_cast<float>(r1);
    out.radiusZ = static_cast<float>(r2);
    out.axisRatio = static_cast<float>(axisRatio);
    out.algebraicResidualRms = static_cast<float>(residualRms);
    return true;
}

TRACKER_MAG_FIT_NOINLINE bool replaceFitFromAccumulator(
    const FitAccumulator& acc,
    const FitNormalization& normalization,
    const MagCalibrationParams& params,
    FitCandidate& inOut) {
    FitCandidate replacement;
    if (!fitFromAccumulator(acc, normalization, params, replacement)) {
        return false;
    }
    inOut = replacement;
    return true;
}

float residualAbs(const FitCandidate& fit, const MagCalibrationStoredSample& s) {
    const Vec3 corrected = fit.softIron * (storedToVec3(s) - fit.hardIron);
    const float n = corrected.norm();
    if (!tracker::isFinite(n)) return 999999.0f;
    return std::fabs(n - fit.expectedNorm);
}

GeometricMetrics computeGeometricMetrics(const FitCandidate& fit,
                                          const MagCalibrationStoredSample* samples,
                                          uint16_t count,
                                          float inlierThreshold) {
    GeometricMetrics m;
    if (!fit.valid || !samples || count == 0 || fit.expectedNorm <= 1.0e-6f) return m;

    double sumSq = 0.0;
    uint16_t faceBins[6] = {};
    for (uint16_t i = 0; i < count; ++i) {
        const Vec3 corrected = fit.softIron * (storedToVec3(samples[i]) - fit.hardIron);
        const float n = corrected.norm();
        if (!corrected.isFinite() || !tracker::isFinite(n) || n <= 1.0e-6f) continue;
        const float absResidual = std::fabs(n - fit.expectedNorm);
        if (absResidual > inlierThreshold) continue;

        sumSq += static_cast<double>(absResidual) * static_cast<double>(absResidual);
        m.inliers++;

        const Vec3 unit = corrected / n;
        const float ax = std::fabs(unit.x);
        const float ay = std::fabs(unit.y);
        const float az = std::fabs(unit.z);
        uint8_t axis = 0;
        float sign = unit.x;
        float best = ax;
        if (ay > best) { best = ay; axis = 1; sign = unit.y; }
        if (az > best) { axis = 2; sign = unit.z; }
        const uint8_t bin = static_cast<uint8_t>(axis * 2 + (sign >= 0.0f ? 0 : 1));
        if (bin < 6) faceBins[bin]++;
    }

    if (m.inliers == 0) return m;
    m.rms = static_cast<float>(std::sqrt(sumSq / static_cast<double>(m.inliers)));
    m.normalizedRms = m.rms / fit.expectedNorm;
    m.inlierRatio = static_cast<float>(m.inliers) / static_cast<float>(count);

    uint8_t occupied = 0;
    uint16_t minOccupied = 0xFFFFu;
    const uint16_t minFaceSamples = static_cast<uint16_t>(m.inliers / 50U + 2U);
    for (uint8_t i = 0; i < 6; ++i) {
        if (faceBins[i] >= minFaceSamples) {
            occupied++;
            if (faceBins[i] < minOccupied) minOccupied = faceBins[i];
        }
    }
    if (occupied == 0) {
        m.directionalCoverageScore = 0.0f;
    } else {
        const float occupiedScore = static_cast<float>(occupied) / 6.0f;
        const float expectedPerFace = static_cast<float>(m.inliers) / 6.0f;
        const float balance = expectedPerFace > 1.0f ? clampf(static_cast<float>(minOccupied) / expectedPerFace, 0.0f, 1.0f) : 0.0f;
        // Mostly require all six hemispheres to be present, but give a small
        // bonus for balanced coverage so the user gets meaningful guidance.
        m.directionalCoverageScore = clampf(0.75f * occupiedScore + 0.25f * balance, 0.0f, 1.0f);
    }
    return m;
}

float robustThreshold(const MagCalibrationParams& params, const FitCandidate& fit, const GeometricMetrics& m) {
    const float sigma = params.outlierSigma > 0.5f ? params.outlierSigma : 3.0f;
    const float minFactor = params.outlierMinResidualFactor > 0.0f ? params.outlierMinResidualFactor : 0.08f;
    const float bySigma = tracker::isFinite(m.rms) ? sigma * m.rms : 0.0f;
    const float byFloor = minFactor * fit.expectedNorm;
    const float byQualityCap = magCalibrationRobustResidualCapFactor(params) * fit.expectedNorm;
    const float boundedSigma = bySigma < byQualityCap ? bySigma : byQualityCap;
    return boundedSigma > byFloor ? boundedSigma : byFloor;
}

bool accumulateInliers(const MagCalibrationStoredSample* samples,
                       uint16_t count,
                       const FitCandidate& reference,
                       float threshold,
                       const FitNormalization& normalization,
                       FitAccumulator& out,
                       uint32_t membershipOut[kInlierMembershipWords]) {
    resetAccumulator(out);
    std::memset(membershipOut, 0, sizeof(uint32_t) * kInlierMembershipWords);
    for (uint16_t i = 0; i < count; ++i) {
        if (residualAbs(reference, samples[i]) <= threshold) {
            accumulateSample(out, storedToVec3(samples[i]), normalization);
            membershipOut[i / 32u] |= 1u << (i % 32u);
        }
    }
    return out.count > 0;
}


TRACKER_MAG_FIT_NOINLINE bool accumulateCenteredNormFiltered(
    const MagCalibrationStoredSample* samples,
    uint16_t count,
    const MagCalibrationParams& params,
    FitNormalization& normalization,
    FitAccumulator& out,
    MagCalibrationFailureReason& failureReason,
    float& boxCoverageScoreOut,
    float& minBoxRadiusOut) {
    resetAccumulator(out);
    normalization = FitNormalization{};
    failureReason = MagCalibrationFailureReason::RawNormFilterFailed;
    boxCoverageScoreOut = 0.0f;
    minBoxRadiusOut = 0.0f;
    if (!samples || count == 0u) return false;

    double mean[3] = {};
    uint32_t finiteCount = 0u;
    for (uint16_t i = 0; i < count; ++i) {
        const Vec3 v = storedToVec3(samples[i]);
        if (!v.isFinite()) continue;
        mean[0] += static_cast<double>(v.x);
        mean[1] += static_cast<double>(v.y);
        mean[2] += static_cast<double>(v.z);
        finiteCount++;
    }
    if (finiteCount < params.minSamples) return false;
    const double invFiniteCount = 1.0 / static_cast<double>(finiteCount);
    for (double& v : mean) v *= invFiniteCount;

    // The old filter used |raw| and therefore changed when hard-iron moved
    // the same ellipsoid relative to the ADC origin. Filter by distance from
    // the sample centroid instead, then derive the final affine normalization
    // from exactly the retained set.
    double radiusSum = 0.0;
    double radiusSumSq = 0.0;
    for (uint16_t i = 0; i < count; ++i) {
        const Vec3 v = storedToVec3(samples[i]);
        const double dx = static_cast<double>(v.x) - mean[0];
        const double dy = static_cast<double>(v.y) - mean[1];
        const double dz = static_cast<double>(v.z) - mean[2];
        const double radius = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (!std::isfinite(radius)) continue;
        radiusSum += radius;
        radiusSumSq += radius * radius;
    }
    const double radiusMean = radiusSum * invFiniteCount;
    double radiusVar = radiusSumSq * invFiniteCount - radiusMean * radiusMean;
    if (radiusVar < 0.0 && radiusVar > -1.0e-6) radiusVar = 0.0;
    if (!std::isfinite(radiusVar) || radiusVar < 0.0) return false;
    const double radiusStd = std::sqrt(radiusVar);
    const double lo = std::max(0.0, radiusMean - 3.5 * radiusStd);
    const double hi = radiusMean + 3.5 * radiusStd;

    double retainedSum[3] = {};
    double retainedSumSq[3] = {};
    uint32_t retained = 0u;
    for (uint16_t i = 0; i < count; ++i) {
        const Vec3 v = storedToVec3(samples[i]);
        const double dx = static_cast<double>(v.x) - mean[0];
        const double dy = static_cast<double>(v.y) - mean[1];
        const double dz = static_cast<double>(v.z) - mean[2];
        const double radius = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (!std::isfinite(radius) || radius < lo || radius > hi) continue;
        const double values[3] = {
            static_cast<double>(v.x),
            static_cast<double>(v.y),
            static_cast<double>(v.z)};
        for (int axis = 0; axis < 3; ++axis) {
            retainedSum[axis] += values[axis];
            retainedSumSq[axis] += values[axis] * values[axis];
        }
        retained++;
    }
    if (retained < params.minSamples) return false;

    for (int axis = 0; axis < 3; ++axis) {
        normalization.center[axis] = retainedSum[axis] / static_cast<double>(retained);
        double variance = retainedSumSq[axis] / static_cast<double>(retained) -
            normalization.center[axis] * normalization.center[axis];
        if (variance < 0.0 && variance > -1.0e-6) variance = 0.0;
        if (!std::isfinite(variance) || variance <= 1.0e-6) {
            failureReason = MagCalibrationFailureReason::FitNormalizationFailed;
            return false;
        }
        normalization.scale[axis] = std::sqrt(variance);
        if (!std::isfinite(normalization.scale[axis]) ||
            normalization.scale[axis] < 1.0) {
            failureReason = MagCalibrationFailureReason::FitNormalizationFailed;
            return false;
        }
    }
    normalization.valid = true;
    double retainedMin[3] = {1.0e300, 1.0e300, 1.0e300};
    double retainedMax[3] = {-1.0e300, -1.0e300, -1.0e300};

    for (uint16_t i = 0; i < count; ++i) {
        const Vec3 v = storedToVec3(samples[i]);
        const double dx = static_cast<double>(v.x) - mean[0];
        const double dy = static_cast<double>(v.y) - mean[1];
        const double dz = static_cast<double>(v.z) - mean[2];
        const double radius = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (std::isfinite(radius) && radius >= lo && radius <= hi) {
            const double values[3] = {
                static_cast<double>(v.x),
                static_cast<double>(v.y),
                static_cast<double>(v.z)};
            for (int axis = 0; axis < 3; ++axis) {
                if (values[axis] < retainedMin[axis]) retainedMin[axis] = values[axis];
                if (values[axis] > retainedMax[axis]) retainedMax[axis] = values[axis];
            }
            accumulateSample(out, v, normalization);
        }
    }
    if (out.count >= params.minSamples) {
        const double radiusX = 0.5 * (retainedMax[0] - retainedMin[0]);
        const double radiusY = 0.5 * (retainedMax[1] - retainedMin[1]);
        const double radiusZ = 0.5 * (retainedMax[2] - retainedMin[2]);
        const double minRadius = std::min(radiusX, std::min(radiusY, radiusZ));
        const double maxRadius = std::max(radiusX, std::max(radiusY, radiusZ));
        minBoxRadiusOut = static_cast<float>(minRadius);
        boxCoverageScoreOut = maxRadius > 0.0
            ? static_cast<float>(minRadius / maxRadius)
            : 0.0f;
    }
    failureReason = out.count >= params.minSamples
        ? MagCalibrationFailureReason::None
        : MagCalibrationFailureReason::RawNormFilterFailed;
    return out.count >= params.minSamples;
}

void copyFitDiagnostics(const FitCandidate& fit, MagCalibrationResult& out) {
    out.fitAvailable = fit.valid;
    out.solverStage = fit.solverStage;
    out.fitNormalizationCenter = Vec3(
        static_cast<float>(fit.normalization.center[0]),
        static_cast<float>(fit.normalization.center[1]),
        static_cast<float>(fit.normalization.center[2]));
    out.fitNormalizationScale = Vec3(
        static_cast<float>(fit.normalization.scale[0]),
        static_cast<float>(fit.normalization.scale[1]),
        static_cast<float>(fit.normalization.scale[2]));
    out.solverPivotRatio = fit.solverPivotRatio;
    out.solverSamples = fit.solverSamples;
}

} // namespace

MagCalibrationCollector::MagCalibrationCollector(const MagCalibrationParams& params)
    : params_(params) {}

void MagCalibrationCollector::setParams(const MagCalibrationParams& params) {
    params_ = params;
}

const MagCalibrationParams& MagCalibrationCollector::params() const {
    return params_;
}

void MagCalibrationCollector::reset() {
    active_ = false;
    hasData_ = false;
    samples_ = 0;
    rejected_ = 0;
    saturated_ = 0;
    startMs_ = 0;
    lastSampleMs_ = 0;

    minX_ = minY_ = minZ_ = 0.0f;
    maxX_ = maxY_ = maxZ_ = 0.0f;
    sumX_ = sumY_ = sumZ_ = 0.0;
    normMin_ = normMax_ = 0.0f;
    normSum_ = 0.0;
    storedSamples_ = 0;
    storedSequence_ = 0;
    reservoirReplacements_ = 0;
    reservoirSkipped_ = 0;
    for (auto& s : stored_) s = MagCalibrationStoredSample{};
    lastResult_ = MagCalibrationResult{};
    lastFailureReason_ = MagCalibrationFailureReason::None;
}

void MagCalibrationCollector::start(uint32_t nowMs) {
    reset();
    active_ = true;
    startMs_ = nowMs;
}

void MagCalibrationCollector::stop() {
    active_ = false;
}

void MagCalibrationCollector::push(const Lsm6dsvFifoReader::MagRawSample& m, float normRaw, uint32_t nowMs) {
    if (!active_) return;

    if ((m.flags & Lsm6dsvFifoReader::MAG_FLAG_RAW_SATURATED) != 0) {
        saturated_++;
        rejected_++;
        return;
    }

    const double x = static_cast<double>(m.x);
    const double y = static_cast<double>(m.y);
    const double z = static_cast<double>(m.z);

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !tracker::isFinite(normRaw)) {
        rejected_++;
        return;
    }

    const float xf = static_cast<float>(x);
    const float yf = static_cast<float>(y);
    const float zf = static_cast<float>(z);

    if (!hasData_) {
        minX_ = maxX_ = xf;
        minY_ = maxY_ = yf;
        minZ_ = maxZ_ = zf;
        normMin_ = normMax_ = normRaw;
        hasData_ = true;
    } else {
        if (xf < minX_) minX_ = xf;
        if (xf > maxX_) maxX_ = xf;
        if (yf < minY_) minY_ = yf;
        if (yf > maxY_) maxY_ = yf;
        if (zf < minZ_) minZ_ = zf;
        if (zf > maxZ_) maxZ_ = zf;
        if (normRaw < normMin_) normMin_ = normRaw;
        if (normRaw > normMax_) normMax_ = normRaw;
    }

    const MagCalibrationStoredSample stored{m.x, m.y, m.z};
    const uint32_t seenCount = storedSequence_ + 1u;
    if (storedSamples_ < kMaxStoredSamples) {
        stored_[storedSamples_++] = stored;
    } else {
        // Deterministic Algorithm-R reservoir sampling.  Every accepted sample
        // seen since start() has the same probability of remaining in the
        // bounded fit set.  The previous implementation permuted all 768 slots
        // repeatedly, so after a few seconds it retained only the most recent
        // orientations and could erase otherwise-good full-sphere coverage.
        const uint32_t candidate = deterministicReservoirCandidate(
            storedSequence_, seenCount, 0x9E3779B9u);
        if (candidate < kMaxStoredSamples) {
            stored_[static_cast<uint16_t>(candidate)] = stored;
            reservoirReplacements_++;
        } else {
            reservoirSkipped_++;
        }
    }
    storedSequence_ = seenCount;

    sumX_ += x;
    sumY_ += y;
    sumZ_ += z;
    normSum_ += static_cast<double>(normRaw);
    samples_++;
    lastSampleMs_ = nowMs;
}

bool MagCalibrationCollector::compute(MagCalibrationResult& out) {
    out = MagCalibrationResult{};
    lastFailureReason_ = MagCalibrationFailureReason::None;

    const uint32_t minSamples = params_.minSamples;
    if (!hasData_ || samples_ < minSamples || storedSamples_ < minSamples) {
        lastFailureReason_ = MagCalibrationFailureReason::InsufficientSamples;
        lastResult_ = out;
        return false;
    }

    // All fit gates must describe the same bounded set that is actually fed
    // to the ellipsoid solver. Full-capture extrema are useful diagnostics,
    // but using them here can claim good coverage from samples that no longer
    // exist in the fit reservoir.
    const MagCalibrationFitSetDiagnostics fitSet = fitSetDiagnostics();
    if (!fitSet.valid || fitSet.samples < minSamples) {
        lastFailureReason_ = MagCalibrationFailureReason::InsufficientSamples;
        lastResult_ = out;
        return false;
    }

    // FitAccumulator is the dominant local object (9x9 doubles plus RHS).
    // Reuse one workspace for the raw and inlier passes instead of keeping two
    // simultaneously live.  This preserves the exact fit/refit semantics while
    // leaving a safe cross-ABI stack margin on MSYS2 and the ESP toolchain.
    FitAccumulator fitAccumulator;
    FitNormalization fitNormalization;
    MagCalibrationFailureReason accumulationFailure = MagCalibrationFailureReason::RawNormFilterFailed;
    float boxCoverageScore = 0.0f;
    float minBoxRadius = 0.0f;
    if (!accumulateCenteredNormFiltered(
            stored_, storedSamples_, params_, fitNormalization, fitAccumulator,
            accumulationFailure, boxCoverageScore, minBoxRadius) ||
        fitAccumulator.count < minSamples) {
        out.solverStage = fitNormalization.valid
            ? MagCalibrationSolverStage::Normalized
            : MagCalibrationSolverStage::None;
        out.fitNormalizationCenter = Vec3(
            static_cast<float>(fitNormalization.center[0]),
            static_cast<float>(fitNormalization.center[1]),
            static_cast<float>(fitNormalization.center[2]));
        out.fitNormalizationScale = Vec3(
            static_cast<float>(fitNormalization.scale[0]),
            static_cast<float>(fitNormalization.scale[1]),
            static_cast<float>(fitNormalization.scale[2]));
        out.solverSamples = fitAccumulator.count;
        lastFailureReason_ = accumulationFailure;
        lastResult_ = out;
        return false;
    }
    // Normalization and retained-sample diagnostics are already authoritative
    // at this point. Preserve them even if a pre-solve physical coverage gate
    // rejects the dataset, so guided setup never reports an opaque all-zero
    // solver state after successfully examining the samples.
    out.solverStage = MagCalibrationSolverStage::Normalized;
    out.fitNormalizationCenter = Vec3(
        static_cast<float>(fitNormalization.center[0]),
        static_cast<float>(fitNormalization.center[1]),
        static_cast<float>(fitNormalization.center[2]));
    out.fitNormalizationScale = Vec3(
        static_cast<float>(fitNormalization.scale[0]),
        static_cast<float>(fitNormalization.scale[1]),
        static_cast<float>(fitNormalization.scale[2]));
    out.solverSamples = fitAccumulator.count;

    // Coverage and minimum-radius gates describe the same centered, robustly
    // retained set that enters the normal equations. Full-reservoir extrema
    // remain diagnostics only and cannot veto the fit before filtering.
    if (!tracker::isFinite(minBoxRadius) || minBoxRadius < params_.minAxisRadius) {
        lastFailureReason_ = MagCalibrationFailureReason::AxisRadiusTooSmall;
        lastResult_ = out;
        return false;
    }
    const float effectiveMinBoxCoverage = magCalibrationEffectiveMinBoxCoverage(params_);
    if (!tracker::isFinite(boxCoverageScore) ||
        boxCoverageScore < effectiveMinBoxCoverage) {
        lastFailureReason_ = MagCalibrationFailureReason::BoxCoverageTooLow;
        lastResult_ = out;
        return false;
    }

    const uint32_t rawFitSamples = fitAccumulator.count;

    FitCandidate finalFit;
    if (!fitFromAccumulator(fitAccumulator, fitNormalization, params_, finalFit)) {
        copyFitDiagnostics(finalFit, out);
        lastFailureReason_ = finalFit.failureReason;
        lastResult_ = out;
        return false;
    }

    GeometricMetrics metrics;
    uint8_t robustRefitPasses = 0u;
    uint32_t previousMembership[kInlierMembershipWords] = {};
    uint32_t membership[kInlierMembershipWords] = {};
    uint32_t previousInlierCount = 0u;
    bool havePreviousMembership = false;
    constexpr uint8_t kMaxRobustRefitPasses = 3u;
    for (uint8_t pass = 0u; pass < kMaxRobustRefitPasses; ++pass) {
        metrics = computeGeometricMetrics(finalFit, stored_, storedSamples_, 999999.0f);
        const float threshold = robustThreshold(params_, finalFit, metrics);
        if (!accumulateInliers(
                stored_, storedSamples_, finalFit, threshold,
                fitNormalization, fitAccumulator, membership) ||
            fitAccumulator.count < minSamples ||
            static_cast<float>(fitAccumulator.count) / static_cast<float>(storedSamples_) < params_.minInlierRatio ||
            fitAccumulator.count >= rawFitSamples) {
            break;
        }
        if (havePreviousMembership &&
            fitAccumulator.count == previousInlierCount &&
            std::memcmp(membership, previousMembership, sizeof(membership)) == 0) {
            break;
        }
        if (!replaceFitFromAccumulator(
                fitAccumulator, fitNormalization, params_, finalFit)) {
            break;
        }
        std::memcpy(previousMembership, membership, sizeof(previousMembership));
        previousInlierCount = fitAccumulator.count;
        havePreviousMembership = true;
        robustRefitPasses++;
    }

    metrics = computeGeometricMetrics(finalFit, stored_, storedSamples_, 999999.0f);
    const float finalThreshold = robustThreshold(params_, finalFit, metrics);
    metrics = computeGeometricMetrics(finalFit, stored_, storedSamples_, finalThreshold);
    const GeometricMetrics& finalMetrics = metrics;

    copyFitDiagnostics(finalFit, out);

    // Keep the best finite fit diagnostics even when a quality gate rejects
    // the model.  Guided setup can then distinguish poor physical data from a
    // solver/policy defect instead of printing only the terminal enum.
    out.hardIron = finalFit.hardIron;
    out.softIron = finalFit.softIron;
    out.expectedNorm = finalFit.expectedNorm;
    out.minTrustNorm = static_cast<float>(static_cast<double>(finalFit.expectedNorm) * static_cast<double>(params_.trustNormMinFactor));
    out.maxTrustNorm = static_cast<float>(static_cast<double>(finalFit.expectedNorm) * static_cast<double>(params_.trustNormMaxFactor));
    out.radiusX = finalFit.radiusX;
    out.radiusY = finalFit.radiusY;
    out.radiusZ = finalFit.radiusZ;
    out.coverageScore = boxCoverageScore;
    out.directionalCoverageScore = finalMetrics.directionalCoverageScore;
    out.residualRms = finalFit.algebraicResidualRms;
    out.geometricResidualRms = finalMetrics.rms;
    out.normalizedResidualRms = finalMetrics.normalizedRms;
    out.axisRatio = finalFit.axisRatio;
    out.inlierRatio = finalMetrics.inlierRatio;
    out.inlierSamples = finalMetrics.inliers;
    out.robustRefitPasses = robustRefitPasses;
    out.robustInlierThresholdFactor = finalFit.expectedNorm > 1.0e-6f
        ? finalThreshold / finalFit.expectedNorm
        : 0.0f;

    if (finalMetrics.inliers < minSamples || finalMetrics.inlierRatio < params_.minInlierRatio) {
        lastFailureReason_ = MagCalibrationFailureReason::InlierRatioTooLow;
        lastResult_ = out;
        return false;
    }
    if (!tracker::isFinite(finalMetrics.normalizedRms) ||
        finalMetrics.normalizedRms > params_.maxGeometricResidualRmsFactor) {
        lastFailureReason_ = MagCalibrationFailureReason::GeometricResidualTooHigh;
        lastResult_ = out;
        return false;
    }
    if (!tracker::isFinite(finalMetrics.directionalCoverageScore) ||
        finalMetrics.directionalCoverageScore < params_.minDirectionalCoverageScore) {
        lastFailureReason_ = MagCalibrationFailureReason::DirectionalCoverageTooLow;
        lastResult_ = out;
        return false;
    }
    if (!tracker::isFinite(finalFit.algebraicResidualRms) ||
        finalFit.algebraicResidualRms > magCalibrationEffectiveMaxAlgebraicResidualRms(params_)) {
        lastFailureReason_ = MagCalibrationFailureReason::AlgebraicResidualTooHigh;
        lastResult_ = out;
        return false;
    }

    out.valid = true;
    lastResult_ = out;
    return true;
}

MagCalibrationFailureReason MagCalibrationCollector::lastFailureReason() const { return lastFailureReason_; }

const char* MagCalibrationCollector::lastFailureReasonName() const {
    return magCalibrationFailureReasonName(lastFailureReason_);
}


bool MagCalibrationCollector::active() const { return active_; }
bool MagCalibrationCollector::hasData() const { return hasData_; }
uint32_t MagCalibrationCollector::samples() const { return samples_; }
uint16_t MagCalibrationCollector::storedSamples() const { return storedSamples_; }
uint32_t MagCalibrationCollector::reservoirReplacements() const { return reservoirReplacements_; }
uint32_t MagCalibrationCollector::reservoirSkipped() const { return reservoirSkipped_; }

MagCalibrationFitSetDiagnostics MagCalibrationCollector::fitSetDiagnostics() const {
    MagCalibrationFitSetDiagnostics out;
    if (storedSamples_ == 0u) return out;

    double normSum = 0.0;
    for (uint16_t i = 0; i < storedSamples_; ++i) {
        const Vec3 v = storedToVec3(stored_[i]);
        const float norm = v.norm();
        if (!v.isFinite() || !tracker::isFinite(norm)) continue;
        if (!out.valid) {
            out.valid = true;
            out.min = out.max = v;
            out.normMin = out.normMax = norm;
        } else {
            if (v.x < out.min.x) out.min.x = v.x;
            if (v.y < out.min.y) out.min.y = v.y;
            if (v.z < out.min.z) out.min.z = v.z;
            if (v.x > out.max.x) out.max.x = v.x;
            if (v.y > out.max.y) out.max.y = v.y;
            if (v.z > out.max.z) out.max.z = v.z;
            if (norm < out.normMin) out.normMin = norm;
            if (norm > out.normMax) out.normMax = norm;
        }
        normSum += static_cast<double>(norm);
        out.samples++;
    }
    if (out.samples != 0u) {
        out.normMean = static_cast<float>(normSum / static_cast<double>(out.samples));
    }
    return out;
}

uint32_t MagCalibrationCollector::rejected() const { return rejected_; }
uint32_t MagCalibrationCollector::saturated() const { return saturated_; }
uint32_t MagCalibrationCollector::startMs() const { return startMs_; }
uint32_t MagCalibrationCollector::lastSampleMs() const { return lastSampleMs_; }

float MagCalibrationCollector::minX() const { return minX_; }
float MagCalibrationCollector::minY() const { return minY_; }
float MagCalibrationCollector::minZ() const { return minZ_; }
float MagCalibrationCollector::maxX() const { return maxX_; }
float MagCalibrationCollector::maxY() const { return maxY_; }
float MagCalibrationCollector::maxZ() const { return maxZ_; }
float MagCalibrationCollector::spanX() const { return maxX_ - minX_; }
float MagCalibrationCollector::spanY() const { return maxY_ - minY_; }
float MagCalibrationCollector::spanZ() const { return maxZ_ - minZ_; }
float MagCalibrationCollector::normMin() const { return normMin_; }
float MagCalibrationCollector::normMax() const { return normMax_; }

float MagCalibrationCollector::meanX() const { return samples_ > 0 ? static_cast<float>(sumX_ / static_cast<double>(samples_)) : 0.0f; }
float MagCalibrationCollector::meanY() const { return samples_ > 0 ? static_cast<float>(sumY_ / static_cast<double>(samples_)) : 0.0f; }
float MagCalibrationCollector::meanZ() const { return samples_ > 0 ? static_cast<float>(sumZ_ / static_cast<double>(samples_)) : 0.0f; }
float MagCalibrationCollector::normMean() const { return samples_ > 0 ? static_cast<float>(normSum_ / static_cast<double>(samples_)) : 0.0f; }

const MagCalibrationResult& MagCalibrationCollector::lastResult() const { return lastResult_; }

#undef TRACKER_MAG_FIT_NOINLINE

} // namespace tracker
