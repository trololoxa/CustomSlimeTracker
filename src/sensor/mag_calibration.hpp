#pragma once

#include <cstdint>

#include "core/math.hpp"
#include "connection/lsm6dsv_fifo.hpp"

namespace tracker {

enum class MagCalibrationFailureReason : uint8_t {
    None = 0,
    InsufficientSamples,
    AxisRadiusTooSmall,
    BoxCoverageTooLow,
    RawNormFilterFailed,
    FitNormalizationFailed,
    LinearSolveFailed,
    QuadraticCenterFailed,
    NonPositiveDefiniteShape,
    EllipsoidFitFailed,
    InlierRatioTooLow,
    GeometricResidualTooHigh,
    DirectionalCoverageTooLow,
    AlgebraicResidualTooHigh,
};

enum class MagCalibrationSolverStage : uint8_t {
    None = 0,
    Normalized,
    LinearSolved,
    CenterSolved,
    ShapeSolved,
    EigenSolved,
    CandidateBuilt,
};

const char* magCalibrationFailureReasonName(MagCalibrationFailureReason reason);
const char* magCalibrationSolverStageName(MagCalibrationSolverStage stage);

struct MagCalibrationParams {
    uint32_t minSamples = 300;
    float minAxisRadius = 20.0f;
    float minCoverageScore = 0.35f;
    float minDirectionalCoverageScore = 0.65f;
    float maxAxisRatio = 6.0f;
    // Centered, dimensionless ellipsoid-equation RMS.
    float maxAlgebraicResidualRms = 0.12f;
    float maxGeometricResidualRmsFactor = 0.10f;
    float outlierSigma = 3.0f;
    float outlierMinResidualFactor = 0.08f;
    float minInlierRatio = 0.82f;
    float trustNormMinFactor = 0.65f;
    float trustNormMaxFactor = 1.35f;
};

inline float magCalibrationEffectiveMinBoxCoverage(const MagCalibrationParams& params) {
    // Raw axis spans combine motion coverage with the very soft-iron
    // anisotropy that this calibration is meant to estimate. A fixed 0.35
    // gate contradicts maxAxisRatio=6 and can reject a fully covered valid
    // ellipsoid before the solver has a chance to correct it. Keep only a
    // gross pre-fit observability floor consistent with the permitted shape;
    // fitted directional coverage and geometric residual remain authoritative.
    const float shapeCompatible = params.maxAxisRatio > 1.0f
        ? 0.8f / params.maxAxisRatio
        : 0.8f;
    return std::fmin(params.minCoverageScore, shapeCompatible);
}

struct MagCalibrationResult {
    bool valid = false;
    bool fitAvailable = false;
    MagCalibrationSolverStage solverStage = MagCalibrationSolverStage::None;
    Vec3 fitNormalizationCenter = Vec3::zero();
    Vec3 fitNormalizationScale = Vec3::zero();
    float solverPivotRatio = 0.0f;
    uint32_t solverSamples = 0;
    Vec3 hardIron = Vec3::zero();
    Mat3 softIron = Mat3::identity();
    float expectedNorm = 1.0f;
    float minTrustNorm = 0.65f;
    float maxTrustNorm = 1.35f;
    float radiusX = 0.0f;
    float radiusY = 0.0f;
    float radiusZ = 0.0f;
    float coverageScore = 0.0f;
    float directionalCoverageScore = 0.0f;
    float residualRms = 0.0f;
    float geometricResidualRms = 0.0f;
    float normalizedResidualRms = 0.0f;
    float axisRatio = 0.0f;
    float inlierRatio = 0.0f;
    uint32_t inlierSamples = 0;
};

struct MagCalibrationStoredSample {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

struct MagCalibrationFitSetDiagnostics {
    bool valid = false;
    uint16_t samples = 0;
    Vec3 min = Vec3::zero();
    Vec3 max = Vec3::zero();
    float normMin = 0.0f;
    float normMean = 0.0f;
    float normMax = 0.0f;
};

class MagCalibrationCollector {
public:
    explicit MagCalibrationCollector(const MagCalibrationParams& params = MagCalibrationParams{});

    void setParams(const MagCalibrationParams& params);
    const MagCalibrationParams& params() const;

    void reset();
    void start(uint32_t nowMs);
    void stop();
    void push(const Lsm6dsvFifoReader::MagRawSample& m, float normRaw, uint32_t nowMs);

    bool compute(MagCalibrationResult& out);

    bool active() const;
    bool hasData() const;
    uint32_t samples() const;
    uint16_t storedSamples() const;
    uint32_t reservoirReplacements() const;
    uint32_t reservoirSkipped() const;
    MagCalibrationFitSetDiagnostics fitSetDiagnostics() const;
    uint32_t rejected() const;
    uint32_t saturated() const;
    uint32_t startMs() const;
    uint32_t lastSampleMs() const;

    float minX() const;
    float minY() const;
    float minZ() const;
    float maxX() const;
    float maxY() const;
    float maxZ() const;
    float spanX() const;
    float spanY() const;
    float spanZ() const;
    float normMin() const;
    float normMax() const;

    float meanX() const;
    float meanY() const;
    float meanZ() const;
    float normMean() const;

    const MagCalibrationResult& lastResult() const;
    MagCalibrationFailureReason lastFailureReason() const;
    const char* lastFailureReasonName() const;

private:
    static constexpr uint16_t kMaxStoredSamples = 768;

    MagCalibrationParams params_;

    bool active_ = false;
    bool hasData_ = false;
    uint32_t samples_ = 0;
    uint32_t rejected_ = 0;
    uint32_t saturated_ = 0;
    uint32_t startMs_ = 0;
    uint32_t lastSampleMs_ = 0;

    float minX_ = 0.0f;
    float minY_ = 0.0f;
    float minZ_ = 0.0f;
    float maxX_ = 0.0f;
    float maxY_ = 0.0f;
    float maxZ_ = 0.0f;

    double sumX_ = 0.0;
    double sumY_ = 0.0;
    double sumZ_ = 0.0;

    float normMin_ = 0.0f;
    float normMax_ = 0.0f;
    double normSum_ = 0.0;

    MagCalibrationStoredSample stored_[kMaxStoredSamples] = {};
    uint16_t storedSamples_ = 0;
    uint32_t storedSequence_ = 0;
    uint32_t reservoirReplacements_ = 0;
    uint32_t reservoirSkipped_ = 0;

    MagCalibrationResult lastResult_;
    MagCalibrationFailureReason lastFailureReason_ = MagCalibrationFailureReason::None;
};

} // namespace tracker
