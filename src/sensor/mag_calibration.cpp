#include "sensor/mag_calibration.hpp"

#include <cmath>

namespace tracker {
namespace {

constexpr int kMagFitTerms = 9;

struct FitAccumulator {
    double normal[kMagFitTerms][kMagFitTerms] = {};
    double rhs[kMagFitTerms] = {};
    uint32_t count = 0;
};

struct FitCandidate {
    bool valid = false;
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

void accumulateSample(FitAccumulator& acc, const Vec3& sample) {
    const double x = static_cast<double>(sample.x);
    const double y = static_cast<double>(sample.y);
    const double z = static_cast<double>(sample.z);
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

bool solveLinear9(const double inA[kMagFitTerms][kMagFitTerms],
                  const double inB[kMagFitTerms],
                  double outX[kMagFitTerms]) {
    double a[kMagFitTerms][kMagFitTerms + 1] = {};
    double maxAbs = 0.0;
    for (int r = 0; r < kMagFitTerms; ++r) {
        for (int c = 0; c < kMagFitTerms; ++c) {
            a[r][c] = inA[r][c];
            const double av = std::fabs(a[r][c]);
            if (av > maxAbs) maxAbs = av;
        }
        a[r][kMagFitTerms] = inB[r];
    }

    if (!std::isfinite(maxAbs) || maxAbs <= 0.0) return false;

    const double eps = maxAbs * 1.0e-12;
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

        if (!std::isfinite(pivotAbs) || pivotAbs <= eps) return false;

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

    for (int i = 0; i < kMagFitTerms; ++i) {
        outX[i] = a[i][kMagFitTerms];
        if (!std::isfinite(outX[i])) return false;
    }
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

bool fitFromAccumulator(const FitAccumulator& acc, const MagCalibrationParams& params, FitCandidate& out) {
    out = FitCandidate{};
    if (acc.count < params.minSamples) return false;

    double coeff[kMagFitTerms] = {};
    if (!solveLinear9(acc.normal, acc.rhs, coeff)) return false;

    Mat3 quad(
        static_cast<float>(coeff[0]), static_cast<float>(coeff[3]), static_cast<float>(coeff[4]),
        static_cast<float>(coeff[3]), static_cast<float>(coeff[1]), static_cast<float>(coeff[5]),
        static_cast<float>(coeff[4]), static_cast<float>(coeff[5]), static_cast<float>(coeff[2])
    );
    if (!quad.isFinite()) return false;

    Mat3 quadInv;
    if (!quad.inverse(quadInv, 1.0e-18f)) return false;

    const Vec3 lin(static_cast<float>(coeff[6]), static_cast<float>(coeff[7]), static_cast<float>(coeff[8]));
    const Vec3 center = (quadInv * lin) * -0.5f;
    if (!center.isFinite()) return false;

    const double cx = static_cast<double>(center.x);
    const double cy = static_cast<double>(center.y);
    const double cz = static_cast<double>(center.z);
    const double centerQuad =
        cx * static_cast<double>(quad.m[0][0]) * cx +
        cy * static_cast<double>(quad.m[1][1]) * cy +
        cz * static_cast<double>(quad.m[2][2]) * cz +
        2.0 * cx * static_cast<double>(quad.m[0][1]) * cy +
        2.0 * cx * static_cast<double>(quad.m[0][2]) * cz +
        2.0 * cy * static_cast<double>(quad.m[1][2]) * cz;
    const double k = 1.0 + centerQuad;
    if (!std::isfinite(k) || std::fabs(k) <= 1.0e-18) return false;

    const double shape[3][3] = {
        {static_cast<double>(quad.m[0][0]) / k, static_cast<double>(quad.m[0][1]) / k, static_cast<double>(quad.m[0][2]) / k},
        {static_cast<double>(quad.m[1][0]) / k, static_cast<double>(quad.m[1][1]) / k, static_cast<double>(quad.m[1][2]) / k},
        {static_cast<double>(quad.m[2][0]) / k, static_cast<double>(quad.m[2][1]) / k, static_cast<double>(quad.m[2][2]) / k},
    };

    double eigVec[3][3] = {};
    double eigVal[3] = {};
    if (!jacobiEigenSymmetric3(shape, eigVec, eigVal)) return false;

    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(eigVal[i]) || eigVal[i] <= 0.0) return false;
    }

    const double r0 = 1.0 / std::sqrt(eigVal[0]);
    const double r1 = 1.0 / std::sqrt(eigVal[1]);
    const double r2 = 1.0 / std::sqrt(eigVal[2]);
    double minRadius = r0;
    if (r1 < minRadius) minRadius = r1;
    if (r2 < minRadius) minRadius = r2;
    double maxRadius = r0;
    if (r1 > maxRadius) maxRadius = r1;
    if (r2 > maxRadius) maxRadius = r2;

    if (!std::isfinite(minRadius) || !std::isfinite(maxRadius) || minRadius < static_cast<double>(params.minAxisRadius)) return false;

    const double axisRatio = maxRadius / minRadius;
    if (!std::isfinite(axisRatio) || axisRatio > static_cast<double>(params.maxAxisRatio)) return false;

    const double targetRadius = (r0 + r1 + r2) / 3.0;
    if (!std::isfinite(targetRadius) || targetRadius <= 0.0) return false;

    const double pNp = [&]() {
        double accSum = 0.0;
        for (int i = 0; i < kMagFitTerms; ++i) {
            for (int j = 0; j < kMagFitTerms; ++j) accSum += coeff[i] * acc.normal[i][j] * coeff[j];
        }
        return accSum;
    }();
    double pRhs = 0.0;
    for (int i = 0; i < kMagFitTerms; ++i) pRhs += coeff[i] * acc.rhs[i];
    double residualVar = (pNp - 2.0 * pRhs + static_cast<double>(acc.count)) / static_cast<double>(acc.count);
    if (residualVar < 0.0 && residualVar > -1.0e-9) residualVar = 0.0;
    if (!std::isfinite(residualVar) || residualVar < 0.0) return false;
    const double residualRms = std::sqrt(residualVar);
    if (!std::isfinite(residualRms)) return false;

    const Mat3 softIron = makeSoftIronFromEigen(eigVec, eigVal, targetRadius);
    if (!softIron.isFinite()) return false;

    out.valid = true;
    out.hardIron = center;
    out.softIron = softIron;
    out.expectedNorm = static_cast<float>(targetRadius);
    out.radiusX = static_cast<float>(r0);
    out.radiusY = static_cast<float>(r1);
    out.radiusZ = static_cast<float>(r2);
    out.axisRatio = static_cast<float>(axisRatio);
    out.algebraicResidualRms = static_cast<float>(residualRms);
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
    return bySigma > byFloor ? bySigma : byFloor;
}

bool accumulateInliers(const MagCalibrationStoredSample* samples,
                       uint16_t count,
                       const FitCandidate& reference,
                       float threshold,
                       FitAccumulator& out) {
    resetAccumulator(out);
    for (uint16_t i = 0; i < count; ++i) {
        if (residualAbs(reference, samples[i]) <= threshold) {
            accumulateSample(out, storedToVec3(samples[i]));
        }
    }
    return out.count > 0;
}


bool accumulateRawNormFiltered(const MagCalibrationStoredSample* samples,
                               uint16_t count,
                               FitAccumulator& out) {
    resetAccumulator(out);
    if (!samples || count == 0) return false;

    double sum = 0.0;
    double sumSq = 0.0;
    for (uint16_t i = 0; i < count; ++i) {
        const double n = static_cast<double>(storedToVec3(samples[i]).norm());
        sum += n;
        sumSq += n * n;
    }
    const double mean = sum / static_cast<double>(count);
    double var = sumSq / static_cast<double>(count) - mean * mean;
    if (var < 0.0 && var > -1.0e-6) var = 0.0;
    const double stddev = var > 0.0 ? std::sqrt(var) : 0.0;
    const double lo = mean - 3.5 * stddev;
    const double hi = mean + 3.5 * stddev;

    for (uint16_t i = 0; i < count; ++i) {
        const Vec3 v = storedToVec3(samples[i]);
        const double n = static_cast<double>(v.norm());
        if (std::isfinite(n) && n >= lo && n <= hi) {
            accumulateSample(out, v);
        }
    }
    return out.count > 0;
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
    for (auto& s : stored_) s = MagCalibrationStoredSample{};
    lastResult_ = MagCalibrationResult{};
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
    if (storedSamples_ < kMaxStoredSamples) {
        stored_[storedSamples_++] = stored;
    } else {
        // Deterministic bounded reservoir. Calibration does not need every raw
        // sample; it needs a representative all-orientation subset without heap.
        const uint16_t idx = static_cast<uint16_t>((storedSequence_ * 2654435761UL) % kMaxStoredSamples);
        stored_[idx] = stored;
    }
    storedSequence_++;

    sumX_ += x;
    sumY_ += y;
    sumZ_ += z;
    normSum_ += static_cast<double>(normRaw);
    samples_++;
    lastSampleMs_ = nowMs;
}

bool MagCalibrationCollector::compute(MagCalibrationResult& out) {
    out = MagCalibrationResult{};

    const uint32_t minSamples = params_.minSamples;
    if (!hasData_ || samples_ < minSamples || storedSamples_ < minSamples) {
        lastResult_ = out;
        return false;
    }

    const float spanX = maxX_ - minX_;
    const float spanY = maxY_ - minY_;
    const float spanZ = maxZ_ - minZ_;

    const float boxRadiusX = 0.5f * spanX;
    const float boxRadiusY = 0.5f * spanY;
    const float boxRadiusZ = 0.5f * spanZ;

    if (boxRadiusX < params_.minAxisRadius ||
        boxRadiusY < params_.minAxisRadius ||
        boxRadiusZ < params_.minAxisRadius) {
        lastResult_ = out;
        return false;
    }

    float minBoxRadius = boxRadiusX;
    if (boxRadiusY < minBoxRadius) minBoxRadius = boxRadiusY;
    if (boxRadiusZ < minBoxRadius) minBoxRadius = boxRadiusZ;
    float maxBoxRadius = boxRadiusX;
    if (boxRadiusY > maxBoxRadius) maxBoxRadius = boxRadiusY;
    if (boxRadiusZ > maxBoxRadius) maxBoxRadius = boxRadiusZ;
    const float boxCoverageScore = maxBoxRadius > 0.0f ? minBoxRadius / maxBoxRadius : 0.0f;
    if (!tracker::isFinite(boxCoverageScore) || boxCoverageScore < params_.minCoverageScore) {
        lastResult_ = out;
        return false;
    }

    FitAccumulator all;
    if (!accumulateRawNormFiltered(stored_, storedSamples_, all) || all.count < minSamples) {
        lastResult_ = out;
        return false;
    }

    FitCandidate initial;
    if (!fitFromAccumulator(all, params_, initial)) {
        lastResult_ = out;
        return false;
    }

    const GeometricMetrics initialAll = computeGeometricMetrics(initial, stored_, storedSamples_, 999999.0f);
    const float initialThreshold = robustThreshold(params_, initial, initialAll);

    FitCandidate finalFit = initial;
    FitAccumulator inlierAcc;
    if (accumulateInliers(stored_, storedSamples_, initial, initialThreshold, inlierAcc) &&
        inlierAcc.count >= minSamples &&
        static_cast<float>(inlierAcc.count) / static_cast<float>(storedSamples_) >= params_.minInlierRatio &&
        inlierAcc.count < all.count) {
        FitCandidate refit;
        if (fitFromAccumulator(inlierAcc, params_, refit)) finalFit = refit;
    }

    const GeometricMetrics finalAll = computeGeometricMetrics(finalFit, stored_, storedSamples_, 999999.0f);
    const float finalThreshold = robustThreshold(params_, finalFit, finalAll);
    const GeometricMetrics finalMetrics = computeGeometricMetrics(finalFit, stored_, storedSamples_, finalThreshold);

    if (finalMetrics.inliers < minSamples || finalMetrics.inlierRatio < params_.minInlierRatio) {
        lastResult_ = out;
        return false;
    }
    if (!tracker::isFinite(finalMetrics.normalizedRms) ||
        finalMetrics.normalizedRms > params_.maxGeometricResidualRmsFactor) {
        lastResult_ = out;
        return false;
    }
    if (!tracker::isFinite(finalMetrics.directionalCoverageScore) ||
        finalMetrics.directionalCoverageScore < params_.minDirectionalCoverageScore) {
        lastResult_ = out;
        return false;
    }
    if (!tracker::isFinite(finalFit.algebraicResidualRms) ||
        finalFit.algebraicResidualRms > params_.maxAlgebraicResidualRms) {
        lastResult_ = out;
        return false;
    }

    out.valid = true;
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

    lastResult_ = out;
    return true;
}

MagCalibrationResult MagCalibrationCollector::compute() {
    MagCalibrationResult out;
    compute(out);
    return out;
}

bool MagCalibrationCollector::active() const { return active_; }
bool MagCalibrationCollector::hasData() const { return hasData_; }
uint32_t MagCalibrationCollector::samples() const { return samples_; }
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

} // namespace tracker
