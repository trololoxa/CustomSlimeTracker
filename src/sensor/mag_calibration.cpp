#include "sensor/mag_calibration.hpp"

#include <cmath>

namespace tracker {
namespace {

constexpr int kMagFitTerms = 9;

void resetNormal(double normal[kMagFitTerms][kMagFitTerms], double rhs[kMagFitTerms]) {
    for (int i = 0; i < kMagFitTerms; ++i) {
        rhs[i] = 0.0;
        for (int j = 0; j < kMagFitTerms; ++j) {
            normal[i][j] = 0.0;
        }
    }
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

    if (!std::isfinite(maxAbs) || maxAbs <= 0.0) {
        return false;
    }

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

        if (!std::isfinite(pivotAbs) || pivotAbs <= eps) {
            return false;
        }

        if (pivot != col) {
            for (int c = col; c <= kMagFitTerms; ++c) {
                const double tmp = a[col][c];
                a[col][c] = a[pivot][c];
                a[pivot][c] = tmp;
            }
        }

        const double invPivot = 1.0 / a[col][col];
        for (int c = col; c <= kMagFitTerms; ++c) {
            a[col][c] *= invPivot;
        }

        for (int row = 0; row < kMagFitTerms; ++row) {
            if (row == col) continue;
            const double f = a[row][col];
            if (f == 0.0) continue;
            for (int c = col; c <= kMagFitTerms; ++c) {
                a[row][c] -= f * a[col][c];
            }
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
        if (maxOff <= (diagScale > 0.0 ? diagScale * 1.0e-12 : 1.0e-12)) {
            break;
        }

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
    for (int k = 0; k < 3; ++k) {
        gain[k] = targetRadius * std::sqrt(eigVal[k]);
    }

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
    resetNormal(ellipsoidNormal_, ellipsoidRhs_);
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
        ellipsoidRhs_[i] += terms[i];
        for (int j = 0; j < kMagFitTerms; ++j) {
            ellipsoidNormal_[i][j] += terms[i] * terms[j];
        }
    }

    sumX_ += x;
    sumY_ += y;
    sumZ_ += z;
    normSum_ += static_cast<double>(normRaw);
    samples_++;
    lastSampleMs_ = nowMs;
}

bool MagCalibrationCollector::compute(MagCalibrationResult& out) {
    out = MagCalibrationResult{};

    if (!hasData_ || samples_ < params_.minSamples) {
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
    const float coverageScore = maxBoxRadius > 0.0f ? minBoxRadius / maxBoxRadius : 0.0f;
    if (!tracker::isFinite(coverageScore) || coverageScore < params_.minCoverageScore) {
        lastResult_ = out;
        return false;
    }

    double coeff[kMagFitTerms] = {};
    if (!solveLinear9(ellipsoidNormal_, ellipsoidRhs_, coeff)) {
        lastResult_ = out;
        return false;
    }

    Mat3 quad(
        static_cast<float>(coeff[0]), static_cast<float>(coeff[3]), static_cast<float>(coeff[4]),
        static_cast<float>(coeff[3]), static_cast<float>(coeff[1]), static_cast<float>(coeff[5]),
        static_cast<float>(coeff[4]), static_cast<float>(coeff[5]), static_cast<float>(coeff[2])
    );
    if (!quad.isFinite()) {
        lastResult_ = out;
        return false;
    }

    Mat3 quadInv;
    if (!quad.inverse(quadInv, 1.0e-18f)) {
        lastResult_ = out;
        return false;
    }

    const Vec3 lin(static_cast<float>(coeff[6]), static_cast<float>(coeff[7]), static_cast<float>(coeff[8]));
    const Vec3 center = (quadInv * lin) * -0.5f;
    if (!center.isFinite()) {
        lastResult_ = out;
        return false;
    }

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
    if (!std::isfinite(k) || std::fabs(k) <= 1.0e-18) {
        lastResult_ = out;
        return false;
    }

    const double shape[3][3] = {
        {static_cast<double>(quad.m[0][0]) / k, static_cast<double>(quad.m[0][1]) / k, static_cast<double>(quad.m[0][2]) / k},
        {static_cast<double>(quad.m[1][0]) / k, static_cast<double>(quad.m[1][1]) / k, static_cast<double>(quad.m[1][2]) / k},
        {static_cast<double>(quad.m[2][0]) / k, static_cast<double>(quad.m[2][1]) / k, static_cast<double>(quad.m[2][2]) / k},
    };

    double eigVec[3][3] = {};
    double eigVal[3] = {};
    if (!jacobiEigenSymmetric3(shape, eigVec, eigVal)) {
        lastResult_ = out;
        return false;
    }

    double minEig = eigVal[0];
    double maxEig = eigVal[0];
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(eigVal[i]) || eigVal[i] <= 0.0) {
            lastResult_ = out;
            return false;
        }
        if (eigVal[i] < minEig) minEig = eigVal[i];
        if (eigVal[i] > maxEig) maxEig = eigVal[i];
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

    if (!std::isfinite(minRadius) || !std::isfinite(maxRadius) || minRadius < static_cast<double>(params_.minAxisRadius)) {
        lastResult_ = out;
        return false;
    }

    const double axisRatio = maxRadius / minRadius;
    if (!std::isfinite(axisRatio) || axisRatio > static_cast<double>(params_.maxAxisRatio)) {
        lastResult_ = out;
        return false;
    }

    const double targetRadius = (r0 + r1 + r2) / 3.0;
    if (!std::isfinite(targetRadius) || targetRadius <= 0.0) {
        lastResult_ = out;
        return false;
    }

    const double pNp = [&]() {
        double acc = 0.0;
        for (int i = 0; i < kMagFitTerms; ++i) {
            for (int j = 0; j < kMagFitTerms; ++j) {
                acc += coeff[i] * ellipsoidNormal_[i][j] * coeff[j];
            }
        }
        return acc;
    }();
    double pRhs = 0.0;
    for (int i = 0; i < kMagFitTerms; ++i) pRhs += coeff[i] * ellipsoidRhs_[i];
    double residualVar = (pNp - 2.0 * pRhs + static_cast<double>(samples_)) / static_cast<double>(samples_);
    if (residualVar < 0.0 && residualVar > -1.0e-9) residualVar = 0.0;
    if (!std::isfinite(residualVar) || residualVar < 0.0) {
        lastResult_ = out;
        return false;
    }
    const double residualRms = std::sqrt(residualVar);
    if (!std::isfinite(residualRms) || residualRms > static_cast<double>(params_.maxAlgebraicResidualRms)) {
        lastResult_ = out;
        return false;
    }

    const Mat3 softIron = makeSoftIronFromEigen(eigVec, eigVal, targetRadius);
    if (!softIron.isFinite()) {
        lastResult_ = out;
        return false;
    }

    out.valid = true;
    out.hardIron = center;
    out.softIron = softIron;
    out.expectedNorm = static_cast<float>(targetRadius);
    out.minTrustNorm = static_cast<float>(targetRadius * static_cast<double>(params_.trustNormMinFactor));
    out.maxTrustNorm = static_cast<float>(targetRadius * static_cast<double>(params_.trustNormMaxFactor));
    out.radiusX = static_cast<float>(r0);
    out.radiusY = static_cast<float>(r1);
    out.radiusZ = static_cast<float>(r2);
    out.coverageScore = coverageScore;
    out.residualRms = static_cast<float>(residualRms);
    out.axisRatio = static_cast<float>(axisRatio);

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
