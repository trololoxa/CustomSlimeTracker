#pragma once

#include <cmath>
#include <cstdint>

#include "core/math.hpp"
#include "connection/lsm6dsv_fifo.hpp"

namespace tracker {

struct MagCalibrationParams {
    uint32_t minSamples = 300;
    float minAxisRadius = 20.0f;
    float trustNormMinFactor = 0.65f;
    float trustNormMaxFactor = 1.35f;
};

struct MagCalibrationResult {
    bool valid = false;
    Vec3 hardIron = Vec3::zero();
    Mat3 softIron = Mat3::identity();
    float expectedNorm = 1.0f;
    float minTrustNorm = 0.65f;
    float maxTrustNorm = 1.35f;
    float radiusX = 0.0f;
    float radiusY = 0.0f;
    float radiusZ = 0.0f;
};

class MagCalibrationCollector {
public:
    explicit MagCalibrationCollector(const MagCalibrationParams& params = MagCalibrationParams{})
        : params_(params) {}

    void setParams(const MagCalibrationParams& params) {
        params_ = params;
    }

    const MagCalibrationParams& params() const { return params_; }

    void reset() {
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
        lastResult_ = MagCalibrationResult{};
    }

    void start(uint32_t nowMs) {
        reset();
        active_ = true;
        startMs_ = nowMs;
    }

    void stop() {
        active_ = false;
    }

    void push(const Lsm6dsvFifoReader::MagRawSample& m, float normRaw, uint32_t nowMs) {
        if (!active_) return;

        if ((m.flags & Lsm6dsvFifoReader::MAG_FLAG_RAW_SATURATED) != 0) {
            saturated_++;
            rejected_++;
            return;
        }

        const float x = static_cast<float>(m.x);
        const float y = static_cast<float>(m.y);
        const float z = static_cast<float>(m.z);

        if (!tracker::isFinite(x) || !tracker::isFinite(y) || !tracker::isFinite(z) || !tracker::isFinite(normRaw)) {
            rejected_++;
            return;
        }

        if (!hasData_) {
            minX_ = maxX_ = x;
            minY_ = maxY_ = y;
            minZ_ = maxZ_ = z;
            normMin_ = normMax_ = normRaw;
            hasData_ = true;
        } else {
            if (x < minX_) minX_ = x;
            if (x > maxX_) maxX_ = x;
            if (y < minY_) minY_ = y;
            if (y > maxY_) maxY_ = y;
            if (z < minZ_) minZ_ = z;
            if (z > maxZ_) maxZ_ = z;
            if (normRaw < normMin_) normMin_ = normRaw;
            if (normRaw > normMax_) normMax_ = normRaw;
        }

        sumX_ += static_cast<double>(x);
        sumY_ += static_cast<double>(y);
        sumZ_ += static_cast<double>(z);
        normSum_ += static_cast<double>(normRaw);
        samples_++;
        lastSampleMs_ = nowMs;
    }

    bool compute(MagCalibrationResult& out) {
        out = MagCalibrationResult{};

        if (!hasData_ || samples_ < params_.minSamples) {
            lastResult_ = out;
            return false;
        }

        const float spanX = maxX_ - minX_;
        const float spanY = maxY_ - minY_;
        const float spanZ = maxZ_ - minZ_;

        const float radiusX = 0.5f * spanX;
        const float radiusY = 0.5f * spanY;
        const float radiusZ = 0.5f * spanZ;

        if (radiusX < params_.minAxisRadius ||
            radiusY < params_.minAxisRadius ||
            radiusZ < params_.minAxisRadius) {
            lastResult_ = out;
            return false;
        }

        const float targetRadius = (radiusX + radiusY + radiusZ) / 3.0f;
        if (!tracker::isFinite(targetRadius) || targetRadius <= 0.0f) {
            lastResult_ = out;
            return false;
        }

        const float sx = targetRadius / radiusX;
        const float sy = targetRadius / radiusY;
        const float sz = targetRadius / radiusZ;

        if (!tracker::isFinite(sx) || !tracker::isFinite(sy) || !tracker::isFinite(sz)) {
            lastResult_ = out;
            return false;
        }

        out.valid = true;
        out.hardIron = Vec3(
            0.5f * (maxX_ + minX_),
            0.5f * (maxY_ + minY_),
            0.5f * (maxZ_ + minZ_)
        );
        out.softIron = Mat3::diagonal(sx, sy, sz);
        out.expectedNorm = targetRadius;
        out.minTrustNorm = targetRadius * params_.trustNormMinFactor;
        out.maxTrustNorm = targetRadius * params_.trustNormMaxFactor;
        out.radiusX = radiusX;
        out.radiusY = radiusY;
        out.radiusZ = radiusZ;

        lastResult_ = out;
        return true;
    }

    MagCalibrationResult compute() {
        MagCalibrationResult out;
        compute(out);
        return out;
    }

    bool active() const { return active_; }
    bool hasData() const { return hasData_; }
    uint32_t samples() const { return samples_; }
    uint32_t rejected() const { return rejected_; }
    uint32_t saturated() const { return saturated_; }
    uint32_t startMs() const { return startMs_; }
    uint32_t lastSampleMs() const { return lastSampleMs_; }

    float minX() const { return minX_; }
    float minY() const { return minY_; }
    float minZ() const { return minZ_; }
    float maxX() const { return maxX_; }
    float maxY() const { return maxY_; }
    float maxZ() const { return maxZ_; }
    float spanX() const { return maxX_ - minX_; }
    float spanY() const { return maxY_ - minY_; }
    float spanZ() const { return maxZ_ - minZ_; }
    float normMin() const { return normMin_; }
    float normMax() const { return normMax_; }

    float meanX() const { return samples_ > 0 ? static_cast<float>(sumX_ / static_cast<double>(samples_)) : 0.0f; }
    float meanY() const { return samples_ > 0 ? static_cast<float>(sumY_ / static_cast<double>(samples_)) : 0.0f; }
    float meanZ() const { return samples_ > 0 ? static_cast<float>(sumZ_ / static_cast<double>(samples_)) : 0.0f; }
    float normMean() const { return samples_ > 0 ? static_cast<float>(normSum_ / static_cast<double>(samples_)) : 0.0f; }

    const MagCalibrationResult& lastResult() const { return lastResult_; }

private:
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

    MagCalibrationResult lastResult_;
};

} // namespace tracker