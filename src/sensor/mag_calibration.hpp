#pragma once

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
    explicit MagCalibrationCollector(const MagCalibrationParams& params = MagCalibrationParams{});

    void setParams(const MagCalibrationParams& params);
    const MagCalibrationParams& params() const;

    void reset();
    void start(uint32_t nowMs);
    void stop();
    void push(const Lsm6dsvFifoReader::MagRawSample& m, float normRaw, uint32_t nowMs);

    bool compute(MagCalibrationResult& out);
    MagCalibrationResult compute();

    bool active() const;
    bool hasData() const;
    uint32_t samples() const;
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
