#pragma once

#include <cstddef>
#include <cstdint>
#include <cmath>

#include "core/math.hpp"

namespace tracker {

// ============================================================
// Rolling gyro mean diagnostics
// ============================================================
// Purpose:
//   - Show residual gyro bias over a rolling 1-2 second window.
//   - Compare BEFORE calibration and AFTER calibration gyro means.
//   - Avoid judging calibration from one noisy sample.
//
// Use:
//   RollingGyroMeanWindow<2048> roll;
//   roll.setWindowSamples(1920); // 2 seconds at ~960 Hz
//   roll.push(gyroBeforeRadS, gyroAfterRadS, accelNormG, tempC);
//   auto s = roll.snapshot();
// ============================================================

template <size_t Capacity>
class RollingGyroMeanWindow {
public:
    struct Snapshot {
        size_t count = 0;
        size_t windowSamples = 0;

        Vec3 beforeMeanRadS = Vec3::zero();
        Vec3 afterMeanRadS = Vec3::zero();
        Vec3 beforeMeanDps = Vec3::zero();
        Vec3 afterMeanDps = Vec3::zero();

        float beforeNormDps = 0.0f;
        float afterNormDps = 0.0f;
        float improvementPct = 0.0f;

        float accelNormMeanG = 0.0f;
        float tempMeanC = 0.0f;
    };

    RollingGyroMeanWindow() {
        setWindowSamples(Capacity);
        reset();
    }

    void setWindowSamples(size_t samples) {
        if (samples == 0) {
            samples = 1;
        }
        if (samples > Capacity) {
            samples = Capacity;
        }
        windowSamples_ = samples;
        reset();
    }

    size_t windowSamples() const {
        return windowSamples_;
    }

    void reset() {
        head_ = 0;
        count_ = 0;
        sumBeforeRadS_ = Vec3::zero();
        sumAfterRadS_ = Vec3::zero();
        sumAccelNormG_ = 0.0f;
        sumTempC_ = 0.0f;

        for (size_t i = 0; i < Capacity; ++i) {
            before_[i] = Vec3::zero();
            after_[i] = Vec3::zero();
            accelNorm_[i] = 0.0f;
            tempC_[i] = 0.0f;
        }
    }

    void push(const Vec3& gyroBeforeRadS,
              const Vec3& gyroAfterRadS,
              float accelNormG,
              float tempC) {
        if (count_ < windowSamples_) {
            before_[head_] = gyroBeforeRadS;
            after_[head_] = gyroAfterRadS;
            accelNorm_[head_] = accelNormG;
            tempC_[head_] = tempC;

            sumBeforeRadS_ += gyroBeforeRadS;
            sumAfterRadS_ += gyroAfterRadS;
            sumAccelNormG_ += accelNormG;
            sumTempC_ += tempC;

            count_++;
            head_ = (head_ + 1) % windowSamples_;
            return;
        }

        sumBeforeRadS_ -= before_[head_];
        sumAfterRadS_ -= after_[head_];
        sumAccelNormG_ -= accelNorm_[head_];
        sumTempC_ -= tempC_[head_];

        before_[head_] = gyroBeforeRadS;
        after_[head_] = gyroAfterRadS;
        accelNorm_[head_] = accelNormG;
        tempC_[head_] = tempC;

        sumBeforeRadS_ += gyroBeforeRadS;
        sumAfterRadS_ += gyroAfterRadS;
        sumAccelNormG_ += accelNormG;
        sumTempC_ += tempC;

        head_ = (head_ + 1) % windowSamples_;
    }

    bool full() const {
        return count_ >= windowSamples_;
    }

    size_t count() const {
        return count_;
    }

    Snapshot snapshot() const {
        Snapshot s;
        s.count = count_;
        s.windowSamples = windowSamples_;

        if (count_ == 0) {
            return s;
        }

        const float inv = 1.0f / static_cast<float>(count_);
        s.beforeMeanRadS = sumBeforeRadS_ * inv;
        s.afterMeanRadS = sumAfterRadS_ * inv;
        s.beforeMeanDps = s.beforeMeanRadS * MATH_RAD_TO_DEG;
        s.afterMeanDps = s.afterMeanRadS * MATH_RAD_TO_DEG;
        s.beforeNormDps = s.beforeMeanDps.norm();
        s.afterNormDps = s.afterMeanDps.norm();

        s.improvementPct = s.beforeNormDps > 1.0e-6f
            ? 100.0f * (s.beforeNormDps - s.afterNormDps) / s.beforeNormDps
            : 0.0f;

        s.accelNormMeanG = sumAccelNormG_ * inv;
        s.tempMeanC = sumTempC_ * inv;
        return s;
    }

private:
    size_t windowSamples_ = Capacity;
    size_t head_ = 0;
    size_t count_ = 0;

    Vec3 before_[Capacity];
    Vec3 after_[Capacity];
    float accelNorm_[Capacity];
    float tempC_[Capacity];

    Vec3 sumBeforeRadS_ = Vec3::zero();
    Vec3 sumAfterRadS_ = Vec3::zero();
    float sumAccelNormG_ = 0.0f;
    float sumTempC_ = 0.0f;
};

} // namespace tracker
