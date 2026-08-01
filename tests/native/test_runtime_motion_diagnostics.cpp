#include "runtime/runtime_motion_diagnostics.hpp"
#include "test_common.hpp"

using namespace tracker;

int main() {
    TestContext t;
    RuntimeMotionDiagnostics diagnostics;
    diagnostics.begin(0u);
    diagnostics.setEnabled(true, 0u);

    Lsm6dsv::RawSample raw;
    Lsm6dsv::Sample sample;
    sample.gyro_rad_s = Vec3(0.01f, 0.02f, 0.03f);
    sample.accel_g = Vec3(0.0f, 0.0f, 1.0f);
    sample.temp_c = 31.5f;

    ImuQualityResult quality;
    quality.flags = imu_quality_flags::TIMESTAMP_HARDWARE;
    quality.dtUs = 1042u;
    quality.accelNormValid = true;
    quality.accelNormG = 1.0f;
    quality.overallConfidence = 0.9f;

    constexpr uint32_t kSamples = TRACKER_MOTION_DIAGNOSTICS_SAMPLE_DIVISOR * 3u + 1u;
    for (uint32_t i = 0u; i < kSamples; ++i) {
        raw.t_us = 1000u + static_cast<uint64_t>(i) * 1042u;
        if (i == 7u) {
            quality.flags |= imu_quality_flags::ACCEL_NORM_OUTLIER;
            quality.estimatedDroppedBefore = 2u;
        } else {
            quality.flags &= ~imu_quality_flags::ACCEL_NORM_OUTLIER;
            quality.estimatedDroppedBefore = 0u;
        }
        diagnostics.recordSample(raw, sample, quality);
    }

    const auto& stats = diagnostics.stats();
    CHECK(t, stats.samples == kSamples);
    CHECK(t, stats.metricSamples == 4u);
    CHECK(t, stats.sampleDivisor == TRACKER_MOTION_DIAGNOSTICS_SAMPLE_DIVISOR);
    CHECK(t, stats.hwTimestampSamples == kSamples);
    CHECK(t, stats.accelNormOutliers == 1u);
    CHECK(t, stats.estimatedDroppedSamples == 2u);
    CHECK(t, stats.dtMinUs == 1042u);
    CHECK(t, stats.dtMaxUs == 1042u);
    CHECK(t, stats.lastTimestampUs == raw.t_us);

    diagnostics.setEnabled(false, 10u);
    diagnostics.recordSample(raw, sample, quality);
    CHECK(t, diagnostics.stats().samples == 0u);

    return t.finish("runtime_motion_diagnostics");
}
