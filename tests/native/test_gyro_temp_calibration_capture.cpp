#include "test_common.hpp"

#include "runtime/gyro_temp_calibration_capture.hpp"

using namespace tracker;

namespace {

ImuQualityResult goodQuality() {
    ImuQualityResult q;
    q.shouldUpdateAhrs = true;
    q.shouldUseAccelCorrection = true;
    q.shouldRequestFifoRecovery = false;
    q.accelNormValid = true;
    q.accelNormG = 1.0f;
    q.accelConfidence = 1.0f;
    return q;
}

Lsm6dsv::Sample sample(float tempC, const Vec3& gyroDps, float accelNormG = 1.0f) {
    Lsm6dsv::Sample s;
    s.temp_c = tempC;
    s.accel_g = Vec3(0.0f, 0.0f, accelNormG);
    s.gyro_rad_s = gyroDps * MATH_DEG_TO_RAD;
    return s;
}

void feedStable(GyroTempCalibrationCapture& capture,
                ImuQualityResult q,
                uint32_t& nowMs,
                uint32_t count,
                float tempC = 25.2f) {
    for (uint32_t i = 0; i < count; ++i) {
        const float wobbleDps = (i & 1u) ? 0.015f : -0.015f;
        capture.updateSample(
            sample(tempC, Vec3(0.05f + wobbleDps, -0.03f, 0.02f)),
            q,
            nowMs++
        );
    }
}

void testStableCaptureAndShortFinalWindow(TestContext& ctx) {
    GyroTempCalibrationCapture capture;
    capture.start(1000, 100000);
    ImuQualityResult q = goodQuality();
    uint32_t nowMs = 1000;

    feedStable(capture, q, nowMs, 640);
    CHECK(ctx, capture.active());
    CHECK(ctx, capture.capture().samples == 640);
    CHECK(ctx, capture.capture().gyroAfterRadS.count == 640);
    CHECK(ctx, capture.diagnostics().acceptedSamples == 512);
    CHECK(ctx, capture.diagnostics().currentWindowSamples == 128);

    capture.stop(nowMs);
    CHECK(ctx, !capture.active());
    CHECK(ctx, capture.completed());
    CHECK(ctx, capture.diagnostics().acceptedSamples == 640);
    CHECK(ctx, capture.diagnostics().acceptedWindows == 3);
    CHECK(ctx, capture.usableTempBins() == 1);
}

void testMovementOnlyDropsCurrentWindow(TestContext& ctx) {
    GyroTempCalibrationCapture capture;
    capture.start(0, 100000);
    ImuQualityResult q = goodQuality();
    uint32_t nowMs = 0;

    feedStable(capture, q, nowMs, 512);
    CHECK(ctx, capture.diagnostics().acceptedSamples == 512);

    feedStable(capture, q, nowMs, 200);
    capture.updateSample(sample(25.2f, Vec3(4.0f, 0.0f, 0.0f)), q, nowMs++);
    CHECK(ctx, capture.diagnostics().acceptedSamples == 512);
    CHECK(ctx, capture.diagnostics().motionWindowResets == 1);
    CHECK(ctx, capture.diagnostics().rejectedSamples == 201);

    // The user does not restart the capture. Returning the tracker to an
    // ordinary stable surface is enough, and the already accepted windows
    // remain available.
    feedStable(capture, q, nowMs, 700);
    capture.stop(nowMs);

    CHECK(ctx, capture.diagnostics().acceptedSamples == 1212);
    CHECK(ctx, capture.diagnostics().acceptedWindows == 5);
    CHECK(ctx, capture.diagnostics().motionRejectedSamples == 201);
    CHECK(ctx, capture.usableTempBins() == 1);
}

void testSlowRotationAndVibrationAreRejected(TestContext& ctx) {
    GyroTempCalibrationCapture capture;
    capture.start(0, 100000);
    ImuQualityResult q = goodQuality();
    uint32_t nowMs = 0;

    // Slow yaw rotation passes the instantaneous 1.5 dps gate but must fail
    // the window mean gate so it cannot become a false temperature bias.
    for (uint32_t i = 0; i < 256; ++i) {
        capture.updateSample(sample(26.2f, Vec3(0.55f, 0.0f, 0.0f)), q, nowMs++);
    }
    CHECK(ctx, capture.diagnostics().gyroRejectedWindows == 1);
    CHECK(ctx, capture.diagnostics().acceptedSamples == 0);

    // Zero-mean vibration also passes the instantaneous gate, but its window
    // standard deviation is too high.
    for (uint32_t i = 0; i < 256; ++i) {
        const float x = (i & 1u) ? 0.45f : -0.45f;
        capture.updateSample(sample(26.2f, Vec3(x, 0.0f, 0.0f)), q, nowMs++);
    }
    CHECK(ctx, capture.diagnostics().gyroRejectedWindows == 2);

    feedStable(capture, q, nowMs, 700, 26.2f);
    capture.stop(nowMs);
    CHECK(ctx, capture.diagnostics().acceptedSamples == 700);
    CHECK(ctx, capture.usableTempBins() == 1);
}

void testAccelAndTemperatureWindowGates(TestContext& ctx) {
    GyroTempCalibrationCapture capture;
    capture.start(0, 100000);
    ImuQualityResult q = goodQuality();
    uint32_t nowMs = 0;

    // Alternating accel norm is a vibration/instability case even though the
    // mean remains 1 g and every individual sample is inside the loose gate.
    for (uint32_t i = 0; i < 256; ++i) {
        q.accelNormG = (i & 1u) ? 1.04f : 0.96f;
        capture.updateSample(sample(29.2f, Vec3(0.04f, 0.0f, 0.0f), q.accelNormG), q, nowMs++);
    }
    CHECK(ctx, capture.diagnostics().accelRejectedWindows == 1);

    q.accelNormG = 1.0f;
    // A rapid temperature jump inside one candidate window is also rejected;
    // normal warm-up is far slower than this threshold.
    for (uint32_t i = 0; i < 256; ++i) {
        const float tempC = 29.10f + 0.30f * static_cast<float>(i) / 255.0f;
        capture.updateSample(sample(tempC, Vec3(0.04f, 0.0f, 0.0f)), q, nowMs++);
    }
    CHECK(ctx, capture.diagnostics().temperatureRejectedWindows == 1);

    feedStable(capture, q, nowMs, 700, 29.2f);
    capture.stop(nowMs);
    CHECK(ctx, capture.diagnostics().acceptedSamples == 700);
}

void testQualityFaultPausesWithoutEndingCapture(TestContext& ctx) {
    GyroTempCalibrationCapture capture;
    capture.start(0, 100000);
    ImuQualityResult q = goodQuality();
    uint32_t nowMs = 0;

    feedStable(capture, q, nowMs, 180, 27.2f);
    ImuQualityResult bad = q;
    bad.flags |= imu_quality_flags::TIMESTAMP_LARGE_GAP;
    bad.shouldUpdateAhrs = false;
    capture.updateSample(sample(27.2f, Vec3::zero()), bad, nowMs++);

    CHECK(ctx, capture.active());
    CHECK(ctx, capture.diagnostics().qualityRejectedSamples == 1);
    CHECK(ctx, capture.diagnostics().rejectedSamples == 181);

    feedStable(capture, q, nowMs, 700, 27.2f);
    capture.stop(nowMs);
    CHECK(ctx, capture.diagnostics().acceptedSamples == 700);
    CHECK(ctx, capture.usableTempBins() == 1);
}

void testDurationAutoStopFlushesValidWindow(TestContext& ctx) {
    GyroTempCalibrationCapture capture;
    capture.start(1000, 150);
    ImuQualityResult q = goodQuality();

    for (uint32_t i = 0; i <= 150; ++i) {
        capture.updateSample(
            sample(28.2f, Vec3(0.04f, -0.02f, 0.01f)),
            q,
            1000 + i
        );
    }

    CHECK(ctx, capture.completed());
    CHECK(ctx, capture.diagnostics().acceptedSamples == 151);
    CHECK(ctx, capture.diagnostics().acceptedWindows == 1);
}

} // namespace

int main() {
    TestContext ctx;

    testStableCaptureAndShortFinalWindow(ctx);
    testMovementOnlyDropsCurrentWindow(ctx);
    testSlowRotationAndVibrationAreRejected(ctx);
    testAccelAndTemperatureWindowGates(ctx);
    testQualityFaultPausesWithoutEndingCapture(ctx);
    testDurationAutoStopFlushesValidWindow(ctx);

    return ctx.finish("test_gyro_temp_calibration_capture");
}
