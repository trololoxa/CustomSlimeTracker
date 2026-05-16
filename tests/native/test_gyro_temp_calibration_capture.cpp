#include "test_common.hpp"

#include "runtime/gyro_temp_calibration_capture.hpp"

using namespace tracker;

int main() {
    TestContext ctx;

    GyroTempCalibrationCapture capture;
    capture.start(1000, 100000);
    CHECK(ctx, capture.active());

    ImuQualityResult q;
    q.shouldUpdateAhrs = true;
    q.shouldRequestFifoRecovery = false;
    q.accelNormValid = true;
    q.accelNormG = 1.0f;
    q.accelConfidence = 1.0f;

    for (uint32_t i = 0; i < 600; ++i) {
        Lsm6dsv::Sample s;
        s.temp_c = 25.2f;
        s.accel_g = Vec3(0.0f, 0.0f, 1.0f);
        s.gyro_rad_s = Vec3(0.001f, -0.002f, 0.0005f);
        capture.updateSample(s, q, 1000 + i);
    }

    CHECK(ctx, capture.active());
    CHECK(ctx, capture.capture().samples == 600);
    CHECK(ctx, capture.capture().gyroAfterRadS.count == 600);
    CHECK(ctx, capture.usableTempBins() == 1);

    capture.stop(2000);
    CHECK(ctx, !capture.active());
    CHECK(ctx, capture.completed());

    return ctx.finish("test_gyro_temp_calibration_capture");
}
