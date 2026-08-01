#include "test_common.hpp"

#include "sensor/frame_transform.hpp"
#include "sensor/mag_runtime.hpp"

using namespace tracker;

static Mat3 quarterTurnZ() {
    return Mat3(
         0.0f, -1.0f, 0.0f,
         1.0f,  0.0f, 0.0f,
         0.0f,  0.0f, 1.0f);
}

int main() {
    TestContext t;

    SensorToDeviceFrameCache cache;
    const Mat3 rotation = quarterTurnZ();
    const SensorToDeviceFrame& first = cache.resolve(1u, true, rotation);
    CHECK(t, first.enabled);
    CHECK(t, cache.refreshes() == 1u);
    CHECK_NEAR(t, first.apply(Vec3::unitX()).y, 1.0f, 1.0e-6f);

    const SensorToDeviceFrame& repeated = cache.resolve(1u, true, rotation);
    CHECK(t, repeated.enabled);
    CHECK(t, cache.refreshes() == 1u);

    const SensorToDeviceFrame& rejected =
        cache.resolve(2u, true, Mat3::diagonal(2.0f, 1.0f, 1.0f));
    CHECK(t, !rejected.enabled);
    CHECK(t, cache.refreshes() == 2u);

    MagRuntimeConfig cfg;
    cfg.enabled = true;
    cfg.calibrationValid = true;
    cfg.axisAlignmentValid = true;
    cfg.sensorToDevicePrevalidated = true;
    cfg.sensorToDeviceFrame = SensorToDeviceFrame{true, rotation};
    cfg.minTrustNorm = 0.1f;
    cfg.maxTrustNorm = 10.0f;

    Lsm6dsvFifoReader::MagRawSample raw;
    raw.x = 1;
    raw.t_us = 1000u;
    MagProcessedSample out;
    MagRuntimeProcessor processor;
    CHECK(t, processor.process(raw, cfg, 1u, out));
    CHECK(t, out.sensorToDeviceApplied);
    CHECK_NEAR(t, out.body.y, 1.0f, 1.0e-6f);

    cfg.sensorToDevicePrevalidated = false;
    cfg.sensorToDeviceValid = true;
    cfg.sensorToDevice = Mat3::diagonal(2.0f, 1.0f, 1.0f);
    CHECK(t, processor.process(raw, cfg, 2u, out));
    CHECK(t, !out.sensorToDeviceApplied);

    return t.finish("pre_0024ab_frame_cache");
}
