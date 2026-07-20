#include <cmath>
#include <cstdio>

#include "config/tracker_config.hpp"
#include "sensor/frame_transform.hpp"
#include "sensor/sensor_to_device_alignment.hpp"
#include "test_common.hpp"

using namespace tracker;

namespace {

bool nearFloat(float a, float b, float eps) {
    return std::isfinite(a) && std::isfinite(b) && std::fabs(a - b) <= eps;
}

bool nearVec(const Vec3& a, const Vec3& b, float eps) {
    return nearFloat(a.x, b.x, eps) && nearFloat(a.y, b.y, eps) && nearFloat(a.z, b.z, eps);
}

bool nearMat(const Mat3& a, const Mat3& b, float eps) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (!nearFloat(a.m[r][c], b.m[r][c], eps)) return false;
        }
    }
    return true;
}

Mat3 permutation(uint8_t a0, float s0, uint8_t a1, float s1, uint8_t a2, float s2) {
    Mat3 m = Mat3::zero();
    m.m[0][a0] = s0;
    m.m[1][a1] = s1;
    m.m[2][a2] = s2;
    return m;
}

Lsm6dsv::Sample sample(const Vec3& accel, const Vec3& gyro = Vec3::zero()) {
    Lsm6dsv::Sample s;
    s.accel_g = accel;
    s.gyro_rad_s = gyro;
    return s;
}

void testAllRightHandedDiscreteMappings(TestContext& ctx) {
    const uint8_t perms[6][3] = {
        {0, 1, 2}, {0, 2, 1}, {1, 0, 2},
        {1, 2, 0}, {2, 0, 1}, {2, 1, 0},
    };
    uint32_t tested = 0;
    for (const auto& p : perms) {
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                for (int sz = -1; sz <= 1; sz += 2) {
                    const Mat3 expected = permutation(
                        p[0], static_cast<float>(sx),
                        p[1], static_cast<float>(sy),
                        p[2], static_cast<float>(sz)
                    );
                    if (expected.determinant() < 0.5f) continue;

                    const Vec3 topSensor = expected.transposed() * Vec3::unitZ();
                    const Vec3 forwardSensor = expected.transposed() * Vec3::unitY();
                    const auto result = solveSensorToDeviceAlignment(topSensor, forwardSensor);
                    CHECK(ctx, result.valid);
                    CHECK(ctx, nearMat(result.sensorToDevice, expected, 1.0e-5f));
                    CHECK(ctx, nearVec(result.sensorToDevice * topSensor, Vec3::unitZ(), 1.0e-5f));
                    CHECK(ctx, nearVec(result.sensorToDevice * forwardSensor, Vec3::unitY(), 1.0e-5f));
                    tested++;
                }
            }
        }
    }
    CHECK(ctx, tested == 24u);
}

void testContiguousCaptureAndSolve(TestContext& ctx) {
    // A non-discrete mounting proves the solver is not limited to signed axis
    // permutations. Rows are device X/Y/Z expressed in sensor coordinates.
    Vec3 zSensor(0.173648f, 0.0f, 0.984808f);
    Vec3 ySeed(-0.086824f, 0.996195f, 0.015308f);
    zSensor.normalizeInPlace();
    Vec3 ySensor = (ySeed - zSensor * dot(ySeed, zSensor)).normalized();
    Vec3 xSensor = cross(ySensor, zSensor).normalized();
    ySensor = cross(zSensor, xSensor).normalized();
    const Mat3 expected = Mat3::fromRows(xSensor, ySensor, zSensor);

    SensorToDeviceObservationCapture topCapture;
    for (uint32_t i = 0; i < 100; ++i) {
        CHECK(ctx, !topCapture.push(sample(zSensor)));
    }
    for (uint32_t i = 0; i < 10; ++i) {
        (void)topCapture.push(sample(Vec3(0.4f, 0.4f, 0.4f), Vec3(0.0f, 0.0f, 20.0f * MATH_DEG_TO_RAD)));
    }
    for (uint32_t i = 0; i < 256; ++i) {
        const float d = static_cast<float>(static_cast<int>(i % 5) - 2) * 0.0002f;
        (void)topCapture.push(sample(zSensor + Vec3(d, -d, d)));
    }
    const auto topStatus = topCapture.status();
    CHECK(ctx, topStatus.complete);
    CHECK(ctx, topStatus.resetCount >= 1u);
    CHECK(ctx, topStatus.acceptedSamples == 256u);

    SensorToDeviceObservationCapture forwardCapture;
    for (uint32_t i = 0; i < 256; ++i) {
        const float d = static_cast<float>(static_cast<int>(i % 7) - 3) * 0.00015f;
        (void)forwardCapture.push(sample(ySensor + Vec3(-d, d, d)));
    }
    const auto forwardStatus = forwardCapture.status();
    CHECK(ctx, forwardStatus.complete);

    const auto result = solveSensorToDeviceAlignment(topStatus.meanAccelG, forwardStatus.meanAccelG);
    CHECK(ctx, result.valid);
    CHECK(ctx, nearMat(result.sensorToDevice, expected, 0.002f));
    CHECK(ctx, isProperRotationMatrix(result.sensorToDevice));

    const Vec3 arbitrarySensor(0.23f, -0.42f, 0.71f);
    CHECK(ctx, nearVec(result.sensorToDevice * arbitrarySensor, expected * arbitrarySensor, 0.002f));

    TrackerConfig cfg;
    cfg.resetDefaults();
    cfg.data.frame.sensorToDevice = result.sensorToDevice;
    cfg.data.frame.sensorToDeviceValid = true;
    cfg.updateCrc();
    CHECK(ctx, cfg.validate());
    const SensorToDeviceFrame frame = makeSensorToDeviceFrame(
        cfg.data.frame.sensorToDeviceValid,
        cfg.data.frame.sensorToDevice
    );
    CHECK(ctx, frame.enabled);
    CHECK(ctx, nearVec(frame.inverseApply(frame.apply(arbitrarySensor)), arbitrarySensor, 0.002f));
}


void testAccelRotationSeparation(TestContext& ctx) {
    // Gross signed mounting P plus a small physical board tilt Q.
    const Mat3 p = permutation(2, 1.0f, 1, -1.0f, 0, 1.0f);
    const float a = 8.0f * MATH_DEG_TO_RAD;
    const Mat3 q(
        std::cos(a), 0.0f, std::sin(a),
        0.0f, 1.0f, 0.0f,
        -std::sin(a), 0.0f, std::cos(a)
    );
    const Mat3 sensorToDevice = p * q;

    // Symmetric positive sensor calibration and native-sensor bias.
    const Mat3 sensorScale(
        1.018f, 0.012f, -0.004f,
        0.012f, 0.991f, 0.006f,
        -0.004f, 0.006f, 1.007f
    );
    Mat3 sensorScaleInv;
    CHECK(ctx, sensorScale.inverse(sensorScaleInv));
    const Vec3 bias(0.018f, -0.011f, 0.007f);

    // The legacy/full face fit maps into gross sensor labels and therefore
    // contains the small Q rotation in front of the physical calibration.
    const Mat3 combinedScale = q * sensorScale;
    const Vec3 topRaw = bias + sensorScaleInv * (sensorToDevice.transposed() * Vec3::unitZ());
    const Vec3 forwardRaw = bias + sensorScaleInv * (sensorToDevice.transposed() * Vec3::unitY());

    const auto separated = separateSensorToDeviceFromAccelCalibration(
        topRaw,
        forwardRaw,
        bias,
        combinedScale
    );
    CHECK(ctx, separated.valid);
    CHECK(ctx, nearMat(separated.sensorToDevice, sensorToDevice, 0.002f));
    CHECK(ctx, nearMat(separated.accelScaleSensorFrame, sensorScale, 0.002f));
    CHECK(ctx, separated.reconstructionError < 1.0e-4f);

    const Vec3 arbitraryRaw(0.21f, -0.37f, 0.82f);
    const Vec3 oldGrossOutput = p * (combinedScale * (arbitraryRaw - bias));
    const Vec3 newDeviceOutput = separated.sensorToDevice *
        (separated.accelScaleSensorFrame * (arbitraryRaw - bias));
    CHECK(ctx, nearVec(oldGrossOutput, newDeviceOutput, 0.002f));
}

void testBadPositionsRejected(TestContext& ctx) {
    const auto sameSide = solveSensorToDeviceAlignment(Vec3::unitZ(), Vec3(0.02f, 0.0f, 0.9998f));
    CHECK(ctx, !sameSide.valid);

    const auto badNorm = solveSensorToDeviceAlignment(Vec3(0.0f, 0.0f, 0.4f), Vec3::unitY());
    CHECK(ctx, !badNorm.valid);

    SensorToDeviceObservationCapture capture;
    for (uint32_t i = 0; i < 400; ++i) {
        const float alternating = (i & 1u) ? 0.08f : -0.08f;
        (void)capture.push(sample(Vec3(alternating, 0.0f, 1.0f)));
    }
    CHECK(ctx, !capture.complete());
    CHECK(ctx, capture.status().resetCount > 0u);
}

} // namespace

int main() {
    TestContext ctx;
    testAllRightHandedDiscreteMappings(ctx);
    testContiguousCaptureAndSolve(ctx);
    testAccelRotationSeparation(ctx);
    testBadPositionsRejected(ctx);

    if (ctx.failures != 0) {
        std::printf("FAIL %d assertion(s)\n", ctx.failures);
        return 1;
    }
    std::printf("PASS sensor-to-device alignment system\n");
    return 0;
}
