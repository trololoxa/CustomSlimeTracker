#include "test_common.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include "connection/lsm6dsv_fifo.hpp"
#include "core/math.hpp"
#include "sensor/mag_calibration.hpp"

using namespace tracker;

namespace {


Mat3 makeRotZ(float angleRad) {
    const float c = std::cos(angleRad);
    const float s = std::sin(angleRad);
    return Mat3(
        c, -s, 0.0f,
        s,  c, 0.0f,
        0.0f, 0.0f, 1.0f
    );
}

void pushRaw(MagCalibrationCollector& collector, const Vec3& raw, uint32_t seq) {
    Lsm6dsvFifoReader::MagRawSample sample;
    sample.x = static_cast<int16_t>(std::lround(raw.x));
    sample.y = static_cast<int16_t>(std::lround(raw.y));
    sample.z = static_cast<int16_t>(std::lround(raw.z));
    sample.seq = seq;
    const float normRaw = Vec3(static_cast<float>(sample.x), static_cast<float>(sample.y), static_cast<float>(sample.z)).norm();
    collector.push(sample, normRaw, seq);
}

std::vector<Vec3> makeDirections() {
    std::vector<Vec3> dirs;
    constexpr int thetaSteps = 18;
    constexpr int phiSteps = 36;
    dirs.reserve(thetaSteps * phiSteps);
    for (int ti = 0; ti < thetaSteps; ++ti) {
        const float theta = MATH_PI * (static_cast<float>(ti) + 0.5f) / static_cast<float>(thetaSteps);
        const float st = std::sin(theta);
        const float ct = std::cos(theta);
        for (int pi = 0; pi < phiSteps; ++pi) {
            const float phi = MATH_TWO_PI * static_cast<float>(pi) / static_cast<float>(phiSteps);
            dirs.emplace_back(st * std::cos(phi), st * std::sin(phi), ct);
        }
    }
    return dirs;
}

float correctedNormRms(const std::vector<Vec3>& rawSamples, const MagCalibrationResult& result) {
    double sumSq = 0.0;
    for (const Vec3& raw : rawSamples) {
        const Vec3 corrected = result.softIron * (raw - result.hardIron);
        const double e = static_cast<double>(corrected.norm() - result.expectedNorm);
        sumSq += e * e;
    }
    return static_cast<float>(std::sqrt(sumSq / static_cast<double>(rawSamples.size())));
}

} // namespace

int main() {
    TestContext ctx;

    {
        MagCalibrationParams params;
        params.minSamples = 300;
        params.maxAlgebraicResidualRms = 0.08f;
        MagCalibrationCollector collector(params);
        collector.start(0);

        const Vec3 hard(230.0f, -140.0f, 75.0f);
        const float target = 100.0f;
        const Mat3 rot = makeRotZ(0.37f);
        const Mat3 diag = Mat3::diagonal(target / 160.0f, target / 95.0f, target / 70.0f);
        const Mat3 trueSoft = rot * diag * rot.transposed();
        Mat3 trueSoftInv;
        CHECK(ctx, trueSoft.inverse(trueSoftInv));

        std::vector<Vec3> rawSamples;
        uint32_t seq = 1;
        for (const Vec3& dir : makeDirections()) {
            const Vec3 raw = hard + trueSoftInv * (dir * target);
            rawSamples.push_back(Vec3(
                static_cast<float>(std::lround(raw.x)),
                static_cast<float>(std::lround(raw.y)),
                static_cast<float>(std::lround(raw.z))
            ));
            pushRaw(collector, raw, seq++);
        }
        collector.stop();

        MagCalibrationResult result;
        CHECK(ctx, collector.compute(result));
        CHECK(ctx, result.valid);
        CHECK_NEAR(ctx, result.hardIron.x, hard.x, 1.5f);
        CHECK_NEAR(ctx, result.hardIron.y, hard.y, 1.5f);
        CHECK_NEAR(ctx, result.hardIron.z, hard.z, 1.5f);
        CHECK(ctx, std::fabs(result.softIron.m[0][1]) > 0.05f);
        const float expectedScale = result.expectedNorm / target;
        CHECK_NEAR(ctx, result.softIron.m[0][0], trueSoft.m[0][0] * expectedScale, 0.04f);
        CHECK_NEAR(ctx, result.softIron.m[0][1], trueSoft.m[0][1] * expectedScale, 0.04f);
        CHECK_NEAR(ctx, result.softIron.m[1][1], trueSoft.m[1][1] * expectedScale, 0.04f);
        CHECK_NEAR(ctx, result.softIron.m[2][2], trueSoft.m[2][2] * expectedScale, 0.04f);
        CHECK_NEAR(ctx, result.expectedNorm, (160.0f + 95.0f + 70.0f) / 3.0f, 2.0f);
        CHECK(ctx, result.coverageScore > 0.40f);
        CHECK(ctx, result.residualRms < 0.05f);
        CHECK(ctx, correctedNormRms(rawSamples, result) < 1.5f);
    }



    {
        MagCalibrationParams params;
        params.minSamples = 300;
        params.maxAlgebraicResidualRms = 0.08f;
        params.minInlierRatio = 0.90f;
        MagCalibrationCollector collector(params);
        collector.start(0);

        const Vec3 hard(-180.0f, 95.0f, 42.0f);
        const float target = 90.0f;
        const Mat3 rot = makeRotZ(-0.51f);
        const Mat3 diag = Mat3::diagonal(target / 130.0f, target / 80.0f, target / 60.0f);
        const Mat3 trueSoft = rot * diag * rot.transposed();
        Mat3 trueSoftInv;
        CHECK(ctx, trueSoft.inverse(trueSoftInv));

        std::vector<Vec3> rawSamples;
        uint32_t seq = 1;
        for (const Vec3& dir : makeDirections()) {
            const Vec3 raw = hard + trueSoftInv * (dir * target);
            rawSamples.push_back(Vec3(
                static_cast<float>(std::lround(raw.x)),
                static_cast<float>(std::lround(raw.y)),
                static_cast<float>(std::lround(raw.z))
            ));
            pushRaw(collector, raw, seq++);
        }
        for (uint32_t i = 0; i < 28; ++i) {
            const float sign = (i & 1U) ? 1.0f : -1.0f;
            pushRaw(collector, Vec3(900.0f * sign, -750.0f, 640.0f + static_cast<float>(i * 3U)), seq++);
        }
        collector.stop();

        MagCalibrationResult result;
        CHECK(ctx, collector.compute(result));
        CHECK(ctx, result.valid);
        CHECK_NEAR(ctx, result.hardIron.x, hard.x, 2.0f);
        CHECK_NEAR(ctx, result.hardIron.y, hard.y, 2.0f);
        CHECK_NEAR(ctx, result.hardIron.z, hard.z, 2.0f);
        CHECK(ctx, result.inlierRatio < 0.98f);
        CHECK(ctx, result.inlierRatio > 0.92f);
        CHECK(ctx, result.directionalCoverageScore > 0.80f);
        CHECK(ctx, result.normalizedResidualRms < 0.04f);
        CHECK(ctx, correctedNormRms(rawSamples, result) < 2.0f);
    }

    {
        MagCalibrationParams params;
        params.minSamples = 50;
        MagCalibrationCollector collector(params);
        collector.start(0);
        for (uint32_t i = 0; i < 100; ++i) {
            const float a = MATH_TWO_PI * static_cast<float>(i) / 100.0f;
            pushRaw(collector, Vec3(100.0f * std::cos(a), 100.0f * std::sin(a), 3.0f), i + 1);
        }
        collector.stop();
        MagCalibrationResult result;
        CHECK(ctx, !collector.compute(result));
        CHECK(ctx, !result.valid);
    }

    return ctx.finish("test_mag_calibration");
}
