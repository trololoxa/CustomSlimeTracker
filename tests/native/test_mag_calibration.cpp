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

void pushModulatedEllipsoid(MagCalibrationCollector& collector,
                            const Vec3& hardIron,
                            float radialModulation,
                            uint32_t& seq) {
    constexpr int thetaSteps = 18;
    constexpr int phiSteps = 36;
    for (int ti = 0; ti < thetaSteps; ++ti) {
        const float theta = MATH_PI * (static_cast<float>(ti) + 0.5f) / static_cast<float>(thetaSteps);
        const float st = std::sin(theta);
        const float ct = std::cos(theta);
        for (int pi = 0; pi < phiSteps; ++pi) {
            const float phi = MATH_TWO_PI * static_cast<float>(pi) / static_cast<float>(phiSteps);
            const Vec3 direction(st * std::cos(phi), st * std::sin(phi), ct);
            const float radial = 1.0f + radialModulation *
                (0.55f * std::sin(3.0f * phi) + 0.45f * std::cos(2.0f * theta));
            const Vec3 raw = hardIron + Vec3(
                direction.x * 520.0f,
                direction.y * 500.0f,
                direction.z * 490.0f
            ) * radial;
            pushRaw(collector, raw, seq++);
        }
    }
}

void pushPartiallyDisturbedEllipsoid(MagCalibrationCollector& collector,
                                     const Vec3& hardIron,
                                     float disturbedFraction,
                                     float disturbedScale,
                                     uint32_t& seq) {
    constexpr int thetaSteps = 18;
    constexpr int phiSteps = 36;
    uint32_t sampleIndex = 0u;
    const uint32_t disturbedPerHundred = static_cast<uint32_t>(
        clampf(disturbedFraction, 0.0f, 1.0f) * 100.0f + 0.5f);
    for (int ti = 0; ti < thetaSteps; ++ti) {
        const float theta = MATH_PI * (static_cast<float>(ti) + 0.5f) /
            static_cast<float>(thetaSteps);
        const float st = std::sin(theta);
        const float ct = std::cos(theta);
        for (int pi = 0; pi < phiSteps; ++pi, ++sampleIndex) {
            const float phi = MATH_TWO_PI * static_cast<float>(pi) /
                static_cast<float>(phiSteps);
            const Vec3 direction(st * std::cos(phi), st * std::sin(phi), ct);
            const bool disturbed = (sampleIndex % 100u) < disturbedPerHundred;
            const float radial = disturbed ? disturbedScale : 1.0f;
            pushRaw(collector, hardIron + Vec3(
                direction.x * 300.0f,
                direction.y * 280.0f,
                direction.z * 320.0f
            ) * radial, seq++);
        }
    }
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
        // A finite one-axis magnetic spike can expand the full reservoir's
        // raw extrema enough to fail box coverage even though the centered
        // robust fit set is healthy. Coverage gates must run on the retained
        // solver set, not before outlier filtering.
        MagCalibrationParams params;
        params.minSamples = 300;
        MagCalibrationCollector collector(params);
        collector.start(0);

        const Vec3 hard(100.0f, -70.0f, 40.0f);
        uint32_t seq = 1u;
        for (const Vec3& dir : makeDirections()) {
            pushRaw(collector, hard + Vec3(
                dir.x * 150.0f,
                dir.y * 100.0f,
                dir.z * 80.0f), seq++);
        }
        pushRaw(collector, Vec3(2500.0f, hard.y, hard.z), seq++);
        collector.stop();

        const MagCalibrationFitSetDiagnostics rawFitSet = collector.fitSetDiagnostics();
        CHECK(ctx, rawFitSet.valid);
        const float rawSpanX = rawFitSet.max.x - rawFitSet.min.x;
        const float rawSpanY = rawFitSet.max.y - rawFitSet.min.y;
        const float rawSpanZ = rawFitSet.max.z - rawFitSet.min.z;
        const float rawMinSpan = std::fmin(rawSpanX, std::fmin(rawSpanY, rawSpanZ));
        const float rawMaxSpan = std::fmax(rawSpanX, std::fmax(rawSpanY, rawSpanZ));
        CHECK(ctx, rawMinSpan / rawMaxSpan < magCalibrationEffectiveMinBoxCoverage(params));

        MagCalibrationResult result;
        CHECK(ctx, collector.compute(result));
        CHECK(ctx, result.valid);
        CHECK(ctx, result.coverageScore >= magCalibrationEffectiveMinBoxCoverage(params));
        CHECK_NEAR(ctx, result.hardIron.x, hard.x, 2.0f);
        CHECK_NEAR(ctx, result.hardIron.y, hard.y, 2.0f);
        CHECK_NEAR(ctx, result.hardIron.z, hard.z, 2.0f);
    }

    {
        // Regression: a long final sweep around one axis must not erase the
        // earlier full-sphere coverage from the bounded fit set.  The old
        // slot-permutation replacement behaved like a recent-sample ring and
        // made guided setup fail after the user had already completed good 3D
        // motion.
        MagCalibrationParams params;
        params.minSamples = 300;
        params.maxAlgebraicResidualRms = 0.08f;
        MagCalibrationCollector collector(params);
        collector.start(0);

        const Vec3 hard(150.0f, -80.0f, 55.0f);
        const float target = 95.0f;
        const Mat3 rot = makeRotZ(0.29f);
        const Mat3 diag = Mat3::diagonal(target / 145.0f, target / 88.0f, target / 68.0f);
        const Mat3 trueSoft = rot * diag * rot.transposed();
        Mat3 trueSoftInv;
        CHECK(ctx, trueSoft.inverse(trueSoftInv));

        uint32_t seq = 1;
        const std::vector<Vec3> dirs = makeDirections();
        for (int repeat = 0; repeat < 1; ++repeat) {
            for (const Vec3& dir : dirs) {
                pushRaw(collector, hard + trueSoftInv * (dir * target), seq++);
            }
        }
        for (uint32_t i = 0; i < 12000; ++i) {
            const float phi = MATH_TWO_PI * static_cast<float>(i % 720U) / 720.0f;
            const Vec3 equator(std::cos(phi), std::sin(phi), 0.02f);
            pushRaw(collector, hard + trueSoftInv * (equator.normalized() * target), seq++);
        }
        collector.stop();

        MagCalibrationResult result;
        CHECK(ctx, collector.storedSamples() == 768u);
        CHECK(ctx, collector.reservoirReplacements() > 0u);
        CHECK(ctx, collector.reservoirSkipped() > 0u);
        const MagCalibrationFitSetDiagnostics fitSet = collector.fitSetDiagnostics();
        CHECK(ctx, fitSet.valid);
        CHECK(ctx, fitSet.samples == collector.storedSamples());
        CHECK(ctx, collector.compute(result));
        CHECK(ctx, result.valid);
        const float fitSpanX = fitSet.max.x - fitSet.min.x;
        const float fitSpanY = fitSet.max.y - fitSet.min.y;
        const float fitSpanZ = fitSet.max.z - fitSet.min.z;
        const float fitMinSpan = std::fmin(fitSpanX, std::fmin(fitSpanY, fitSpanZ));
        const float fitMaxSpan = std::fmax(fitSpanX, std::fmax(fitSpanY, fitSpanZ));
        CHECK_NEAR(ctx, result.coverageScore, fitMinSpan / fitMaxSpan, 1.0e-6f);
        CHECK(ctx, result.directionalCoverageScore > 0.65f);
        CHECK_NEAR(ctx, result.hardIron.x, hard.x, 3.0f);
        CHECK_NEAR(ctx, result.hardIron.y, hard.y, 3.0f);
        CHECK_NEAR(ctx, result.hardIron.z, hard.z, 3.0f);
    }

    {
        // Pre-fit raw span balance must not reject the strong but still
        // permitted soft-iron anisotropy that the ellipsoid solver is meant
        // to estimate. With maxAxisRatio=6, a fully covered 4:1 ellipsoid is
        // valid even though its raw box ratio is below the legacy 0.35 gate.
        MagCalibrationParams params;
        params.minSamples = 300;
        MagCalibrationCollector collector(params);
        collector.start(0);
        const Vec3 hard(120.0f, -90.0f, 60.0f);
        const float target = 100.0f;
        const Mat3 trueSoft = Mat3::diagonal(
            target / 600.0f, target / 300.0f, target / 150.0f);
        Mat3 inverse;
        CHECK(ctx, trueSoft.inverse(inverse));
        uint32_t seq = 1u;
        for (const Vec3& dir : makeDirections()) {
            pushRaw(collector, hard + inverse * (dir * target), seq++);
        }
        collector.stop();
        MagCalibrationResult result;
        CHECK(ctx, collector.compute(result));
        CHECK(ctx, result.valid);
        CHECK(ctx, result.coverageScore < params.minCoverageScore);
        CHECK(ctx, result.coverageScore >= magCalibrationEffectiveMinBoxCoverage(params));
        CHECK(ctx, result.axisRatio > 3.5f && result.axisRatio < 4.5f);
        CHECK(ctx, result.directionalCoverageScore >= params.minDirectionalCoverageScore);
    }

    {
        // Regression: algebraic fit quality must describe the centered
        // ellipsoid, not its arbitrary translation from the raw origin.  The
        // pre-0023gb metric multiplied the same physical residual by
        // k = 1 + c^T A c and could reject a good fit only because hard-iron
        // bias was large.
        MagCalibrationParams params;
        params.minSamples = 300;
        MagCalibrationCollector centered(params);
        MagCalibrationCollector translated(params);
        centered.start(0);
        translated.start(0);
        uint32_t centeredSeq = 1;
        uint32_t translatedSeq = 1;
        pushModulatedEllipsoid(centered, Vec3::zero(), 0.0f, centeredSeq);
        pushModulatedEllipsoid(translated, Vec3(400.0f, -300.0f, 180.0f), 0.0f, translatedSeq);
        centered.stop();
        translated.stop();

        MagCalibrationResult centeredResult;
        MagCalibrationResult translatedResult;
        CHECK(ctx, centered.compute(centeredResult));
        CHECK(ctx, translated.compute(translatedResult));
        CHECK(ctx, centeredResult.valid);
        CHECK(ctx, translatedResult.valid);
        CHECK_NEAR(ctx, centeredResult.residualRms, translatedResult.residualRms, 2.0e-6f);
        CHECK_NEAR(ctx, centeredResult.normalizedResidualRms, translatedResult.normalizedResidualRms, 2.0e-6f);
        CHECK_NEAR(ctx, translatedResult.hardIron.x, 400.0f, 0.1f);
        CHECK_NEAR(ctx, translatedResult.hardIron.y, -300.0f, 0.1f);
        CHECK_NEAR(ctx, translatedResult.hardIron.z, 180.0f, 0.1f);
    }

    {
        // Regression for the hardware failure where hard-iron is about one
        // field radius from the ADC origin. The raw ellipsoid passes through
        // (or very near) zero, so the uncentered fixed-constant equation is
        // singular even though physical 3D coverage is excellent.
        MagCalibrationParams params;
        params.minSamples = 300;
        MagCalibrationCollector collector(params);
        collector.start(0);

        const Vec3 hard(500.0f, 0.0f, 0.0f);
        const float target = 100.0f;
        const Mat3 rot = Quat::fromEulerXYZ(0.17f, -0.23f, 0.31f).toRotationMatrix();
        const Mat3 diag = Mat3::diagonal(target / 520.0f, target / 500.0f, target / 490.0f);
        const Mat3 trueSoft = rot * diag * rot.transposed();
        Mat3 trueSoftInv;
        CHECK(ctx, trueSoft.inverse(trueSoftInv));
        uint32_t seq = 1u;
        for (const Vec3& dir : makeDirections()) {
            pushRaw(collector, hard + trueSoftInv * (dir * target), seq++);
        }
        collector.stop();

        MagCalibrationResult result;
        CHECK(ctx, collector.compute(result));
        CHECK(ctx, result.valid);
        CHECK(ctx, result.fitAvailable);
        CHECK(ctx, result.solverStage == MagCalibrationSolverStage::CandidateBuilt);
        CHECK(ctx, result.solverPivotRatio > 1.0e-5f);
        CHECK_NEAR(ctx, result.hardIron.x, hard.x, 1.5f);
        CHECK_NEAR(ctx, result.hardIron.y, hard.y, 1.5f);
        CHECK_NEAR(ctx, result.hardIron.z, hard.z, 1.5f);
        CHECK(ctx, result.normalizedResidualRms < 0.02f);
    }

    {
        // Deterministic convergence sweep: translation, 3D eigenvector rotation
        // and permitted anisotropy must not change whether the same physical
        // ellipsoid can be solved. This guards the complete normalization ->
        // center -> SPD-shape -> geometric-validation chain rather than one
        // hand-picked orientation.
        struct FitCase {
            Vec3 hard;
            Vec3 euler;
            Vec3 radii;
        };
        const FitCase cases[] = {
            {Vec3::zero(), Vec3::zero(), Vec3(500.0f, 500.0f, 500.0f)},
            {Vec3(500.0f, 0.0f, 0.0f), Vec3(0.2f, -0.3f, 0.4f), Vec3(520.0f, 490.0f, 505.0f)},
            {Vec3(-620.0f, 310.0f, -180.0f), Vec3(-0.5f, 0.25f, 0.7f), Vec3(650.0f, 340.0f, 180.0f)},
            {Vec3(820.0f, -730.0f, 610.0f), Vec3(0.65f, -0.45f, -0.35f), Vec3(720.0f, 240.0f, 145.0f)},
        };
        for (const FitCase& c : cases) {
            MagCalibrationParams params;
            params.minSamples = 300;
            MagCalibrationCollector collector(params);
            collector.start(0);
            const float target = 100.0f;
            const Mat3 rotation = Quat::fromEulerXYZ(c.euler.x, c.euler.y, c.euler.z).toRotationMatrix();
            const Mat3 trueSoft = rotation *
                Mat3::diagonal(target / c.radii.x, target / c.radii.y, target / c.radii.z) *
                rotation.transposed();
            Mat3 inverse;
            CHECK(ctx, trueSoft.inverse(inverse));
            uint32_t seq = 1u;
            for (const Vec3& dir : makeDirections()) {
                const float deterministicNoise = static_cast<float>(static_cast<int>(seq % 5u) - 2) * 0.20f;
                pushRaw(collector, c.hard + inverse * (dir * (target + deterministicNoise)), seq++);
            }
            collector.stop();
            MagCalibrationResult result;
            CHECK(ctx, collector.compute(result));
            CHECK(ctx, result.valid);
            CHECK(ctx, result.solverStage == MagCalibrationSolverStage::CandidateBuilt);
            CHECK(ctx, result.solverPivotRatio > 1.0e-7f);
            CHECK_NEAR(ctx, result.hardIron.x, c.hard.x, 3.0f);
            CHECK_NEAR(ctx, result.hardIron.y, c.hard.y, 3.0f);
            CHECK_NEAR(ctx, result.hardIron.z, c.hard.z, 3.0f);
            CHECK(ctx, result.normalizedResidualRms < 0.035f);
            CHECK(ctx, result.directionalCoverageScore >= params.minDirectionalCoverageScore);
        }
    }

    {
        // Mild non-ellipsoidal field variation may still be physically useful
        // after robust geometric validation.  The old origin-dependent
        // algebraic metric rejected this fixture despite sub-3% normalized
        // geometric error.
        MagCalibrationParams params;
        params.minSamples = 300;
        MagCalibrationCollector collector(params);
        collector.start(0);
        uint32_t seq = 1;
        pushModulatedEllipsoid(collector, Vec3(400.0f, -300.0f, 180.0f), 0.05f, seq);
        collector.stop();

        MagCalibrationResult result;
        CHECK(ctx, collector.compute(result));
        CHECK(ctx, result.valid);
        CHECK(ctx, result.residualRms < magCalibrationEffectiveMaxAlgebraicResidualRms(params));
        CHECK(ctx, result.normalizedResidualRms < 0.04f);
        CHECK(ctx, result.inlierRatio > 0.95f);
    }

    {
        // The centered algebraic residual is approximately twice radial
        // geometric error. It is a numerical consistency backstop and must
        // not reject a model that passes the authoritative geometric gate.
        MagCalibrationParams params;
        params.minSamples = 300;
        MagCalibrationCollector collector(params);
        collector.start(0);
        uint32_t seq = 1;
        pushModulatedEllipsoid(collector, Vec3(115.0f, -86.0f, 251.0f), 0.20f, seq);
        collector.stop();

        MagCalibrationResult result;
        CHECK(ctx, collector.compute(result));
        CHECK(ctx, result.valid);
        CHECK(ctx, result.normalizedResidualRms < params.maxGeometricResidualRmsFactor);
        CHECK(ctx, result.residualRms > params.maxAlgebraicResidualRms);
        CHECK(ctx, result.residualRms < magCalibrationEffectiveMaxAlgebraicResidualRms(params));
    }

    {
        // Hardware-shaped moderate contamination must not inflate its own
        // sigma threshold and poison the fit. A bounded iterative refit can
        // discard up to the configured inlier budget and recover the common
        // ellipsoid without weakening the final geometric gate.
        MagCalibrationParams params;
        params.minSamples = 300;
        MagCalibrationCollector collector(params);
        collector.start(0);
        uint32_t seq = 1;
        const Vec3 hard(115.0f, -86.0f, 251.0f);
        pushPartiallyDisturbedEllipsoid(collector, hard, 0.15f, 1.35f, seq);
        collector.stop();

        MagCalibrationResult result;
        CHECK(ctx, collector.compute(result));
        CHECK(ctx, result.valid);
        CHECK(ctx, result.robustRefitPasses > 0u);
        CHECK_NEAR(ctx, result.robustInlierThresholdFactor,
                   magCalibrationRobustResidualCapFactor(params), 1.0e-6f);
        CHECK(ctx, result.inlierRatio >= params.minInlierRatio);
        CHECK(ctx, result.inlierRatio < 0.90f);
        CHECK(ctx, result.normalizedResidualRms < 0.01f);
        CHECK_NEAR(ctx, result.hardIron.x, hard.x, 2.0f);
        CHECK_NEAR(ctx, result.hardIron.y, hard.y, 2.0f);
        CHECK_NEAR(ctx, result.hardIron.z, hard.z, 2.0f);
    }

    {
        // The bounded refit is not a blanket acceptance path. Once the
        // disturbed population exceeds the configured 18% outlier budget,
        // the same fixture remains fail-closed on independent inlier evidence.
        MagCalibrationParams params;
        params.minSamples = 300;
        MagCalibrationCollector collector(params);
        collector.start(0);
        uint32_t seq = 1;
        pushPartiallyDisturbedEllipsoid(
            collector, Vec3(115.0f, -86.0f, 251.0f), 0.20f, 1.35f, seq);
        collector.stop();

        MagCalibrationResult result;
        CHECK(ctx, !collector.compute(result));
        CHECK(ctx, collector.lastFailureReason() == MagCalibrationFailureReason::InlierRatioTooLow);
        CHECK(ctx, collector.lastResult().inlierRatio < params.minInlierRatio);
    }

    {
        // Normalization must not turn the algebraic gate into a blanket pass.
        // Strongly non-ellipsoidal data still fails the physical geometric
        // quality gate, and the rejected fit remains available for diagnosis.
        MagCalibrationParams params;
        params.minSamples = 300;
        // Keep every point in the quality population so this fixture isolates
        // the final geometric gate rather than the robust inlier-budget gate.
        params.outlierMinResidualFactor = 0.50f;
        MagCalibrationCollector collector(params);
        collector.start(0);
        uint32_t seq = 1;
        pushModulatedEllipsoid(collector, Vec3(400.0f, -300.0f, 180.0f), 0.30f, seq);
        collector.stop();

        MagCalibrationResult result;
        CHECK(ctx, !collector.compute(result));
        CHECK(ctx, !result.valid);
        CHECK(ctx, collector.lastFailureReason() == MagCalibrationFailureReason::GeometricResidualTooHigh);
        const MagCalibrationResult& failed = collector.lastResult();
        CHECK(ctx, failed.fitAvailable);
        CHECK(ctx, failed.expectedNorm > 0.0f);
        CHECK(ctx, failed.softIron.isFinite());
        CHECK(ctx, failed.normalizedResidualRms > params.maxGeometricResidualRmsFactor);
    }


    {
        // A physical pre-solve coverage rejection still completed centering and
        // normalization. Preserve those diagnostics instead of returning an
        // all-zero solver state that hides whether the data or the algebra failed.
        MagCalibrationParams params;
        params.minSamples = 300;
        MagCalibrationCollector collector(params);
        collector.start(0);
        uint32_t seq = 1u;
        for (const Vec3& dir : makeDirections()) {
            pushRaw(collector, Vec3(240.0f, -130.0f, 75.0f) + Vec3(
                dir.x * 120.0f, dir.y * 90.0f, dir.z * 10.0f), seq++);
        }
        collector.stop();

        MagCalibrationResult result;
        CHECK(ctx, !collector.compute(result));
        CHECK(ctx, collector.lastFailureReason() == MagCalibrationFailureReason::AxisRadiusTooSmall);
        CHECK(ctx, result.solverStage == MagCalibrationSolverStage::Normalized);
        CHECK(ctx, result.solverSamples >= params.minSamples);
        CHECK_NEAR(ctx, result.fitNormalizationCenter.x, 240.0f, 1.0f);
        CHECK_NEAR(ctx, result.fitNormalizationCenter.y, -130.0f, 1.0f);
        CHECK_NEAR(ctx, result.fitNormalizationCenter.z, 75.0f, 1.0f);
        CHECK(ctx, result.fitNormalizationScale.x > 10.0f);
        CHECK(ctx, result.fitNormalizationScale.y > 10.0f);
        CHECK(ctx, result.fitNormalizationScale.z > 1.0f);
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
