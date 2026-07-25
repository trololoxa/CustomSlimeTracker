#include "test_common.hpp"

#include <initializer_list>

#include "sensor/mag_axis_alignment.hpp"
#include "sensor/mag_field_reliability.hpp"
#include "sensor/mag_yaw_correction.hpp"

using namespace tracker;

static MagFieldReliabilityInput makeFieldInput(uint32_t ms, float yawDeg, float dipDeg, float norm) {
    MagFieldReliabilityInput in;
    in.nowMs = ms;
    in.processorTrustedForUse = true;
    in.gyroNormDps = 0.2f;
    in.accelTrust = 1.0f;
    in.mag.valid = true;
    in.mag.trusted = true;
    in.mag.bodyNorm = norm;
    in.mag.seq = ms + 1u;
    in.mag.receivedMs = ms;
    in.mag.t_us = static_cast<uint64_t>(ms) * 1000ULL;
    in.heading.valid = true;
    in.heading.dipDeg = dipDeg;
    in.heading.magneticNorthWorldYawRad = yawDeg * MATH_DEG_TO_RAD;
    return in;
}

static void testFieldReliabilityFailsClosedAcrossChangedEnvironment(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 1000;
    cfg.recoverStableMs = 1000;
    cfg.suspectClearMs = 300;
    cfg.newEnvironmentStableMs = 400;
    cfg.referenceReturnStableMs = 300;

    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(1101, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK(ctx, out.referenceValid);

    monitor.update(makeFieldInput(1201, 35.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
    CHECK(ctx, !out.trustedForYaw);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_HEADING_STEP_HARD) != 0u);

    // A stable but directionally different field is not silently accepted as a
    // replacement environment while the old absolute heading reference exists.
    monitor.update(makeFieldInput(1301, 35.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(1801, 35.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(2901, 35.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
    CHECK(ctx, !out.trustedForYaw);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_ENVIRONMENT_CHANGED) != 0u);
    CHECK(ctx, monitor.stats().environmentChangesDetected == 1u);

    // Returning to the original environment enters a fresh recovery dwell.
    monitor.update(makeFieldInput(3001, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
    monitor.update(makeFieldInput(3402, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Recovering);
    CHECK(ctx, !out.trustedForYaw);
    monitor.update(makeFieldInput(4503, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);

    // An explicit reference reset is the only path that can accept a genuinely
    // new environment; it averages a new acquisition window rather than one sample.
    monitor.restartAcquisition();
    monitor.update(makeFieldInput(5000, 35.1f, 54.8f, 502.0f), cfg, out);
    monitor.update(makeFieldInput(6101, 34.9f, 55.2f, 498.0f), cfg, out);
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK_NEAR(ctx, out.referenceHeadingYawDeg, 35.0f, 0.2f);
    CHECK_NEAR(ctx, out.referenceNorm, 500.0f, 0.5f);
}

static void testModerateStationaryHeadingJumpStaysFailClosed(TestContext& ctx) {
    for (float shiftDeg : {5.0f, 9.0f, 12.0f, 19.0f}) {
        MagFieldReliabilityConfig cfg;
        cfg.acquireStableMs = 500;
        cfg.referenceReturnStableMs = 200;
        cfg.recoverStableMs = 300;

        MagFieldReliabilityMonitor monitor;
        MagFieldReliabilityOutput out;
        monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
        monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
        CHECK(ctx, out.trustedForYaw);

        monitor.update(makeFieldInput(618u, shiftDeg, 55.0f, 500.0f), cfg, out);
        CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
        CHECK(ctx, !out.trustedForYaw);
        CHECK(ctx, out.stationaryHeadingJumpLatched);
        CHECK(ctx, (out.flags & MAG_FIELD_FLAG_STATIONARY_HEADING_JUMP) != 0u);

        for (uint32_t i = 0; i < 180u; ++i) {
            monitor.update(makeFieldInput(635u + i * 17u, shiftDeg, 55.0f, 500.0f), cfg, out);
        }
        CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
        CHECK(ctx, !out.trustedForYaw);
        CHECK(ctx, out.stationaryHeadingJumpLatched);
    }
}

static void testStationaryJumpAtWindowBoundaryStillLatches(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    cfg.stationaryHeadingWindowMs = 1500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    // Refresh the rolling anchor, then place the jump just beyond its nominal
    // window boundary. Adjacent-sample detection must cover this boundary.
    monitor.update(makeFieldInput(2202u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(2219u, 9.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.stationaryHeadingJumpLatched);
    CHECK(ctx, out.state == MagFieldReliabilityState::Disturbed);
    CHECK(ctx, !out.trustedForYaw);
}

static void testSlowStationaryYawDriftRemainsCorrectable(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    uint32_t nowMs = 618u;
    float yawDeg = 0.0f;
    for (uint32_t i = 0; i < 360u; ++i) {
        yawDeg += 1.0f * 0.017f; // 1 deg/s: faster than normal gyro drift, below discontinuity gate.
        monitor.update(makeFieldInput(nowMs, yawDeg, 55.0f, 500.0f), cfg, out);
        nowMs += 17u;
    }
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    CHECK(ctx, out.referenceHeadingErrorDeg > 5.0f);
}

static void testFastButPlausibleYawDriftDoesNotSelfLatch(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    uint32_t nowMs = 618u;
    float yawDeg = 0.0f;
    for (uint32_t i = 0; i < 240u; ++i) {
        yawDeg += 4.0f * 0.017f;
        monitor.update(makeFieldInput(nowMs, yawDeg, 55.0f, 500.0f), cfg, out);
        nowMs += 17u;
    }
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
}

static void testMaximumNormalYawCorrectionDoesNotSelfLatch(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    uint32_t nowMs = 618u;
    float yawDeg = 0.0f;
    for (uint32_t i = 0; i < 360u; ++i) {
        yawDeg += 2.0f * 0.017f; // Current maximum normal yaw-correction rate.
        monitor.update(makeFieldInput(nowMs, yawDeg, 55.0f, 500.0f), cfg, out);
        nowMs += 17u;
    }
    CHECK(ctx, out.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, out.trustedForYaw);
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
}

static void testRealRotationDoesNotTriggerStationaryJumpLatch(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    uint32_t nowMs = 618u;
    float yawDeg = 0.0f;
    for (uint32_t i = 0; i < 120u; ++i) {
        yawDeg += 30.0f * 0.017f;
        auto in = makeFieldInput(nowMs, yawDeg, 55.0f, 500.0f);
        in.gyroNormDps = 30.0f;
        monitor.update(in, cfg, out);
        nowMs += 17u;
    }
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_STATIONARY_HEADING_JUMP) == 0u);
}

static void testVerySlowPhysicalRotationDoesNotTriggerJumpLatch(TestContext& ctx) {
    MagFieldReliabilityConfig cfg;
    cfg.acquireStableMs = 500;
    MagFieldReliabilityMonitor monitor;
    MagFieldReliabilityOutput out;
    monitor.update(makeFieldInput(1u, 0.0f, 55.0f, 500.0f), cfg, out);
    monitor.update(makeFieldInput(601u, 0.0f, 55.0f, 500.0f), cfg, out);
    CHECK(ctx, out.trustedForYaw);

    uint32_t nowMs = 618u;
    float yawDeg = 0.0f;
    for (uint32_t i = 0; i < 240u; ++i) {
        yawDeg += 3.0f * 0.017f;
        auto in = makeFieldInput(nowMs, yawDeg, 55.0f, 500.0f);
        in.gyroNormDps = 3.0f;
        monitor.update(in, cfg, out);
        nowMs += 17u;
    }
    CHECK(ctx, !out.stationaryHeadingJumpLatched);
    CHECK(ctx, (out.flags & MAG_FIELD_FLAG_STATIONARY_HEADING_JUMP) == 0u);
}

static void testFilteredHeadingRateAllowsNoisySixtyHertzReacquisition(TestContext& ctx) {
    MagFieldReliabilityConfig fieldCfg;
    fieldCfg.acquireStableMs = 1000;
    fieldCfg.headingRateFilterTimeConstantS = 0.75f;

    MagYawCorrectionConfig yawCfg;
    yawCfg.applyEnabled = true;
    yawCfg.maxInnovationDeg = 25.0f;
    yawCfg.reacquireMinFieldStableMs = 8000;
    yawCfg.reacquireMaxHeadingRateDegS = 2.0f;

    MagFieldReliabilityMonitor monitor;
    MagYawCorrectionController yaw;
    MagFieldReliabilityOutput fieldOut;
    MagYawCorrectionOutput yawOut;
    bool reacquired = false;
    float maxFilteredAfterSettling = 0.0f;

    uint32_t nowMs = 1u;
    for (uint32_t i = 0; i < 780u; ++i) {
        const float jitter = (i & 1u) ? 0.05f : -0.05f;
        const float headingDeg = 40.0f + jitter;
        const auto fieldIn = makeFieldInput(nowMs, headingDeg, 55.0f, 500.0f);
        monitor.update(fieldIn, fieldCfg, fieldOut);
        if (nowMs > 3000u) {
            maxFilteredAfterSettling = std::max(maxFilteredAfterSettling, fieldOut.headingRateDegS);
        }

        MagYawCorrectionInput yi;
        yi.referenceValid = true;
        yi.referenceWorldYawRad = 0.0f;
        yi.mag = fieldIn.mag;
        yi.heading = fieldIn.heading;
        yi.heading.horizontalNorm = 300.0f;
        yi.magTrustedForUse = true;
        yi.magRejectFlagsForUse = MAG_REJECT_NONE;
        yi.gyroNormDps = 0.2f;
        yi.accelTrust = 1.0f;
        yi.fieldReliable = fieldOut.trustedForYaw;
        yi.fieldStableMs = fieldOut.stableMs;
        yi.magneticHeadingRateDegS = fieldOut.headingRateDegS;
        yi.nowMs = nowMs;
        yaw.update(yi, yawCfg, yawOut);
        reacquired = reacquired || yawOut.reacquireActive;
        nowMs += 17u;
    }

    CHECK(ctx, fieldOut.state == MagFieldReliabilityState::Trusted);
    CHECK(ctx, fieldOut.trustedForYaw);
    CHECK(ctx, maxFilteredAfterSettling < yawCfg.reacquireMaxHeadingRateDegS);
    CHECK(ctx, reacquired);
    CHECK(ctx, yaw.stats().reacquireGateOpenCount >= 1u);
}

static void testOnlyProperSignedPermutationsAreAccepted(TestContext& ctx) {
    static constexpr uint8_t perms[6][3] = {
        {0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}
    };
    int proper = 0;
    for (const auto& p : perms) {
        for (int sx=-1; sx<=1; sx+=2) for (int sy=-1; sy<=1; sy+=2) for (int sz=-1; sz<=1; sz+=2) {
            const Mat3 m = MagAxisAlignmentCollector::signedPermutation(
                p[0], static_cast<float>(sx), p[1], static_cast<float>(sy), p[2], static_cast<float>(sz));
            if (MagAxisAlignmentCollector::properRotation(m)) proper++;
        }
    }
    CHECK(ctx, proper == 24);
    CHECK(ctx, !MagAxisAlignmentCollector::properRotation(Mat3(-1,0,0, 0,1,0, 0,0,1)));
}


static void testAxisCollectorRejectsStaleGyroPair(TestContext& ctx) {
    MagAxisAlignmentCollector collector;
    MagProcessedSample mag;
    mag.valid = mag.trusted = true;
    mag.raw = Vec3(1.0f, 0.0f, 0.0f);
    mag.rawNorm = 1.0f;
    mag.seq = 1u;
    mag.t_us = 100000u;
    mag.receivedMs = 100u;
    CHECK(ctx, !collector.observe(Vec3(0.0f, 0.0f, 1.0f), 90000u, mag, true));
    CHECK(ctx, collector.stats().intervalsRejectedGyroSkew == 1u);
    CHECK(ctx, collector.intervalCount() == 0u);
}


static void testSixtyHertzCollectionSpansIndependentWindows(TestContext& ctx) {
    MagAxisAlignmentCollector collector;
    Vec3 m = Vec3(0.45f, 0.2f, 0.87f).normalized();
    uint64_t tUs = 100000u;
    uint32_t ms = 100u;

    MagProcessedSample first;
    first.valid = first.trusted = true;
    first.raw = m;
    first.rawNorm = 1.0f;
    first.seq = 1u;
    first.t_us = tUs;
    first.receivedMs = ms;
    collector.observe(Vec3::zero(), tUs, first, true);

    for (uint32_t i = 0; i < 240u; ++i) {
        const Vec3 gyro = (i % 2u == 0u) ? Vec3(0.8f, 0.1f, 0.0f)
                                            : Vec3(0.0f, 0.9f, 0.1f);
        const float dt = 0.017f;
        m = (m - cross(gyro, m) * dt).normalized();
        tUs += 17000u;
        ms += 17u;
        MagProcessedSample mag;
        mag.valid = mag.trusted = true;
        mag.raw = m;
        mag.rawNorm = 1.0f;
        mag.seq = i + 2u;
        mag.t_us = tUs;
        mag.receivedMs = ms;
        collector.observe(gyro, tUs, mag, true);
    }

    CHECK(ctx, collector.intervalCount() >= MagAxisAlignmentCollector::kTargetIntervals);
    CHECK(ctx, collector.independentWindows() >= 4u);
    CHECK(ctx, collector.readyToSolve());
    CHECK(ctx, collector.stats().intervalsSkippedCadence > 0u);
    CHECK(ctx, collector.stats().intervalsRejectedCapacity == 0u);
}

static void testDynamicAxisSolverRefinesMechanicalMisalignment(TestContext& ctx) {
    const Mat3 coarse(0,-1,0, 1,0,0, 0,0,1);
    const Mat3 residual = Quat::fromEulerXYZ(
        0.7f * MATH_DEG_TO_RAD,
       -1.1f * MATH_DEG_TO_RAD,
        1.6f * MATH_DEG_TO_RAD).toRotationMatrix();
    const Mat3 expected = residual * coarse;
    const Mat3 inverse = expected.transposed();

    MagAxisAlignmentInterval intervals[64];
    const Vec3 starts[4] = {
        Vec3(0.45f, 0.20f, 0.87f).normalized(),
        Vec3(-0.30f, 0.91f, 0.28f).normalized(),
        Vec3(0.82f, -0.44f, 0.36f).normalized(),
        Vec3(-0.70f, -0.18f, 0.69f).normalized(),
    };
    for (uint16_t i = 0; i < 64u; ++i) {
        const uint16_t local = static_cast<uint16_t>(i % 16u);
        Vec3 m0 = local == 0u ? starts[i / 16u]
                              : expected * intervals[i - 1u].mag1Raw;
        const Vec3 gyro = (i % 5u == 0u) ? Vec3(1.10f, 0.20f, -0.05f)
                         : (i % 5u == 1u) ? Vec3(-0.10f, 0.95f, 0.35f)
                         : (i % 5u == 2u) ? Vec3(0.30f, -0.15f, 1.05f)
                         : (i % 5u == 3u) ? Vec3(0.75f, 0.65f, -0.25f)
                                          : Vec3(-0.55f, 0.40f, 0.85f);
        const float dt = 0.050f;
        const Vec3 m1 = Quat::fromRotationVector(gyro * (-dt)).rotate(m0).normalized();
        intervals[i].gyroSensorRadS = gyro;
        intervals[i].mag0Raw = inverse * m0;
        intervals[i].mag1Raw = inverse * m1;
        intervals[i].dtS = dt;
        intervals[i].windowId = static_cast<uint16_t>(i / 8u);
    }

    MagAxisAlignmentSolvePolicy policy;
    policy.minIntervals = 24u;
    policy.minTrainingIntervals = 12u;
    policy.minValidationIntervals = 12u;
    policy.minIndependentWindows = 4u;
    policy.minTrainingWindows = 2u;
    policy.minValidationWindows = 2u;
    MagAxisAlignmentResult result;
    CHECK(ctx, solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), Mat3::identity(), 3u, 5u,
        &coarse, policy, result));
    CHECK(ctx, result.valid);
    CHECK(ctx, result.refined);
    CHECK(ctx, result.activeCompared);
    CHECK(ctx, result.improvesActive);
    CHECK(ctx, result.validationPassed);
    CHECK(ctx, result.validationWinnerMatchesTraining);
    CHECK(ctx, MagAxisAlignmentCollector::properRotation(result.magToImu));
    CHECK(ctx, magAxisRotationDifferenceDeg(result.coarseMagToImu, coarse) < 0.1f);
    CHECK(ctx, magAxisRotationDifferenceDeg(result.magToImu, expected) < 0.65f);
    CHECK(ctx, result.refinementAngleDeg > 1.0f);
    CHECK(ctx, result.refinementAngleDeg < 4.0f);
    CHECK(ctx, result.score < result.coarseScore);
    CHECK(ctx, result.score < result.activeScore);
    CHECK(ctx, result.qualityScore > 0.62f);
}

static bool makeRateDataset(float rateDegS,
                            MagAxisAlignmentInterval (&intervals)[64],
                            Mat3& coarse,
                            Mat3& expected) {
    coarse = Mat3(0,-1,0, 1,0,0, 0,0,1);
    const Mat3 residual = Quat::fromEulerXYZ(
        0.7f * MATH_DEG_TO_RAD,
       -1.1f * MATH_DEG_TO_RAD,
        1.6f * MATH_DEG_TO_RAD).toRotationMatrix();
    expected = residual * coarse;
    const Mat3 inverse = expected.transposed();
    Vec3 m = Vec3(0.45f, 0.20f, 0.87f).normalized();
    const float rate = rateDegS * MATH_DEG_TO_RAD;
    for (uint16_t i = 0; i < 64u; ++i) {
        const Vec3 axis = (i % 4u == 0u) ? Vec3(1.0f, 0.2f, 0.1f).normalized()
                        : (i % 4u == 1u) ? Vec3(0.1f, 1.0f, 0.3f).normalized()
                        : (i % 4u == 2u) ? Vec3(0.25f, -0.1f, 1.0f).normalized()
                                         : Vec3(-0.7f, 0.5f, 0.5f).normalized();
        const Vec3 gyro = axis * rate;
        const float dt = 0.017f;
        const Vec3 next = Quat::fromRotationVector(gyro * (-dt)).rotate(m).normalized();
        intervals[i] = MagAxisAlignmentInterval{
            gyro, inverse * m, inverse * next, dt,
            static_cast<uint16_t>(i / 8u)};
        m = next;
    }
    return true;
}

static void testSolverConfidenceIsRateNormalized(TestContext& ctx) {
    for (float rateDegS : {30.0f, 60.0f, 90.0f, 120.0f}) {
        MagAxisAlignmentInterval intervals[64];
        Mat3 coarse;
        Mat3 expected;
        makeRateDataset(rateDegS, intervals, coarse, expected);
        MagAxisAlignmentSolvePolicy policy;
        MagAxisAlignmentResult result;
        CHECK(ctx, solveMagAxisAlignmentDataset(
            intervals, 64u, Vec3::zero(), Mat3::identity(), 3u, 8u,
            &coarse, policy, result));
        CHECK(ctx, result.valid);
        CHECK(ctx, result.validationPassed);
        CHECK(ctx, result.validationWinnerMatchesTraining);
        CHECK(ctx, result.trainingValidationRotationDifferenceDeg <=
                   policy.maxTrainingValidationRotationDifferenceDeg);
        CHECK(ctx, result.normalizedSeparation >= policy.minNormalizedSeparation);
        CHECK(ctx, result.totalObservableRotationDeg >= policy.minTotalObservableRotationDeg);
        CHECK(ctx, magAxisRotationDifferenceDeg(result.magToImu, expected) < 1.5f);
        if (rateDegS >= 60.0f) CHECK(ctx, result.improvesActive);
    }
}

static void testHoldoutRejectsTrainingOnlyFit(TestContext& ctx) {
    MagAxisAlignmentInterval intervals[64];
    Mat3 coarse;
    Mat3 expected;
    makeRateDataset(120.0f, intervals, coarse, expected);

    // Corrupt only validation windows with a non-rigid extra endpoint change.
    // Training remains perfectly solvable, but its fitted matrix must not be
    // considered proven when independent windows violate the same dynamics.
    const Mat3 validationBias = Quat::fromEulerXYZ(
        8.0f * MATH_DEG_TO_RAD, 0.0f, 0.0f).toRotationMatrix();
    for (auto& in : intervals) {
        if ((in.windowId & 1u) != 0u) {
            in.mag1Raw = validationBias * in.mag1Raw;
        }
    }

    MagAxisAlignmentSolvePolicy policy;
    MagAxisAlignmentResult result;
    CHECK(ctx, !solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), Mat3::identity(), 3u, 8u,
        &coarse, policy, result));
    CHECK(ctx, !result.valid);
    CHECK(ctx, !result.validationPassed || !result.validationWinnerMatchesTraining ||
               result.validationScore - result.trainingScore > policy.maxValidationGeneralizationGap);
}

static void testDynamicAxisSolverKeepsPureRotationConstraint(TestContext& ctx) {
    const Mat3 reflection(-1,0,0, 0,1,0, 0,0,1);
    const Mat3 shear(1,0.1f,0, 0,1,0, 0,0,1);
    const Mat3 smallRotation = Quat::fromEulerXYZ(
        1.0f * MATH_DEG_TO_RAD,
       -2.0f * MATH_DEG_TO_RAD,
        0.5f * MATH_DEG_TO_RAD).toRotationMatrix();
    CHECK(ctx, !MagAxisAlignmentCollector::properRotation(reflection));
    CHECK(ctx, !MagAxisAlignmentCollector::properRotation(shear));
    CHECK(ctx, MagAxisAlignmentCollector::properRotation(smallRotation));
    CHECK_NEAR(ctx, smallRotation.determinant(), 1.0f, 1.0e-4f);
}

int main() {
    TestContext ctx;
    testFieldReliabilityFailsClosedAcrossChangedEnvironment(ctx);
    testModerateStationaryHeadingJumpStaysFailClosed(ctx);
    testStationaryJumpAtWindowBoundaryStillLatches(ctx);
    testSlowStationaryYawDriftRemainsCorrectable(ctx);
    testFastButPlausibleYawDriftDoesNotSelfLatch(ctx);
    testMaximumNormalYawCorrectionDoesNotSelfLatch(ctx);
    testRealRotationDoesNotTriggerStationaryJumpLatch(ctx);
    testVerySlowPhysicalRotationDoesNotTriggerJumpLatch(ctx);
    testFilteredHeadingRateAllowsNoisySixtyHertzReacquisition(ctx);
    testOnlyProperSignedPermutationsAreAccepted(ctx);
    testAxisCollectorRejectsStaleGyroPair(ctx);
    testSixtyHertzCollectionSpansIndependentWindows(ctx);
    testDynamicAxisSolverRefinesMechanicalMisalignment(ctx);
    testSolverConfidenceIsRateNormalized(ctx);
    testHoldoutRejectsTrainingOnlyFit(ctx);
    testDynamicAxisSolverKeepsPureRotationConstraint(ctx);
    return ctx.finish("test_mag_heading_reliability");
}
