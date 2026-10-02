#pragma once
// alignment collection scenarios; single suite translation unit.
static void testIntervalBuilderUsesTimestampCoherentGyroEndpoints(TestContext& ctx) {
    MagAxisAlignmentInterval interval;
    MagAxisIntervalBuildFailure failure = MagAxisIntervalBuildFailure::InvalidInput;
    const Vec3 gyro0(1.0f, 0.0f, 0.0f);
    const Vec3 gyro1(0.0f, 1.0f, 0.0f);
    CHECK(ctx, buildMagAxisAlignmentInterval(
        gyro0, gyro1,
        Vec3(1.0f, 0.0f, 0.0f), Vec3(0.99f, -0.01f, 0.0f),
        0.02f, 7u, interval, &failure));
    CHECK(ctx, failure == MagAxisIntervalBuildFailure::None);
    CHECK_NEAR(ctx, interval.gyroSensorRadS.x, 0.5f, 1.0e-6f);
    CHECK_NEAR(ctx, interval.gyroSensorRadS.y, 0.5f, 1.0e-6f);
    CHECK_NEAR(ctx, interval.gyroSensorRadS.z, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, interval.dtS, 0.02f, 1.0e-7f);
    CHECK(ctx, interval.windowId == 7u);

    CHECK(ctx, !buildMagAxisAlignmentInterval(
        gyro0, gyro1, Vec3(1,0,0), Vec3(0,1,0),
        0.001f, 0u, interval, &failure));
    CHECK(ctx, failure == MagAxisIntervalBuildFailure::InvalidTiming);

    CHECK(ctx, !buildMagAxisAlignmentInterval(
        Vec3::zero(), Vec3::zero(), Vec3(1,0,0), Vec3(0,1,0),
        0.02f, 0u, interval, &failure));
    CHECK(ctx, failure == MagAxisIntervalBuildFailure::MotionOutOfRange);

    CHECK(ctx, !buildMagAxisAlignmentInterval(
        Vec3(std::numeric_limits<float>::quiet_NaN(), 0, 0), gyro1,
        Vec3(1,0,0), Vec3(0,1,0), 0.02f, 0u, interval, &failure));
    CHECK(ctx, failure == MagAxisIntervalBuildFailure::InvalidInput);
}

static void makeAlignmentDatasetForTransform(
    const Mat3& magToImu,
    MagAxisAlignmentInterval (&intervals)[64]) {
    const Mat3 magFromImu = magToImu.transposed();
    Vec3 mImu = Vec3(0.45f, 0.20f, 0.87f).normalized();
    for (uint16_t i = 0; i < 64u; ++i) {
        const Vec3 axis = (i % 6u == 0u) ? Vec3(1.0f, 0.20f, 0.10f).normalized()
                        : (i % 6u == 1u) ? Vec3(0.10f, 1.0f, 0.30f).normalized()
                        : (i % 6u == 2u) ? Vec3(0.25f, -0.10f, 1.0f).normalized()
                        : (i % 6u == 3u) ? Vec3(-0.70f, 0.50f, 0.50f).normalized()
                        : (i % 6u == 4u) ? Vec3(0.55f, 0.70f, -0.35f).normalized()
                                         : Vec3(-0.40f, -0.25f, 0.88f).normalized();
        const Vec3 gyro = axis * ((70.0f + static_cast<float>(i % 5u) * 8.0f) * MATH_DEG_TO_RAD);
        const float dt = 0.025f;
        const Vec3 nextImu = Quat::fromRotationVector(gyro * (-dt)).rotate(mImu).normalized();
        intervals[i] = MagAxisAlignmentInterval{
            gyro,
            magFromImu * mImu,
            magFromImu * nextImu,
            dt,
            static_cast<uint16_t>(i / 8u)};
        mImu = nextImu;
    }
}

static void testAllProperSignedPermutationMountingsConverge(TestContext& ctx) {
    static constexpr uint8_t permutations[6][3] = {
        {0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}
    };
    const Mat3 residual = Quat::fromEulerXYZ(
        1.3f * MATH_DEG_TO_RAD,
       -0.9f * MATH_DEG_TO_RAD,
        1.7f * MATH_DEG_TO_RAD).toRotationMatrix();
    uint32_t solved = 0u;
    for (const auto& axes : permutations) {
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                for (int sz = -1; sz <= 1; sz += 2) {
                    const Mat3 coarse = MagAxisAlignmentCollector::signedPermutation(
                        axes[0], static_cast<float>(sx),
                        axes[1], static_cast<float>(sy),
                        axes[2], static_cast<float>(sz));
                    if (!MagAxisAlignmentCollector::properRotation(coarse)) continue;
                    const Mat3 expected = residual * coarse;
                    MagAxisAlignmentInterval intervals[64];
                    makeAlignmentDatasetForTransform(expected, intervals);
                    MagAxisAlignmentSolvePolicy policy;
                    MagAxisAlignmentResult result;
                    CHECK(ctx, solveMagAxisAlignmentDataset(
                        intervals, 64u, Vec3::zero(), Mat3::identity(),
                        nullptr, policy, result));
                    CHECK(ctx, result.valid);
                    CHECK(ctx, result.validationPassed);
                    CHECK(ctx, result.validationWinnerMatchesTraining);
                    CHECK(ctx, MagAxisAlignmentCollector::properRotation(result.magToImu));
                    CHECK(ctx, magAxisRotationDifferenceDeg(result.magToImu, expected) < 1.0f);
                    solved++;
                }
            }
        }
    }
    CHECK(ctx, solved == 24u);
}

static void testReflectionAmbiguityRequiresRightHandedDriverContract(TestContext& ctx) {
    const Mat3 reflected(-1,0,0, 0,1,0, 0,0,1);
    MagAxisAlignmentInterval intervals[64];
    makeAlignmentDatasetForTransform(reflected, intervals);
    MagAxisAlignmentSolvePolicy policy;
    MagAxisAlignmentResult result;
    CHECK(ctx, solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), Mat3::identity(),
        nullptr, policy, result));
    CHECK(ctx, result.valid);
    CHECK(ctx, !MagAxisAlignmentCollector::properRotation(reflected));

    // Dynamic vector kinematics cannot distinguish F from -F because both m
    // and -m obey the same dm/dt equation. If a driver silently emits a
    // left-handed frame F, the SO(3)-only solver finds -F, a proper rotation
    // with globally inverted magnetic polarity. Therefore reflections must be
    // corrected in the driver; calibration must never persist det=-1.
    const Mat3 polarityEquivalent = reflected * -1.0f;
    CHECK(ctx, MagAxisAlignmentCollector::properRotation(polarityEquivalent));
    CHECK(ctx, magAxisRotationDifferenceDeg(result.magToImu, polarityEquivalent) < 1.0f);
    CHECK(ctx, MagAxisAlignmentCollector::properRotation(result.magToImu));
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
    mag.calibratedMagFrame = mag.raw;
    mag.rawNorm = 1.0f;
    mag.calibratedNorm = 1.0f;
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
    first.calibratedMagFrame = m;
    first.rawNorm = 1.0f;
    first.calibratedNorm = 1.0f;
    first.seq = 1u;
    first.t_us = tUs;
    first.receivedMs = ms;
    collector.observe(Vec3::zero(), tUs, first, true);

    for (uint32_t i = 0; i < 480u; ++i) {
        const Vec3 gyro = ((i / 60u) & 1u) == 0u ? Vec3(0.8f, 0.1f, 0.0f)
                                                  : Vec3(0.0f, 0.9f, 0.1f);
        const float dt = 0.017f;
        m = (m - cross(gyro, m) * dt).normalized();
        tUs += 17000u;
        ms += 17u;
        MagProcessedSample mag;
        mag.valid = mag.trusted = true;
        mag.raw = m;
        mag.calibratedMagFrame = m;
        mag.rawNorm = 1.0f;
        mag.calibratedNorm = 1.0f;
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

static void testRuntimeCollectorUsesSensorTimeAcrossProcessingBacklog(TestContext& ctx) {
    MagAxisAlignmentCollector collector;
    Vec3 m = Vec3(0.45f, 0.2f, 0.87f).normalized();
    uint64_t tUs = 100000u;
    constexpr uint32_t kCollapsedProcessingMs = 500u;

    MagProcessedSample first;
    first.valid = first.trusted = true;
    first.raw = m;
    first.calibratedMagFrame = m;
    first.rawNorm = 1.0f;
    first.calibratedNorm = 1.0f;
    first.seq = 1u;
    first.t_us = tUs;
    first.receivedMs = kCollapsedProcessingMs;
    collector.observe(Vec3::zero(), tUs, first, true);

    for (uint32_t i = 0; i < 480u; ++i) {
        const Vec3 gyro = ((i / 60u) & 1u) == 0u ? Vec3(0.8f, 0.1f, 0.0f)
                                                  : Vec3(0.0f, 0.9f, 0.1f);
        constexpr float dt = 0.017f;
        m = Quat::fromRotationVector(gyro * (-dt)).rotate(m).normalized();
        tUs += 17000u;
        MagProcessedSample mag;
        mag.valid = mag.trusted = true;
        mag.raw = m;
        mag.calibratedMagFrame = m;
        mag.rawNorm = 1.0f;
        mag.calibratedNorm = 1.0f;
        mag.seq = i + 2u;
        mag.t_us = tUs;
        // Model a queued FIFO burst drained inside one millisecond. Runtime
        // cadence/window evidence must remain tied to sensor time, not this
        // collapsed processing-time stamp.
        mag.receivedMs = kCollapsedProcessingMs;
        collector.observe(gyro, tUs, mag, true);
    }

    CHECK(ctx, collector.intervalCount() >= MagAxisAlignmentCollector::kTargetIntervals);
    CHECK(ctx, collector.independentWindows() >= 4u);
    CHECK(ctx, collector.readyToSolve());
    CHECK(ctx, collector.stats().intervalsSkippedCadence > 0u);
}
