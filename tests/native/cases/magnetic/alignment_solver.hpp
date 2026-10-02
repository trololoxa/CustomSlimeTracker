#pragma once
// alignment solver scenarios; single suite translation unit.
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
    policy.minExcitedAxes = 2u;
    policy.minTrainingWindows = 2u;
    policy.minValidationWindows = 2u;
    MagAxisAlignmentResult result;
    CHECK(ctx, solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), Mat3::identity(),
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
            intervals, 64u, Vec3::zero(), Mat3::identity(),
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


static void testSameCoarseWinnerFallsBackWhenRefinementsDisagree(TestContext& ctx) {
    const Mat3 coarse(0,-1,0, 1,0,0, 0,0,1);
    const Mat3 inverse = coarse.transposed();
    const Mat3 validationFrameBias = Quat::fromEulerXYZ(
        2.2f * MATH_DEG_TO_RAD, 0.0f, 0.0f).toRotationMatrix();
    const Mat3 validationRawBias = validationFrameBias.transposed();

    MagAxisAlignmentInterval intervals[64];
    Vec3 m = Vec3(0.45f, 0.20f, 0.87f).normalized();
    for (uint16_t i = 0; i < 64u; ++i) {
        const Vec3 axis = (i % 4u == 0u) ? Vec3(1.0f, 0.2f, 0.1f).normalized()
                        : (i % 4u == 1u) ? Vec3(0.1f, 1.0f, 0.3f).normalized()
                        : (i % 4u == 2u) ? Vec3(0.25f, -0.1f, 1.0f).normalized()
                                         : Vec3(-0.7f, 0.5f, 0.5f).normalized();
        const Vec3 gyro = axis * (90.0f * MATH_DEG_TO_RAD);
        const float dt = 0.025f;
        const Vec3 next = Quat::fromRotationVector(gyro * (-dt)).rotate(m).normalized();
        Vec3 raw0 = inverse * m;
        Vec3 raw1 = inverse * next;
        const uint16_t windowId = static_cast<uint16_t>(i / 8u);
        if ((windowId & 1u) != 0u) {
            raw0 = validationRawBias * raw0;
            raw1 = validationRawBias * raw1;
        }
        intervals[i] = MagAxisAlignmentInterval{gyro, raw0, raw1, dt, windowId};
        m = next;
    }

    MagAxisAlignmentSolvePolicy policy;
    MagAxisAlignmentResult result;
    CHECK(ctx, solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), Mat3::identity(),
        nullptr, policy, result));
    CHECK(ctx, result.valid);
    CHECK(ctx, result.coarseWinnerMatchesTraining);
    CHECK(ctx, !result.continuousRefinementAgreement);
    CHECK(ctx, result.coarseConsensusFallbackUsed);
    CHECK(ctx, result.validationWinnerMatchesTraining);
    CHECK(ctx, result.validationPassed);
    CHECK(ctx, !result.refined);
    CHECK(ctx, result.trainingValidationRotationDifferenceDeg >
               policy.maxTrainingValidationRotationDifferenceDeg);
    CHECK(ctx, magAxisRotationDifferenceDeg(result.magToImu, coarse) < 0.1f);
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
        intervals, 64u, Vec3::zero(), Mat3::identity(),
        &coarse, policy, result));
    CHECK(ctx, !result.valid);
    CHECK(ctx, !result.validationPassed || !result.validationWinnerMatchesTraining ||
               result.validationScore - result.trainingScore > policy.maxValidationGeneralizationGap);
}

static void testGuidedAxisReservoirPreservesLateAxesAndPartitions(TestContext& ctx) {
    MagAxisIntervalReservoir<60> reservoir;
    uint16_t window = 0u;

    auto feedAxis = [&](uint8_t axis, uint32_t count) {
        for (uint32_t i = 0; i < count; ++i) {
            Vec3 gyro = Vec3::zero();
            if (axis == 0u) gyro.x = 1.0f;
            else if (axis == 1u) gyro.y = 1.0f;
            else gyro.z = 1.0f;
            const float phase = static_cast<float>(i % 360u) * MATH_DEG_TO_RAD;
            const Vec3 m0(std::cos(phase), std::sin(phase), 0.25f);
            const Vec3 m1(std::cos(phase + 0.02f), std::sin(phase + 0.02f), 0.25f);
            reservoir.consider(MagAxisAlignmentInterval{
                gyro, m0, m1, 0.02f, window});
            if ((i % 8u) == 7u) window++;
        }
    };

    // A first-N collector would freeze in the X segment and discard all later
    // Y/Z evidence. The stratified reservoir must retain every observed axis
    // and both train/validation window parities despite a very long final tail.
    feedAxis(0u, 600u);
    feedAxis(1u, 600u);
    feedAxis(2u, 6000u);

    CHECK(ctx, reservoir.size() == 60u);
    CHECK(ctx, reservoir.seen() == 7200u);
    CHECK(ctx, reservoir.replacements() > 0u);
    CHECK(ctx, reservoir.skipped() > 0u);
    CHECK(ctx, reservoir.excitedAxes() == 3u);
    CHECK(ctx, reservoir.partitionConfirmedAxes() == 3u);
    CHECK(ctx, reservoir.independentWindows() >= 6u);
    uint32_t bruteForceWindows = 0u;
    for (uint16_t i = 0; i < reservoir.size(); ++i) {
        bool first = true;
        for (uint16_t j = 0; j < i; ++j) {
            if (reservoir.data()[j].windowId == reservoir.data()[i].windowId) {
                first = false;
                break;
            }
        }
        if (first) bruteForceWindows++;
    }
    CHECK(ctx, reservoir.independentWindows() == bruteForceWindows);
    for (uint8_t bucket = 0; bucket < 6u; ++bucket) {
        CHECK(ctx, reservoir.bucketCount(bucket) == 10u);
        CHECK(ctx, reservoir.bucketSeen(bucket) > reservoir.bucketCount(bucket));
    }

    reservoir.reset();
    CHECK(ctx, reservoir.size() == 0u);
    CHECK(ctx, reservoir.seen() == 0u);
    CHECK(ctx, reservoir.replacements() == 0u);
    CHECK(ctx, reservoir.skipped() == 0u);
    CHECK(ctx, reservoir.independentWindows() == 0u);
    CHECK(ctx, reservoir.excitedAxes() == 0u);
    CHECK(ctx, reservoir.partitionConfirmedAxes() == 0u);

    auto addPartitionEvidence = [&](uint8_t axis, uint16_t windowId) {
        for (uint8_t i = 0u; i < 8u; ++i) {
            Vec3 gyro = Vec3::zero();
            if (axis == 0u) gyro.x = 1.0f;
            else gyro.y = 1.0f;
            reservoir.consider(MagAxisAlignmentInterval{
                gyro, Vec3(1.0f, 0.0f, 0.2f), Vec3(0.99f, 0.02f, 0.2f), 0.02f, windowId});
        }
    };
    addPartitionEvidence(0u, 0u);
    addPartitionEvidence(1u, 2u);
    CHECK(ctx, reservoir.excitedAxes() == 2u);
    CHECK(ctx, reservoir.partitionConfirmedAxes() == 0u);
    addPartitionEvidence(0u, 1u);
    addPartitionEvidence(1u, 3u);
    CHECK(ctx, reservoir.partitionConfirmedAxes() == 2u);
}

static void testDynamicAxisSolverWithHardSoftAndNearOriginRawData(TestContext& ctx) {
    const Mat3 coarse(0,-1,0, 1,0,0, 0,0,1);
    const Mat3 residual = Quat::fromEulerXYZ(
        1.1f * MATH_DEG_TO_RAD,
       -0.8f * MATH_DEG_TO_RAD,
        1.4f * MATH_DEG_TO_RAD).toRotationMatrix();
    const Mat3 expected = residual * coarse;
    const Mat3 magFromImu = expected.transposed();
    const Vec3 hardIron(500.0f, 0.0f, 0.0f);
    const Mat3 softIron = Quat::fromEulerXYZ(0.15f, -0.21f, 0.27f).toRotationMatrix() *
        Mat3::diagonal(0.21f, 0.19f, 0.20f) *
        Quat::fromEulerXYZ(0.15f, -0.21f, 0.27f).toRotationMatrix().transposed();
    Mat3 softIronInv;
    CHECK(ctx, softIron.inverse(softIronInv));

    MagAxisAlignmentInterval intervals[64];
    Vec3 mImu = Vec3(0.45f, 0.20f, 0.87f).normalized() * 100.0f;
    for (uint16_t i = 0; i < 64u; ++i) {
        const Vec3 axis = (i % 4u == 0u) ? Vec3(1.0f, 0.2f, 0.1f).normalized()
                        : (i % 4u == 1u) ? Vec3(0.1f, 1.0f, 0.3f).normalized()
                        : (i % 4u == 2u) ? Vec3(0.25f, -0.1f, 1.0f).normalized()
                                         : Vec3(-0.7f, 0.5f, 0.5f).normalized();
        const Vec3 gyro = axis * (90.0f * MATH_DEG_TO_RAD);
        const float dt = 0.025f;
        const Vec3 nextImu = Quat::fromRotationVector(gyro * (-dt)).rotate(mImu);
        const Vec3 mag0Cal = magFromImu * mImu;
        const Vec3 mag1Cal = magFromImu * nextImu;
        intervals[i] = MagAxisAlignmentInterval{
            gyro,
            hardIron + softIronInv * mag0Cal,
            hardIron + softIronInv * mag1Cal,
            dt, static_cast<uint16_t>(i / 8u)};
        mImu = nextImu;
    }

    MagAxisAlignmentSolvePolicy policy;
    MagAxisAlignmentResult result;
    CHECK(ctx, solveMagAxisAlignmentDataset(
        intervals, 64u, hardIron, softIron,
        &coarse, policy, result));
    CHECK(ctx, result.valid);
    CHECK(ctx, result.failureReason == MagAxisAlignmentFailureReason::None);
    CHECK(ctx, result.excitedAxes == 3u);
    CHECK(ctx, result.partitionConfirmedAxes >= 2u);
    CHECK(ctx, result.independentWindows == 8u);
    CHECK(ctx, magAxisRotationDifferenceDeg(result.magToImu, expected) < 1.0f);
}

static void testSolverCountsUniqueReservoirWindows(TestContext& ctx) {
    MagAxisAlignmentInterval intervals[64];
    Mat3 coarse;
    Mat3 expected;
    makeRateDataset(90.0f, intervals, coarse, expected);
    for (uint16_t i = 0; i < 64u; ++i) intervals[i].windowId = static_cast<uint16_t>(i & 1u);
    MagAxisAlignmentSolvePolicy policy;
    policy.minIndependentWindows = 4u;
    policy.minExcitedAxes = 1u;
    policy.minTrainingWindows = 2u;
    policy.minValidationWindows = 2u;
    MagAxisAlignmentResult result;
    CHECK(ctx, !solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), Mat3::identity(),
        &coarse, policy, result));
    CHECK(ctx, result.independentWindows == 2u);
    CHECK(ctx, result.failureReason == MagAxisAlignmentFailureReason::InsufficientWindows);
}

static void testSolverRejectsInvalidHardSoftTransform(TestContext& ctx) {
    MagAxisAlignmentInterval intervals[64];
    Mat3 coarse;
    Mat3 expected;
    makeRateDataset(90.0f, intervals, coarse, expected);
    MagAxisAlignmentSolvePolicy policy;
    MagAxisAlignmentResult result;
    const Mat3 reflection(-1,0,0, 0,1,0, 0,0,1);
    CHECK(ctx, !solveMagAxisAlignmentDataset(
        intervals, 64u, Vec3::zero(), reflection,
        &coarse, policy, result));
    CHECK(ctx, result.failureReason == MagAxisAlignmentFailureReason::InvalidCalibration);
}

static void testRuntimeCollectorUsesCalibratedDirectionAndReservoir(TestContext& ctx) {
    MagAxisAlignmentCollector collector;
    Vec3 calibrated = Vec3(0.45f, 0.2f, 0.87f).normalized();
    uint64_t tUs = 100000u;
    uint32_t ms = 100u;
    for (uint32_t i = 0; i < 900u; ++i) {
        const Vec3 gyro = (i < 300u) ? Vec3(0.9f, 0.05f, 0.0f)
                         : (i < 600u) ? Vec3(0.0f, 0.95f, 0.05f)
                                      : Vec3(0.05f, 0.0f, 1.0f);
        const float dt = 0.017f;
        calibrated = Quat::fromRotationVector(gyro * (-dt)).rotate(calibrated).normalized();
        tUs += 17000u;
        ms += 17u;
        MagProcessedSample mag;
        mag.valid = mag.trusted = true;
        mag.raw = calibrated * 500.0f + Vec3(500.0f, 0.0f, 0.0f);
        mag.calibratedMagFrame = calibrated;
        mag.rawNorm = mag.raw.norm();
        mag.calibratedNorm = 1.0f;
        mag.seq = i + 1u;
        mag.t_us = tUs;
        mag.receivedMs = ms;
        collector.observe(gyro, tUs, mag, true);
    }
    CHECK(ctx, collector.intervalCount() == MagAxisAlignmentCollector::kMaxIntervals);
    CHECK(ctx, collector.excitedAxes() == 3u);
    CHECK(ctx, collector.partitionConfirmedAxes() == 3u);
    CHECK(ctx, collector.independentWindows() >= 4u);
    CHECK(ctx, collector.stats().intervalsReservoirReplaced > 0u);
    CHECK(ctx, collector.stats().intervalsReservoirSkipped > 0u);
    CHECK(ctx, collector.stats().intervalsRejectedCapacity == 0u);
}

static void testCalibratedRawOriginRemainsUsable(TestContext& ctx) {
    MagRuntimeProcessor processor;
    MagRuntimeConfig config;
    config.enabled = true;
    config.calibrationValid = true;
    config.axisAlignmentValid = true;
    config.hardIron = Vec3(500.0f, 0.0f, 0.0f);
    config.softIron = Mat3::identity();
    config.magToImu = Mat3::identity();
    config.expectedFieldNorm = 500.0f;
    config.minTrustNorm = 400.0f;
    config.maxTrustNorm = 600.0f;

    Lsm6dsvFifoReader::MagRawSample raw;
    raw.x = raw.y = raw.z = 0;
    raw.seq = 1u;
    raw.t_us = 1000u;
    MagProcessedSample processed;
    CHECK(ctx, processor.process(raw, config, 1u, processed));
    CHECK(ctx, processed.rawNorm == 0.0f);
    CHECK_NEAR(ctx, processed.calibratedNorm, 500.0f, 1.0e-5f);
    CHECK(ctx, (processed.rejectFlags & MAG_REJECT_ZERO_NORM) == 0u);
    CHECK(ctx, processed.trusted);
}

static void testMagRuntimeRetainsValidatedDeviceFrameDecision(TestContext& ctx) {
    MagRuntimeProcessor processor;
    MagRuntimeConfig config;
    config.enabled = true;
    config.calibrationValid = true;
    config.axisAlignmentValid = true;
    config.minTrustNorm = 0.1f;
    config.maxTrustNorm = 1000.0f;
    config.sensorToDeviceValid = true;
    config.sensorToDevice = Quat::fromEulerXYZ(0.0f, 0.0f, 0.5f * MATH_PI).toRotationMatrix();

    Lsm6dsvFifoReader::MagRawSample raw;
    raw.x = 100;
    raw.seq = 1u;
    raw.t_us = 1000u;
    MagProcessedSample processed;
    CHECK(ctx, processor.process(raw, config, 1u, processed));
    CHECK(ctx, processed.sensorToDeviceApplied);
    CHECK_NEAR(ctx, processed.body.x, 0.0f, 1.0e-4f);
    CHECK_NEAR(ctx, processed.body.y, 100.0f, 1.0e-4f);
    const SensorToDeviceFrame acceptedFrame = makeSensorToDeviceFrame(
        config.sensorToDeviceValid, config.sensorToDevice);
    CHECK(ctx, acceptedFrame.enabled);
    const Vec3 recovered = acceptedFrame.inverseApply(processed.body);
    CHECK_NEAR(ctx, recovered.x, 100.0f, 1.0e-4f);
    CHECK_NEAR(ctx, recovered.y, 0.0f, 1.0e-4f);

    config.sensorToDevice = Mat3::diagonal(2.0f, 1.0f, 1.0f);
    CHECK(ctx, processor.process(raw, config, 2u, processed));
    CHECK(ctx, !processed.sensorToDeviceApplied);
    CHECK_NEAR(ctx, processed.body.x, 100.0f, 1.0e-4f);
    CHECK_NEAR(ctx, processed.body.y, 0.0f, 1.0e-4f);
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
