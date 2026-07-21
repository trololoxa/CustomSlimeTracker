#include "test_common.hpp"

#include <string>

#include "runtime/tracking_state_controller.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/mag_runtime.hpp"
#include "sensor/mag_yaw_correction.hpp"

using namespace tracker;

static TrackingStateInputs readyInputs() {
    TrackingStateInputs in;
    in.accelCalValid = true;
    in.gyroBiasValid = true;
    in.ahrsInitialized = true;
    return in;
}

static void testNominalStates(TestContext& ctx) {
    TrackingStateController c;
    TrackingStateInputs in = readyInputs();
    CHECK(ctx, c.evaluateState(in) == TrackingStateId::Tracking6Dof);
    CHECK(ctx, c.stateName(in) == trackingStateIdName(TrackingStateId::Tracking6Dof));

    in.magHeadingReferenceValid = true;
    in.magYawApplied = true;
    CHECK(ctx, c.evaluateState(in) == TrackingStateId::Tracking6DofMagYaw);
}

static void testPriorityOrder(TestContext& ctx) {
    TrackingStateController c;
    TrackingStateInputs in = readyInputs();

    in.accelCalValid = false;
    in.qualityFlags = imu_quality_flags::TIMESTAMP_LARGE_GAP;
    CHECK(ctx, c.evaluateState(in) == TrackingStateId::CalibrationRequired);

    in = readyInputs();
    in.qualityFlags = imu_quality_flags::GYRO_SATURATED | imu_quality_flags::ACCEL_NORM_OUTLIER;
    CHECK(ctx, c.evaluateState(in) == TrackingStateId::SensorFault);

    in = readyInputs();
    in.qualityFlags = imu_quality_flags::TIMESTAMP_LARGE_GAP | imu_quality_flags::ACCEL_NORM_OUTLIER;
    CHECK(ctx, c.evaluateState(in) == TrackingStateId::DegradedTiming);

    in = readyInputs();
    in.qualityFlags = imu_quality_flags::ACCEL_NORM_OUTLIER;
    CHECK(ctx, c.evaluateState(in) == TrackingStateId::DegradedAccel);
}

static void testTimestampGapRecoveryPolicy(TestContext& ctx) {
    ImuQualityResult q;
    q.flags = imu_quality_flags::TIMESTAMP_HARDWARE;
    q.dtUs = 1042;
    CHECK(ctx, !trackingTimestampGapRequiresRecovery(q, 0.020f));

    q.flags |= imu_quality_flags::TIMESTAMP_LARGE_GAP |
               imu_quality_flags::SAMPLE_DROPPED_BEFORE;
    q.estimatedDroppedBefore = 1;
    q.dtUs = 2084;
    CHECK(ctx, !trackingTimestampGapRequiresRecovery(q, 0.020f));

    q.estimatedDroppedBefore = 7;
    q.dtUs = 8336;
    CHECK(ctx, !trackingTimestampGapRequiresRecovery(q, 0.020f));

    q.estimatedDroppedBefore = 19;
    q.dtUs = 20000;
    CHECK(ctx, !trackingTimestampGapRequiresRecovery(q, 0.020f));

    q.dtUs = 20001;
    CHECK(ctx, trackingTimestampGapRequiresRecovery(q, 0.020f));
    CHECK(ctx, trackingTimestampGapRequiresRecovery(q, 0.0f));
}

static void testRecoveryBootstrapPolicy(TestContext& ctx) {
    CHECK(ctx, trackingRecoveryNeedsAhrsBootstrap(true, false));
    CHECK(ctx, !trackingRecoveryNeedsAhrsBootstrap(true, true));
    CHECK(ctx, !trackingRecoveryNeedsAhrsBootstrap(false, false));
}

static void testFifoSoftRecoveryBounds(TestContext& ctx) {
    ImuQualityResult q;
    q.flags = imu_quality_flags::FIFO_OVERRUN | imu_quality_flags::FIFO_FULL;
    q.dtUs = 154000;
    q.estimatedDroppedBefore = 147;
    CHECK(ctx, trackingFifoLossCanUseSoftRecovery(q));

    q.dtUs = 250001;
    CHECK(ctx, !trackingFifoLossCanUseSoftRecovery(q));
    q.dtUs = 154000;
    q.estimatedDroppedBefore = 257;
    CHECK(ctx, !trackingFifoLossCanUseSoftRecovery(q));

    q.estimatedDroppedBefore = 147;
    q.flags |= imu_quality_flags::TIMESTAMP_BACKWARDS;
    CHECK(ctx, !trackingFifoLossCanUseSoftRecovery(q));
}

static void testRecoveryReasonDiagnostics(TestContext& ctx) {
    TrackingStateController c;
    TrackingStateEventSink sink;

    c.enterRecovery(imu_quality_flags::FIFO_FULL, "fifo_recovery_soft", 1000, sink);
    c.enterRecovery(imu_quality_flags::TIMESTAMP_LARGE_GAP, "unreconstructable_dt_gap", 2000, sink);
    c.enterRecovery(imu_quality_flags::FIFO_RECOVERY_REQUESTED, "manual_fifo_reset", 3000, sink);
    c.enterRecovery(imu_quality_flags::FIFO_RECOVERY_REQUESTED, "blocking_wifi_scan", 4000, sink);
    c.enterRecovery(imu_quality_flags::FIFO_RECOVERY_REQUESTED, "imu_fifo_reconfigure", 5000, sink);
    c.enterRecovery(0, "unexpected_reason", 6000, sink);

    const auto s = c.snapshot();
    CHECK(ctx, s.recoveryEnterCount == 1);
    CHECK(ctx, s.recoveryFifoQualityRequests == 1);
    CHECK(ctx, s.recoveryTimestampGapRequests == 1);
    CHECK(ctx, s.recoveryManualResetRequests == 1);
    CHECK(ctx, s.recoveryBlockingOperationRequests == 1);
    CHECK(ctx, s.recoveryRuntimeReconfigureRequests == 1);
    CHECK(ctx, s.recoveryOtherRequests == 1);
    CHECK(ctx, s.recoveryLastReason == TrackingRecoveryReasonId::Other);
    CHECK(ctx, std::string(trackingRecoveryReasonName(TrackingRecoveryReasonId::BlockingOperation)) == "blocking_operation");
}

static void testRecoveryOverridesDegradation(TestContext& ctx) {
    TrackingStateController c;
    TrackingStateEventSink sink;
    c.enterRecovery(imu_quality_flags::FIFO_RECOVERY_REQUESTED, "test", 1000, sink);

    TrackingStateInputs in = readyInputs();
    in.qualityFlags = imu_quality_flags::GYRO_SATURATED;
    CHECK(ctx, c.evaluateState(in) == TrackingStateId::Recovering);
    CHECK(ctx, c.recoveryEnterCount() == 1);
}

static void testMagDegradedState(TestContext& ctx) {
    TrackingStateController c;
    TrackingStateInputs in = readyInputs();
    in.magRuntimeEnabled = true;
    in.magSampleSeen = true;
    in.magTrusted = false;
    in.magRejectFlags = MAG_REJECT_NORM_TOO_HIGH;
    CHECK(ctx, c.evaluateState(in) == TrackingStateId::DegradedMag);

    in = readyInputs();
    in.magRuntimeEnabled = true;
    in.magYawControllerEnabled = true;
    in.magYawRejectFlags = MAG_YAW_REJECT_INNOVATION_TOO_LARGE;
    CHECK(ctx, c.evaluateState(in) == TrackingStateId::DegradedMag);
}


static ImuQualityResult stableRecoveryQuality();

struct PrepareProbe {
    uint32_t calls = 0;
    uint64_t timestampUs = 0;
    bool rebase = false;
};

static void prepareProbeCallback(const char*, uint64_t timestampUs, bool rebase, void* user) {
    PrepareProbe* probe = static_cast<PrepareProbe*>(user);
    probe->calls++;
    probe->timestampUs = timestampUs;
    probe->rebase = rebase;
}

static void testRecoveryBeforeFirstOrientationUsesStartupConvergence(TestContext& ctx) {
    TrackingStateController c;
    PrepareProbe probe;
    TrackingStateEventSink sink;
    sink.hasRecoverableOrientation = false;
    sink.prepareRecovery = prepareProbeCallback;
    sink.prepareRecoveryUser = &probe;

    c.enterRecovery(imu_quality_flags::FIFO_RECOVERY_REQUESTED,
                    "imu_fifo_reconfigure",
                    1000,
                    sink);

    CHECK(ctx, !c.recoveryActive());
    CHECK(ctx, !c.softRecoveryActive());
    CHECK(ctx, c.recoveryEnterCount() == 0);
    CHECK(ctx, c.recoveryBootstrapBypassCount() == 1);
    CHECK(ctx, c.recoveryLastReason() == TrackingRecoveryReasonId::RuntimeReconfigure);
    CHECK(ctx, probe.calls == 1);
    CHECK(ctx, !probe.rebase);

    TrackingStateInputs in = readyInputs();
    in.ahrsInitialized = false;
    CHECK(ctx, c.evaluateState(in) == TrackingStateId::StartupConvergence);
}

static void testFifoLossUsesContinuousSoftRecovery(TestContext& ctx) {
    TrackingStateController c;
    PrepareProbe probe;
    TrackingStateEventSink sink;
    sink.prepareRecovery = prepareProbeCallback;
    sink.prepareRecoveryUser = &probe;

    const uint32_t flags = imu_quality_flags::FIFO_OVERRUN |
                           imu_quality_flags::FIFO_FULL |
                           imu_quality_flags::FIFO_RECOVERY_REQUESTED;
    c.enterRecovery(flags, "fifo_recovery_soft", 1000, sink);
    CHECK(ctx, !c.recoveryActive());
    CHECK(ctx, c.softRecoveryActive());
    CHECK(ctx, c.softRecoveryEnterCount() == 1);
    CHECK(ctx, c.recoveryEnterCount() == 0);
    CHECK(ctx, probe.calls == 1);
    CHECK(ctx, probe.timestampUs == 1000);
    CHECK(ctx, probe.rebase);

    TrackingStateInputs in = readyInputs();
    CHECK(ctx, c.evaluateState(in) == TrackingStateId::DegradedTiming);

    ImuQualityResult q = stableRecoveryQuality();
    // Soft recovery is explicitly allowed to finish during motion. It is a
    // timestamp-stream validation window, not a gravity/stillness capture.
    for (uint32_t i = 0; i < 31; ++i) {
        c.updateRecovery(q,
                         Vec3(120.0f * MATH_DEG_TO_RAD, 0.0f, 0.0f),
                         Vec3(0.3f, 0.0f, 1.0f),
                         2000 + i * 1000,
                         true,
                         sink);
    }
    CHECK(ctx, c.softRecoveryActive());
    CHECK(ctx, c.softRecoveryGoodSamples() == 31);

    c.updateRecovery(q, Vec3::zero(), Vec3::unitZ(), 33000, true, sink);
    CHECK(ctx, !c.softRecoveryActive());
    CHECK(ctx, c.softRecoveryCompleteCount() == 1);
    CHECK(ctx, c.evaluateState(in) == TrackingStateId::Tracking6Dof);
}

static void testCorruptFifoTimingStillUsesStrictRecovery(TestContext& ctx) {
    TrackingStateController c;
    TrackingStateEventSink sink;
    const uint32_t flags = imu_quality_flags::FIFO_FULL |
                           imu_quality_flags::TIMESTAMP_BACKWARDS |
                           imu_quality_flags::FIFO_RECOVERY_REQUESTED;
    c.enterRecovery(flags, "fifo_recovery_soft", 1000, sink);
    CHECK(ctx, c.recoveryActive());
    CHECK(ctx, !c.softRecoveryActive());
    CHECK(ctx, c.recoveryEnterCount() == 1);
    CHECK(ctx, c.softRecoveryEnterCount() == 0);
}

struct ReacquireProbe {
    uint32_t calls = 0;
    Vec3 accel = Vec3::zero();
    uint64_t timestampUs = 0;
    bool result = true;
};

static bool reacquireProbeCallback(const Vec3& accelG, uint64_t timestampUs, void* user) {
    ReacquireProbe* probe = static_cast<ReacquireProbe*>(user);
    probe->calls++;
    probe->accel = accelG;
    probe->timestampUs = timestampUs;
    return probe->result;
}

static ImuQualityResult stableRecoveryQuality() {
    ImuQualityResult q;
    q.dtUs = 1000;
    q.accelNormG = 1.0f;
    q.accelNormValid = true;
    q.accelConfidence = 1.0f;
    q.shouldUpdateAhrs = true;
    q.shouldUseAccelCorrection = true;
    return q;
}

static void testRecoveryRequiresStableTiltReacquisition(TestContext& ctx) {
    TrackingStateController c;
    c.setStableSamplesRequired(3);

    ReacquireProbe probe;
    TrackingStateEventSink sink;
    sink.reacquireTilt = reacquireProbeCallback;
    sink.reacquireTiltUser = &probe;

    c.enterRecovery(imu_quality_flags::TIMESTAMP_LARGE_GAP, "gap", 1000, sink);
    ImuQualityResult q = stableRecoveryQuality();

    c.updateRecovery(q, Vec3::zero(), Vec3(0.0f, 0.0f, 0.99f), 2000, true, sink);
    c.updateRecovery(q, Vec3::zero(), Vec3(0.0f, 0.0f, 1.01f), 3000, true, sink);
    CHECK(ctx, c.recoveryActive());
    CHECK(ctx, c.recoveryStableSamples() == 2);

    c.updateRecovery(q, Vec3(4.0f * MATH_DEG_TO_RAD, 0.0f, 0.0f), Vec3::unitZ(), 4000, true, sink);
    CHECK(ctx, c.recoveryStableSamples() == 0);
    CHECK(ctx, probe.calls == 0);

    c.updateRecovery(q, Vec3::zero(), Vec3(0.01f, 0.0f, 1.00f), 5000, true, sink);
    c.updateRecovery(q, Vec3::zero(), Vec3(-0.01f, 0.0f, 1.00f), 6000, true, sink);
    c.updateRecovery(q, Vec3::zero(), Vec3(0.00f, 0.0f, 1.00f), 7000, true, sink);

    CHECK(ctx, !c.recoveryActive());
    CHECK(ctx, probe.calls == 1);
    CHECK(ctx, probe.timestampUs == 7000);
    CHECK_NEAR(ctx, probe.accel.x, 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, probe.accel.z, 1.0f, 1.0e-6f);
    CHECK(ctx, c.recoveryTiltReacquireCount() == 1);
}

static void testRecoveryDoesNotExitWithoutReacquireConsumer(TestContext& ctx) {
    TrackingStateController c;
    c.setStableSamplesRequired(1);
    TrackingStateEventSink sink;
    c.enterRecovery(imu_quality_flags::TIMESTAMP_LARGE_GAP, "gap", 1000, sink);
    c.updateRecovery(stableRecoveryQuality(), Vec3::zero(), Vec3::unitZ(), 2000, true, sink);
    CHECK(ctx, c.recoveryActive());
    CHECK(ctx, c.recoveryTiltReacquireCount() == 0);
}

static void testRecoveryRejectsIncoherentGravityWindow(TestContext& ctx) {
    TrackingStateController c;
    c.setStableSamplesRequired(2);

    ReacquireProbe probe;
    TrackingStateEventSink sink;
    sink.reacquireTilt = reacquireProbeCallback;
    sink.reacquireTiltUser = &probe;
    c.enterRecovery(imu_quality_flags::TIMESTAMP_LARGE_GAP, "gap", 1000, sink);

    const ImuQualityResult q = stableRecoveryQuality();
    c.updateRecovery(q, Vec3::zero(), Vec3(0.5f, 0.0f, 0.8660254f), 2000, true, sink);
    c.updateRecovery(q, Vec3::zero(), Vec3(-0.5f, 0.0f, 0.8660254f), 3000, true, sink);
    CHECK(ctx, c.recoveryActive());
    CHECK(ctx, c.recoveryStableSamples() == 0);
    CHECK(ctx, probe.calls == 0);

    c.updateRecovery(q, Vec3::zero(), Vec3::unitZ(), 4000, true, sink);
    c.updateRecovery(q, Vec3::zero(), Vec3::unitZ(), 5000, true, sink);
    CHECK(ctx, !c.recoveryActive());
    CHECK(ctx, probe.calls == 1);
}


static void testRecoveryToleratesIsolatedQualityRejects(TestContext& ctx) {
    TrackingStateController c;
    c.setStableSamplesRequired(3);

    ReacquireProbe probe;
    TrackingStateEventSink sink;
    sink.reacquireTilt = reacquireProbeCallback;
    sink.reacquireTiltUser = &probe;
    c.enterRecovery(imu_quality_flags::FIFO_RECOVERY_REQUESTED, "gap", 1000, sink);

    ImuQualityResult q = stableRecoveryQuality();
    c.updateRecovery(q, Vec3::zero(), Vec3::unitZ(), 2000, true, sink);
    CHECK(ctx, c.recoveryStableSamples() == 1);

    ImuQualityResult transient = q;
    transient.shouldUpdateAhrs = false;
    transient.flags = imu_quality_flags::TIMESTAMP_NON_MONOTONIC;
    c.updateRecovery(transient, Vec3::zero(), Vec3::unitZ(), 3000, false, sink);
    CHECK(ctx, c.recoveryStableSamples() == 1);
    CHECK(ctx, c.recoveryRejectStreak() == 1);

    ImuQualityResult shortGap = q;
    shortGap.flags = imu_quality_flags::TIMESTAMP_LARGE_GAP |
                     imu_quality_flags::SAMPLE_DROPPED_BEFORE;
    shortGap.dtUs = 2084;
    c.updateRecovery(shortGap, Vec3::zero(), Vec3::unitZ(), 4000, true, sink);
    CHECK(ctx, c.recoveryStableSamples() == 2);
    CHECK(ctx, c.recoveryRejectStreak() == 0);

    c.updateRecovery(q, Vec3::zero(), Vec3::unitZ(), 5000, true, sink);
    CHECK(ctx, !c.recoveryActive());
    CHECK(ctx, probe.calls == 1);
}

static void testRecoveryRejectStreakEventuallyRestartsWindow(TestContext& ctx) {
    TrackingStateController c;
    c.setStableSamplesRequired(3);
    TrackingStateEventSink sink;
    c.enterRecovery(imu_quality_flags::FIFO_RECOVERY_REQUESTED, "gap", 1000, sink);

    ImuQualityResult q = stableRecoveryQuality();
    c.updateRecovery(q, Vec3::zero(), Vec3::unitZ(), 2000, true, sink);
    CHECK(ctx, c.recoveryStableSamples() == 1);

    q.shouldUpdateAhrs = false;
    q.flags = imu_quality_flags::TIMESTAMP_NON_MONOTONIC;
    for (uint32_t i = 0; i < 9; ++i) {
        c.updateRecovery(q, Vec3::zero(), Vec3::unitZ(), 3000 + i * 1000, false, sink);
    }
    CHECK(ctx, c.recoveryStableSamples() == 0);
    CHECK(ctx, c.recoveryRejectStreak() == 9);
}

static bool reacquireAhrsCallback(const Vec3& accelG, uint64_t timestampUs, void* user) {
    return static_cast<Ahrs6Dof*>(user)->reacquireTiltFromAccelPreserveHeading(accelG, timestampUs);
}

static void testStrictRecoveryCanBootstrapUninitializedAhrs(TestContext& ctx) {
    TrackingStateController c;
    c.setStableSamplesRequired(2);

    Ahrs6Dof ahrs;
    TrackingStateEventSink sink;
    sink.reacquireTilt = reacquireAhrsCallback;
    sink.reacquireTiltUser = &ahrs;
    c.enterRecovery(imu_quality_flags::FIFO_RECOVERY_REQUESTED,
                    "manual_fifo_reset",
                    1000,
                    sink);
    CHECK(ctx, c.recoveryActive());
    CHECK(ctx, trackingRecoveryNeedsAhrsBootstrap(c.recoveryActive(), ahrs.initialized()));

    const ImuQualityResult q = stableRecoveryQuality();
    // Ahrs6Dof reports false on the sample that establishes its initial
    // gravity orientation, but it must become initialized.
    CHECK(ctx, !ahrs.update(Vec3::zero(), Vec3::unitZ(), 1.0f, 2000));
    CHECK(ctx, ahrs.initialized());
    c.updateRecovery(q, Vec3::zero(), Vec3::unitZ(), 2000, false, sink);
    CHECK(ctx, c.recoveryStableSamples() == 0);

    CHECK(ctx, !trackingRecoveryNeedsAhrsBootstrap(c.recoveryActive(), ahrs.initialized()));
    CHECK(ctx, ahrs.update(Vec3::zero(), Vec3::zero(), 0.0f, 3000));
    c.updateRecovery(q, Vec3::zero(), Vec3::unitZ(), 3000, true, sink);
    CHECK(ctx, c.recoveryStableSamples() == 1);

    CHECK(ctx, ahrs.update(Vec3::zero(), Vec3::zero(), 0.0f, 4000));
    c.updateRecovery(q, Vec3::zero(), Vec3::unitZ(), 4000, true, sink);
    CHECK(ctx, !c.recoveryActive());
    CHECK(ctx, c.recoveryTiltReacquireCount() == 1);
}

static void testRecoveryKeepsPostGapGyroAndRestoresTilt(TestContext& ctx) {
    TrackingStateController c;
    c.setStableSamplesRequired(4);

    Ahrs6Dof ahrs;
    const Quat initial = Quat::fromAxisAngle(Vec3::unitZ(), 32.0f * MATH_DEG_TO_RAD);
    ahrs.reset(initial, 1000);

    TrackingStateEventSink sink;
    sink.reacquireTilt = reacquireAhrsCallback;
    sink.reacquireTiltUser = &ahrs;
    c.enterRecovery(imu_quality_flags::TIMESTAMP_LARGE_GAP, "gap", 1000, sink);

    // This is the same gyro-only prediction performed by the runtime while
    // network output is blocked. The resulting post-gap motion must survive
    // the later gravity reacquisition.
    CHECK(ctx, ahrs.update(Vec3(0.0f, 0.0f, 20.0f * MATH_DEG_TO_RAD), Vec3::zero(), 0.0f, 11000));
    const Quat predictedAfterGyro = ahrs.quaternion();
    const Quat tiltCorrection = Quat::fromAxisAngle(Vec3::unitX(), 24.0f * MATH_DEG_TO_RAD);
    const Quat expected = (tiltCorrection * predictedAfterGyro).normalized();
    const Vec3 measuredAccel = expected.conjugated().rotate(Vec3::unitZ());

    const ImuQualityResult q = stableRecoveryQuality();
    for (uint64_t ts = 12000; ts <= 15000; ts += 1000) {
        c.updateRecovery(q, Vec3::zero(), measuredAccel, ts, true, sink);
    }

    CHECK(ctx, !c.recoveryActive());
    CHECK(ctx, c.recoveryTiltReacquireCount() == 1);
    const Quat recovered = ahrs.quaternion();
    CHECK_NEAR(ctx, dot(recovered.rotate(Vec3::unitX()), expected.rotate(Vec3::unitX())), 1.0f, 2.0e-4f);
    CHECK_NEAR(ctx, dot(recovered.rotate(Vec3::unitY()), expected.rotate(Vec3::unitY())), 1.0f, 2.0e-4f);
    CHECK_NEAR(ctx, dot(recovered.rotate(measuredAccel), Vec3::unitZ()), 1.0f, 2.0e-4f);
    CHECK(ctx, ahrs.stats().lastIntegratedTimestampUs == 15000);
}

static void testCompatibilityWrapper(TestContext& ctx) {
    TrackingStateController c;
    CHECK(ctx, c.stateName(false, true, false, true, false, false) == trackingStateIdName(TrackingStateId::CalibrationRequired));
    CHECK(ctx, c.stateName(true, true, true, true, false, false) == trackingStateIdName(TrackingStateId::DegradedTiming));
    CHECK(ctx, c.stateName(true, true, false, false, false, false) == trackingStateIdName(TrackingStateId::StartupConvergence));
    CHECK(ctx, c.stateName(true, true, false, true, true, true) == trackingStateIdName(TrackingStateId::Tracking6DofMagYaw));
}

int main() {
    TestContext ctx;
    testNominalStates(ctx);
    testPriorityOrder(ctx);
    testTimestampGapRecoveryPolicy(ctx);
    testRecoveryBootstrapPolicy(ctx);
    testFifoSoftRecoveryBounds(ctx);
    testRecoveryReasonDiagnostics(ctx);
    testRecoveryBeforeFirstOrientationUsesStartupConvergence(ctx);
    testFifoLossUsesContinuousSoftRecovery(ctx);
    testCorruptFifoTimingStillUsesStrictRecovery(ctx);
    testRecoveryOverridesDegradation(ctx);
    testMagDegradedState(ctx);
    testCompatibilityWrapper(ctx);
    testRecoveryRequiresStableTiltReacquisition(ctx);
    testRecoveryDoesNotExitWithoutReacquireConsumer(ctx);
    testRecoveryRejectsIncoherentGravityWindow(ctx);
    testRecoveryToleratesIsolatedQualityRejects(ctx);
    testRecoveryRejectStreakEventuallyRestartsWindow(ctx);
    testStrictRecoveryCanBootstrapUninitializedAhrs(ctx);
    testRecoveryKeepsPostGapGyroAndRestoresTilt(ctx);
    return ctx.finish("test_tracking_state_controller");
}
