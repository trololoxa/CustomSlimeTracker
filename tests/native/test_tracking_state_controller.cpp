#include "test_common.hpp"

#include "runtime/tracking_state_controller.hpp"
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
    testRecoveryOverridesDegradation(ctx);
    testMagDegradedState(ctx);
    testCompatibilityWrapper(ctx);
    return ctx.finish("test_tracking_state_controller");
}
