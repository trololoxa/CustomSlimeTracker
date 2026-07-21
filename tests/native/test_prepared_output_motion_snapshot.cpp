#include "test_common.hpp"

#include <limits>

#include "config/tracker_config_runtime.hpp"
#include "runtime/output_runtime.hpp"

using namespace tracker;

static void checkVec(TestContext& ctx, const Vec3& actual, const Vec3& expected, float eps) {
    CHECK_NEAR(ctx, actual.x, expected.x, eps);
    CHECK_NEAR(ctx, actual.y, expected.y, eps);
    CHECK_NEAR(ctx, actual.z, expected.z, eps);
}

static void testCoherentLinearAccelerationAndDerivedWorldFrame(TestContext& ctx) {
    TrackerConfig config;
    config.resetDefaults();
    config.data.accelCal.valid = true;
    config.data.frame.sensorToDeviceValid = true;

    const Quat qWorldFromDevice = Quat::fromAxisAngle(Vec3::unitX(), 35.0f * MATH_DEG_TO_RAD);
    Ahrs6Dof ahrs(config.makeAhrsConfig());
    ahrs.reset(qWorldFromDevice, 1000000);

    const Vec3 linearWorldG(0.20f, -0.15f, 0.30f);
    const Vec3 specificForceWorldG = Vec3::unitZ() + linearWorldG;
    const Vec3 accelDeviceG = qWorldFromDevice.inverseRotate(specificForceWorldG);
    const Vec3 expectedLinearDeviceG = qWorldFromDevice.inverseRotate(linearWorldG);

    ImuQualityResult quality;
    quality.flags = imu_quality_flags::TIMESTAMP_HARDWARE;
    quality.overallConfidence = 0.91f;

    PreparedOutputRuntime prepared;
    prepared.update(config, 77, 1000000, ahrs, quality, accelDeviceG);

    TrackerPreparedOutputSnapshot snapshot;
    CHECK(ctx, prepared.copy(snapshot));
    CHECK(ctx, snapshot.valid);
    CHECK(ctx, snapshot.linearAccelerationValid);
    CHECK(ctx, snapshot.timestampUs == 1000000);
    checkVec(ctx, snapshot.linearAccelerationDeviceG, expectedLinearDeviceG, 1.0e-5f);
    checkVec(ctx,
             snapshot.q.rotate(snapshot.linearAccelerationDeviceG),
             linearWorldG,
             1.0e-5f);
}

static void testOrientationTimestampMustMatchSample(TestContext& ctx) {
    TrackerConfig config;
    config.resetDefaults();

    Ahrs6Dof ahrs(config.makeAhrsConfig());
    ahrs.reset(Quat::identity(), 5000);

    ImuQualityResult quality;
    PreparedOutputRuntime prepared;
    prepared.update(config, 1, 6000, ahrs, quality, Vec3::unitZ());

    TrackerPreparedOutputSnapshot snapshot;
    CHECK(ctx, !prepared.copy(snapshot));
    CHECK(ctx, !snapshot.valid);
    CHECK(ctx, !snapshot.linearAccelerationValid);
}

static void testRejectedLargeDtCannotMasqueradeAsFreshOrientation(TestContext& ctx) {
    TrackerConfig config;
    config.resetDefaults();

    Ahrs6DofConfig ahrsConfig = config.makeAhrsConfig();
    ahrsConfig.maxDtS = 0.020f;
    ahrsConfig.clampLargeDt = false;
    Ahrs6Dof ahrs(ahrsConfig);
    ahrs.reset(Quat::identity(), 1000);

    const Vec3 gyro(0.0f, 0.0f, 1.0f);
    CHECK(ctx, !ahrs.update(gyro, Vec3::unitZ(), 25000));
    CHECK(ctx, ahrs.stats().lastTimestampUs == 25000);
    CHECK(ctx, ahrs.stats().lastIntegratedTimestampUs == 1000);

    ImuQualityResult quality;
    quality.flags = imu_quality_flags::TIMESTAMP_HARDWARE |
                    imu_quality_flags::TIMESTAMP_LARGE_GAP |
                    imu_quality_flags::SAMPLE_DROPPED_BEFORE;
    quality.dtUs = 24000;

    PreparedOutputRuntime prepared;
    prepared.update(config, 1, 25000, ahrs, quality, Vec3::unitZ());
    TrackerPreparedOutputSnapshot snapshot;
    CHECK(ctx, !prepared.copy(snapshot));

    CHECK(ctx, ahrs.update(gyro, Vec3::unitZ(), 26000));
    prepared.update(config, 2, 26000, ahrs, quality, Vec3::unitZ());
    CHECK(ctx, prepared.copy(snapshot));
    CHECK(ctx, snapshot.timestampUs == 26000);
}

static void testHardAccelFailureKeepsOrientationButInvalidatesMotion(TestContext& ctx) {
    TrackerConfig config;
    config.resetDefaults();
    config.data.accelCal.valid = true;
    config.data.frame.sensorToDeviceValid = true;

    Ahrs6Dof ahrs(config.makeAhrsConfig());
    ahrs.reset(Quat::identity(), 9000);

    ImuQualityResult quality;
    quality.flags = imu_quality_flags::ACCEL_SATURATED;

    PreparedOutputRuntime prepared;
    prepared.update(config, 2, 9000, ahrs, quality, Vec3(0.0f, 0.0f, 16.0f));

    TrackerPreparedOutputSnapshot snapshot;
    CHECK(ctx, prepared.copy(snapshot));
    CHECK(ctx, snapshot.valid);
    CHECK(ctx, !snapshot.linearAccelerationValid);
    checkVec(ctx, snapshot.linearAccelerationDeviceG, Vec3::zero(), 1.0e-6f);

    quality.flags = 0;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    prepared.update(config, 3, 9000, ahrs, quality, Vec3(nan, 0.0f, 1.0f));
    CHECK(ctx, prepared.copy(snapshot));
    CHECK(ctx, !snapshot.linearAccelerationValid);
}

static void testMotionRequiresAccelAndFrameCalibration(TestContext& ctx) {
    TrackerConfig config;
    config.resetDefaults();

    Ahrs6Dof ahrs(config.makeAhrsConfig());
    ahrs.reset(Quat::identity(), 12000);

    ImuQualityResult quality;
    PreparedOutputRuntime prepared;
    TrackerPreparedOutputSnapshot snapshot;

    prepared.update(config, 1, 12000, ahrs, quality, Vec3::unitZ());
    CHECK(ctx, prepared.copy(snapshot));
    CHECK(ctx, !snapshot.linearAccelerationValid);

    config.data.accelCal.valid = true;
    prepared.update(config, 2, 12000, ahrs, quality, Vec3::unitZ());
    CHECK(ctx, prepared.copy(snapshot));
    CHECK(ctx, !snapshot.linearAccelerationValid);

    config.data.frame.sensorToDeviceValid = true;
    prepared.update(config, 3, 12000, ahrs, quality, Vec3::unitZ());
    CHECK(ctx, prepared.copy(snapshot));
    CHECK(ctx, snapshot.linearAccelerationValid);
}

static void testPreparedSnapshotRateLimitAndFailClosed(TestContext& ctx) {
    TrackerConfig config;
    config.resetDefaults();
    config.data.accelCal.valid = true;
    config.data.frame.sensorToDeviceValid = true;

    Ahrs6Dof ahrs(config.makeAhrsConfig());
    ahrs.reset(Quat::identity(), 10000);
    ImuQualityResult quality;
    PreparedOutputRuntime prepared;
    TrackerPreparedOutputSnapshot snapshot;

    prepared.update(config, 1, 10000, ahrs, quality, Vec3::unitZ());
    CHECK(ctx, prepared.copy(snapshot));
    const uint32_t firstSequence = snapshot.sequence;

    CHECK(ctx, ahrs.update(Vec3(0.0f, 0.0f, 0.2f), Vec3::unitZ(), 11000));
    prepared.update(config, 2, 11000, ahrs, quality, Vec3::unitZ());
    CHECK(ctx, prepared.copy(snapshot));
    CHECK(ctx, snapshot.sequence == firstSequence);
    CHECK(ctx, snapshot.timestampUs == 10000);

    CHECK(ctx, ahrs.update(Vec3(0.0f, 0.0f, 0.2f), Vec3::unitZ(), 15000));
    prepared.update(config, 3, 15000, ahrs, quality, Vec3::unitZ());
    CHECK(ctx, prepared.copy(snapshot));
    CHECK(ctx, snapshot.sequence == firstSequence + 1u);
    CHECK(ctx, snapshot.timestampUs == 15000);

    // A mismatched/rejected sample invalidates immediately; rate limiting may
    // never keep the previous quaternion marked fresh.
    prepared.update(config, 4, 16000, ahrs, quality, Vec3::unitZ());
    CHECK(ctx, !prepared.copy(snapshot));
}

int main() {
    TestContext ctx;
    testCoherentLinearAccelerationAndDerivedWorldFrame(ctx);
    testOrientationTimestampMustMatchSample(ctx);
    testRejectedLargeDtCannotMasqueradeAsFreshOrientation(ctx);
    testHardAccelFailureKeepsOrientationButInvalidatesMotion(ctx);
    testMotionRequiresAccelAndFrameCalibration(ctx);
    testPreparedSnapshotRateLimitAndFailClosed(ctx);
    return ctx.finish("test_prepared_output_motion_snapshot");
}
