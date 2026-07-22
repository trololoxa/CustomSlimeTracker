#include "test_common.hpp"

#include "output/slimevr_motion_frame.hpp"

using namespace tracker;

namespace {

void checkVec(TestContext& ctx, const Vec3& actual, const Vec3& expected, float eps = 1.0e-5f) {
    CHECK_NEAR(ctx, actual.x, expected.x, eps);
    CHECK_NEAR(ctx, actual.y, expected.y, eps);
    CHECK_NEAR(ctx, actual.z, expected.z, eps);
}

void testWireContractPreservesDeviceAxes(TestContext& ctx) {
    const Vec3 deviceG(0.25f, -0.50f, 1.75f);
    const Vec3 wire = slimevr_motion_frame::accelerationWireMps2FromDeviceG(deviceG);
    checkVec(ctx, wire, deviceG * 9.80665f);

    CHECK(ctx, slimevr_motion_frame::PROTOCOL_VERSION == 22u);
    CHECK(ctx, slimevr_motion_frame::protocolUsesCorrectedAcceleration(22u));
    CHECK(ctx, !slimevr_motion_frame::protocolUsesCorrectedAcceleration(21u));
}

void testRotationBoundaryKeepsWorldFromDeviceConvention(TestContext& ctx) {
    const Quat qWorldFromDevice =
        Quat::fromAxisAngle(Vec3::unitZ(), 37.0f * MATH_DEG_TO_RAD) *
        Quat::fromAxisAngle(Vec3::unitX(), -21.0f * MATH_DEG_TO_RAD);
    const Quat wire = slimevr_motion_frame::rotationWireFromWorldDevice(qWorldFromDevice * 2.0f);

    CHECK_NEAR(ctx, wire.norm(), 1.0f, 1.0e-6f);
    CHECK(ctx, wire.w >= 0.0f);
    checkVec(ctx, wire.rotate(Vec3::unitY()), qWorldFromDevice.rotate(Vec3::unitY()), 1.0e-5f);
}

void testProtocol22MatchesServerMotionBasis(TestContext& ctx) {
    // Current SlimeVR Server always left-multiplies tracker rotation by a
    // -90-degree X world-axis adapter. Protocol <22 additionally rotates only
    // acceleration by -90 degrees around local Z. That legacy-only correction
    // breaks the shared local basis when firmware already emits device-frame
    // acceleration. Protocol 22 intentionally removes it.
    const Quat serverAxesOffset =
        Quat::fromAxisAngle(Vec3::unitX(), -90.0f * MATH_DEG_TO_RAD);
    const Quat legacyAccelCorrection =
        Quat::fromAxisAngle(Vec3::unitZ(), -90.0f * MATH_DEG_TO_RAD);
    const Quat qWorldFromDevice =
        Quat::fromAxisAngle(Vec3::unitY(), 28.0f * MATH_DEG_TO_RAD) *
        Quat::fromAxisAngle(Vec3::unitZ(), -43.0f * MATH_DEG_TO_RAD);
    const Vec3 accelDevice(0.35f, 1.10f, -0.20f);

    const Quat qServerFromDevice =
        serverAxesOffset * slimevr_motion_frame::rotationWireFromWorldDevice(qWorldFromDevice);
    const Vec3 expectedServerWorld =
        serverAxesOffset.rotate(qWorldFromDevice.rotate(accelDevice));

    // Protocol 22: server consumes the device-frame acceleration unchanged.
    const Vec3 protocol22World = qServerFromDevice.rotate(accelDevice);
    checkVec(ctx, protocol22World, expectedServerWorld, 1.0e-5f);

    // Protocol 21 and below: the server's historical local-Z correction turns
    // the same physical forward impulse sideways relative to the quaternion.
    const Vec3 legacyWorld =
        qServerFromDevice.rotate(legacyAccelCorrection.rotate(accelDevice));
    CHECK(ctx, (legacyWorld - expectedServerWorld).norm() > 0.5f);
}

} // namespace

int main() {
    TestContext ctx;
    testWireContractPreservesDeviceAxes(ctx);
    testRotationBoundaryKeepsWorldFromDeviceConvention(ctx);
    testProtocol22MatchesServerMotionBasis(ctx);
    return ctx.finish("test_slimevr_motion_frame");
}
