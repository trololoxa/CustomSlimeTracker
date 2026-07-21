#include "test_common.hpp"

#include <cstring>

#include "config/tracker_config_runtime.hpp"
#include "runtime/orientation_runtime_reset.hpp"
#include "sensor/imu_quality.hpp"

using namespace tracker;

namespace {
uint32_t g_callbackCount = 0;
uint64_t g_callbackTimestampUs = 0;
bool g_callbackRebase = true;
char g_callbackReason[64] = {};

void dependentResetCallback(const char* reason, uint64_t timestampUs, bool rebaseAhrsTimebase) {
    g_callbackCount++;
    g_callbackTimestampUs = timestampUs;
    g_callbackRebase = rebaseAhrsTimebase;
    std::strncpy(g_callbackReason, reason ? reason : "", sizeof(g_callbackReason) - 1);
    g_callbackReason[sizeof(g_callbackReason) - 1] = '\0';
}
}

static void testDeliberateResetInvalidatesWholeOrientationContract(TestContext& ctx) {
    TrackerConfig config;
    config.resetDefaults();

    Ahrs6Dof ahrs(config.makeAhrsConfig());
    CHECK(ctx, !ahrs.update(Vec3::zero(), Vec3::unitZ(), 1000));
    CHECK(ctx, ahrs.update(Vec3(0.0f, 0.0f, 0.2f), Vec3::unitZ(), 2000));
    CHECK(ctx, ahrs.initialized());

    PreparedOutputRuntime prepared;
    ImuQualityResult quality;
    quality.overallConfidence = 0.9f;
    prepared.update(config, 2, 2000, ahrs, quality, Vec3::unitZ());

    TrackerPreparedOutputSnapshot before;
    CHECK(ctx, prepared.copy(before));
    CHECK(ctx, before.valid);

    OrientationRuntimeResetDeps deps;
    deps.ahrs = &ahrs;
    deps.preparedOutput = &prepared;
    deps.resetDependentState = dependentResetCallback;

    g_callbackCount = 0;
    g_callbackTimestampUs = 0;
    g_callbackRebase = true;
    g_callbackReason[0] = '\0';

    CHECK(ctx, resetOrientationRuntime(deps, "frame_changed", 123456));
    CHECK(ctx, !ahrs.initialized());
    CHECK_NEAR(ctx, ahrs.quaternion().w, 1.0f, 1.0e-6f);
    CHECK_NEAR(ctx, ahrs.quaternion().x, 0.0f, 1.0e-6f);
    CHECK(ctx, ahrs.stats().updateCount == 0);

    TrackerPreparedOutputSnapshot after;
    CHECK(ctx, !prepared.copy(after));
    CHECK(ctx, !after.valid);

    CHECK(ctx, g_callbackCount == 1);
    CHECK(ctx, g_callbackTimestampUs == 123456);
    CHECK(ctx, !g_callbackRebase);
    CHECK(ctx, std::strcmp(g_callbackReason, "frame_changed") == 0);

    // The next plausible sample reacquires tilt in the new frame without
    // integrating a stale pre-change quaternion.
    CHECK(ctx, !ahrs.update(Vec3::zero(), Vec3::unitY(), 124000));
    CHECK(ctx, ahrs.initialized());
    CHECK(ctx, ahrs.quaternion().isFinite());
}

static void testIncompleteDependenciesFailClosed(TestContext& ctx) {
    OrientationRuntimeResetDeps deps;
    CHECK(ctx, !resetOrientationRuntime(deps, "missing", 1));

    Ahrs6Dof ahrs;
    deps.ahrs = &ahrs;
    CHECK(ctx, !resetOrientationRuntime(deps, "missing_output", 1));
}

int main() {
    TestContext ctx;
    testDeliberateResetInvalidatesWholeOrientationContract(ctx);
    testIncompleteDependenciesFailClosed(ctx);
    return ctx.finish("test_orientation_runtime_reset");
}
