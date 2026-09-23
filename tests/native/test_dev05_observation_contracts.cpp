#include "test_common.hpp"
#include "dev05/observation_contracts.hpp"
#include <limits>
using namespace dev05;

static void firstCompletedWindow(TestContext& ctx) {
    FirstSampleDwell dwell(4, 0);
    // Interrupted candidate, then an irregularly sampled complete window.
    dwell.observe(100, true);
    dwell.observe(200, true);
    dwell.observe(300, false);
    dwell.observe(410, true);
    dwell.observe(520, true);
    dwell.observe(800, true);
    CHECK(ctx, !dwell.firstWindowStartUs());
    dwell.observe(1200, true);
    CHECK(ctx, dwell.firstWindowStartUs() == 410);
    // Losing and recovering trust must not replace the first result.
    dwell.observe(1500, false);
    for (uint64_t t : {2000ULL, 2100ULL, 2300ULL, 2800ULL, 2900ULL}) dwell.observe(t, true);
    CHECK(ctx, dwell.firstWindowStartUs() == 410);
    bool rejected = false;
    try { dwell.observe(2900, true); } catch (const std::invalid_argument&) { rejected = true; }
    CHECK(ctx, rejected);
    CHECK(ctx, dwell.firstWindowStartUs() == 410);

    // Exact boundary of the production probe's 240-sample window, across 2^32.
    constexpr uint64_t epoch = 4294967000ULL;
    FirstSampleDwell fractional(240, epoch);
    for (uint64_t i = 1; i < 240; ++i) fractional.observe(epoch + i * 1000000 / 960, true);
    CHECK(ctx, !fractional.firstWindowStartUs());
    fractional.observe(epoch + 250000, true);
    CHECK(ctx, fractional.firstWindowStartUs() == epoch + 1041);
    FirstSampleDwell never(2, 0);
    for (uint64_t t = 1; t <= 20; ++t) never.observe(t, t % 2 == 1);
    CHECK(ctx, !never.firstWindowStartUs());
    rejected = false;
    try { FirstSampleDwell invalid(0, 0); } catch (const std::invalid_argument&) { rejected = true; }
    CHECK(ctx, rejected);
}

static void propagationRequiresMotionAndTime(TestContext& ctx) {
    const Q before = exp({0.2, -0.3, 0.1});
    const Q expected = product(before, exp({0, 0, 0.0002}));
    Q rounded{};
    for (size_t i = 0; i < 4; ++i) rounded[i] = static_cast<float>(expected[i]);
    constexpr uint64_t oldUs = 4296970000ULL, nowUs = oldUs + 1000;
    CHECK(ctx, gyroPropagationObserved(true, rounded, expected, oldUs, nowUs, nowUs));
    for (double& v : rounded) v = -v;
    CHECK(ctx, gyroPropagationObserved(true, rounded, expected, oldUs, nowUs, nowUs));
    // Bool-only success, timestamp-only success and quaternion-only success.
    CHECK(ctx, !gyroPropagationObserved(true, before, expected, oldUs, oldUs, nowUs));
    CHECK(ctx, !gyroPropagationObserved(true, before, expected, oldUs, nowUs, nowUs));
    CHECK(ctx, !gyroPropagationObserved(true, expected, expected, oldUs, oldUs, nowUs));
    CHECK(ctx, !gyroPropagationObserved(true, expected, expected, oldUs, nowUs + 1, nowUs));
    CHECK(ctx, !gyroPropagationObserved(true, expected, expected, nowUs, nowUs, nowUs));
    CHECK(ctx, !gyroPropagationObserved(false, expected, expected, oldUs, nowUs, nowUs));
    CHECK(ctx, !gyroPropagationObserved(true, product(before, exp({0, 0, -0.0002})), expected, oldUs, nowUs, nowUs));
    CHECK(ctx, !gyroPropagationObserved(true, product(before, exp({0, 0, 0.0004})), expected, oldUs, nowUs, nowUs));
    for (double bad : {0.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        CHECK(ctx, !gyroPropagationObserved(true, {bad,0,0,0}, expected, oldUs, nowUs, nowUs));
    }
    Q scaled = expected;
    for (double& v : scaled) v *= 2;
    CHECK(ctx, !gyroPropagationObserved(true, scaled, expected, oldUs, nowUs, nowUs));
}

int main() {
    TestContext ctx;
    firstCompletedWindow(ctx);
    propagationRequiresMotionAndTime(ctx);
    return ctx.finish("test_dev05_observation_contracts");
}
