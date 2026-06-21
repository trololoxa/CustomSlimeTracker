#include <cstdint>

#include "runtime/motion_light_sleep_controller.hpp"
#include "test_common.hpp"

using tracker::MotionLightSleepController;

int main() {
    TestContext ctx;

    MotionLightSleepController controller(60000u);
    CHECK(ctx, !controller.shouldEnter(true, 100u));
    CHECK(ctx, !controller.serverAbsenceActive());

    CHECK(ctx, !controller.shouldEnter(false, 1000u));
    CHECK(ctx, controller.serverAbsenceActive());
    CHECK(ctx, controller.serverAbsentSinceMs() == 1000u);
    CHECK(ctx, !controller.shouldEnter(false, 60999u));
    CHECK(ctx, controller.shouldEnter(false, 61000u));

    controller.noteSleepAttempted();
    CHECK(ctx, !controller.serverAbsenceActive());
    CHECK(ctx, !controller.shouldEnter(false, 61001u));

    // A newly found server must cancel an existing continuous-absence timer.
    CHECK(ctx, !controller.shouldEnter(true, 62000u));
    CHECK(ctx, !controller.serverAbsenceActive());
    CHECK(ctx, !controller.shouldEnter(false, 62001u));
    CHECK(ctx, !controller.shouldEnter(false, 122000u - 1u));
    CHECK(ctx, controller.shouldEnter(false, 122001u));

    // unsigned subtraction keeps the timeout correct across millis() wrap.
    controller.setServerAbsenceTimeoutMs(32u);
    CHECK(ctx, !controller.shouldEnter(false, 0xfffffff0u));
    CHECK(ctx, !controller.shouldEnter(false, 0x0000000fu));
    CHECK(ctx, controller.shouldEnter(false, 0x00000010u));

    controller.setServerAbsenceTimeoutMs(0u);
    CHECK(ctx, !controller.shouldEnter(false, 5u));

    return ctx.finish("motion_light_sleep_controller");
}
