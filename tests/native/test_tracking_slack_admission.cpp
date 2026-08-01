#include "test_common.hpp"

#include "runtime/tracking_slack_admission.hpp"

using namespace tracker;

int main() {
    TestContext ctx;

    TrackingSlackAdmissionInput input;
    CHECK(ctx, trackingOptionalRuntimeAdmitted(
        input, TrackingOptionalServiceClass::Background));

    input.softwareQueuePending = true;
    CHECK(ctx, !trackingOptionalRuntimeAdmitted(
        input, TrackingOptionalServiceClass::Short));
    input.softwareQueuePending = false;
    input.fifoUrgent = true;
    CHECK(ctx, !trackingOptionalRuntimeAdmitted(
        input, TrackingOptionalServiceClass::Console));

    input.fifoUrgent = false;
    input.rotationSlackKnown = true;
    input.rotationSlackUs = TRACKER_OPTIONAL_SHORT_SERVICE_MIN_SLACK_US - 1u;
    CHECK(ctx, !trackingOptionalRuntimeAdmitted(
        input, TrackingOptionalServiceClass::Short));
    input.rotationSlackUs = TRACKER_OPTIONAL_SHORT_SERVICE_MIN_SLACK_US;
    CHECK(ctx, trackingOptionalRuntimeAdmitted(
        input, TrackingOptionalServiceClass::Short));

    input.rotationSlackUs = TRACKER_OPTIONAL_CONSOLE_SERVICE_MIN_SLACK_US - 1u;
    CHECK(ctx, !trackingOptionalRuntimeAdmitted(
        input, TrackingOptionalServiceClass::Console));
    input.rotationSlackUs = TRACKER_OPTIONAL_BACKGROUND_SERVICE_MIN_SLACK_US;
    CHECK(ctx, trackingOptionalRuntimeAdmitted(
        input, TrackingOptionalServiceClass::Background));
    CHECK(ctx, trackingBackgroundRuntimeAdmitted(input, false));
    CHECK(ctx, !trackingBackgroundRuntimeAdmitted(input, true));

    return ctx.finish("test_tracking_slack_admission");
}
