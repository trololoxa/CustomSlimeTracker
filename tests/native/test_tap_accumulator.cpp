#include "test_common.hpp"

#include "runtime/tap_accumulator.hpp"

using namespace tracker;

static TapAccumulatorConfig testConfig() {
    TapAccumulatorConfig cfg;
    cfg.minCount = 2;
    cfg.maxCount = 10;
    cfg.aggregationWindowMs = 300;
    cfg.slidingWindow = true;
    cfg.duplicateSuppressMs = 35;
    cfg.postSendLockoutMs = 150;
    return cfg;
}

int main() {
    TestContext ctx;

    {
        TapAccumulator a;
        a.configure(testConfig());
        CHECK(ctx, !a.recordPhysicalTaps(1, 100).send);
        CHECK(ctx, a.status().pendingCount == 1);
        CHECK(ctx, !a.update(399).send);
        CHECK(ctx, !a.update(400).send);
        CHECK(ctx, a.status().suppressedBelowMin == 1);
        CHECK(ctx, a.status().pendingCount == 0);
    }

    {
        TapAccumulator a;
        a.configure(testConfig());
        CHECK(ctx, !a.recordPhysicalTaps(1, 1000).send);
        CHECK(ctx, !a.recordPhysicalTaps(1, 1100).send);
        CHECK(ctx, a.status().pendingCount == 2);
        CHECK(ctx, a.status().flushDeadlineMs == 1400);
        CHECK(ctx, !a.update(1399).send);
        const TapAccumulatorAction action = a.update(1400);
        CHECK(ctx, action.send);
        CHECK(ctx, action.value == 2);
        CHECK(ctx, a.status().lastOutputValue == 2);
        CHECK(ctx, a.status().postSendLockoutUntilMs == 1550);
    }

    {
        TapAccumulator a;
        a.configure(testConfig());
        for (uint8_t i = 0; i < 5; ++i) {
            CHECK(ctx, !a.recordPhysicalTaps(1, 2000 + static_cast<uint32_t>(i) * 80u).send);
        }
        CHECK(ctx, a.status().pendingCount == 5);
        CHECK(ctx, a.status().flushDeadlineMs == 2620);
        const TapAccumulatorAction action = a.update(2620);
        CHECK(ctx, action.send);
        CHECK(ctx, action.value == 5);
    }

    {
        TapAccumulator a;
        a.configure(testConfig());
        TapAccumulatorAction sent;
        for (uint8_t i = 0; i < 11; ++i) {
            const TapAccumulatorAction action = a.recordPhysicalTaps(1, 3000 + static_cast<uint32_t>(i) * 40u);
            if (action.send) sent = action;
        }
        CHECK(ctx, sent.send);
        CHECK(ctx, sent.value == 10);
        CHECK(ctx, a.status().clampedOverflow == 0);
        CHECK(ctx, a.status().pendingCount == 0);
        CHECK(ctx, !a.recordPhysicalTaps(1, 3500).send);
        CHECK(ctx, a.status().suppressedLockout >= 1);
    }

    {
        TapAccumulator a;
        a.configure(testConfig());
        CHECK(ctx, !a.recordPhysicalTaps(1, 4000).send);
        CHECK(ctx, !a.recordPhysicalTaps(1, 4010).send);
        CHECK(ctx, a.status().suppressedDuplicate == 1);
        CHECK(ctx, a.status().pendingCount == 1);
    }

    {
        TapAccumulator a;
        a.configure(testConfig());
        CHECK(ctx, !a.recordPhysicalTaps(2, 5000).send);
        const TapAccumulatorAction action = a.update(5300);
        CHECK(ctx, action.send);
        CHECK(ctx, action.value == 2);
        CHECK(ctx, !a.recordPhysicalTaps(1, 5400).send);
        CHECK(ctx, a.status().suppressedLockout == 1);
    }

    return ctx.finish("tap_accumulator");
}
