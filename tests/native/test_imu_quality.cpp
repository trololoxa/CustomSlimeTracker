#include "test_common.hpp"

#include "sensor/imu_quality.hpp"

using namespace tracker;

static Lsm6dsv::RawSample makeRaw(uint64_t tUs, uint16_t flags = Lsm6dsvFifoReader::FIFO_FLAG_TS_HARDWARE) {
    Lsm6dsv::RawSample raw;
    raw.t_us = tUs;
    raw.flags = flags;
    raw.ax = 0;
    raw.ay = 0;
    raw.az = 8192;
    return raw;
}

static Lsm6dsv::Sample makeSample(uint64_t tUs, const Vec3& accelG = Vec3::unitZ()) {
    Lsm6dsv::Sample sample;
    sample.t_us = tUs;
    sample.accel_g = accelG;
    sample.gyro_rad_s = Vec3::zero();
    return sample;
}

static Lsm6dsvFifoReader::DrainStats makeStats() {
    Lsm6dsvFifoReader::DrainStats stats;
    stats.samplePeriodUs = 1000.0f;
    return stats;
}

static void testTimestampGapAndRecovery(TestContext& ctx) {
    ImuQualityConfig cfg;
    cfg.expectedDtUs = 1000.0f;
    ImuQualityMonitor monitor(cfg);
    Lsm6dsvFifoReader::DrainStats stats = makeStats();

    ImuQualityResult first = monitor.evaluate(makeRaw(1000), makeSample(1000), stats, false);
    CHECK(ctx, first.has(imu_quality_flags::TIMESTAMP_HARDWARE));
    CHECK(ctx, first.dtUs == 0);
    CHECK(ctx, first.shouldUpdateAhrs);

    ImuQualityResult normal = monitor.evaluate(makeRaw(2000), makeSample(2000), stats, false);
    CHECK(ctx, normal.dtUs == 1000);
    CHECK(ctx, !normal.has(imu_quality_flags::TIMESTAMP_LARGE_GAP));
    CHECK(ctx, normal.shouldUpdateAhrs);

    ImuQualityResult gap = monitor.evaluate(makeRaw(6000), makeSample(6000), stats, false);
    CHECK(ctx, gap.has(imu_quality_flags::TIMESTAMP_LARGE_GAP));
    CHECK(ctx, gap.has(imu_quality_flags::SAMPLE_DROPPED_BEFORE));
    CHECK(ctx, gap.estimatedDroppedBefore == 3);
    CHECK(ctx, gap.shouldUpdateAhrs);

    ImuQualityResult backwards = monitor.evaluate(makeRaw(5000), makeSample(5000), stats, false);
    CHECK(ctx, backwards.has(imu_quality_flags::TIMESTAMP_NON_MONOTONIC));
    CHECK(ctx, !backwards.shouldUpdateAhrs);

    // Rejected timestamp must not poison the next normal dt baseline.
    ImuQualityResult afterBackwards = monitor.evaluate(makeRaw(7000), makeSample(7000), stats, false);
    CHECK(ctx, afterBackwards.dtUs == 1000);
    CHECK(ctx, afterBackwards.shouldUpdateAhrs);

    const ImuQualityCounters& c = monitor.counters();
    CHECK(ctx, c.hwTimestampSamples == 5);
    CHECK(ctx, c.largeGapSamples == 1);
    CHECK(ctx, c.estimatedDroppedSamples == 3);
    CHECK(ctx, c.nonMonotonicTimestampSamples == 1);
    CHECK(ctx, c.ahrsSkippedSamples == 1);
    CHECK_NEAR(ctx, c.meanDtUs(), 1500.0f, 1.0e-4f);
}

static void testSaturationAndAccelOutlierGates(TestContext& ctx) {
    ImuQualityConfig cfg;
    cfg.expectedDtUs = 1000.0f;
    ImuQualityMonitor monitor(cfg);
    Lsm6dsvFifoReader::DrainStats stats = makeStats();

    monitor.evaluate(makeRaw(1000), makeSample(1000), stats, false);

    Lsm6dsv::RawSample raw = makeRaw(2000, Lsm6dsvFifoReader::FIFO_FLAG_TS_HARDWARE);
    raw.flags |= Lsm6dsv::FLAG_GYRO_SATURATED;
    raw.gx = 32767;
    raw.az = 32000;

    Lsm6dsv::Sample sample = makeSample(2000, Vec3(0.0f, 0.0f, 2.0f));
    ImuQualityResult q = monitor.evaluate(raw, sample, stats, false);

    CHECK(ctx, q.has(imu_quality_flags::GYRO_SATURATED));
    CHECK(ctx, q.has(imu_quality_flags::GYRO_NEAR_SATURATION));
    CHECK(ctx, q.has(imu_quality_flags::ACCEL_NEAR_SATURATION));
    CHECK(ctx, q.has(imu_quality_flags::ACCEL_NORM_OUTLIER));
    CHECK(ctx, !q.shouldUpdateAhrs);
    CHECK(ctx, !q.shouldUseAccelCorrection);
    CHECK(ctx, q.has(imu_quality_flags::SAMPLE_NOT_AHRS_USABLE));
    CHECK(ctx, q.has(imu_quality_flags::ACCEL_NOT_AHRS_USABLE));

    const ImuQualityCounters& c = monitor.counters();
    CHECK(ctx, c.gyroSaturatedSamples == 1);
    CHECK(ctx, c.gyroNearSaturatedSamples == 1);
    CHECK(ctx, c.accelNearSaturatedSamples == 1);
    CHECK(ctx, c.accelNormOutliers == 1);
    CHECK(ctx, c.accelCorrectionDisabledSamples == 1);
}


static void testUnknownTagAloneDoesNotRequestRecoveryByDefault(TestContext& ctx) {
    ImuQualityMonitor monitor;
    Lsm6dsvFifoReader::DrainStats stats = makeStats();

    monitor.syncFifoStats(stats);
    stats.unknownWords = 1;

    ImuQualityResult q = monitor.evaluate(makeRaw(1000), makeSample(1000), stats, true);
    CHECK(ctx, q.has(imu_quality_flags::FIFO_UNKNOWN_TAG));
    CHECK(ctx, !q.has(imu_quality_flags::FIFO_RECOVERY_REQUESTED));
    CHECK(ctx, !q.shouldRequestFifoRecovery);
    CHECK(ctx, !monitor.recoveryRequested());

    const ImuQualityCounters& c = monitor.counters();
    CHECK(ctx, c.fifoUnknownTagEvents == 1);
    CHECK(ctx, c.fifoRecoveryRequests == 0);
}

static void testFifoStatsDeltaRequestsRecovery(TestContext& ctx) {
    ImuQualityMonitor monitor;
    Lsm6dsvFifoReader::DrainStats stats = makeStats();

    monitor.syncFifoStats(stats);
    stats.overrunEvents = 1;
    stats.fullEvents = 1;
    stats.unknownWords = 2;
    stats.timestampBackwards = 1;

    ImuQualityResult q = monitor.evaluate(makeRaw(1000), makeSample(1000), stats, true);
    CHECK(ctx, q.has(imu_quality_flags::FIFO_OVERRUN));
    CHECK(ctx, q.has(imu_quality_flags::FIFO_FULL));
    CHECK(ctx, q.has(imu_quality_flags::FIFO_UNKNOWN_TAG));
    CHECK(ctx, q.has(imu_quality_flags::TIMESTAMP_BACKWARDS));
    CHECK(ctx, q.has(imu_quality_flags::FIFO_RECOVERY_REQUESTED));
    CHECK(ctx, q.shouldRequestFifoRecovery);
    CHECK(ctx, monitor.recoveryRequested());
    CHECK(ctx, monitor.lastRecoveryFlags() != 0);

    monitor.clearRecoveryRequest();
    CHECK(ctx, !monitor.recoveryRequested());
    CHECK(ctx, monitor.lastRecoveryFlags() == 0);

    const ImuQualityCounters& c = monitor.counters();
    CHECK(ctx, c.fifoOverrunEvents == 1);
    CHECK(ctx, c.fifoFullEvents == 1);
    CHECK(ctx, c.fifoUnknownTagEvents == 2);
    CHECK(ctx, c.timestampBackwards == 1);
    CHECK(ctx, c.fifoRecoveryRequests == 1);
}

int main() {
    TestContext ctx;
    testTimestampGapAndRecovery(ctx);
    testSaturationAndAccelOutlierGates(ctx);
    testUnknownTagAloneDoesNotRequestRecoveryByDefault(ctx);
    testFifoStatsDeltaRequestsRecovery(ctx);
    return ctx.finish("test_imu_quality");
}
