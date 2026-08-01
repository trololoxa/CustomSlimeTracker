#include "test_common.hpp"



#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "runtime/fifo_runtime_processor.hpp"

using namespace tracker;

namespace {

class FifoTransport final : public Lsm6dsvTransport {
public:
    bool read(uint8_t, uint8_t* dst, size_t len) override {
        if (!dst || len == 0) return false;
        if (len == 1) {
            dst[0] = 0;
            return true;
        }
        if (len == 2) {
            const size_t remaining = words_.size() - readIndex_;
            dst[0] = static_cast<uint8_t>(remaining & 0xffu);
            dst[1] = static_cast<uint8_t>((remaining >> 8) & 0x01u);
            return true;
        }
        if ((len % Lsm6dsvFifoReader::FIFO_WORD_BYTES) != 0u) return false;

        const size_t count = len / Lsm6dsvFifoReader::FIFO_WORD_BYTES;
        if (readIndex_ + count > words_.size()) return false;
        for (size_t i = 0; i < count; ++i) {
            std::memcpy(dst + i * Lsm6dsvFifoReader::FIFO_WORD_BYTES,
                        words_[readIndex_ + i].data(),
                        Lsm6dsvFifoReader::FIFO_WORD_BYTES);
        }
        readIndex_ += count;
        dataReads_++;
        trackerTestAdvanceMicros(dataReadCostUs_);
        return true;
    }

    bool write(uint8_t, const uint8_t*, size_t) override { return true; }
    void delayMs(uint32_t) override {}

    void addSample(int16_t sequence) {
        words_.push_back(makeWord(Lsm6dsvFifoReader::TAG_GYRO_NC,
                                  sequence,
                                  static_cast<int16_t>(sequence + 1),
                                  static_cast<int16_t>(sequence + 2)));
        words_.push_back(makeWord(Lsm6dsvFifoReader::TAG_ACCEL_NC,
                                  static_cast<int16_t>(sequence + 10),
                                  static_cast<int16_t>(sequence + 11),
                                  static_cast<int16_t>(sequence + 12)));
    }

    void addMag(int16_t sequence) {
        words_.push_back(makeWord(Lsm6dsvFifoReader::TAG_SENSORHUB_SLAVE0,
                                  static_cast<int16_t>(sequence + 100),
                                  static_cast<int16_t>(sequence + 200),
                                  static_cast<int16_t>(sequence + 300)));
    }

    size_t remainingWords() const { return words_.size() - readIndex_; }
    uint32_t dataReads() const { return dataReads_; }
    void setDataReadCostUs(uint32_t value) { dataReadCostUs_ = value; }

private:
    static std::array<uint8_t, Lsm6dsvFifoReader::FIFO_WORD_BYTES> makeWord(
        uint8_t sensorTag,
        int16_t x,
        int16_t y,
        int16_t z) {
        std::array<uint8_t, Lsm6dsvFifoReader::FIFO_WORD_BYTES> out{};
        out[0] = static_cast<uint8_t>(sensorTag << 3);
        writeLe16(out.data() + 1, x);
        writeLe16(out.data() + 3, y);
        writeLe16(out.data() + 5, z);
        return out;
    }

    static void writeLe16(uint8_t* dst, int16_t value) {
        const uint16_t u = static_cast<uint16_t>(value);
        dst[0] = static_cast<uint8_t>(u & 0xffu);
        dst[1] = static_cast<uint8_t>((u >> 8) & 0xffu);
    }

    std::vector<std::array<uint8_t, Lsm6dsvFifoReader::FIFO_WORD_BYTES>> words_;
    size_t readIndex_ = 0;
    uint32_t dataReads_ = 0;
    uint32_t dataReadCostUs_ = 0;
};

struct CallbackProbe {
    uint32_t rawCalls = 0;
    uint32_t magCalls = 0;
    uint32_t recoverOnCall = 0;
    uint32_t yieldOnCall = 0;
    uint32_t rawCallbackCostUs = 0;
    uint32_t magCallbackCostUs = 0;
    uint64_t latestRawTimestampUs = 0;
    std::vector<uint64_t> magEndpointSkewUs;
    std::vector<uint32_t> statsDeltaCalls;
};

FifoRuntimeSampleResult onRaw(const Lsm6dsv::RawSample& raw,
                              bool checkFifoStatsDelta,
                              void* user) {
    auto& probe = *static_cast<CallbackProbe*>(user);
    probe.rawCalls++;
    probe.latestRawTimestampUs = raw.t_us;
    trackerTestAdvanceMicros(probe.rawCallbackCostUs);
    if (checkFifoStatsDelta) probe.statsDeltaCalls.push_back(probe.rawCalls);
    if (probe.recoverOnCall != 0u && probe.rawCalls == probe.recoverOnCall) {
        return FifoRuntimeSampleResult::FifoRecovered;
    }
    if (probe.yieldOnCall != 0u && probe.rawCalls == probe.yieldOnCall) {
        return FifoRuntimeSampleResult::YieldRequested;
    }
    return FifoRuntimeSampleResult::Continue;
}

void onMag(const Lsm6dsvFifoReader::MagRawSample& mag, void* user) {
    auto& probe = *static_cast<CallbackProbe*>(user);
    probe.magCalls++;
    trackerTestAdvanceMicros(probe.magCallbackCostUs);
    const uint64_t skew = probe.latestRawTimestampUs >= mag.t_us
        ? probe.latestRawTimestampUs - mag.t_us
        : UINT64_MAX;
    probe.magEndpointSkewUs.push_back(skew);
}

void onTime(uint32_t, void*) {}

struct Fixture {
    FifoTransport bus;
    Lsm6dsv lsm{bus};
    Lsm6dsvFifoReader fifo{bus, lsm};
    volatile uint32_t irqCount = 0;
    FifoInterruptEventSource events;
    Lsm6dsv::RawSample raw[160]{};
    Lsm6dsvFifoReader::MagRawSample mag[48]{};
    Lsm6dsv::RawSample rawQueue[256]{};
    uint8_t rawQueueFlags[256]{};
    Lsm6dsvFifoReader::MagRawSample magQueue[64]{};
    CallbackProbe probe;
    float magPeriodUs = 16667.0f;
    FifoRuntimeProcessor processor;
    Stream out;

    bool begin() {
        Lsm6dsvFifoReader::Config config;
        config.enableTimestampCounter = false;
        config.useHardwareTimestamps = false;
        config.timestampBatch = Lsm6dsvFifoReader::TimestampBatch::Off;
        config.temperatureBatch = Lsm6dsvFifoReader::TemperatureBatch::Off;
        config.enableSensorHubSlave0 = true;
        config.sensorHubSlave0PeriodUs = magPeriodUs;
        if (!fifo.configure(config)) return false;
        events.begin(&irqCount, &fifo, nullptr, 2000);
        processor.begin(&events,
                        &fifo,
                        raw,
                        sizeof(raw) / sizeof(raw[0]),
                        mag,
                        sizeof(mag) / sizeof(mag[0]),
                        rawQueue,
                        rawQueueFlags,
                        sizeof(rawQueue) / sizeof(rawQueue[0]),
                        magQueue,
                        sizeof(magQueue) / sizeof(magQueue[0]),
                        onRaw,
                        onMag,
                        onTime,
                        &probe);
        return true;
    }
};

void testCallbacksAreSlicedAcrossAppPasses(TestContext& ctx) {
    trackerTestSetMicros(0);
    Fixture f;
    for (int16_t i = 0; i < 30; ++i) f.bus.addSample(i);
    CHECK(ctx, f.begin());
    f.probe.rawCallbackCostUs = 1000;
    f.irqCount = 1;

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE);
    CHECK(ctx, f.bus.dataReads() == 1);
    CHECK(ctx, f.bus.remainingWords() == 0);
    CHECK(ctx, f.processor.queueStats().sliceBudgetStops == 1u);
    CHECK(ctx, f.processor.queueStats().sliceBudgetOvershootEvents == 1u);
    CHECK(ctx, f.processor.queueStats().sliceBudgetOvershootMaxUs <= 500u);

    uint32_t expectedCalls = cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE;
    while (expectedCalls < 30u) {
        CHECK(ctx, f.processor.process(12, 384, 6, f.out));
        expectedCalls += cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE;
        if (expectedCalls > 30u) expectedCalls = 30u;
        CHECK(ctx, f.probe.rawCalls == expectedCalls);
        CHECK(ctx, f.bus.dataReads() == 1);
    }

    // FIFO stats deltas are evaluated once per hardware drain, not once per
    // cooperative callback slice.
    CHECK(ctx, f.probe.statsDeltaCalls.size() == 1);
    CHECK(ctx, f.probe.statsDeltaCalls[0] == 1);
}
void testHardwareDrainTimeSharesBudgetButPreservesMinimumProgress(TestContext& ctx) {
    trackerTestSetMicros(0);
    Fixture f;
    for (int16_t i = 0; i < 30; ++i) f.bus.addSample(i);
    CHECK(ctx, f.begin());
    f.bus.setDataReadCostUs(7000);
    f.probe.rawCallbackCostUs = 500;
    f.irqCount = 1;

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    // The 7 ms SPI read consumes the absolute slice budget. The processor
    // still completes the bounded minimum progress needed to avoid livelock.
    CHECK(ctx, f.probe.rawCalls == cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE);
    CHECK(ctx, f.bus.dataReads() == 1);
    CHECK(ctx, f.processor.queueStats().sliceBudgetOvershootEvents == 1u);
}

void testRecoveryDropsRemainderOfPredateBatch(TestContext& ctx) {
    trackerTestSetMicros(0);
    Fixture f;
    for (int16_t i = 0; i < 20; ++i) f.bus.addSample(i);
    CHECK(ctx, f.begin());
    f.probe.recoverOnCall = 3;
    f.irqCount = 1;

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == 3);

    // The local samples were captured before the callback reset the FIFO.
    // They must never leak into later app passes.
    CHECK(ctx, !f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == 3);
}


void testExternalResetDropsPendingBatch(TestContext& ctx) {
    trackerTestSetMicros(0);
    Fixture f;
    for (int16_t i = 0; i < 20; ++i) f.bus.addSample(i);
    CHECK(ctx, f.begin());
    f.probe.rawCallbackCostUs = 1000;
    f.irqCount = 1;

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE);
    f.processor.resetWork();

    CHECK(ctx, !f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE);
}

void testMagCallbacksFollowNearestRawTimestamp(TestContext& ctx) {
    trackerTestSetMicros(1000000);
    Fixture f;
    for (int16_t i = 0; i < 80; ++i) {
        f.bus.addSample(i);
        if ((i + 1) % 16 == 0) {
            f.bus.addMag(i);
        }
    }
    CHECK(ctx, f.begin());
    f.irqCount = 1;

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == 64);
    CHECK(ctx, f.probe.magCalls == 4);
    CHECK(ctx, f.probe.magEndpointSkewUs.size() == 4);
    for (const uint64_t skewUs : f.probe.magEndpointSkewUs) {
        // One 960 Hz raw period plus rounding headroom. The historical
        // raw-first scheduler produced tens of milliseconds here.
        CHECK(ctx, skewUs <= 1200u);
    }

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == 80);
    CHECK(ctx, f.probe.magCalls == 5);
    CHECK(ctx, f.probe.magEndpointSkewUs.back() <= 1200u);
    CHECK(ctx, f.processor.queueStats().magChronologicalDeferrals == 0u);
}

void testMagCallbacksRespectCountBudget(TestContext& ctx) {
    trackerTestSetMicros(1000000);
    Fixture f;
    f.magPeriodUs = 1.0f;
    for (int16_t i = 0; i < 10; ++i) f.bus.addMag(i);
    for (int16_t i = 0; i < 32; ++i) f.bus.addSample(i);
    CHECK(ctx, f.begin());
    f.irqCount = 1;

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.magCalls == cfg::FIFO_RUNTIME_MAX_MAG_CALLBACKS_PER_SLICE);
    CHECK(ctx, f.processor.queueStats().magCallbackCountDeferrals == 1u);
    CHECK(ctx, f.processor.queueStats().magCallbackBudgetDeferrals == 0u);
    CHECK(ctx, f.processor.queueStats().magChronologicalDeferrals == 1u);
    const uint32_t rawAfterFirstPass = f.probe.rawCalls;

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.magCalls == 10u);
    CHECK(ctx, f.probe.rawCalls >= rawAfterFirstPass);
    for (const uint64_t skewUs : f.probe.magEndpointSkewUs) {
        CHECK(ctx, skewUs <= 1200u);
    }
}

void testMagCallbacksRespectCooperativeTimeBudget(TestContext& ctx) {
    trackerTestSetMicros(1000000);
    Fixture f;
    f.magPeriodUs = 1.0f;
    // A delayed sensor-hub burst can place many old mag frames before the
    // next raw endpoint. They are all chronologically due at once.
    for (int16_t i = 0; i < 10; ++i) f.bus.addMag(i);
    for (int16_t i = 0; i < 32; ++i) f.bus.addSample(i);
    CHECK(ctx, f.begin());
    f.probe.magCallbackCostUs = 1000;
    f.irqCount = 1;

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.magCalls == 4u);
    CHECK(ctx, f.processor.queueStats().magCallbackBudgetDeferrals == 1u);
    CHECK(ctx, f.processor.queueStats().magChronologicalDeferrals == 1u);
    const uint32_t rawAfterFirstPass = f.probe.rawCalls;
    CHECK(ctx, rawAfterFirstPass <= 2u);

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.magCalls >= 8u);
    CHECK(ctx, f.probe.rawCalls == rawAfterFirstPass);
    for (const uint64_t skewUs : f.probe.magEndpointSkewUs) {
        CHECK(ctx, skewUs <= 1200u);
    }
}




void testYieldPreservesSameTimestampMagOrdering(TestContext& ctx) {
    trackerTestSetMicros(1000000u);
    Fixture f;
    f.bus.addSample(1);
    f.bus.addMag(1);
    f.bus.addSample(2);
    CHECK(ctx, f.begin());
    f.probe.yieldOnCall = 1u;
    f.irqCount = 1u;

    CHECK(ctx, f.processor.process(12u, 384u, 6u, f.out));
    CHECK(ctx, f.probe.rawCalls == 1u);
    CHECK(ctx, f.probe.magCalls == 1u);
    CHECK(ctx, f.probe.magEndpointSkewUs.size() == 1u);
    CHECK(ctx, f.probe.magEndpointSkewUs[0] == 0u);
    CHECK(ctx, f.processor.rawQueueDepth() == 1u);

    f.probe.yieldOnCall = 0u;
    CHECK(ctx, f.processor.process(12u, 384u, 6u, f.out));
    CHECK(ctx, f.probe.rawCalls == 2u);
}

void testRamQueueAbsorbsSecondHardwareBurst(TestContext& ctx) {
    trackerTestSetMicros(0);
    Fixture f;
    for (int16_t i = 0; i < 80; ++i) f.bus.addSample(i);
    CHECK(ctx, f.begin());
    f.probe.rawCallbackCostUs = 1000;
    f.irqCount = 1;

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE);
    CHECK(ctx, f.processor.rawQueueDepth() ==
               80u - cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE);
    // Depth is well below the legacy threshold, but the queue already spans
    // more than the 40 ms freshness threshold.
    CHECK(ctx, !f.processor.urgentByDepth());
    CHECK(ctx, f.processor.rawQueueSpanUs() >= cfg::FIFO_RUNTIME_URGENT_SPAN_US);
    CHECK(ctx, f.processor.urgentByAge());
    CHECK(ctx, f.processor.urgent());

    for (int16_t i = 80; i < 120; ++i) f.bus.addSample(i);
    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls ==
               2u * cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE);
    CHECK(ctx, f.processor.rawQueueDepth() ==
               120u - 2u * cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE);
    CHECK(ctx, f.processor.queueStats().rawQueueHighWater >=
               120u - cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE);
    CHECK(ctx, f.processor.queueStats().rawQueueOverflow == 0);
}
} // namespace

int main() {
    TestContext ctx;
    testCallbacksAreSlicedAcrossAppPasses(ctx);
    testHardwareDrainTimeSharesBudgetButPreservesMinimumProgress(ctx);
    testRecoveryDropsRemainderOfPredateBatch(ctx);
    testExternalResetDropsPendingBatch(ctx);
    testMagCallbacksFollowNearestRawTimestamp(ctx);
    testMagCallbacksRespectCountBudget(ctx);
    testMagCallbacksRespectCooperativeTimeBudget(ctx);
    testYieldPreservesSameTimestampMagOrdering(ctx);
    testRamQueueAbsorbsSecondHardwareBurst(ctx);
    return ctx.finish("test_fifo_runtime_processor");
}
