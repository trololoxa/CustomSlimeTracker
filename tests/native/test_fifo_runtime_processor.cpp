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
    uint32_t rawCallbackCostUs = 0;
    std::vector<uint32_t> statsDeltaCalls;
};

FifoRuntimeSampleResult onRaw(const Lsm6dsv::RawSample&,
                              bool checkFifoStatsDelta,
                              void* user) {
    auto& probe = *static_cast<CallbackProbe*>(user);
    probe.rawCalls++;
    trackerTestAdvanceMicros(probe.rawCallbackCostUs);
    if (checkFifoStatsDelta) probe.statsDeltaCalls.push_back(probe.rawCalls);
    if (probe.recoverOnCall != 0u && probe.rawCalls == probe.recoverOnCall) {
        return FifoRuntimeSampleResult::FifoRecovered;
    }
    return FifoRuntimeSampleResult::Continue;
}

void onMag(const Lsm6dsvFifoReader::MagRawSample&, void* user) {
    static_cast<CallbackProbe*>(user)->magCalls++;
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
    FifoRuntimeProcessor processor;
    Stream out;

    bool begin() {
        Lsm6dsvFifoReader::Config config;
        config.enableTimestampCounter = false;
        config.useHardwareTimestamps = false;
        config.timestampBatch = Lsm6dsvFifoReader::TimestampBatch::Off;
        config.temperatureBatch = Lsm6dsvFifoReader::TemperatureBatch::Off;
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
    f.probe.rawCallbackCostUs = 500;
    f.irqCount = 1;

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == 12);
    CHECK(ctx, f.bus.dataReads() == 1);
    CHECK(ctx, f.bus.remainingWords() == 0);

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == 24);
    CHECK(ctx, f.bus.dataReads() == 1);

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == 30);
    CHECK(ctx, f.bus.dataReads() == 1);

    // FIFO stats deltas are evaluated once per hardware drain, not once per
    // cooperative callback slice.
    CHECK(ctx, f.probe.statsDeltaCalls.size() == 1);
    CHECK(ctx, f.probe.statsDeltaCalls[0] == 1);
}

void testHardwareDrainTimeDoesNotConsumeCallbackBudget(TestContext& ctx) {
    trackerTestSetMicros(0);
    Fixture f;
    for (int16_t i = 0; i < 30; ++i) f.bus.addSample(i);
    CHECK(ctx, f.begin());
    f.bus.setDataReadCostUs(7000);
    f.probe.rawCallbackCostUs = 500;
    f.irqCount = 1;

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    // The 7 ms SPI read already exceeds the 4.5 ms callback budget. It must
    // not reduce this pass to one callback: the processor still guarantees
    // the minimum forward progress required to keep up with the sensor.
    CHECK(ctx, f.probe.rawCalls == 12);
    CHECK(ctx, f.bus.dataReads() == 1);
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
    f.probe.rawCallbackCostUs = 500;
    f.irqCount = 1;

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == 12);
    f.processor.resetWork();

    CHECK(ctx, !f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == 12);
}

void testRamQueueAbsorbsSecondHardwareBurst(TestContext& ctx) {
    trackerTestSetMicros(0);
    Fixture f;
    for (int16_t i = 0; i < 80; ++i) f.bus.addSample(i);
    CHECK(ctx, f.begin());
    f.probe.rawCallbackCostUs = 500;
    f.irqCount = 1;

    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == 12);
    CHECK(ctx, f.processor.rawQueueDepth() == 68);

    for (int16_t i = 80; i < 120; ++i) f.bus.addSample(i);
    CHECK(ctx, f.processor.process(12, 384, 6, f.out));
    CHECK(ctx, f.probe.rawCalls == 24);
    CHECK(ctx, f.processor.rawQueueDepth() == 96);
    CHECK(ctx, f.processor.queueStats().rawQueueHighWater >= 108);
    CHECK(ctx, f.processor.queueStats().rawQueueOverflow == 0);
}

} // namespace

int main() {
    TestContext ctx;
    testCallbacksAreSlicedAcrossAppPasses(ctx);
    testHardwareDrainTimeDoesNotConsumeCallbackBudget(ctx);
    testRecoveryDropsRemainderOfPredateBatch(ctx);
    testExternalResetDropsPendingBatch(ctx);
    testRamQueueAbsorbsSecondHardwareBurst(ctx);
    return ctx.finish("test_fifo_runtime_processor");
}
