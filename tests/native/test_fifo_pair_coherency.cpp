#include "test_common.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/imu_quality.hpp"

using namespace tracker;

namespace {

class PairingTransport final : public Lsm6dsvTransport {
public:
    using Word = std::array<uint8_t, Lsm6dsvFifoReader::FIFO_WORD_BYTES>;

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
        return true;
    }

    bool write(uint8_t, const uint8_t*, size_t) override { return true; }
    void delayMs(uint32_t) override {}

    void gyro(uint8_t counter, int16_t value) {
        words_.push_back(makeWord(Lsm6dsvFifoReader::TAG_GYRO_NC, counter,
                                  value, static_cast<int16_t>(value + 1), static_cast<int16_t>(value + 2)));
    }

    void accel(uint8_t counter, int16_t value) {
        words_.push_back(makeWord(Lsm6dsvFifoReader::TAG_ACCEL_NC, counter,
                                  value, static_cast<int16_t>(value + 1), static_cast<int16_t>(value + 2)));
    }

private:
    static Word makeWord(uint8_t sensorTag, uint8_t counter, int16_t x, int16_t y, int16_t z) {
        Word out{};
        out[0] = static_cast<uint8_t>((sensorTag << 3) | ((counter & 0x03u) << 1));
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

    std::vector<Word> words_;
    size_t readIndex_ = 0;
};

struct Fixture {
    PairingTransport bus;
    Lsm6dsv lsm{bus};
    Lsm6dsvFifoReader fifo{bus, lsm};

    bool begin() {
        Lsm6dsvFifoReader::Config cfg;
        cfg.enableTimestampCounter = false;
        cfg.useHardwareTimestamps = false;
        cfg.timestampBatch = Lsm6dsvFifoReader::TimestampBatch::Off;
        cfg.temperatureBatch = Lsm6dsvFifoReader::TemperatureBatch::Off;
        return fifo.configure(cfg);
    }

    size_t drain(Lsm6dsv::RawSample* out, size_t cap) {
        size_t count = 0;
        CHECK_LOCAL(fifo.drainRawSamples(out, cap, count, 1000000, 256));
        return count;
    }

private:
    static void CHECK_LOCAL(bool value) {
        if (!value) std::abort();
    }
};

void testNormalPairsRemainComplete(TestContext& ctx) {
    Fixture f;
    CHECK(ctx, f.begin());
    f.bus.gyro(0, 10); f.bus.accel(0, 100);
    f.bus.gyro(1, 20); f.bus.accel(1, 110);

    Lsm6dsv::RawSample out[8]{};
    const size_t count = f.drain(out, 8);
    CHECK(ctx, count == 2);
    CHECK(ctx, out[0].components == Lsm6dsv::SAMPLE_COMPONENT_COMPLETE);
    CHECK(ctx, out[0].coherency == Lsm6dsv::SampleCoherency::Coherent);
    CHECK(ctx, out[1].components == Lsm6dsv::SAMPLE_COMPONENT_COMPLETE);
    CHECK(ctx, f.fifo.stats().completePairsProduced == 2);
    CHECK(ctx, f.fifo.stats().gyroOnlySamplesProduced == 0);
}

void testDrainBoundaryDoesNotCreateFalseOrphan(TestContext& ctx) {
    Fixture f;
    CHECK(ctx, f.begin());
    f.bus.gyro(0, 10);

    Lsm6dsv::RawSample out[8]{};
    CHECK(ctx, f.drain(out, 8) == 0);

    f.bus.accel(0, 100);
    const size_t count = f.drain(out, 8);
    CHECK(ctx, count == 1);
    CHECK(ctx, out[0].components == Lsm6dsv::SAMPLE_COMPONENT_COMPLETE);
    CHECK(ctx, (out[0].flags & Lsm6dsvFifoReader::FIFO_FLAG_ORPHAN_WORDS) == 0u);
}

void testMissingAccelPreservesGyroContinuity(TestContext& ctx) {
    Fixture f;
    CHECK(ctx, f.begin());
    f.bus.gyro(0, 10);
    f.bus.gyro(1, 20);
    f.bus.accel(1, 110);

    Lsm6dsv::RawSample out[8]{};
    const size_t count = f.drain(out, 8);
    CHECK(ctx, count == 2);
    CHECK(ctx, out[0].gx == 10);
    CHECK(ctx, out[0].components == Lsm6dsv::SAMPLE_COMPONENT_GYRO);
    CHECK(ctx, out[0].coherency == Lsm6dsv::SampleCoherency::GyroOnly);
    CHECK(ctx, (out[0].flags & Lsm6dsvFifoReader::FIFO_FLAG_ORPHAN_WORDS) != 0u);
    CHECK(ctx, out[1].gx == 20);
    CHECK(ctx, out[1].ax == 110);
    CHECK(ctx, out[1].components == Lsm6dsv::SAMPLE_COMPONENT_COMPLETE);
    CHECK(ctx, f.fifo.stats().gyroPendingReplaced == 1);
    CHECK(ctx, f.fifo.stats().gyroOnlySamplesProduced == 1);

    ImuQualityMonitor quality;
    const Lsm6dsv::Sample scaled = f.lsm.scale(out[0]);
    const ImuQualityResult q = quality.evaluate(out[0], scaled, f.fifo.stats(), false);
    CHECK(ctx, q.shouldUpdateAhrs);
    CHECK(ctx, !q.shouldUseAccelCorrection);
    CHECK(ctx, !q.shouldRequestFifoRecovery);
    CHECK(ctx, q.has(imu_quality_flags::ACCEL_COMPONENT_MISSING));
}

void testRepeatedAccelKeepsNewestObservation(TestContext& ctx) {
    Fixture f;
    CHECK(ctx, f.begin());
    f.bus.accel(0, 100);
    f.bus.accel(1, 110);
    f.bus.gyro(1, 20);

    Lsm6dsv::RawSample out[8]{};
    const size_t count = f.drain(out, 8);
    CHECK(ctx, count == 1);
    CHECK(ctx, out[0].gx == 20);
    CHECK(ctx, out[0].ax == 110);
    CHECK(ctx, f.fifo.stats().accelPendingReplaced == 1);
    CHECK(ctx, f.fifo.stats().gyroOnlySamplesProduced == 0);
}

void testCounterMismatchDegradesOnlyAccel(TestContext& ctx) {
    Fixture f;
    CHECK(ctx, f.begin());
    for (uint8_t i = 0; i < 4; ++i) {
        f.bus.gyro(i, static_cast<int16_t>(10 + i));
        f.bus.accel(i, static_cast<int16_t>(100 + i));
    }
    f.bus.gyro(0, 30);
    f.bus.accel(3, 130); // learned offset 0, observed offset 1

    Lsm6dsv::RawSample out[12]{};
    const size_t count = f.drain(out, 12);
    CHECK(ctx, count == 5);
    CHECK(ctx, out[4].components == Lsm6dsv::SAMPLE_COMPONENT_COMPLETE);
    CHECK(ctx, out[4].coherency == Lsm6dsv::SampleCoherency::PairCounterMismatch);
    CHECK(ctx, f.fifo.stats().pairCounterOffsetLocks == 1);
    CHECK(ctx, f.fifo.stats().pairCounterMismatches == 1);

    ImuQualityMonitor quality;
    for (size_t i = 0; i < 4; ++i) {
        (void)quality.evaluate(out[i], f.lsm.scale(out[i]), f.fifo.stats(), false);
    }
    const ImuQualityResult q = quality.evaluate(out[4], f.lsm.scale(out[4]), f.fifo.stats(), false);
    CHECK(ctx, q.shouldUpdateAhrs);
    CHECK(ctx, !q.shouldUseAccelCorrection);
    CHECK(ctx, !q.shouldRequestFifoRecovery);
    CHECK(ctx, q.has(imu_quality_flags::FIFO_PAIR_DEGRADED));
}

} // namespace

int main() {
    TestContext ctx;
    testNormalPairsRemainComplete(ctx);
    testDrainBoundaryDoesNotCreateFalseOrphan(ctx);
    testMissingAccelPreservesGyroContinuity(ctx);
    testRepeatedAccelKeepsNewestObservation(ctx);
    testCounterMismatchDegradesOnlyAccel(ctx);
    return ctx.finish("test_fifo_pair_coherency");
}
