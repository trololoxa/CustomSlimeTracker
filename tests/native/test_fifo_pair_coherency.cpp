#include "test_common.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/imu_quality.hpp"
#include "sensor/qmc6309.hpp"

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

    bool write(uint8_t reg, const uint8_t* src, size_t len) override {
        if (src == nullptr || len == 0) return false;
        writes_.push_back({reg, src[0]});
        return true;
    }
    void delayMs(uint32_t) override {}

    void gyro(uint8_t counter, int16_t value) {
        words_.push_back(makeWord(Lsm6dsvFifoReader::TAG_GYRO_NC, counter,
                                  value, static_cast<int16_t>(value + 1), static_cast<int16_t>(value + 2)));
    }

    void accel(uint8_t counter, int16_t value) {
        words_.push_back(makeWord(Lsm6dsvFifoReader::TAG_ACCEL_NC, counter,
                                  value, static_cast<int16_t>(value + 1), static_cast<int16_t>(value + 2)));
    }

    void sensorHub(uint8_t counter, int16_t x, int16_t y, int16_t z) {
        words_.push_back(makeWord(Lsm6dsvFifoReader::TAG_SENSORHUB_SLAVE0, counter, x, y, z));
    }

    void timestamp(uint32_t ticks) {
        words_.push_back(makeWord(
            Lsm6dsvFifoReader::TAG_TIMESTAMP, 0u,
            static_cast<int16_t>(ticks & 0xFFFFu),
            static_cast<int16_t>((ticks >> 16) & 0xFFFFu),
            0));
    }

    void clearWrites() { writes_.clear(); }
    const std::vector<std::array<uint8_t, 2>>& writes() const { return writes_; }

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
    std::vector<std::array<uint8_t, 2>> writes_;
};

struct Fixture {
    PairingTransport bus;
    Lsm6dsv lsm{bus};
    Lsm6dsvFifoReader fifo{bus, lsm};

    bool begin(uint16_t sensorHubSaturationAbs = 0u,
               bool enableSensorHub = false,
               float sensorHubPeriodUs = 0.0f,
               bool useHardwareTimestamps = false) {
        Lsm6dsvFifoReader::Config cfg;
        cfg.enableTimestampCounter = useHardwareTimestamps;
        cfg.useHardwareTimestamps = useHardwareTimestamps;
        cfg.timestampBatch = useHardwareTimestamps
            ? Lsm6dsvFifoReader::TimestampBatch::Decimation1
            : Lsm6dsvFifoReader::TimestampBatch::Off;
        cfg.temperatureBatch = Lsm6dsvFifoReader::TemperatureBatch::Off;
        cfg.enableSensorHubSlave0 = enableSensorHub || sensorHubSaturationAbs != 0u;
        cfg.sensorHubSlave0PeriodUs = sensorHubPeriodUs;
        cfg.sensorHubSlave0SaturationAbs = sensorHubSaturationAbs;
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



void testCompletedQueueOverflowIsAccountedSeparately(TestContext& ctx) {
    Fixture f;
    CHECK(ctx, f.begin());
    for (uint16_t i = 0; i < 130; ++i) {
        const uint8_t counter = static_cast<uint8_t>(i & 0x03u);
        f.bus.gyro(counter, static_cast<int16_t>(10 + i));
        f.bus.accel(counter, static_cast<int16_t>(100 + i));
    }

    Lsm6dsv::RawSample out[1]{};
    size_t count = 0;
    CHECK(ctx, f.fifo.drainRawSamples(out, 1, count, 1000000, 512));
    CHECK(ctx, count == 1);
    CHECK(ctx, f.fifo.stats().completedSampleQueueOverflow == 2);
    CHECK(ctx, f.fifo.stats().waitingSampleQueueOverflow == 0);
}


void testPausePreservesConfiguredFifoRegisters(TestContext& ctx) {
    PairingTransport bus;
    Lsm6dsv lsm(bus);
    Lsm6dsvFifoReader fifo(bus, lsm);

    Lsm6dsvFifoReader::Config cfg;
    cfg.enableTimestampCounter = false;
    cfg.timestampBatch = Lsm6dsvFifoReader::TimestampBatch::Decimation1;
    cfg.temperatureBatch = Lsm6dsvFifoReader::TemperatureBatch::Hz1_875;
    cfg.mode = Lsm6dsvFifoReader::FifoMode::Continuous;
    CHECK(ctx, fifo.configure(cfg));

    bus.clearWrites();
    CHECK(ctx, fifo.pauseFifo());
    CHECK(ctx, bus.writes().size() == 1);
    CHECK(ctx, bus.writes()[0][0] == 0x0A);
    CHECK(ctx, bus.writes()[0][1] == 0x00);

    CHECK(ctx, fifo.resetFifo());
    CHECK(ctx, bus.writes().size() == 3);
    CHECK(ctx, bus.writes()[1][0] == 0x0A);
    CHECK(ctx, bus.writes()[1][1] == 0x00);
    CHECK(ctx, bus.writes()[2][0] == 0x0A);
    CHECK(ctx, bus.writes()[2][1] != 0x00);

    // No pause/resume write may destroy watermark or accel/gyro BDR registers.
    for (const auto& write : bus.writes()) {
        CHECK(ctx, write[0] != 0x07);
        CHECK(ctx, write[0] != 0x08);
        CHECK(ctx, write[0] != 0x09);
    }
}


void testSensorHubTimestampsReanchorToImuTimeline(TestContext& ctx) {
    Fixture f;
    CHECK(ctx, f.begin(0u, true, 1000000.0f / 60.0f));

    // Model a real sensor-hub cadence that differs from the nominal 60 Hz
    // period. A free-running mag clock would accumulate more than 5 ms of
    // gyro/mag skew after only a few frames; every frame must instead anchor
    // to the current FIFO/IMU timeline.
    uint8_t counter = 0u;
    for (uint8_t magIndex = 0u; magIndex < 6u; ++magIndex) {
        for (uint8_t i = 0u; i < 18u; ++i) {
            f.bus.gyro(counter, static_cast<int16_t>(100 + magIndex * 20 + i));
            f.bus.accel(counter, static_cast<int16_t>(200 + magIndex * 20 + i));
            counter = static_cast<uint8_t>((counter + 1u) & 0x03u);
        }
        f.bus.sensorHub(magIndex & 0x03u,
                        static_cast<int16_t>(1000 + magIndex),
                        static_cast<int16_t>(-500 - magIndex),
                        static_cast<int16_t>(250 + magIndex));
    }

    Lsm6dsv::RawSample out[128]{};
    CHECK(ctx, f.drain(out, 128) == 108u);

    Lsm6dsvFifoReader::MagRawSample mags[6]{};
    for (auto& mag : mags) {
        CHECK(ctx, f.fifo.popMagSample(mag));
        CHECK(ctx, (mag.flags & Lsm6dsvFifoReader::MAG_FLAG_TIMESTAMP_IMU_ANCHORED) != 0u);
    }
    CHECK(ctx, !f.fifo.hasMagSamples());

    const uint64_t nominalLast = mags[0].t_us + 5u * 16667u;
    CHECK(ctx, mags[5].t_us > nominalLast + 5000u);
    for (uint8_t i = 1u; i < 6u; ++i) {
        const uint64_t dtUs = mags[i].t_us - mags[i - 1u].t_us;
        CHECK(ctx, dtUs >= 18000u);
        CHECK(ctx, dtUs <= 19500u);
    }
    CHECK(ctx, f.fifo.stats().magTimestampImuAnchors == 6u);
    CHECK(ctx, f.fifo.stats().magTimestampNominalFallbacks == 0u);
    CHECK(ctx, f.fifo.stats().magTimestampMonotonicAdjustments == 0u);
    CHECK(ctx, f.fifo.stats().maxMagAnchorCorrectionUs < 5000u);
}


void testSensorHubTimestampsUseHardwareImuAnchor(TestContext& ctx) {
    Fixture f;
    CHECK(ctx, f.begin(0u, true, 1000000.0f / 60.0f, true));

    for (uint8_t i = 0u; i < 4u; ++i) {
        f.bus.gyro(i & 0x03u, static_cast<int16_t>(10 + i));
        f.bus.accel(i & 0x03u, static_cast<int16_t>(100 + i));
        f.bus.timestamp(static_cast<uint32_t>(1000u + i * 700u));
        f.bus.sensorHub(i & 0x03u, static_cast<int16_t>(500 + i), 20, -30);
    }

    Lsm6dsv::RawSample raw[8]{};
    CHECK(ctx, f.drain(raw, 8) == 4u);
    for (uint8_t i = 0u; i < 4u; ++i) {
        Lsm6dsvFifoReader::MagRawSample mag{};
        CHECK(ctx, f.fifo.popMagSample(mag));
        CHECK(ctx, mag.t_us == raw[i].t_us);
        CHECK(ctx, (mag.flags & Lsm6dsvFifoReader::MAG_FLAG_TIMESTAMP_IMU_ANCHORED) != 0u);
        CHECK(ctx, (mag.flags & Lsm6dsvFifoReader::MAG_FLAG_TIMESTAMP_FALLBACK) == 0u);
    }
    CHECK(ctx, f.fifo.stats().magTimestampImuAnchors == 4u);
}


void testSensorHubRepeatedAnchorUsesFailHonestMonotonicMarker(TestContext& ctx) {
    Fixture f;
    CHECK(ctx, f.begin(0u, true, 1000000.0f / 60.0f));

    f.bus.gyro(0u, 10);
    f.bus.accel(0u, 20);
    f.bus.sensorHub(0u, 100, 200, 300);
    // A second external-sensor word without any intervening IMU timeline
    // advance has no honest timestamp. It must not fabricate a full nominal
    // period; mark a minimal monotonic value that interval admission rejects.
    f.bus.sensorHub(1u, 101, 201, 301);

    Lsm6dsv::RawSample raw[2]{};
    CHECK(ctx, f.drain(raw, 2) == 1u);
    Lsm6dsvFifoReader::MagRawSample first{};
    Lsm6dsvFifoReader::MagRawSample second{};
    CHECK(ctx, f.fifo.popMagSample(first));
    CHECK(ctx, f.fifo.popMagSample(second));
    CHECK(ctx, second.t_us == first.t_us + 1u);
    CHECK(ctx, (first.flags & Lsm6dsvFifoReader::MAG_FLAG_TIMESTAMP_IMU_ANCHORED) != 0u);
    CHECK(ctx, (second.flags & Lsm6dsvFifoReader::MAG_FLAG_TIMESTAMP_FALLBACK) != 0u);
    CHECK(ctx, f.fifo.stats().magTimestampMonotonicAdjustments == 1u);
    CHECK(ctx, f.fifo.stats().magTimestampNominalFallbacks == 0u);
}

void testSensorHubNearRailSaturationIsFlagged(TestContext& ctx) {
    Fixture f;
    CHECK(ctx, f.begin(Qmc6309::RAW_SATURATION_ABS_COUNTS));

    f.bus.sensorHub(0, 31800, -1200, 900);
    f.bus.sensorHub(1, 31950, 200, -300);

    Lsm6dsv::RawSample out[1]{};
    CHECK(ctx, f.drain(out, 1) == 0);

    Lsm6dsvFifoReader::MagRawSample mag{};
    CHECK(ctx, f.fifo.popMagSample(mag));
    CHECK(ctx, (mag.flags & Lsm6dsvFifoReader::MAG_FLAG_RAW_SATURATED) == 0u);
    CHECK(ctx, f.fifo.popMagSample(mag));
    CHECK(ctx, (mag.flags & Lsm6dsvFifoReader::MAG_FLAG_RAW_SATURATED) != 0u);
    CHECK(ctx, f.fifo.stats().magRawSaturationCount == 1u);
}

void testFirstFallbackTimestampUsesDrainClock(TestContext& ctx) {
    PairingTransport bus;
    Lsm6dsv lsm(bus);
    Lsm6dsvFifoReader fifo(bus, lsm);

    Lsm6dsvFifoReader::Config cfg;
    cfg.enableTimestampCounter = false;
    cfg.timestampBatch = Lsm6dsvFifoReader::TimestampBatch::Off;
    cfg.temperatureBatch = Lsm6dsvFifoReader::TemperatureBatch::Off;
    cfg.useHardwareTimestamps = true;
    cfg.allowTimestampFallback = true;
    cfg.maxWaitingSamplesBeforeFallback = 0;
    CHECK(ctx, fifo.configure(cfg));

    bus.gyro(0, 10);
    bus.accel(0, 100);

    constexpr uint64_t drainTimestampUs = 1000000ULL;
    Lsm6dsv::RawSample out[2]{};
    size_t count = 0;
    CHECK(ctx, fifo.drainRawSamples(out, 2, count, drainTimestampUs, 16));
    CHECK(ctx, count == 1);
    CHECK(ctx, out[0].t_us == drainTimestampUs);
    CHECK(ctx, (out[0].flags & Lsm6dsvFifoReader::FIFO_FLAG_TS_FALLBACK) != 0u);
}

} // namespace

int main() {
    TestContext ctx;
    testNormalPairsRemainComplete(ctx);
    testDrainBoundaryDoesNotCreateFalseOrphan(ctx);
    testMissingAccelPreservesGyroContinuity(ctx);
    testRepeatedAccelKeepsNewestObservation(ctx);
    testCounterMismatchDegradesOnlyAccel(ctx);
    testCompletedQueueOverflowIsAccountedSeparately(ctx);
    testPausePreservesConfiguredFifoRegisters(ctx);
    testSensorHubTimestampsReanchorToImuTimeline(ctx);
    testSensorHubTimestampsUseHardwareImuAnchor(ctx);
    testSensorHubRepeatedAnchorUsesFailHonestMonotonicMarker(ctx);
    testSensorHubNearRailSaturationIsFlagged(ctx);
    testFirstFallbackTimestampUsesDrainClock(ctx);
    return ctx.finish("test_fifo_pair_coherency");
}
