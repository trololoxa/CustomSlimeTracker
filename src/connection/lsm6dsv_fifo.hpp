#pragma once

#include <cstddef>
#include <cstdint>
#include <cmath>

#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"

namespace tracker {

// ============================================================
// LSM6DSV FIFO reader v2
// ============================================================
// Production-oriented FIFO layer for robust IMU pipeline:
//   - accel + gyro FIFO batching
//   - FIFO INT1 watermark / overrun / full routing
//   - FIFO tag parser
//   - FIFO timestamp tag parsing
//   - FIFO temperature tag parsing
//   - tag-counter diagnostics
//   - saturation counters
//   - dropped/overflow diagnostics
//   - fallback timestamp reconstruction when hardware timestamp is missing
//
// LSM6DSV FIFO word = 1 tag byte + 6 data bytes.
// TAG_SENSOR_[4:0] is stored in FIFO_DATA_OUT_TAG[7:3].
// TAG_CNT_[1:0] is stored in FIFO_DATA_OUT_TAG[2:1].
// ============================================================

class Lsm6dsvFifoReader {
public:
    static constexpr uint8_t TAG_EMPTY       = 0x00;
    static constexpr uint8_t TAG_GYRO_NC     = 0x01;
    static constexpr uint8_t TAG_ACCEL_NC    = 0x02;
    static constexpr uint8_t TAG_TEMPERATURE = 0x03;
    static constexpr uint8_t TAG_TIMESTAMP   = 0x04;
    static constexpr uint8_t TAG_CFG_CHANGE  = 0x05;

    // One FIFO entry is one TAG byte followed by three little-endian 16-bit axis words.
    // LSM6DSV supports multiple-byte reads from FIFO_DATA_OUT_TAG; after FIFO_DATA_OUT_Z_H
    // the internal FIFO output pointer wraps back to FIFO_DATA_OUT_TAG for the next entry.
    static constexpr size_t FIFO_WORD_BYTES = 7;
    static constexpr uint16_t FIFO_BURST_WORD_CAPACITY = 64;

    // External sensor hub FIFO tags. QMC6309 arrives here when SLV0 is
    // configured with BATCH_EXT_SENS_0_EN=1. TAG_SENSORHUB_NACK indicates
    // that the sensor hub saw a NACK during an auxiliary I2C transaction.
    static constexpr uint8_t TAG_SENSORHUB_SLAVE0 = 0x0E;
    static constexpr uint8_t TAG_SENSORHUB_NACK   = 0x19;

    static constexpr uint16_t FIFO_FLAG_SOURCE_FIFO       = 1u << 8;
    static constexpr uint16_t FIFO_FLAG_STATUS_WTM        = 1u << 9;
    static constexpr uint16_t FIFO_FLAG_STATUS_OVR        = 1u << 10;
    static constexpr uint16_t FIFO_FLAG_STATUS_FULL       = 1u << 11;
    static constexpr uint16_t FIFO_FLAG_UNKNOWN_TAG       = 1u << 12;
    static constexpr uint16_t FIFO_FLAG_ORPHAN_WORDS      = 1u << 13;
    static constexpr uint16_t FIFO_FLAG_TS_FALLBACK       = 1u << 14;
    static constexpr uint16_t FIFO_FLAG_TS_HARDWARE       = 1u << 15;

    enum class FifoMode : uint8_t {
        Bypass = 0x00,
        Fifo = 0x01,
        ContinuousWtmToFull = 0x02,
        ContinuousToFifo = 0x03,
        BypassToContinuous = 0x04,
        Continuous = 0x06,
        BypassToFifo = 0x07,
    };

    enum class TimestampBatch : uint8_t {
        Off = 0x00,
        Decimation1 = 0x01,
        Decimation8 = 0x02,
        Decimation32 = 0x03,
    };

    enum class TemperatureBatch : uint8_t {
        Off = 0x00,
        Hz1_875 = 0x01,
        Hz15 = 0x02,
        Hz60 = 0x03,
    };

    struct Config {
        Lsm6dsv::Odr accelBdr = Lsm6dsv::Odr::Hz960;
        Lsm6dsv::Odr gyroBdr  = Lsm6dsv::Odr::Hz960;

        // FIFO watermark is in FIFO words, not IMU samples.
        // With gyro + accel + timestamp per sample: 48 words ~= 16 IMU samples.
        uint8_t watermarkWords = 48;

        FifoMode mode = FifoMode::Continuous;

        bool routeWatermarkToInt1 = true;
        bool routeOverrunToInt1 = true;
        bool routeFullToInt1 = true;

        bool routeWatermarkToInt2 = false;
        bool routeOverrunToInt2 = false;
        bool routeFullToInt2 = false;

        // New robust scheme.
        TimestampBatch timestampBatch = TimestampBatch::Decimation1;
        TemperatureBatch temperatureBatch = TemperatureBatch::Hz1_875;
        bool enableTimestampCounter = true;
        bool useHardwareTimestamps = true;

        // If timestamp tags are missing or delayed too far, fallback keeps the
        // stream alive and marks samples with FIFO_FLAG_TS_FALLBACK.
        bool allowTimestampFallback = true;
        uint8_t maxWaitingSamplesBeforeFallback = 32;

        // If 0, computed from ODR + INTERNAL_FREQ_FINE. Override only for debug.
        float samplePeriodUsOverride = 0.0f;

        // Parser support for external sensor hub slave 0 data. This does not
        // enable the LSM sensor hub by itself; Qmc6309::armHubFifoRead() does
        // that. This only tells the FIFO parser not to count tag 0x0E as
        // unknown and to queue it as MagRawSample.
        bool enableSensorHubSlave0 = false;

        // Used only for approximate mag timestamps. LSM6DSV timestamp tags are
        // attached to FIFO timing, not directly to the external sensor word.
        // For SHUB_ODR=60 Hz use 16666.666f. If 0, drainTimestampUs is used.
        float sensorHubSlave0PeriodUs = 0.0f;
    };

    struct Status {
        uint16_t unreadWords = 0;
        uint8_t rawStatus1 = 0;
        uint8_t rawStatus2 = 0;
        bool watermark = false;
        bool overrun = false;
        bool full = false;
        bool counterBdr = false;
        bool overrunLatched = false;
    };

    struct FifoWord {
        uint8_t rawTag = 0;
        uint8_t tagSensor = 0;
        uint8_t tagCounter = 0;
        int16_t x = 0;
        int16_t y = 0;
        int16_t z = 0;
    };

    struct MagRawSample {
        uint64_t t_us = 0;
        int16_t x = 0;
        int16_t y = 0;
        int16_t z = 0;
        uint8_t rawTag = 0;
        uint8_t tagCounter = 0;
        uint16_t flags = 0;
        uint32_t seq = 0;
    };

    enum MagFlags : uint16_t {
        MAG_FLAG_NONE = 0,
        MAG_FLAG_FROM_SENSORHUB_SLAVE0 = 1u << 0,
        MAG_FLAG_TIMESTAMP_FALLBACK = 1u << 1,
        MAG_FLAG_QUEUE_OVERFLOW = 1u << 2,
        MAG_FLAG_RAW_SATURATED = 1u << 3,
    };

    struct DrainStats {
        uint32_t drainCalls = 0;
        uint32_t fifoWordsRead = 0;
        uint32_t fifoBurstReads = 0;
        uint32_t fifoBurstReadWords = 0;
        uint32_t maxBurstWordsRead = 0;
        uint32_t imuSamplesProduced = 0;
        uint32_t gyroWords = 0;
        uint32_t accelWords = 0;
        uint32_t tempWords = 0;
        uint32_t timestampWords = 0;
        uint32_t cfgChangeWords = 0;
        uint32_t emptyWords = 0;
        uint32_t unknownWords = 0;

        uint32_t sensorHubSlave0Words = 0;
        uint32_t sensorHubNackWords = 0;
        uint32_t magSamplesProduced = 0;
        uint32_t magQueueOverflow = 0;
        uint32_t magTagCounterJumps = 0;
        uint32_t magRawSaturationCount = 0;
        uint64_t lastMagTimestampUs = 0;
        uint32_t lastMagDtUs = 0;
        uint32_t minMagDtUs = 0;
        uint32_t maxMagDtUs = 0;
        double sumMagDtUs = 0.0;
        uint32_t magDtCount = 0;
        int16_t lastMagX = 0;
        int16_t lastMagY = 0;
        int16_t lastMagZ = 0;
        float lastMagRawNorm = 0.0f;

        uint32_t overrunEvents = 0;
        uint32_t fullEvents = 0;
        uint32_t watermarkEvents = 0;
        uint32_t maxUnreadWordsSeen = 0;

        uint32_t accelSaturationCount = 0;
        uint32_t gyroSaturationCount = 0;

        // Tag counter diagnostics.
        uint32_t tagCounterJumps = 0;
        uint32_t gyroTagCounterJumps = 0;
        uint32_t accelTagCounterJumps = 0;

        // Timestamp diagnostics.
        uint32_t hwTimestampAssigned = 0;
        uint32_t fallbackTimestampAssigned = 0;
        uint32_t timestampQueueOverflow = 0;
        uint32_t waitingSampleQueueOverflow = 0;
        uint32_t timestampBackwards = 0;
        uint32_t timestampWraps = 0;
        uint32_t timestampDuplicate = 0;
        uint32_t timestampLargeGap = 0;

        uint32_t timestampMetaBdrXlMismatch = 0;
        uint32_t timestampMetaBdrGyMismatch = 0;

        uint64_t lastAssignedTimestampUs = 0;
        uint64_t lastHwTimestampUs = 0;
        uint32_t lastRawTimestampTicks = 0;
        uint64_t timestampWrapHigh = 0;

        float timestampTickUs = 0.0f;
        float samplePeriodUs = 0.0f;
        int8_t internalFreqFine = 0;

        float latestTempC = 25.0f;
        bool latestTempValid = false;
    };

    Lsm6dsvFifoReader(Lsm6dsvTransport& bus, const Lsm6dsv& lsm)
        : bus_(bus), lsm_(lsm) {}

    bool configure(const Config& config) {
        cfg_ = config;
        stats_ = DrainStats{};
        configured_ = false;
        resetParserState();

        if (!readInternalFreqFine()) {
            // Not fatal; use nominal timing.
            stats_.internalFreqFine = 0;
        }

        stats_.timestampTickUs = timestampTickUsFromFine(stats_.internalFreqFine);
        stats_.samplePeriodUs = configuredSamplePeriodUs();

        if (cfg_.enableTimestampCounter) {
            if (!enableTimestampCounter()) return false;
        }

        // Reset FIFO by bypass mode first.
        if (!writeReg(REG_FIFO_CTRL4, 0x00)) return false;
        bus_.delayMs(2);

        if (!writeReg(REG_FIFO_CTRL1, cfg_.watermarkWords)) return false;

        // Compression off, stop-on-watermark off, ODR-change batching off.
        if (!writeReg(REG_FIFO_CTRL2, 0x00)) return false;

        const uint8_t bdrGy = odrToBdrNibble(cfg_.gyroBdr);
        const uint8_t bdrXl = odrToBdrNibble(cfg_.accelBdr);
        const uint8_t fifoCtrl3 = static_cast<uint8_t>((bdrGy << 4) | bdrXl);
        if (!writeReg(REG_FIFO_CTRL3, fifoCtrl3)) return false;

        uint8_t fifoCtrl4 = 0;
        fifoCtrl4 |= (static_cast<uint8_t>(cfg_.timestampBatch) & 0x03u) << 6;
        fifoCtrl4 |= (static_cast<uint8_t>(cfg_.temperatureBatch) & 0x03u) << 4;
        fifoCtrl4 |= static_cast<uint8_t>(cfg_.mode) & 0x07u;
        if (!writeReg(REG_FIFO_CTRL4, fifoCtrl4)) return false;

        if (!configureFifoInterrupts()) return false;

        configured_ = true;
        return true;
    }

    bool resetFifo() {
        if (!writeReg(REG_FIFO_CTRL4, 0x00)) return false;
        bus_.delayMs(2);

        resetParserState();
        stats_.lastAssignedTimestampUs = 0;
        stats_.lastHwTimestampUs = 0;
        stats_.lastRawTimestampTicks = 0;
        stats_.timestampWrapHigh = 0;

        uint8_t fifoCtrl4 = 0;
        fifoCtrl4 |= (static_cast<uint8_t>(cfg_.timestampBatch) & 0x03u) << 6;
        fifoCtrl4 |= (static_cast<uint8_t>(cfg_.temperatureBatch) & 0x03u) << 4;
        fifoCtrl4 |= static_cast<uint8_t>(cfg_.mode) & 0x07u;
        return writeReg(REG_FIFO_CTRL4, fifoCtrl4);
    }

    bool isConfigured() const { return configured_; }
    float samplePeriodUs() const { return stats_.samplePeriodUs; }
    float timestampTickUs() const { return stats_.timestampTickUs; }
    const DrainStats& stats() const { return stats_; }

    bool hasMagSamples() const { return magCount_ > 0; }

    bool popMagSample(MagRawSample& out) {
        if (magCount_ == 0) return false;
        out = magSamples_[magHead_];
        magHead_ = (magHead_ + 1) % MAG_SAMPLE_CAP;
        magCount_--;
        return true;
    }

    size_t popMagSamples(MagRawSample* out, size_t capacity) {
        if (out == nullptr || capacity == 0) return 0;
        size_t n = 0;
        while (n < capacity && popMagSample(out[n])) {
            n++;
        }
        return n;
    }

    void resetTimestampReconstruction(uint64_t lastTimestampUs = 0) {
        stats_.lastAssignedTimestampUs = lastTimestampUs;
        stats_.lastHwTimestampUs = 0;
        stats_.lastRawTimestampTicks = 0;
        stats_.timestampWrapHigh = 0;
        tsHead_ = tsTail_ = tsCount_ = 0;
        waitingHead_ = waitingTail_ = waitingCount_ = 0;
        completedHead_ = completedTail_ = completedCount_ = 0;
    }

    bool readStatus(Status& s) {
        uint8_t b[2] = {};
        if (!bus_.read(REG_FIFO_STATUS1, b, sizeof(b))) return false;

        s.rawStatus1 = b[0];
        s.rawStatus2 = b[1];
        s.unreadWords = static_cast<uint16_t>(b[0]) |
                        static_cast<uint16_t>((b[1] & 0x01u) << 8);
        s.watermark = (b[1] & STATUS2_FIFO_WTM_IA) != 0;
        s.overrun = (b[1] & STATUS2_FIFO_OVR_IA) != 0;
        s.full = (b[1] & STATUS2_FIFO_FULL_IA) != 0;
        s.counterBdr = (b[1] & STATUS2_COUNTER_BDR_IA) != 0;
        s.overrunLatched = (b[1] & STATUS2_FIFO_OVR_LATCHED) != 0;

        if (s.watermark) stats_.watermarkEvents++;
        if (s.overrun || s.overrunLatched) stats_.overrunEvents++;
        if (s.full) stats_.fullEvents++;
        if (s.unreadWords > stats_.maxUnreadWordsSeen) stats_.maxUnreadWordsSeen = s.unreadWords;
        return true;
    }

    bool readWord(FifoWord& w) {
        uint8_t b[FIFO_WORD_BYTES] = {};
        if (!bus_.read(REG_FIFO_DATA_OUT_TAG, b, sizeof(b))) return false;
        parseWordBytes(b, w);
        stats_.fifoWordsRead++;
        return true;
    }

    bool readWords(FifoWord* out, uint16_t count) {
        if (out == nullptr || count == 0) return false;

        uint16_t done = 0;
        while (done < count) {
            uint16_t chunkWords = static_cast<uint16_t>(count - done);
            if (chunkWords > FIFO_BURST_WORD_CAPACITY) {
                chunkWords = FIFO_BURST_WORD_CAPACITY;
            }

            if (!readWordsToBurstBuffer(chunkWords)) return false;

            for (uint16_t i = 0; i < chunkWords; ++i) {
                parseWordBytes(&fifoBurstBytes_[static_cast<size_t>(i) * FIFO_WORD_BYTES], out[done + i]);
            }

            done = static_cast<uint16_t>(done + chunkWords);
        }

        return true;
    }

    bool drainRawSamples(Lsm6dsv::RawSample* out,
                         size_t capacity,
                         size_t& outCount,
                         uint64_t drainTimestampUs,
                         uint16_t maxWordsToRead = 256) {
        outCount = 0;
        if (out == nullptr || capacity == 0) return false;

        popCompletedToOutput(out, capacity, outCount);
        if (outCount >= capacity) return true;

        Status status;
        if (!readStatus(status)) return false;
        if (status.unreadWords == 0) return true;

        stats_.drainCalls++;

        uint16_t wordsToRead = status.unreadWords;
        if (wordsToRead > maxWordsToRead) wordsToRead = maxWordsToRead;

        uint16_t statusFlags = FIFO_FLAG_SOURCE_FIFO;
        if (status.watermark) statusFlags |= FIFO_FLAG_STATUS_WTM;
        if (status.overrun || status.overrunLatched) statusFlags |= FIFO_FLAG_STATUS_OVR;
        if (status.full) statusFlags |= FIFO_FLAG_STATUS_FULL;

        uint16_t wordsRead = 0;
        while (wordsRead < wordsToRead) {
            uint16_t chunkWords = static_cast<uint16_t>(wordsToRead - wordsRead);
            if (chunkWords > FIFO_BURST_WORD_CAPACITY) {
                chunkWords = FIFO_BURST_WORD_CAPACITY;
            }

            if (!readWordsToBurstBuffer(chunkWords)) return false;

            for (uint16_t i = 0; i < chunkWords; ++i) {
                FifoWord w;
                parseWordBytes(&fifoBurstBytes_[static_cast<size_t>(i) * FIFO_WORD_BYTES], w);
                processWord(w, statusFlags, drainTimestampUs);
            }

            wordsRead = static_cast<uint16_t>(wordsRead + chunkWords);
        }

        if (pendingGyroValid_ != pendingAccelValid_) {
            pendingFlags_ |= FIFO_FLAG_ORPHAN_WORDS;
        }

        if (cfg_.allowTimestampFallback && waitingCount_ > cfg_.maxWaitingSamplesBeforeFallback) {
            fallbackWaitingSamples(waitingCount_ - cfg_.maxWaitingSamplesBeforeFallback);
        }

        popCompletedToOutput(out, capacity, outCount);
        return true;
    }

private:
    static constexpr uint8_t REG_FIFO_CTRL1 = 0x07;
    static constexpr uint8_t REG_FIFO_CTRL2 = 0x08;
    static constexpr uint8_t REG_FIFO_CTRL3 = 0x09;
    static constexpr uint8_t REG_FIFO_CTRL4 = 0x0A;
    static constexpr uint8_t REG_INT1_CTRL = 0x0D;
    static constexpr uint8_t REG_INT2_CTRL = 0x0E;
    static constexpr uint8_t REG_FIFO_STATUS1 = 0x1B;
    static constexpr uint8_t REG_FUNC_EN = 0x50;
    static constexpr uint8_t REG_INTERNAL_FREQ_FINE = 0x4F;
    static constexpr uint8_t REG_FIFO_DATA_OUT_TAG = 0x78;

    static constexpr uint8_t STATUS2_FIFO_WTM_IA = 1u << 7;
    static constexpr uint8_t STATUS2_FIFO_OVR_IA = 1u << 6;
    static constexpr uint8_t STATUS2_FIFO_FULL_IA = 1u << 5;
    static constexpr uint8_t STATUS2_COUNTER_BDR_IA = 1u << 4;
    static constexpr uint8_t STATUS2_FIFO_OVR_LATCHED = 1u << 3;

    static constexpr uint8_t INT_FIFO_FULL = 1u << 5;
    static constexpr uint8_t INT_FIFO_OVR  = 1u << 4;
    static constexpr uint8_t INT_FIFO_TH   = 1u << 3;

    static constexpr uint8_t FUNC_EN_TIMESTAMP = 1u << 6;

    static constexpr size_t TIMESTAMP_QUEUE_CAP = 64;
    static constexpr size_t WAITING_SAMPLE_CAP = 96;
    static constexpr size_t COMPLETED_SAMPLE_CAP = 128;
    static constexpr size_t MAG_SAMPLE_CAP = 96;

    bool configureFifoInterrupts() {
        uint8_t int1 = 0;
        if (cfg_.routeWatermarkToInt1) int1 |= INT_FIFO_TH;
        if (cfg_.routeOverrunToInt1)   int1 |= INT_FIFO_OVR;
        if (cfg_.routeFullToInt1)      int1 |= INT_FIFO_FULL;
        if (!writeReg(REG_INT1_CTRL, int1)) return false;

        uint8_t int2 = 0;
        if (cfg_.routeWatermarkToInt2) int2 |= INT_FIFO_TH;
        if (cfg_.routeOverrunToInt2)   int2 |= INT_FIFO_OVR;
        if (cfg_.routeFullToInt2)      int2 |= INT_FIFO_FULL;
        return writeReg(REG_INT2_CTRL, int2);
    }

    bool readInternalFreqFine() {
        uint8_t raw = 0;
        if (!bus_.read(REG_INTERNAL_FREQ_FINE, &raw, 1)) return false;
        stats_.internalFreqFine = static_cast<int8_t>(raw);
        return true;
    }

    bool enableTimestampCounter() {
        uint8_t v = 0;
        if (!bus_.read(REG_FUNC_EN, &v, 1)) return false;
        v |= FUNC_EN_TIMESTAMP;
        return writeReg(REG_FUNC_EN, v);
    }

    float configuredSamplePeriodUs() const {
        if (cfg_.samplePeriodUsOverride > 0.0f) return cfg_.samplePeriodUsOverride;

        const float gyroHz = actualOdrHz(cfg_.gyroBdr, stats_.internalFreqFine);
        const float accelHz = actualOdrHz(cfg_.accelBdr, stats_.internalFreqFine);
        const float hz = gyroHz > accelHz ? gyroHz : accelHz;
        if (hz <= 0.0f) return 0.0f;
        return 1000000.0f / hz;
    }

    static float timestampTickUsFromFine(int8_t fine) {
        const float scale = 1.0f + 0.0013f * static_cast<float>(fine);
        return 1000000.0f / (46080.0f * scale);
    }

    static float actualOdrHz(Lsm6dsv::Odr odr, int8_t fine) {
        const float scale = 1.0f + 0.0013f * static_cast<float>(fine);
        return Lsm6dsv::odrHz(odr) * scale;
    }

    static uint8_t odrToBdrNibble(Lsm6dsv::Odr odr) {
        return static_cast<uint8_t>(odr) & 0x0Fu;
    }

    bool writeReg(uint8_t reg, uint8_t value) {
        return bus_.writeReg(reg, value);
    }

    bool readWordsToBurstBuffer(uint16_t count) {
        if (count == 0 || count > FIFO_BURST_WORD_CAPACITY) return false;

        const size_t len = static_cast<size_t>(count) * FIFO_WORD_BYTES;
        if (!bus_.read(REG_FIFO_DATA_OUT_TAG, fifoBurstBytes_, len)) return false;

        stats_.fifoWordsRead += count;
        stats_.fifoBurstReads++;
        stats_.fifoBurstReadWords += count;
        if (count > stats_.maxBurstWordsRead) {
            stats_.maxBurstWordsRead = count;
        }
        return true;
    }

    static void parseWordBytes(const uint8_t* b, FifoWord& w) {
        w.rawTag = b[0];
        w.tagSensor = static_cast<uint8_t>(w.rawTag >> 3);
        w.tagCounter = static_cast<uint8_t>((w.rawTag >> 1) & 0x03u);
        w.x = le16(&b[1]);
        w.y = le16(&b[3]);
        w.z = le16(&b[5]);
    }

    void processWord(const FifoWord& w, uint16_t statusFlags, uint64_t drainTimestampUs) {
        checkTagCounter(w);

        switch (w.tagSensor) {
            case TAG_GYRO_NC:
                stats_.gyroWords++;
                pendingGyro_ = w;
                pendingGyroValid_ = true;
                pendingFlags_ |= statusFlags;
                break;

            case TAG_ACCEL_NC:
                stats_.accelWords++;
                pendingAccel_ = w;
                pendingAccelValid_ = true;
                pendingFlags_ |= statusFlags;
                break;

            case TAG_TEMPERATURE:
                stats_.tempWords++;
                parseTemperatureWord(w);
                break;

            case TAG_TIMESTAMP:
                stats_.timestampWords++;
                parseTimestampWord(w);
                break;

            case TAG_CFG_CHANGE:
                stats_.cfgChangeWords++;
                break;

            case TAG_SENSORHUB_SLAVE0:
                if (cfg_.enableSensorHubSlave0) {
                    stats_.sensorHubSlave0Words++;
                    parseSensorHubSlave0Word(w, drainTimestampUs);
                } else {
                    stats_.unknownWords++;
                    pendingFlags_ |= FIFO_FLAG_UNKNOWN_TAG;
                }
                break;

            case TAG_SENSORHUB_NACK:
                stats_.sensorHubNackWords++;
                break;

            case TAG_EMPTY:
                stats_.emptyWords++;
                break;

            default:
                stats_.unknownWords++;
                pendingFlags_ |= FIFO_FLAG_UNKNOWN_TAG;
                break;
        }

        if (pendingGyroValid_ && pendingAccelValid_) {
            Lsm6dsv::RawSample s;
            buildRawSampleFromPending(s, pendingFlags_);
            enqueueSampleForTimestamp(s, drainTimestampUs);
            pendingGyroValid_ = false;
            pendingAccelValid_ = false;
            pendingFlags_ = 0;
        }
    }

    static int16_t le16(const uint8_t* p) {
        return static_cast<int16_t>(
            static_cast<uint16_t>(p[0]) |
            static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8)
        );
    }

    static bool isRawSaturated(int16_t v) {
        return v == INT16_MAX || v == INT16_MIN;
    }

    static float tempRawToC(int16_t raw) {
        return 25.0f + static_cast<float>(raw) / 256.0f;
    }

    void resetParserState() {
        pendingGyroValid_ = false;
        pendingAccelValid_ = false;
        pendingFlags_ = 0;
        for (uint8_t i = 0; i < 32; ++i) {
            lastTagCounter_[i] = 0xFF;
        }
        tsHead_ = tsTail_ = tsCount_ = 0;
        waitingHead_ = waitingTail_ = waitingCount_ = 0;
        completedHead_ = completedTail_ = completedCount_ = 0;
        magHead_ = magTail_ = magCount_ = 0;
    }

    void checkTagCounter(const FifoWord& w) {
    if (w.tagSensor >= 32 ||
        w.tagSensor == TAG_TEMPERATURE ||
        w.tagSensor == TAG_CFG_CHANGE ||
        w.tagSensor == TAG_EMPTY ||
        w.tagSensor == TAG_SENSORHUB_SLAVE0 ||
        w.tagSensor == TAG_SENSORHUB_NACK) {
        return;
    }

    uint8_t& last = lastTagCounter_[w.tagSensor];
    if (last != 0xFF) {
        const uint8_t expected = static_cast<uint8_t>((last + 1u) & 0x03u);
        if (w.tagCounter != expected) {
            stats_.tagCounterJumps++;
            if (w.tagSensor == TAG_GYRO_NC) stats_.gyroTagCounterJumps++;
            if (w.tagSensor == TAG_ACCEL_NC) stats_.accelTagCounterJumps++;
        }
    }
    last = w.tagCounter;
}

    void parseTemperatureWord(const FifoWord& w) {
        stats_.latestTempC = tempRawToC(w.x);
        stats_.latestTempValid = true;
    }

    void parseSensorHubSlave0Word(const FifoWord& w, uint64_t drainTimestampUs) {
        MagRawSample m;
        m.x = w.x;
        m.y = w.y;
        m.z = w.z;
        m.rawTag = w.rawTag;
        m.tagCounter = w.tagCounter;
        m.flags = MAG_FLAG_FROM_SENSORHUB_SLAVE0;
        if (isRawSaturated(m.x) || isRawSaturated(m.y) || isRawSaturated(m.z)) {
            m.flags |= MAG_FLAG_RAW_SATURATED;
            stats_.magRawSaturationCount++;
        }

        const uint64_t periodUs = cfg_.sensorHubSlave0PeriodUs > 0.0f
            ? static_cast<uint64_t>(cfg_.sensorHubSlave0PeriodUs + 0.5f)
            : 0;

        if (stats_.lastMagTimestampUs != 0 && periodUs > 0) {
            m.t_us = stats_.lastMagTimestampUs + periodUs;
        } else if (stats_.lastAssignedTimestampUs != 0) {
            m.t_us = stats_.lastAssignedTimestampUs;
            m.flags |= MAG_FLAG_TIMESTAMP_FALLBACK;
        } else if (stats_.lastHwTimestampUs != 0) {
            m.t_us = stats_.lastHwTimestampUs;
            m.flags |= MAG_FLAG_TIMESTAMP_FALLBACK;
        } else {
            m.t_us = drainTimestampUs;
            m.flags |= MAG_FLAG_TIMESTAMP_FALLBACK;
        }
        if (m.t_us == 0) m.t_us = 1;

        if (stats_.lastMagTimestampUs != 0 && m.t_us > stats_.lastMagTimestampUs) {
            const uint32_t dt = static_cast<uint32_t>(m.t_us - stats_.lastMagTimestampUs);
            stats_.lastMagDtUs = dt;
            if (stats_.magDtCount == 0 || dt < stats_.minMagDtUs) stats_.minMagDtUs = dt;
            if (dt > stats_.maxMagDtUs) stats_.maxMagDtUs = dt;
            stats_.sumMagDtUs += static_cast<double>(dt);
            stats_.magDtCount++;
        }

        stats_.lastMagTimestampUs = m.t_us;
        stats_.lastMagX = m.x;
        stats_.lastMagY = m.y;
        stats_.lastMagZ = m.z;
        stats_.lastMagRawNorm = std::sqrt(static_cast<float>(m.x) * static_cast<float>(m.x) +
                                          static_cast<float>(m.y) * static_cast<float>(m.y) +
                                          static_cast<float>(m.z) * static_cast<float>(m.z));
        m.seq = stats_.magSamplesProduced + 1;

        pushMagSample(m);
    }

    void parseTimestampWord(const FifoWord& w) {
        const uint32_t rawTicks = static_cast<uint32_t>(static_cast<uint16_t>(w.x)) |
                                  (static_cast<uint32_t>(static_cast<uint16_t>(w.y)) << 16);

        const uint8_t bdrXlMeta = static_cast<uint8_t>((static_cast<uint16_t>(w.z) >> 8) & 0x0Fu);
        const uint8_t bdrGyMeta = static_cast<uint8_t>((static_cast<uint16_t>(w.z) >> 12) & 0x0Fu);
        if (bdrXlMeta != odrToBdrNibble(cfg_.accelBdr)) stats_.timestampMetaBdrXlMismatch++;
        if (bdrGyMeta != odrToBdrNibble(cfg_.gyroBdr)) stats_.timestampMetaBdrGyMismatch++;

        if (stats_.lastRawTimestampTicks != 0) {
            if (rawTicks == stats_.lastRawTimestampTicks) {
                stats_.timestampDuplicate++;
            } else if (rawTicks < stats_.lastRawTimestampTicks) {
                const uint32_t backwards = stats_.lastRawTimestampTicks - rawTicks;
                if (backwards > 0x80000000UL) {
                    stats_.timestampWrapHigh += 0x100000000ULL;
                    stats_.timestampWraps++;
                } else {
                    stats_.timestampBackwards++;
                }
            }
        }

        const uint64_t extendedTicks = stats_.timestampWrapHigh + rawTicks;
        const uint64_t tsUs = static_cast<uint64_t>(extendedTicks * static_cast<double>(stats_.timestampTickUs) + 0.5);

        if (stats_.lastHwTimestampUs != 0 && tsUs > stats_.lastHwTimestampUs) {
            const uint64_t gapUs = tsUs - stats_.lastHwTimestampUs;
            if (stats_.samplePeriodUs > 0.0f && gapUs > static_cast<uint64_t>(stats_.samplePeriodUs * 4.0f)) {
                stats_.timestampLargeGap++;
            }
        }

        stats_.lastRawTimestampTicks = rawTicks;
        stats_.lastHwTimestampUs = tsUs;

        if (waitingCount_ > 0) {
            Lsm6dsv::RawSample s;
            popWaiting(s);
            assignHardwareTimestamp(s, tsUs);
            pushCompleted(s);
        } else {
            pushTimestamp(tsUs);
        }
    }

    void buildRawSampleFromPending(Lsm6dsv::RawSample& s, uint16_t flags) {
        s.t_us = 0;
        s.gx = pendingGyro_.x;
        s.gy = pendingGyro_.y;
        s.gz = pendingGyro_.z;
        s.ax = pendingAccel_.x;
        s.ay = pendingAccel_.y;
        s.az = pendingAccel_.z;
        s.temp = stats_.latestTempValid ? static_cast<int16_t>((stats_.latestTempC - 25.0f) * 256.0f) : 0;
        s.statusRaw = 0;
        s.flags = static_cast<uint16_t>(flags | Lsm6dsv::FLAG_READ_OUTPUT_OK);

        if (isRawSaturated(s.ax) || isRawSaturated(s.ay) || isRawSaturated(s.az)) {
            s.flags |= Lsm6dsv::FLAG_ACCEL_SATURATED;
            stats_.accelSaturationCount++;
        }
        if (isRawSaturated(s.gx) || isRawSaturated(s.gy) || isRawSaturated(s.gz)) {
            s.flags |= Lsm6dsv::FLAG_GYRO_SATURATED;
            stats_.gyroSaturationCount++;
        }
    }

    void enqueueSampleForTimestamp(Lsm6dsv::RawSample& s, uint64_t fallbackBaseUs) {
        if (cfg_.useHardwareTimestamps && tsCount_ > 0) {
            const uint64_t tsUs = popTimestamp();
            assignHardwareTimestamp(s, tsUs);
            pushCompleted(s);
            return;
        }

        if (cfg_.useHardwareTimestamps) {
            if (!pushWaiting(s)) {
                stats_.waitingSampleQueueOverflow++;
                assignFallbackTimestamp(s, fallbackBaseUs);
                pushCompleted(s);
            }
            return;
        }

        assignFallbackTimestamp(s, fallbackBaseUs);
        pushCompleted(s);
    }

    void assignHardwareTimestamp(Lsm6dsv::RawSample& s, uint64_t tsUs) {
        if (tsUs == 0) tsUs = 1;
        s.t_us = tsUs;
        s.flags = static_cast<uint16_t>((s.flags & ~FIFO_FLAG_TS_FALLBACK) | FIFO_FLAG_TS_HARDWARE);
        stats_.lastAssignedTimestampUs = tsUs;
        stats_.hwTimestampAssigned++;
    }

    void assignFallbackTimestamp(Lsm6dsv::RawSample& s, uint64_t drainTimestampUs) {
        uint64_t tsUs = 0;
        const uint64_t periodUs = static_cast<uint64_t>(stats_.samplePeriodUs + 0.5f);
        if (stats_.lastAssignedTimestampUs == 0) {
            tsUs = drainTimestampUs;
        } else {
            tsUs = stats_.lastAssignedTimestampUs + (periodUs > 0 ? periodUs : 1);
        }
        if (tsUs == 0) tsUs = 1;
        s.t_us = tsUs;
        s.flags = static_cast<uint16_t>((s.flags & ~FIFO_FLAG_TS_HARDWARE) | FIFO_FLAG_TS_FALLBACK);
        stats_.lastAssignedTimestampUs = tsUs;
        stats_.fallbackTimestampAssigned++;
    }

    void fallbackWaitingSamples(size_t howMany) {
        while (howMany > 0 && waitingCount_ > 0) {
            Lsm6dsv::RawSample s;
            popWaiting(s);
            assignFallbackTimestamp(s, stats_.lastAssignedTimestampUs);
            pushCompleted(s);
            howMany--;
        }
    }

    bool pushTimestamp(uint64_t tsUs) {
        if (tsCount_ >= TIMESTAMP_QUEUE_CAP) {
            stats_.timestampQueueOverflow++;
            return false;
        }
        timestampQueue_[tsTail_] = tsUs;
        tsTail_ = (tsTail_ + 1) % TIMESTAMP_QUEUE_CAP;
        tsCount_++;
        return true;
    }

    uint64_t popTimestamp() {
        if (tsCount_ == 0) return 0;
        const uint64_t v = timestampQueue_[tsHead_];
        tsHead_ = (tsHead_ + 1) % TIMESTAMP_QUEUE_CAP;
        tsCount_--;
        return v;
    }

    bool pushWaiting(const Lsm6dsv::RawSample& s) {
        if (waitingCount_ >= WAITING_SAMPLE_CAP) return false;
        waitingSamples_[waitingTail_] = s;
        waitingTail_ = (waitingTail_ + 1) % WAITING_SAMPLE_CAP;
        waitingCount_++;
        return true;
    }

    bool popWaiting(Lsm6dsv::RawSample& s) {
        if (waitingCount_ == 0) return false;
        s = waitingSamples_[waitingHead_];
        waitingHead_ = (waitingHead_ + 1) % WAITING_SAMPLE_CAP;
        waitingCount_--;
        return true;
    }

    bool pushMagSample(const MagRawSample& m) {
        MagRawSample sample = m;
        if (magCount_ >= MAG_SAMPLE_CAP) {
            magHead_ = (magHead_ + 1) % MAG_SAMPLE_CAP;
            magCount_--;
            stats_.magQueueOverflow++;
            sample.flags |= MAG_FLAG_QUEUE_OVERFLOW;
        }
        magSamples_[magTail_] = sample;
        magTail_ = (magTail_ + 1) % MAG_SAMPLE_CAP;
        magCount_++;
        stats_.magSamplesProduced++;
        return true;
    }

    bool pushCompleted(const Lsm6dsv::RawSample& s) {
        if (completedCount_ >= COMPLETED_SAMPLE_CAP) {
            // Drop oldest completed sample to preserve newest data and report via fallback/overflow.
            completedHead_ = (completedHead_ + 1) % COMPLETED_SAMPLE_CAP;
            completedCount_--;
            stats_.waitingSampleQueueOverflow++;
        }
        completedSamples_[completedTail_] = s;
        completedTail_ = (completedTail_ + 1) % COMPLETED_SAMPLE_CAP;
        completedCount_++;
        stats_.imuSamplesProduced++;
        return true;
    }

    bool popCompleted(Lsm6dsv::RawSample& s) {
        if (completedCount_ == 0) return false;
        s = completedSamples_[completedHead_];
        completedHead_ = (completedHead_ + 1) % COMPLETED_SAMPLE_CAP;
        completedCount_--;
        return true;
    }

    void popCompletedToOutput(Lsm6dsv::RawSample* out, size_t capacity, size_t& outCount) {
        while (outCount < capacity && completedCount_ > 0) {
            popCompleted(out[outCount++]);
        }
    }

    Lsm6dsvTransport& bus_;
    const Lsm6dsv& lsm_;
    Config cfg_;
    bool configured_ = false;

    FifoWord pendingGyro_;
    FifoWord pendingAccel_;
    bool pendingGyroValid_ = false;
    bool pendingAccelValid_ = false;
    uint16_t pendingFlags_ = 0;
    uint8_t lastTagCounter_[32];

    uint64_t timestampQueue_[TIMESTAMP_QUEUE_CAP];
    size_t tsHead_ = 0;
    size_t tsTail_ = 0;
    size_t tsCount_ = 0;

    Lsm6dsv::RawSample waitingSamples_[WAITING_SAMPLE_CAP];
    size_t waitingHead_ = 0;
    size_t waitingTail_ = 0;
    size_t waitingCount_ = 0;

    Lsm6dsv::RawSample completedSamples_[COMPLETED_SAMPLE_CAP];
    size_t completedHead_ = 0;
    size_t completedTail_ = 0;
    size_t completedCount_ = 0;

    MagRawSample magSamples_[MAG_SAMPLE_CAP];
    size_t magHead_ = 0;
    size_t magTail_ = 0;
    size_t magCount_ = 0;

    uint8_t fifoBurstBytes_[static_cast<size_t>(FIFO_BURST_WORD_CAPACITY) * FIFO_WORD_BYTES] = {};

    DrainStats stats_;
};

} // namespace tracker
