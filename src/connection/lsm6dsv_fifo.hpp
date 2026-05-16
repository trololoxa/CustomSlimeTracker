#pragma once

#include <cstddef>
#include <cstdint>
#include <cmath>

#include "defines.h"
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
        uint8_t watermarkWords = cfg::FIFO_WATERMARK_WORDS;

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
    };    Lsm6dsvFifoReader(Lsm6dsvTransport& bus, const Lsm6dsv& lsm);

    bool configure(const Config& config);

    bool resetFifo();

    bool isConfigured() const;

    float samplePeriodUs() const;

    float timestampTickUs() const;

    const DrainStats& stats() const;

    bool hasMagSamples() const;

    bool popMagSample(MagRawSample& out);

    size_t popMagSamples(MagRawSample* out, size_t capacity);

    void resetTimestampReconstruction(uint64_t lastTimestampUs = 0);

    bool readStatus(Status& s);

    bool readWord(FifoWord& w);

    bool readWords(FifoWord* out, uint16_t count);

    bool drainRawSamples(Lsm6dsv::RawSample* out,
                         size_t capacity,
                         size_t& outCount,
                         uint64_t drainTimestampUs,
                         uint16_t maxWordsToRead = 256);

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

    bool configureFifoInterrupts();

    bool readInternalFreqFine();

    bool enableTimestampCounter();

    float configuredSamplePeriodUs() const;

    static float timestampTickUsFromFine(int8_t fine);

    static float actualOdrHz(Lsm6dsv::Odr odr, int8_t fine);

    static uint8_t odrToBdrNibble(Lsm6dsv::Odr odr);

    bool writeReg(uint8_t reg, uint8_t value);

    bool readWordsToBurstBuffer(uint16_t count);

    static void parseWordBytes(const uint8_t* b, FifoWord& w);

    void processWord(const FifoWord& w, uint16_t statusFlags, uint64_t drainTimestampUs);

    static int16_t le16(const uint8_t* p);

    static bool isRawSaturated(int16_t v);

    static float tempRawToC(int16_t raw);

    void resetParserState();

    void checkTagCounter(const FifoWord& w);

    void parseTemperatureWord(const FifoWord& w);

    void parseSensorHubSlave0Word(const FifoWord& w, uint64_t drainTimestampUs);

    void parseTimestampWord(const FifoWord& w);

    void buildRawSampleFromPending(Lsm6dsv::RawSample& s, uint16_t flags);

    void enqueueSampleForTimestamp(Lsm6dsv::RawSample& s, uint64_t fallbackBaseUs);

    void assignHardwareTimestamp(Lsm6dsv::RawSample& s, uint64_t tsUs);

    void assignFallbackTimestamp(Lsm6dsv::RawSample& s, uint64_t drainTimestampUs);

    void fallbackWaitingSamples(size_t howMany);

    bool pushTimestamp(uint64_t tsUs);

    uint64_t popTimestamp();

    bool pushWaiting(const Lsm6dsv::RawSample& s);

    bool popWaiting(Lsm6dsv::RawSample& s);

    bool pushMagSample(const MagRawSample& m);

    bool pushCompleted(const Lsm6dsv::RawSample& s);

    bool popCompleted(Lsm6dsv::RawSample& s);

    void popCompletedToOutput(Lsm6dsv::RawSample* out, size_t capacity, size_t& outCount);

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
