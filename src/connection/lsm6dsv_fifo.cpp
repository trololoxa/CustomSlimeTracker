#include "connection/lsm6dsv_fifo.hpp"

namespace tracker {

Lsm6dsvFifoReader::Lsm6dsvFifoReader(Lsm6dsvTransport& bus, const Lsm6dsv& lsm)
        : bus_(bus), lsm_(lsm) {}

bool Lsm6dsvFifoReader::configure(const Config& config) {
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

bool Lsm6dsvFifoReader::resetFifo() {
        if (!writeReg(REG_FIFO_CTRL4, 0x00)) return false;
        bus_.delayMs(2);

        resetParserState();
        stats_.lastAssignedTimestampUs = 0;
        stats_.lastHwTimestampUs = 0;
        stats_.lastRawTimestampTicks = 0;
        stats_.timestampWrapHigh = 0;
        resetMagTimestampBaseline();

        uint8_t fifoCtrl4 = 0;
        fifoCtrl4 |= (static_cast<uint8_t>(cfg_.timestampBatch) & 0x03u) << 6;
        fifoCtrl4 |= (static_cast<uint8_t>(cfg_.temperatureBatch) & 0x03u) << 4;
        fifoCtrl4 |= static_cast<uint8_t>(cfg_.mode) & 0x07u;
        return writeReg(REG_FIFO_CTRL4, fifoCtrl4);
    }

bool Lsm6dsvFifoReader::isConfigured() const { return configured_; }

float Lsm6dsvFifoReader::samplePeriodUs() const { return stats_.samplePeriodUs; }

float Lsm6dsvFifoReader::timestampTickUs() const { return stats_.timestampTickUs; }

const Lsm6dsvFifoReader::DrainStats& Lsm6dsvFifoReader::stats() const { return stats_; }

bool Lsm6dsvFifoReader::hasMagSamples() const { return magCount_ > 0; }

bool Lsm6dsvFifoReader::popMagSample(MagRawSample& out) {
        if (magCount_ == 0) return false;
        out = magSamples_[magHead_];
        magHead_ = (magHead_ + 1) % MAG_SAMPLE_CAP;
        magCount_--;
        return true;
    }

size_t Lsm6dsvFifoReader::popMagSamples(MagRawSample* out, size_t capacity) {
        if (out == nullptr || capacity == 0) return 0;
        size_t n = 0;
        while (n < capacity && popMagSample(out[n])) {
            n++;
        }
        return n;
    }

void Lsm6dsvFifoReader::resetTimestampReconstruction(uint64_t lastTimestampUs) {
        stats_.lastAssignedTimestampUs = lastTimestampUs;
        stats_.lastHwTimestampUs = 0;
        stats_.lastRawTimestampTicks = 0;
        stats_.timestampWrapHigh = 0;
        tsHead_ = tsTail_ = tsCount_ = 0;
        waitingHead_ = waitingTail_ = waitingCount_ = 0;
        completedHead_ = completedTail_ = completedCount_ = 0;
        resetMagTimestampBaseline();
    }

bool Lsm6dsvFifoReader::readStatus(Status& s) {
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

bool Lsm6dsvFifoReader::readWord(FifoWord& w) {
        uint8_t b[FIFO_WORD_BYTES] = {};
        if (!bus_.read(REG_FIFO_DATA_OUT_TAG, b, sizeof(b))) return false;
        parseWordBytes(b, w);
        stats_.fifoWordsRead++;
        return true;
    }

bool Lsm6dsvFifoReader::readWords(FifoWord* out, uint16_t count) {
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

bool Lsm6dsvFifoReader::drainRawSamples(Lsm6dsv::RawSample* out,
                         size_t capacity,
                         size_t& outCount,
                         uint64_t drainTimestampUs,
                         uint16_t maxWordsToRead) {
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


        if (cfg_.allowTimestampFallback && waitingCount_ > cfg_.maxWaitingSamplesBeforeFallback) {
            fallbackWaitingSamples(waitingCount_ - cfg_.maxWaitingSamplesBeforeFallback);
        }

        popCompletedToOutput(out, capacity, outCount);
        return true;
    }

bool Lsm6dsvFifoReader::configureFifoInterrupts() {
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

bool Lsm6dsvFifoReader::readInternalFreqFine() {
        uint8_t raw = 0;
        if (!bus_.read(REG_INTERNAL_FREQ_FINE, &raw, 1)) return false;
        stats_.internalFreqFine = static_cast<int8_t>(raw);
        return true;
    }

bool Lsm6dsvFifoReader::enableTimestampCounter() {
        uint8_t v = 0;
        if (!bus_.read(REG_FUNC_EN, &v, 1)) return false;
        v |= FUNC_EN_TIMESTAMP;
        return writeReg(REG_FUNC_EN, v);
    }

float Lsm6dsvFifoReader::configuredSamplePeriodUs() const {
        if (cfg_.samplePeriodUsOverride > 0.0f) return cfg_.samplePeriodUsOverride;

        const float gyroHz = actualOdrHz(cfg_.gyroBdr, stats_.internalFreqFine);
        const float accelHz = actualOdrHz(cfg_.accelBdr, stats_.internalFreqFine);
        const float hz = gyroHz > accelHz ? gyroHz : accelHz;
        if (hz <= 0.0f) return 0.0f;
        return 1000000.0f / hz;
    }

float Lsm6dsvFifoReader::timestampTickUsFromFine(int8_t fine) {
        const float scale = 1.0f + 0.0013f * static_cast<float>(fine);
        return 1000000.0f / (46080.0f * scale);
    }

float Lsm6dsvFifoReader::actualOdrHz(Lsm6dsv::Odr odr, int8_t fine) {
        const float scale = 1.0f + 0.0013f * static_cast<float>(fine);
        return Lsm6dsv::odrHz(odr) * scale;
    }

uint8_t Lsm6dsvFifoReader::odrToBdrNibble(Lsm6dsv::Odr odr) {
        return static_cast<uint8_t>(odr) & 0x0Fu;
    }

bool Lsm6dsvFifoReader::writeReg(uint8_t reg, uint8_t value) {
        return bus_.writeReg(reg, value);
    }

bool Lsm6dsvFifoReader::readWordsToBurstBuffer(uint16_t count) {
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

void Lsm6dsvFifoReader::parseWordBytes(const uint8_t* b, FifoWord& w) {
        w.rawTag = b[0];
        w.tagSensor = static_cast<uint8_t>(w.rawTag >> 3);
        w.tagCounter = static_cast<uint8_t>((w.rawTag >> 1) & 0x03u);
        w.x = le16(&b[1]);
        w.y = le16(&b[3]);
        w.z = le16(&b[5]);
    }

void Lsm6dsvFifoReader::processWord(const FifoWord& w, uint16_t statusFlags, uint64_t drainTimestampUs) {
        checkTagCounter(w);

        switch (w.tagSensor) {
            case TAG_GYRO_NC:
                stats_.gyroWords++;
                if (pendingGyroValid_) {
                    // A second gyro word means the previous cycle never got a
                    // matching accel word. Preserve heading continuity by
                    // publishing the older gyro as a gyro-only sample instead
                    // of silently overwriting it or pairing it with a later
                    // accelerometer observation.
                    Lsm6dsv::RawSample gyroOnly;
                    buildGyroOnlySampleFromPending(
                        gyroOnly,
                        pendingFlags_ | statusFlags | FIFO_FLAG_ORPHAN_WORDS
                    );
                    enqueueSampleForTimestamp(gyroOnly, drainTimestampUs);
                    stats_.gyroPendingReplaced++;
                    pendingFlags_ = 0;
                }
                pendingGyro_ = w;
                pendingGyroValid_ = true;
                pendingFlags_ |= statusFlags;
                break;

            case TAG_ACCEL_NC:
                stats_.accelWords++;
                if (pendingAccelValid_) {
                    // Accel-only observations cannot advance orientation. Keep
                    // the newest one for the next gyro and account for the
                    // discarded stale observation.
                    stats_.accelPendingReplaced++;
                    pendingFlags_ = 0;
                }
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
                // TAG 0x0E is a valid LSM6DSV sensor-hub FIFO tag. If the
                // firmware currently has the mag parser disabled, drop it as
                // an inactive external-sensor word instead of treating it as
                // FIFO corruption. This prevents first-run/empty-NVS boots or
                // abort paths from entering an endless FIFO recovery loop if
                // stale sensor-hub batching is still present in the FIFO.
                stats_.sensorHubSlave0Words++;
                if (cfg_.enableSensorHubSlave0) {
                    parseSensorHubSlave0Word(w, drainTimestampUs);
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
            const Lsm6dsv::SampleCoherency coherency = observePairCounterOffset();
            buildRawSampleFromPending(s, pendingFlags_, coherency);
            enqueueSampleForTimestamp(s, drainTimestampUs);
            stats_.completePairsProduced++;
            pendingGyroValid_ = false;
            pendingAccelValid_ = false;
            pendingFlags_ = 0;
        }
    }

int16_t Lsm6dsvFifoReader::le16(const uint8_t* p) {
        return static_cast<int16_t>(
            static_cast<uint16_t>(p[0]) |
            static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8)
        );
    }

bool Lsm6dsvFifoReader::isRawSaturated(int16_t v) {
        return v == INT16_MAX || v == INT16_MIN;
    }

float Lsm6dsvFifoReader::tempRawToC(int16_t raw) {
        return 25.0f + static_cast<float>(raw) / 256.0f;
    }

void Lsm6dsvFifoReader::resetParserState() {
        pendingGyroValid_ = false;
        pendingAccelValid_ = false;
        pendingFlags_ = 0;
        pairCounterOffsetValid_ = false;
        pairCounterOffset_ = 0;
        pairCounterCandidate_ = 0;
        pairCounterCandidateCount_ = 0;
        pairCounterMismatchCandidate_ = 0;
        pairCounterMismatchCount_ = 0;
        for (uint8_t i = 0; i < 32; ++i) {
            lastTagCounter_[i] = 0xFF;
        }
        tsHead_ = tsTail_ = tsCount_ = 0;
        waitingHead_ = waitingTail_ = waitingCount_ = 0;
        completedHead_ = completedTail_ = completedCount_ = 0;
        magHead_ = magTail_ = magCount_ = 0;
    }

void Lsm6dsvFifoReader::resetMagTimestampBaseline() {
        stats_.lastMagTimestampUs = 0;
        stats_.lastMagDtUs = 0;
        stats_.minMagDtUs = 0;
        stats_.maxMagDtUs = 0;
        stats_.sumMagDtUs = 0.0;
        stats_.magDtCount = 0;
    }

void Lsm6dsvFifoReader::checkTagCounter(const FifoWord& w) {
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

void Lsm6dsvFifoReader::parseTemperatureWord(const FifoWord& w) {
        stats_.latestTempC = tempRawToC(w.x);
        stats_.latestTempValid = true;
    }

void Lsm6dsvFifoReader::parseSensorHubSlave0Word(const FifoWord& w, uint64_t drainTimestampUs) {
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

void Lsm6dsvFifoReader::parseTimestampWord(const FifoWord& w) {
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

void Lsm6dsvFifoReader::buildRawSampleFromPending(Lsm6dsv::RawSample& s,
                                                    uint16_t flags,
                                                    Lsm6dsv::SampleCoherency coherency) {
        s.t_us = 0;
        s.gx = pendingGyro_.x;
        s.gy = pendingGyro_.y;
        s.gz = pendingGyro_.z;
        s.ax = pendingAccel_.x;
        s.ay = pendingAccel_.y;
        s.az = pendingAccel_.z;
        s.temp = stats_.latestTempValid ? static_cast<int16_t>((stats_.latestTempC - 25.0f) * 256.0f) : 0;
        s.statusRaw = 0;
        s.components = Lsm6dsv::SAMPLE_COMPONENT_COMPLETE;
        s.coherency = coherency;
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

void Lsm6dsvFifoReader::buildGyroOnlySampleFromPending(Lsm6dsv::RawSample& s, uint16_t flags) {
        s.t_us = 0;
        s.gx = pendingGyro_.x;
        s.gy = pendingGyro_.y;
        s.gz = pendingGyro_.z;
        s.ax = 0;
        s.ay = 0;
        s.az = 0;
        s.temp = stats_.latestTempValid ? static_cast<int16_t>((stats_.latestTempC - 25.0f) * 256.0f) : 0;
        s.statusRaw = 0;
        s.components = Lsm6dsv::SAMPLE_COMPONENT_GYRO;
        s.coherency = Lsm6dsv::SampleCoherency::GyroOnly;
        s.flags = static_cast<uint16_t>(flags | Lsm6dsv::FLAG_READ_OUTPUT_OK);

        if (isRawSaturated(s.gx) || isRawSaturated(s.gy) || isRawSaturated(s.gz)) {
            s.flags |= Lsm6dsv::FLAG_GYRO_SATURATED;
            stats_.gyroSaturationCount++;
        }
        stats_.gyroOnlySamplesProduced++;
    }

Lsm6dsv::SampleCoherency Lsm6dsvFifoReader::observePairCounterOffset() {
        const uint8_t observed = static_cast<uint8_t>((pendingGyro_.tagCounter - pendingAccel_.tagCounter) & 0x03u);
        constexpr uint8_t kInitialLockPairs = 4;
        constexpr uint8_t kRelockPairs = 8;

        if (!pairCounterOffsetValid_) {
            if (pairCounterCandidateCount_ == 0 || pairCounterCandidate_ != observed) {
                pairCounterCandidate_ = observed;
                pairCounterCandidateCount_ = 1;
            } else if (pairCounterCandidateCount_ < 0xffu) {
                ++pairCounterCandidateCount_;
            }
            if (pairCounterCandidateCount_ >= kInitialLockPairs) {
                pairCounterOffsetValid_ = true;
                pairCounterOffset_ = pairCounterCandidate_;
                pairCounterMismatchCount_ = 0;
                stats_.pairCounterOffsetLocks++;
            }
            return Lsm6dsv::SampleCoherency::Coherent;
        }

        if (observed == pairCounterOffset_) {
            pairCounterMismatchCount_ = 0;
            return Lsm6dsv::SampleCoherency::Coherent;
        }

        stats_.pairCounterMismatches++;
        if (pairCounterMismatchCount_ == 0 || pairCounterMismatchCandidate_ != observed) {
            pairCounterMismatchCandidate_ = observed;
            pairCounterMismatchCount_ = 1;
        } else if (pairCounterMismatchCount_ < 0xffu) {
            ++pairCounterMismatchCount_;
        }

        // A persistent new offset normally means a clean FIFO/configuration
        // epoch change. Re-lock only after several identical mismatches. Until
        // then gyro is integrated but accel correction is disabled.
        if (pairCounterMismatchCount_ >= kRelockPairs) {
            pairCounterOffset_ = pairCounterMismatchCandidate_;
            pairCounterMismatchCount_ = 0;
            stats_.pairCounterOffsetRelocks++;
        }
        return Lsm6dsv::SampleCoherency::PairCounterMismatch;
    }

void Lsm6dsvFifoReader::enqueueSampleForTimestamp(Lsm6dsv::RawSample& s, uint64_t fallbackBaseUs) {
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

void Lsm6dsvFifoReader::assignHardwareTimestamp(Lsm6dsv::RawSample& s, uint64_t tsUs) {
        if (tsUs == 0) tsUs = 1;
        s.t_us = tsUs;
        s.flags = static_cast<uint16_t>((s.flags & ~FIFO_FLAG_TS_FALLBACK) | FIFO_FLAG_TS_HARDWARE);
        stats_.lastAssignedTimestampUs = tsUs;
        stats_.hwTimestampAssigned++;
    }

void Lsm6dsvFifoReader::assignFallbackTimestamp(Lsm6dsv::RawSample& s, uint64_t drainTimestampUs) {
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

void Lsm6dsvFifoReader::fallbackWaitingSamples(size_t howMany) {
        while (howMany > 0 && waitingCount_ > 0) {
            Lsm6dsv::RawSample s;
            popWaiting(s);
            assignFallbackTimestamp(s, stats_.lastAssignedTimestampUs);
            pushCompleted(s);
            howMany--;
        }
    }

bool Lsm6dsvFifoReader::pushTimestamp(uint64_t tsUs) {
        if (tsCount_ >= TIMESTAMP_QUEUE_CAP) {
            stats_.timestampQueueOverflow++;
            return false;
        }
        timestampQueue_[tsTail_] = tsUs;
        tsTail_ = (tsTail_ + 1) % TIMESTAMP_QUEUE_CAP;
        tsCount_++;
        return true;
    }

uint64_t Lsm6dsvFifoReader::popTimestamp() {
        if (tsCount_ == 0) return 0;
        const uint64_t v = timestampQueue_[tsHead_];
        tsHead_ = (tsHead_ + 1) % TIMESTAMP_QUEUE_CAP;
        tsCount_--;
        return v;
    }

bool Lsm6dsvFifoReader::pushWaiting(const Lsm6dsv::RawSample& s) {
        if (waitingCount_ >= WAITING_SAMPLE_CAP) return false;
        waitingSamples_[waitingTail_] = s;
        waitingTail_ = (waitingTail_ + 1) % WAITING_SAMPLE_CAP;
        waitingCount_++;
        return true;
    }

bool Lsm6dsvFifoReader::popWaiting(Lsm6dsv::RawSample& s) {
        if (waitingCount_ == 0) return false;
        s = waitingSamples_[waitingHead_];
        waitingHead_ = (waitingHead_ + 1) % WAITING_SAMPLE_CAP;
        waitingCount_--;
        return true;
    }

bool Lsm6dsvFifoReader::pushMagSample(const MagRawSample& m) {
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

bool Lsm6dsvFifoReader::pushCompleted(const Lsm6dsv::RawSample& s) {
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

bool Lsm6dsvFifoReader::popCompleted(Lsm6dsv::RawSample& s) {
        if (completedCount_ == 0) return false;
        s = completedSamples_[completedHead_];
        completedHead_ = (completedHead_ + 1) % COMPLETED_SAMPLE_CAP;
        completedCount_--;
        return true;
    }

void Lsm6dsvFifoReader::popCompletedToOutput(Lsm6dsv::RawSample* out, size_t capacity, size_t& outCount) {
        while (outCount < capacity && completedCount_ > 0) {
            popCompleted(out[outCount++]);
        }
    }

} // namespace tracker
