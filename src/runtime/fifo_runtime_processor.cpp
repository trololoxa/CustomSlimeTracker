#include "runtime/fifo_runtime_processor.hpp"

#include "build_config/profile_contract.hpp"
#include "build_config/tracking_tuning.hpp"

namespace tracker {

void FifoInterruptEventSource::begin(volatile uint32_t* irqCount,
                                     Lsm6dsvFifoReader* fifo,
                                     TrackerPerfCounters* perf,
                                     uint32_t nonblockingStatusPollIntervalUs) {
    irqCount_ = irqCount;
    fifo_ = fifo;
    perf_ = perf;
    nonblockingStatusPollIntervalUs_ = nonblockingStatusPollIntervalUs;
    reset();
}

void FifoInterruptEventSource::reset() {
    lastHandledIrqCount_ = 0;
    missedIrqCount_ = 0;
    fallbackEvents_ = 0;
    waitTimeouts_ = 0;
    lastNonblockingStatusPollUs_ = 0;
}

bool FifoInterruptEventSource::consume(uint32_t timeoutMs, uint16_t watermarkWords) {
    if (irqCount_ == nullptr || fifo_ == nullptr) return false;

    const uint32_t startMs = millis();

    do {
        noInterrupts();
        const uint32_t current = *irqCount_;
        interrupts();

        if (current != lastHandledIrqCount_) {
            const uint32_t delta = current - lastHandledIrqCount_;
            if (delta > 1u) missedIrqCount_ += delta - 1u;
            lastHandledIrqCount_ = current;
#if TRACKER_HAS_HOTPATH_PERF
            if (perf_ != nullptr) perf_->fifoIrqEvents++;
#endif
            return true;
        }

        if (timeoutMs == 0u) {
            const uint32_t nowUs = micros();
            if (static_cast<uint32_t>(nowUs - lastNonblockingStatusPollUs_) < nonblockingStatusPollIntervalUs_) {
#if TRACKER_HAS_HOTPATH_PERF
                if (perf_ != nullptr) perf_->fifoEmptyPolls++;
#endif
                return false;
            }
            lastNonblockingStatusPollUs_ = nowUs;
            break;
        }

        yield();
    } while (millis() - startMs < timeoutMs);

    // Safety fallback: check FIFO_STATUS after a blocking wait timeout, or rarely
    // during non-blocking runtime polling. This avoids an SPI transaction on every
    // empty loop iteration while still recovering from a missed INT1 edge.
    Lsm6dsvFifoReader::Status st;
#if TRACKER_HAS_HOTPATH_PERF
    if (perf_ != nullptr) perf_->fifoFallbackStatusPolls++;
#endif
    if (fifo_->readStatus(st)) {
        if (st.unreadWords >= watermarkWords || st.overrun || st.full || st.overrunLatched) {
            fallbackEvents_++;
#if TRACKER_HAS_HOTPATH_PERF
            if (perf_ != nullptr) perf_->fifoFallbackEvents++;
#endif
            return true;
        }
    }

    if (timeoutMs > 0u) {
        waitTimeouts_++;
    } else {
#if TRACKER_HAS_HOTPATH_PERF
        if (perf_ != nullptr) perf_->fifoEmptyPolls++;
#endif
    }
    return false;
}

uint32_t FifoInterruptEventSource::missedIrqCount() const { return missedIrqCount_; }
uint32_t FifoInterruptEventSource::fallbackEvents() const { return fallbackEvents_; }
uint32_t FifoInterruptEventSource::waitTimeouts() const { return waitTimeouts_; }
uint32_t FifoInterruptEventSource::lastHandledIrqCount() const { return lastHandledIrqCount_; }

void FifoRuntimeProcessor::begin(FifoInterruptEventSource* eventSource,
                                 Lsm6dsvFifoReader* fifo,
                                 Lsm6dsv::RawSample* rawDrainBuffer,
                                 size_t rawDrainBufferCapacity,
                                 Lsm6dsvFifoReader::MagRawSample* magDrainBuffer,
                                 size_t magDrainBufferCapacity,
                                 Lsm6dsv::RawSample* rawQueueBuffer,
                                 uint8_t* rawQueueFlags,
                                 size_t rawQueueCapacity,
                                 Lsm6dsvFifoReader::MagRawSample* magQueueBuffer,
                                 size_t magQueueCapacity,
                                 FifoRuntimeSampleCallback sampleCallback,
                                 FifoRuntimeMagCallback magCallback,
                                 FifoRuntimeRecordTimeCallback recordTimeCallback,
                                 void* callbackUser) {
    eventSource_ = eventSource;
    fifo_ = fifo;
    rawDrainBuffer_ = rawDrainBuffer;
    rawDrainBufferCapacity_ = rawDrainBufferCapacity;
    magDrainBuffer_ = magDrainBuffer;
    magDrainBufferCapacity_ = magDrainBufferCapacity;
    rawQueueBuffer_ = rawQueueBuffer;
    rawQueueFlags_ = rawQueueFlags;
    rawQueueCapacity_ = rawQueueCapacity;
    magQueueBuffer_ = magQueueBuffer;
    magQueueCapacity_ = magQueueCapacity;
    sampleCallback_ = sampleCallback;
    magCallback_ = magCallback;
    recordTimeCallback_ = recordTimeCallback;
    callbackUser_ = callbackUser;
    queueStats_ = FifoRuntimeQueueStats{};
    resetWork();
}

void FifoRuntimeProcessor::resetWork() {
    clearQueues();
    drainActive_ = false;
    drainRoundsRemaining_ = 0;
    lastDispatchedRawTimestampUs_ = 0;
}

bool FifoRuntimeProcessor::process(uint16_t watermarkWords,
                                   uint16_t maxWordsPerDrain,
                                   uint8_t maxDrainRoundsPerEvent,
                                   Stream& out) {
    if (!ready()) return false;

#if TRACKER_HAS_HOTPATH_PERF
    const uint32_t fifoProcessStartUs = micros();
#else
    const uint32_t fifoProcessStartUs = 0;
#endif

    bool worked = false;
    if (!drainActive_) {
        worked = beginDrainEvent(watermarkWords, maxDrainRoundsPerEvent);
    }

    // Copy hardware FIFO data into the RAM queue before spending time on AHRS.
    // This gives short Wi-Fi/CLI stalls hundreds of milliseconds of headroom.
    if (drainActive_ && drainRoundsRemaining_ > 0u &&
        rawQueueCount_ < rawQueueCapacity_) {
        worked = drainOneRound(maxWordsPerDrain, out) || worked;
    }

    const uint32_t callbackSliceStartUs = micros();
    uint8_t rawCallbacks = 0;
    uint8_t magCallbacks = 0;

    // A previous pass may have stopped exactly when its bounded mag allowance
    // was exhausted. Drain only mag samples that are already due relative to
    // the last dispatched raw endpoint before advancing the raw timeline.
    bool chronologicalReady = true;
    if (lastDispatchedRawTimestampUs_ != 0u &&
        !dispatchDueMagCallbacks(lastDispatchedRawTimestampUs_, callbackSliceStartUs,
                                 magCallbacks, worked)) {
        queueStats_.magChronologicalDeferrals++;
        chronologicalReady = false;
    }

    Lsm6dsv::RawSample raw;
    bool checkStats = false;
    while (chronologicalReady && dequeueRaw(raw, checkStats)) {
        worked = true;
        if (sampleCallback_(raw, checkStats, callbackUser_) ==
            FifoRuntimeSampleResult::FifoRecovered) {
            // Every queued sample predates the hardware reset.
            resetWork();
            recordElapsed(fifoProcessStartUs);
            return true;
        }
        queueStats_.rawProcessed++;
        ++rawCallbacks;
        lastDispatchedRawTimestampUs_ = raw.t_us;

        // Sensor-hub samples share the FIFO time domain but live in a separate
        // queue. Dispatch each one as soon as raw processing reaches/passes its
        // timestamp. The mag callback therefore observes the nearest coherent
        // gyro endpoint instead of the end of a 64-sample IMU burst.
        if (!dispatchDueMagCallbacks(lastDispatchedRawTimestampUs_, callbackSliceStartUs,
                                     magCallbacks, worked)) {
            queueStats_.magChronologicalDeferrals++;
            break;
        }

        if (rawCallbacks >= cfg::FIFO_RUNTIME_MAX_RAW_CALLBACKS_PER_SLICE) break;
        if (rawCallbacks >= cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE &&
            cfg::FIFO_RUNTIME_SLICE_BUDGET_US != 0u &&
            static_cast<uint32_t>(micros() - callbackSliceStartUs) >=
                cfg::FIFO_RUNTIME_SLICE_BUDGET_US) {
            break;
        }
    }

    if (drainActive_ && drainRoundsRemaining_ == 0u) {
        drainActive_ = false;
    }

    recordElapsed(fifoProcessStartUs);
    return worked;
}

bool FifoRuntimeProcessor::beginDrainEvent(uint16_t watermarkWords,
                                           uint8_t maxDrainRoundsPerEvent) {
    if (!eventSource_->consume(0, watermarkWords)) return false;
    drainActive_ = true;
    drainRoundsRemaining_ = maxDrainRoundsPerEvent > 0u ? maxDrainRoundsPerEvent : 1u;
    return true;
}

bool FifoRuntimeProcessor::drainOneRound(uint16_t maxWordsPerDrain, Stream& out) {
    const size_t rawFree = rawQueueCapacity_ - rawQueueCount_;
    const size_t magFree = magQueueCapacity_ - magQueueCount_;
    if (rawFree == 0u || drainRoundsRemaining_ == 0u) return false;

    --drainRoundsRemaining_;
    const size_t rawCapacity = rawFree < rawDrainBufferCapacity_ ? rawFree : rawDrainBufferCapacity_;
    const size_t magCapacity = magFree < magDrainBufferCapacity_ ? magFree : magDrainBufferCapacity_;
    const uint16_t configuredMaxWords = maxWordsPerDrain > 0u ? maxWordsPerDrain : 1u;

    size_t rawCount = 0;
    const bool ok = fifo_->drainRawSamples(rawDrainBuffer_,
                                           rawCapacity,
                                           rawCount,
                                           micros(),
                                           configuredMaxWords);
    if (!ok) {
        out.println("# ERR FIFO drain failed");
        resetWork();
        return true;
    }

    const size_t magCount = fifo_->popMagSamples(magDrainBuffer_, magCapacity);
    queueStats_.hardwareDrains++;

    for (size_t i = 0; i < rawCount; ++i) {
        if (!enqueueRaw(rawDrainBuffer_[i], i == 0u)) {
            queueStats_.rawQueueOverflow++;
            break;
        }
    }
    for (size_t i = 0; i < magCount; ++i) {
        if (!enqueueMag(magDrainBuffer_[i])) {
            queueStats_.magQueueOverflow++;
            break;
        }
    }

    if (rawCount == 0u && magCount == 0u) {
        drainActive_ = false;
        drainRoundsRemaining_ = 0;
    }
    return true;
}

bool FifoRuntimeProcessor::enqueueRaw(const Lsm6dsv::RawSample& raw, bool checkStats) {
    if (rawQueueCount_ >= rawQueueCapacity_) return false;
    rawQueueBuffer_[rawQueueTail_] = raw;
    rawQueueFlags_[rawQueueTail_] = checkStats ? 1u : 0u;
    rawQueueTail_ = (rawQueueTail_ + 1u) % rawQueueCapacity_;
    ++rawQueueCount_;
    ++queueStats_.rawQueued;
    if (rawQueueCount_ > queueStats_.rawQueueHighWater) {
        queueStats_.rawQueueHighWater = rawQueueCount_;
    }
    return true;
}

bool FifoRuntimeProcessor::enqueueMag(const Lsm6dsvFifoReader::MagRawSample& mag) {
    if (magQueueCount_ >= magQueueCapacity_) return false;
    magQueueBuffer_[magQueueTail_] = mag;
    magQueueTail_ = (magQueueTail_ + 1u) % magQueueCapacity_;
    ++magQueueCount_;
    ++queueStats_.magQueued;
    if (magQueueCount_ > queueStats_.magQueueHighWater) {
        queueStats_.magQueueHighWater = magQueueCount_;
    }
    return true;
}

bool FifoRuntimeProcessor::dequeueRaw(Lsm6dsv::RawSample& raw, bool& checkStats) {
    if (rawQueueCount_ == 0u) return false;
    raw = rawQueueBuffer_[rawQueueHead_];
    checkStats = rawQueueFlags_[rawQueueHead_] != 0u;
    rawQueueHead_ = (rawQueueHead_ + 1u) % rawQueueCapacity_;
    --rawQueueCount_;
    return true;
}

bool FifoRuntimeProcessor::dequeueMag(Lsm6dsvFifoReader::MagRawSample& mag) {
    if (magQueueCount_ == 0u) return false;
    mag = magQueueBuffer_[magQueueHead_];
    magQueueHead_ = (magQueueHead_ + 1u) % magQueueCapacity_;
    --magQueueCount_;
    return true;
}

bool FifoRuntimeProcessor::peekMagTimestamp(uint64_t& timestampUs) const {
    if (magQueueCount_ == 0u) return false;
    timestampUs = magQueueBuffer_[magQueueHead_].t_us;
    return true;
}

bool FifoRuntimeProcessor::dispatchDueMagCallbacks(uint64_t rawTimestampUs,
                                                     uint32_t callbackSliceStartUs,
                                                     uint8_t& magCallbacks,
                                                     bool& worked) {
    if (rawTimestampUs == 0u) return true;

    uint64_t magTimestampUs = 0u;
    while (peekMagTimestamp(magTimestampUs) && magTimestampUs <= rawTimestampUs) {
        if (magCallbacks >= cfg::FIFO_RUNTIME_MAX_MAG_CALLBACKS_PER_SLICE) {
            queueStats_.magCallbackCountDeferrals++;
            return false;
        }
        // Hardware drain time is deliberately excluded because
        // callbackSliceStartUs is captured after the drain. Mag callbacks,
        // however, are cooperative app work just like raw/AHRS callbacks and
        // must not bypass the same output-latency budget.
        if (cfg::FIFO_RUNTIME_SLICE_BUDGET_US != 0u &&
            static_cast<uint32_t>(micros() - callbackSliceStartUs) >=
                cfg::FIFO_RUNTIME_SLICE_BUDGET_US) {
            queueStats_.magCallbackBudgetDeferrals++;
            return false;
        }

        Lsm6dsvFifoReader::MagRawSample mag;
        (void)dequeueMag(mag);
        magCallback_(mag, callbackUser_);
        queueStats_.magProcessed++;
        ++magCallbacks;
        worked = true;
    }
    return true;
}

void FifoRuntimeProcessor::clearQueues() {
    rawQueueHead_ = 0;
    rawQueueTail_ = 0;
    rawQueueCount_ = 0;
    magQueueHead_ = 0;
    magQueueTail_ = 0;
    magQueueCount_ = 0;
}

bool FifoRuntimeProcessor::hasPendingWork() const {
    return drainActive_ || rawQueueCount_ != 0u || magQueueCount_ != 0u;
}

bool FifoRuntimeProcessor::urgent() const {
    return rawQueueCount_ >= cfg::FIFO_RUNTIME_RAW_QUEUE_HIGH_WATER;
}

size_t FifoRuntimeProcessor::rawQueueDepth() const { return rawQueueCount_; }
size_t FifoRuntimeProcessor::magQueueDepth() const { return magQueueCount_; }
const FifoRuntimeQueueStats& FifoRuntimeProcessor::queueStats() const { return queueStats_; }

bool FifoRuntimeProcessor::ready() const {
    return eventSource_ != nullptr && fifo_ != nullptr &&
           rawDrainBuffer_ != nullptr && rawDrainBufferCapacity_ > 0u &&
           magDrainBuffer_ != nullptr && magDrainBufferCapacity_ > 0u &&
           rawQueueBuffer_ != nullptr && rawQueueFlags_ != nullptr && rawQueueCapacity_ > 0u &&
           magQueueBuffer_ != nullptr && magQueueCapacity_ > 0u &&
           sampleCallback_ != nullptr && magCallback_ != nullptr &&
           recordTimeCallback_ != nullptr;
}

void FifoRuntimeProcessor::recordElapsed(uint32_t startUs) {
#if TRACKER_HAS_HOTPATH_PERF
    if (recordTimeCallback_ != nullptr) {
        recordTimeCallback_(micros() - startUs, callbackUser_);
    }
#else
    (void)startUs;
#endif
}

} // namespace tracker
