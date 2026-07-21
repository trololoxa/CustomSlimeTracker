#include "runtime/fifo_runtime_processor.hpp"

#include "build_config/profile_contract.hpp"

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
                                 Lsm6dsv::RawSample* rawBuffer,
                                 size_t rawBufferCapacity,
                                 Lsm6dsvFifoReader::MagRawSample* magBuffer,
                                 size_t magBufferCapacity,
                                 FifoRuntimeSampleCallback sampleCallback,
                                 FifoRuntimeMagCallback magCallback,
                                 FifoRuntimeRecordTimeCallback recordTimeCallback,
                                 void* callbackUser) {
    eventSource_ = eventSource;
    fifo_ = fifo;
    rawBuffer_ = rawBuffer;
    rawBufferCapacity_ = rawBufferCapacity;
    magBuffer_ = magBuffer;
    magBufferCapacity_ = magBufferCapacity;
    sampleCallback_ = sampleCallback;
    magCallback_ = magCallback;
    recordTimeCallback_ = recordTimeCallback;
    callbackUser_ = callbackUser;
    resetWork();
}

void FifoRuntimeProcessor::resetWork() {
    clearPendingBatch();
    drainActive_ = false;
    drainRoundsRemaining_ = 0;
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

    if (!hasPendingCallbacks() && !drainActive_) {
        if (!eventSource_->consume(0, watermarkWords)) return false;
        drainActive_ = true;
        drainRoundsRemaining_ = maxDrainRoundsPerEvent > 0u ? maxDrainRoundsPerEvent : 1u;
    }

    if (!hasPendingCallbacks()) {
        if (!drainActive_ || drainRoundsRemaining_ == 0u) {
            drainActive_ = false;
            recordElapsed(fifoProcessStartUs);
            return true;
        }

        --drainRoundsRemaining_;
        const uint16_t configuredMaxWords = maxWordsPerDrain > 0u ? maxWordsPerDrain : 1u;
        const uint64_t drainTimestampUs = micros();

        size_t rawCount = 0;
        const bool ok = fifo_->drainRawSamples(
            rawBuffer_,
            rawBufferCapacity_,
            rawCount,
            drainTimestampUs,
            configuredMaxWords
        );
        pendingRawCount_ = rawCount;
        pendingRawIndex_ = 0;
        pendingMagCount_ = fifo_->popMagSamples(magBuffer_, magBufferCapacity_);
        pendingMagIndex_ = 0;
        pendingCheckFifoStatsDelta_ = pendingRawCount_ != 0u;

        if (!ok) {
            out.println("# ERR FIFO drain failed");
            clearPendingBatch();
            drainActive_ = false;
            drainRoundsRemaining_ = 0;
            recordElapsed(fifoProcessStartUs);
            return true;
        }

        if (!hasPendingCallbacks()) {
            // A status read with no produced samples means this event is fully
            // drained.  Do not burn the remaining configured rounds in the
            // same app pass; a new IRQ/fallback poll will restart work.
            drainActive_ = false;
            drainRoundsRemaining_ = 0;
            recordElapsed(fifoProcessStartUs);
            return true;
        }
    }

    // The SPI drain above can legitimately take several milliseconds for a
    // large batch. Do not charge that time to the callback/AHRS slice: doing so
    // can reduce progress to one sample per app pass and let the hardware FIFO
    // overflow while already-drained samples wait in RAM.
    const uint32_t callbackSliceStartUs = micros();
    uint8_t callbacks = 0;
    while (hasPendingCallbacks()) {
        if (pendingMagIndex_ < pendingMagCount_) {
            magCallback_(magBuffer_[pendingMagIndex_++], callbackUser_);
        } else {
            const bool checkStats = pendingCheckFifoStatsDelta_;
            pendingCheckFifoStatsDelta_ = false;
            if (sampleCallback_(rawBuffer_[pendingRawIndex_++], checkStats, callbackUser_) ==
                FifoRuntimeSampleResult::FifoRecovered) {
                // The local batch predates the reset.  Discard every remaining
                // callback and require a fresh interrupt/status observation.
                clearPendingBatch();
                drainActive_ = false;
                drainRoundsRemaining_ = 0;
                recordElapsed(fifoProcessStartUs);
                return true;
            }
        }

        ++callbacks;
        if (callbacks >= cfg::FIFO_RUNTIME_MAX_CALLBACKS_PER_SLICE) break;
        if (callbacks >= cfg::FIFO_RUNTIME_MIN_CALLBACKS_PER_SLICE &&
            cfg::FIFO_RUNTIME_SLICE_BUDGET_US != 0u &&
            static_cast<uint32_t>(micros() - callbackSliceStartUs) >=
                cfg::FIFO_RUNTIME_SLICE_BUDGET_US) {
            break;
        }
    }

    if (!hasPendingCallbacks()) {
        clearPendingBatch();
        if (drainRoundsRemaining_ == 0u) {
            drainActive_ = false;
        }
    }

    recordElapsed(fifoProcessStartUs);
    return true;
}

bool FifoRuntimeProcessor::hasPendingCallbacks() const {
    return pendingMagIndex_ < pendingMagCount_ || pendingRawIndex_ < pendingRawCount_;
}

void FifoRuntimeProcessor::clearPendingBatch() {
    pendingRawCount_ = 0;
    pendingRawIndex_ = 0;
    pendingMagCount_ = 0;
    pendingMagIndex_ = 0;
    pendingCheckFifoStatsDelta_ = false;
}

bool FifoRuntimeProcessor::ready() const {
    return eventSource_ != nullptr &&
           fifo_ != nullptr &&
           rawBuffer_ != nullptr &&
           rawBufferCapacity_ > 0u &&
           magBuffer_ != nullptr &&
           magBufferCapacity_ > 0u &&
           sampleCallback_ != nullptr &&
           magCallback_ != nullptr &&
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
