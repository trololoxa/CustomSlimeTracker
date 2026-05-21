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
}

bool FifoRuntimeProcessor::process(uint16_t watermarkWords,
                                   uint16_t maxWordsPerDrain,
                                   uint8_t maxDrainRoundsPerEvent,
                                   Stream& out) {
    if (!ready()) return false;
    if (!eventSource_->consume(0, watermarkWords)) return false;

#if TRACKER_HAS_HOTPATH_PERF
    const uint32_t fifoProcessStartUs = micros();
#endif
    const uint8_t rounds = maxDrainRoundsPerEvent > 0u ? maxDrainRoundsPerEvent : 1u;
    const uint16_t maxWords = maxWordsPerDrain > 0u ? maxWordsPerDrain : 1u;

    for (uint8_t round = 0; round < rounds; ++round) {
        size_t count = 0;
        const uint64_t drainTimestampUs = micros();

        const bool ok = fifo_->drainRawSamples(
            rawBuffer_,
            rawBufferCapacity_,
            count,
            drainTimestampUs,
            maxWords
        );

        const size_t magCount = fifo_->popMagSamples(magBuffer_, magBufferCapacity_);
        for (size_t i = 0; i < magCount; ++i) {
            magCallback_(magBuffer_[i], callbackUser_);
        }

        if (!ok) {
            out.println("# ERR FIFO drain failed");
#if TRACKER_HAS_HOTPATH_PERF
            recordElapsed(fifoProcessStartUs);
#endif
            return true;
        }

        if (count == 0u && magCount == 0u) break;

        bool checkFifoStatsDelta = true;
        for (size_t i = 0; i < count; ++i) {
            if (sampleCallback_(rawBuffer_[i], checkFifoStatsDelta, callbackUser_) == FifoRuntimeSampleResult::FifoRecovered) {
                // The FIFO was reset while this local batch was being processed.
                // Remaining entries were captured before reset and may still carry
                // latched FIFO_FULL/OVR flags. Drop them instead of causing a
                // recovery storm from one hardware event.
#if TRACKER_HAS_HOTPATH_PERF
                recordElapsed(fifoProcessStartUs);
#endif
                return true;
            }
            checkFifoStatsDelta = false;
        }
    }

#if TRACKER_HAS_HOTPATH_PERF
    recordElapsed(fifoProcessStartUs);
#endif
    return true;
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
