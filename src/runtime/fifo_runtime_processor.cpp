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
    diagnosticsTimingEnabled_ = false;
    diagnosticsTimingSampled_ = false;
    diagnosticsTimingDecimator_ = 0u;
    resetWork();
}

void FifoRuntimeProcessor::setDiagnosticsTimingEnabled(bool enabled) {
    if (enabled == diagnosticsTimingEnabled_) return;
    diagnosticsTimingEnabled_ = enabled;
    diagnosticsTimingSampled_ = false;
    diagnosticsTimingDecimator_ = 0u;
}

void FifoRuntimeProcessor::resetWork() {
    clearQueues();
    drainActive_ = false;
    drainRoundsRemaining_ = 0;
    lastDispatchedRawTimestampUs_ = 0;
    lastDequeuedQueueAgeUs_ = 0;
#if TRACKER_HAS_RUNTIME_PROFILER
    for (uint32_t& queuedAtUs : rawQueueEnqueuedAtUs_) queuedAtUs = 0u;
#endif
}

bool FifoRuntimeProcessor::process(uint16_t watermarkWords,
                                   uint16_t maxWordsPerDrain,
                                   uint8_t maxDrainRoundsPerEvent,
                                   Stream& out,
                                   uint32_t sliceBudgetUs) {
    if (!ready()) return false;

    // This timestamp is part of scheduling correctness, not optional profiling:
    // hardware drain and callbacks share one absolute slice budget in every profile.
    const uint32_t fifoProcessStartUs = micros();
    diagnosticsTimingSampled_ = diagnosticsTimingEnabled_ &&
        (diagnosticsTimingDecimator_++ % TRACKER_DIAGNOSTIC_TIMING_SAMPLE_DIVISOR) == 0u;

    bool worked = false;
    if (!drainActive_) {
        worked = beginDrainEvent(watermarkWords, maxDrainRoundsPerEvent);
    }

    // Copy hardware FIFO data into the RAM queue before spending time on AHRS.
    // The hardware drain is part of the same absolute slice budget; the old
    // implementation started timing only after SPI and could overshoot a pose
    // deadline by an entire drain burst.
    if (drainActive_ && drainRoundsRemaining_ > 0u &&
        rawQueueCount_ < rawQueueCapacity_) {
        const uint32_t beforeDrainElapsedUs = micros() - fifoProcessStartUs;
        if (sliceBudgetUs != 0u && beforeDrainElapsedUs >= sliceBudgetUs) {
            ++queueStats_.drainBudgetDeferrals;
        } else {
            uint16_t boundedMaxWords = maxWordsPerDrain;
            if (sliceBudgetUs != 0u) {
                const uint32_t remainingUs = sliceBudgetUs - beforeDrainElapsedUs;
                if (remainingUs <= 1500u && boundedMaxWords > 16u) {
                    boundedMaxWords = 16u;
                    ++queueStats_.nearDeadlineReducedDrains;
                } else if (remainingUs <= 2500u && boundedMaxWords > 32u) {
                    boundedMaxWords = 32u;
                    ++queueStats_.nearDeadlineReducedDrains;
                }
            }
            worked = drainOneRound(boundedMaxWords, out) || worked;
        }
    }

#if TRACKER_HAS_RUNTIME_PROFILER
    const uint32_t callbackSliceStartUs = diagnosticsTimingSampled_ ? micros() : 0u;
#endif
    uint8_t rawCallbacks = 0;
    uint8_t magCallbacks = 0;

    // A previous pass may have stopped exactly when its bounded mag allowance
    // was exhausted. Drain only mag samples that are already due relative to
    // the last dispatched raw endpoint before advancing the raw timeline.
    bool chronologicalReady = true;
    if (lastDispatchedRawTimestampUs_ != 0u &&
        !dispatchDueMagCallbacks(lastDispatchedRawTimestampUs_, fifoProcessStartUs,
                                 sliceBudgetUs, magCallbacks, worked)) {
        queueStats_.magChronologicalDeferrals++;
        chronologicalReady = false;
    }

    Lsm6dsv::RawSample raw;
    bool checkStats = false;
    while (chronologicalReady) {
        if (rawCallbacks >= cfg::FIFO_RUNTIME_MIN_RAW_CALLBACKS_PER_SLICE &&
            sliceBudgetUs != 0u &&
            static_cast<uint32_t>(micros() - fifoProcessStartUs) >= sliceBudgetUs) {
            ++queueStats_.sliceBudgetStops;
            break;
        }
        if (!dequeueRaw(raw, checkStats)) break;
        worked = true;
#if TRACKER_HAS_RUNTIME_PROFILER
        const bool timeRawCallback = diagnosticsTimingSampled_ &&
            (queueStats_.rawProcessed & 31u) == 0u;
        const uint32_t rawCallbackStartUs = timeRawCallback ? micros() : 0u;
#endif
        const FifoRuntimeSampleResult sampleResult =
            sampleCallback_(raw, checkStats, callbackUser_);
#if TRACKER_HAS_RUNTIME_PROFILER
        if (timeRawCallback) {
            const uint32_t rawCallbackUs = micros() - rawCallbackStartUs;
            ++queueStats_.rawCallbackTimedSamples;
            queueStats_.rawCallbackTimeSumUs += rawCallbackUs;
            if (rawCallbackUs > queueStats_.rawCallbackTimeMaxUs) {
                queueStats_.rawCallbackTimeMaxUs = rawCallbackUs;
            }
        }
#endif
        if (sampleResult == FifoRuntimeSampleResult::FifoRecovered) {
            // Every queued sample predates the hardware reset.
            resetWork();
            recordElapsed(fifoProcessStartUs);
            return true;
        }
        const bool yieldRequested =
            sampleResult == FifoRuntimeSampleResult::YieldRequested;
        queueStats_.rawProcessed++;
        ++rawCallbacks;
        lastDispatchedRawTimestampUs_ = raw.t_us;

        // Sensor-hub samples share the FIFO time domain but live in a separate
        // queue. Dispatch each one as soon as raw processing reaches/passes its
        // timestamp. The mag callback therefore observes the nearest coherent
        // gyro endpoint instead of the end of a 64-sample IMU burst.
        if (!dispatchDueMagCallbacks(lastDispatchedRawTimestampUs_, fifoProcessStartUs,
                                     sliceBudgetUs, magCallbacks, worked)) {
            queueStats_.magChronologicalDeferrals++;
            break;
        }

        // A completed runtime-bias window must be finalized before the next
        // raw sample observes the updated trim. Yield only after all magnetic
        // samples due at this timestamp have preserved chronological order.
        if (yieldRequested) break;

        if (rawCallbacks >= cfg::FIFO_RUNTIME_MAX_RAW_CALLBACKS_PER_SLICE) break;
    }

#if TRACKER_HAS_RUNTIME_PROFILER
    if (diagnosticsTimingSampled_) {
        const uint32_t callbackElapsedUs = micros() - callbackSliceStartUs;
        ++queueStats_.callbackTimeCalls;
        queueStats_.callbackTimeSumUs += callbackElapsedUs;
        if (callbackElapsedUs > queueStats_.callbackTimeMaxUs) {
            queueStats_.callbackTimeMaxUs = callbackElapsedUs;
        }

        const uint32_t nowUs = micros();
        queueStats_.rawQueueOldestAgeLastUs = rawQueueOldestAgeUs(nowUs);
        if (queueStats_.rawQueueOldestAgeLastUs > queueStats_.rawQueueOldestAgeMaxUs) {
            queueStats_.rawQueueOldestAgeMaxUs = queueStats_.rawQueueOldestAgeLastUs;
        }
        queueStats_.rawQueueSpanLastUs = rawQueueSpanUs();
        if (queueStats_.rawQueueSpanLastUs > queueStats_.rawQueueSpanMaxUs) {
            queueStats_.rawQueueSpanMaxUs = queueStats_.rawQueueSpanLastUs;
        }
    }
#endif

    if (drainActive_ && drainRoundsRemaining_ == 0u) {
        drainActive_ = false;
    }

    if (sliceBudgetUs != 0u) {
        const uint32_t elapsedUs = micros() - fifoProcessStartUs;
        if (elapsedUs > sliceBudgetUs) {
            const uint32_t overshootUs = elapsedUs - sliceBudgetUs;
            ++queueStats_.sliceBudgetOvershootEvents;
            if (overshootUs > queueStats_.sliceBudgetOvershootMaxUs) {
                queueStats_.sliceBudgetOvershootMaxUs = overshootUs;
            }
        }
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
#if TRACKER_HAS_RUNTIME_PROFILER
    const uint32_t drainStartUs = diagnosticsTimingSampled_ ? micros() : 0u;
#endif
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
#if TRACKER_HAS_RUNTIME_PROFILER
    const uint32_t queuedAtUs = diagnosticsTimingSampled_ ? micros() : 0u;
#else
    const uint32_t queuedAtUs = 0u;
#endif

    for (size_t i = 0; i < rawCount; ++i) {
        if (!enqueueRaw(rawDrainBuffer_[i], i == 0u, queuedAtUs)) {
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
#if TRACKER_HAS_RUNTIME_PROFILER
    if (diagnosticsTimingSampled_) {
        const uint32_t drainElapsedUs = micros() - drainStartUs;
        ++queueStats_.hardwareDrainTimeCalls;
        queueStats_.hardwareDrainTimeSumUs += drainElapsedUs;
        if (drainElapsedUs > queueStats_.hardwareDrainTimeMaxUs) {
            queueStats_.hardwareDrainTimeMaxUs = drainElapsedUs;
        }
    }
#endif
    return true;
}

bool FifoRuntimeProcessor::enqueueRaw(const Lsm6dsv::RawSample& raw, bool checkStats, uint32_t queuedAtUs) {
    if (rawQueueCount_ >= rawQueueCapacity_) return false;
    rawQueueBuffer_[rawQueueTail_] = raw;
    rawQueueFlags_[rawQueueTail_] = checkStats ? 1u : 0u;
#if TRACKER_HAS_RUNTIME_PROFILER
    rawQueueEnqueuedAtUs_[rawQueueTail_] = queuedAtUs;
#else
    (void)queuedAtUs;
#endif
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
#if TRACKER_HAS_RUNTIME_PROFILER
    const uint32_t queuedAtUs = rawQueueEnqueuedAtUs_[rawQueueHead_];
    lastDequeuedQueueAgeUs_ = 0u;
    if (diagnosticsTimingSampled_ && queuedAtUs != 0u) {
        lastDequeuedQueueAgeUs_ = micros() - queuedAtUs;
        ++queueStats_.rawQueueWaitSamples;
        queueStats_.rawQueueWaitSumUs += lastDequeuedQueueAgeUs_;
        queueStats_.rawQueueWaitLastUs = lastDequeuedQueueAgeUs_;
        if (lastDequeuedQueueAgeUs_ > queueStats_.rawQueueWaitMaxUs) {
            queueStats_.rawQueueWaitMaxUs = lastDequeuedQueueAgeUs_;
        }
    }
    rawQueueEnqueuedAtUs_[rawQueueHead_] = 0u;
#else
    lastDequeuedQueueAgeUs_ = 0u;
#endif
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
                                                     uint32_t sliceStartUs,
                                                     uint32_t sliceBudgetUs,
                                                     uint8_t& magCallbacks,
                                                     bool& worked) {
    if (rawTimestampUs == 0u) return true;

    uint64_t magTimestampUs = 0u;
    while (peekMagTimestamp(magTimestampUs) && magTimestampUs <= rawTimestampUs) {
        if (magCallbacks >= cfg::FIFO_RUNTIME_MAX_MAG_CALLBACKS_PER_SLICE) {
            queueStats_.magCallbackCountDeferrals++;
            return false;
        }
        // Mag callbacks are cooperative app work and share the same absolute
        // budget as SPI drain and raw/AHRS callbacks.
        if (sliceBudgetUs != 0u &&
            static_cast<uint32_t>(micros() - sliceStartUs) >= sliceBudgetUs) {
            queueStats_.magCallbackBudgetDeferrals++;
            return false;
        }

        Lsm6dsvFifoReader::MagRawSample mag;
        (void)dequeueMag(mag);
#if TRACKER_HAS_RUNTIME_PROFILER
        const uint32_t magCallbackStartUs = diagnosticsTimingSampled_ ? micros() : 0u;
#endif
        magCallback_(mag, callbackUser_);
#if TRACKER_HAS_RUNTIME_PROFILER
        if (diagnosticsTimingSampled_) {
            const uint32_t magCallbackUs = micros() - magCallbackStartUs;
            ++queueStats_.magCallbackTimeCalls;
            queueStats_.magCallbackTimeSumUs += magCallbackUs;
            if (magCallbackUs > queueStats_.magCallbackTimeMaxUs) {
                queueStats_.magCallbackTimeMaxUs = magCallbackUs;
            }
        }
#endif
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
    lastDequeuedQueueAgeUs_ = 0u;
#if TRACKER_HAS_RUNTIME_PROFILER
    for (uint32_t& queuedAtUs : rawQueueEnqueuedAtUs_) queuedAtUs = 0u;
#endif
}

bool FifoRuntimeProcessor::hasPendingWork() const {
    return drainActive_ || rawQueueCount_ != 0u || magQueueCount_ != 0u;
}

bool FifoRuntimeProcessor::urgentByDepth() const {
    return rawQueueCount_ >= cfg::FIFO_RUNTIME_RAW_QUEUE_HIGH_WATER;
}

bool FifoRuntimeProcessor::urgentByAge() const {
    return rawQueueSpanUs() >= cfg::FIFO_RUNTIME_URGENT_SPAN_US;
}

bool FifoRuntimeProcessor::urgent() const {
    return urgentByDepth() || urgentByAge();
}

size_t FifoRuntimeProcessor::rawQueueDepth() const { return rawQueueCount_; }
size_t FifoRuntimeProcessor::magQueueDepth() const { return magQueueCount_; }

uint32_t FifoRuntimeProcessor::rawQueueOldestAgeUs(uint32_t nowUs) const {
#if TRACKER_HAS_RUNTIME_PROFILER
    if (rawQueueCount_ == 0u) return 0u;
    const uint32_t queuedAtUs = rawQueueEnqueuedAtUs_[rawQueueHead_];
    return queuedAtUs == 0u ? 0u : nowUs - queuedAtUs;
#else
    (void)nowUs;
    return 0u;
#endif
}

uint32_t FifoRuntimeProcessor::rawQueueSpanUs() const {
    if (rawQueueCount_ < 2u) return 0u;
    const size_t lastIndex = rawQueueTail_ == 0u ? rawQueueCapacity_ - 1u : rawQueueTail_ - 1u;
    const uint64_t oldest = rawQueueBuffer_[rawQueueHead_].t_us;
    const uint64_t newest = rawQueueBuffer_[lastIndex].t_us;
    if (newest <= oldest) return 0u;
    const uint64_t span = newest - oldest;
    return span > 0xffffffffULL ? 0xffffffffu : static_cast<uint32_t>(span);
}

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
    if (diagnosticsTimingSampled_ && recordTimeCallback_ != nullptr) {
        recordTimeCallback_(micros() - startUs, callbackUser_);
    }
#else
    (void)startUs;
#endif
}

} // namespace tracker
