#pragma once

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "build_config/profile_contract.hpp"
#include "build_config/tracking_tuning.hpp"
#include "runtime/tracker_runtime_types.hpp"

namespace tracker {

class FifoInterruptEventSource {
public:
    void begin(volatile uint32_t* irqCount,
               Lsm6dsvFifoReader* fifo,
               TrackerPerfCounters* perf,
               uint32_t nonblockingStatusPollIntervalUs);

    void reset();
    bool consume(uint32_t timeoutMs, uint16_t watermarkWords);

    uint32_t missedIrqCount() const;
    uint32_t fallbackEvents() const;
    uint32_t waitTimeouts() const;
    uint32_t lastHandledIrqCount() const;

private:
    volatile uint32_t* irqCount_ = nullptr;
    Lsm6dsvFifoReader* fifo_ = nullptr;
    TrackerPerfCounters* perf_ = nullptr;
    uint32_t nonblockingStatusPollIntervalUs_ = 0;
    uint32_t lastHandledIrqCount_ = 0;
    uint32_t missedIrqCount_ = 0;
    uint32_t fallbackEvents_ = 0;
    uint32_t waitTimeouts_ = 0;
    uint32_t lastNonblockingStatusPollUs_ = 0;
};

enum class FifoRuntimeSampleResult : uint8_t {
    Continue,
    FifoRecovered,
    YieldRequested,
};

using FifoRuntimeSampleCallback = FifoRuntimeSampleResult (*)(const Lsm6dsv::RawSample& raw,
                                                              bool checkFifoStatsDelta,
                                                              void* user);
using FifoRuntimeMagCallback = void (*)(const Lsm6dsvFifoReader::MagRawSample& mag,
                                        void* user);
using FifoRuntimeRecordTimeCallback = void (*)(uint32_t processUs,
                                               void* user);

struct FifoRuntimeQueueStats {
    uint32_t hardwareDrains = 0;
    uint32_t rawQueued = 0;
    uint32_t rawProcessed = 0;
    uint32_t magQueued = 0;
    uint32_t magProcessed = 0;
    uint32_t rawQueueOverflow = 0;
    uint32_t magQueueOverflow = 0;
    // A due mag callback was intentionally deferred because the bounded mag
    // callback allowance was exhausted. Raw processing stops at the same
    // timestamp so the next app pass preserves endpoint coherence.
    uint32_t magChronologicalDeferrals = 0;
    // Split the reason so perf diagnostics can distinguish normal count
    // slicing from a callback that consumed the cooperative time budget.
    uint32_t magCallbackCountDeferrals = 0;
    uint32_t magCallbackBudgetDeferrals = 0;
    size_t rawQueueHighWater = 0;
    size_t magQueueHighWater = 0;

    // Exact MCU-clock wait inside the software raw queue. Hardware-FIFO
    // residence is reported separately by FIFO counters; this metric proves
    // whether the application itself is serving historical samples.
    uint32_t rawQueueWaitSamples = 0;
    uint64_t rawQueueWaitSumUs = 0;
    uint32_t rawQueueWaitLastUs = 0;
    uint32_t rawQueueWaitMaxUs = 0;
    uint32_t rawQueueOldestAgeLastUs = 0;
    uint32_t rawQueueOldestAgeMaxUs = 0;
    uint32_t rawQueueSpanLastUs = 0;
    uint32_t rawQueueSpanMaxUs = 0;

    uint32_t hardwareDrainTimeCalls = 0;
    uint64_t hardwareDrainTimeSumUs = 0;
    uint32_t hardwareDrainTimeMaxUs = 0;
    uint32_t callbackTimeCalls = 0;
    uint64_t callbackTimeSumUs = 0;
    uint32_t callbackTimeMaxUs = 0;
    uint32_t rawCallbackTimedSamples = 0;
    uint64_t rawCallbackTimeSumUs = 0;
    uint32_t rawCallbackTimeMaxUs = 0;
    uint32_t magCallbackTimeCalls = 0;
    uint64_t magCallbackTimeSumUs = 0;
    uint32_t magCallbackTimeMaxUs = 0;
    uint32_t sliceBudgetStops = 0;
    uint32_t drainBudgetDeferrals = 0;
    uint32_t nearDeadlineReducedDrains = 0;
    uint32_t sliceBudgetOvershootEvents = 0;
    uint32_t sliceBudgetOvershootMaxUs = 0;
};

class FifoRuntimeProcessor {
public:
    void begin(FifoInterruptEventSource* eventSource,
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
               void* callbackUser);

    bool process(uint16_t watermarkWords,
                 uint16_t maxWordsPerDrain,
                 uint8_t maxDrainRoundsPerEvent,
                 Stream& out,
                 uint32_t sliceBudgetUs = cfg::FIFO_RUNTIME_SLICE_BUDGET_US);

    // Discard all samples captured before an external/manual FIFO reset.
    void resetWork();

    bool hasPendingWork() const;
    bool urgent() const;
    bool urgentByDepth() const;
    bool urgentByAge() const;
    size_t rawQueueDepth() const;
    size_t magQueueDepth() const;
    uint32_t rawQueueOldestAgeUs(uint32_t nowUs) const;
    uint32_t rawQueueSpanUs() const;
    uint32_t lastDequeuedQueueAgeUs() const { return lastDequeuedQueueAgeUs_; }
    const FifoRuntimeQueueStats& queueStats() const;

private:
    bool ready() const;
    bool beginDrainEvent(uint16_t watermarkWords, uint8_t maxDrainRoundsPerEvent);
    bool drainOneRound(uint16_t maxWordsPerDrain, Stream& out);
    bool enqueueRaw(const Lsm6dsv::RawSample& raw, bool checkStats, uint32_t queuedAtUs);
    bool enqueueMag(const Lsm6dsvFifoReader::MagRawSample& mag);
    bool dequeueRaw(Lsm6dsv::RawSample& raw, bool& checkStats);
    bool dequeueMag(Lsm6dsvFifoReader::MagRawSample& mag);
    bool peekMagTimestamp(uint64_t& timestampUs) const;
    bool dispatchDueMagCallbacks(uint64_t rawTimestampUs,
                                 uint32_t sliceStartUs,
                                 uint32_t sliceBudgetUs,
                                 uint8_t& magCallbacks,
                                 bool& worked);
    void clearQueues();
    void recordElapsed(uint32_t startUs);

    FifoInterruptEventSource* eventSource_ = nullptr;
    Lsm6dsvFifoReader* fifo_ = nullptr;
    Lsm6dsv::RawSample* rawDrainBuffer_ = nullptr;
    size_t rawDrainBufferCapacity_ = 0;
    Lsm6dsvFifoReader::MagRawSample* magDrainBuffer_ = nullptr;
    size_t magDrainBufferCapacity_ = 0;
    Lsm6dsv::RawSample* rawQueueBuffer_ = nullptr;
    uint8_t* rawQueueFlags_ = nullptr;
    size_t rawQueueCapacity_ = 0;
    Lsm6dsvFifoReader::MagRawSample* magQueueBuffer_ = nullptr;
    size_t magQueueCapacity_ = 0;
    FifoRuntimeSampleCallback sampleCallback_ = nullptr;
    FifoRuntimeMagCallback magCallback_ = nullptr;
    FifoRuntimeRecordTimeCallback recordTimeCallback_ = nullptr;
    void* callbackUser_ = nullptr;

    size_t rawQueueHead_ = 0;
    size_t rawQueueTail_ = 0;
    size_t rawQueueCount_ = 0;
    size_t magQueueHead_ = 0;
    size_t magQueueTail_ = 0;
    size_t magQueueCount_ = 0;
    bool drainActive_ = false;
    uint8_t drainRoundsRemaining_ = 0;
    uint64_t lastDispatchedRawTimestampUs_ = 0;
    static constexpr size_t kTrackedRawQueueCapacity = 512u;
    static_assert(cfg::FIFO_RUNTIME_RAW_QUEUE_CAPACITY <= kTrackedRawQueueCapacity,
                  "runtime queue age storage must cover the configured queue");
#if TRACKER_HAS_RUNTIME_PROFILER
    uint32_t rawQueueEnqueuedAtUs_[kTrackedRawQueueCapacity] = {};
#endif
    uint32_t lastDequeuedQueueAgeUs_ = 0;
    FifoRuntimeQueueStats queueStats_;
};

} // namespace tracker
