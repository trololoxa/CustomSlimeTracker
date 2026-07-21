#pragma once

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
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
    size_t rawQueueHighWater = 0;
    size_t magQueueHighWater = 0;
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
                 Stream& out);

    // Discard all samples captured before an external/manual FIFO reset.
    void resetWork();

    bool hasPendingWork() const;
    bool urgent() const;
    size_t rawQueueDepth() const;
    size_t magQueueDepth() const;
    const FifoRuntimeQueueStats& queueStats() const;

private:
    bool ready() const;
    bool beginDrainEvent(uint16_t watermarkWords, uint8_t maxDrainRoundsPerEvent);
    bool drainOneRound(uint16_t maxWordsPerDrain, Stream& out);
    bool enqueueRaw(const Lsm6dsv::RawSample& raw, bool checkStats);
    bool enqueueMag(const Lsm6dsvFifoReader::MagRawSample& mag);
    bool dequeueRaw(Lsm6dsv::RawSample& raw, bool& checkStats);
    bool dequeueMag(Lsm6dsvFifoReader::MagRawSample& mag);
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
    FifoRuntimeQueueStats queueStats_;
};

} // namespace tracker
