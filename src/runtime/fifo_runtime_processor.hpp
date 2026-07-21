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

class FifoRuntimeProcessor {
public:
    void begin(FifoInterruptEventSource* eventSource,
               Lsm6dsvFifoReader* fifo,
               Lsm6dsv::RawSample* rawBuffer,
               size_t rawBufferCapacity,
               Lsm6dsvFifoReader::MagRawSample* magBuffer,
               size_t magBufferCapacity,
               FifoRuntimeSampleCallback sampleCallback,
               FifoRuntimeMagCallback magCallback,
               FifoRuntimeRecordTimeCallback recordTimeCallback,
               void* callbackUser);

    bool process(uint16_t watermarkWords,
                 uint16_t maxWordsPerDrain,
                 uint8_t maxDrainRoundsPerEvent,
                 Stream& out);

    // Discard local pre-reset batches after an external/manual FIFO reset.
    // Hardware recovery initiated by a sample callback is handled directly
    // by process(), but command paths reset the hardware outside that call.
    void resetWork();

private:
    bool ready() const;
    bool hasPendingCallbacks() const;
    void clearPendingBatch();
    void recordElapsed(uint32_t startUs);

    FifoInterruptEventSource* eventSource_ = nullptr;
    Lsm6dsvFifoReader* fifo_ = nullptr;
    Lsm6dsv::RawSample* rawBuffer_ = nullptr;
    size_t rawBufferCapacity_ = 0;
    Lsm6dsvFifoReader::MagRawSample* magBuffer_ = nullptr;
    size_t magBufferCapacity_ = 0;
    FifoRuntimeSampleCallback sampleCallback_ = nullptr;
    FifoRuntimeMagCallback magCallback_ = nullptr;
    FifoRuntimeRecordTimeCallback recordTimeCallback_ = nullptr;
    void* callbackUser_ = nullptr;

    size_t pendingRawCount_ = 0;
    size_t pendingRawIndex_ = 0;
    size_t pendingMagCount_ = 0;
    size_t pendingMagIndex_ = 0;
    bool pendingCheckFifoStatsDelta_ = false;
    bool drainActive_ = false;
    uint8_t drainRoundsRemaining_ = 0;
};

} // namespace tracker
