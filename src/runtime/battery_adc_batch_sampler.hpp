#pragma once

#include <cstdint>

namespace tracker {

using BatteryAdcReadOneMillivoltsFn = bool (*)(uint16_t& outMillivolts, void* user);

struct BatteryAdcBatchSamplerConfig {
    uint8_t totalReads = 64u;
    uint8_t discardReads = 4u;
};

struct BatteryAdcBatchSamplerStatus {
    bool active = false;
    bool resultReady = false;
    bool resultValid = false;
    uint8_t attempts = 0u;
    uint8_t validReads = 0u;
    uint32_t batchesStarted = 0u;
    uint32_t batchesCompleted = 0u;
    uint32_t batchesFailed = 0u;
    uint32_t readFailures = 0u;
    uint32_t serviceCalls = 0u;
    uint8_t maxReadsPerService = 0u;
};

// Spreads a sparse ADC oversampling burst across bounded app-loop services.
// It preserves the previous discard + trimmed-mean estimator exactly for a
// deterministic input sequence, but never performs dozens of conversions in
// one tracker loop.
class BatteryAdcBatchSampler {
public:
    static constexpr uint8_t kMaxReads = 128u;

    void begin(BatteryAdcReadOneMillivoltsFn readOne, void* user = nullptr);
    void configure(const BatteryAdcBatchSamplerConfig& config);
    void reset();

    bool request();
    bool service(uint8_t maxReads);
    bool consume(uint16_t& outMillivolts);

    bool active() const { return status_.active; }
    bool resultReady() const { return status_.resultReady; }
    const BatteryAdcBatchSamplerStatus& status() const { return status_; }

private:
    void finish();
    void insertSorted(uint16_t value);

    BatteryAdcReadOneMillivoltsFn readOne_ = nullptr;
    void* user_ = nullptr;
    BatteryAdcBatchSamplerConfig config_;
    BatteryAdcBatchSamplerStatus status_;
    uint16_t reads_[kMaxReads] = {};
    uint16_t resultMillivolts_ = 0u;
};

} // namespace tracker
