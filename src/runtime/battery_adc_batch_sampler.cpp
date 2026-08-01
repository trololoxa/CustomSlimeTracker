#include "runtime/battery_adc_batch_sampler.hpp"

namespace tracker {

void BatteryAdcBatchSampler::begin(BatteryAdcReadOneMillivoltsFn readOne, void* user) {
    readOne_ = readOne;
    user_ = user;
    reset();
}

void BatteryAdcBatchSampler::configure(const BatteryAdcBatchSamplerConfig& config) {
    config_ = config;
    if (config_.totalReads == 0u) config_.totalReads = 1u;
    if (config_.totalReads > kMaxReads) config_.totalReads = kMaxReads;
    if (config_.discardReads >= config_.totalReads) {
        config_.discardReads = config_.totalReads > 1u
            ? static_cast<uint8_t>(config_.totalReads - 1u)
            : 0u;
    }
}

void BatteryAdcBatchSampler::reset() {
    status_ = BatteryAdcBatchSamplerStatus{};
    resultMillivolts_ = 0u;
    for (uint16_t& read : reads_) read = 0u;
}

bool BatteryAdcBatchSampler::request() {
    if (readOne_ == nullptr || status_.active || status_.resultReady) return false;
    status_.active = true;
    status_.attempts = 0u;
    status_.validReads = 0u;
    status_.resultValid = false;
    resultMillivolts_ = 0u;
    ++status_.batchesStarted;
    return true;
}

bool BatteryAdcBatchSampler::service(uint8_t maxReads) {
    if (!status_.active || readOne_ == nullptr || maxReads == 0u) return false;
    ++status_.serviceCalls;
    uint8_t performed = 0u;
    while (status_.active && performed < maxReads) {
        uint16_t millivolts = 0u;
        const bool ok = readOne_(millivolts, user_);
        const uint8_t attempt = status_.attempts++;
        ++performed;
        if (attempt >= config_.discardReads) {
            if (ok && status_.validReads < kMaxReads) {
                insertSorted(millivolts);
            } else if (!ok) {
                ++status_.readFailures;
            }
        }
        if (status_.attempts >= config_.totalReads) finish();
    }
    if (performed > status_.maxReadsPerService) {
        status_.maxReadsPerService = performed;
    }
    return performed != 0u;
}

void BatteryAdcBatchSampler::finish() {
    status_.active = false;
    status_.resultReady = true;
    status_.resultValid = status_.validReads != 0u;
    ++status_.batchesCompleted;
    if (!status_.resultValid) {
        ++status_.batchesFailed;
        resultMillivolts_ = 0u;
        return;
    }

    uint8_t trim = 0u;
    if (status_.validReads >= 16u) {
        trim = static_cast<uint8_t>(status_.validReads / 8u);
    } else if (status_.validReads >= 5u) {
        trim = 1u;
    }

    uint8_t begin = trim;
    uint8_t end = static_cast<uint8_t>(status_.validReads - trim);
    if (begin >= end) {
        begin = 0u;
        end = status_.validReads;
    }

    uint32_t sum = 0u;
    uint8_t used = 0u;
    for (uint8_t i = begin; i < end; ++i) {
        sum += reads_[i];
        ++used;
    }
    status_.resultValid = used != 0u;
    if (!status_.resultValid) {
        ++status_.batchesFailed;
        resultMillivolts_ = 0u;
        return;
    }
    resultMillivolts_ = static_cast<uint16_t>((sum + used / 2u) / used);
}

bool BatteryAdcBatchSampler::consume(uint16_t& outMillivolts) {
    if (!status_.resultReady) return false;
    const bool valid = status_.resultValid;
    outMillivolts = valid ? resultMillivolts_ : 0u;
    status_.resultReady = false;
    status_.resultValid = false;
    status_.attempts = 0u;
    status_.validReads = 0u;
    resultMillivolts_ = 0u;
    return valid;
}

void BatteryAdcBatchSampler::insertSorted(uint16_t value) {
    uint8_t index = status_.validReads;
    while (index > 0u && reads_[index - 1u] > value) {
        reads_[index] = reads_[index - 1u];
        --index;
    }
    reads_[index] = value;
    ++status_.validReads;
}

} // namespace tracker
