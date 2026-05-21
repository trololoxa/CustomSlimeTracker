#pragma once

#include <cstdint>

#include <Arduino.h>

#include "connection/lsm6dsv_fifo.hpp"
#include "network/wifi_manager.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "runtime/tracker_runtime_types.hpp"
#include "runtime/tracking_state_controller.hpp"
#include "sensor/imu_quality.hpp"

namespace tracker {

struct RuntimeLoopTimingSample {
    uint32_t loopUs = 0;
    uint32_t cliUs = 0;
    uint32_t fifoUs = 0;
    uint32_t networkUs = 0;
    uint32_t heartbeatUs = 0;
    bool fifoWorked = false;
    bool batteryWorked = false;
    bool networkWorked = false;
    bool tapWorked = false;
    bool ledWorked = false;
    bool heartbeatWorked = false;
    bool anyWork = false;
};

class RuntimeTestRunner {
public:
    struct Dependencies {
        const TrackerPerfCounters* perf = nullptr;
        const Lsm6dsvFifoReader* fifo = nullptr;
        const ImuQualityMonitor* quality = nullptr;
        const TrackerWifiManager* wifi = nullptr;
        const SlimeVROutputRuntime* slimevr = nullptr;
        const TrackingStateController* trackingState = nullptr;
        const uint32_t* runtimeSamples = nullptr;
        const float* latestTempC = nullptr;
        uint32_t progressPeriodMs = 30000UL;
    };

    void begin(const Dependencies& deps);
    bool active() const;
    bool start(uint32_t durationMs, uint32_t nowMs, Stream& out);
    bool stop();
    void printStatus(Stream& out, uint32_t nowMs) const;
    void recordLoopTiming(const RuntimeLoopTimingSample& timing);
    void update(uint32_t nowMs, Stream& out);

public:
    struct Stats {
        uint32_t count = 0;
        uint64_t sum = 0;
        uint32_t min = 0;
        uint32_t max = 0;

        void reset();
        void push(uint32_t v);
        float mean() const;
    };

private:
    struct Snapshot {
        uint32_t runtimeSamples = 0;
        uint32_t recoveryEnterCount = 0;
        TrackerPerfCounters perf;
        ImuQualityCounters quality;
        Lsm6dsvFifoReader::DrainStats fifo;
        TrackerWifiManagerStatus wifi;
        SlimeVROutputRuntimeStatus slime;
    };

    bool ready() const;
    Snapshot makeSnapshot() const;
    void reset();
    void printProgress(Stream& out, uint32_t nowMs) const;
    void finish(uint32_t nowMs, Stream& out);
    static uint32_t deltaU32(uint32_t current, uint32_t start);
    static uint64_t deltaU64(uint64_t current, uint64_t start);
    static float safeRate(uint32_t delta, float durationS);

    Dependencies deps_;
    bool active_ = false;
    bool stopRequested_ = false;
    uint32_t durationMs_ = 0;
    uint32_t startMs_ = 0;
    uint32_t lastProgressMs_ = 0;
    uint32_t loopCount_ = 0;
    uint32_t slowLoopCount_ = 0;
    uint32_t slowNetworkCount_ = 0;
    uint32_t slowFifoCount_ = 0;
    uint32_t workLoopCount_ = 0;
    uint32_t idleCandidateLoopCount_ = 0;
    uint32_t fifoWorkCount_ = 0;
    uint32_t batteryWorkCount_ = 0;
    uint32_t networkWorkCount_ = 0;
    uint32_t tapWorkCount_ = 0;
    uint32_t ledWorkCount_ = 0;
    uint32_t heartbeatWorkCount_ = 0;
    float tempStartC_ = 0.0f;
    float tempEndC_ = 0.0f;
    bool tempValid_ = false;

    Snapshot start_;
    Snapshot last_;
    Stats loopUs_;
    Stats cliUs_;
    Stats fifoUs_;
    Stats networkUs_;
    Stats heartbeatUs_;
};

} // namespace tracker
