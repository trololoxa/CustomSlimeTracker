#pragma once

#include <Arduino.h>
#include <cmath>
#include <cstdint>

#include "defines.h"
#include "config/tracker_config_runtime.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "core/math.hpp"
#include "runtime/mag_runtime_state.hpp"
#include "network/wifi_manager.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "runtime/static_test_types.hpp"
#include "runtime/tracker_runtime_types.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/imu_quality.hpp"
#include "sensor/mag_heading.hpp"
#include "sensor/mag_runtime.hpp"
#include "sensor/mag_yaw_correction.hpp"

namespace tracker {

float staticTestAvgUs(uint64_t sumUs, uint32_t calls);
float staticTestAngleDiffDeg(float a, float b);

class StaticTestRunner {
public:
    struct Dependencies {
        StaticRuntimeTest* activeTest = nullptr;
        StaticRuntimeTest* lastCompletedTest = nullptr;
        bool* lastCompletedValid = nullptr;
        uint32_t* lastCompletedFinishedMs = nullptr;

        ImuQualityMonitor* quality = nullptr;
        Lsm6dsvFifoReader* fifo = nullptr;
        const TrackerPerfCounters* perf = nullptr;
        const TrackerConfig* config = nullptr;
        const Ahrs6Dof* ahrs = nullptr;
        const TrackerWifiManager* wifi = nullptr;
        const SlimeVROutputRuntime* slimevr = nullptr;

        const MagRuntimeProcessor* magProcessor = nullptr;
        const MagHeadingEstimator* magHeading = nullptr;
        const MagYawCorrectionController* magYawCorrection = nullptr;
        const MagHeadingReferenceState* magHeadingRef = nullptr;
        const MagYawCorrectionOutput* lastMagYawCorrection = nullptr;

        uint32_t progressPeriodMs = 30000UL;
    };

    void begin(const Dependencies& deps);
    bool active() const;
    const StaticRuntimeTest* lastCompleted() const;

    bool start(uint32_t durationMs, Stream& out, float magErrorStartDeg);
    bool stop(Stream& out, bool force = false);
    bool abortOutput(Stream& out);
    void printStatus(Stream& out) const;
    bool printLastSummary(Stream& out) const;
    bool printLastReport(Stream& out) const;

    void recordMagYawSample(float headingErrorDeg,
                            const MagHeadingSample& heading,
                            const MagYawCorrectionOutput& yaw);
    void updateSample(const Lsm6dsv::Sample& calibrated,
                      const ImuQualityResult& quality);

    void recordSampleProcessTime(uint32_t processUs);
    void recordFifoProcessTime(uint32_t processUs);
    void finish();

private:
    struct PerfDelta {
        uint32_t sampleCalls = 0;
        uint64_t sampleSumUs = 0;
        uint32_t fifoCalls = 0;
        uint64_t fifoSumUs = 0;
        uint32_t emptyPolls = 0;
        uint32_t irqEvents = 0;
        uint32_t fallbackPolls = 0;
        uint32_t fallbackEvents = 0;
    };

    bool ready() const;
    void snapshotPerfCounters(StaticRuntimeTest& t) const;
    PerfDelta perfDelta(const StaticRuntimeTest& t) const;
    void printProgress(Stream& out, uint32_t elapsedMs) const;
    void flushStats(StaticRuntimeTest& test);

    Dependencies deps_;
    Stream* output_ = nullptr;
    StaticTestStatsBlock statsBlock_;
};

} // namespace tracker
