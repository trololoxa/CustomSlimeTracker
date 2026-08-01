#pragma once

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

#include "defines.h"

namespace tracker {

// Fixed-memory latency histogram used only by the optional runtime profiler.
// Recording is one bounded bucket lookup; no heap, sorting, or raw sample log.
struct RuntimeLatencyHistogram {
    static constexpr size_t kBucketCount = 24u;

    uint32_t buckets[kBucketCount] = {};
    uint32_t count = 0;
    uint32_t maxUs = 0;

    void reset();
    void record(uint32_t elapsedUs);
    uint32_t percentileUpperBoundUs(uint8_t percentile) const;
};

class RuntimeProfiler {
public:
    enum class Section : uint8_t {
        Loop = 0,
        Cli,
        RemoteConsole,
        Fifo,
        Battery,
        Network,
        NetworkNested,
        Calibration0022,
        Calibration0023,
        RuntimeBiasDeferred,
        Tap,
        Led,
        Heartbeat,
        IdleYield,
        Count
    };

    struct SectionStats {
        uint32_t calls = 0;
        uint32_t worked = 0;
        uint64_t sumUs = 0;
        uint32_t maxUs = 0;
        uint32_t lastUs = 0;
        uint32_t slow = 0;
        RuntimeLatencyHistogram latency;
    };

    enum class ImuStage : uint8_t {
        ScaleAndCalibration = 0,
        Quality,
        AhrsAndRecovery,
        PreparedOutput,
        PerSampleOutputs,
        Count
    };

    enum class OptionalService : uint8_t {
        Battery = 0,
        Led,
        MagDeferred,
        CalibrationAutonomy,
        RemoteConsole,
        Count
    };

    struct SampledStageStats {
        uint32_t samples = 0u;
        uint64_t sumUs = 0u;
        uint32_t maxUs = 0u;
        RuntimeLatencyHistogram latency;
    };

    struct FrameStats {
        uint32_t completed = 0;
        uint32_t overBudget = 0;
        uint64_t busySumUs = 0;
        uint32_t busyMaxUs = 0;
        RuntimeLatencyHistogram busy;
        RuntimeLatencyHistogram headroom;
    };

    void begin(uint32_t nowMs);
    void setEnabled(bool enabled, uint32_t nowMs);
    bool enabled() const { return enabled_; }
    void reset(uint32_t nowMs);
    void record(Section section, uint32_t elapsedUs, bool worked, uint32_t nowMs);

    // Account one complete app-loop interval into absolute 10 ms frame bins.
    // This is called once after the loop has completed and therefore adds no
    // timing probes inside the 960 Hz sample path.
    void recordLoopInterval(uint32_t loopStartUs, uint32_t elapsedUs);
    void recordProfilerOverhead(uint32_t elapsedUs);

    // Software-pipeline freshness telemetry. queueAgeUs is exact wait after
    // hardware FIFO drain; prepared/rotation add publication/scheduler delay.
    // Hardware-FIFO residence is reported separately and is not hidden inside
    // these lower-bound ages.
    void recordProcessedSoftwareAge(uint32_t queueAgeUs);
    void recordPreparedSoftwareAge(uint32_t ageUs);
    void recordRotationSoftwareAge(uint32_t ageUs);
    void recordOptionalServiceAdmissionSkip(OptionalService service);
    void recordImuStage(ImuStage stage, uint32_t elapsedUs);

    void printStatus(Stream& out, uint32_t nowMs) const;
    void printTop(Stream& out, uint32_t nowMs) const;

    const SectionStats& stats(Section section) const;
    const FrameStats& frameStats() const { return frames_; }
    const RuntimeLatencyHistogram& processedSoftwareAge() const { return processedSoftwareAge_; }
    const RuntimeLatencyHistogram& preparedSoftwareAge() const { return preparedSoftwareAge_; }
    const RuntimeLatencyHistogram& rotationSoftwareAge() const { return rotationSoftwareAge_; }
    const RuntimeLatencyHistogram& profilerOverhead() const { return profilerOverhead_; }
    const SampledStageStats& imuStageStats(ImuStage stage) const;
    uint32_t windowMs(uint32_t nowMs) const;

    static const char* sectionName(Section section);
    static const char* optionalServiceName(OptionalService service);
    static const char* imuStageName(ImuStage stage);

private:
    static constexpr size_t kSectionCount = static_cast<size_t>(Section::Count);
    static constexpr uint32_t kFramePeriodUs = 10000u;

    static size_t index(Section section) { return static_cast<size_t>(section); }
    static uint32_t slowThresholdUs(Section section);
    static float meanUs(const SectionStats& s);
    static float pctOfLoop(const SectionStats& section, const SectionStats& loop);
    static void printSectionLine(Stream& out,
                                 Section section,
                                 const SectionStats& stats,
                                 const SectionStats& loop);
    static void printHistogramLine(Stream& out,
                                   const char* prefix,
                                   const RuntimeLatencyHistogram& histogram);

    uint64_t extendMicros(uint32_t valueUs);
    void finalizeFrame();

    bool enabled_ = false;
    bool configured_ = false;
    uint32_t resetMs_ = 0;
    uint32_t lastRecordMs_ = 0;
    SectionStats sections_[kSectionCount] = {};

    FrameStats frames_;
    RuntimeLatencyHistogram processedSoftwareAge_;
    RuntimeLatencyHistogram preparedSoftwareAge_;
    RuntimeLatencyHistogram rotationSoftwareAge_;
    RuntimeLatencyHistogram profilerOverhead_;
    static constexpr size_t kOptionalServiceCount =
        static_cast<size_t>(OptionalService::Count);
    uint32_t optionalServiceAdmissionSkips_[kOptionalServiceCount] = {};
    static constexpr size_t kImuStageCount =
        static_cast<size_t>(ImuStage::Count);
    SampledStageStats imuStages_[kImuStageCount] = {};

    bool microsExtensionValid_ = false;
    uint32_t lastMicros32_ = 0;
    uint64_t microsHigh_ = 0;
    bool frameValid_ = false;
    uint64_t frameStartUs_ = 0;
    uint32_t frameBusyUs_ = 0;
};

} // namespace tracker
