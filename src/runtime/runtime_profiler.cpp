#include "runtime/runtime_profiler.hpp"

#include <algorithm>

namespace tracker {

namespace {
const char* yesNo(bool v) { return v ? "yes" : "no"; }

// Upper bounds are intentionally dense around the 10 ms output deadline.
constexpr uint32_t kLatencyUpperBoundsUs[RuntimeLatencyHistogram::kBucketCount] = {
    0u, 10u, 25u, 50u, 100u, 250u, 500u, 1000u, 2000u,
    3500u, 5000u, 7500u, 10000u, 15000u, 25000u, 50000u,
    100000u, 150000u, 250000u, 500000u, 750000u, 1000000u,
    2000000u, 0xffffffffu
};
} // namespace

void RuntimeLatencyHistogram::reset() {
    for (uint32_t& bucket : buckets) bucket = 0u;
    count = 0u;
    maxUs = 0u;
}

void RuntimeLatencyHistogram::record(uint32_t elapsedUs) {
    // Bounded lower-bound search keeps profiler overhead independent of the
    // extended overload range (five comparisons for 24 buckets).
    size_t low = 0u;
    size_t high = kBucketCount - 1u;
    while (low < high) {
        const size_t mid = low + ((high - low) / 2u);
        if (elapsedUs <= kLatencyUpperBoundsUs[mid]) {
            high = mid;
        } else {
            low = mid + 1u;
        }
    }
    const size_t bucket = low;
    ++buckets[bucket];
    ++count;
    if (elapsedUs > maxUs) maxUs = elapsedUs;
}

uint32_t RuntimeLatencyHistogram::percentileUpperBoundUs(uint8_t percentile) const {
    if (count == 0u) return 0u;
    if (percentile == 0u) percentile = 1u;
    if (percentile > 100u) percentile = 100u;
    const uint64_t target =
        (static_cast<uint64_t>(count) * percentile + 99u) / 100u;
    uint64_t cumulative = 0u;
    for (size_t i = 0; i < kBucketCount; ++i) {
        cumulative += buckets[i];
        if (cumulative >= target) {
            // The final bucket is open-ended. Returning UINT32_MAX made a
            // real overload distribution unreadable; report the observed
            // maximum instead while keeping all recording bounded.
            return i + 1u == kBucketCount ? maxUs : kLatencyUpperBoundsUs[i];
        }
    }
    return maxUs;
}

void RuntimeProfiler::begin(uint32_t nowMs) {
    configured_ = true;
    enabled_ = TRACKER_ENABLE_RUNTIME_PROFILER_DEFAULT_ON != 0;
    reset(nowMs);
}

void RuntimeProfiler::setEnabled(bool enabled, uint32_t nowMs) {
    enabled_ = enabled;
    reset(nowMs);
}

void RuntimeProfiler::reset(uint32_t nowMs) {
    resetMs_ = nowMs;
    lastRecordMs_ = 0;
    for (auto& section : sections_) section = SectionStats{};
    frames_ = FrameStats{};
    processedSoftwareAge_.reset();
    preparedSoftwareAge_.reset();
    rotationSoftwareAge_.reset();
    profilerOverhead_.reset();
    for (uint32_t& count : optionalServiceAdmissionSkips_) count = 0u;
    for (SampledStageStats& stage : imuStages_) stage = SampledStageStats{};
    microsExtensionValid_ = false;
    lastMicros32_ = 0u;
    microsHigh_ = 0u;
    frameValid_ = false;
    frameStartUs_ = 0u;
    frameBusyUs_ = 0u;
}

void RuntimeProfiler::record(Section section, uint32_t elapsedUs, bool worked, uint32_t nowMs) {
    if (!enabled_) return;
    if (index(section) >= kSectionCount) return;
    SectionStats& s = sections_[index(section)];
    ++s.calls;
    if (worked) ++s.worked;
    s.sumUs += elapsedUs;
    s.lastUs = elapsedUs;
    if (elapsedUs > s.maxUs) s.maxUs = elapsedUs;
    if (elapsedUs > slowThresholdUs(section)) ++s.slow;
    s.latency.record(elapsedUs);
    lastRecordMs_ = nowMs;
}

uint64_t RuntimeProfiler::extendMicros(uint32_t valueUs) {
    if (!microsExtensionValid_) {
        microsExtensionValid_ = true;
        lastMicros32_ = valueUs;
        return valueUs;
    }
    if (valueUs < lastMicros32_ &&
        static_cast<uint32_t>(lastMicros32_ - valueUs) > 0x80000000u) {
        microsHigh_ += 0x100000000ULL;
    }
    lastMicros32_ = valueUs;
    return microsHigh_ + valueUs;
}

void RuntimeProfiler::finalizeFrame() {
    const uint32_t busyUs = std::min(frameBusyUs_, kFramePeriodUs);
    const uint32_t headroomUs = kFramePeriodUs - busyUs;
    ++frames_.completed;
    frames_.busySumUs += busyUs;
    if (busyUs > frames_.busyMaxUs) frames_.busyMaxUs = busyUs;
    if (frameBusyUs_ >= kFramePeriodUs) ++frames_.overBudget;
    frames_.busy.record(busyUs);
    frames_.headroom.record(headroomUs);
    frameBusyUs_ = 0u;
}

void RuntimeProfiler::recordLoopInterval(uint32_t loopStartUs, uint32_t elapsedUs) {
    if (!enabled_ || elapsedUs == 0u) return;

    uint64_t cursorUs = extendMicros(loopStartUs);
    uint64_t endUs = cursorUs + elapsedUs;
    if (!frameValid_) {
        frameStartUs_ = cursorUs - (cursorUs % kFramePeriodUs);
        frameValid_ = true;
    }

    // Close any fully idle frames before this loop interval. Normally the app
    // spin-loop has none; recording them makes future delay/yield policy visible.
    while (cursorUs >= frameStartUs_ + kFramePeriodUs) {
        finalizeFrame();
        frameStartUs_ += kFramePeriodUs;
    }

    while (cursorUs < endUs) {
        const uint64_t frameEndUs = frameStartUs_ + kFramePeriodUs;
        const uint64_t segmentEndUs = std::min(endUs, frameEndUs);
        frameBusyUs_ += static_cast<uint32_t>(segmentEndUs - cursorUs);
        cursorUs = segmentEndUs;
        if (cursorUs >= frameEndUs) {
            finalizeFrame();
            frameStartUs_ = frameEndUs;
        }
    }
}

void RuntimeProfiler::recordProfilerOverhead(uint32_t elapsedUs) {
    if (enabled_) profilerOverhead_.record(elapsedUs);
}

void RuntimeProfiler::recordProcessedSoftwareAge(uint32_t queueAgeUs) {
    if (enabled_) processedSoftwareAge_.record(queueAgeUs);
}

void RuntimeProfiler::recordPreparedSoftwareAge(uint32_t ageUs) {
    if (enabled_) preparedSoftwareAge_.record(ageUs);
}

void RuntimeProfiler::recordRotationSoftwareAge(uint32_t ageUs) {
    if (enabled_) rotationSoftwareAge_.record(ageUs);
}

void RuntimeProfiler::recordOptionalServiceAdmissionSkip(OptionalService service) {
    if (!enabled_) return;
    const size_t i = static_cast<size_t>(service);
    if (i < kOptionalServiceCount) {
        ++optionalServiceAdmissionSkips_[i];
    }
}

void RuntimeProfiler::recordImuStage(ImuStage stage, uint32_t elapsedUs) {
    if (!enabled_) return;
    const size_t i = static_cast<size_t>(stage);
    if (i >= kImuStageCount) return;
    SampledStageStats& stats = imuStages_[i];
    ++stats.samples;
    stats.sumUs += elapsedUs;
    if (elapsedUs > stats.maxUs) stats.maxUs = elapsedUs;
    stats.latency.record(elapsedUs);
}

const RuntimeProfiler::SectionStats& RuntimeProfiler::stats(Section section) const {
    static const SectionStats empty;
    if (index(section) >= kSectionCount) return empty;
    return sections_[index(section)];
}

const RuntimeProfiler::SampledStageStats& RuntimeProfiler::imuStageStats(
    ImuStage stage) const {
    static const SampledStageStats empty;
    const size_t i = static_cast<size_t>(stage);
    return i < kImuStageCount ? imuStages_[i] : empty;
}

uint32_t RuntimeProfiler::windowMs(uint32_t nowMs) const {
    return nowMs - resetMs_;
}

const char* RuntimeProfiler::sectionName(Section section) {
    switch (section) {
        case Section::Loop:           return "loop";
        case Section::Cli:            return "cli";
        case Section::RemoteConsole:  return "remote";
        case Section::Fifo:           return "fifo";
        case Section::Battery:        return "battery";
        case Section::Network:        return "network_outer";
        case Section::NetworkNested:  return "network_nested";
        case Section::Calibration0022:return "calibration_0022";
        case Section::Calibration0023:return "calibration_0023";
        case Section::RuntimeBiasDeferred:return "runtime_bias_deferred";
        case Section::Tap:            return "tap";
        case Section::Led:            return "led";
        case Section::Heartbeat:      return "heartbeat";
        case Section::IdleYield:      return "idle_yield";
        case Section::Count:          return "count";
    }
    return "unknown";
}

const char* RuntimeProfiler::optionalServiceName(OptionalService service) {
    switch (service) {
        case OptionalService::Battery: return "battery";
        case OptionalService::Led: return "led";
        case OptionalService::MagDeferred: return "mag_deferred";
        case OptionalService::CalibrationAutonomy: return "calibration_autonomy";
        case OptionalService::RemoteConsole: return "remote_console";
        case OptionalService::Count: break;
    }
    return "unknown";
}

const char* RuntimeProfiler::imuStageName(ImuStage stage) {
    switch (stage) {
        case ImuStage::ScaleAndCalibration: return "scale_calibration";
        case ImuStage::Quality: return "quality";
        case ImuStage::AhrsAndRecovery: return "ahrs_recovery";
        case ImuStage::PreparedOutput: return "prepared_output";
        case ImuStage::PerSampleOutputs: return "per_sample_outputs";
        case ImuStage::Count: break;
    }
    return "unknown";
}

uint32_t RuntimeProfiler::slowThresholdUs(Section section) {
    switch (section) {
        case Section::Loop:           return TRACKER_RUNTIME_PROFILER_SLOW_LOOP_US;
        case Section::Cli:            return TRACKER_RUNTIME_PROFILER_SLOW_CLI_US;
        case Section::RemoteConsole:  return TRACKER_RUNTIME_PROFILER_SLOW_REMOTE_US;
        case Section::Fifo:           return TRACKER_RUNTIME_PROFILER_SLOW_FIFO_US;
        case Section::Battery:        return TRACKER_RUNTIME_PROFILER_SLOW_BATTERY_US;
        case Section::Network:
        case Section::NetworkNested:  return TRACKER_RUNTIME_PROFILER_SLOW_NETWORK_US;
        case Section::Calibration0022:return TRACKER_RUNTIME_PROFILER_SLOW_CALIBRATION_0022_US;
        case Section::Calibration0023:return TRACKER_RUNTIME_PROFILER_SLOW_CALIBRATION_0023_US;
        case Section::RuntimeBiasDeferred:return TRACKER_RUNTIME_PROFILER_SLOW_CALIBRATION_0023_US;
        case Section::Tap:            return TRACKER_RUNTIME_PROFILER_SLOW_TAP_US;
        case Section::Led:            return TRACKER_RUNTIME_PROFILER_SLOW_LED_US;
        case Section::Heartbeat:      return TRACKER_RUNTIME_PROFILER_SLOW_HEARTBEAT_US;
        case Section::IdleYield:      return TRACKER_RUNTIME_PROFILER_SLOW_IDLE_YIELD_US;
        case Section::Count:          return UINT32_MAX;
    }
    return UINT32_MAX;
}

float RuntimeProfiler::meanUs(const SectionStats& s) {
    return s.calls == 0 ? 0.0f : static_cast<float>(s.sumUs) / static_cast<float>(s.calls);
}

float RuntimeProfiler::pctOfLoop(const SectionStats& section, const SectionStats& loop) {
    if (loop.sumUs == 0) return 0.0f;
    return 100.0f * static_cast<float>(section.sumUs) / static_cast<float>(loop.sumUs);
}

void RuntimeProfiler::printSectionLine(Stream& out,
                                       Section section,
                                       const SectionStats& s,
                                       const SectionStats& loop) {
    out.print("perf_section="); out.print(sectionName(section));
    out.print(" calls="); out.print(s.calls);
    out.print(" worked="); out.print(s.worked);
    out.print(" avg_us="); out.print(meanUs(s), 3);
    out.print(" p50_us="); out.print(s.latency.percentileUpperBoundUs(50));
    out.print(" p95_us="); out.print(s.latency.percentileUpperBoundUs(95));
    out.print(" p99_us="); out.print(s.latency.percentileUpperBoundUs(99));
    out.print(" max_us="); out.print(s.maxUs);
    out.print(" last_us="); out.print(s.lastUs);
    out.print(" slow="); out.print(s.slow);
    out.print(" pct_loop="); out.println(section == Section::Loop ? 100.0f : pctOfLoop(s, loop), 2);
}

void RuntimeProfiler::printHistogramLine(Stream& out,
                                         const char* prefix,
                                         const RuntimeLatencyHistogram& h) {
    out.print(prefix); out.print("_count="); out.println(h.count);
    out.print(prefix); out.print("_p50_us="); out.println(h.percentileUpperBoundUs(50));
    out.print(prefix); out.print("_p95_us="); out.println(h.percentileUpperBoundUs(95));
    out.print(prefix); out.print("_p99_us="); out.println(h.percentileUpperBoundUs(99));
    out.print(prefix); out.print("_max_us="); out.println(h.maxUs);
}

void RuntimeProfiler::printTop(Stream& out, uint32_t nowMs) const {
    (void)nowMs;
    Section topAvg = Section::Cli;
    Section topMax = Section::Cli;
    float topAvgUs = 0.0f;
    uint32_t topMaxUs = 0;

    for (size_t i = 1; i < kSectionCount; ++i) {
        const auto section = static_cast<Section>(i);
        const SectionStats& s = sections_[i];
        const float avg = meanUs(s);
        if (avg > topAvgUs) {
            topAvgUs = avg;
            topAvg = section;
        }
        if (s.maxUs > topMaxUs) {
            topMaxUs = s.maxUs;
            topMax = section;
        }
    }

    out.print("perf_top_avg_section="); out.println(sectionName(topAvg));
    out.print("perf_top_avg_us="); out.println(topAvgUs, 3);
    out.print("perf_top_max_section="); out.println(sectionName(topMax));
    out.print("perf_top_max_us="); out.println(topMaxUs);
}

void RuntimeProfiler::printStatus(Stream& out, uint32_t nowMs) const {
    out.println("# RUNTIME PERF");
    out.print("perf_compiled="); out.println(TRACKER_ENABLE_RUNTIME_PROFILER ? "yes" : "no");
    out.print("perf_configured="); out.println(yesNo(configured_));
    out.print("perf_enabled="); out.println(yesNo(enabled_));
    out.print("perf_window_ms="); out.println(windowMs(nowMs));
    out.print("perf_last_record_age_ms="); out.println(lastRecordMs_ == 0 ? 0 : nowMs - lastRecordMs_);
    out.print("perf_loop_calls="); out.println(stats(Section::Loop).calls);
    out.print("perf_loop_avg_us="); out.println(meanUs(stats(Section::Loop)), 3);
    out.print("perf_loop_p95_us="); out.println(stats(Section::Loop).latency.percentileUpperBoundUs(95));
    out.print("perf_loop_p99_us="); out.println(stats(Section::Loop).latency.percentileUpperBoundUs(99));
    out.print("perf_loop_max_us="); out.println(stats(Section::Loop).maxUs);
    out.print("perf_loop_slow="); out.println(stats(Section::Loop).slow);

    out.print("perf_frame_period_us="); out.println(kFramePeriodUs);
    out.print("perf_frame_completed="); out.println(frames_.completed);
    out.print("perf_frame_over_budget="); out.println(frames_.overBudget);
    out.print("perf_frame_busy_avg_us=");
    out.println(frames_.completed == 0u
        ? 0.0f
        : static_cast<float>(frames_.busySumUs) / static_cast<float>(frames_.completed), 3);
    out.print("perf_frame_busy_p50_us="); out.println(frames_.busy.percentileUpperBoundUs(50));
    out.print("perf_frame_busy_p95_us="); out.println(frames_.busy.percentileUpperBoundUs(95));
    out.print("perf_frame_busy_p99_us="); out.println(frames_.busy.percentileUpperBoundUs(99));
    out.print("perf_frame_busy_max_us="); out.println(frames_.busyMaxUs);
    // Low headroom percentiles are the useful tail: p01/p05 show the worst
    // frames without retaining every frame sample.
    out.print("perf_frame_headroom_p01_us="); out.println(frames_.headroom.percentileUpperBoundUs(1));
    out.print("perf_frame_headroom_p05_us="); out.println(frames_.headroom.percentileUpperBoundUs(5));
    out.print("perf_frame_headroom_p50_us="); out.println(frames_.headroom.percentileUpperBoundUs(50));

    printHistogramLine(out, "perf_processed_software_age", processedSoftwareAge_);
    printHistogramLine(out, "perf_prepared_software_age", preparedSoftwareAge_);
    printHistogramLine(out, "perf_rotation_software_age", rotationSoftwareAge_);
    printHistogramLine(out, "perf_profiler_overhead", profilerOverhead_);
    for (size_t i = 0; i < kOptionalServiceCount; ++i) {
        out.print("perf_optional_service_admission_skips_");
        out.print(optionalServiceName(static_cast<OptionalService>(i)));
        out.print('=');
        out.println(optionalServiceAdmissionSkips_[i]);
    }
    out.print("perf_imu_stage_sample_divisor=");
    out.println(TRACKER_IMU_STAGE_PROFILER_SAMPLE_DIVISOR);
    for (size_t i = 0; i < kImuStageCount; ++i) {
        const SampledStageStats& stage = imuStages_[i];
        out.print("perf_imu_stage=");
        out.print(imuStageName(static_cast<ImuStage>(i)));
        out.print(" samples="); out.print(stage.samples);
        out.print(" avg_us=");
        out.print(stage.samples == 0u
            ? 0.0f
            : static_cast<float>(stage.sumUs) / static_cast<float>(stage.samples), 3);
        out.print(" p95_us="); out.print(stage.latency.percentileUpperBoundUs(95));
        out.print(" p99_us="); out.print(stage.latency.percentileUpperBoundUs(99));
        out.print(" max_us="); out.println(stage.maxUs);
    }

    printTop(out, nowMs);
    out.println("# section,calls,worked,avg_us,p50_us,p95_us,p99_us,max_us,last_us,slow,pct_loop");
    for (size_t i = 0; i < kSectionCount; ++i) {
        printSectionLine(out, static_cast<Section>(i), sections_[i], stats(Section::Loop));
    }
}

} // namespace tracker
