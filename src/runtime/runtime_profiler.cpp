#include "runtime/runtime_profiler.hpp"

namespace tracker {

namespace {
const char* yesNo(bool v) { return v ? "yes" : "no"; }
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
    lastRecordMs_ = nowMs;
}

const RuntimeProfiler::SectionStats& RuntimeProfiler::stats(Section section) const {
    static const SectionStats empty;
    if (index(section) >= kSectionCount) return empty;
    return sections_[index(section)];
}

uint32_t RuntimeProfiler::windowMs(uint32_t nowMs) const {
    return nowMs - resetMs_;
}

const char* RuntimeProfiler::sectionName(Section section) {
    switch (section) {
        case Section::Loop:          return "loop";
        case Section::Cli:           return "cli";
        case Section::RemoteConsole: return "remote";
        case Section::Fifo:          return "fifo";
        case Section::Battery:       return "battery";
        case Section::Network:       return "network";
        case Section::Calibration0022:return "calibration_0022";
        case Section::Calibration0023:return "calibration_0023";
        case Section::Tap:           return "tap";
        case Section::Led:           return "led";
        case Section::Heartbeat:     return "heartbeat";
        case Section::IdleYield:     return "idle_yield";
        case Section::Count:         return "count";
    }
    return "unknown";
}

uint32_t RuntimeProfiler::slowThresholdUs(Section section) {
    switch (section) {
        case Section::Loop:          return TRACKER_RUNTIME_PROFILER_SLOW_LOOP_US;
        case Section::Cli:           return TRACKER_RUNTIME_PROFILER_SLOW_CLI_US;
        case Section::RemoteConsole: return TRACKER_RUNTIME_PROFILER_SLOW_REMOTE_US;
        case Section::Fifo:          return TRACKER_RUNTIME_PROFILER_SLOW_FIFO_US;
        case Section::Battery:       return TRACKER_RUNTIME_PROFILER_SLOW_BATTERY_US;
        case Section::Network:       return TRACKER_RUNTIME_PROFILER_SLOW_NETWORK_US;
        case Section::Calibration0022:return TRACKER_RUNTIME_PROFILER_SLOW_CALIBRATION_0022_US;
        case Section::Calibration0023:return TRACKER_RUNTIME_PROFILER_SLOW_CALIBRATION_0023_US;
        case Section::Tap:           return TRACKER_RUNTIME_PROFILER_SLOW_TAP_US;
        case Section::Led:           return TRACKER_RUNTIME_PROFILER_SLOW_LED_US;
        case Section::Heartbeat:     return TRACKER_RUNTIME_PROFILER_SLOW_HEARTBEAT_US;
        case Section::IdleYield:     return TRACKER_RUNTIME_PROFILER_SLOW_IDLE_YIELD_US;
        case Section::Count:         return UINT32_MAX;
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

void RuntimeProfiler::printSectionLine(Stream& out, Section section, const SectionStats& s, const SectionStats& loop) {
    out.print("perf_section="); out.print(sectionName(section));
    out.print(" calls="); out.print(s.calls);
    out.print(" worked="); out.print(s.worked);
    out.print(" avg_us="); out.print(meanUs(s), 3);
    out.print(" max_us="); out.print(s.maxUs);
    out.print(" last_us="); out.print(s.lastUs);
    out.print(" slow="); out.print(s.slow);
    out.print(" pct_loop="); out.println(section == Section::Loop ? 100.0f : pctOfLoop(s, loop), 2);
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
    out.print("perf_loop_max_us="); out.println(stats(Section::Loop).maxUs);
    out.print("perf_loop_slow="); out.println(stats(Section::Loop).slow);
    printTop(out, nowMs);
    out.println("# section,calls,worked,avg_us,max_us,last_us,slow,pct_loop");
    for (size_t i = 0; i < kSectionCount; ++i) {
        printSectionLine(out, static_cast<Section>(i), sections_[i], stats(Section::Loop));
    }
}

} // namespace tracker
