#pragma once

#include <Arduino.h>
#include <cstdint>

#include "defines.h"

namespace tracker {

class RuntimeProfiler {
public:
    enum class Section : uint8_t {
        Loop = 0,
        Cli,
        RemoteConsole,
        Fifo,
        Battery,
        Network,
        Calibration0022,
        Calibration0023,
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
    };

    void begin(uint32_t nowMs);
    void setEnabled(bool enabled, uint32_t nowMs);
    bool enabled() const { return enabled_; }
    void reset(uint32_t nowMs);
    void record(Section section, uint32_t elapsedUs, bool worked, uint32_t nowMs);
    void printStatus(Stream& out, uint32_t nowMs) const;
    void printTop(Stream& out, uint32_t nowMs) const;

    const SectionStats& stats(Section section) const;
    uint32_t windowMs(uint32_t nowMs) const;

    static const char* sectionName(Section section);

private:
    static constexpr size_t kSectionCount = static_cast<size_t>(Section::Count);

    static size_t index(Section section) { return static_cast<size_t>(section); }
    static uint32_t slowThresholdUs(Section section);
    static float meanUs(const SectionStats& s);
    static float pctOfLoop(const SectionStats& section, const SectionStats& loop);
    static void printSectionLine(Stream& out, Section section, const SectionStats& stats, const SectionStats& loop);

    bool enabled_ = false;
    bool configured_ = false;
    uint32_t resetMs_ = 0;
    uint32_t lastRecordMs_ = 0;
    SectionStats sections_[kSectionCount] = {};
};


} // namespace tracker
