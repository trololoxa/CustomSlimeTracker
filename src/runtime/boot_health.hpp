#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace tracker {

enum class BootResetClass : uint8_t {
    Cold = 0,
    PlannedSoftware,
    CrashOrWatchdog,
    Other,
};

inline const char* bootResetClassName(BootResetClass value) {
    switch (value) {
        case BootResetClass::Cold: return "cold";
        case BootResetClass::PlannedSoftware: return "software";
        case BootResetClass::CrashOrWatchdog: return "crash_or_watchdog";
        case BootResetClass::Other: return "other";
    }
    return "unknown";
}

struct BootHealthRecord {
    uint32_t magic = 0u;
    uint16_t version = 0u;
    uint16_t size = 0u;
    uint32_t sequence = 0u;
    uint32_t sequenceInv = 0u;
    uint16_t consecutiveCrashBoots = 0u;
    uint16_t reserved = 0u;
    uint32_t crc32 = 0u;
};

static_assert(sizeof(BootHealthRecord) == 24u, "BootHealthRecord layout changed");
static_assert(std::is_standard_layout<BootHealthRecord>::value,
              "BootHealthRecord must remain a stable retained-memory record");

class BootHealthController {
public:
    static constexpr uint32_t kMagic = 0x32484254UL; // "TBH2"
    static constexpr uint16_t kVersion = 1u;
    static constexpr uint16_t kSafeModeCrashBoots = 3u;

    void begin(BootHealthRecord& retained, BootResetClass resetClass) {
        const bool priorValid = valid(retained);
        uint16_t consecutive = 0u;
        uint32_t sequence = 1u;
        if (priorValid) {
            sequence = retained.sequence + 1u;
            if (sequence == 0u) sequence = 1u;
            if (resetClass == BootResetClass::CrashOrWatchdog) {
                consecutive = retained.consecutiveCrashBoots == 0xffffu
                    ? 0xffffu
                    : static_cast<uint16_t>(retained.consecutiveCrashBoots + 1u);
            }
        } else if (resetClass == BootResetClass::CrashOrWatchdog) {
            consecutive = 1u;
        }

        retained = make(sequence, consecutive);
        safeMode_ = consecutive >= kSafeModeCrashBoots;
        stable_ = false;
        resetClass_ = resetClass;
    }

    void markStable(BootHealthRecord& retained) {
        if (!valid(retained)) retained = make(1u, 0u);
        else retained = make(retained.sequence, 0u);
        safeMode_ = false;
        stable_ = true;
    }

    bool safeMode() const { return safeMode_; }
    bool stable() const { return stable_; }
    BootResetClass resetClass() const { return resetClass_; }

    static bool valid(const BootHealthRecord& record) {
        return record.magic == kMagic &&
               record.version == kVersion &&
               record.size == sizeof(BootHealthRecord) &&
               record.sequence != 0u &&
               record.sequenceInv == ~record.sequence &&
               record.crc32 == crc(record);
    }

    static uint32_t crc(const BootHealthRecord& record) {
        BootHealthRecord copy = record;
        copy.crc32 = 0u;
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&copy);
        uint32_t value = 0xFFFFFFFFu;
        for (size_t i = 0; i < sizeof(copy); ++i) {
            value ^= bytes[i];
            for (uint8_t bit = 0; bit < 8u; ++bit) {
                const uint32_t mask = 0u - (value & 1u);
                value = (value >> 1u) ^ (0xEDB88320u & mask);
            }
        }
        return ~value;
    }

private:
    static BootHealthRecord make(uint32_t sequence, uint16_t consecutive) {
        BootHealthRecord record;
        record.magic = kMagic;
        record.version = kVersion;
        record.size = sizeof(BootHealthRecord);
        record.sequence = sequence;
        record.sequenceInv = ~sequence;
        record.consecutiveCrashBoots = consecutive;
        record.crc32 = crc(record);
        return record;
    }

    bool safeMode_ = false;
    bool stable_ = false;
    BootResetClass resetClass_ = BootResetClass::Other;
};

} // namespace tracker
