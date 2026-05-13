#pragma once

#include <cstddef>
#include <cstdint>

#include "config/tracker_config_runtime.hpp"

namespace tracker {

enum class TrackerConfigError : uint8_t {
    None,
    NvsBeginFailed,
    NotFound,
    SizeMismatch,
    ReadFailed,
    WriteFailed,
    RemoveFailed,
    CrcOrValidationFailed,
};

struct TrackerConfigNvsInfo {
    bool beginOk = false;
    bool exists = false;
    size_t storedLen = 0;
    size_t expectedLen = sizeof(TrackerConfigBlob);
    uint32_t storedMagic = 0;
    uint16_t storedVersion = 0;
    uint16_t storedSize = 0;
    uint32_t storedCrc = 0;
    bool headerReadable = false;
    bool fullReadable = false;
    bool valid = false;
    TrackerConfigError error = TrackerConfigError::None;
};

class TrackerConfigStore {
public:
    explicit TrackerConfigStore(const char* nvsNamespace = tracker_config_detail::NVS_NAMESPACE,
                                const char* key = tracker_config_detail::NVS_KEY_CONFIG);

    TrackerConfigError lastError() const;
    const char* lastErrorName() const;
    static const char* errorName(TrackerConfigError e);

    bool inspect(TrackerConfigNvsInfo& info);
    bool load(TrackerConfig& out);
    bool loadOrDefaults(TrackerConfig& out, bool* loadedFromNvs = nullptr);
    bool save(TrackerConfig config);
    bool erase();
    bool exists();

private:
    const char* ns_ = nullptr;
    const char* key_ = nullptr;
    TrackerConfigError lastError_ = TrackerConfigError::None;
};

} // namespace tracker
