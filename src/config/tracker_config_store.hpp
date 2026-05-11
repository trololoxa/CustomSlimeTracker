#pragma once

#include <Arduino.h>
#include <Preferences.h>
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
                                const char* key = tracker_config_detail::NVS_KEY_CONFIG)
        : ns_(nvsNamespace), key_(key) {}

    TrackerConfigError lastError() const {
        return lastError_;
    }

    const char* lastErrorName() const {
        return errorName(lastError_);
    }

    static const char* errorName(TrackerConfigError e) {
        switch (e) {
            case TrackerConfigError::None:                  return "None";
            case TrackerConfigError::NvsBeginFailed:        return "NvsBeginFailed";
            case TrackerConfigError::NotFound:              return "NotFound";
            case TrackerConfigError::SizeMismatch:          return "SizeMismatch";
            case TrackerConfigError::ReadFailed:            return "ReadFailed";
            case TrackerConfigError::WriteFailed:           return "WriteFailed";
            case TrackerConfigError::RemoveFailed:          return "RemoveFailed";
            case TrackerConfigError::CrcOrValidationFailed: return "CrcOrValidationFailed";
        }
        return "Unknown";
    }

    bool inspect(TrackerConfigNvsInfo& info) {
        info = TrackerConfigNvsInfo{};
        Preferences prefs;
        if (!prefs.begin(ns_, true)) {
            info.error = TrackerConfigError::NvsBeginFailed;
            lastError_ = info.error;
            return false;
        }

        info.beginOk = true;
        info.storedLen = prefs.getBytesLength(key_);
        info.exists = info.storedLen > 0;

        if (!info.exists) {
            prefs.end();
            info.error = TrackerConfigError::NotFound;
            lastError_ = info.error;
            return true;
        }

        if (info.storedLen > sizeof(TrackerConfigBlob)) {
            prefs.end();
            info.error = TrackerConfigError::SizeMismatch;
            lastError_ = info.error;
            return true;
        }

        TrackerConfig tmp;
        const size_t readLen = prefs.getBytes(key_, &tmp.data, sizeof(tmp.data));
        prefs.end();

        if (readLen != info.storedLen) {
            info.error = TrackerConfigError::ReadFailed;
            lastError_ = info.error;
            return true;
        }

        if (readLen >= sizeof(tmp.data.magic) +
                       sizeof(tmp.data.version) +
                       sizeof(tmp.data.size) +
                       sizeof(tmp.data.crc32)) {
            info.headerReadable = true;
            info.storedMagic = tmp.data.magic;
            info.storedVersion = tmp.data.version;
            info.storedSize = tmp.data.size;
            info.storedCrc = tmp.data.crc32;
        }

        if (info.storedLen != sizeof(TrackerConfigBlob)) {
            info.error = TrackerConfigError::SizeMismatch;
            lastError_ = info.error;
            return true;
        }

        info.fullReadable = true;
        info.valid = tmp.validate();
        info.error = info.valid ? TrackerConfigError::None : TrackerConfigError::CrcOrValidationFailed;
        lastError_ = info.error;
        return true;
    }

    bool load(TrackerConfig& out) {
        Preferences prefs;
        if (!prefs.begin(ns_, true)) {
            lastError_ = TrackerConfigError::NvsBeginFailed;
            return false;
        }

        const size_t storedLen = prefs.getBytesLength(key_);
        if (storedLen == 0) {
            prefs.end();
            lastError_ = TrackerConfigError::NotFound;
            return false;
        }

        if (storedLen != sizeof(TrackerConfigBlob)) {
            prefs.end();
            lastError_ = TrackerConfigError::SizeMismatch;
            return false;
        }

        TrackerConfig tmp;
        const size_t readLen = prefs.getBytes(key_, &tmp.data, sizeof(tmp.data));
        prefs.end();

        if (readLen != sizeof(tmp.data)) {
            lastError_ = TrackerConfigError::ReadFailed;
            return false;
        }

        if (!tmp.validate()) {
            lastError_ = TrackerConfigError::CrcOrValidationFailed;
            return false;
        }

        out = tmp;
        lastError_ = TrackerConfigError::None;
        return true;
    }

    bool loadOrDefaults(TrackerConfig& out, bool* loadedFromNvs = nullptr) {
        if (load(out)) {
            if (loadedFromNvs) *loadedFromNvs = true;
            return true;
        }

        out.resetDefaults();
        if (loadedFromNvs) *loadedFromNvs = false;
        return true;
    }

    bool save(TrackerConfig config) {
        config.sanitize();
        config.updateCrc();

        if (!config.validate()) {
            lastError_ = TrackerConfigError::CrcOrValidationFailed;
            return false;
        }

        Preferences prefs;
        if (!prefs.begin(ns_, false)) {
            lastError_ = TrackerConfigError::NvsBeginFailed;
            return false;
        }

        const size_t written = prefs.putBytes(key_, &config.data, sizeof(config.data));
        prefs.end();

        if (written != sizeof(config.data)) {
            lastError_ = TrackerConfigError::WriteFailed;
            return false;
        }

        lastError_ = TrackerConfigError::None;
        return true;
    }

    bool erase() {
        Preferences prefs;
        if (!prefs.begin(ns_, false)) {
            lastError_ = TrackerConfigError::NvsBeginFailed;
            return false;
        }

        const bool ok = prefs.remove(key_);
        prefs.end();

        if (!ok) {
            lastError_ = TrackerConfigError::RemoveFailed;
            return false;
        }

        lastError_ = TrackerConfigError::None;
        return true;
    }

    bool exists() {
        Preferences prefs;
        if (!prefs.begin(ns_, true)) {
            lastError_ = TrackerConfigError::NvsBeginFailed;
            return false;
        }
        const size_t len = prefs.getBytesLength(key_);
        prefs.end();
        lastError_ = TrackerConfigError::None;
        return len == sizeof(TrackerConfigBlob);
    }

private:
    const char* ns_;
    const char* key_;
    TrackerConfigError lastError_ = TrackerConfigError::None;
};

} // namespace tracker
