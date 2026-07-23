#include "config/tracker_config_store.hpp"

#include <Preferences.h>

namespace tracker {

TrackerConfigStore::TrackerConfigStore(const char* nvsNamespace,
                                         const char* key)
    : ns_(nvsNamespace), key_(key) {}

TrackerConfigError TrackerConfigStore::lastError() const {
    return lastError_;
}

const char* TrackerConfigStore::lastErrorName() const {
    return errorName(lastError_);
}

const char* TrackerConfigStore::errorName(TrackerConfigError e) {
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

bool TrackerConfigStore::inspect(TrackerConfigNvsInfo& info) {
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

bool TrackerConfigStore::load(TrackerConfig& out) {
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

bool TrackerConfigStore::loadOrDefaults(TrackerConfig& out, bool* loadedFromNvs) {
    if (load(out)) {
        if (loadedFromNvs) *loadedFromNvs = true;
        return true;
    }

    out.resetDefaults();
    if (loadedFromNvs) *loadedFromNvs = false;
    return true;
}

bool TrackerConfigStore::save(TrackerConfig& config) {
    TrackerConfig candidate = config;
    candidate.sanitize();
    candidate.updateCrc();

    if (!candidate.validate()) {
        lastError_ = TrackerConfigError::CrcOrValidationFailed;
        return false;
    }

    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }

    const size_t written = prefs.putBytes(key_, &candidate.data, sizeof(candidate.data));
    prefs.end();

    if (written != sizeof(candidate.data)) {
        lastError_ = TrackerConfigError::WriteFailed;
        return false;
    }

    // Keep the live/config mirror byte-for-byte consistent with what was
    // persisted. Failed writes never alter the caller's state.
    config = candidate;
    lastError_ = TrackerConfigError::None;
    return true;
}

bool TrackerConfigStore::erase() {
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

bool TrackerConfigStore::exists() {
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

} // namespace tracker
