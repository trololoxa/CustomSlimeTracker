#include "config/tracker_network_config.hpp"

#include <Preferences.h>
#include <cstring>


namespace tracker {

namespace {

bool validUserActionValue(uint8_t value) {
    return value == static_cast<uint8_t>(SlimeVRUserAction::None) ||
           value == static_cast<uint8_t>(SlimeVRUserAction::FullReset) ||
           value == static_cast<uint8_t>(SlimeVRUserAction::YawReset) ||
           value == static_cast<uint8_t>(SlimeVRUserAction::MountingReset) ||
           value == static_cast<uint8_t>(SlimeVRUserAction::PauseTracking);
}

bool terminatedWithin(const char* value, size_t capacity) {
    return value && std::memchr(value, '\0', capacity) != nullptr;
}

bool validHostname(const char* value) {
    if (!value || value[0] == '\0' || value[0] == '-') return false;
    size_t length = 0;
    for (const char* p = value; *p; ++p, ++length) {
        const bool alpha = (*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z');
        const bool digit = (*p >= '0' && *p <= '9');
        if (!alpha && !digit && *p != '-') return false;
    }
    return length != 0u && value[length - 1u] != '-';
}

} // namespace


void TrackerNetworkConfig::resetDefaults() {
    data = TrackerNetworkConfigBlob{};
    updateCrc();
}

uint32_t TrackerNetworkConfig::computeCrc() const {
    TrackerNetworkConfigBlob tmp = data;
    tmp.crc32 = 0;
    return tracker_config_detail::fnv1a32(
        reinterpret_cast<const uint8_t*>(&tmp),
        sizeof(tmp)
    );
}

void TrackerNetworkConfig::updateCrc() {
    data.magic = tracker_network_detail::CONFIG_MAGIC;
    data.version = tracker_network_detail::CONFIG_VERSION;
    data.size = sizeof(TrackerNetworkConfigBlob);
    data.crc32 = 0;
    data.crc32 = computeCrc();
}

bool TrackerNetworkConfig::validate() const {
    if (data.magic != tracker_network_detail::CONFIG_MAGIC) return false;
    if (data.version != tracker_network_detail::CONFIG_VERSION) return false;
    if (data.size != sizeof(TrackerNetworkConfigBlob)) return false;
    if (computeCrc() != data.crc32) return false;
    return validateSemanticConfig();
}

bool TrackerNetworkConfig::validateSemanticConfig() const {
    if (!terminatedWithin(data.ssid, sizeof(data.ssid)) ||
        !terminatedWithin(data.password, sizeof(data.password)) ||
        !terminatedWithin(data.serverHost, sizeof(data.serverHost)) ||
        !terminatedWithin(data.deviceName, sizeof(data.deviceName))) return false;
    if (data.serverPort == 0u || !validHostname(data.deviceName)) return false;
    if (data.reserved1 != 0u) return false;
    if (data.credentialsValid && data.ssid[0] == '\0') return false;
    if (data.wifiEnabled && !data.credentialsValid) return false;
    if (data.manualServerEnabled && data.serverHost[0] == '\0') return false;
    return validUserActionValue(data.tapUserAction);
}

SlimeVRUserAction TrackerNetworkConfig::tapUserAction() const {
    return validUserActionValue(data.tapUserAction)
        ? static_cast<SlimeVRUserAction>(data.tapUserAction)
        : SlimeVRUserAction::None;
}

void TrackerNetworkConfig::setTapUserAction(SlimeVRUserAction action) {
    data.tapUserAction = static_cast<uint8_t>(action);
    updateCrc();
}

TrackerNetworkConfigStore::TrackerNetworkConfigStore(const char* nvsNamespace,
                                                       const char* key)
    : ns_(nvsNamespace), key_(key) {}

TrackerConfigError TrackerNetworkConfigStore::lastError() const { return lastError_; }

const char* TrackerNetworkConfigStore::lastErrorName() const { return TrackerConfigStore::errorName(lastError_); }

bool TrackerNetworkConfigStore::load(TrackerNetworkConfig& out) {
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
    if (storedLen != sizeof(TrackerNetworkConfigBlob)) {
        prefs.end();
        lastError_ = TrackerConfigError::SizeMismatch;
        return false;
    }
    TrackerNetworkConfig tmp;
    const size_t readLen = prefs.getBytes(key_, &tmp.data, sizeof(tmp.data));
    prefs.end();
    if (readLen != sizeof(tmp.data)) {
        lastError_ = TrackerConfigError::ReadFailed;
        return false;
    }
    // First validate the stored envelope exactly as written. Sanitizing before
    // this check would turn corrupted bytes into apparently valid input.
    if (tmp.data.magic != tracker_network_detail::CONFIG_MAGIC ||
        tmp.data.version != tracker_network_detail::CONFIG_VERSION ||
        tmp.data.size != sizeof(TrackerNetworkConfigBlob) ||
        tmp.computeCrc() != tmp.data.crc32) {
        lastError_ = TrackerConfigError::CrcOrValidationFailed;
        return false;
    }
    // The only same-schema legacy exception is the exact old default DHCP
    // hostname. Do not run the general sanitizer here: a CRC-valid but
    // semantically impossible current record must still fail closed.
    if (terminatedWithin(tmp.data.deviceName, sizeof(tmp.data.deviceName)) &&
        std::strcmp(tmp.data.deviceName, "c3_6dsv_tracker") == 0) {
        std::strncpy(
            tmp.data.deviceName, "c3-6dsv-tracker", sizeof(tmp.data.deviceName) - 1u);
        tmp.data.deviceName[sizeof(tmp.data.deviceName) - 1u] = '\0';
        tmp.updateCrc();
    }
    if (!tmp.validate()) {
        lastError_ = TrackerConfigError::CrcOrValidationFailed;
        return false;
    }
    out = tmp;
    lastError_ = TrackerConfigError::None;
    return true;
}

bool TrackerNetworkConfigStore::save(TrackerNetworkConfig& config) {
    if (writeInhibited_) {
        lastError_ = TrackerConfigError::WriteInhibited;
        return false;
    }
    TrackerNetworkConfig candidate = config;
    if (!candidate.validateSemanticConfig()) {
        lastError_ = TrackerConfigError::CrcOrValidationFailed;
        return false;
    }
    candidate.updateCrc();
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
    TrackerNetworkConfig verify;
    if (!load(verify) || std::memcmp(&verify.data, &candidate.data, sizeof(candidate.data)) != 0) {
        lastError_ = TrackerConfigError::CommitUncertain;
        return false;
    }
    config = candidate;
    lastError_ = TrackerConfigError::None;
    return true;
}

bool TrackerNetworkConfigStore::erase() {
    if (writeInhibited_) {
        lastError_ = TrackerConfigError::WriteInhibited;
        return false;
    }
    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    const bool ok = !prefs.isKey(key_) || prefs.remove(key_);
    prefs.end();
    if (!ok) {
        lastError_ = TrackerConfigError::RemoveFailed;
        return false;
    }
    Preferences verify;
    if (!verify.begin(ns_, true)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    const bool gone = !verify.isKey(key_);
    verify.end();
    lastError_ = gone ? TrackerConfigError::None : TrackerConfigError::RemoveFailed;
    return gone;
}

} // namespace tracker
