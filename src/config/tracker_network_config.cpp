#include "config/tracker_network_config.hpp"

#include <Preferences.h>
#include <cstring>


namespace tracker {

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

void TrackerNetworkConfig::sanitize() {
    data.ssid[sizeof(data.ssid) - 1] = '\0';
    data.password[sizeof(data.password) - 1] = '\0';
    data.serverHost[sizeof(data.serverHost) - 1] = '\0';
    data.deviceName[sizeof(data.deviceName) - 1] = '\0';
    if (data.deviceName[0] == '\0' || std::strcmp(data.deviceName, "c3_6dsv_tracker") == 0) {
        std::strncpy(data.deviceName, "c3-6dsv-tracker", sizeof(data.deviceName) - 1);
        data.deviceName[sizeof(data.deviceName) - 1] = '\0';
    }

    // DHCP hostnames should be simple LDH labels. Some routers tolerate
    // underscores/spaces, some do not. Keep the stored name safe before it is
    // passed to WiFi.setHostname().
    for (char* p = data.deviceName; *p; ++p) {
        const bool alpha = (*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z');
        const bool digit = (*p >= '0' && *p <= '9');
        if (!alpha && !digit && *p != '-') *p = '-';
    }
    if (data.deviceName[0] == '-') data.deviceName[0] = 't';
    const size_t nameLen = std::strlen(data.deviceName);
    if (nameLen > 0 && data.deviceName[nameLen - 1] == '-') data.deviceName[nameLen - 1] = 'r';
    if (data.serverPort == 0) data.serverPort = tracker_network_detail::DEFAULT_SLIMEVR_PORT;
    if (!data.credentialsValid) {
        data.wifiEnabled = false;
    }
    updateCrc();
}

bool TrackerNetworkConfig::validate() const {
    if (data.magic != tracker_network_detail::CONFIG_MAGIC) return false;
    if (data.version != tracker_network_detail::CONFIG_VERSION) return false;
    if (data.size != sizeof(TrackerNetworkConfigBlob)) return false;
    if (computeCrc() != data.crc32) return false;
    if (data.serverPort == 0) return false;
    if (data.credentialsValid && data.ssid[0] == '\0') return false;
    return true;
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
    if (!tmp.validate()) {
        lastError_ = TrackerConfigError::CrcOrValidationFailed;
        return false;
    }
    out = tmp;
    lastError_ = TrackerConfigError::None;
    return true;
}

bool TrackerNetworkConfigStore::save(TrackerNetworkConfig config) {
    config.sanitize();
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

bool TrackerNetworkConfigStore::erase() {
    Preferences prefs;
    if (!prefs.begin(ns_, false)) {
        lastError_ = TrackerConfigError::NvsBeginFailed;
        return false;
    }
    const bool ok = prefs.remove(key_);
    prefs.end();
    lastError_ = ok ? TrackerConfigError::None : TrackerConfigError::RemoveFailed;
    return ok;
}

} // namespace tracker
