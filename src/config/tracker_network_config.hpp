#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <cstdint>
#include <cstring>

#include "config/tracker_config_detail.hpp"
#include "config/tracker_config_store.hpp"

namespace tracker {

// ============================================================
// Future network / SlimeVR persistent storage
// ============================================================
// Kept in a separate NVS namespace so normal config dumps can stay safe and
// calibration resets do not have to imply Wi-Fi credential resets. This is not
// wired to runtime yet; it reserves the schema before the next calibration pass.

namespace tracker_network_detail {
static constexpr uint32_t CONFIG_MAGIC = 0x544E4554UL; // 'TNET'
static constexpr uint16_t CONFIG_VERSION = 1;
static constexpr const char* NVS_NAMESPACE = "tracker_net";
static constexpr const char* NVS_KEY_CONFIG = "netcfg";
static constexpr uint16_t DEFAULT_SLIMEVR_PORT = 6969;
}

struct TrackerNetworkConfigBlob {
    uint32_t magic = tracker_network_detail::CONFIG_MAGIC;
    uint16_t version = tracker_network_detail::CONFIG_VERSION;
    uint16_t size = sizeof(TrackerNetworkConfigBlob);
    uint32_t crc32 = 0;

    bool wifiEnabled = false;
    bool credentialsValid = false;
    bool manualServerEnabled = false;
    bool discoveryEnabled = true;

    char ssid[33] = "";
    char password[65] = "";
    char serverHost[64] = "";
    uint16_t serverPort = tracker_network_detail::DEFAULT_SLIMEVR_PORT;

    uint32_t deviceId = 0;
    uint8_t sensorId = 0;
    uint8_t reserved0 = 0;
    uint16_t reserved1 = 0;
    char deviceName[32] = "c3_6dsv_tracker";
};

class TrackerNetworkConfig {
public:
    TrackerNetworkConfigBlob data;

    void resetDefaults() {
        data = TrackerNetworkConfigBlob{};
        updateCrc();
    }

    uint32_t computeCrc() const {
        TrackerNetworkConfigBlob tmp = data;
        tmp.crc32 = 0;
        return tracker_config_detail::fnv1a32(
            reinterpret_cast<const uint8_t*>(&tmp),
            sizeof(tmp)
        );
    }

    void updateCrc() {
        data.magic = tracker_network_detail::CONFIG_MAGIC;
        data.version = tracker_network_detail::CONFIG_VERSION;
        data.size = sizeof(TrackerNetworkConfigBlob);
        data.crc32 = 0;
        data.crc32 = computeCrc();
    }

    void sanitize() {
        data.ssid[sizeof(data.ssid) - 1] = '\0';
        data.password[sizeof(data.password) - 1] = '\0';
        data.serverHost[sizeof(data.serverHost) - 1] = '\0';
        data.deviceName[sizeof(data.deviceName) - 1] = '\0';
        if (data.deviceName[0] == '\0') {
            std::strncpy(data.deviceName, "c3_6dsv_tracker", sizeof(data.deviceName) - 1);
            data.deviceName[sizeof(data.deviceName) - 1] = '\0';
        }
        if (data.serverPort == 0) data.serverPort = tracker_network_detail::DEFAULT_SLIMEVR_PORT;
        if (!data.credentialsValid) {
            data.wifiEnabled = false;
        }
        updateCrc();
    }

    bool validate() const {
        if (data.magic != tracker_network_detail::CONFIG_MAGIC) return false;
        if (data.version != tracker_network_detail::CONFIG_VERSION) return false;
        if (data.size != sizeof(TrackerNetworkConfigBlob)) return false;
        if (computeCrc() != data.crc32) return false;
        if (data.serverPort == 0) return false;
        if (data.credentialsValid && data.ssid[0] == '\0') return false;
        return true;
    }
};

class TrackerNetworkConfigStore {
public:
    explicit TrackerNetworkConfigStore(const char* nvsNamespace = tracker_network_detail::NVS_NAMESPACE,
                                       const char* key = tracker_network_detail::NVS_KEY_CONFIG)
        : ns_(nvsNamespace), key_(key) {}

    TrackerConfigError lastError() const { return lastError_; }
    const char* lastErrorName() const { return TrackerConfigStore::errorName(lastError_); }

    bool load(TrackerNetworkConfig& out) {
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

    bool save(TrackerNetworkConfig config) {
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

    bool erase() {
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

private:
    const char* ns_;
    const char* key_;
    TrackerConfigError lastError_ = TrackerConfigError::None;
};

// ============================================================
// Serial-friendly summary helpers
// ============================================================

} // namespace tracker
