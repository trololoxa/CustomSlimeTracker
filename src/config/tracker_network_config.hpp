#pragma once

#include <cstdint>

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
    char deviceName[32] = "c3-6dsv-tracker";
};

class TrackerNetworkConfig {
public:
    TrackerNetworkConfigBlob data;

    void resetDefaults();
    uint32_t computeCrc() const;
    void updateCrc();
    void sanitize();
    bool validate() const;
};

class TrackerNetworkConfigStore {
public:
    explicit TrackerNetworkConfigStore(const char* nvsNamespace = tracker_network_detail::NVS_NAMESPACE,
                                       const char* key = tracker_network_detail::NVS_KEY_CONFIG);

    TrackerConfigError lastError() const;
    const char* lastErrorName() const;

    bool load(TrackerNetworkConfig& out);
    bool save(TrackerNetworkConfig config);
    bool erase();

private:
    const char* ns_ = nullptr;
    const char* key_ = nullptr;
    TrackerConfigError lastError_ = TrackerConfigError::None;
};

} // namespace tracker
