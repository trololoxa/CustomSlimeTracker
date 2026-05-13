#pragma once

#include <cstdint>

namespace tracker {

// Host-safe Wi-Fi state machine primitives.
//
// This file intentionally does not include Arduino/WiFi headers. The state
// machine is tested natively through IWifiStationAdapter, while ESP32-specific
// calls live in esp32_wifi_station.*.

enum class WifiLinkStatus : uint8_t {
    Disconnected,
    Connecting,
    Connected,
    NoSsid,
    ConnectFailed,
};

enum class TrackerWifiState : uint8_t {
    Disabled,
    Idle,
    Connecting,
    Connected,
    Backoff,
};

const char* wifiLinkStatusName(WifiLinkStatus status);
const char* trackerWifiStateName(TrackerWifiState state);

struct WifiStationInfo {
    WifiLinkStatus linkStatus = WifiLinkStatus::Disconnected;
    uint32_t ipv4 = 0;          // a.b.c.d encoded as 0xAABBCCDD
    int32_t rssiDbm = 0;
    uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
};

class IWifiStationAdapter {
public:
    virtual ~IWifiStationAdapter() = default;

    virtual bool begin(const char* ssid, const char* password, const char* hostname) = 0;
    virtual void disconnect() = 0;
    virtual WifiStationInfo info() const = 0;
};

struct TrackerWifiManagerConfig {
    bool enabled = false;
    bool credentialsValid = false;
    const char* ssid = nullptr;
    const char* password = nullptr;
    const char* hostname = nullptr;

    uint32_t connectTimeoutMs = 15000;
    uint32_t reconnectBackoffMs = 5000;
    uint32_t statusPollIntervalMs = 250;
};

struct TrackerWifiManagerStatus {
    TrackerWifiState state = TrackerWifiState::Disabled;
    WifiLinkStatus linkStatus = WifiLinkStatus::Disconnected;

    bool desiredEnabled = false;
    bool credentialsValid = false;
    bool connected = false;

    uint32_t attempts = 0;
    uint32_t connectTimeouts = 0;
    uint32_t disconnects = 0;
    uint32_t lastTransitionMs = 0;
    uint32_t nextRetryMs = 0;
    uint32_t connectedSinceMs = 0;

    uint32_t ipv4 = 0;
    int32_t rssiDbm = 0;
    uint8_t mac[6] = {0, 0, 0, 0, 0, 0};

    char ssid[33] = "";
    char hostname[32] = "";
};

class TrackerWifiManager {
public:
    void begin(IWifiStationAdapter& adapter);
    void configure(const TrackerWifiManagerConfig& config);
    void resetCounters();
    void reset();

    void update(uint32_t nowMs);

    bool connected() const;
    TrackerWifiState state() const;
    WifiLinkStatus linkStatus() const;
    TrackerWifiManagerStatus status() const;

private:
    void forceDisable(uint32_t nowMs);
    void startConnect(uint32_t nowMs);
    void enterBackoff(uint32_t nowMs);
    void transitionTo(TrackerWifiState state, uint32_t nowMs);
    void refreshInfo();
    bool configReady() const;

    static void copyCString(char* dst, uint32_t dstSize, const char* src);

    IWifiStationAdapter* adapter_ = nullptr;

    TrackerWifiState state_ = TrackerWifiState::Disabled;
    WifiLinkStatus linkStatus_ = WifiLinkStatus::Disconnected;

    bool desiredEnabled_ = false;
    bool credentialsValid_ = false;

    char ssid_[33] = "";
    char password_[65] = "";
    char hostname_[32] = "";

    uint32_t connectTimeoutMs_ = 15000;
    uint32_t reconnectBackoffMs_ = 5000;
    uint32_t statusPollIntervalMs_ = 250;

    uint32_t attempts_ = 0;
    uint32_t connectTimeouts_ = 0;
    uint32_t disconnects_ = 0;
    uint32_t lastTransitionMs_ = 0;
    uint32_t connectStartedMs_ = 0;
    uint32_t nextRetryMs_ = 0;
    uint32_t lastPollMs_ = 0;
    uint32_t connectedSinceMs_ = 0;

    WifiStationInfo lastInfo_;
};

} // namespace tracker
