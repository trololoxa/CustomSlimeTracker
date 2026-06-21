#include "network/wifi_manager.hpp"

#include <cstring>

namespace tracker {

const char* wifiLinkStatusName(WifiLinkStatus status) {
    switch (status) {
        case WifiLinkStatus::Disconnected: return "disconnected";
        case WifiLinkStatus::Connecting: return "connecting";
        case WifiLinkStatus::Connected: return "connected";
        case WifiLinkStatus::NoSsid: return "no_ssid";
        case WifiLinkStatus::ConnectFailed: return "connect_failed";
    }
    return "unknown";
}

const char* wifiAuthTypeName(WifiAuthType auth) {
    switch (auth) {
        case WifiAuthType::Open: return "open";
        case WifiAuthType::Wep: return "WEP";
        case WifiAuthType::WpaPsk: return "WPA-PSK";
        case WifiAuthType::Wpa2Psk: return "WPA2-PSK";
        case WifiAuthType::WpaWpa2Psk: return "WPA/WPA2-PSK";
        case WifiAuthType::Wpa2Enterprise: return "WPA2-Enterprise";
        case WifiAuthType::Wpa3Psk: return "WPA3-PSK";
        case WifiAuthType::Wpa2Wpa3Psk: return "WPA2/WPA3-PSK";
        case WifiAuthType::WapiPsk: return "WAPI-PSK";
        case WifiAuthType::Unknown: return "unknown";
    }
    return "unknown";
}

const char* trackerWifiStateName(TrackerWifiState state) {
    switch (state) {
        case TrackerWifiState::Disabled: return "disabled";
        case TrackerWifiState::Idle: return "idle";
        case TrackerWifiState::Connecting: return "connecting";
        case TrackerWifiState::Connected: return "connected";
        case TrackerWifiState::Backoff: return "backoff";
    }
    return "unknown";
}

const char* wifiPowerSaveModeName(WifiPowerSaveMode mode) {
    switch (mode) {
        case WifiPowerSaveMode::Unknown: return "unknown";
        case WifiPowerSaveMode::None: return "none";
        case WifiPowerSaveMode::MinModem: return "min_modem";
        case WifiPowerSaveMode::MaxModem: return "max_modem";
    }
    return "unknown";
}

void TrackerWifiManager::begin(IWifiStationAdapter& adapter) {
    adapter_ = &adapter;
    refreshInfo();
}

void TrackerWifiManager::configure(const TrackerWifiManagerConfig& config) {
    const bool oldReady = configReady();
    const bool oldEnabled = desiredEnabled_;
    const bool active = state_ == TrackerWifiState::Connecting ||
                        state_ == TrackerWifiState::Connected ||
                        state_ == TrackerWifiState::Backoff;

    char oldSsid[sizeof(ssid_)];
    char oldPassword[sizeof(password_)];
    char oldHostname[sizeof(hostname_)];
    copyCString(oldSsid, sizeof(oldSsid), ssid_);
    copyCString(oldPassword, sizeof(oldPassword), password_);
    copyCString(oldHostname, sizeof(oldHostname), hostname_);

    desiredEnabled_ = config.enabled;
    credentialsValid_ = config.credentialsValid;
    copyCString(ssid_, sizeof(ssid_), config.ssid);
    copyCString(password_, sizeof(password_), config.password);
    copyCString(hostname_, sizeof(hostname_), config.hostname);

    connectTimeoutMs_ = config.connectTimeoutMs == 0 ? 1u : config.connectTimeoutMs;
    reconnectBackoffMs_ = config.reconnectBackoffMs;
    statusPollIntervalMs_ = config.statusPollIntervalMs == 0 ? 1u : config.statusPollIntervalMs;

    const bool newReady = configReady();
    const bool connectionConfigChanged = std::strcmp(oldSsid, ssid_) != 0 ||
                                         std::strcmp(oldPassword, password_) != 0 ||
                                         std::strcmp(oldHostname, hostname_) != 0 ||
                                         oldReady != newReady ||
                                         oldEnabled != desiredEnabled_;
    if (active && connectionConfigChanged && adapter_) {
        adapter_->disconnect();
        ++disconnects_;
        linkStatus_ = WifiLinkStatus::Disconnected;
        connectedSinceMs_ = 0;
        state_ = TrackerWifiState::Disabled;
        nextRetryMs_ = 0;
    }
}

void TrackerWifiManager::resetCounters() {
    attempts_ = 0;
    connectTimeouts_ = 0;
    disconnects_ = 0;
}

void TrackerWifiManager::reset() {
    if (adapter_) adapter_->disconnect();
    state_ = TrackerWifiState::Disabled;
    linkStatus_ = WifiLinkStatus::Disconnected;
    desiredEnabled_ = false;
    credentialsValid_ = false;
    ssid_[0] = '\0';
    password_[0] = '\0';
    hostname_[0] = '\0';
    connectTimeoutMs_ = 15000;
    reconnectBackoffMs_ = 5000;
    statusPollIntervalMs_ = 250;
    attempts_ = 0;
    connectTimeouts_ = 0;
    disconnects_ = 0;
    lastTransitionMs_ = 0;
    connectStartedMs_ = 0;
    nextRetryMs_ = 0;
    lastPollMs_ = 0;
    connectedSinceMs_ = 0;
    lastInfo_ = WifiStationInfo{};
}

void TrackerWifiManager::suspend() {
    if (adapter_ && state_ != TrackerWifiState::Disabled) {
        adapter_->disconnect();
    }
    state_ = TrackerWifiState::Disabled;
    linkStatus_ = WifiLinkStatus::Disconnected;
    connectedSinceMs_ = 0;
    connectStartedMs_ = 0;
    nextRetryMs_ = 0;
    lastPollMs_ = 0;
    lastInfo_ = WifiStationInfo{};
}

void TrackerWifiManager::update(uint32_t nowMs) {
    if (!adapter_) return;

    if (!configReady()) {
        forceDisable(nowMs);
        return;
    }

    if (state_ == TrackerWifiState::Disabled) {
        transitionTo(TrackerWifiState::Idle, nowMs);
    }

    if (static_cast<uint32_t>(nowMs - lastPollMs_) >= statusPollIntervalMs_) {
        lastPollMs_ = nowMs;
        refreshInfo();
    }

    switch (state_) {
        case TrackerWifiState::Disabled:
            break;

        case TrackerWifiState::Idle:
            startConnect(nowMs);
            break;

        case TrackerWifiState::Connecting:
            if (linkStatus_ == WifiLinkStatus::Connected) {
                connectedSinceMs_ = nowMs;
                transitionTo(TrackerWifiState::Connected, nowMs);
            } else if (static_cast<uint32_t>(nowMs - connectStartedMs_) >= connectTimeoutMs_) {
                // ESP32 Arduino may report WL_CONNECT_FAILED / WL_NO_SSID_AVAIL
                // transiently while association is still in progress. The old
                // blocking reference client simply waited up to its full
                // timeout for WL_CONNECTED, so mirror that behavior here: do
                // not abort a connection attempt before connectTimeoutMs.
                ++connectTimeouts_;
                enterBackoff(nowMs);
            }
            break;

        case TrackerWifiState::Connected:
            if (linkStatus_ != WifiLinkStatus::Connected) {
                ++disconnects_;
                enterBackoff(nowMs);
            }
            break;

        case TrackerWifiState::Backoff:
            if (static_cast<int32_t>(nowMs - nextRetryMs_) >= 0) {
                startConnect(nowMs);
            }
            break;
    }
}

bool TrackerWifiManager::connected() const {
    return state_ == TrackerWifiState::Connected && linkStatus_ == WifiLinkStatus::Connected;
}

TrackerWifiState TrackerWifiManager::state() const { return state_; }
WifiLinkStatus TrackerWifiManager::linkStatus() const { return linkStatus_; }

TrackerWifiManagerStatus TrackerWifiManager::status() const {
    TrackerWifiManagerStatus s;
    s.state = state_;
    s.linkStatus = linkStatus_;
    s.desiredEnabled = desiredEnabled_;
    s.credentialsValid = credentialsValid_;
    s.connected = connected();
    s.attempts = attempts_;
    s.connectTimeouts = connectTimeouts_;
    s.disconnects = disconnects_;
    s.lastTransitionMs = lastTransitionMs_;
    s.nextRetryMs = nextRetryMs_;
    s.connectedSinceMs = connectedSinceMs_;
    s.ipv4 = lastInfo_.ipv4;
    s.rssiDbm = lastInfo_.rssiDbm;
    s.powerSaveMode = lastInfo_.powerSaveMode;
    s.txPowerValid = lastInfo_.txPowerValid;
    s.txPowerQuarterDbm = lastInfo_.txPowerQuarterDbm;
    std::memcpy(s.mac, lastInfo_.mac, sizeof(s.mac));
    copyCString(s.ssid, sizeof(s.ssid), ssid_);
    copyCString(s.hostname, sizeof(s.hostname), hostname_);
    return s;
}


int16_t TrackerWifiManager::scanNetworks(WifiScanResult* results, uint8_t maxResults, bool showHidden) {
    if (!adapter_ || !results || maxResults == 0) return -1;
    return adapter_->scanNetworks(results, maxResults, showHidden);
}

void TrackerWifiManager::clearScanResults() {
    if (adapter_) adapter_->clearScanResults();
}

void TrackerWifiManager::forceDisable(uint32_t nowMs) {
    if (state_ != TrackerWifiState::Disabled && adapter_) {
        adapter_->disconnect();
        ++disconnects_;
    }
    linkStatus_ = WifiLinkStatus::Disconnected;
    connectedSinceMs_ = 0;
    transitionTo(TrackerWifiState::Disabled, nowMs);
}

void TrackerWifiManager::startConnect(uint32_t nowMs) {
    if (!adapter_) return;
    ++attempts_;
    connectStartedMs_ = nowMs;
    lastPollMs_ = nowMs;
    linkStatus_ = WifiLinkStatus::Connecting;

    const bool beginOk = adapter_->begin(ssid_, password_, hostname_);
    if (!beginOk) {
        enterBackoff(nowMs);
        return;
    }

    refreshInfo();
    transitionTo(TrackerWifiState::Connecting, nowMs);
}

void TrackerWifiManager::enterBackoff(uint32_t nowMs) {
    if (adapter_) adapter_->disconnect();
    connectedSinceMs_ = 0;
    nextRetryMs_ = nowMs + reconnectBackoffMs_;
    refreshInfo();
    transitionTo(TrackerWifiState::Backoff, nowMs);
}

void TrackerWifiManager::transitionTo(TrackerWifiState state, uint32_t nowMs) {
    if (state_ == state) return;
    state_ = state;
    lastTransitionMs_ = nowMs;
}

void TrackerWifiManager::refreshInfo() {
    if (!adapter_) {
        lastInfo_ = WifiStationInfo{};
        linkStatus_ = WifiLinkStatus::Disconnected;
        return;
    }
    lastInfo_ = adapter_->info();
    linkStatus_ = lastInfo_.linkStatus;
}

bool TrackerWifiManager::configReady() const {
    return desiredEnabled_ && credentialsValid_ && ssid_[0] != '\0';
}

void TrackerWifiManager::copyCString(char* dst, uint32_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    uint32_t i = 0;
    for (; i + 1u < dstSize && src[i] != '\0'; ++i) {
        dst[i] = src[i];
    }
    dst[i] = '\0';
}

} // namespace tracker
