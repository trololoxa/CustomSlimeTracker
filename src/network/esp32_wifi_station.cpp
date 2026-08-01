#include "network/esp32_wifi_station.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <cstring>

#include "defines.h"

namespace tracker {

namespace {

void applyTrackerWifiTxPower() {
#if TRACKER_WIFI_APPLY_TX_POWER_AFTER_BEGIN
    // Some ESP32-C3 board revisions have RF problems at maximum TX power
    // because the antenna is too close to the crystal. Keep this board policy
    // in build_config/network_tuning.hpp so other revisions can override or
    // disable it cleanly.
#ifdef TRACKER_WIFI_TX_POWER_QUARTER_DBM
    // Numeric quarter-dBm override is used by wave-5 A/B environments. It
    // avoids depending on Arduino enum token availability across ESP32 cores.
    esp_wifi_set_max_tx_power(static_cast<int8_t>(TRACKER_WIFI_TX_POWER_QUARTER_DBM));
#else
    WiFi.setTxPower(static_cast<wifi_power_t>(TRACKER_WIFI_TX_POWER));
#endif
#endif
}

constexpr bool trackerWifiPowerSaveEnabled() {
    return TRACKER_WIFI_POWER_SAVE_MODE != TRACKER_WIFI_POWER_SAVE_NONE;
}

wifi_ps_type_t trackerWifiPowerSaveType() {
#if TRACKER_WIFI_POWER_SAVE_MODE == TRACKER_WIFI_POWER_SAVE_MIN_MODEM
    return WIFI_PS_MIN_MODEM;
#elif TRACKER_WIFI_POWER_SAVE_MODE == TRACKER_WIFI_POWER_SAVE_MAX_MODEM
    return WIFI_PS_MAX_MODEM;
#else
    return WIFI_PS_NONE;
#endif
}

WifiPowerSaveMode mapPowerSaveMode(wifi_ps_type_t ps) {
    switch (ps) {
        case WIFI_PS_NONE: return WifiPowerSaveMode::None;
        case WIFI_PS_MIN_MODEM: return WifiPowerSaveMode::MinModem;
        case WIFI_PS_MAX_MODEM: return WifiPowerSaveMode::MaxModem;
        default: return WifiPowerSaveMode::Unknown;
    }
}

void applyTrackerWifiPowerSave() {
    WiFi.setSleep(trackerWifiPowerSaveEnabled());
    esp_wifi_set_ps(trackerWifiPowerSaveType());
}

void copyBounded(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    if (!src) src = "";
    std::strncpy(dst, src, dstSize - 1);
    dst[dstSize - 1] = '\0';
}

WifiAuthType mapAuthType(int auth) {
    switch (auth) {
        case WIFI_AUTH_OPEN: return WifiAuthType::Open;
        case WIFI_AUTH_WEP: return WifiAuthType::Wep;
        case WIFI_AUTH_WPA_PSK: return WifiAuthType::WpaPsk;
        case WIFI_AUTH_WPA2_PSK: return WifiAuthType::Wpa2Psk;
        case WIFI_AUTH_WPA_WPA2_PSK: return WifiAuthType::WpaWpa2Psk;
        case WIFI_AUTH_WPA2_ENTERPRISE: return WifiAuthType::Wpa2Enterprise;
#if defined(WIFI_AUTH_WPA3_PSK)
        case WIFI_AUTH_WPA3_PSK: return WifiAuthType::Wpa3Psk;
#endif
#if defined(WIFI_AUTH_WPA2_WPA3_PSK)
        case WIFI_AUTH_WPA2_WPA3_PSK: return WifiAuthType::Wpa2Wpa3Psk;
#endif
#if defined(WIFI_AUTH_WAPI_PSK)
        case WIFI_AUTH_WAPI_PSK: return WifiAuthType::WapiPsk;
#endif
        default: return WifiAuthType::Unknown;
    }
}

} // namespace

bool Esp32WifiStationAdapter::begin(const char* ssid, const char* password, const char* hostname) {
    if (!ssid || ssid[0] == '\0') return false;

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(trackerWifiPowerSaveEnabled());

    // Match the reference client's behavior of resetting the station before a
    // new connection attempt. Avoid delay() here; TrackerWifiManager provides
    // the non-blocking timeout/backoff window around this call.
    WiFi.disconnect(false, false);

    if (hostname && hostname[0] != '\0') {
        WiFi.setHostname(hostname);
    }

    if (password && password[0] != '\0') {
        WiFi.begin(ssid, password);
    } else {
        WiFi.begin(ssid);
    }
    applyTrackerWifiTxPower();
    applyTrackerWifiPowerSave();
    invalidateInfoCache();
    return true;
}

void Esp32WifiStationAdapter::disconnect() {
    WiFi.disconnect(false, false);
    invalidateInfoCache();
}

void Esp32WifiStationAdapter::radioOff() {
    WiFi.disconnect(false, false);
    WiFi.mode(WIFI_OFF);
    invalidateInfoCache();
}

void Esp32WifiStationAdapter::invalidateInfoCache() {
    cachedSlowInfo_ = WifiStationInfo{};
    slowInfoValid_ = false;
    lastSlowInfoMs_ = 0u;
    lastSlowLinkStatus_ = WifiLinkStatus::Disconnected;
}

WifiStationInfo Esp32WifiStationAdapter::info() const {
    WifiStationInfo out;

    const wl_status_t s = WiFi.status();
    switch (s) {
        case WL_CONNECTED:
            out.linkStatus = WifiLinkStatus::Connected;
            break;
        case WL_IDLE_STATUS:
        case WL_SCAN_COMPLETED:
            out.linkStatus = WifiLinkStatus::Connecting;
            break;
        case WL_NO_SSID_AVAIL:
            out.linkStatus = WifiLinkStatus::NoSsid;
            break;
        case WL_CONNECT_FAILED:
            out.linkStatus = WifiLinkStatus::ConnectFailed;
            break;
        case WL_CONNECTION_LOST:
        case WL_DISCONNECTED:
        default:
            out.linkStatus = WifiLinkStatus::Disconnected;
            break;
    }

    if (out.linkStatus == WifiLinkStatus::Connected) {
        const IPAddress ip = WiFi.localIP();
        out.ipv4 = (static_cast<uint32_t>(ip[0]) << 24) |
                   (static_cast<uint32_t>(ip[1]) << 16) |
                   (static_cast<uint32_t>(ip[2]) << 8) |
                   static_cast<uint32_t>(ip[3]);
    }

    const uint32_t nowMs = millis();
    const bool refreshSlow = !slowInfoValid_ ||
        out.linkStatus != lastSlowLinkStatus_ ||
        static_cast<uint32_t>(nowMs - lastSlowInfoMs_) >= TRACKER_WIFI_DIAGNOSTIC_INFO_REFRESH_MS;
    if (refreshSlow) {
        cachedSlowInfo_ = WifiStationInfo{};
        cachedSlowInfo_.linkStatus = out.linkStatus;
        cachedSlowInfo_.ipv4 = out.ipv4;
        if (out.linkStatus == WifiLinkStatus::Connected) {
            cachedSlowInfo_.rssiDbm = WiFi.RSSI();
        }
        wifi_ps_type_t ps = WIFI_PS_NONE;
        if (esp_wifi_get_ps(&ps) == ESP_OK) {
            cachedSlowInfo_.powerSaveMode = mapPowerSaveMode(ps);
        }
        int8_t txPower = 0;
        if (esp_wifi_get_max_tx_power(&txPower) == ESP_OK) {
            cachedSlowInfo_.txPowerValid = true;
            cachedSlowInfo_.txPowerQuarterDbm = txPower;
        }
        WiFi.macAddress(cachedSlowInfo_.mac);
        slowInfoValid_ = true;
        lastSlowInfoMs_ = nowMs;
        lastSlowLinkStatus_ = out.linkStatus;
    }

    out.rssiDbm = cachedSlowInfo_.rssiDbm;
    out.powerSaveMode = cachedSlowInfo_.powerSaveMode;
    out.txPowerValid = cachedSlowInfo_.txPowerValid;
    out.txPowerQuarterDbm = cachedSlowInfo_.txPowerQuarterDbm;
    std::memcpy(out.mac, cachedSlowInfo_.mac, sizeof(out.mac));
    return out;
}

int16_t Esp32WifiStationAdapter::scanNetworks(WifiScanResult* results, uint8_t maxResults, bool showHidden) {
    if (!results || maxResults == 0) return -1;

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);

    WiFi.scanDelete();

    int16_t count = WiFi.scanNetworks(false, showHidden);
    if (count == -2) {
        // WIFI_SCAN_FAILED can happen when the radio is still settling after a
        // previous connection/scan operation. Retry once for CLI diagnostics.
        delay(250);
        WiFi.scanDelete();
        count = WiFi.scanNetworks(false, showHidden);
    }
    if (count <= 0) {
#if TRACKER_WIFI_RESTORE_POWER_SAVE_AFTER_SCAN
        applyTrackerWifiPowerSave();
#endif
        return count;
    }

    const uint8_t copied = static_cast<uint8_t>(count < maxResults ? count : maxResults);
    for (uint8_t i = 0; i < copied; ++i) {
        WifiScanResult& r = results[i];
        r = WifiScanResult{};
        const String ssid = WiFi.SSID(i);
        copyBounded(r.ssid, sizeof(r.ssid), ssid.c_str());
        const String bssid = WiFi.BSSIDstr(i);
        copyBounded(r.bssid, sizeof(r.bssid), bssid.c_str());
        r.rssiDbm = WiFi.RSSI(i);
        r.channel = static_cast<uint8_t>(WiFi.channel(i));
        r.authType = mapAuthType(WiFi.encryptionType(i));
        r.hidden = r.ssid[0] == '\0';
    }

#if TRACKER_WIFI_RESTORE_POWER_SAVE_AFTER_SCAN
    applyTrackerWifiPowerSave();
#endif
    return count;
}

void Esp32WifiStationAdapter::clearScanResults() {
    WiFi.scanDelete();
}

} // namespace tracker
