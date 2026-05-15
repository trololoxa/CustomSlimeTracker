#include "network/esp32_wifi_station.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <cstring>

#include "defines.h"

namespace tracker {

namespace {

void applyTrackerWifiTxPower() {
#if TRACKER_WIFI_APPLY_TX_POWER_AFTER_BEGIN
    // Some ESP32-C3 board revisions have RF problems at maximum TX power
    // because the antenna is too close to the crystal. Keep this board policy
    // in defines.h so other revisions can override or disable it cleanly.
    WiFi.setTxPower(static_cast<wifi_power_t>(TRACKER_WIFI_TX_POWER));
#endif
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
    WiFi.setSleep(false);

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
    return true;
}

void Esp32WifiStationAdapter::disconnect() {
    WiFi.disconnect(false, false);
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
        out.rssiDbm = WiFi.RSSI();
    }

    WiFi.macAddress(out.mac);
    return out;
}

int16_t Esp32WifiStationAdapter::scanNetworks(WifiScanResult* results, uint8_t maxResults, bool showHidden) {
    if (!results || maxResults == 0) return -1;

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    WiFi.scanDelete();

    int16_t count = WiFi.scanNetworks(false, showHidden);
    if (count == -2) {
        // WIFI_SCAN_FAILED can happen when the radio is still settling after a
        // previous connection/scan operation. Retry once for CLI diagnostics.
        delay(250);
        WiFi.scanDelete();
        count = WiFi.scanNetworks(false, showHidden);
    }
    if (count <= 0) return count;

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

    return count;
}

void Esp32WifiStationAdapter::clearScanResults() {
    WiFi.scanDelete();
}

} // namespace tracker
