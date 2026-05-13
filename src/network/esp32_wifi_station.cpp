#include "network/esp32_wifi_station.hpp"

#include <Arduino.h>
#include <WiFi.h>

namespace tracker {

bool Esp32WifiStationAdapter::begin(const char* ssid, const char* password, const char* hostname) {
    if (!ssid || ssid[0] == '\0') return false;

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    if (hostname && hostname[0] != '\0') {
        WiFi.setHostname(hostname);
    }

    if (password && password[0] != '\0') {
        WiFi.begin(ssid, password);
    } else {
        WiFi.begin(ssid);
    }
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

} // namespace tracker
