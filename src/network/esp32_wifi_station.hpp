#pragma once

#include "network/wifi_manager.hpp"

namespace tracker {

class Esp32WifiStationAdapter final : public IWifiStationAdapter {
public:
    bool begin(const char* ssid, const char* password, const char* hostname) override;
    void disconnect() override;
    WifiStationInfo info() const override;
    int16_t scanNetworks(WifiScanResult* results, uint8_t maxResults, bool showHidden) override;
    void clearScanResults() override;
};

} // namespace tracker
