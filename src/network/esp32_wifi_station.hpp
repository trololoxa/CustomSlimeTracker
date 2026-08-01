#pragma once

#include "network/wifi_manager.hpp"

namespace tracker {

class Esp32WifiStationAdapter final : public IWifiStationAdapter {
public:
    bool begin(const char* ssid, const char* password, const char* hostname) override;
    void disconnect() override;

    // Stronger than disconnect(): shuts down the ESP Wi-Fi radio before light
    // sleep so it cannot keep receiving/beaconing in the background.
    void radioOff();

    WifiStationInfo info() const override;
    int16_t scanNetworks(WifiScanResult* results, uint8_t maxResults, bool showHidden) override;
    void clearScanResults() override;

private:
    void invalidateInfoCache();
    mutable WifiStationInfo cachedSlowInfo_;
    mutable bool slowInfoValid_ = false;
    mutable uint32_t lastSlowInfoMs_ = 0u;
    mutable WifiLinkStatus lastSlowLinkStatus_ = WifiLinkStatus::Disconnected;
};

} // namespace tracker
