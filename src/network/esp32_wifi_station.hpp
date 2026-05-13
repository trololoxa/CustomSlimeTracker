#pragma once

#include "network/wifi_manager.hpp"

namespace tracker {

class Esp32WifiStationAdapter final : public IWifiStationAdapter {
public:
    bool begin(const char* ssid, const char* password, const char* hostname) override;
    void disconnect() override;
    WifiStationInfo info() const override;
};

} // namespace tracker
