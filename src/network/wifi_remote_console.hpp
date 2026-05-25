#pragma once

#include <Arduino.h>

#include "defines.h"
#include "serial/tracker_serial_commands.hpp"

#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
#include <WiFi.h>
#endif

namespace tracker {

struct WifiRemoteConsoleStatus {
    bool compiled = TRACKER_ENABLE_WIFI_REMOTE_CONSOLE != 0;
    bool enabled = false;
    bool serverStarted = false;
    bool clientConnected = false;
    uint16_t port = TRACKER_REMOTE_CONSOLE_PORT;
    uint32_t acceptedClients = 0;
    uint32_t rejectedClients = 0;
    uint32_t droppedClients = 0;
    uint32_t bytesIn = 0;
    uint32_t bytesDropped = 0;
};

class WifiRemoteConsoleRuntime {
public:
    void begin(const TrackerSerialCommandContext& baseContext);
    bool update(bool wifiConnected, uint32_t nowMs, size_t maxBytesPerUpdate);
    void setEnabled(bool enabled);
    bool enabled() const { return enabled_; }
    WifiRemoteConsoleStatus status() const;
    void printStatus(Stream& out) const;

private:
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    void startServer();
    void stopServer();
    void stopClient();
    void acceptClient(WiFiClient& candidate);
#endif

    bool configured_ = false;
    bool enabled_ = false;
    bool serverStarted_ = false;
    bool clientConnected_ = false;
    uint32_t acceptedClients_ = 0;
    uint32_t rejectedClients_ = 0;
    uint32_t droppedClients_ = 0;
    uint32_t bytesIn_ = 0;
    uint32_t bytesDropped_ = 0;

#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    WiFiServer server_{TRACKER_REMOTE_CONSOLE_PORT};
    WiFiClient client_;
    TrackerSerialCommandContext baseContext_;
    TrackerSerialCommandContext clientContext_;
    TrackerSerialCommandInterface<> cli_;
#endif
};

} // namespace tracker
