#include "network/wifi_remote_console.hpp"

namespace tracker {

void WifiRemoteConsoleRuntime::begin(const TrackerSerialCommandContext& baseContext) {
    configured_ = true;
    enabled_ = TRACKER_ENABLE_WIFI_REMOTE_CONSOLE != 0;
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    baseContext_ = baseContext;
    baseContext_.io = nullptr;
#else
    (void)baseContext;
#endif
}

bool WifiRemoteConsoleRuntime::update(bool wifiConnected, uint32_t nowMs, size_t maxBytesPerUpdate) {
    (void)nowMs;
#if !TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    (void)wifiConnected;
    (void)maxBytesPerUpdate;
    return false;
#else
    if (!configured_) return false;

    bool worked = false;

    if (!enabled_ || !wifiConnected) {
        if (clientConnected_ || client_) {
            stopClient();
            worked = true;
        }
        if (serverStarted_) {
            stopServer();
            worked = true;
        }
        return worked;
    }

    if (!serverStarted_) {
        startServer();
        worked = true;
    }

    WiFiClient candidate = server_.available();
    if (candidate) {
        if (client_ && client_.connected()) {
            candidate.println("# ERR remote console busy");
            candidate.flush();
            candidate.stop();
            ++rejectedClients_;
        } else {
            acceptClient(candidate);
        }
        worked = true;
    }

    if (client_) {
        if (!client_.connected()) {
            stopClient();
            worked = true;
        } else {
            const size_t consumed = cli_.poll(maxBytesPerUpdate);
            if (consumed > 0) {
                bytesIn_ += static_cast<uint32_t>(consumed);
                worked = true;
            }
        }
    }

    return worked;
#endif
}

void WifiRemoteConsoleRuntime::setEnabled(bool enabled) {
    enabled_ = enabled && (TRACKER_ENABLE_WIFI_REMOTE_CONSOLE != 0);
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    if (!enabled_) {
        stopClient();
        stopServer();
    }
#endif
}

WifiRemoteConsoleStatus WifiRemoteConsoleRuntime::status() const {
    WifiRemoteConsoleStatus s;
    s.compiled = TRACKER_ENABLE_WIFI_REMOTE_CONSOLE != 0;
    s.enabled = enabled_;
    s.serverStarted = serverStarted_;
    s.clientConnected = clientConnected_;
    s.port = TRACKER_REMOTE_CONSOLE_PORT;
    s.acceptedClients = acceptedClients_;
    s.rejectedClients = rejectedClients_;
    s.droppedClients = droppedClients_;
    s.bytesIn = bytesIn_;
    s.bytesDropped = bytesDropped_;
    return s;
}

void WifiRemoteConsoleRuntime::printStatus(Stream& out) const {
    const WifiRemoteConsoleStatus s = status();
    out.println("# REMOTE CONSOLE STATUS");
    out.print("remote_console_compiled="); out.println(s.compiled ? "yes" : "no");
    out.print("remote_console_enabled="); out.println(s.enabled ? "yes" : "no");
    out.print("remote_console_port="); out.println(s.port);
    out.print("remote_console_server_started="); out.println(s.serverStarted ? "yes" : "no");
    out.print("remote_console_client_connected="); out.println(s.clientConnected ? "yes" : "no");
    out.print("remote_console_accepted_clients="); out.println(s.acceptedClients);
    out.print("remote_console_rejected_clients="); out.println(s.rejectedClients);
    out.print("remote_console_dropped_clients="); out.println(s.droppedClients);
    out.print("remote_console_bytes_in="); out.println(s.bytesIn);
    out.print("remote_console_bytes_dropped="); out.println(s.bytesDropped);
}

#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
void WifiRemoteConsoleRuntime::startServer() {
    server_.begin();
#if defined(ARDUINO_ARCH_ESP32)
    server_.setNoDelay(true);
#endif
    serverStarted_ = true;
}

void WifiRemoteConsoleRuntime::stopServer() {
    if (!serverStarted_) return;
#if defined(ARDUINO_ARCH_ESP32)
    server_.end();
#else
    server_.stop();
#endif
    serverStarted_ = false;
}

void WifiRemoteConsoleRuntime::stopClient() {
    if (client_) {
        client_.flush();
        client_.stop();
    }
    if (clientConnected_) {
        ++droppedClients_;
    }
    clientConnected_ = false;
    clientContext_ = TrackerSerialCommandContext{};
}

void WifiRemoteConsoleRuntime::acceptClient(WiFiClient& candidate) {
    client_ = candidate;
#if defined(ARDUINO_ARCH_ESP32)
    client_.setNoDelay(true);
#endif
    clientContext_ = baseContext_;
    clientContext_.io = &client_;
    cli_.begin(clientContext_);
    clientConnected_ = true;
    ++acceptedClients_;

    client_.println("# Tracker remote console");
    client_.print("# build_profile=");
    client_.print(trackerBuildProfileName());
    client_.print(" port=");
    client_.println(static_cast<uint16_t>(TRACKER_REMOTE_CONSOLE_PORT));
    client_.println("# Type: help");
}
#endif

} // namespace tracker
