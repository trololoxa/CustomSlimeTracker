#include "network/wifi_remote_console.hpp"

#include "build_config/build_identity.hpp"

#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
#include <cerrno>
#include <cstring>
#include <sys/socket.h>
#endif

namespace tracker {

void WifiRemoteConsoleRuntime::begin(const TrackerSerialCommandContext& baseContext) {
    const bool firstBegin = !configured_;
    configured_ = true;
    if (firstBegin) {
        enabled_ = TRACKER_ENABLE_WIFI_REMOTE_CONSOLE != 0;
    }
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    baseContext_ = baseContext;
    baseContext_.io = nullptr;
#else
    (void)baseContext;
#endif
}

bool WifiRemoteConsoleRuntime::update(bool wifiConnected,
                                      uint32_t nowMs,
                                      size_t maxInputBytesPerUpdate,
                                      size_t maxOutputBytesPerUpdate) {
#if !TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    (void)wifiConnected;
    (void)nowMs;
    (void)maxInputBytesPerUpdate;
    (void)maxOutputBytesPerUpdate;
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

    const bool acceptPollDue = !acceptPollScheduled_ ||
                               static_cast<uint32_t>(nowMs - lastAcceptPollMs_) >=
                                   TRACKER_REMOTE_CONSOLE_ACCEPT_POLL_INTERVAL_MS;
    if (acceptPollDue) {
        lastAcceptPollMs_ = nowMs;
        acceptPollScheduled_ = true;
        ++acceptPolls_;
        WiFiClient candidate = server_.available();
        if (candidate) {
            if (client_ && client_.connected()) {
                static constexpr char kBusy[] = "# ERR remote console busy\r\n";
                const int fd = candidate.fd();
                if (fd >= 0) {
                    (void)::send(fd, kBusy, sizeof(kBusy) - 1u, MSG_DONTWAIT);
                }
                candidate.stop();
                ++rejectedClients_;
            } else {
                acceptClient(candidate);
            }
            worked = true;
        }
    } else {
        ++acceptPollSkips_;
    }

    if (client_) {
        if (!client_.connected()) {
            stopClient();
            worked = true;
        } else {
            const size_t consumed = cli_.poll(maxInputBytesPerUpdate);
            if (consumed > 0u) {
                bytesIn_ += static_cast<uint32_t>(consumed);
                worked = true;
            }

            const bool drainDue = clientStream_.hasPending() &&
                                  maxOutputBytesPerUpdate > 0u &&
                                  (lastOutputDrainMs_ == 0u ||
                                   static_cast<uint32_t>(nowMs - lastOutputDrainMs_) >=
                                       TRACKER_REMOTE_CONSOLE_OUTPUT_DRAIN_INTERVAL_MS);
            if (drainDue) {
                lastOutputDrainMs_ = nowMs;
                worked = drainClientOutput(maxOutputBytesPerUpdate) > 0u || worked;
            }
        }
    }

    return worked;
#endif
}

void WifiRemoteConsoleRuntime::suspend() {
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    stopClient();
    stopServer();
#endif
}

void WifiRemoteConsoleRuntime::flushClientOutput(void* user) {
    auto* self = static_cast<WifiRemoteConsoleRuntime*>(user);
    if (!self) return;
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    (void)self->drainClientOutput(TRACKER_REMOTE_CONSOLE_OUTPUT_BYTES_PER_DRAIN);
#endif
}

size_t WifiRemoteConsoleRuntime::drainClientOutput(size_t byteBudget) {
#if !TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    (void)byteBudget;
    return 0u;
#else
    if (!clientConnected_ || !client_ || byteBudget == 0u) return 0u;

    bool fatalSocketError = false;
    const size_t drained = clientStream_.drainWith(
        byteBudget,
        [this, &fatalSocketError](const uint8_t* data, size_t len) -> size_t {
            const int socketFd = client_.fd();
            if (socketFd < 0) {
                fatalSocketError = true;
                return 0u;
            }
            const int written = ::send(socketFd, data, len, MSG_DONTWAIT);
            if (written > 0) return static_cast<size_t>(written);
            if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0u;
            fatalSocketError = true;
            return 0u;
        });

    if (fatalSocketError) stopClient();
    return drained;
#endif
}

bool WifiRemoteConsoleRuntime::writeDiagnosticLine(const char* line) {
#if !TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    (void)line;
    return false;
#else
    if (!line || !configured_ || !enabled_ || !client_ || !client_.connected()) {
        return false;
    }
    const size_t len = std::strlen(line);
    const size_t recordBytes = len + 2u;
    if (!clientStream_.canAccept(recordBytes)) {
        clientStream_.recordDroppedRecord(recordBytes);
        return false;
    }
    return clientStream_.println(line) == recordBytes;
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
    s.acceptPolls = acceptPolls_;
    s.acceptPollSkips = acceptPollSkips_;
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    s.output = clientStream_.status();
#endif
    return s;
}

void WifiRemoteConsoleRuntime::resetOutputState() {
#if TRACKER_ENABLE_WIFI_REMOTE_CONSOLE
    clientStream_.resetOutputState();
#endif
    bytesDropped_ = 0u;
    lastOutputDrainMs_ = 0u;
    lastAcceptPollMs_ = 0u;
    acceptPollScheduled_ = false;
    acceptPolls_ = 0u;
    acceptPollSkips_ = 0u;
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
    out.print("remote_console_accept_polls="); out.println(s.acceptPolls);
    out.print("remote_console_accept_poll_skips="); out.println(s.acceptPollSkips);
    printBoundedDuplexStreamStatus(out, "remote_console_output", s.output);
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
    lastAcceptPollMs_ = 0u;
    acceptPollScheduled_ = false;
}

void WifiRemoteConsoleRuntime::stopClient() {
    bytesDropped_ += static_cast<uint32_t>(clientStream_.discardPending());
    lastOutputDrainMs_ = 0u;
    clientStream_.detach(false);
    if (client_) {
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
    clientStream_.begin(client_);
    clientStream_.setFlushHandler(flushClientOutput, this);
    lastOutputDrainMs_ = 0u;
    clientContext_ = baseContext_;
    clientContext_.io = &clientStream_;
    clientContext_.commandOutputNeedsExplicitFlush = true;
    clientContext_.lastCommandOutputFlushMs = 0u;
    cli_.begin(clientContext_);
    clientConnected_ = true;
    ++acceptedClients_;

    clientStream_.println("# Tracker remote console");
    clientStream_.print("# build_profile=");
    clientStream_.print(trackerBuildProfileName());
    clientStream_.print(" pio_env=");
    clientStream_.print(trackerBuildPioEnvironment());
    clientStream_.print(" git=");
    clientStream_.print(trackerBuildIdentityString());
    clientStream_.print(" port=");
    clientStream_.println(static_cast<uint16_t>(TRACKER_REMOTE_CONSOLE_PORT));
    clientStream_.println("# Type: help");
}
#endif

} // namespace tracker
