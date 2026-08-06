#pragma once

#include <Arduino.h>

#include <cstdint>

// Compile-only Arduino Wi-Fi surface for host verification of the enabled
// remote-console translation unit. It does not model sockets or networking.
class WiFiClient : public Stream {
public:
    explicit operator bool() const { return valid_; }
    bool connected() const { return connected_; }
    void stop() {
        valid_ = false;
        connected_ = false;
    }
    int fd() const { return -1; }
    void setNoDelay(bool) {}

private:
    bool valid_ = false;
    bool connected_ = false;
};

class WiFiServer {
public:
    explicit WiFiServer(uint16_t) {}
    void begin() {}
    void stop() {}
    void end() {}
    void setNoDelay(bool) {}
    WiFiClient available() { return WiFiClient{}; }
};
