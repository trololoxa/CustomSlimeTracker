#pragma once

#include <cstddef>
#include <cstdint>

namespace tracker {

// Host-safe UDP transport interface used by SlimeVR runtime. ESP32-specific
// WiFiUDP code lives in esp32_udp_transport.*; packet/runtime code should only
// depend on this small abstraction.
struct UdpEndpoint {
    uint32_t ipv4 = 0; // a.b.c.d encoded as 0xAABBCCDD
    uint16_t port = 0;

    bool valid() const { return ipv4 != 0 && port != 0; }
    bool operator==(const UdpEndpoint& other) const {
        return ipv4 == other.ipv4 && port == other.port;
    }
    bool operator!=(const UdpEndpoint& other) const { return !(*this == other); }
};

bool udpParseIpv4(const char* text, uint32_t& outIpv4);

class IUdpTransport {
public:
    virtual ~IUdpTransport() = default;

    virtual bool begin(uint16_t localPort) = 0;
    virtual void stop() = 0;
    virtual bool active() const = 0;
    virtual uint16_t localPort() const = 0;

    virtual bool send(const UdpEndpoint& endpoint, const uint8_t* data, size_t len) = 0;

    // Resolve a manual server hostname to IPv4. Host-safe transports inherit
    // the dotted-decimal parser; ESP32 overrides this with DNS support.
    virtual bool resolveHost(const char* host, uint32_t& outIpv4) {
        return udpParseIpv4(host, outIpv4);
    }

    // Returns next packet size, or <= 0 when no packet is available.
    virtual int parsePacket() = 0;
    virtual int read(uint8_t* data, size_t maxLen) = 0;
    virtual UdpEndpoint remoteEndpoint() const = 0;
};

const char* udpIpv4ToCString(uint32_t ipv4, char* out, size_t outSize);

} // namespace tracker
