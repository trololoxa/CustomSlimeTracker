#include "network/esp32_udp_transport.hpp"

#include <Arduino.h>
#include <WiFi.h>

namespace tracker {

namespace {

IPAddress ipFromU32(uint32_t ipv4) {
    return IPAddress(static_cast<uint8_t>((ipv4 >> 24) & 0xffu),
                     static_cast<uint8_t>((ipv4 >> 16) & 0xffu),
                     static_cast<uint8_t>((ipv4 >> 8) & 0xffu),
                     static_cast<uint8_t>(ipv4 & 0xffu));
}

uint32_t ipToU32(const IPAddress& ip) {
    return (static_cast<uint32_t>(ip[0]) << 24) |
           (static_cast<uint32_t>(ip[1]) << 16) |
           (static_cast<uint32_t>(ip[2]) << 8) |
           static_cast<uint32_t>(ip[3]);
}

} // namespace

bool Esp32UdpTransport::begin(uint16_t localPort) {
    if (active_ && localPort_ == localPort) return true;
    stop();
    if (WiFi.status() != WL_CONNECTED) return false;
    const uint16_t requestedPort = localPort;
    if (!udp_.begin(requestedPort)) {
        active_ = false;
        localPort_ = 0;
        return false;
    }
    active_ = true;
    localPort_ = requestedPort;
    return true;
}

void Esp32UdpTransport::stop() {
    if (active_) udp_.stop();
    active_ = false;
    localPort_ = 0;
}

bool Esp32UdpTransport::active() const {
    return active_;
}

uint16_t Esp32UdpTransport::localPort() const {
    return localPort_;
}

bool Esp32UdpTransport::resolveHost(const char* host, uint32_t& outIpv4) {
    if (udpParseIpv4(host, outIpv4)) return true;
    outIpv4 = 0;
    if (!host || host[0] == '\0' || WiFi.status() != WL_CONNECTED) return false;
    IPAddress resolved;
    if (WiFi.hostByName(host, resolved) != 1) return false;
    const uint32_t value = ipToU32(resolved);
    if (value == 0u || value == 0xffffffffu) return false;
    outIpv4 = value;
    return true;
}

bool Esp32UdpTransport::send(const UdpEndpoint& endpoint, const uint8_t* data, size_t len) {
    if (!active_ || !endpoint.valid() || !data || len == 0) return false;
    if (!udp_.beginPacket(ipFromU32(endpoint.ipv4), endpoint.port)) return false;
    const size_t written = udp_.write(data, len);
    const int result = udp_.endPacket();
    return result > 0 && written == len;
}

int Esp32UdpTransport::parsePacket() {
    if (!active_) return 0;
    return udp_.parsePacket();
}

int Esp32UdpTransport::read(uint8_t* data, size_t maxLen) {
    if (!active_ || !data || maxLen == 0) return 0;
    return udp_.read(data, maxLen);
}

UdpEndpoint Esp32UdpTransport::remoteEndpoint() const {
    UdpEndpoint out;
    if (!active_) return out;
    out.ipv4 = ipToU32(udp_.remoteIP());
    out.port = udp_.remotePort();
    return out;
}

} // namespace tracker
