#pragma once

#include <WiFiUdp.h>

#include "network/udp_transport.hpp"

namespace tracker {

class Esp32UdpTransport final : public IUdpTransport {
public:
    bool begin(uint16_t localPort) override;
    void stop() override;
    bool active() const override;
    uint16_t localPort() const override;

    bool send(const UdpEndpoint& endpoint, const uint8_t* data, size_t len) override;
    int lastSendError() const override;
    bool resolveHost(const char* host, uint32_t& outIpv4) override;
    int parsePacket() override;
    int read(uint8_t* data, size_t maxLen) override;
    UdpEndpoint remoteEndpoint() const override;

private:
    mutable WiFiUDP udp_;
    bool active_ = false;
    uint16_t localPort_ = 0;
    int lastSendError_ = 0;
};

} // namespace tracker
