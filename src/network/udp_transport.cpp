#include "network/udp_transport.hpp"

#include <cstdio>

namespace tracker {

bool udpParseIpv4(const char* text, uint32_t& outIpv4) {
    outIpv4 = 0;
    if (!text || text[0] == '\0') return false;

    uint32_t value = 0;
    const char* p = text;
    for (unsigned part = 0; part < 4; ++part) {
        if (*p < '0' || *p > '9') return false;
        unsigned octet = 0;
        unsigned digits = 0;
        while (*p >= '0' && *p <= '9') {
            octet = octet * 10u + static_cast<unsigned>(*p - '0');
            if (octet > 255u || ++digits > 3u) return false;
            ++p;
        }
        value = (value << 8u) | octet;
        if (part < 3u) {
            if (*p != '.') return false;
            ++p;
        } else if (*p != '\0') {
            return false;
        }
    }
    if (value == 0u || value == 0xffffffffu) return false;
    outIpv4 = value;
    return true;
}

const char* udpIpv4ToCString(uint32_t ipv4, char* out, size_t outSize) {
    if (!out || outSize == 0) return "";
    std::snprintf(out,
                  outSize,
                  "%u.%u.%u.%u",
                  static_cast<unsigned>((ipv4 >> 24) & 0xffu),
                  static_cast<unsigned>((ipv4 >> 16) & 0xffu),
                  static_cast<unsigned>((ipv4 >> 8) & 0xffu),
                  static_cast<unsigned>(ipv4 & 0xffu));
    out[outSize - 1] = '\0';
    return out;
}

} // namespace tracker
