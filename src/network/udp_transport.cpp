#include "network/udp_transport.hpp"

#include <cstdio>

namespace tracker {

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
