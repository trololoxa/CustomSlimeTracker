#pragma once

#include <cstdint>

namespace tracker {

enum class TrackerCommandOrigin : uint8_t {
    UsbSerial = 0,
    RemoteTcp = 1,
};

inline const char* trackerCommandOriginName(TrackerCommandOrigin origin) {
    return origin == TrackerCommandOrigin::RemoteTcp ? "remote_tcp" : "usb_serial";
}

} // namespace tracker
