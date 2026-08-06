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

namespace tracker_command_origin_detail {

inline char asciiLower(char c) {
    return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
}

inline bool is(const char* lhs, const char* rhs) {
    if (!lhs || !rhs) return false;
    while (*lhs && *rhs) {
        if (asciiLower(*lhs++) != asciiLower(*rhs++)) return false;
    }
    return *lhs == '\0' && *rhs == '\0';
}

inline bool oneOf(const char* value, const char* a, const char* b = nullptr,
                  const char* c = nullptr, const char* d = nullptr) {
    return is(value, a) || (b && is(value, b)) || (c && is(value, c)) ||
           (d && is(value, d));
}

} // namespace tracker_command_origin_detail

// The TCP console is a capture/diagnostic transport, not a second privileged
// administration interface. Keep this policy exact and fail closed: persistent
// config, calibration, setup, reboot/reset and network mutations are USB-only.
inline bool trackerRemoteDiagnosticCommandAllowed(int argc, char* const* argv) {
    using namespace tracker_command_origin_detail;
    if (argc <= 0 || !argv || !argv[0]) return false;

    if (argc == 1 && oneOf(argv[0], "help", "?", "status", "health")) return true;
    if (argc == 1 && is(argv[0], "version")) return true;

    if (is(argv[0], "console")) {
        return argc == 2 && oneOf(argv[1], "status", "reset");
    }
    if (is(argv[0], "remote")) {
        return argc == 2 && is(argv[1], "status");
    }

    if (is(argv[0], "perf")) {
        if (argc == 1) return true;
        if (argc == 2 && oneOf(argv[1], "status", "top", "tracking", "brief")) return true;
        return argc == 3 && oneOf(argv[1], "tracking", "brief") && is(argv[2], "reset");
    }
    if (is(argv[0], "motion")) {
        return argc == 1 || (argc == 2 && is(argv[1], "status"));
    }

    if (is(argv[0], "log")) {
        if (argc == 1) return true;
        if (argc == 2 && oneOf(argv[1], "header", "summary", "reset", "off")) return true;
        if (argc == 2 && is(argv[1], "finish")) return true;
        if (argc == 2 && oneOf(argv[1], "stop", "basic", "full", "start")) return true;
        if (argc == 3 && is(argv[1], "rate")) return true;
        return argc == 3 && is(argv[1], "start") && oneOf(argv[2], "basic", "full");
    }

    if (is(argv[0], "test")) {
        if (argc == 2 && oneOf(argv[1], "status", "stop")) return true;
        if (argc == 3 && is(argv[1], "summary") && oneOf(argv[2], "static", "runtime")) return true;
        return argc == 3 && oneOf(argv[1], "static", "runtime");
    }

    if (is(argv[0], "mag")) {
        return argc == 1 || (argc == 2 && oneOf(argv[1], "status", "processed", "trust"));
    }
    if (is(argv[0], "net") || is(argv[0], "slime") ||
        is(argv[0], "battery") || is(argv[0], "bat")) {
        return argc == 2 && is(argv[1], "status");
    }
    if (is(argv[0], "fifo")) {
        return argc == 2 && oneOf(argv[1], "status", "stats");
    }
    if (is(argv[0], "quality")) {
        return argc == 2 && is(argv[1], "stats");
    }
    if (is(argv[0], "imu")) {
        return argc == 2 && is(argv[1], "status");
    }
    if (is(argv[0], "bias")) {
        return argc == 2 && is(argv[1], "status");
    }
    if (is(argv[0], "ahrs")) {
        return argc == 2 && oneOf(argv[1], "status", "config");
    }

    return false;
}

} // namespace tracker
